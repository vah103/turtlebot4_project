#!/usr/bin/env python3
"""Nearest-Frontier benchmark entrypoint with post-run offline diagnostics.

The online recorder/execution implementation is kept in ``_nf_run_core.py``.
This entrypoint re-exports that implementation for MapEx compatibility. The
paper-style IoU/TU pass is performed later from the saved raw maps using the
shared all-training LaMa predictor. Immediately after a run, this wrapper may
still write explicitly-labelled observed-map diagnostics so existing run
workflows remain usable before that post-processing step.
"""
from __future__ import annotations

import argparse
from pathlib import Path as FilePath

import rclpy
from rclpy.executors import ExternalShutdownException

# Re-export the existing recorder API because mapex_run.py imports the shared
# constants, Stage2Run, and helper functions from nf_run.py.
from _nf_run_core import *  # noqa: F401,F403
from _nf_run_core import _deep_merge, _git_value, _sha256_file, _stamp_s  # noqa: F401
from evaluate_nf_profiled import evaluate_run


# ``slam.launch.py`` is now the normal launch path for the current experiments.
# RUNTIME_PROFILES is the same mutable dict owned by ``_nf_run_core.py``;
# registering these profiles here therefore makes them available to both
# Stage2Run and mapex_run.py without changing either exploration policy.
RUNTIME_PROFILES["slam"] = {
    "environment": "new_room",
    "id": "new_room_slam_adaptive_anchor",
    "note": (
        "New Room benchmark launched with launch/slam.launch.py world:=new_room. "
        "Uses stock Karto sequential scan matching with the tested Adaptive "
        "Temporal Anchor pose-graph weighting enabled; the custom OldMap "
        "scan-level matcher is disabled."
    ),
    "launch_relative": "launch/slam.launch.py",
    "slam_relative": None,
    "extra_hashes": {
        "adaptive_ceres_solver": "../slam_toolbox/solvers/ceres_solver.cpp",
    },
}

RUNTIME_PROFILES["hospital_slam"] = {
    "environment": "hospital",
    "id": "hospital_slam_adaptive_anchor",
    "note": (
        "Hospital benchmark launched with launch/slam.launch.py world:=hospital. "
        "The simulation resolves Hospital geometry/spawn through "
        "hospital_flat_simulation.launch.py and hospital_scale.py; the active "
        "Hospital scale is therefore part of the recorded source/config provenance."
    ),
    "launch_relative": "launch/slam.launch.py",
    "slam_relative": None,
    "extra_hashes": {
        "adaptive_ceres_solver": "../slam_toolbox/solvers/ceres_solver.cpp",
        "hospital_scale": "scripts/hospital_scale.py",
        "hospital_world": "map/hospital_aws_flat.sdf",
    },
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--environment",
        choices=sorted(ENVIRONMENT_PROFILES),
        default="new_room",
        help=(
            "Evaluation environment/profile. Default is new_room; "
            "use --environment hospital for Hospital."
        ),
    )
    parser.add_argument(
        "--runtime-profile",
        choices=sorted(RUNTIME_PROFILES),
        default=None,
        help="Runtime provenance profile.",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = None
    run_dir = None
    ground_truth_path = None
    roi_path = None
    try:
        node = Stage2Run(
            args.run_id,
            args.odom_topic,
            args.environment,
            args.runtime_profile,
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if not node.finalized:
                node.finalize("keyboard_interrupt")

            run_dir = node.run
            ground_truth_path = node.ground_truth_path
            roi_path = node.roi_path

            # Preserve provenance for the implementation split introduced by this
            # post-run wrapper. The git commit still pins the complete source tree.
            core_path = FilePath(__file__).resolve().with_name("_nf_run_core.py")
            node.metadata.setdefault("config_sha256", {})["nf_run_core"] = (
                _sha256_file(core_path)
            )
            node.metadata["nf_run_core_file"] = str(core_path)
            node.metadata["offline_evaluation_semantics"] = (
                "paper_style_alltrain_required_for_primary_iou_tu; "
                "immediate_observed_map_metrics_are_legacy_diagnostics"
            )
            node._write_metadata(node.metadata)

            node.close_recorder_files()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # Offline only: ROS recording/navigation has already stopped above. At this
    # point alltrain predictions normally do not exist yet, so preserve the old
    # observed-map evaluation as an explicitly-labelled diagnostic. Running
    # predict_alltrain_offline.py and then evaluate_nf_profiled.py replaces it
    # with the paper-style primary metrics.
    if run_dir is not None and ground_truth_path is not None and roi_path is not None:
        result = evaluate_run(
            run_dir,
            ground_truth_path,
            roi_path,
            allow_observed_fallback=True,
        )
        status = result.get("status", "unknown")
        if status == "ok":
            print(
                "NEAREST OFFLINE EVAL: "
                f"env={args.environment}, source={result.get('prediction_source')}, "
                f"IoU={result['final_occupied_iou']:.6f}, "
                f"TU={result['final_tu']:.6f}, "
                f"decisions={result['evaluated_decisions']}"
            )
            if result.get("prediction_source") != "alltrain":
                print(
                    "NEAREST OFFLINE EVAL NOTE: observed-map values are legacy "
                    "diagnostics; run alltrain post-processing before paper-style comparison."
                )
        else:
            print(
                "NEAREST OFFLINE EVAL: "
                f"env={args.environment}, {status}: "
                f"{result.get('reason', 'no reason')}"
            )


if __name__ == "__main__":
    main()
