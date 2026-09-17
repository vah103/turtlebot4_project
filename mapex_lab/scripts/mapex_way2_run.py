#!/usr/bin/env python3
"""Run MapEx Way2 with the existing MapEx recorder/evaluator stack.

The canonical MapEx baseline remains in ``mapex.py`` and the frozen Way2 policy
remains in ``mapex_way2.py``. This wrapper reuses ``mapex_run.py`` only for
measurement, persistence, provenance, and offline evaluation, while dispatching
policy decisions through ``MapExWay2Explorer.exploration_step``.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException

import mapex_way2
from evaluate_mapex_profiled import evaluate_run
from mapex_run import ENVIRONMENT_PROFILES, MapExRun
from nf_run import RUNTIME_PROFILES, _sha256_file


WAY2_RULE_METADATA = {
    "r_threshold": mapex_way2.WAY2_R_THRESHOLD,
    "u_threshold_cells_per_m": mapex_way2.WAY2_U_THRESHOLD_CELLS_PER_M,
    "single_candidate_cutoff": mapex_way2.WAY2_SINGLE_CANDIDATE_CUTOFF,
    "second_confirmation": mapex_way2.WAY2_SECOND_CONFIRMATION,
    "third_confirmation": mapex_way2.WAY2_THIRD_CONFIRMATION,
    "rule_frozen": True,
}


class MapExWay2Run(MapExRun):
    """MapEx recorder with the frozen Way2 policy inserted before next-goal issue."""

    _way2_u_value = staticmethod(mapex_way2.MapExWay2Explorer._way2_u_value)
    publish_way2_check = mapex_way2.MapExWay2Explorer.publish_way2_check
    mark_way2_early_stop = mapex_way2.MapExWay2Explorer.mark_way2_early_stop

    def __init__(
        self,
        run_id: str,
        odom_topic: str,
        save_predictions: bool,
        environment: str,
        runtime_profile: str | None = None,
    ) -> None:
        self.way2_valid_count = 0
        self._way2_last_selectable_execution_candidates = []
        self._way2_last_metrics = None
        super().__init__(
            run_id,
            odom_topic,
            save_predictions,
            environment,
            runtime_profile,
        )
        self.get_logger().warn(
            "MAPEX WAY2 RECORDING: frozen early-stopping rule active "
            f"(R_t<={mapex_way2.WAY2_R_THRESHOLD:.2f}, "
            f"U_t<={mapex_way2.WAY2_U_THRESHOLD_CELLS_PER_M:.1f} cells/m, "
            f"candidate cutoff<={mapex_way2.WAY2_SINGLE_CANDIDATE_CUTOFF})"
        )

    def _write_initial_provenance(self, run_id: str) -> dict:
        metadata = super()._write_initial_provenance(run_id)
        metadata.update(
            {
                "method": "mapex_way2",
                "recorder": "mapex_way2_run.py",
                "canonical_policy": "mapex_way2.py",
                "baseline_policy": "mapex.py",
                "way2_rule": dict(WAY2_RULE_METADATA),
            }
        )
        scripts_dir = Path(__file__).resolve().parent
        hashes = metadata.setdefault("config_sha256", {})
        for name, path in {
            "mapex_way2_recorder": scripts_dir / "mapex_way2_run.py",
            "mapex_way2_policy": scripts_dir / "mapex_way2.py",
        }.items():
            if path.is_file():
                hashes[name] = _sha256_file(path)
        self._write_metadata(metadata)
        return metadata

    def evaluate_way2_stop(
        self,
        selectable_evaluations: list[dict],
    ) -> tuple[bool, dict]:
        # Capture the exact selectable set so an early-stop decision is recorded
        # with the same F_t that Way2 actually evaluated.
        self._way2_last_selectable_execution_candidates = [
            self.to_execution_candidate(evaluation)
            for evaluation in selectable_evaluations
        ]
        should_stop, metrics = mapex_way2.MapExWay2Explorer.evaluate_way2_stop(
            self,
            selectable_evaluations,
        )
        self._way2_last_metrics = dict(metrics)
        return should_stop, metrics

    def exploration_step(self):
        """Keep MapExRun instrumentation but execute the Way2 policy decision."""
        if not self.startup_ready():
            return

        ready = (
            not self.completed
            and self.map_msg is not None
            and not self.goal_active
            and self.main_goal is None
            and not self.revalidation_active
            and self.nav_client.server_is_ready()
        )
        robot_pose = self.robot_position() if ready else None
        if ready and robot_pose is not None:
            self.decision_id += 1
            self.active_decision = self.decision_id
            self.compute_sim_t0 = self.now_s()
            self.compute_t0 = time.perf_counter()
            self.decision_compute_ms = None
            self.pending_decision_selection = None
            self.decision_map_msg = self.map_msg
            self.decision_robot_xy = (float(robot_pose[0]), float(robot_pose[1]))
            self.decision_robot_yaw = self.robot_map_pose()[2]

            self.last_candidate_metrics = []
            self.last_ensemble_predictions = None
            self.last_mean_map = None
            self.last_variance_map = None
            self._way2_last_selectable_execution_candidates = []
            self._way2_last_metrics = None

            if self.t0 is None:
                self.t0 = self.compute_sim_t0
                self._update_metadata(
                    exploration_start_sim_s=self.t0,
                    evaluation_start_x=float(robot_pose[0]),
                    evaluation_start_y=float(robot_pose[1]),
                    runtime_map_resolution_m=float(self.map_msg.info.resolution),
                )

        # Same instrumentation boundary as MapExRun, but the policy path is Way2.
        mapex_way2.MapExWay2Explorer.exploration_step(self)

        if self.pending_decision_selection is not None:
            candidates, selected = self.pending_decision_selection
            self.pending_decision_selection = None
            self.record_decision(candidates, selected, "SELECTED")
        elif self.compute_t0 is not None:
            if (
                self.completed
                and self.completion_reason == "way2_early_stop"
                and self._way2_last_selectable_execution_candidates
            ):
                self.record_decision(
                    self._way2_last_selectable_execution_candidates,
                    None,
                    "WAY2_EARLY_STOP",
                )
            else:
                self.record_decision([], None, "NO_SELECTION")

    def finalize(self, reason: str):
        result = super().finalize(reason)

        summary_path = self.run / "summary.json"
        if summary_path.is_file():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            except Exception:
                summary = {}
            summary["method"] = "mapex_way2"
            summary["way2_rule"] = dict(WAY2_RULE_METADATA)
            summary["way2_last_metrics"] = self._way2_last_metrics
            summary_path.write_text(
                json.dumps(summary, indent=2, allow_nan=True),
                encoding="utf-8",
            )

        self.metadata["method"] = "mapex_way2"
        self.metadata["way2_rule"] = dict(WAY2_RULE_METADATA)
        self.metadata["way2_last_metrics"] = self._way2_last_metrics
        self._write_metadata(self.metadata)
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--environment",
        choices=sorted(ENVIRONMENT_PROFILES),
        default="new_room",
    )
    parser.add_argument(
        "--runtime-profile",
        choices=sorted(RUNTIME_PROFILES),
        default=None,
    )
    parser.add_argument(
        "--save-predictions",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=mapex_way2._mapex_way2_init_args(ros_args))
    node = None
    run_dir = None
    ground_truth_path = None
    roi_path = None
    try:
        node = MapExWay2Run(
            args.run_id,
            args.odom_topic,
            args.save_predictions,
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
            node.close_recorder_files()
            ensemble = getattr(node, "ensemble", None)
            if ensemble is not None and hasattr(ensemble, "close"):
                ensemble.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if run_dir is not None and ground_truth_path is not None and roi_path is not None:
        result = evaluate_run(run_dir, ground_truth_path, roi_path)
        status = result.get("status", "unknown")
        if status == "ok":
            print(
                "MAPEX WAY2 OFFLINE EVAL: "
                f"env={args.environment}, IoU={result['final_occupied_iou']:.6f}, "
                f"TU={result['final_tu']:.6f}, decisions={result['evaluated_decisions']}"
            )
        else:
            print(
                "MAPEX WAY2 OFFLINE EVAL: "
                f"env={args.environment}, {status}: {result.get('reason', 'no reason')}"
            )


if __name__ == "__main__":
    main()
