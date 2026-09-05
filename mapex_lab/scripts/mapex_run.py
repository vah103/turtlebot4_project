#!/usr/bin/env python3
"""Run canonical MapEx policy with integrated measurement and offline evaluation.

The exploration policy remains in mapex.py. This wrapper reuses Stage2Run's
measurement hooks and nf_basic.py's shared execution layer, while adding
MapEx-specific candidate metrics, G1/G2/G3 + mean/variance prediction storage,
provenance, environment-specific coverage ROI, and post-run IoU/TU evaluation.

New Room defaults to the same ``submap`` runtime provenance profile as nf_run.py.
Use --runtime-profile stock2 when the surrounding simulation/SLAM launch is
launch/stock2.launch.py.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path as FilePath
import statistics
import subprocess
import sys
import time
from collections import Counter

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

import mapex
from evaluate_mapex_profiled import evaluate_run
from generate_new_room_ground_truth import generate as generate_new_room_ground_truth
from mapex_lama_bridge import LamaEnsembleBridge
from nf_run import (
    CANVAS_RES,
    EVALUATION_ROI_ID,
    FIXED_CANVAS_ID,
    NAV2_READY_STABLE_S,
    PROTOCOL_VERSION,
    ROI_N,
    RUNTIME_PROFILES,
    SIM_SEED_POLICY,
    Stage2Run,
    _deep_merge,
    _git_value,
    _sha256_file,
)


# Keep the canonical MapEx policy untouched; isolate only legacy LaMa inference.
mapex.LamaEnsemble = LamaEnsembleBridge


ENVIRONMENT_PROFILES = {
    "new_room": {
        "protocol_version": "new_room_v1",
        "roi_id": "new_room_connected_free_v1",
        "roi_relative": "ground_truth/new_room/generated/new_room_connected_free_v1.npy",
        "ground_truth_id": "new_room_structural_gt_v1",
        "ground_truth_relative": (
            "ground_truth/new_room/generated/new_room_structural_gt_v1.npz"
        ),
        "auto_generate_ground_truth": True,
    },
    "hospital": {
        "protocol_version": PROTOCOL_VERSION,
        "roi_id": EVALUATION_ROI_ID,
        "roi_relative": (
            "ground_truth/hospital/generated/hospital_connected_free_v1.npy"
        ),
        "ground_truth_id": "hospital_structural_gt_v1",
        "ground_truth_relative": (
            "ground_truth/hospital/generated/hospital_structural_gt_v1.npz"
        ),
        "auto_generate_ground_truth": False,
    },
}


def _ensure_new_room_ground_truth(root: FilePath) -> None:
    """Regenerate New Room GT only when missing or stale relative to the SDF."""
    sdf = root / "map" / "new_room.sdf"
    output_dir = root / "ground_truth" / "new_room" / "generated"
    gt = output_dir / "new_room_structural_gt_v1.npz"
    roi = output_dir / "new_room_connected_free_v1.npy"
    summary = output_dir / "new_room_structural_gt_v1_summary.json"

    current_sha = _sha256_file(sdf)
    generated_sha = None
    if summary.is_file():
        try:
            generated_sha = json.loads(summary.read_text(encoding="utf-8")).get(
                "source_sdf_sha256"
            )
        except Exception:
            generated_sha = None

    if gt.is_file() and roi.is_file() and generated_sha == current_sha:
        return

    generate_new_room_ground_truth(
        sdf_path=sdf.resolve(),
        output_dir=output_dir.resolve(),
        z_slice_m=0.20,
        spawn_x=0.0,
        spawn_y=0.0,
    )


class MapExRun(Stage2Run, mapex.MapExExplorer):
    """MapExExplorer plus the shared recorder and environment-aware evaluation."""

    def __init__(
        self,
        run_id: str,
        odom_topic: str,
        save_predictions: bool,
        environment: str,
        runtime_profile: str | None = None,
    ):
        root = FilePath(__file__).resolve().parents[1]
        if environment not in ENVIRONMENT_PROFILES:
            raise RuntimeError(f"Unknown environment: {environment}")

        if runtime_profile is None:
            runtime_profile = "submap" if environment == "new_room" else "hospital"
        if runtime_profile not in RUNTIME_PROFILES:
            raise RuntimeError(f"Unknown runtime profile: {runtime_profile}")
        profile = dict(RUNTIME_PROFILES[runtime_profile])
        if profile["environment"] != environment:
            raise RuntimeError(
                f"Runtime profile '{runtime_profile}' belongs to "
                f"{profile['environment']}, not {environment}"
            )

        self.root = root
        self.repo_root = root.parent
        self.environment = environment
        self.environment_profile = dict(ENVIRONMENT_PROFILES[environment])

        if self.environment_profile["auto_generate_ground_truth"]:
            _ensure_new_room_ground_truth(root)

        self.roi_path = root / self.environment_profile["roi_relative"]
        self.ground_truth_path = root / self.environment_profile[
            "ground_truth_relative"
        ]
        self.roi = (
            np.load(self.roi_path).astype(bool)
            if self.roi_path.is_file()
            else None
        )
        self.roi_n = (
            int(np.count_nonzero(self.roi)) if self.roi is not None else 0
        )

        run = root / "experiments" / "mapex" / run_id
        if run.exists():
            raise RuntimeError(f"Run exists: {run}")

        # Deliberately skip Stage2Run.__init__: it hard-codes experiments/nearest.
        # MapExExplorer still initializes through the exact canonical policy class.
        mapex.MapExExplorer.__init__(self)

        self.runtime_profile_name = runtime_profile
        self.runtime_profile = profile
        self.run = run
        self.save_predictions = bool(save_predictions)
        (self.run / "maps").mkdir(parents=True)
        (self.run / "decision_maps").mkdir()
        if self.save_predictions:
            (self.run / "predictions").mkdir()

        self.metadata = self._write_initial_provenance(run_id)

        if self.roi is None or self.roi_n <= 0:
            self.get_logger().warn(
                f"{environment} ROI missing/empty: coverage will be NaN: "
                f"{self.roi_path}"
            )

        # Common Stage2 recorder state.
        self.t0 = None
        self.last_xy = None
        self.distance = 0.0
        self.last_traj = -1e9
        self.last_snap = -1e9
        self.decision_id = 0
        self.active_decision = None
        self.compute_t0 = None
        self.compute_sim_t0 = None
        self.goal_id = 0
        self.active_goal = None
        self.count = Counter()
        self.errors = Counter()
        self.comp = []
        self.known = math.nan
        self.coverage = math.nan
        self.finalized = False
        self.nav_ready_since = None
        self.startup_gate_open = False
        self.startup_wait_logged = False

        # MapEx-specific statistics/state.
        self.prediction_ms = []
        self.frontier_scoring_ms = []
        self.selected_ig = []
        self.selected_score = []
        self.selected_distance = []
        self.selection_verification_failures = 0
        self.last_ensemble_predictions: np.ndarray | None = None

        q = QoSProfile(depth=100)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, odom_topic, self.odom_cb, q)
        self._open_files()
        self.create_timer(1.0, self.metric_tick)

        self.get_logger().warn(
            f"MAPEX RECORDING: env={self.environment}, "
            f"profile={self.runtime_profile_name}, output={self.run}"
        )
        self.get_logger().info(
            "MapEx provenance saved: metadata.json + runtime_nav2_merged.yaml "
            "+ runtime_mapex.yaml"
        )

    # ----- provenance -----
    def _write_initial_provenance(self, run_id: str) -> dict:
        tb4_nav_pkg = FilePath(get_package_share_directory("turtlebot4_navigation"))
        nav2_base = tb4_nav_pkg / "config" / "nav2.yaml"
        nav2_override = self.root / "config" / "nav2.yaml"
        mapex_config = self.root / "config" / "mapex.yaml"

        with nav2_base.open("r", encoding="utf-8") as stream:
            base_cfg = yaml.safe_load(stream) or {}
        with nav2_override.open("r", encoding="utf-8") as stream:
            override_cfg = yaml.safe_load(stream) or {}

        merged_cfg = _deep_merge(base_cfg, override_cfg)
        merged_path = self.run / "runtime_nav2_merged.yaml"
        with merged_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(merged_cfg, stream, sort_keys=False)

        runtime_mapex = self.run / "runtime_mapex.yaml"
        with runtime_mapex.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(self.config, stream, sort_keys=False)

        git_commit = _git_value(self.repo_root, "rev-parse", "HEAD")
        git_status = _git_value(self.repo_root, "status", "--porcelain")
        mapex_reference_checkout = _git_value(
            self.mapex_root.expanduser().resolve(), "rev-parse", "HEAD"
        )
        mapex_reference_status = _git_value(
            self.mapex_root.expanduser().resolve(), "status", "--porcelain"
        )

        worker_python = FilePath(
            os.environ.get(
                "MAPEX_LAMA_PYTHON",
                str(
                    FilePath.home()
                    / "miniforge3"
                    / "envs"
                    / "lama"
                    / "bin"
                    / "python"
                ),
            )
        ).expanduser()
        worker_python_version = None
        if worker_python.is_file():
            try:
                proc = subprocess.run(
                    [str(worker_python), "--version"],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                worker_python_version = (proc.stdout or proc.stderr).strip()
            except (OSError, subprocess.CalledProcessError):
                pass

        slam_relative = self.runtime_profile["slam_relative"]
        launch_relative = self.runtime_profile["launch_relative"]
        active_slam = self.root / slam_relative if slam_relative else None
        active_launch = self.root / launch_relative if launch_relative else None

        hash_paths = {
            "mapex_run": self.root / "scripts" / "mapex_run.py",
            "mapex_evaluator": self.root / "scripts" / "evaluate_mapex_run.py",
            "mapex_profiled_evaluator": (
                self.root / "scripts" / "evaluate_mapex_profiled.py"
            ),
            "mapex_policy": self.root / "scripts" / "mapex.py",
            "mapex_ros_launcher": self.root / "scripts" / "mapex_ros.py",
            "mapex_lama_bridge": self.root / "scripts" / "mapex_lama_bridge.py",
            "mapex_lama_worker": self.root / "scripts" / "mapex_lama_worker.py",
            "nf_run_recorder_base": self.root / "scripts" / "nf_run.py",
            "nf_basic_shared_execution": self.root / "scripts" / "nf_basic.py",
            "mapex_config": mapex_config,
            "runtime_mapex": runtime_mapex,
            "nav2_override": nav2_override,
            "installed_nav2_base_params": nav2_base,
            "runtime_nav2_merged": merged_path,
        }
        if active_slam is not None:
            hash_paths["runtime_slam"] = active_slam
        if active_launch is not None:
            hash_paths["runtime_launch"] = active_launch
        for name, relative in self.runtime_profile["extra_hashes"].items():
            hash_paths[name] = self.root / relative

        if self.environment == "new_room":
            hash_paths.update(
                {
                    "environment_world": self.root / "map" / "new_room.sdf",
                    "ground_truth_generator": (
                        self.root / "scripts" / "generate_new_room_ground_truth.py"
                    ),
                    "ground_truth_contract": (
                        self.root
                        / "ground_truth"
                        / "new_room"
                        / "structural_gt_v1.yaml"
                    ),
                }
            )

        config_sha256 = {
            name: _sha256_file(path)
            for name, path in hash_paths.items()
            if path.is_file()
        }

        checkpoints = []
        for index, source in enumerate(self.ensemble.source_names, start=1):
            path = FilePath(source).expanduser()
            checkpoints.append(
                {
                    "member": f"G{index}",
                    "path": str(path),
                    "size_bytes": path.stat().st_size if path.is_file() else None,
                    "sha256": _sha256_file(path) if path.is_file() else None,
                }
            )

        metadata = {
            "run_id": run_id,
            "method": "mapex",
            "environment": self.environment,
            "recorder": "mapex_run.py",
            "canonical_policy": "mapex.py",
            "runtime_launcher_equivalent": "mapex_ros.py bridge backend",
            "shared_execution": "nf_basic.py",
            "git_commit": git_commit,
            "git_dirty_at_recorder_start": (
                None if git_status is None else bool(git_status)
            ),
            "protocol_version": self.environment_profile["protocol_version"],
            "runtime_profile": self.runtime_profile["id"],
            "runtime_profile_name": self.runtime_profile_name,
            "runtime_profile_note": self.runtime_profile["note"],
            "runtime_launch_file": (
                None if active_launch is None else str(active_launch)
            ),
            "runtime_slam_file": None if active_slam is None else str(active_slam),
            "runtime_map_resolution_m": None,
            "fixed_canvas_id": FIXED_CANVAS_ID,
            "fixed_canvas_resolution_m": CANVAS_RES,
            "evaluation_roi_id": self.environment_profile["roi_id"],
            "evaluation_roi_denominator": (
                self.roi_n if self.roi_n > 0 else None
            ),
            "evaluation_roi_file": str(self.roi_path),
            "structural_ground_truth_id": self.environment_profile[
                "ground_truth_id"
            ],
            "structural_ground_truth_file": str(self.ground_truth_path),
            "structural_ground_truth_exists_at_start": self.ground_truth_path.is_file(),
            "evaluation_start_x": None,
            "evaluation_start_y": None,
            "exploration_start_sim_s": None,
            "exploration_start_source": "first_policy_decision_before_compute",
            "execution_goal_semantics": "exact_frontier_center_xy",
            "goal_yaw_semantics": "ignored_by_hospital_goal_checker",
            "planner_path_semantics": "reachability_evidence_only",
            "selected_path_length_semantics": (
                "planner_validation_path_length_not_executed_trajectory"
            ),
            "runtime_nav2_merged_file": merged_path.name,
            "runtime_mapex_file": runtime_mapex.name,
            "nav2_base_params_file": str(nav2_base),
            "nav2_override_file": str(nav2_override),
            "mapex_config_file": str(mapex_config),
            "mapex_reference_commit_expected": mapex.MAPEX_REFERENCE_COMMIT,
            "mapex_reference_checkout_commit": mapex_reference_checkout,
            "mapex_reference_checkout_dirty": (
                None if mapex_reference_status is None else bool(mapex_reference_status)
            ),
            "mapex_root": str(self.mapex_root.expanduser().resolve()),
            "mapex_device": self.mapex_device,
            "ros_python_executable": sys.executable,
            "ros_python_version": sys.version.split()[0],
            "lama_worker_python": str(worker_python),
            "lama_worker_python_version": worker_python_version,
            "ensemble_checkpoints": checkpoints,
            "save_prediction_maps": self.save_predictions,
            "saved_prediction_members": 3 if self.save_predictions else 0,
            "sim_seed": None,
            "sim_seed_policy": SIM_SEED_POLICY,
            "config_sha256": config_sha256,
            "termination_reason": None,
        }
        self._write_metadata(metadata)
        return metadata

    # ----- CSV setup -----
    def _open_files(self):
        self.fm, self.wm = self._open(
            "metrics.csv",
            [
                "time_s",
                "distance_m",
                "known_fraction",
                "coverage",
                "occupied_iou",
                "tu",
                "frontiers_selected",
                "main_attempts",
                "main_succeeded",
                "main_failed",
                "main_interrupted",
                "abandoned_206",
                "abandoned_208",
                "planner_blocked_abandoned_total",
                "main_success_rate",
                "subgoal_attempts",
                "subgoal_succeeded",
                "subgoal_failed",
                "subgoal_interrupted",
            ],
        )
        self.ft, self.wt = self._open(
            "trajectory.csv",
            ["time_s", "x", "y", "yaw", "cumulative_distance_m"],
        )
        self.fd, self.wd = self._open(
            "decisions.csv",
            [
                "decision_id",
                "mapex_policy_decision_id",
                "time_s",
                "map_generation",
                "candidate_total",
                "candidate_selectable",
                "candidate_suppressed",
                "ensemble_prediction_ms",
                "all_frontier_scoring_ms",
                "total_computation_ms",
                "selected_x",
                "selected_y",
                "selected_distance_m",
                "selected_information_gain",
                "selected_score",
                "selected_visible_unknown_cells",
                "selected_region_size",
                "selection_verified_max_score",
                "outcome",
                "raw_map",
                "canvas_map",
                "g1_map",
                "g2_map",
                "g3_map",
                "mean_map",
                "variance_map",
            ],
        )
        self.fc, self.wc = self._open(
            "candidates.csv",
            [
                "decision_id",
                "rank_all",
                "rank_selectable",
                "candidate_id",
                "row",
                "col",
                "x",
                "y",
                "distance_m",
                "region_size",
                "information_gain",
                "score",
                "visible_unknown_cells",
                "selectable",
                "suppressed_planner_blocked",
                "selected",
            ],
        )
        self.fg, self.wg = self._open(
            "goals.csv",
            [
                "goal_id",
                "decision_id",
                "mode",
                "start_time_s",
                "end_time_s",
                "target_x",
                "target_y",
                "result",
                "status",
                "error_code",
                "error_msg",
            ],
        )
        self.fp, self.wp = self._open(
            "plans.csv",
            [
                "time_s",
                "goal_id",
                "decision_id",
                "poses",
                "path_length_m",
                "endpoint_x",
                "endpoint_y",
                "frontier_x",
                "frontier_y",
                "endpoint_error_m",
                "usable",
            ],
        )

    # ----- benchmark gate / policy decision instrumentation -----
    def startup_ready(self):
        if self.startup_gate_open:
            return True
        if not self.nav_client.server_is_ready():
            self.nav_ready_since = None
            if not self.startup_wait_logged:
                self.get_logger().info(
                    "MapEx run waiting for NavigateToPose action server "
                    "before benchmark start"
                )
                self.startup_wait_logged = True
            return False
        now = self.now_s()
        if self.nav_ready_since is None:
            self.nav_ready_since = now
            return False
        if now - self.nav_ready_since < NAV2_READY_STABLE_S:
            return False
        if self.map_msg is None or self.robot_position() is None:
            return False
        self.startup_gate_open = True
        self.get_logger().warn(
            f"MAPEX READY: Nav2 stable for >= {NAV2_READY_STABLE_S:.1f}s; "
            "benchmark clock will start at first frontier decision"
        )
        return True

    def predict_maps(self, observed_map: np.ndarray):
        """Capture P1/P2/P3 while preserving the canonical MapEx policy call."""
        result = mapex.MapExExplorer.predict_maps(self, observed_map)
        predictions = result[0]
        if hasattr(predictions, "detach"):
            captured = predictions.detach().float().cpu().numpy()
        else:
            captured = np.asarray(predictions, dtype=np.float32)
        if captured.ndim != 3 or captured.shape[0] != 3:
            raise RuntimeError(
                "Recorder expected three LaMa predictions with shape (3,H,W); "
                f"got {captured.shape}"
            )
        self.last_ensemble_predictions = captured.astype(np.float32, copy=True)
        return result

    def exploration_step(self):
        if not self.startup_ready():
            return
        if self.t0 is None and not self.nav_client.server_is_ready():
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

            # Prevent failed/no-frontier decisions from inheriting old diagnostics.
            self.last_candidate_metrics = []
            self.last_ensemble_predictions = None
            self.last_mean_map = None
            self.last_variance_map = None

            if self.t0 is None:
                self.t0 = self.compute_sim_t0
                self._update_metadata(
                    exploration_start_sim_s=self.t0,
                    evaluation_start_x=float(robot_pose[0]),
                    evaluation_start_y=float(robot_pose[1]),
                    runtime_map_resolution_m=float(self.map_msg.info.resolution),
                )

        # Successful selections call Stage2Run.publish_goal_markers through MRO,
        # which invokes this class's record_decision without altering MapEx policy.
        mapex.MapExExplorer.exploration_step(self)

        if self.compute_t0 is not None:
            self.record_decision([], None, "NO_SELECTION")

    def _save_prediction_maps(
        self,
        decision_id: int,
    ) -> tuple[str, str, str, str, str]:
        if (
            not self.save_predictions
            or self.last_ensemble_predictions is None
            or self.last_mean_map is None
            or self.last_variance_map is None
            or self.map_msg is None
        ):
            return "", "", "", "", ""

        predictions = np.asarray(self.last_ensemble_predictions, dtype=np.float32)
        mean = np.asarray(self.last_mean_map, dtype=np.float32)
        variance = np.asarray(self.last_variance_map, dtype=np.float32)
        if predictions.ndim != 3 or predictions.shape[0] != 3:
            raise RuntimeError(
                f"Cannot save LaMa ensemble: expected (3,H,W), got {predictions.shape}"
            )
        if predictions.shape[1:] != mean.shape or variance.shape != mean.shape:
            raise RuntimeError(
                "Prediction save shape mismatch: "
                f"ensemble={predictions.shape}, mean={mean.shape}, "
                f"variance={variance.shape}"
            )

        source_h = int(self.map_msg.info.height)
        source_w = int(self.map_msg.info.width)
        pad_top = max(0, (mean.shape[0] - source_h) // 2)
        pad_left = max(0, (mean.shape[1] - source_w) // 2)
        pred_dir = self.run / "predictions"
        paths = [
            pred_dir / f"decision_{decision_id:06d}_g1.npz",
            pred_dir / f"decision_{decision_id:06d}_g2.npz",
            pred_dir / f"decision_{decision_id:06d}_g3.npz",
            pred_dir / f"decision_{decision_id:06d}_mean.npz",
            pred_dir / f"decision_{decision_id:06d}_variance.npz",
        ]
        common = {
            "resolution": float(self.map_msg.info.resolution),
            "source_height": source_h,
            "source_width": source_w,
            "pad_top": pad_top,
            "pad_left": pad_left,
            "origin_x": float(self.map_msg.info.origin.position.x),
            "origin_y": float(self.map_msg.info.origin.position.y),
            "environment": self.environment,
        }
        for index in range(3):
            np.savez_compressed(
                paths[index], data=predictions[index], member=f"G{index + 1}", **common
            )
        np.savez_compressed(paths[3], data=mean, member="mean", **common)
        np.savez_compressed(paths[4], data=variance, member="variance", **common)
        return tuple(str(path.relative_to(self.run)) for path in paths)

    def record_decision(self, candidates, selected, outcome):
        ms = (time.perf_counter() - self.compute_t0) * 1000.0
        self.comp.append(ms)
        did = self.active_decision

        raw, canvas = self.save_map_pair(
            self.run / "decision_maps" / f"decision_{did:06d}"
        )
        g1_path, g2_path, g3_path, mean_path, var_path = self._save_prediction_maps(
            did
        )

        metrics = list(self.last_candidate_metrics or [])
        metric_by_grid = {(m["row"], m["col"]): m for m in metrics}
        selectable_keys = {(c[3], c[4]) for c in candidates}
        selected_key = None if selected is None else (selected[3], selected[4])

        prediction_ms = math.nan
        scoring_ms = math.nan
        if metrics:
            prediction_ms = float(metrics[0]["ensemble_prediction_s"]) * 1000.0
            scoring_ms = float(metrics[0]["all_frontier_scoring_s"]) * 1000.0
            self.prediction_ms.append(prediction_ms)
            self.frontier_scoring_ms.append(scoring_ms)

        selected_metric = (
            None if selected_key is None else metric_by_grid.get(selected_key)
        )
        if selected_metric is None:
            sx = sy = sd = sig = ss = sv = sr = math.nan
            verified = ""
        else:
            sx = float(selected_metric["x"])
            sy = float(selected_metric["y"])
            sd = float(selected_metric["distance_m"])
            sig = float(selected_metric["information_gain"])
            ss = float(selected_metric["score"])
            sv = int(selected_metric["visible_unknown_cells"])
            sr = int(selected_metric["region_size"])
            selectable_scores = [
                float(m["score"])
                for m in metrics
                if (m["row"], m["col"]) in selectable_keys
            ]
            best_score = max(selectable_scores) if selectable_scores else math.nan
            verified_bool = (
                math.isfinite(best_score)
                and math.isclose(ss, best_score, rel_tol=1e-12, abs_tol=1e-12)
            )
            verified = int(verified_bool)
            if not verified_bool:
                self.selection_verification_failures += 1
            self.selected_ig.append(sig)
            self.selected_score.append(ss)
            self.selected_distance.append(sd)

        self.wd.writerow(
            {
                "decision_id": did,
                "mapex_policy_decision_id": self.mapex_decision_id if metrics else "",
                "time_s": self.fmt(self.elapsed(self.compute_sim_t0)),
                "map_generation": self.map_generation,
                "candidate_total": len(metrics),
                "candidate_selectable": len(selectable_keys),
                "candidate_suppressed": max(0, len(metrics) - len(selectable_keys)),
                "ensemble_prediction_ms": self.fmt(prediction_ms),
                "all_frontier_scoring_ms": self.fmt(scoring_ms),
                "total_computation_ms": self.fmt(ms),
                "selected_x": self.fmt(sx),
                "selected_y": self.fmt(sy),
                "selected_distance_m": self.fmt(sd),
                "selected_information_gain": self.fmt(sig),
                "selected_score": self.fmt(ss),
                "selected_visible_unknown_cells": self.fmt(sv),
                "selected_region_size": self.fmt(sr),
                "selection_verified_max_score": verified,
                "outcome": outcome,
                "raw_map": raw,
                "canvas_map": canvas,
                "g1_map": g1_path,
                "g2_map": g2_path,
                "g3_map": g3_path,
                "mean_map": mean_path,
                "variance_map": var_path,
            }
        )
        self.fd.flush()

        if selected is not None:
            self.count["frontiers_selected"] += 1

        selectable_rank = 0
        ordered = sorted(metrics, key=lambda metric: float(metric["score"]), reverse=True)
        for rank_all, metric in enumerate(ordered, 1):
            key = (metric["row"], metric["col"])
            selectable = key in selectable_keys
            if selectable:
                selectable_rank += 1
                rank_selectable = selectable_rank
            else:
                rank_selectable = ""
            self.wc.writerow(
                {
                    "decision_id": did,
                    "rank_all": rank_all,
                    "rank_selectable": rank_selectable,
                    "candidate_id": metric["candidate_id"],
                    "row": metric["row"],
                    "col": metric["col"],
                    "x": self.fmt(metric["x"]),
                    "y": self.fmt(metric["y"]),
                    "distance_m": self.fmt(metric["distance_m"]),
                    "region_size": metric["region_size"],
                    "information_gain": self.fmt(metric["information_gain"]),
                    "score": self.fmt(metric["score"]),
                    "visible_unknown_cells": metric["visible_unknown_cells"],
                    "selectable": int(selectable),
                    "suppressed_planner_blocked": int(not selectable),
                    "selected": int(selected_key is not None and key == selected_key),
                }
            )
        self.fc.flush()

        self.compute_t0 = None
        self.compute_sim_t0 = None
        if selected is None:
            self.active_decision = None

    # ----- shared goal handling with explicit 206 accounting -----
    def goal_result_callback(self, future, mode):
        code = None
        try:
            wrapped = future.result()
            code = getattr(wrapped.result, "error_code", None)
        except Exception:
            pass
        super().goal_result_callback(future, mode)
        if mode == "main" and code == 206:
            self.count["abandoned_206"] += 1
            self.active_decision = None

    # ----- environment-specific coverage -----
    def map_metrics(self):
        if self.map_msg is None:
            return math.nan, math.nan
        canvas = self.fixed_canvas(self.map_msg)
        if canvas is None:
            return math.nan, math.nan
        known = canvas >= 0
        known_fraction = np.count_nonzero(known) / known.size
        coverage = (
            np.count_nonzero(known & self.roi) / self.roi_n
            if self.roi is not None and self.roi_n > 0
            else math.nan
        )
        return float(known_fraction), float(coverage)

    # ----- online metrics -----
    def metric_tick(self):
        if self.t0 is None or self.finalized:
            return
        self.known, self.coverage = self.map_metrics()
        main_attempts = self.count["main_attempts"]
        main_succeeded = self.count["main_succeeded"]
        rate = main_succeeded / main_attempts if main_attempts else math.nan
        blocked_total = self.count["abandoned_206"] + self.count["abandoned_208"]

        # IoU/TU remain NaN online and are backfilled after the run.
        self.wm.writerow(
            {
                "time_s": self.fmt(self.elapsed()),
                "distance_m": self.fmt(self.distance),
                "known_fraction": self.fmt(self.known),
                "coverage": self.fmt(self.coverage),
                "occupied_iou": "nan",
                "tu": "nan",
                "frontiers_selected": self.count["frontiers_selected"],
                "main_attempts": main_attempts,
                "main_succeeded": main_succeeded,
                "main_failed": self.count["main_failed"],
                "main_interrupted": self.count["main_interrupted"],
                "abandoned_206": self.count["abandoned_206"],
                "abandoned_208": self.count["abandoned_208"],
                "planner_blocked_abandoned_total": blocked_total,
                "main_success_rate": self.fmt(rate),
                "subgoal_attempts": self.count["subgoal_attempts"],
                "subgoal_succeeded": self.count["subgoal_succeeded"],
                "subgoal_failed": self.count["subgoal_failed"],
                "subgoal_interrupted": self.count["subgoal_interrupted"],
            }
        )
        self.fm.flush()
        if self.elapsed() - self.last_snap >= 10:
            self.save_canvas(self.run / "maps" / f"snapshot_{int(self.elapsed()):06d}")
            self.last_snap = self.elapsed()

    @staticmethod
    def _mean(values):
        return statistics.fmean(values) if values else math.nan

    @staticmethod
    def _std(values):
        return (
            statistics.pstdev(values)
            if len(values) > 1
            else (0.0 if values else math.nan)
        )

    # ----- finish -----
    def finalize(self, reason):
        if self.finalized:
            return
        if self.active_goal is not None:
            self.finish_goal("interrupted", "", "", "run_interrupted")

        self.finalized = True
        self.known, self.coverage = self.map_metrics()
        self.save_map_pair(self.run / "maps" / "final")
        main_attempts = self.count["main_attempts"]
        main_succeeded = self.count["main_succeeded"]
        total_time = None if self.t0 is None else self.elapsed()
        blocked_total = self.count["abandoned_206"] + self.count["abandoned_208"]

        summary = {
            "run_id": self.run.name,
            "method": "mapex",
            "environment": self.environment,
            "runtime_profile": self.runtime_profile["id"],
            "runtime_profile_name": self.runtime_profile_name,
            "evaluation_roi_id": self.environment_profile["roi_id"],
            "evaluation_roi_denominator": self.roi_n if self.roi_n > 0 else None,
            "structural_ground_truth_id": self.environment_profile["ground_truth_id"],
            "final_coverage": self.coverage,
            "final_known_fraction": self.known,
            "total_distance_m": self.distance,
            "total_time_s": total_time,
            "frontiers_selected": self.count["frontiers_selected"],
            "main_attempts": main_attempts,
            "main_succeeded": main_succeeded,
            "main_failed": self.count["main_failed"],
            "main_interrupted": self.count["main_interrupted"],
            "abandoned_206": self.count["abandoned_206"],
            "abandoned_208": self.count["abandoned_208"],
            "planner_blocked_abandoned_total": blocked_total,
            "main_success_rate": (
                main_succeeded / main_attempts if main_attempts else math.nan
            ),
            "subgoal_attempts": self.count["subgoal_attempts"],
            "subgoal_succeeded": self.count["subgoal_succeeded"],
            "subgoal_failed": self.count["subgoal_failed"],
            "subgoal_interrupted": self.count["subgoal_interrupted"],
            "error_code_counts": dict(self.errors),
            "decision_computation_ms_mean": self._mean(self.comp),
            "decision_computation_ms_std": self._std(self.comp),
            "ensemble_prediction_ms_mean": self._mean(self.prediction_ms),
            "ensemble_prediction_ms_std": self._std(self.prediction_ms),
            "all_frontier_scoring_ms_mean": self._mean(self.frontier_scoring_ms),
            "all_frontier_scoring_ms_std": self._std(self.frontier_scoring_ms),
            "selected_information_gain_mean": self._mean(self.selected_ig),
            "selected_score_mean": self._mean(self.selected_score),
            "selected_distance_m_mean": self._mean(self.selected_distance),
            "selection_verification_failures": self.selection_verification_failures,
            "selection_verification_passed": self.selection_verification_failures == 0,
            "termination_reason": reason,
            "nav2_startup_stable_s": NAV2_READY_STABLE_S,
            "occupied_iou_online": None,
            "tu_online": None,
            "prediction_maps_saved": self.save_predictions,
            "prediction_members_saved": 3 if self.save_predictions else 0,
        }
        (self.run / "summary.json").write_text(
            json.dumps(summary, indent=2, allow_nan=True), encoding="utf-8"
        )

        self._update_metadata(
            termination_reason=reason,
            runtime_map_resolution_m=(
                None if self.map_msg is None else float(self.map_msg.info.resolution)
            ),
            final_coverage=self.coverage,
            final_known_fraction=self.known,
            total_distance_m=self.distance,
            total_time_s=total_time,
            selection_verification_failures=self.selection_verification_failures,
        )
        self.get_logger().warn(
            f"MAPEX SAVED: env={self.environment}, "
            f"profile={self.runtime_profile_name}, "
            f"coverage={self.fmt(self.coverage)}, "
            f"distance={self.distance:.2f}m, output={self.run}"
        )

    def close_recorder_files(self):
        """Flush/close CSVs before the offline evaluator rewrites metrics.csv."""
        for name in ("fm", "ft", "fd", "fc", "fg", "fp"):
            stream = getattr(self, name, None)
            if stream is not None and not stream.closed:
                stream.flush()
                stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument(
        "--environment",
        choices=sorted(ENVIRONMENT_PROFILES),
        default="new_room",
        help=(
            "Evaluation environment/profile. Default is new_room; use "
            "--environment hospital for the Hospital benchmark."
        ),
    )
    parser.add_argument(
        "--runtime-profile",
        choices=sorted(RUNTIME_PROFILES),
        default=None,
        help=(
            "Runtime provenance profile. For new_room the default is submap; "
            "use stock2 for launch/stock2.launch.py. Hospital defaults to hospital."
        ),
    )
    parser.add_argument(
        "--save-predictions",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Save per-decision LaMa G1/G2/G3, ensemble mean, and variance NPZ "
            "files (disable with --no-save-predictions to reduce disk use)."
        ),
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=mapex._mapex_init_args(ros_args))
    node = None
    run_dir = None
    ground_truth_path = None
    roi_path = None
    try:
        node = MapExRun(
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
                "MAPEX OFFLINE EVAL: "
                f"env={args.environment}, "
                f"IoU={result['final_occupied_iou']:.6f}, "
                f"TU={result['final_tu']:.6f}, "
                f"decisions={result['evaluated_decisions']}"
            )
        else:
            print(
                "MAPEX OFFLINE EVAL: "
                f"env={args.environment}, {status}: "
                f"{result.get('reason', 'no reason')}"
            )


if __name__ == "__main__":
    main()
