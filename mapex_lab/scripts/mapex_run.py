#!/usr/bin/env python3
"""Run canonical MapEx with the shared exploration recorder and offline evaluation.

The MapEx policy stays in mapex.py. This wrapper reuses nf_run.py's shared
measurement/execution recorder, adds MapEx prediction/scoring diagnostics, and
stores replayable per-decision data without changing frontier selection.

For selected frontiers, recorder/disk I/O is deliberately deferred until after
NavigateToPose has been dispatched. This keeps prediction-map persistence and
benchmark logging off the policy-to-navigation critical path while preserving
all replay artifacts.
"""
from __future__ import annotations

import argparse
import csv
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
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy

import mapex
from evaluate_mapex_profiled import evaluate_run
from generate_new_room_ground_truth import generate as generate_new_room_ground_truth
from mapex_lama_bridge import LamaEnsembleBridge
from nf_basic import MIN_DISTANCE_THRESHOLD, MIN_REGION_SIZE, PLANNER_BLOCKED_SKIP_RADIUS_M
from nf_run import (
    CANVAS_RES,
    CANDIDATE_FIELDS,
    EVALUATION_ROI_ID,
    FIXED_CANVAS_ID,
    NAV2_READY_STABLE_S,
    POLICY_DECISION_FIELDS,
    PROTOCOL_VERSION,
    ROI_N,
    RUNTIME_PROFILES,
    SIM_SEED_POLICY,
    SNAPSHOT_FIELDS,
    PAPER500_BUDGET_ID,
    PAPER500_EVAL_ID,
    PAPER500_PROFILE_ID,
    Stage2Run,
    _deep_merge,
    _git_value,
    _sha256_file,
    _stamp_s,
)


# Keep the canonical MapEx policy untouched; isolate only legacy LaMa inference.
mapex.LamaEnsemble = LamaEnsembleBridge

MAPEX_POLICY_DECISION_FIELDS = POLICY_DECISION_FIELDS + [
    "mapex_policy_decision_id",
    "ensemble_prediction_ms",
    "all_frontier_scoring_ms",
    "prediction_save_ms",
    "selected_information_gain",
    "selected_score",
    "selected_visible_unknown_cells",
]

MAPEX_CANDIDATE_FIELDS = [
    "decision_id",
    "mapex_policy_decision_id",
] + CANDIDATE_FIELDS + [
    "rank_all",
    "rank_selectable",
    "information_gain",
    "score",
    "visible_unknown_cells",
    "ensemble_prediction_ms",
    "all_frontier_scoring_ms",
    "suppressed_planner_blocked",
]

ENVIRONMENT_PROFILES = {
    "new_room": {
        "protocol_version": "new_room_v2",
        "roi_id": "new_room_connected_free_v2",
        "roi_relative": "ground_truth/new_room/generated/new_room_connected_free_v2.npy",
        "ground_truth_id": "new_room_structural_gt_v2",
        "ground_truth_relative": "ground_truth/new_room/generated/new_room_structural_gt_v2.npz",
        "auto_generate_ground_truth": True,
    },
    "hospital": {
        "protocol_version": "hospital_v2_gt_v2_prospective",
        "roi_id": "hospital_connected_free_v2",
        "roi_relative": (
            "analysis/d1/results/mx018_hospital_gt_recovery_v1/gt_v2/"
            "hospital_connected_free_v2.npy"
        ),
        "roi_sha256": "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1",
        "ground_truth_id": "hospital_structural_gt_v2",
        "ground_truth_relative": (
            "analysis/d1/results/mx018_hospital_gt_recovery_v1/gt_v2/"
            "hospital_structural_gt_v2.npz"
        ),
        "ground_truth_sha256": "080c7d708f12ae71c1ed1881bfd630dbb491401d95831f613e3116cb9926dce1",
        "ground_truth_semantic_digest": "d27ba692b313ba784729ea1fd10b2963c20fea9e4465d27081ece9c8757cf4ea",
        "ground_truth_source_revision": "27fad5306f6a2868f93a269b82719fb189d77ebd",
        "auto_generate_ground_truth": False,
    },
}


def _ensure_new_room_ground_truth(root: FilePath) -> None:
    sdf = root / "map" / "new_room.sdf"
    output_dir = root / "ground_truth" / "new_room" / "generated"
    gt = output_dir / "new_room_structural_gt_v2.npz"
    roi = output_dir / "new_room_connected_free_v2.npy"
    summary = output_dir / "new_room_structural_gt_v2_summary.json"

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
        spawn_y=3.0,
        spawn_yaw=0.0,
    )


class MapExRun(Stage2Run, mapex.MapExExplorer):
    """Canonical MapEx policy plus the shared NF/MapEx recorder schema."""

    def __init__(
        self,
        run_id: str,
        odom_topic: str,
        save_predictions: bool,
        environment: str,
        runtime_profile: str | None = None,
        paper500: bool = False,
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
        self.paper500_enabled = bool(paper500)
        if self.paper500_enabled and environment != "new_room":
            raise RuntimeError("R003 paper500 is New Room only")
        if self.environment_profile["auto_generate_ground_truth"]:
            _ensure_new_room_ground_truth(root)

        self.roi_path = root / self.environment_profile["roi_relative"]
        self.ground_truth_path = root / self.environment_profile["ground_truth_relative"]
        if self.environment == "hospital":
            expected_gt = self.environment_profile["ground_truth_sha256"]
            expected_roi = self.environment_profile["roi_sha256"]
            semantic_path = self.ground_truth_path.with_name(
                "gt_v2_semantic_manifest.canonical.json"
            )
            if not self.ground_truth_path.is_file() or _sha256_file(self.ground_truth_path) != expected_gt:
                raise RuntimeError("Hospital GT-v2 identity mismatch before execution")
            if not self.roi_path.is_file() or _sha256_file(self.roi_path) != expected_roi:
                raise RuntimeError("Hospital GT-v2 ROI identity mismatch before execution")
            if (
                not semantic_path.is_file()
                or _sha256_file(semantic_path)
                != self.environment_profile["ground_truth_semantic_digest"]
            ):
                raise RuntimeError("Hospital GT-v2 semantic identity mismatch before execution")
        self.roi = np.load(self.roi_path).astype(bool) if self.roi_path.is_file() else None
        self.roi_n = int(np.count_nonzero(self.roi)) if self.roi is not None else 0
        self.paper500_profile_path = (
            root / "ground_truth" / "new_room" / "generated" / "r003_paper500"
            / f"{PAPER500_EVAL_ID}.npz"
        )
        self.paper500_profile = None
        if self.paper500_enabled:
            if not self.paper500_profile_path.is_file():
                raise RuntimeError(
                    "R003 profile is missing; generate masks and obtain WORK review first"
                )
            self.paper500_profile = np.load(self.paper500_profile_path)
            self.roi = self.paper500_profile["valid_space"].astype(bool)
            self.roi_n = int(self.roi.sum())
            self.ground_truth_path = self.paper500_profile_path
            self.environment_profile.update(
                protocol_version=PAPER500_PROFILE_ID,
                roi_id=PAPER500_EVAL_ID,
                ground_truth_id=PAPER500_EVAL_ID,
            )

        run = root / "experiments" / "mapex" / run_id
        if run.exists():
            raise RuntimeError(f"Run exists: {run}")

        # Stage2Run.__init__ is intentionally skipped because it hard-codes the
        # nearest output directory. MapExExplorer still initializes canonical policy.
        mapex.MapExExplorer.__init__(self)

        self.runtime_profile_name = runtime_profile
        self.runtime_profile = profile
        self.run = run
        self.save_predictions = bool(save_predictions)
        (self.run / "maps").mkdir(parents=True)
        (self.run / "decision_maps").mkdir()
        (self.run / "decisions").mkdir()
        if self.save_predictions:
            (self.run / "predictions").mkdir()

        # Initial provenance records paper500 budget integrity limits, so the
        # budget object must exist before provenance is written.
        from r003_paper500 import OdomProgressBudget
        self.paper500_budget = OdomProgressBudget() if self.paper500_enabled else None
        self.metadata = self._write_initial_provenance(run_id)
        if self.roi is None or self.roi_n <= 0:
            self.get_logger().warn(
                f"{environment} ROI missing/empty: coverage will be NaN: {self.roi_path}"
            )

        # Shared recorder state normally created by Stage2Run.__init__.
        self.t0 = None
        self.last_xy = None
        self.distance = 0.0
        self.last_traj = -1e9
        self.last_snap = -1e9
        self.goal_id = 0
        self.active_goal = None
        self.goal_decision_ids = {}
        self.count = Counter()
        self.errors = Counter()
        self.known = math.nan
        self.coverage = math.nan
        self.finalized = False
        self.nav_ready_since = None
        self.startup_gate_open = False
        self.startup_wait_logged = False
        self.snapshot_id = 0
        # State required by the shared Stage2Run paper500 callbacks.
        import threading
        self.paper500_lock = threading.RLock()
        self.paper500_latest_odom = None
        self.paper500_latest_map = None
        self.paper500_latest_map_receive_s = None
        self.paper500_sample_id = 0
        self.paper500_last_raw_sha256 = None
        self.paper500_cutoff_requested = False
        self.paper500_cutoff_wall_t0 = None
        self.paper500_active_goal_handle = None
        self.paper500_post_cancel_distance_m = None
        self.paper500_cutoff_detection_overshoot_m = None
        self.paper500_cutoff_map_age_s = None
        self.paper500_shutdown_requested = False

        self.decision_id = 0
        self.active_decision = None
        self.compute_t0 = None
        self.compute_sim_t0 = None
        self.decision_compute_ms = None
        self.pending_decision_selection = None
        self.decision_map_msg = None
        self.decision_robot_xy = None
        self.decision_robot_yaw = math.nan
        self.capture_policy_robot = False
        self.policy_compute_ms = []
        self.near_frontier_fallback_count = 0

        # MapEx-specific statistics/state.
        self.comp = []
        self.prediction_ms = []
        self.frontier_scoring_ms = []
        self.prediction_save_ms = []
        self.selected_ig = []
        self.selected_score = []
        self.selected_distance = []
        self.selection_verification_failures = 0
        self.last_ensemble_predictions: np.ndarray | None = None

        q = QoSProfile(depth=100)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, odom_topic, self.odom_cb, q)
        if self.paper500_enabled:
            responsive_group = ReentrantCallbackGroup()
            self.create_subscription(
                Odometry, odom_topic, self.paper500_odom_cb, q,
                callback_group=responsive_group,
            )
            self.create_subscription(
                OccupancyGrid, "/map", self.paper500_map_cb, 20,
                callback_group=responsive_group,
            )
            self.create_timer(
                0.2, self.paper500_control_tick,
                callback_group=responsive_group,
            )
        self._open_files()
        self.create_timer(1.0, self.metric_tick)

        self.get_logger().warn(
            f"MAPEX RECORDING: env={self.environment}, "
            f"profile={self.runtime_profile_name}, output={self.run}"
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
                str(FilePath.home() / "miniforge3" / "envs" / "lama" / "bin" / "python"),
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
            "mapex_profiled_evaluator": self.root / "scripts" / "evaluate_mapex_profiled.py",
            "mapex_policy": self.root / "scripts" / "mapex.py",
            "mapex_lama_bridge": self.root / "scripts" / "mapex_lama_bridge.py",
            "mapex_lama_worker": self.root / "scripts" / "mapex_lama_worker.py",
            "nf_run_recorder_base": self.root / "scripts" / "nf_run.py",
            "nf_basic_shared_execution": self.root / "scripts" / "nf_basic.py",
            "mapex_config": mapex_config,
            "runtime_mapex": runtime_mapex,
            "nav2_override": nav2_override,
            "installed_nav2_base_params": nav2_base,
            "runtime_nav2_merged": merged_path,
            "execution_runner": self.root.parent / ".run_core",
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
                    "ground_truth_generator": self.root / "scripts" / "generate_new_room_ground_truth.py",
                    "ground_truth_contract": self.root / "ground_truth" / "new_room" / "structural_gt_v1.yaml",
                }
            )
        elif self.environment == "hospital":
            hash_paths.update(
                {
                    "hospital_gt_v2": self.ground_truth_path,
                    "hospital_gt_v2_roi": self.roi_path,
                    "hospital_gt_v2_semantic": self.ground_truth_path.with_name(
                        "gt_v2_semantic_manifest.canonical.json"
                    ),
                }
            )
        if self.paper500_enabled:
            hash_paths.update(
                {
                    "r003_protocol": self.root / "docs" / "R003_PAPER500_PROTOCOL_V1.md",
                    "r003_core": self.root / "scripts" / "r003_paper500.py",
                    "r003_evaluator": self.root / "scripts" / "evaluate_r003_paper500.py",
                    "r003_profile": self.paper500_profile_path,
                    "r003_profile_manifest": self.paper500_profile_path.with_name(
                        f"{PAPER500_EVAL_ID}_manifest.json"
                    ),
                }
            )
        config_sha256 = {
            name: _sha256_file(path) for name, path in hash_paths.items() if path.is_file()
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
            "shared_execution": "nf_basic.py",
            "git_commit": git_commit,
            "git_dirty_at_recorder_start": None if git_status is None else bool(git_status),
            "protocol_version": self.environment_profile["protocol_version"],
            "runtime_profile": self.runtime_profile["id"],
            "runtime_profile_name": self.runtime_profile_name,
            "runtime_profile_note": self.runtime_profile["note"],
            "runtime_launch_file": None if active_launch is None else str(active_launch),
            "runtime_slam_file": None if active_slam is None else str(active_slam),
            "runtime_map_resolution_m": None,
            "fixed_canvas_id": FIXED_CANVAS_ID,
            "fixed_canvas_resolution_m": CANVAS_RES,
            "evaluation_roi_id": self.environment_profile["roi_id"],
            "evaluation_roi_denominator": self.roi_n if self.roi_n > 0 else None,
            "evaluation_roi_file": str(self.roi_path),
            "structural_ground_truth_id": self.environment_profile["ground_truth_id"],
            "structural_ground_truth_file": str(self.ground_truth_path),
            "structural_ground_truth_exists_at_start": self.ground_truth_path.is_file(),
            "structural_ground_truth_sha256": self.environment_profile.get("ground_truth_sha256"),
            "structural_ground_truth_semantic_digest": self.environment_profile.get("ground_truth_semantic_digest"),
            "structural_ground_truth_source_revision": self.environment_profile.get("ground_truth_source_revision"),
            "evaluation_roi_sha256": self.environment_profile.get("roi_sha256"),
            "ground_truth_binding_timing": "before_execution" if self.environment == "hospital" else None,
            "cohort_provenance_label": "PROSPECTIVE_GT_V2" if self.environment == "hospital" else None,
            "collection_task": "MX029" if self.environment == "hospital" else None,
            "experimental_early_stop_enabled": False,
            "execution_headless": os.environ.get("MAPEX_RUN_HEADLESS", "False").lower()
            == "true",
            "execution_use_rviz": os.environ.get("MAPEX_RUN_USE_RVIZ", "True").lower()
            == "true",
            "evaluation_start_x": None,
            "evaluation_start_y": None,
            "exploration_start_sim_s": None,
            "exploration_start_source": "first_policy_decision_before_compute",
            "execution_goal_semantics": "exact_frontier_center_xy",
            "goal_yaw_semantics": "neutral_seed_policy_position_only",
            "planner_path_semantics": "navigation_diagnostic_and_recovery_path",
            "selected_path_length_semantics": "nav2_plan_diagnostic_not_executed_trajectory",
            "preferred_min_frontier_distance_m": MIN_DISTANCE_THRESHOLD,
            "near_frontier_fallback": "all_frontiers_below_threshold",
            "frontier_region_min_cells_strictly_greater_than": MIN_REGION_SIZE,
            "distance_metric": "euclidean",
            "recorder_io_semantics": "selected-decision disk I/O occurs after NavigateToPose dispatch",
            "prediction_save_timing_semantics": "measured separately; excluded from policy computation time",
            "runtime_nav2_merged_file": merged_path.name,
            "runtime_mapex_file": runtime_mapex.name,
            "nav2_base_params_file": str(nav2_base),
            "nav2_override_file": str(nav2_override),
            "mapex_config_file": str(mapex_config),
            "mapex_reference_commit_expected": mapex.MAPEX_REFERENCE_COMMIT,
            "mapex_reference_checkout_commit": mapex_reference_checkout,
            "mapex_reference_checkout_dirty": None if mapex_reference_status is None else bool(mapex_reference_status),
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
            "evaluation_profile": PAPER500_PROFILE_ID if self.paper500_enabled else None,
            "budget_semantics": PAPER500_BUDGET_ID if self.paper500_enabled else None,
            "paper500_profile_file": str(self.paper500_profile_path) if self.paper500_enabled else None,
            "paper500_max_odom_gap_s": self.paper500_budget.max_gap_s if self.paper500_enabled else None,
            "paper500_max_odom_increment_m": self.paper500_budget.max_increment_m if self.paper500_enabled else None,
        }
        self._write_metadata(metadata)
        return metadata

    # ----- CSV setup -----
    def _open_files(self):
        self.fm, self.wm = self._open(
            "metrics.csv",
            [
                "time_s", "distance_m", "known_fraction", "coverage", "occupied_iou", "tu",
                "frontiers_selected", "main_attempts", "main_succeeded", "main_failed",
                "main_interrupted", "abandoned_206", "abandoned_208",
                "planner_blocked_abandoned_total", "main_success_rate", "subgoal_attempts",
                "subgoal_succeeded", "subgoal_failed", "subgoal_interrupted",
            ],
        )
        self.ft, self.wt = self._open(
            "trajectory.csv", ["time_s", "x", "y", "yaw", "cumulative_distance_m"]
        )
        self.fpd, self.wpd = self._open("policy_decisions.csv", MAPEX_POLICY_DECISION_FIELDS)
        self.fd, self.wd = self._open(
            "decisions.csv",
            [
                "decision_id", "mapex_policy_decision_id", "time_s", "map_generation",
                "candidate_total", "candidate_selectable", "candidate_suppressed",
                "ensemble_prediction_ms", "all_frontier_scoring_ms", "total_computation_ms",
                "prediction_save_ms",
                "selected_x", "selected_y", "selected_distance_m", "selected_information_gain",
                "selected_score", "selected_visible_unknown_cells", "selected_region_size",
                "selection_verified_max_score", "near_frontier_fallback", "below_1m_count",
                "outcome", "raw_map", "canvas_map", "g1_map", "g2_map", "g3_map",
                "mean_map", "variance_map",
            ],
        )
        self.fc, self.wc = self._open("candidates.csv", MAPEX_CANDIDATE_FIELDS)
        self.fs, self.ws = self._open("snapshots.csv", SNAPSHOT_FIELDS)
        self.fg, self.wg = self._open(
            "goals.csv",
            [
                "goal_id", "decision_id", "mode", "start_time_s", "end_time_s",
                "target_x", "target_y", "result", "status", "error_code", "error_msg",
            ],
        )
        self.fp, self.wp = self._open(
            "plans.csv",
            [
                "time_s", "goal_id", "decision_id", "poses", "path_length_m",
                "endpoint_x", "endpoint_y", "frontier_x", "frontier_y",
                "endpoint_error_m", "usable",
            ],
        )
        if self.paper500_enabled:
            from nf_run import PAPER500_SNAPSHOT_FIELDS
            self.fpaper, self.wpaper = self._open(
                "paper500_snapshots.csv", PAPER500_SNAPSHOT_FIELDS
            )

    # ----- benchmark gate / policy decision instrumentation -----
    def startup_ready(self):
        if self.startup_gate_open:
            return True
        if not self.nav_client.server_is_ready():
            self.nav_ready_since = None
            if not self.startup_wait_logged:
                self.get_logger().info(
                    "MapEx run waiting for NavigateToPose action server before benchmark start"
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

    def publish_goal_markers(self, candidates, selected):
        """Publish markers now, but defer recorder I/O until after goal dispatch."""
        if self.compute_t0 is not None:
            self.decision_compute_ms = (time.perf_counter() - self.compute_t0) * 1000.0
        # Call the canonical policy/execution marker publisher directly, bypassing
        # Stage2Run.publish_goal_markers because that wrapper records immediately.
        result = mapex.MapExExplorer.publish_goal_markers(self, candidates, selected)
        if self.compute_t0 is not None:
            self.pending_decision_selection = (list(candidates), selected)
        return result

    def exploration_step(self):
        if self.paper500_cutoff_requested:
            return
        if not self.startup_ready():
            return
        ready = (
            not self.completed and self.map_msg is not None and not self.goal_active
            and self.main_goal is None and not self.revalidation_active
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

            if self.t0 is None:
                self.t0 = self.compute_sim_t0
                self._update_metadata(
                    exploration_start_sim_s=self.t0,
                    evaluation_start_x=float(robot_pose[0]),
                    evaluation_start_y=float(robot_pose[1]),
                    runtime_map_resolution_m=float(self.map_msg.info.resolution),
                )
                self._start_paper500_if_needed()

        # A successful MapEx selection publishes markers and dispatches
        # NavigateToPose inside this call. Recorder I/O is intentionally deferred
        # until control returns here, so disk writes cannot delay goal dispatch.
        mapex.MapExExplorer.exploration_step(self)
        if self.paper500_cutoff_requested:
            self.compute_t0 = None
            self.compute_sim_t0 = None
            self.pending_decision_selection = None
            return
        if self.pending_decision_selection is not None:
            candidates, selected = self.pending_decision_selection
            self.pending_decision_selection = None
            self.record_decision(candidates, selected, "SELECTED")
        elif self.compute_t0 is not None:
            self.record_decision([], None, "NO_SELECTION")

    def _save_prediction_maps(self, decision_id: int) -> tuple[str, str, str, str, str]:
        source_msg = getattr(self, "decision_map_msg", None) or self.map_msg
        if (
            not self.save_predictions
            or self.last_ensemble_predictions is None
            or self.last_mean_map is None
            or self.last_variance_map is None
            or source_msg is None
        ):
            return "", "", "", "", ""

        predictions = np.asarray(self.last_ensemble_predictions, dtype=np.float32)
        mean = np.asarray(self.last_mean_map, dtype=np.float32)
        variance = np.asarray(self.last_variance_map, dtype=np.float32)
        if predictions.ndim != 3 or predictions.shape[0] != 3:
            raise RuntimeError(f"Cannot save LaMa ensemble: expected (3,H,W), got {predictions.shape}")
        if predictions.shape[1:] != mean.shape or variance.shape != mean.shape:
            raise RuntimeError(
                f"Prediction save shape mismatch: ensemble={predictions.shape}, "
                f"mean={mean.shape}, variance={variance.shape}"
            )

        source_h = int(source_msg.info.height)
        source_w = int(source_msg.info.width)
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
            "resolution": float(source_msg.info.resolution),
            "source_height": source_h,
            "source_width": source_w,
            "pad_top": pad_top,
            "pad_left": pad_left,
            "origin_x": float(source_msg.info.origin.position.x),
            "origin_y": float(source_msg.info.origin.position.y),
            "source_map_stamp_s": (
                _stamp_s(source_msg) if hasattr(source_msg, "header") else float("nan")
            ),
            "environment": self.environment,
        }
        for index in range(3):
            np.savez_compressed(paths[index], data=predictions[index], member=f"G{index + 1}", **common)
        np.savez_compressed(paths[3], data=mean, member="mean", **common)
        np.savez_compressed(paths[4], data=variance, member="variance", **common)
        return tuple(str(path.relative_to(self.run)) for path in paths)

    def record_decision(self, candidates, selected, outcome):
        if self.compute_t0 is None or self.active_decision is None:
            return
        ms = self.decision_compute_ms
        if ms is None:
            ms = (time.perf_counter() - self.compute_t0) * 1000.0
        self.comp.append(float(ms))
        self.policy_compute_ms.append(float(ms))
        did = int(self.active_decision)

        source_msg = self.decision_map_msg or self.map_msg
        decision_dir = self.run / "decisions" / f"policy_decision_{did:06d}"
        decision_dir.mkdir(parents=True, exist_ok=True)
        raw, canvas = self.save_map_pair(decision_dir / "observed_map", msg=source_msg)
        # Keep legacy flat decision-map paths for existing evaluator compatibility.
        legacy_raw, legacy_canvas = self.save_map_pair(
            self.run / "decision_maps" / f"decision_{did:06d}", msg=source_msg
        )

        prediction_save_start = time.perf_counter()
        g1_path, g2_path, g3_path, mean_path, var_path = self._save_prediction_maps(did)
        if any((g1_path, g2_path, g3_path, mean_path, var_path)):
            prediction_save_ms = (time.perf_counter() - prediction_save_start) * 1000.0
            self.prediction_save_ms.append(float(prediction_save_ms))
        else:
            prediction_save_ms = math.nan

        metrics = list(self.last_candidate_metrics or [])
        metric_by_grid = {(int(m["row"]), int(m["col"])): m for m in metrics}
        selectable_keys = {(int(c[3]), int(c[4])) for c in candidates}
        selected_key = None if selected is None else (int(selected[3]), int(selected[4]))

        distant = [m for m in metrics if float(m["distance_m"]) >= MIN_DISTANCE_THRESHOLD]
        near_fallback = bool(metrics) and not distant
        distance_eligible = distant if distant else metrics
        distance_eligible_keys = {(int(m["row"]), int(m["col"])) for m in distance_eligible}
        if near_fallback:
            self.near_frontier_fallback_count += 1

        prediction_ms = math.nan
        scoring_ms = math.nan
        if metrics:
            prediction_ms = float(metrics[0]["ensemble_prediction_s"]) * 1000.0
            scoring_ms = float(metrics[0]["all_frontier_scoring_s"]) * 1000.0
            self.prediction_ms.append(prediction_ms)
            self.frontier_scoring_ms.append(scoring_ms)

        ordered = sorted(
            metrics,
            key=lambda metric: (-float(metric["score"]), int(metric["row"]), int(metric["col"])),
        )
        raw_rank = {
            (int(metric["row"]), int(metric["col"])): rank
            for rank, metric in enumerate(ordered, 1)
        }
        selectable_ordered = [
            metric for metric in ordered
            if (int(metric["row"]), int(metric["col"])) in selectable_keys
        ]
        policy_rank = {
            (int(metric["row"]), int(metric["col"])): rank
            for rank, metric in enumerate(selectable_ordered, 1)
        }

        selected_metric = None if selected_key is None else metric_by_grid.get(selected_key)
        if selected_metric is None:
            sx = sy = sd = sig = ss = sv = sr = math.nan
            selected_candidate_id = ""
            selected_rank = ""
            verified = ""
        else:
            sx = float(selected_metric["x"])
            sy = float(selected_metric["y"])
            sd = float(selected_metric["distance_m"])
            sig = float(selected_metric["information_gain"])
            ss = float(selected_metric["score"])
            sv = int(selected_metric["visible_unknown_cells"])
            sr = int(selected_metric["region_size"])
            selected_candidate_id = selected_metric["candidate_id"]
            selected_rank = policy_rank.get(selected_key, "")
            best_score = max((float(m["score"]) for m in selectable_ordered), default=math.nan)
            verified_bool = math.isfinite(best_score) and math.isclose(
                ss, best_score, rel_tol=1e-12, abs_tol=1e-12
            )
            verified = int(verified_bool)
            if not verified_bool:
                self.selection_verification_failures += 1
            self.selected_ig.append(sig)
            self.selected_score.append(ss)
            self.selected_distance.append(sd)

        candidate_rows = []
        for metric in ordered:
            key = (int(metric["row"]), int(metric["col"]))
            is_distance_eligible = key in distance_eligible_keys
            execution_suppressed = (
                self.is_execution_blocked_suppressed(float(metric["x"]), float(metric["y"]))
                if is_distance_eligible else False
            )
            planner_suppressed = (
                any(
                    math.hypot(float(metric["x"]) - px, float(metric["y"]) - py)
                    <= PLANNER_BLOCKED_SKIP_RADIUS_M
                    for px, py in self.planner_blocked_goals
                )
                if is_distance_eligible else False
            )
            selectable = key in selectable_keys
            is_selected = selected_key is not None and key == selected_key
            if is_selected:
                status = "selected"
            elif not is_distance_eligible:
                status = "distance_deferred_lt_1m"
            elif execution_suppressed:
                status = "execution_suppressed"
            elif planner_suppressed:
                status = "planner_suppressed"
            elif selectable:
                status = "ranked"
            else:
                status = "not_selectable"

            row = {
                "decision_id": did,
                "mapex_policy_decision_id": self.mapex_decision_id if metrics else "",
                "policy_decision_id": did,
                "candidate_id": metric["candidate_id"],
                "raw_rank": raw_rank[key],
                "policy_rank": policy_rank.get(key, ""),
                "row": int(metric["row"]),
                "col": int(metric["col"]),
                "x": self.fmt(float(metric["x"])),
                "y": self.fmt(float(metric["y"])),
                "distance_cells": self.fmt(float(metric["distance_m"]) / float(source_msg.info.resolution)) if source_msg is not None else "",
                "distance_m": self.fmt(float(metric["distance_m"])),
                "region_size": int(metric["region_size"]),
                "below_1m": int(float(metric["distance_m"]) < MIN_DISTANCE_THRESHOLD),
                "distance_eligible": int(is_distance_eligible),
                "near_frontier_fallback": int(near_fallback),
                "execution_suppressed": int(execution_suppressed),
                "planner_suppressed": int(planner_suppressed),
                "selectable": int(selectable),
                "status": status,
                "selected": int(is_selected),
                "execution_result": "",
                "rank_all": raw_rank[key],
                "rank_selectable": policy_rank.get(key, ""),
                "information_gain": self.fmt(float(metric["information_gain"])),
                "score": self.fmt(float(metric["score"])),
                "visible_unknown_cells": int(metric["visible_unknown_cells"]),
                "ensemble_prediction_ms": self.fmt(prediction_ms),
                "all_frontier_scoring_ms": self.fmt(scoring_ms),
                "suppressed_planner_blocked": int(execution_suppressed or planner_suppressed),
            }
            candidate_rows.append(row)
            self.wc.writerow(row)
        self.fc.flush()

        below_count = sum(int(row["below_1m"]) for row in candidate_rows)
        distance_eligible_count = sum(int(row["distance_eligible"]) for row in candidate_rows)
        selectable_count = sum(int(row["selectable"]) for row in candidate_rows)
        suppressed_count = sum(
            1 for row in candidate_rows
            if int(row["distance_eligible"]) and not int(row["selectable"])
        )
        frontier_region_count = len(metrics)
        if selected_metric is not None:
            normalized_outcome = "selected"
        elif not metrics:
            normalized_outcome = "no_selection"
        elif distance_eligible_count and selectable_count == 0:
            normalized_outcome = "no_normally_selectable_candidate"
        else:
            normalized_outcome = outcome.lower()

        robot_x = robot_y = math.nan
        if self.decision_robot_xy is not None:
            robot_x, robot_y = self.decision_robot_xy
        common_decision = {
            "policy_decision_id": did,
            "sim_time_s": self.fmt(self.elapsed(self.compute_sim_t0)),
            "map_stamp_s": self.fmt(_stamp_s(source_msg)),
            "map_generation": self.map_generation,
            "robot_x": self.fmt(float(robot_x)),
            "robot_y": self.fmt(float(robot_y)),
            "robot_yaw": self.fmt(float(self.decision_robot_yaw)),
            "candidate_compute_ms": self.fmt(float(ms)),
            "frontier_region_count": frontier_region_count,
            "candidate_count": len(metrics),
            "distance_eligible_count": distance_eligible_count,
            "selectable_count": selectable_count,
            "suppressed_count": suppressed_count,
            "below_1m_count": below_count,
            "preferred_min_distance_m": self.fmt(MIN_DISTANCE_THRESHOLD),
            "near_frontier_fallback": int(near_fallback),
            "selected_candidate_id": selected_candidate_id,
            "selected_rank": selected_rank,
            "selected_x": self.fmt(sx),
            "selected_y": self.fmt(sy),
            "selected_distance_m": self.fmt(sd),
            "planner_revalidation_started": int(bool(self.revalidation_active)),
            "outcome": normalized_outcome,
            "raw_map": raw,
            "canvas_map": canvas,
            "mapex_policy_decision_id": self.mapex_decision_id if metrics else "",
            "ensemble_prediction_ms": self.fmt(prediction_ms),
            "all_frontier_scoring_ms": self.fmt(scoring_ms),
            "prediction_save_ms": self.fmt(prediction_save_ms),
            "selected_information_gain": self.fmt(sig),
            "selected_score": self.fmt(ss),
            "selected_visible_unknown_cells": self.fmt(sv),
        }
        self.wpd.writerow(common_decision)
        self.fpd.flush()

        self.wd.writerow(
            {
                "decision_id": did,
                "mapex_policy_decision_id": self.mapex_decision_id if metrics else "",
                "time_s": self.fmt(self.elapsed(self.compute_sim_t0)),
                "map_generation": self.map_generation,
                "candidate_total": len(metrics),
                "candidate_selectable": selectable_count,
                "candidate_suppressed": suppressed_count,
                "ensemble_prediction_ms": self.fmt(prediction_ms),
                "all_frontier_scoring_ms": self.fmt(scoring_ms),
                "total_computation_ms": self.fmt(ms),
                "prediction_save_ms": self.fmt(prediction_save_ms),
                "selected_x": self.fmt(sx),
                "selected_y": self.fmt(sy),
                "selected_distance_m": self.fmt(sd),
                "selected_information_gain": self.fmt(sig),
                "selected_score": self.fmt(ss),
                "selected_visible_unknown_cells": self.fmt(sv),
                "selected_region_size": self.fmt(sr),
                "selection_verified_max_score": verified,
                "near_frontier_fallback": int(near_fallback),
                "below_1m_count": below_count,
                "outcome": normalized_outcome,
                "raw_map": legacy_raw,
                "canvas_map": legacy_canvas,
                "g1_map": g1_path,
                "g2_map": g2_path,
                "g3_map": g3_path,
                "mean_map": mean_path,
                "variance_map": var_path,
            }
        )
        self.fd.flush()

        with (decision_dir / "candidates.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=MAPEX_CANDIDATE_FIELDS)
            writer.writeheader()
            writer.writerows(candidate_rows)
        (decision_dir / "decision.json").write_text(
            json.dumps(common_decision, indent=2, allow_nan=True), encoding="utf-8"
        )

        self.compute_t0 = None
        self.compute_sim_t0 = None
        self.decision_compute_ms = None
        self.decision_map_msg = None
        self.decision_robot_xy = None
        self.decision_robot_yaw = math.nan
        if selected is None:
            self.active_decision = None

    @staticmethod
    def _mean(values):
        return statistics.fmean(values) if values else math.nan

    @staticmethod
    def _std(values):
        return statistics.pstdev(values) if len(values) > 1 else (0.0 if values else math.nan)

    # Shared metric_tick(), odom_cb(), plan/goal logging, snapshots, and Nav2
    # result accounting come from Stage2Run. This avoids NF/MapEx schema drift.

    def finalize(self, reason):
        if self.finalized:
            return
        if self.paper500_enabled and reason != "budget_500_reached":
            event = "natural_completion" if reason == "exploration_complete_before_budget" else "abnormal_final"
            if self.paper500_budget.started:
                self._record_paper500_sample(
                    event, self.paper500_budget.step, False, 0.0
                )
        if self.active_goal is not None:
            self.finish_goal("interrupted", "", "", "run_interrupted")

        self.finalized = True
        self.known, self.coverage = self.map_metrics()
        self.record_snapshot("final")
        main_attempts = self.count["main_attempts"]
        main_succeeded = self.count["main_succeeded"]
        total_time = None if self.t0 is None else self.elapsed()
        blocked_total = self.count["abandoned_206"] + self.count["abandoned_208"]
        decision_count = int(self.decision_id)

        summary = {
            "run_id": self.run.name,
            "method": "mapex",
            "environment": self.environment,
            "runtime_profile": self.runtime_profile["id"],
            "runtime_profile_name": self.runtime_profile_name,
            "evaluation_roi_id": self.environment_profile["roi_id"],
            "evaluation_roi_denominator": self.roi_n if self.roi_n > 0 else None,
            "structural_ground_truth_id": self.environment_profile["ground_truth_id"],
            "structural_ground_truth_sha256": self.environment_profile.get("ground_truth_sha256"),
            "structural_ground_truth_semantic_digest": self.environment_profile.get("ground_truth_semantic_digest"),
            "final_coverage": self.coverage,
            "final_known_fraction": self.known,
            "total_distance_m": self.distance,
            "total_time_s": total_time,
            "frontiers_selected": self.count["frontiers_selected"],
            "policy_decisions": decision_count,
            "near_frontier_fallback_count": self.near_frontier_fallback_count,
            "near_frontier_fallback_fraction": (
                self.near_frontier_fallback_count / decision_count if decision_count else math.nan
            ),
            "main_attempts": main_attempts,
            "main_succeeded": main_succeeded,
            "main_failed": self.count["main_failed"],
            "main_interrupted": self.count["main_interrupted"],
            "abandoned_206": self.count["abandoned_206"],
            "abandoned_208": self.count["abandoned_208"],
            "planner_blocked_abandoned_total": blocked_total,
            "main_success_rate": main_succeeded / main_attempts if main_attempts else math.nan,
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
            "prediction_save_ms_mean": self._mean(self.prediction_save_ms),
            "prediction_save_ms_std": self._std(self.prediction_save_ms),
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
            "prediction_save_semantics": "post_goal_dispatch",
            "paper500_profile": PAPER500_PROFILE_ID if self.paper500_enabled else None,
            "paper500_progress_step": self.paper500_budget.step if self.paper500_enabled else None,
            "paper500_distance_m": self.paper500_budget.distance_m if self.paper500_enabled else None,
            "paper500_post_cancellation_distance_m": self.paper500_post_cancel_distance_m if self.paper500_enabled else None,
            "paper500_cutoff_detection_overshoot_m": self.paper500_cutoff_detection_overshoot_m if self.paper500_enabled else None,
            "paper500_cutoff_map_age_s": self.paper500_cutoff_map_age_s if self.paper500_enabled else None,
            "paper500_integrity_faults": list(self.paper500_budget.integrity_faults) if self.paper500_enabled else [],
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
            policy_decisions=decision_count,
            near_frontier_fallback_count=self.near_frontier_fallback_count,
            planner_blocked_abandoned_total=blocked_total,
            selection_verification_failures=self.selection_verification_failures,
            prediction_save_ms_mean=self._mean(self.prediction_save_ms),
            prediction_save_ms_std=self._std(self.prediction_save_ms),
            paper500_progress_step=self.paper500_budget.step if self.paper500_enabled else None,
            paper500_distance_m=self.paper500_budget.distance_m if self.paper500_enabled else None,
            paper500_post_cancellation_distance_m=self.paper500_post_cancel_distance_m if self.paper500_enabled else None,
            paper500_cutoff_detection_overshoot_m=self.paper500_cutoff_detection_overshoot_m if self.paper500_enabled else None,
            paper500_cutoff_map_age_s=self.paper500_cutoff_map_age_s if self.paper500_enabled else None,
        )
        self.get_logger().warn(
            f"MAPEX SAVED: env={self.environment}, profile={self.runtime_profile_name}, "
            f"coverage={self.fmt(self.coverage)}, distance={self.distance:.2f}m, output={self.run}"
        )
        if self.paper500_enabled:
            self.paper500_shutdown_requested = True

    def close_recorder_files(self):
        """Flush/close CSVs before the offline evaluator rewrites metrics.csv."""
        for name in ("fm", "ft", "fpd", "fd", "fc", "fs", "fg", "fp", "fpaper"):
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
        help="Evaluation environment/profile. Default is new_room; use hospital for Hospital.",
    )
    parser.add_argument(
        "--runtime-profile",
        choices=sorted(RUNTIME_PROFILES),
        default=None,
        help="Runtime provenance profile. New Room defaults to submap.",
    )
    parser.add_argument(
        "--save-predictions",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Save per-decision LaMa G1/G2/G3, ensemble mean, and variance NPZ "
            "files after goal dispatch (disable with --no-save-predictions to reduce disk use)."
        ),
    )
    parser.add_argument("--paper500", action="store_true", help="enable approved R003 New Room paper500 profile")
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
            args.paper500,
        )
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        while rclpy.ok() and not node.completed:
            executor.spin_once(timeout_sec=0.2)
        if node.completed and not node.finalized:
            node.finalize("exploration_complete")
        executor.shutdown()
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

    if run_dir is not None and ground_truth_path is not None and roi_path is not None and not args.paper500:
        result = evaluate_run(run_dir, ground_truth_path, roi_path)
        status = result.get("status", "unknown")
        if status == "ok":
            print(
                "MAPEX OFFLINE EVAL: "
                f"env={args.environment}, IoU={result['final_occupied_iou']:.6f}, "
                f"TU={result['final_tu']:.6f}, decisions={result['evaluated_decisions']}"
            )
        else:
            print(
                "MAPEX OFFLINE EVAL: "
                f"env={args.environment}, {status}: {result.get('reason', 'no reason')}"
            )


if __name__ == "__main__":
    main()
