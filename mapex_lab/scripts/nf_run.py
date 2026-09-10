#!/usr/bin/env python3
"""Run canonical nf_basic.py with an integrated Nearest-Frontier recorder.

The exploration policy remains in nf_basic.py and is inherited unchanged from
nf_basic.NearestEuclideanFrontier. This wrapper adds benchmark setup, provenance,
policy-decision replay data, periodic/final map snapshots, trajectory/goal/path
logging, and summary metrics.

New Room defaults to the ``submap`` runtime profile. Other launch profiles are
selected explicitly with --runtime-profile. Hospital remains available as the
legacy benchmark profile.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import subprocess
import time
from collections import Counter
from pathlib import Path as FilePath

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy

from generate_new_room_ground_truth import generate as generate_new_room_ground_truth
from nf_basic import (
    GOAL_OCCUPIED_ERROR_CODE,
    MAIN_PLAN_ENDPOINT_TOLERANCE_M,
    MAP_FRAME,
    MIN_DISTANCE_THRESHOLD,
    MIN_REGION_SIZE,
    NO_VALID_PATH_ERROR_CODE,
    PLANNER_BLOCKED_SKIP_RADIUS_M,
    ROBOT_FRAME,
    NearestEuclideanFrontier,
)

CANVAS_RES = 0.05
CANVAS_W = 1504
CANVAS_H = 2123
CANVAS_X = -25.6
CANVAS_Y = -60.1

# Legacy Hospital constants are kept because mapex_run.py imports them.
ROI_N = 215435
PROTOCOL_VERSION = "hospital_v2"
FIXED_CANVAS_ID = "hospital_canvas_v1"
EVALUATION_ROI_ID = "hospital_connected_free_v1"

NAV2_READY_STABLE_S = 3.0
SIM_SEED_POLICY = "intentionally_uncontrolled_gazebo_default_multiple_run_statistics"

POLICY_DECISION_FIELDS = [
    "policy_decision_id",
    "sim_time_s",
    "map_stamp_s",
    "map_generation",
    "robot_x",
    "robot_y",
    "robot_yaw",
    "candidate_compute_ms",
    "frontier_region_count",
    "candidate_count",
    "distance_eligible_count",
    "selectable_count",
    "suppressed_count",
    "below_1m_count",
    "preferred_min_distance_m",
    "near_frontier_fallback",
    "selected_candidate_id",
    "selected_rank",
    "selected_x",
    "selected_y",
    "selected_distance_m",
    "planner_revalidation_started",
    "outcome",
    "raw_map",
    "canvas_map",
]

CANDIDATE_FIELDS = [
    "policy_decision_id",
    "candidate_id",
    "raw_rank",
    "policy_rank",
    "row",
    "col",
    "x",
    "y",
    "distance_cells",
    "distance_m",
    "region_size",
    "below_1m",
    "distance_eligible",
    "near_frontier_fallback",
    "execution_suppressed",
    "planner_suppressed",
    "selectable",
    "status",
    "selected",
    "execution_result",
]

SNAPSHOT_FIELDS = [
    "snapshot_id",
    "event",
    "time_s",
    "coverage",
    "known_fraction",
    "distance_m",
    "robot_map_x",
    "robot_map_y",
    "robot_map_yaw",
    "raw_map_file",
    "canvas_map_file",
    "source_map_stamp_s",
    "canvas_id",
]


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


# Runtime provenance lives here so nf_run.py and mapex_run.py use one source of truth.
# The oldmap profile is intentionally absent because oldmap_toolbox.launch.py is no
# longer present on main.
RUNTIME_PROFILES = {
    "submap": {
        "environment": "new_room",
        "id": "new_room_submap",
        "note": (
            "New Room benchmark launched with launch/submap.launch.py. SLAM "
            "consumes /scan_submap from scripts/submap.py; shared config/slam.yaml "
            "is overridden at launch only for the scan topic."
        ),
        "launch_relative": "launch/submap.launch.py",
        "slam_relative": "config/slam.yaml",
        "extra_hashes": {
            "submap_frontend": "scripts/submap.py",
            "local_scan_frontend": "scripts/local_scan.py",
        },
    },
    "local": {
        "environment": "new_room",
        "id": "new_room_local",
        "note": (
            "New Room benchmark launched with launch/local.launch.py using the "
            "local scan frontend from scripts/local_scan.py."
        ),
        "launch_relative": "launch/local.launch.py",
        "slam_relative": "config/slam.yaml",
        "extra_hashes": {
            "local_scan_frontend": "scripts/local_scan.py",
        },
    },
    "stock": {
        "environment": "new_room",
        "id": "new_room_stock",
        "note": (
            "New Room benchmark launched with launch/stock.launch.py using the "
            "stock SLAM scan path."
        ),
        "launch_relative": "launch/stock.launch.py",
        "slam_relative": "config/slam.yaml",
        "extra_hashes": {},
    },
    "toolbox": {
        "environment": "new_room",
        "id": "new_room_toolbox_tuned_loop",
        "note": (
            "New Room benchmark launched with launch/toolbox.launch.py using "
            "upstream SLAM Toolbox online_async plus the tuned loop-closure and "
            "scan-cadence overrides embedded in that launch file."
        ),
        "launch_relative": "launch/toolbox.launch.py",
        "slam_relative": None,
        "extra_hashes": {},
    },
    "new_toolbox": {
        "environment": "new_room",
        "id": "new_room_new_toolbox_adaptive_v1_buffer30",
        "note": (
            "New Room benchmark launched with launch/new_toolbox.launch.py using "
            "vendored SLAM Toolbox + Adaptive Temporal Anchor V1, scan_buffer_size=30, "
            "and the Nav2 settings embedded/merged by that launch file."
        ),
        "launch_relative": "launch/new_toolbox.launch.py",
        "slam_relative": None,
        "extra_hashes": {
            "adaptive_ceres_solver": "../slam_toolbox/solvers/ceres_solver.cpp",
        },
    },
    "hospital": {
        "environment": "hospital",
        "id": "hospital_not_auto_detected",
        "note": (
            "Legacy Hospital benchmark. The recorder stores the shared SLAM and "
            "Nav2 configuration; the exact launch file is not auto-detected."
        ),
        "launch_relative": None,
        "slam_relative": "config/slam.yaml",
        "extra_hashes": {},
    },
}


def _deep_merge(base: dict, override: dict) -> dict:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _sha256_file(path: FilePath) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git_value(repo_root: FilePath, *args: str) -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(repo_root), *args],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def _stamp_s(msg) -> float:
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None:
        return math.nan
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _yaw_from_quaternion(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class Stage2Run(NearestEuclideanFrontier):
    def __init__(
        self,
        run_id: str,
        odom_topic: str,
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
            self._ensure_new_room_ground_truth(root)

        self.roi_path = root / self.environment_profile["roi_relative"]
        self.ground_truth_path = root / self.environment_profile[
            "ground_truth_relative"
        ]
        self.roi = (
            np.load(self.roi_path).astype(bool)
            if self.roi_path.is_file()
            else None
        )
        self.roi_n = int(np.count_nonzero(self.roi)) if self.roi is not None else 0

        run = root / "experiments" / "nearest" / run_id
        if run.exists():
            raise RuntimeError(f"Run exists: {run}")

        super().__init__()

        self.runtime_profile_name = runtime_profile
        self.runtime_profile = profile
        self.run = run
        (self.run / "maps").mkdir(parents=True)
        (self.run / "decisions").mkdir()

        self.metadata = self._write_initial_provenance(run_id)

        if self.roi is None or self.roi_n <= 0:
            self.get_logger().warn(
                f"{environment} ROI missing/empty: coverage will be NaN: "
                f"{self.roi_path}"
            )

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

        self.decision_id = 0
        self.active_decision = None
        self.compute_t0 = None
        self.compute_sim_t0 = None
        self.decision_compute_ms = None
        self.pending_decision_selection = None
        self.decision_map_msg = None
        self.decision_robot_xy = None
        self.capture_policy_robot = False
        self.policy_compute_ms = []
        self.near_frontier_fallback_count = 0
        self.snapshot_id = 0

        q = QoSProfile(depth=100)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, odom_topic, self.odom_cb, q)
        self._open_files()
        self.create_timer(1.0, self.metric_tick)

        self.get_logger().warn(
            f"NEAREST RECORDING: env={self.environment}, "
            f"profile={self.runtime_profile_name}, output={self.run}"
        )

    @staticmethod
    def _ensure_new_room_ground_truth(root: FilePath) -> None:
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

    def _open(self, name, fields):
        stream = (self.run / name).open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        return stream, writer

    def _write_metadata(self, metadata: dict) -> None:
        (self.run / "metadata.json").write_text(
            json.dumps(metadata, indent=2, allow_nan=True), encoding="utf-8"
        )

    def _update_metadata(self, **changes) -> None:
        self.metadata.update(changes)
        self._write_metadata(self.metadata)

    def _write_initial_provenance(self, run_id: str) -> dict:
        tb4_nav_pkg = FilePath(get_package_share_directory("turtlebot4_navigation"))
        nav2_base = tb4_nav_pkg / "config" / "nav2.yaml"
        nav2_override = self.root / "config" / "nav2.yaml"

        with nav2_base.open("r", encoding="utf-8") as stream:
            base_cfg = yaml.safe_load(stream) or {}
        with nav2_override.open("r", encoding="utf-8") as stream:
            override_cfg = yaml.safe_load(stream) or {}

        merged_cfg = _deep_merge(base_cfg, override_cfg)
        merged_path = self.run / "runtime_nav2_merged.yaml"
        with merged_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(merged_cfg, stream, sort_keys=False)

        git_commit = _git_value(self.repo_root, "rev-parse", "HEAD")
        git_status = _git_value(self.repo_root, "status", "--porcelain")
        slam_relative = self.runtime_profile["slam_relative"]
        launch_relative = self.runtime_profile["launch_relative"]
        active_slam = self.root / slam_relative if slam_relative else None
        active_launch = self.root / launch_relative if launch_relative else None

        hash_paths = {
            "nf_run_recorder": self.root / "scripts" / "nf_run.py",
            "nf_basic_policy": self.root / "scripts" / "nf_basic.py",
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
                    "ground_truth_generator": self.root / "scripts" / "generate_new_room_ground_truth.py",
                    "ground_truth_contract": self.root / "ground_truth" / "new_room" / "structural_gt_v1.yaml",
                }
            )

        config_sha256 = {
            name: _sha256_file(path)
            for name, path in hash_paths.items()
            if path.is_file()
        }

        metadata = {
            "run_id": run_id,
            "method": "nearest_frontier_euclidean",
            "environment": self.environment,
            "recorder": "nf_run.py",
            "canonical_policy": "nf_basic.py",
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
            "exploration_start_sim_s": None,
            "exploration_start_source": "first_policy_decision_before_compute",
            "execution_goal_semantics": "exact_frontier_center_xy",
            "goal_yaw_semantics": "neutral_seed_policy_position_only",
            "planner_path_semantics": "navigation_diagnostic_and_recovery_path",
            "preferred_min_frontier_distance_m": MIN_DISTANCE_THRESHOLD,
            "near_frontier_fallback": "all_frontiers_below_threshold",
            "frontier_region_min_cells_strictly_greater_than": MIN_REGION_SIZE,
            "distance_metric": "euclidean",
            "runtime_nav2_merged_file": merged_path.name,
            "nav2_base_params_file": str(nav2_base),
            "nav2_override_file": str(nav2_override),
            "sim_seed": None,
            "sim_seed_policy": SIM_SEED_POLICY,
            "config_sha256": config_sha256,
            "termination_reason": None,
        }
        self._write_metadata(metadata)
        return metadata

    def _open_files(self):
        self.fm, self.wm = self._open("metrics.csv", [
            "time_s", "distance_m", "known_fraction", "coverage", "occupied_iou", "tu",
            "frontiers_selected", "main_attempts", "main_succeeded", "main_failed",
            "main_interrupted", "abandoned_206", "abandoned_208",
            "planner_blocked_abandoned_total", "main_success_rate", "subgoal_attempts",
            "subgoal_succeeded", "subgoal_failed", "subgoal_interrupted",
        ])
        self.ft, self.wt = self._open(
            "trajectory.csv", ["time_s", "x", "y", "yaw", "cumulative_distance_m"]
        )
        self.fpd, self.wpd = self._open("policy_decisions.csv", POLICY_DECISION_FIELDS)
        self.fc, self.wc = self._open("candidates.csv", CANDIDATE_FIELDS)
        self.fs, self.ws = self._open("snapshots.csv", SNAPSHOT_FIELDS)
        self.fg, self.wg = self._open("goals.csv", [
            "goal_id", "decision_id", "mode", "start_time_s", "end_time_s",
            "target_x", "target_y", "result", "status", "error_code", "error_msg",
        ])
        self.fp, self.wp = self._open("plans.csv", [
            "time_s", "goal_id", "decision_id", "poses", "path_length_m",
            "endpoint_x", "endpoint_y", "frontier_x", "frontier_y",
            "endpoint_error_m", "usable",
        ])

    def startup_ready(self):
        if self.startup_gate_open:
            return True
        if not self.nav_client.server_is_ready():
            self.nav_ready_since = None
            if not self.startup_wait_logged:
                self.get_logger().info("Nearest run waiting for NavigateToPose action server before benchmark start")
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
            f"NEAREST READY: Nav2 stable for >= {NAV2_READY_STABLE_S:.1f}s; benchmark clock will start at first frontier decision"
        )
        return True

    def robot_position(self):
        position = super().robot_position()
        if position is not None and getattr(self, "capture_policy_robot", False) and getattr(self, "decision_robot_xy", None) is None:
            self.decision_robot_xy = (float(position[0]), float(position[1]))
        return position

    def robot_map_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            q = transform.transform.rotation
            return (
                float(transform.transform.translation.x),
                float(transform.transform.translation.y),
                float(_yaw_from_quaternion(q)),
            )
        except Exception:
            position = self.robot_position()
            if position is None:
                return math.nan, math.nan, math.nan
            return float(position[0]), float(position[1]), math.nan

    def _begin_policy_decision(self, ready_pose):
        self.decision_id += 1
        self.active_decision = self.decision_id
        self.compute_sim_t0 = self.now_s()
        self.compute_t0 = time.perf_counter()
        self.decision_compute_ms = None
        self.pending_decision_selection = None
        self.decision_map_msg = self.map_msg
        self.decision_robot_xy = None
        self.capture_policy_robot = True
        if self.t0 is None:
            self.t0 = self.compute_sim_t0
            self._update_metadata(
                exploration_start_sim_s=self.t0,
                evaluation_start_x=float(ready_pose[0]),
                evaluation_start_y=float(ready_pose[1]),
                runtime_map_resolution_m=float(self.map_msg.info.resolution),
            )

    def exploration_step(self):
        if not self.startup_ready():
            return
        ready = (
            not self.completed and self.map_msg is not None and not self.goal_active
            and self.main_goal is None and not self.revalidation_active
            and self.nav_client.server_is_ready()
        )
        ready_pose = self.robot_position() if ready else None
        if ready and ready_pose is not None:
            self._begin_policy_decision(ready_pose)
        super().exploration_step()
        self.capture_policy_robot = False
        if self.compute_t0 is not None:
            if self.pending_decision_selection is not None:
                candidates, selected = self.pending_decision_selection
                self.record_decision(candidates, selected, "SELECTED")
            else:
                self.record_decision([], None, "NO_SELECTION")

    def publish_goal_markers(self, candidates, selected):
        if getattr(self, "compute_t0", None) is not None:
            self.decision_compute_ms = (time.perf_counter() - self.compute_t0) * 1000.0
        result = super().publish_goal_markers(candidates, selected)
        if getattr(self, "compute_t0", None) is not None:
            self.record_decision(candidates, selected, "SELECTED")
        return result

    def odom_cb(self, msg: Odometry):
        if self.t0 is None or self.finalized:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        if self.last_xy is not None:
            self.distance += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        if self.elapsed() - self.last_traj >= 0.5:
            self.wt.writerow({
                "time_s": self.fmt(self.elapsed()), "x": self.fmt(x), "y": self.fmt(y),
                "yaw": self.fmt(yaw), "cumulative_distance_m": self.fmt(self.distance),
            })
            self.ft.flush()
            self.last_traj = self.elapsed()

    def elapsed(self, now: float | None = None):
        if self.t0 is None:
            return 0.0
        return (self.now_s() if now is None else now) - self.t0

    @staticmethod
    def fmt(value):
        if value is None:
            return ""
        if isinstance(value, float) and math.isnan(value):
            return "nan"
        if isinstance(value, float):
            return f"{value:.9f}"
        return value

    def map_metrics(self):
        if self.map_msg is None:
            return math.nan, math.nan
        canvas = self.fixed_canvas(self.map_msg)
        if canvas is None:
            return math.nan, math.nan
        known = canvas >= 0
        known_fraction = np.count_nonzero(known) / known.size
        coverage = np.count_nonzero(known & self.roi) / self.roi_n if self.roi is not None and self.roi_n > 0 else math.nan
        return float(known_fraction), float(coverage)

    def metric_tick(self):
        if self.t0 is None or self.finalized:
            return
        self.known, self.coverage = self.map_metrics()
        main_attempts = self.count["main_attempts"]
        main_succeeded = self.count["main_succeeded"]
        rate = main_succeeded / main_attempts if main_attempts else math.nan
        blocked_total = self.count["abandoned_206"] + self.count["abandoned_208"]
        self.wm.writerow({
            "time_s": self.fmt(self.elapsed()), "distance_m": self.fmt(self.distance),
            "known_fraction": self.fmt(self.known), "coverage": self.fmt(self.coverage),
            "occupied_iou": "nan", "tu": "nan", "frontiers_selected": self.count["frontiers_selected"],
            "main_attempts": main_attempts, "main_succeeded": main_succeeded,
            "main_failed": self.count["main_failed"], "main_interrupted": self.count["main_interrupted"],
            "abandoned_206": self.count["abandoned_206"], "abandoned_208": self.count["abandoned_208"],
            "planner_blocked_abandoned_total": blocked_total, "main_success_rate": self.fmt(rate),
            "subgoal_attempts": self.count["subgoal_attempts"], "subgoal_succeeded": self.count["subgoal_succeeded"],
            "subgoal_failed": self.count["subgoal_failed"], "subgoal_interrupted": self.count["subgoal_interrupted"],
        })
        self.fm.flush()
        if self.elapsed() - self.last_snap >= 10.0:
            self.record_snapshot("periodic")
            self.last_snap = self.elapsed()

    def fixed_canvas(self, msg):
        data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        canvas = np.full((CANVAS_H, CANVAS_W), -1, dtype=np.int16)
        res = float(msg.info.resolution)
        if abs(res - CANVAS_RES) > 1e-6:
            scale = res / CANVAS_RES
            if abs(scale - round(scale)) > 1e-6:
                return None
            scale = int(round(scale))
            data = np.repeat(np.repeat(data, scale, axis=0), scale, axis=1)
        x0 = int(round((msg.info.origin.position.x - CANVAS_X) / CANVAS_RES))
        y0 = int(round((msg.info.origin.position.y - CANVAS_Y) / CANVAS_RES))
        y1 = y0 + data.shape[0]
        x1 = x0 + data.shape[1]
        cy0, cx0 = max(0, y0), max(0, x0)
        cy1, cx1 = min(CANVAS_H, y1), min(CANVAS_W, x1)
        if cy1 <= cy0 or cx1 <= cx0:
            return None
        sy0, sx0 = cy0 - y0, cx0 - x0
        canvas[cy0:cy1, cx0:cx1] = data[sy0:sy0 + (cy1 - cy0), sx0:sx0 + (cx1 - cx0)]
        return canvas

    def save_canvas(self, path_without_suffix, msg=None):
        msg = self.map_msg if msg is None else msg
        if msg is None:
            return ""
        canvas = self.fixed_canvas(msg)
        if canvas is None:
            return ""
        path = FilePath(path_without_suffix).with_suffix(".npz")
        np.savez_compressed(path, data=canvas, resolution=CANVAS_RES, origin_x=CANVAS_X,
                            origin_y=CANVAS_Y, source_map_stamp_s=_stamp_s(msg), canvas_id=FIXED_CANVAS_ID)
        return str(path.relative_to(self.run))

    def save_map_pair(self, base, msg=None):
        msg = self.map_msg if msg is None else msg
        raw_path = FilePath(base).with_name(FilePath(base).name + "_raw").with_suffix(".npz")
        canvas_path = FilePath(base).with_name(FilePath(base).name + "_canvas").with_suffix(".npz")
        if msg is None:
            return "", ""
        raw = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        np.savez_compressed(
            raw_path, data=raw, resolution=float(msg.info.resolution), width=int(msg.info.width),
            height=int(msg.info.height), origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y), origin_yaw=float(_yaw_from_quaternion(msg.info.origin.orientation)),
            frame_id=str(msg.header.frame_id), source_stamp_s=_stamp_s(msg),
        )
        canvas = self.fixed_canvas(msg)
        if canvas is not None:
            np.savez_compressed(canvas_path, data=canvas, resolution=CANVAS_RES, origin_x=CANVAS_X,
                                origin_y=CANVAS_Y, source_map_stamp_s=_stamp_s(msg), canvas_id=FIXED_CANVAS_ID)
        return str(raw_path.relative_to(self.run)), str(canvas_path.relative_to(self.run))

    def record_snapshot(self, event: str):
        if self.map_msg is None or not hasattr(self, "ws"):
            return
        self.snapshot_id = getattr(self, "snapshot_id", 0) + 1
        self.known, self.coverage = self.map_metrics()
        raw, canvas = self.save_map_pair(self.run / "maps" / f"snapshot_{self.snapshot_id:06d}_{event}")
        rx, ry, ryaw = self.robot_map_pose()
        self.ws.writerow({
            "snapshot_id": self.snapshot_id, "event": event, "time_s": self.fmt(self.elapsed()),
            "coverage": self.fmt(self.coverage), "known_fraction": self.fmt(self.known),
            "distance_m": self.fmt(self.distance), "robot_map_x": self.fmt(rx), "robot_map_y": self.fmt(ry),
            "robot_map_yaw": self.fmt(ryaw), "raw_map_file": raw, "canvas_map_file": canvas,
            "source_map_stamp_s": self.fmt(_stamp_s(self.map_msg)), "canvas_id": FIXED_CANVAS_ID,
        })
        self.fs.flush()

    def _nf_candidate_rows(self, candidates, selected):
        msg = self.decision_map_msg or self.map_msg
        if msg is None:
            return [], False, 0
        grid = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        mask = NearestEuclideanFrontier.frontier_mask(grid)
        regions = NearestEuclideanFrontier.frontier_regions(mask)
        robot_xy = self.decision_robot_xy or self.robot_position()
        if robot_xy is None:
            return [], False, len(regions)
        robot_x, robot_y = robot_xy
        raw_candidates = []
        for region in regions:
            row, col = NearestEuclideanFrontier.representative(region)
            x = msg.info.origin.position.x + (col + 0.5) * msg.info.resolution
            y = msg.info.origin.position.y + (row + 0.5) * msg.info.resolution
            raw_candidates.append({
                "row": int(row), "col": int(col), "x": float(x), "y": float(y),
                "distance_m": float(math.hypot(x - robot_x, y - robot_y)), "region_size": int(len(region)),
            })
        distant = [item for item in raw_candidates if item["distance_m"] >= MIN_DISTANCE_THRESHOLD]
        near_fallback = bool(raw_candidates) and not distant
        distance_eligible = distant if distant else raw_candidates
        eligible_keys = {(item["row"], item["col"]) for item in distance_eligible}
        selectable_keys = {(int(c[3]), int(c[4])) for c in candidates}
        selected_key = None if selected is None else (int(selected[3]), int(selected[4]))
        raw_order = sorted(raw_candidates, key=lambda item: (item["distance_m"], item["row"], item["col"]))
        raw_rank = {(item["row"], item["col"]): rank for rank, item in enumerate(raw_order, 1)}
        policy_order = sorted([item for item in raw_candidates if (item["row"], item["col"]) in selectable_keys],
                              key=lambda item: (item["distance_m"], item["row"], item["col"]))
        policy_rank = {(item["row"], item["col"]): rank for rank, item in enumerate(policy_order, 1)}
        rows = []
        for item in raw_order:
            key = (item["row"], item["col"])
            is_distance_eligible = key in eligible_keys
            execution_suppressed = self.is_execution_blocked_suppressed(item["x"], item["y"]) if is_distance_eligible else False
            planner_suppressed = any(
                math.hypot(item["x"] - px, item["y"] - py) <= PLANNER_BLOCKED_SKIP_RADIUS_M
                for px, py in self.planner_blocked_goals
            ) if is_distance_eligible else False
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
            rows.append({
                "policy_decision_id": self.active_decision, "candidate_id": f"r{item['row']}_c{item['col']}",
                "raw_rank": raw_rank[key], "policy_rank": policy_rank.get(key, ""), "row": item["row"], "col": item["col"],
                "x": self.fmt(item["x"]), "y": self.fmt(item["y"]),
                "distance_cells": self.fmt(item["distance_m"] / float(msg.info.resolution)),
                "distance_m": self.fmt(item["distance_m"]), "region_size": item["region_size"],
                "below_1m": int(item["distance_m"] < MIN_DISTANCE_THRESHOLD), "distance_eligible": int(is_distance_eligible),
                "near_frontier_fallback": int(near_fallback), "execution_suppressed": int(execution_suppressed),
                "planner_suppressed": int(planner_suppressed), "selectable": int(selectable), "status": status,
                "selected": int(is_selected), "execution_result": "",
            })
        return rows, near_fallback, len(regions)

    def record_decision(self, candidates, selected, outcome):
        if self.compute_t0 is None or self.active_decision is None:
            return
        ms = self.decision_compute_ms
        if ms is None:
            ms = (time.perf_counter() - self.compute_t0) * 1000.0
        self.policy_compute_ms.append(float(ms))
        candidate_rows, near_fallback, frontier_region_count = self._nf_candidate_rows(candidates, selected)
        if near_fallback:
            self.near_frontier_fallback_count += 1
        did = int(self.active_decision)
        decision_dir = self.run / "decisions" / f"policy_decision_{did:06d}"
        decision_dir.mkdir(parents=True, exist_ok=True)
        raw, canvas = self.save_map_pair(decision_dir / "observed_map", msg=self.decision_map_msg)
        selected_row = next((row for row in candidate_rows if row["selected"]), None)
        selected_candidate_id = "" if selected_row is None else selected_row["candidate_id"]
        selected_rank = "" if selected_row is None else selected_row["policy_rank"]
        selected_x = math.nan if selected_row is None else float(selected_row["x"])
        selected_y = math.nan if selected_row is None else float(selected_row["y"])
        selected_distance = math.nan if selected_row is None else float(selected_row["distance_m"])
        robot_x = robot_y = math.nan
        if self.decision_robot_xy is not None:
            robot_x, robot_y = self.decision_robot_xy
        _, _, robot_yaw = self.robot_map_pose()
        distance_eligible_count = sum(row["distance_eligible"] for row in candidate_rows)
        selectable_count = sum(row["selectable"] for row in candidate_rows)
        suppressed_count = sum(1 for row in candidate_rows if row["distance_eligible"] and not row["selectable"])
        below_count = sum(row["below_1m"] for row in candidate_rows)
        if selected_row is not None:
            normalized_outcome = "selected"
        elif frontier_region_count == 0:
            normalized_outcome = "exhausted_no_frontier_region"
        elif distance_eligible_count and selectable_count == 0:
            normalized_outcome = "no_normally_selectable_candidate"
        else:
            normalized_outcome = outcome.lower()
        decision_row = {
            "policy_decision_id": did, "sim_time_s": self.fmt(self.elapsed(self.compute_sim_t0)),
            "map_stamp_s": self.fmt(_stamp_s(self.decision_map_msg)), "map_generation": self.map_generation,
            "robot_x": self.fmt(float(robot_x)), "robot_y": self.fmt(float(robot_y)), "robot_yaw": self.fmt(float(robot_yaw)),
            "candidate_compute_ms": self.fmt(float(ms)), "frontier_region_count": frontier_region_count,
            "candidate_count": len(candidate_rows), "distance_eligible_count": distance_eligible_count,
            "selectable_count": selectable_count, "suppressed_count": suppressed_count, "below_1m_count": below_count,
            "preferred_min_distance_m": self.fmt(MIN_DISTANCE_THRESHOLD), "near_frontier_fallback": int(near_fallback),
            "selected_candidate_id": selected_candidate_id, "selected_rank": selected_rank,
            "selected_x": self.fmt(selected_x), "selected_y": self.fmt(selected_y),
            "selected_distance_m": self.fmt(selected_distance), "planner_revalidation_started": int(bool(self.revalidation_active)),
            "outcome": normalized_outcome, "raw_map": raw, "canvas_map": canvas,
        }
        self.wpd.writerow(decision_row)
        self.fpd.flush()
        for row in candidate_rows:
            self.wc.writerow(row)
        self.fc.flush()
        with (decision_dir / "candidates.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=CANDIDATE_FIELDS)
            writer.writeheader(); writer.writerows(candidate_rows)
        (decision_dir / "decision.json").write_text(json.dumps(decision_row, indent=2, allow_nan=True), encoding="utf-8")
        self.compute_t0 = None; self.compute_sim_t0 = None; self.decision_compute_ms = None
        self.pending_decision_selection = None; self.decision_map_msg = None; self.decision_robot_xy = None
        if selected is None:
            self.active_decision = None

    def send_navigation_goal(self, x, y, mode="main"):
        previous_goal_id = self.active_goal
        super().send_navigation_goal(x, y, mode)
        if not self.goal_active or self.current_goal_mode != mode:
            return
        if mode == "main" and previous_goal_id is None:
            self.count["frontiers_selected"] += 1
        self.goal_id += 1
        self.active_goal = self.goal_id
        decision_id = getattr(self, "active_decision", None)
        if not hasattr(self, "goal_decision_ids"):
            self.goal_decision_ids = {}
        self.goal_decision_ids[self.goal_id] = decision_id
        self.wg.writerow({
            "goal_id": self.goal_id, "decision_id": "" if decision_id is None else decision_id, "mode": mode,
            "start_time_s": self.fmt(self.elapsed()), "end_time_s": "", "target_x": self.fmt(x), "target_y": self.fmt(y),
            "result": "", "status": "", "error_code": "", "error_msg": "",
        })
        self.fg.flush()

    def plan_callback(self, msg: NavPath):
        super().plan_callback(msg)
        if self.t0 is None:
            return
        frontier_x = frontier_y = math.nan
        if self.main_goal is not None:
            frontier_x, frontier_y = self.main_goal
        poses = len(msg.poses)
        endpoint_x = endpoint_y = path_length = endpoint_error = math.nan
        usable = 0
        if poses:
            endpoint_x = float(msg.poses[-1].pose.position.x); endpoint_y = float(msg.poses[-1].pose.position.y)
        if poses >= 2:
            path_length = 0.0
            for first, second in zip(msg.poses[:-1], msg.poses[1:]):
                p0, p1 = first.pose.position, second.pose.position
                path_length += math.hypot(p1.x - p0.x, p1.y - p0.y)
        if self.main_goal is not None and poses:
            endpoint_error = math.hypot(endpoint_x - frontier_x, endpoint_y - frontier_y)
            usable = int(endpoint_error <= MAIN_PLAN_ENDPOINT_TOLERANCE_M)
        decision_id = getattr(self, "goal_decision_ids", {}).get(self.active_goal, "") if self.active_goal is not None else ""
        self.wp.writerow({
            "time_s": self.fmt(self.elapsed()), "goal_id": self.active_goal if self.active_goal is not None else "",
            "decision_id": decision_id, "poses": poses, "path_length_m": self.fmt(path_length),
            "endpoint_x": self.fmt(endpoint_x), "endpoint_y": self.fmt(endpoint_y),
            "frontier_x": self.fmt(frontier_x), "frontier_y": self.fmt(frontier_y),
            "endpoint_error_m": self.fmt(endpoint_error), "usable": usable,
        })
        self.fp.flush()

    def goal_response_callback(self, future, mode):
        goal_id = self.active_goal
        accepted = False; response_error = ""
        try:
            goal_handle = future.result(); accepted = bool(goal_handle.accepted)
        except Exception as exc:
            response_error = str(exc)
        super().goal_response_callback(future, mode)
        if not accepted:
            self.finish_goal("rejected", "", "", response_error or "goal_rejected", goal_id=goal_id)

    def goal_result_callback(self, future, mode):
        completed_goal_id = self.active_goal
        code = None; status = None; message = ""
        try:
            wrapped = future.result(); status = wrapped.status; result = wrapped.result
            code = getattr(result, "error_code", None); message = getattr(result, "error_msg", "") or ""
        except Exception as exc:
            message = str(exc)
        super().goal_result_callback(future, mode)
        if mode == "main":
            self.count["main_attempts"] += 1
            if code == GOAL_OCCUPIED_ERROR_CODE: self.count["abandoned_206"] += 1
            if code == NO_VALID_PATH_ERROR_CODE: self.count["abandoned_208"] += 1
            if status == 4: self.count["main_succeeded"] += 1
            else: self.count["main_failed"] += 1
        else:
            self.count["subgoal_attempts"] += 1
            if status == 4: self.count["subgoal_succeeded"] += 1
            else: self.count["subgoal_failed"] += 1
        if code is not None: self.errors[str(code)] += 1
        self.finish_goal("succeeded" if status == 4 else "failed", status, code, message, goal_id=completed_goal_id)
        if self.main_goal is None and not self.goal_active:
            self.active_decision = None

    def finish_goal(self, result, status, code, message, goal_id=None):
        if goal_id is None: goal_id = self.active_goal
        if goal_id is None: return
        decision_id = getattr(self, "goal_decision_ids", {}).get(goal_id, "")
        self.wg.writerow({
            "goal_id": goal_id, "decision_id": decision_id, "mode": "", "start_time_s": "",
            "end_time_s": self.fmt(self.elapsed()), "target_x": "", "target_y": "", "result": result,
            "status": status, "error_code": code, "error_msg": message,
        })
        self.fg.flush(); getattr(self, "goal_decision_ids", {}).pop(goal_id, None)
        if self.active_goal == goal_id: self.active_goal = None

    @staticmethod
    def _mean(values): return statistics.fmean(values) if values else math.nan

    @staticmethod
    def _std(values): return statistics.pstdev(values) if len(values) > 1 else (0.0 if values else math.nan)

    def finalize(self, reason):
        if self.finalized: return
        if self.active_goal is not None: self.finish_goal("interrupted", "", "", "run_interrupted")
        self.finalized = True
        self.known, self.coverage = self.map_metrics()
        self.record_snapshot("final")
        main_attempts = self.count["main_attempts"]; main_succeeded = self.count["main_succeeded"]
        total_time = None if self.t0 is None else self.elapsed()
        blocked_total = self.count["abandoned_206"] + self.count["abandoned_208"]
        decision_count = int(self.decision_id)
        summary = {
            "run_id": self.run.name, "method": "nearest_frontier_euclidean", "environment": self.environment,
            "runtime_profile": self.runtime_profile["id"], "runtime_profile_name": self.runtime_profile_name,
            "evaluation_roi_id": self.environment_profile["roi_id"],
            "evaluation_roi_denominator": self.roi_n if self.roi_n > 0 else None,
            "structural_ground_truth_id": self.environment_profile["ground_truth_id"],
            "final_coverage": self.coverage, "final_known_fraction": self.known, "total_distance_m": self.distance,
            "total_time_s": total_time, "frontiers_selected": self.count["frontiers_selected"],
            "policy_decisions": decision_count, "near_frontier_fallback_count": self.near_frontier_fallback_count,
            "near_frontier_fallback_fraction": self.near_frontier_fallback_count / decision_count if decision_count else math.nan,
            "candidate_compute_ms_mean": self._mean(self.policy_compute_ms), "candidate_compute_ms_std": self._std(self.policy_compute_ms),
            "main_attempts": main_attempts, "main_succeeded": main_succeeded, "main_failed": self.count["main_failed"],
            "main_interrupted": self.count["main_interrupted"], "abandoned_206": self.count["abandoned_206"],
            "abandoned_208": self.count["abandoned_208"], "planner_blocked_abandoned_total": blocked_total,
            "main_success_rate": main_succeeded / main_attempts if main_attempts else math.nan,
            "subgoal_attempts": self.count["subgoal_attempts"], "subgoal_succeeded": self.count["subgoal_succeeded"],
            "subgoal_failed": self.count["subgoal_failed"], "subgoal_interrupted": self.count["subgoal_interrupted"],
            "error_code_counts": dict(self.errors), "termination_reason": reason,
            "nav2_startup_stable_s": NAV2_READY_STABLE_S, "occupied_iou_online": None, "tu_online": None,
        }
        (self.run / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=True), encoding="utf-8")
        self._update_metadata(
            termination_reason=reason, final_coverage=self.coverage, final_known_fraction=self.known,
            total_distance_m=self.distance, total_time_s=total_time, policy_decisions=decision_count,
            near_frontier_fallback_count=self.near_frontier_fallback_count, planner_blocked_abandoned_total=blocked_total,
        )
        self.get_logger().warn(
            f"NEAREST SAVED: env={self.environment}, profile={self.runtime_profile_name}, coverage={self.fmt(self.coverage)}, distance={self.distance:.2f}m, output={self.run}"
        )

    def close_recorder_files(self):
        for name in ("fm", "ft", "fpd", "fc", "fs", "fg", "fp"):
            stream = getattr(self, name, None)
            if stream is not None and not stream.closed:
                stream.flush(); stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--environment", choices=sorted(ENVIRONMENT_PROFILES), default="new_room",
                        help="Evaluation environment/profile. Default is new_room; use --environment hospital for Hospital.")
    parser.add_argument("--runtime-profile", choices=sorted(RUNTIME_PROFILES), default=None,
                        help="Runtime provenance profile. New Room defaults to submap.")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Stage2Run(args.run_id, args.odom_topic, args.environment, args.runtime_profile)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.finalized: node.finalize("keyboard_interrupt")
        node.close_recorder_files(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == "__main__":
    main()
