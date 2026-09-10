#!/usr/bin/env python3
"""Run canonical nf_basic.py with an integrated Nearest-Frontier recorder.

The exploration policy remains in nf_basic.py and is inherited unchanged from
nf_basic.NearestEuclideanFrontier. This wrapper only adds benchmark setup,
measurement, provenance, map snapshots, and result logging.

New Room is the default environment. The default runtime profile is ``submap``
so runs launched with launch/submap.launch.py record the active SLAM/frontend
files accurately. Use --runtime-profile stock for stock SLAM runs and
--environment hospital for the legacy Hospital benchmark.
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
    NO_VALID_PATH_ERROR_CODE,
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
        self.roi_n = (
            int(np.count_nonzero(self.roi)) if self.roi is not None else 0
        )

        run = root / "experiments" / "nearest" / run_id
        if run.exists():
            raise RuntimeError(f"Run exists: {run}")

        super().__init__()

        self.runtime_profile_name = runtime_profile
        self.runtime_profile = profile
        self.run = run
        (self.run / "maps").mkdir(parents=True)

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
        self.count = Counter()
        self.errors = Counter()
        self.known = math.nan
        self.coverage = math.nan
        self.finalized = False
        self.nav_ready_since = None
        self.startup_gate_open = False
        self.startup_wait_logged = False

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
        import csv

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

        metadata = {
            "run_id": run_id,
            "method": "nearest_frontier_euclidean",
            "environment": self.environment,
            "recorder": "nf_run.py",
            "canonical_policy": "nf_basic.py",
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
            "exploration_start_sim_s": None,
            "exploration_start_source": "first_policy_decision_before_compute",
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
        self.fm, self.wm = self._open(
            "metrics.csv",
            [
                "time_s",
                "distance_m",
                "known_fraction",
                "coverage",
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
        self.fg, self.wg = self._open(
            "goals.csv",
            [
                "goal_id",
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

    def startup_ready(self):
        if self.startup_gate_open:
            return True
        if not self.nav_client.server_is_ready():
            self.nav_ready_since = None
            if not self.startup_wait_logged:
                self.get_logger().info(
                    "Nearest run waiting for NavigateToPose action server "
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
            f"NEAREST READY: Nav2 stable for >= {NAV2_READY_STABLE_S:.1f}s; "
            "benchmark clock will start at first frontier decision"
        )
        return True

    def exploration_step(self):
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
        if ready and self.t0 is None:
            robot_pose = self.robot_position()
            if robot_pose is not None:
                self.t0 = self.now_s()
                self._update_metadata(
                    exploration_start_sim_s=self.t0,
                    evaluation_start_x=float(robot_pose[0]),
                    evaluation_start_y=float(robot_pose[1]),
                    runtime_map_resolution_m=float(self.map_msg.info.resolution),
                )
        super().exploration_step()

    def odom_cb(self, msg: Odometry):
        if self.t0 is None or self.finalized:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if self.last_xy is not None:
            self.distance += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        if self.elapsed() - self.last_traj >= 0.5:
            self.wt.writerow(
                {
                    "time_s": self.fmt(self.elapsed()),
                    "x": self.fmt(x),
                    "y": self.fmt(y),
                    "yaw": self.fmt(yaw),
                    "cumulative_distance_m": self.fmt(self.distance),
                }
            )
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
        coverage = (
            np.count_nonzero(known & self.roi) / self.roi_n
            if self.roi is not None and self.roi_n > 0
            else math.nan
        )
        return float(known_fraction), float(coverage)

    def metric_tick(self):
        if self.t0 is None or self.finalized:
            return
        self.known, self.coverage = self.map_metrics()
        main_attempts = self.count["main_attempts"]
        main_succeeded = self.count["main_succeeded"]
        rate = main_succeeded / main_attempts if main_attempts else math.nan
        blocked_total = self.count["abandoned_206"] + self.count["abandoned_208"]

        self.wm.writerow(
            {
                "time_s": self.fmt(self.elapsed()),
                "distance_m": self.fmt(self.distance),
                "known_fraction": self.fmt(self.known),
                "coverage": self.fmt(self.coverage),
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
            res = CANVAS_RES

        x0 = int(round((msg.info.origin.position.x - CANVAS_X) / CANVAS_RES))
        y0 = int(round((msg.info.origin.position.y - CANVAS_Y) / CANVAS_RES))
        y1 = y0 + data.shape[0]
        x1 = x0 + data.shape[1]
        cy0 = max(0, y0)
        cx0 = max(0, x0)
        cy1 = min(CANVAS_H, y1)
        cx1 = min(CANVAS_W, x1)
        if cy1 <= cy0 or cx1 <= cx0:
            return None
        sy0 = cy0 - y0
        sx0 = cx0 - x0
        canvas[cy0:cy1, cx0:cx1] = data[
            sy0 : sy0 + (cy1 - cy0),
            sx0 : sx0 + (cx1 - cx0),
        ]
        return canvas

    def save_canvas(self, path_without_suffix):
        if self.map_msg is None:
            return ""
        canvas = self.fixed_canvas(self.map_msg)
        if canvas is None:
            return ""
        path = FilePath(path_without_suffix).with_suffix(".npz")
        np.savez_compressed(
            path,
            data=canvas,
            resolution=CANVAS_RES,
            origin_x=CANVAS_X,
            origin_y=CANVAS_Y,
        )
        return str(path.relative_to(self.run))

    def save_map_pair(self, base):
        raw_path = FilePath(base).with_name(FilePath(base).name + "_raw").with_suffix(
            ".npz"
        )
        canvas_path = FilePath(base).with_name(
            FilePath(base).name + "_canvas"
        ).with_suffix(".npz")
        if self.map_msg is None:
            return "", ""
        raw = np.array(self.map_msg.data, dtype=np.int16).reshape(
            self.map_msg.info.height, self.map_msg.info.width
        )
        np.savez_compressed(
            raw_path,
            data=raw,
            resolution=float(self.map_msg.info.resolution),
            origin_x=float(self.map_msg.info.origin.position.x),
            origin_y=float(self.map_msg.info.origin.position.y),
        )
        canvas = self.fixed_canvas(self.map_msg)
        if canvas is not None:
            np.savez_compressed(
                canvas_path,
                data=canvas,
                resolution=CANVAS_RES,
                origin_x=CANVAS_X,
                origin_y=CANVAS_Y,
            )
        return str(raw_path.relative_to(self.run)), str(canvas_path.relative_to(self.run))

    def send_navigation_goal(self, x, y, mode="main"):
        previous_goal_id = self.active_goal
        super().send_navigation_goal(x, y, mode)
        if not self.goal_active or self.current_goal_mode != mode:
            return
        if mode == "main" and previous_goal_id is None:
            self.count["frontiers_selected"] += 1
        self.goal_id += 1
        self.active_goal = self.goal_id
        self.wg.writerow(
            {
                "goal_id": self.goal_id,
                "mode": mode,
                "start_time_s": self.fmt(self.elapsed()),
                "end_time_s": "",
                "target_x": self.fmt(x),
                "target_y": self.fmt(y),
                "result": "",
                "status": "",
                "error_code": "",
                "error_msg": "",
            }
        )
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
            endpoint_x = float(msg.poses[-1].pose.position.x)
            endpoint_y = float(msg.poses[-1].pose.position.y)
        if poses >= 2:
            path_length = 0.0
            for first, second in zip(msg.poses[:-1], msg.poses[1:]):
                p0 = first.pose.position
                p1 = second.pose.position
                path_length += math.hypot(p1.x - p0.x, p1.y - p0.y)
        if self.main_goal is not None and poses:
            endpoint_error = math.hypot(
                endpoint_x - frontier_x,
                endpoint_y - frontier_y,
            )
            usable = int(endpoint_error <= MAIN_PLAN_ENDPOINT_TOLERANCE_M)
        self.wp.writerow(
            {
                "time_s": self.fmt(self.elapsed()),
                "goal_id": self.active_goal if self.active_goal is not None else "",
                "poses": poses,
                "path_length_m": self.fmt(path_length),
                "endpoint_x": self.fmt(endpoint_x),
                "endpoint_y": self.fmt(endpoint_y),
                "frontier_x": self.fmt(frontier_x),
                "frontier_y": self.fmt(frontier_y),
                "endpoint_error_m": self.fmt(endpoint_error),
                "usable": usable,
            }
        )
        self.fp.flush()

    def goal_response_callback(self, future, mode):
        goal_id = self.active_goal
        accepted = False
        response_error = ""
        try:
            goal_handle = future.result()
            accepted = bool(goal_handle.accepted)
        except Exception as exc:
            response_error = str(exc)

        super().goal_response_callback(future, mode)

        if not accepted:
            self.finish_goal(
                "rejected",
                "",
                "",
                response_error or "goal_rejected",
                goal_id=goal_id,
            )

    def goal_result_callback(self, future, mode):
        completed_goal_id = self.active_goal
        code = None
        status = None
        message = ""
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
            code = getattr(result, "error_code", None)
            message = getattr(result, "error_msg", "") or ""
        except Exception as exc:
            message = str(exc)

        super().goal_result_callback(future, mode)

        if mode == "main":
            self.count["main_attempts"] += 1
            if code == GOAL_OCCUPIED_ERROR_CODE:
                self.count["abandoned_206"] += 1
            if code == NO_VALID_PATH_ERROR_CODE:
                self.count["abandoned_208"] += 1
            if status == 4:
                self.count["main_succeeded"] += 1
            else:
                self.count["main_failed"] += 1
        else:
            self.count["subgoal_attempts"] += 1
            if status == 4:
                self.count["subgoal_succeeded"] += 1
            else:
                self.count["subgoal_failed"] += 1
        if code is not None:
            self.errors[str(code)] += 1

        self.finish_goal(
            "succeeded" if status == 4 else "failed",
            status,
            code,
            message,
            goal_id=completed_goal_id,
        )

    def finish_goal(self, result, status, code, message, goal_id=None):
        if goal_id is None:
            goal_id = self.active_goal
        if goal_id is None:
            return
        self.wg.writerow(
            {
                "goal_id": goal_id,
                "mode": "",
                "start_time_s": "",
                "end_time_s": self.fmt(self.elapsed()),
                "target_x": "",
                "target_y": "",
                "result": result,
                "status": status,
                "error_code": code,
                "error_msg": message,
            }
        )
        self.fg.flush()
        if self.active_goal == goal_id:
            self.active_goal = None

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
            "method": "nearest_frontier_euclidean",
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
            "termination_reason": reason,
            "nav2_startup_stable_s": NAV2_READY_STABLE_S,
        }
        (self.run / "summary.json").write_text(
            json.dumps(summary, indent=2, allow_nan=True), encoding="utf-8"
        )

        self._update_metadata(
            termination_reason=reason,
            final_coverage=self.coverage,
            final_known_fraction=self.known,
            total_distance_m=self.distance,
            total_time_s=total_time,
            planner_blocked_abandoned_total=blocked_total,
        )
        self.get_logger().warn(
            f"NEAREST SAVED: env={self.environment}, "
            f"profile={self.runtime_profile_name}, "
            f"coverage={self.fmt(self.coverage)}, "
            f"distance={self.distance:.2f}m, output={self.run}"
        )


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
            "use stock for launch/stock.launch.py. Hospital defaults to hospital."
        ),
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Stage2Run(
        args.run_id,
        args.odom_topic,
        args.environment,
        args.runtime_profile,
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.finalized:
            node.finalize("keyboard_interrupt")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
