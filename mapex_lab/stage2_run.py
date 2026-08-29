#!/usr/bin/env python3
"""Run canonical nf_basic.py with one integrated Stage-2 recorder.

The exploration policy is inherited unchanged from nf_basic.NearestEuclideanFrontier.
This wrapper only adds measurement/logging around the existing callbacks.
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

from nf_basic import MAIN_PLAN_ENDPOINT_TOLERANCE_M, NearestEuclideanFrontier

CANVAS_RES = 0.05
CANVAS_W = 1504
CANVAS_H = 2123
CANVAS_X = -25.6
CANVAS_Y = -60.1
ROI_N = 215435
NAV2_READY_STABLE_S = 3.0

PROTOCOL_VERSION = "hospital_v2"
FIXED_CANVAS_ID = "hospital_canvas_v1"
EVALUATION_ROI_ID = "hospital_connected_free_v1"
SIM_SEED_POLICY = "intentionally_uncontrolled_gazebo_default_multiple_run_statistics"


def _deep_merge(base: dict, override: dict) -> dict:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _sha256_file(path: FilePath) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _git_value(repo_root: FilePath, *args: str) -> str | None:
    try:
        proc = subprocess.run(
            ["git", "-C", str(repo_root), *args],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return proc.stdout.strip()


class Stage2Run(NearestEuclideanFrontier):
    def __init__(self, run_id: str, odom_topic: str):
        super().__init__()
        self.root = FilePath(__file__).resolve().parent
        self.repo_root = self.root.parent
        self.run = self.root / "experiments" / "nearest" / run_id
        if self.run.exists():
            raise RuntimeError(f"Run exists: {self.run}")
        (self.run / "maps").mkdir(parents=True)
        (self.run / "decision_maps").mkdir()

        self.metadata = self._write_initial_provenance(run_id)

        roi = self.root / "ground_truth" / "hospital" / "generated" / "hospital_connected_free_v1.npy"
        self.roi = np.load(roi).astype(bool) if roi.exists() else None
        if self.roi is None:
            self.get_logger().warn("ROI missing: coverage will be NaN")

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

        q = QoSProfile(depth=100)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, odom_topic, self.odom_cb, q)
        self._open_files()
        self.create_timer(1.0, self.metric_tick)
        self.get_logger().warn(f"STAGE2 RECORDING: {self.run}")
        self.get_logger().info(
            "Stage-2 provenance saved: metadata.json + runtime_nav2_merged.yaml"
        )

    # ----- provenance -----
    def _write_initial_provenance(self, run_id: str) -> dict:
        tb4_nav_pkg = FilePath(get_package_share_directory("turtlebot4_navigation"))
        nav2_base = tb4_nav_pkg / "config" / "nav2.yaml"
        nav2_override = self.root / "config" / "nav2_stock_xy_only.yaml"

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

        hash_paths = {
            "stage2_run": self.root / "stage2_run.py",
            "nf_basic": self.root / "nf_basic.py",
            "nav2_override": nav2_override,
            "slam": self.root / "config" / "slam.yaml",
            "slam_local_window": self.root / "config" / "slam_local_window.yaml",
            "installed_nav2_base_params": nav2_base,
            "runtime_nav2_merged": merged_path,
        }
        config_sha256 = {
            name: _sha256_file(path)
            for name, path in hash_paths.items()
            if path.is_file()
        }

        metadata = {
            "run_id": run_id,
            "method": "nearest",
            "recorder": "stage2_run.py",
            "git_commit": git_commit,
            "git_dirty_at_recorder_start": None if git_status is None else bool(git_status),
            "protocol_version": PROTOCOL_VERSION,
            "runtime_profile": "not_auto_detected",
            "runtime_profile_note": (
                "Recorder archives the same TurtleBot4 stock Nav2 + "
                "mapex_lab/config/nav2_stock_xy_only.yaml merge used by the current "
                "stock/local debug launchers; the launch file itself is not auto-detected."
            ),
            "runtime_map_resolution_m": None,
            "fixed_canvas_id": FIXED_CANVAS_ID,
            "fixed_canvas_resolution_m": CANVAS_RES,
            "evaluation_roi_id": EVALUATION_ROI_ID,
            "evaluation_roi_denominator": ROI_N,
            "exploration_start_sim_s": None,
            "exploration_start_source": "first_policy_decision_before_compute",
            "execution_goal_semantics": "exact_frontier_center_xy",
            "goal_yaw_semantics": "ignored_by_hospital_goal_checker",
            "planner_path_semantics": "reachability_evidence_only",
            "selected_path_length_semantics": "planner_validation_path_length_not_executed_trajectory",
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

    def _write_metadata(self, metadata: dict | None = None):
        payload = self.metadata if metadata is None else metadata
        (self.run / "metadata.json").write_text(
            json.dumps(payload, indent=2, allow_nan=True),
            encoding="utf-8",
        )

    def _update_metadata(self, **updates):
        self.metadata.update(updates)
        self._write_metadata()

    # ----- csv setup -----
    def _open(self, name, cols):
        f = (self.run / name).open("w", newline="", encoding="utf-8")
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        f.flush()
        return f, w

    def _open_files(self):
        self.fm, self.wm = self._open(
            "metrics.csv",
            [
                "time_s", "distance_m", "known_fraction", "coverage", "occupied_iou", "tu",
                "frontiers_selected", "main_attempts", "main_succeeded", "main_failed",
                "main_interrupted", "abandoned_208", "main_success_rate", "subgoal_attempts",
                "subgoal_succeeded", "subgoal_failed", "subgoal_interrupted",
            ],
        )
        self.ft, self.wt = self._open(
            "trajectory.csv", ["time_s", "x", "y", "yaw", "cumulative_distance_m"]
        )
        self.fd, self.wd = self._open(
            "decisions.csv",
            [
                "decision_id", "time_s", "map_generation", "candidate_count", "computation_ms",
                "selected_x", "selected_y", "selected_distance_m", "region_size", "outcome",
                "raw_map", "canvas_map",
            ],
        )
        self.fc, self.wc = self._open(
            "candidates.csv",
            [
                "decision_id", "rank", "row", "col", "x", "y", "distance_m",
                "region_size", "below_1m", "selected",
            ],
        )
        self.fg, self.wg = self._open(
            "goals.csv",
            [
                "goal_id", "decision_id", "mode", "start_time_s", "end_time_s", "target_x",
                "target_y", "result", "status", "error_code", "error_msg",
            ],
        )
        self.fp, self.wp = self._open(
            "plans.csv",
            [
                "time_s", "goal_id", "decision_id", "poses", "path_length_m", "endpoint_x",
                "endpoint_y", "frontier_x", "frontier_y", "endpoint_error_m", "usable",
            ],
        )

    def elapsed(self, t=None):
        if self.t0 is None:
            return 0.0
        return max(0.0, (self.now_s() if t is None else t) - self.t0)

    @staticmethod
    def yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    @staticmethod
    def fmt(x):
        try:
            x = float(x)
        except Exception:
            return ""
        return f"{x:.9f}" if math.isfinite(x) else "nan"

    # ----- candidate computation -----
    def startup_ready(self):
        if self.startup_gate_open:
            return True
        if not self.nav_client.server_is_ready():
            self.nav_ready_since = None
            if not self.startup_wait_logged:
                self.get_logger().info(
                    "Stage-2 waiting for NavigateToPose action server before benchmark start"
                )
                self.startup_wait_logged = True
            return False
        now = self.now_s()
        if self.nav_ready_since is None:
            self.nav_ready_since = now
            return False
        if now - self.nav_ready_since < NAV2_READY_STABLE_S:
            return False
        if self.map_msg is None:
            return False
        if self.robot_position() is None:
            return False
        self.startup_gate_open = True
        self.get_logger().warn(
            f"STAGE2 READY: Nav2 stable for >= {NAV2_READY_STABLE_S:.1f}s; "
            "benchmark clock will start at first frontier decision"
        )
        return True

    def exploration_step(self):
        if not self.startup_ready():
            return
        # Do not let a transient readiness drop contaminate t=0 before the first
        # actual benchmark decision. After t=0, nf_basic owns normal runtime behavior.
        if self.t0 is None and not self.nav_client.server_is_ready():
            return
        ready = (
            not self.completed
            and self.map_msg is not None
            and not self.goal_active
            and self.main_goal is None
            and self.nav_client.server_is_ready()
        )
        if ready and self.robot_position() is not None:
            self.decision_id += 1
            self.active_decision = self.decision_id
            self.compute_sim_t0 = self.now_s()
            self.compute_t0 = time.perf_counter()
            if self.t0 is None:
                self.t0 = self.compute_sim_t0
                self._update_metadata(
                    exploration_start_sim_s=self.t0,
                    runtime_map_resolution_m=(
                        None if self.map_msg is None else float(self.map_msg.info.resolution)
                    ),
                )
        super().exploration_step()
        if self.compute_t0 is not None:
            self.record_decision([], None, "NO_SELECTION")

    def publish_goal_markers(self, candidates, selected):
        if self.compute_t0 is not None:
            self.record_decision(candidates, selected, "SELECTED")
        super().publish_goal_markers(candidates, selected)

    def record_decision(self, candidates, selected, outcome):
        ms = (time.perf_counter() - self.compute_t0) * 1000
        self.comp.append(ms)
        did = self.active_decision
        raw, canvas = self.save_map_pair(self.run / "decision_maps" / f"decision_{did:06d}")
        if selected is None:
            sx = sy = sd = rs = math.nan
        else:
            sd, sx, sy, _, _, rs = selected
        self.wd.writerow(
            {
                "decision_id": did,
                "time_s": self.fmt(self.elapsed(self.compute_sim_t0)),
                "map_generation": self.map_generation,
                "candidate_count": len(candidates),
                "computation_ms": self.fmt(ms),
                "selected_x": self.fmt(sx),
                "selected_y": self.fmt(sy),
                "selected_distance_m": self.fmt(sd),
                "region_size": self.fmt(rs),
                "outcome": outcome,
                "raw_map": raw,
                "canvas_map": canvas,
            }
        )
        self.fd.flush()
        if selected is not None:
            self.count["frontiers_selected"] += 1
        for rank, c in enumerate(sorted(candidates, key=lambda z: z[0]), 1):
            d, x, y, r, col, n = c
            self.wc.writerow(
                {
                    "decision_id": did,
                    "rank": rank,
                    "row": r,
                    "col": col,
                    "x": self.fmt(x),
                    "y": self.fmt(y),
                    "distance_m": self.fmt(d),
                    "region_size": n,
                    "below_1m": int(d < 1.0),
                    "selected": int(selected is not None and c == selected),
                }
            )
        self.fc.flush()
        self.compute_t0 = None
        self.compute_sim_t0 = None
        if selected is None:
            self.active_decision = None

    # ----- plans / goals -----
    def plan_callback(self, msg: NavPath):
        mode = self.current_goal_mode
        frontier = self.main_goal
        gid = self.active_goal["goal_id"] if self.active_goal else ""
        super().plan_callback(msg)
        if mode != "main" or frontier is None:
            return
        length = 0.0
        for i in range(1, len(msg.poses)):
            a = msg.poses[i - 1].pose.position
            b = msg.poses[i].pose.position
            length += math.hypot(b.x - a.x, b.y - a.y)
        if msg.poses:
            endpoint = msg.poses[-1].pose.position
            ex, ey = endpoint.x, endpoint.y
            err = math.hypot(ex - frontier[0], ey - frontier[1])
        else:
            ex = ey = err = math.nan
        usable = (
            len(msg.poses) >= 2
            and math.isfinite(err)
            and err <= MAIN_PLAN_ENDPOINT_TOLERANCE_M
        )
        self.wp.writerow(
            {
                "time_s": self.fmt(self.elapsed()),
                "goal_id": gid,
                "decision_id": self.active_decision or "",
                "poses": len(msg.poses),
                "path_length_m": self.fmt(length),
                "endpoint_x": self.fmt(ex),
                "endpoint_y": self.fmt(ey),
                "frontier_x": self.fmt(frontier[0]),
                "frontier_y": self.fmt(frontier[1]),
                "endpoint_error_m": self.fmt(err),
                "usable": int(usable),
            }
        )
        self.fp.flush()

    def send_navigation_goal(self, x, y, mode):
        super().send_navigation_goal(x, y, mode)
        if self.goal_active and self.current_goal_mode == mode:
            self.goal_id += 1
            self.active_goal = {
                "goal_id": self.goal_id,
                "decision_id": self.active_decision,
                "mode": mode,
                "start": self.elapsed(),
                "x": x,
                "y": y,
            }
            self.count[f"{mode}_attempts"] += 1

    def goal_response_callback(self, future, mode):
        try:
            handle = future.result()
        except Exception as exc:
            self.finish_goal("request_error", "", "", str(exc))
            return super().goal_response_callback(future, mode)
        if not handle.accepted:
            self.finish_goal("rejected", "", "", "goal_rejected")
        super().goal_response_callback(future, mode)

    def goal_result_callback(self, future, mode):
        try:
            wrapped = future.result()
            status = wrapped.status
            result = wrapped.result
            code = getattr(result, "error_code", "")
            msg = getattr(result, "error_msg", "")
        except Exception as exc:
            self.finish_goal("result_error", "", "", str(exc))
            return super().goal_result_callback(future, mode)
        self.finish_goal("succeeded" if status == 4 else "failed", status, code, msg)
        if mode == "main" and status == 4:
            self.active_decision = None
        if mode == "main" and code == 208:
            self.count["abandoned_208"] += 1
            self.active_decision = None
        super().goal_result_callback(future, mode)

    def finish_goal(self, result, status, code, msg):
        goal = self.active_goal
        if goal is None:
            return
        mode = goal["mode"]
        if result == "succeeded":
            self.count[f"{mode}_succeeded"] += 1
        elif result == "interrupted":
            self.count[f"{mode}_interrupted"] += 1
        else:
            self.count[f"{mode}_failed"] += 1
        if code not in ("", None, 0):
            self.errors[str(code)] += 1
        self.wg.writerow(
            {
                "goal_id": goal["goal_id"],
                "decision_id": goal["decision_id"] or "",
                "mode": mode,
                "start_time_s": self.fmt(goal["start"]),
                "end_time_s": self.fmt(self.elapsed()),
                "target_x": self.fmt(goal["x"]),
                "target_y": self.fmt(goal["y"]),
                "result": result,
                "status": status,
                "error_code": code,
                "error_msg": msg,
            }
        )
        self.fg.flush()
        self.active_goal = None

    # ----- odometry + map metrics -----
    def odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.yaw(msg.pose.pose.orientation)
        if self.t0 is None:
            return
        if self.last_xy is not None:
            step = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.distance += step if step < 2.0 else 0.0
        self.last_xy = (x, y)
        t = self.elapsed()
        if t - self.last_traj >= 0.2:
            self.wt.writerow(
                {
                    "time_s": self.fmt(t),
                    "x": self.fmt(x),
                    "y": self.fmt(y),
                    "yaw": self.fmt(yaw),
                    "cumulative_distance_m": self.fmt(self.distance),
                }
            )
            self.ft.flush()
            self.last_traj = t

    def metric_tick(self):
        if self.t0 is None or self.finalized:
            return
        self.known, self.coverage = self.map_metrics()
        ma = self.count["main_attempts"]
        ms = self.count["main_succeeded"]
        rate = ms / ma if ma else math.nan
        self.wm.writerow(
            {
                "time_s": self.fmt(self.elapsed()),
                "distance_m": self.fmt(self.distance),
                "known_fraction": self.fmt(self.known),
                "coverage": self.fmt(self.coverage),
                "occupied_iou": "nan",
                "tu": "nan",
                "frontiers_selected": self.count["frontiers_selected"],
                "main_attempts": ma,
                "main_succeeded": ms,
                "main_failed": self.count["main_failed"],
                "main_interrupted": self.count["main_interrupted"],
                "abandoned_208": self.count["abandoned_208"],
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
        ratio = round(msg.info.resolution / CANVAS_RES)
        if ratio < 1 or abs(msg.info.resolution / CANVAS_RES - ratio) > 1e-6:
            return None
        raw = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        if ratio > 1:
            raw = np.repeat(np.repeat(raw, ratio, 0), ratio, 1)
        out = np.full((CANVAS_H, CANVAS_W), -1, dtype=np.int16)
        c0 = round((msg.info.origin.position.x - CANVAS_X) / CANVAS_RES)
        r0 = round((msg.info.origin.position.y - CANVAS_Y) / CANVAS_RES)
        sr = max(0, -r0)
        sc = max(0, -c0)
        dr = max(0, r0)
        dc = max(0, c0)
        nr = min(raw.shape[0] - sr, CANVAS_H - dr)
        nc = min(raw.shape[1] - sc, CANVAS_W - dc)
        if nr > 0 and nc > 0:
            out[dr : dr + nr, dc : dc + nc] = raw[sr : sr + nr, sc : sc + nc]
        return out

    def map_metrics(self):
        if self.map_msg is None:
            return math.nan, math.nan
        canvas = self.fixed_canvas(self.map_msg)
        if canvas is None:
            return math.nan, math.nan
        known = canvas >= 0
        known_fraction = np.count_nonzero(known) / known.size
        coverage = (
            np.count_nonzero(known & self.roi) / ROI_N
            if self.roi is not None
            else math.nan
        )
        return float(known_fraction), float(coverage)

    def save_canvas(self, prefix: FilePath):
        if self.map_msg is None:
            return ""
        canvas = self.fixed_canvas(self.map_msg)
        if canvas is None:
            return ""
        cp = prefix.with_name(prefix.name + "_canvas.npz")
        np.savez_compressed(
            cp,
            data=canvas,
            resolution=CANVAS_RES,
            width=CANVAS_W,
            height=CANVAS_H,
            origin_x=CANVAS_X,
            origin_y=CANVAS_Y,
        )
        return str(cp.relative_to(self.run))

    def save_map_pair(self, prefix: FilePath):
        if self.map_msg is None:
            return "", ""
        msg = self.map_msg
        raw = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        canvas = self.fixed_canvas(msg)
        rp = prefix.with_name(prefix.name + "_raw.npz")
        cp = prefix.with_name(prefix.name + "_canvas.npz")
        np.savez_compressed(
            rp,
            data=raw,
            resolution=msg.info.resolution,
            width=msg.info.width,
            height=msg.info.height,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )
        if canvas is not None:
            np.savez_compressed(
                cp,
                data=canvas,
                resolution=CANVAS_RES,
                width=CANVAS_W,
                height=CANVAS_H,
                origin_x=CANVAS_X,
                origin_y=CANVAS_Y,
            )
        return (
            str(rp.relative_to(self.run)),
            str(cp.relative_to(self.run)) if canvas is not None else "",
        )

    # ----- finish -----
    def mark_exploration_complete(self):
        if self.completed:
            return
        super().mark_exploration_complete()
        self.finalize(self.completion_reason or "complete")
        if rclpy.ok():
            rclpy.shutdown()

    def finalize(self, reason):
        if self.finalized:
            return
        if self.active_goal is not None:
            self.finish_goal("interrupted", "", "", "run_interrupted")
        self.finalized = True
        self.known, self.coverage = self.map_metrics()
        self.save_map_pair(self.run / "maps" / "final")
        ma = self.count["main_attempts"]
        ms = self.count["main_succeeded"]
        total_time = None if self.t0 is None else self.elapsed()
        summary = {
            "run_id": self.run.name,
            "final_coverage": self.coverage,
            "final_known_fraction": self.known,
            "total_distance_m": self.distance,
            "total_time_s": total_time,
            "frontiers_selected": self.count["frontiers_selected"],
            "main_attempts": ma,
            "main_succeeded": ms,
            "main_failed": self.count["main_failed"],
            "main_interrupted": self.count["main_interrupted"],
            "abandoned_208": self.count["abandoned_208"],
            "main_success_rate": ms / ma if ma else math.nan,
            "subgoal_attempts": self.count["subgoal_attempts"],
            "subgoal_succeeded": self.count["subgoal_succeeded"],
            "subgoal_failed": self.count["subgoal_failed"],
            "subgoal_interrupted": self.count["subgoal_interrupted"],
            "error_code_counts": dict(self.errors),
            "decision_computation_ms_mean": statistics.fmean(self.comp) if self.comp else math.nan,
            "decision_computation_ms_std": statistics.pstdev(self.comp) if len(self.comp) > 1 else 0.0,
            "termination_reason": reason,
            "nav2_startup_stable_s": NAV2_READY_STABLE_S,
            "occupied_iou_online": None,
            "tu_online": None,
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
        )
        self.get_logger().warn(
            f"STAGE2 SAVED: coverage={self.fmt(self.coverage)}, "
            f"distance={self.distance:.2f}m, output={self.run}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--odom-topic", default="/odom")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Stage2Run(args.run_id, args.odom_topic)
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
