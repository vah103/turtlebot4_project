#!/usr/bin/env python3
"""Record Hospital exploration runs on the frozen research canvas/ROI."""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Bool, Int32
from tf2_ros import Buffer, TransformException, TransformListener


WORKSPACE = Path(__file__).resolve().parents[1]
REPO_ROOT = WORKSPACE.parent
EXPERIMENTS_ROOT = WORKSPACE / "experiments"
ROI_MASK_PATH = (
    WORKSPACE
    / "ground_truth"
    / "hospital"
    / "generated"
    / "hospital_connected_free_v1.npy"
)

PROTOCOL_VERSION = "hospital_v1"
CANVAS_ID = "hospital_canvas_v1"
ROI_ID = "hospital_connected_free_v1"
CANVAS_RESOLUTION = 0.05
CANVAS_WIDTH = 1504
CANVAS_HEIGHT = 2123
CANVAS_ORIGIN_X = -25.6
CANVAS_ORIGIN_Y = -60.1
ROI_DENOMINATOR = 215435
ROI_SHA256 = "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"
MAPEX_SOURCE_REPOSITORY = "castacks/MapEx"
MAPEX_SOURCE_COMMIT = "53636bd1c79153acc3c74a532837d78c926bae5e"


def git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=REPO_ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:  # noqa: BLE001
        return None


def path_length(path: NavPath) -> float:
    total = 0.0
    for first, second in zip(path.poses, path.poses[1:]):
        total += math.hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y,
        )
    return total


class ResearchRecorder(Node):
    def __init__(self, method: str, run_id: str) -> None:
        super().__init__(
            "mapex_hospital_research_recorder",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.method = method
        self.run_id = run_id
        self.run_dir = EXPERIMENTS_ROOT / method / run_id
        if self.run_dir.exists():
            raise RuntimeError(f"Run already exists; refusing overwrite: {self.run_dir}")
        self.run_dir.mkdir(parents=True, exist_ok=False)
        for name in (
            "maps",
            "predictions",
            "variance",
            "visibility",
            "decisions",
            "logs",
        ):
            (self.run_dir / name).mkdir()

        if not ROI_MASK_PATH.exists():
            raise RuntimeError(
                f"Frozen ROI mask is missing: {ROI_MASK_PATH}\n"
                "Run scripts/generate_hospital_roi.py first."
            )
        self.roi = np.load(ROI_MASK_PATH, allow_pickle=False).astype(bool)
        if self.roi.shape != (CANVAS_HEIGHT, CANVAS_WIDTH):
            raise RuntimeError(
                f"ROI shape mismatch: {self.roi.shape} != "
                f"({CANVAS_HEIGHT}, {CANVAS_WIDTH})"
            )
        if int(self.roi.sum()) != ROI_DENOMINATOR:
            raise RuntimeError(
                f"ROI denominator mismatch: {int(self.roi.sum())} != {ROI_DENOMINATOR}"
            )

        self.metadata = {
            "run_id": run_id,
            "method": method,
            "start_time": datetime.now(timezone.utc).isoformat(),
            "git_commit": git_commit(),
            "protocol_version": PROTOCOL_VERSION,
            "fixed_canvas_id": CANVAS_ID,
            "evaluation_roi_id": ROI_ID,
            "evaluation_roi_denominator_cells": ROI_DENOMINATOR,
            "evaluation_roi_sha256": ROI_SHA256,
            "world": "hospital_flat",
            "spawn": {"x": 0.0, "y": 12.0, "yaw": -1.57},
            "exploration_start_sim_s": None,
            "termination_reason": None,
            "notes": "",
        }
        if method == "nearest":
            self.metadata["policy_source_repository"] = MAPEX_SOURCE_REPOSITORY
            self.metadata["policy_source_commit"] = MAPEX_SOURCE_COMMIT
            self.metadata["policy_adapter"] = (
                "mapex_hospital_research/scripts/mapex_nearest_ros.py"
            )
            self.metadata["execution_adapter"] = (
                "Nav2 ComputePathToPose + NavigateToPose replaces MapEx pyastar2d"
            )
        self.metadata_path = self.run_dir / "metadata.json"
        self._write_metadata()

        self.metrics_handle = (self.run_dir / "metrics.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.metrics_writer = csv.writer(self.metrics_handle)
        self.metrics_writer.writerow(
            [
                "time_s",
                "distance_m",
                "known_fraction",
                "coverage",
                "occupied_iou",
                "tu",
            ]
        )
        self.metrics_handle.flush()

        self.trajectory_handle = (self.run_dir / "trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.trajectory_writer = csv.writer(self.trajectory_handle)
        self.trajectory_writer.writerow(
            ["time_s", "x", "y", "yaw", "cumulative_distance_m"]
        )
        self.trajectory_handle.flush()

        self.decisions_path = self.run_dir / "decisions.csv"
        self.decisions: list[dict] = []
        self._flush_decisions()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_map: OccupancyGrid | None = None
        self.latest_known_fraction: float | None = None
        self.latest_coverage: float | None = None
        self.latest_candidate_count: int | None = None
        self.t0_sim: float | None = None
        self.last_odom_xy: tuple[float, float] | None = None
        self.last_trajectory_write_sim: float | None = None
        self.cumulative_distance_m = 0.0
        self.active_decision_index: int | None = None
        self.closed = False

        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_subscription(
            NavPath, "/frontier_selected_path", self._on_selected_path, 10
        )
        self.create_subscription(
            PointStamped, "/frontier_selected", self._on_selected_frontier, 10
        )
        self.create_subscription(
            Int32, "/frontier_candidate_count", self._on_candidate_count, 10
        )
        self.create_subscription(
            PointStamped, "/frontier_completed_goal", self._on_goal_success, 10
        )
        self.create_subscription(
            PointStamped, "/frontier_failed_goal", self._on_goal_failure, 10
        )
        self.create_subscription(Bool, "/exploration_complete", self._on_complete, 10)
        self.create_timer(2.0, self._sample_metrics)

        self.get_logger().info(f"Research run directory: {self.run_dir}")
        self.get_logger().info(
            f"Frozen ROI loaded: {ROI_DENOMINATOR} cells ({ROI_ID})"
        )

    def _now_sim(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed(self) -> float | None:
        if self.t0_sim is None:
            return None
        return max(0.0, self._now_sim() - self.t0_sim)

    def _write_metadata(self) -> None:
        self.metadata_path.write_text(
            json.dumps(self.metadata, indent=2) + "\n", encoding="utf-8"
        )

    def _flush_decisions(self) -> None:
        fields = [
            "decision_id",
            "time_s",
            "known_fraction",
            "num_candidates",
            "selected_candidate_id",
            "selected_distance_m",
            "selected_path_length_m",
            "frontier_x",
            "frontier_y",
            "goal_x",
            "goal_y",
            "result",
        ]
        with self.decisions_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.decisions)

    def _start_exploration_if_needed(self) -> None:
        if self.t0_sim is not None:
            return
        self.t0_sim = self._now_sim()
        self.metadata["exploration_start_sim_s"] = self.t0_sim
        self._write_metadata()
        self.last_odom_xy = None
        self.cumulative_distance_m = 0.0
        self.get_logger().info("Exploration timer started at first selected frontier path")
        self._sample_metrics()

    def _map_metrics(self, msg: OccupancyGrid) -> tuple[float, float]:
        if not math.isclose(
            float(msg.info.resolution), CANVAS_RESOLUTION, rel_tol=1e-6, abs_tol=1e-9
        ):
            raise RuntimeError(
                f"Map resolution changed: {msg.info.resolution} != {CANVAS_RESOLUTION}"
            )

        q = msg.info.origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if abs(yaw) > 1e-5:
            raise RuntimeError(f"Expected axis-aligned SLAM map, origin yaw={yaw}")

        source = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        xoff = math.floor(
            (float(msg.info.origin.position.x) - CANVAS_ORIGIN_X)
            / CANVAS_RESOLUTION
            + 0.5
        )
        yoff = math.floor(
            (float(msg.info.origin.position.y) - CANVAS_ORIGIN_Y)
            / CANVAS_RESOLUTION
            + 0.5
        )

        src_x0 = max(0, -xoff)
        src_y0 = max(0, -yoff)
        dst_x0 = max(0, xoff)
        dst_y0 = max(0, yoff)
        copy_w = min(source.shape[1] - src_x0, CANVAS_WIDTH - dst_x0)
        copy_h = min(source.shape[0] - src_y0, CANVAS_HEIGHT - dst_y0)
        if copy_w <= 0 or copy_h <= 0:
            return 0.0, 0.0

        source_slice = source[
            src_y0 : src_y0 + copy_h,
            src_x0 : src_x0 + copy_w,
        ]
        known = source_slice >= 0
        known_count = int(known.sum())
        roi_slice = self.roi[
            dst_y0 : dst_y0 + copy_h,
            dst_x0 : dst_x0 + copy_w,
        ]
        covered_count = int(np.logical_and(known, roi_slice).sum())
        known_fraction = known_count / float(CANVAS_WIDTH * CANVAS_HEIGHT)
        coverage = covered_count / float(ROI_DENOMINATOR)
        return known_fraction, coverage

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        try:
            self.latest_known_fraction, self.latest_coverage = self._map_metrics(msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Map metric computation failed: {exc}")
            raise

    def _on_candidate_count(self, msg: Int32) -> None:
        self.latest_candidate_count = int(msg.data)

    def _sample_metrics(self) -> None:
        elapsed = self._elapsed()
        if (
            elapsed is None
            or self.latest_known_fraction is None
            or self.latest_coverage is None
        ):
            return
        self.metrics_writer.writerow(
            [
                f"{elapsed:.3f}",
                f"{self.cumulative_distance_m:.4f}",
                f"{self.latest_known_fraction:.8f}",
                f"{self.latest_coverage:.8f}",
                "",
                "",
            ]
        )
        self.metrics_handle.flush()

    def _on_odom(self, msg: Odometry) -> None:
        if self.t0_sim is None:
            return
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        if self.last_odom_xy is not None:
            step = math.hypot(x - self.last_odom_xy[0], y - self.last_odom_xy[1])
            if step <= 1.0:
                self.cumulative_distance_m += step
        self.last_odom_xy = (x, y)

        now = self._now_sim()
        if (
            self.last_trajectory_write_sim is not None
            and now - self.last_trajectory_write_sim < 0.2
        ):
            return
        self.last_trajectory_write_sim = now
        elapsed = max(0.0, now - self.t0_sim)
        self.trajectory_writer.writerow(
            [
                f"{elapsed:.3f}",
                f"{x:.5f}",
                f"{y:.5f}",
                f"{yaw:.6f}",
                f"{self.cumulative_distance_m:.4f}",
            ]
        )
        self.trajectory_handle.flush()

    def _on_selected_path(self, msg: NavPath) -> None:
        if not msg.poses:
            return
        goal = msg.poses[-1].pose.position
        if self.active_decision_index is not None:
            active = self.decisions[self.active_decision_index]
            if (
                active["result"] == "PENDING"
                and active["goal_x"] != ""
                and math.hypot(
                    float(active["goal_x"]) - float(goal.x),
                    float(active["goal_y"]) - float(goal.y),
                ) < 0.05
            ):
                return

        self._start_exploration_if_needed()
        decision_number = len(self.decisions) + 1
        elapsed = self._elapsed() or 0.0
        row = {
            "decision_id": f"decision_{decision_number:06d}",
            "time_s": f"{elapsed:.3f}",
            "known_fraction": (
                ""
                if self.latest_known_fraction is None
                else f"{self.latest_known_fraction:.8f}"
            ),
            "num_candidates": (
                "" if self.latest_candidate_count is None else str(self.latest_candidate_count)
            ),
            "selected_candidate_id": f"frontier_{decision_number:06d}",
            "selected_distance_m": "",
            "selected_path_length_m": f"{path_length(msg):.4f}",
            "frontier_x": "",
            "frontier_y": "",
            "goal_x": f"{float(goal.x):.5f}",
            "goal_y": f"{float(goal.y):.5f}",
            "result": "PENDING",
        }
        self.decisions.append(row)
        self.active_decision_index = len(self.decisions) - 1
        self._flush_decisions()

    def _on_selected_frontier(self, msg: PointStamped) -> None:
        if self.active_decision_index is None:
            return
        row = self.decisions[self.active_decision_index]
        row["frontier_x"] = f"{float(msg.point.x):.5f}"
        row["frontier_y"] = f"{float(msg.point.y):.5f}"
        frame = msg.header.frame_id or "map"
        try:
            transform = self.tf_buffer.lookup_transform(frame, "base_link", Time())
            rx = float(transform.transform.translation.x)
            ry = float(transform.transform.translation.y)
            row["selected_distance_m"] = f"{math.hypot(msg.point.x-rx, msg.point.y-ry):.4f}"
        except TransformException:
            pass
        self._flush_decisions()

    def _finish_active_decision(self, result: str) -> None:
        if self.active_decision_index is None:
            return
        row = self.decisions[self.active_decision_index]
        if row["result"] == "PENDING":
            row["result"] = result
            self._flush_decisions()
        self.active_decision_index = None
        self._sample_metrics()

    def _on_goal_success(self, _msg: PointStamped) -> None:
        self._finish_active_decision("SUCCEEDED")

    def _on_goal_failure(self, _msg: PointStamped) -> None:
        self._finish_active_decision("FAILED")

    def _on_complete(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self.t0_sim is None:
            self.t0_sim = self._now_sim()
            self.metadata["exploration_start_sim_s"] = self.t0_sim
        self._sample_metrics()
        self.finalize("exploration_complete")

    def finalize(self, reason: str) -> None:
        if self.closed:
            return
        self.closed = True
        if self.metadata.get("termination_reason") is None:
            self.metadata["termination_reason"] = reason
        self.metadata["final_distance_m"] = self.cumulative_distance_m
        self.metadata["final_known_fraction"] = self.latest_known_fraction
        self.metadata["final_coverage"] = self.latest_coverage
        self.metadata["end_time"] = datetime.now(timezone.utc).isoformat()
        self._write_metadata()
        self.metrics_handle.flush()
        self.trajectory_handle.flush()
        self.metrics_handle.close()
        self.trajectory_handle.close()
        self.get_logger().info(
            f"Run finalized: reason={reason}, distance={self.cumulative_distance_m:.2f} m, "
            f"coverage={self.latest_coverage}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=["nearest", "mapex"], required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    rclpy.init()
    node: ResearchRecorder | None = None
    try:
        node = ResearchRecorder(args.method, args.run_id)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.finalize("interrupted")
    finally:
        if node is not None:
            node.finalize(node.metadata.get("termination_reason") or "process_shutdown")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
