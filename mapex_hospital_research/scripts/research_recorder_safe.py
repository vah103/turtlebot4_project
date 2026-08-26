#!/usr/bin/env python3
"""Safe runtime wrapper for the Hospital research recorder.

Adds runtime guarantees without changing metric semantics:
* callbacks become no-ops after files are finalized/closed;
* explicit exploration-policy failures are recorded in metadata;
* the live SLAM map is saved on the frozen Hospital canvas at every new
  frontier decision and once more at run finalization.

Snapshots preserve ROS OccupancyGrid values exactly (-1 unknown, 0..100 known)
and are stored as NumPy arrays under ``maps/``. ``snapshots.csv`` links every
snapshot to the decision, time, coverage, traveled distance, and robot pose.
"""

from __future__ import annotations

import argparse
import csv
import math

import numpy as np
import rclpy
from std_msgs.msg import String

from research_recorder import (
    CANVAS_HEIGHT,
    CANVAS_ID,
    CANVAS_ORIGIN_X,
    CANVAS_ORIGIN_Y,
    CANVAS_RESOLUTION,
    CANVAS_WIDTH,
    ResearchRecorder,
)


class SafeResearchRecorder(ResearchRecorder):
    def __init__(self, method: str, run_id: str) -> None:
        super().__init__(method, run_id)

        self.snapshot_index = 0
        self.snapshot_pose: tuple[float, float, float] | None = None
        self.snapshots_path = self.run_dir / "snapshots.csv"
        self.snapshots_handle = self.snapshots_path.open(
            "w", newline="", encoding="utf-8"
        )
        self.snapshots_writer = csv.writer(self.snapshots_handle)
        self.snapshots_writer.writerow(
            [
                "snapshot_id",
                "event",
                "decision_id",
                "time_s",
                "coverage",
                "known_fraction",
                "distance_m",
                "robot_x",
                "robot_y",
                "robot_yaw",
                "goal_x",
                "goal_y",
                "map_file",
                "canvas_id",
                "resolution_m",
                "width",
                "height",
                "origin_x",
                "origin_y",
                "dtype",
                "unknown_value",
            ]
        )
        self.snapshots_handle.flush()

        self.metadata["snapshot_strategy"] = "each_frontier_decision_plus_final"
        self.metadata["snapshot_canvas_id"] = CANVAS_ID
        self.metadata["snapshot_format"] = "numpy_npy_fixed_canvas_ros_occupancy_grid"
        self.metadata["snapshot_dtype"] = "int8"
        self.metadata["snapshot_unknown_value"] = -1
        self.metadata["snapshot_known_value_range"] = [0, 100]
        self._write_metadata()

        self.create_subscription(
            String, "/exploration_failed", self._on_policy_failure, 10
        )
        self.get_logger().info(
            "Map snapshots enabled: every new frontier decision + final map "
            f"on fixed canvas {CANVAS_WIDTH}x{CANVAS_HEIGHT} @ "
            f"{CANVAS_RESOLUTION:.2f} m/cell"
        )

    @staticmethod
    def _origin_yaw(msg) -> float:
        q = msg.info.origin.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _map_on_fixed_canvas(self, msg) -> np.ndarray:
        """Place one live OccupancyGrid on the frozen Hospital research canvas."""
        if not math.isclose(
            float(msg.info.resolution),
            CANVAS_RESOLUTION,
            rel_tol=1e-6,
            abs_tol=1e-9,
        ):
            raise RuntimeError(
                f"Map resolution changed: {msg.info.resolution} != "
                f"{CANVAS_RESOLUTION}"
            )

        yaw = self._origin_yaw(msg)
        if abs(yaw) > 1e-5:
            raise RuntimeError(
                f"Expected axis-aligned SLAM map for snapshot, origin yaw={yaw}"
            )

        source = np.asarray(msg.data, dtype=np.int8).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        canvas = np.full((CANVAS_HEIGHT, CANVAS_WIDTH), -1, dtype=np.int8)

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

        if copy_w > 0 and copy_h > 0:
            canvas[
                dst_y0 : dst_y0 + copy_h,
                dst_x0 : dst_x0 + copy_w,
            ] = source[
                src_y0 : src_y0 + copy_h,
                src_x0 : src_x0 + copy_w,
            ]

        return canvas

    def _save_snapshot(self, event: str, decision_id: str = "") -> None:
        if self.latest_map is None:
            self.get_logger().warning(
                f"Skipping {event} snapshot because no /map has been received yet"
            )
            return

        canvas = self._map_on_fixed_canvas(self.latest_map)
        self.snapshot_index += 1
        snapshot_id = f"map_{self.snapshot_index:06d}"
        relative_file = f"maps/{snapshot_id}.npy"
        np.save(self.run_dir / relative_file, canvas, allow_pickle=False)

        elapsed = self._elapsed()
        pose = self.snapshot_pose
        goal_x = ""
        goal_y = ""
        if decision_id and self.decisions:
            row = next(
                (item for item in reversed(self.decisions) if item["decision_id"] == decision_id),
                None,
            )
            if row is not None:
                goal_x = row.get("goal_x", "")
                goal_y = row.get("goal_y", "")

        self.snapshots_writer.writerow(
            [
                snapshot_id,
                event,
                decision_id,
                "" if elapsed is None else f"{elapsed:.3f}",
                "" if self.latest_coverage is None else f"{self.latest_coverage:.8f}",
                (
                    ""
                    if self.latest_known_fraction is None
                    else f"{self.latest_known_fraction:.8f}"
                ),
                f"{self.cumulative_distance_m:.4f}",
                "" if pose is None else f"{pose[0]:.5f}",
                "" if pose is None else f"{pose[1]:.5f}",
                "" if pose is None else f"{pose[2]:.6f}",
                goal_x,
                goal_y,
                relative_file,
                CANVAS_ID,
                f"{CANVAS_RESOLUTION:.3f}",
                CANVAS_WIDTH,
                CANVAS_HEIGHT,
                f"{CANVAS_ORIGIN_X:.3f}",
                f"{CANVAS_ORIGIN_Y:.3f}",
                "int8",
                -1,
            ]
        )
        self.snapshots_handle.flush()
        self.get_logger().info(
            f"Saved {event} map snapshot: {relative_file} "
            f"(decision={decision_id or '-'}, coverage={self.latest_coverage})"
        )

    def _on_map(self, msg) -> None:
        if self.closed:
            return
        super()._on_map(msg)

    def _on_candidate_count(self, msg) -> None:
        if self.closed:
            return
        super()._on_candidate_count(msg)

    def _sample_metrics(self) -> None:
        if self.closed:
            return
        super()._sample_metrics()

    def _on_odom(self, msg) -> None:
        if self.closed:
            return

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.snapshot_pose = (x, y, yaw)
        super()._on_odom(msg)

    def _on_selected_path(self, msg) -> None:
        if self.closed:
            return

        decisions_before = len(self.decisions)
        super()._on_selected_path(msg)
        if len(self.decisions) > decisions_before:
            decision_id = self.decisions[-1]["decision_id"]
            self._save_snapshot("decision", decision_id)

    def _on_selected_frontier(self, msg) -> None:
        if self.closed:
            return
        super()._on_selected_frontier(msg)

    def _finish_active_decision(self, result: str) -> None:
        if self.closed:
            return
        super()._finish_active_decision(result)

    def _on_goal_success(self, msg) -> None:
        if self.closed:
            return
        super()._on_goal_success(msg)

    def _on_goal_failure(self, msg) -> None:
        if self.closed:
            return
        super()._on_goal_failure(msg)

    def _on_complete(self, msg) -> None:
        if self.closed:
            return
        super()._on_complete(msg)

    def _on_policy_failure(self, msg: String) -> None:
        if self.closed:
            return
        if self.t0_sim is None:
            self.t0_sim = self._now_sim()
            self.metadata["exploration_start_sim_s"] = self.t0_sim
        self.metadata["policy_failure"] = msg.data
        self._sample_metrics()
        self.finalize(f"policy_failure:{msg.data}")

    def finalize(self, reason: str) -> None:
        if self.closed:
            return

        active_decision_id = ""
        if self.active_decision_index is not None:
            active_decision_id = self.decisions[self.active_decision_index]["decision_id"]

        try:
            self._save_snapshot("final", active_decision_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Final map snapshot failed: {exc}")
            self.metadata["snapshot_final_error"] = str(exc)

        self.metadata["snapshot_count"] = self.snapshot_index
        self._write_metadata()
        super().finalize(reason)

        self.snapshots_handle.flush()
        self.snapshots_handle.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=["nearest", "mapex"], required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    rclpy.init()
    node: SafeResearchRecorder | None = None
    try:
        node = SafeResearchRecorder(args.method, args.run_id)
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
