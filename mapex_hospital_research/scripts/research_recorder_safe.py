#!/usr/bin/env python3
"""Safe and replay-oriented Hospital research recorder.

Responsibilities:
- preserve the base run metrics/trajectory/decision semantics;
- never write after finalization;
- record explicit policy failures;
- link selected goals to the exact policy-decision ID;
- retain map-frame robot pose and detailed navigation failure reason;
- save compressed periodic + final OccupancyGrid snapshots for offline IoU/TU.

Exact *decision* maps/candidates are intentionally written by
``mapex_nearest_ros_research.py`` because only the policy process owns the
frozen OccupancyGrid that was actually used for ranking. This recorder does not
mislabel a later ``latest_map`` as the decision map.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path

import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException

from research_recorder import (
    CANVAS_HEIGHT,
    CANVAS_ID,
    CANVAS_ORIGIN_X,
    CANVAS_ORIGIN_Y,
    CANVAS_RESOLUTION,
    CANVAS_WIDTH,
    REPO_ROOT,
    ResearchRecorder,
)


PERIODIC_MAP_INTERVAL_S = 10.0
DECISION_ID_TOPIC = "/frontier_policy_decision_id"
GOAL_DETAIL_TOPIC = "/frontier_goal_result_detail"


def _sha256_file(path: Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except OSError:
        return None


def _git_dirty() -> bool | None:
    try:
        output = subprocess.check_output(
            ["git", "status", "--porcelain"],
            cwd=REPO_ROOT,
            text=True,
            stderr=subprocess.DEVNULL,
        )
        return bool(output.strip())
    except Exception:  # noqa: BLE001
        return None


class SafeResearchRecorder(ResearchRecorder):
    def __init__(self, method: str, run_id: str) -> None:
        super().__init__(method, run_id)

        self._pending_policy_decision_id: str | None = None
        self._periodic_index = 0
        self._last_periodic_sim_s: float | None = None

        self.snapshots_path = self.run_dir / "snapshots.csv"
        self.snapshots_handle = self.snapshots_path.open(
            "w", newline="", encoding="utf-8"
        )
        self.snapshots_writer = csv.writer(self.snapshots_handle)
        self.snapshots_writer.writerow(
            [
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
        )
        self.snapshots_handle.flush()

        self.metadata["exact_policy_decision_state"] = (
            "written_by_mapex_nearest_ros_research_under_decisions/"
        )
        self.metadata["recorder_snapshot_strategy"] = (
            f"periodic_every_{PERIODIC_MAP_INTERVAL_S:g}s_plus_final"
        )
        self.metadata["recorder_snapshot_format"] = "numpy_npz_compressed"
        self.metadata["git_dirty_at_recorder_start"] = _git_dirty()
        self.metadata["sim_seed"] = "not_explicitly_configured"

        tracked = {
            "hospital_nearest_launch": REPO_ROOT
            / "mapex_hospital_research/launch/hospital_nearest.launch.py",
            "nearest_base_policy": REPO_ROOT
            / "mapex_hospital_research/scripts/mapex_nearest_ros.py",
            "nearest_hospital_wrapper": REPO_ROOT
            / "mapex_hospital_research/scripts/mapex_nearest_ros_hospital.py",
            "nearest_research_wrapper": REPO_ROOT
            / "mapex_hospital_research/scripts/mapex_nearest_ros_research.py",
            "research_recorder_safe": REPO_ROOT
            / "mapex_hospital_research/scripts/research_recorder_safe.py",
            "research_manager": REPO_ROOT
            / "mapex_hospital_research/scripts/exploration_manager_research.py",
            "hospital_slam": REPO_ROOT
            / "ros2_ws/src/frontier_exploration/config/hospital_slam.yaml",
            "hospital_nav2_override": REPO_ROOT
            / "ros2_ws/src/frontier_exploration/config/nav2_hospital_override.yaml",
            "frontier_config": REPO_ROOT
            / "ros2_ws/src/frontier_exploration/config/frontier.yaml",
        }
        self.metadata["config_sha256"] = {
            name: _sha256_file(path) for name, path in tracked.items()
        }
        self._write_metadata()

        self.create_subscription(
            String, "/exploration_failed", self._on_policy_failure, 10
        )
        self.create_subscription(
            String, DECISION_ID_TOPIC, self._on_policy_decision_id, 10
        )
        self.create_subscription(
            String, GOAL_DETAIL_TOPIC, self._on_goal_result_detail, 10
        )
        self.create_timer(1.0, self._maybe_save_periodic_snapshot)

        self.get_logger().info(
            "Replay-safe recorder active: exact policy states come from policy node; "
            f"periodic map snapshots every {PERIODIC_MAP_INTERVAL_S:g}s + final"
        )

    def _flush_decisions(self) -> None:
        fields = [
            "decision_id",
            "policy_decision_id",
            "time_s",
            "known_fraction",
            "coverage",
            "num_candidates",
            "selected_candidate_id",
            "selected_distance_m",
            "selected_path_length_m",
            "frontier_x",
            "frontier_y",
            "goal_x",
            "goal_y",
            "robot_map_x",
            "robot_map_y",
            "robot_map_yaw",
            "result",
            "navigation_detail",
            "failure_reason",
        ]
        with self.decisions_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fields)
            writer.writeheader()
            for row in self.decisions:
                writer.writerow({field: row.get(field, "") for field in fields})

    @staticmethod
    def _origin_yaw(msg) -> float:
        q = msg.info.origin.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def _stamp_s(msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

    def _map_pose(self, frame: str) -> tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(frame, "base_link", Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            float(yaw),
        )

    def _map_on_fixed_canvas(self, msg) -> np.ndarray:
        if not math.isclose(
            float(msg.info.resolution), CANVAS_RESOLUTION, rel_tol=1e-6, abs_tol=1e-9
        ):
            raise RuntimeError(
                f"Map resolution changed: {msg.info.resolution} != {CANVAS_RESOLUTION}"
            )
        if abs(self._origin_yaw(msg)) > 1e-5:
            raise RuntimeError("Expected axis-aligned SLAM map for snapshot")

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

    def _save_recorder_snapshot(self, event: str) -> None:
        if self.latest_map is None:
            return
        msg = self.latest_map
        self._periodic_index += 1
        snapshot_id = f"{event}_{self._periodic_index:06d}"
        raw_file = f"maps/{snapshot_id}_raw.npz"
        canvas_file = f"maps/{snapshot_id}_canvas.npz"

        raw = np.asarray(msg.data, dtype=np.int8).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        np.savez_compressed(
            self.run_dir / raw_file,
            data=raw,
            resolution=np.float64(msg.info.resolution),
            width=np.int32(msg.info.width),
            height=np.int32(msg.info.height),
            origin_x=np.float64(msg.info.origin.position.x),
            origin_y=np.float64(msg.info.origin.position.y),
            origin_yaw=np.float64(self._origin_yaw(msg)),
            frame_id=np.asarray(msg.header.frame_id or "map"),
            source_stamp_s=np.float64(self._stamp_s(msg)),
        )
        canvas = self._map_on_fixed_canvas(msg)
        np.savez_compressed(
            self.run_dir / canvas_file,
            data=canvas,
            resolution=np.float64(CANVAS_RESOLUTION),
            width=np.int32(CANVAS_WIDTH),
            height=np.int32(CANVAS_HEIGHT),
            origin_x=np.float64(CANVAS_ORIGIN_X),
            origin_y=np.float64(CANVAS_ORIGIN_Y),
            canvas_id=np.asarray(CANVAS_ID),
        )

        frame = msg.header.frame_id or "map"
        pose = self._map_pose(frame)
        elapsed = self._elapsed()
        self.snapshots_writer.writerow(
            [
                snapshot_id,
                event,
                "" if elapsed is None else f"{elapsed:.3f}",
                "" if self.latest_coverage is None else f"{self.latest_coverage:.8f}",
                "" if self.latest_known_fraction is None else f"{self.latest_known_fraction:.8f}",
                f"{self.cumulative_distance_m:.4f}",
                "" if pose is None else f"{pose[0]:.5f}",
                "" if pose is None else f"{pose[1]:.5f}",
                "" if pose is None else f"{pose[2]:.6f}",
                raw_file,
                canvas_file,
                f"{self._stamp_s(msg):.9f}",
                CANVAS_ID,
            ]
        )
        self.snapshots_handle.flush()

    def _maybe_save_periodic_snapshot(self) -> None:
        if self.closed or self.t0_sim is None or self.latest_map is None:
            return
        now = self._now_sim()
        if (
            self._last_periodic_sim_s is not None
            and now - self._last_periodic_sim_s < PERIODIC_MAP_INTERVAL_S
        ):
            return
        self._last_periodic_sim_s = now
        try:
            self._save_recorder_snapshot("periodic")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Periodic map snapshot failed: {exc}")

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
        super()._on_odom(msg)

    def _on_selected_path(self, msg) -> None:
        if self.closed:
            return
        before = len(self.decisions)
        super()._on_selected_path(msg)
        if len(self.decisions) == before:
            return

        row = self.decisions[-1]
        row["policy_decision_id"] = self._pending_policy_decision_id or ""
        self._pending_policy_decision_id = None
        row["coverage"] = (
            "" if self.latest_coverage is None else f"{self.latest_coverage:.8f}"
        )
        frame = msg.header.frame_id or "map"
        pose = self._map_pose(frame)
        row["robot_map_x"] = "" if pose is None else f"{pose[0]:.5f}"
        row["robot_map_y"] = "" if pose is None else f"{pose[1]:.5f}"
        row["robot_map_yaw"] = "" if pose is None else f"{pose[2]:.6f}"
        row["navigation_detail"] = ""
        row["failure_reason"] = ""
        self._flush_decisions()

    def _on_selected_frontier(self, msg) -> None:
        if self.closed:
            return
        super()._on_selected_frontier(msg)

    def _on_policy_decision_id(self, msg: String) -> None:
        if self.closed:
            return
        decision_id = msg.data.strip()
        if not decision_id:
            return
        if self.active_decision_index is not None:
            row = self.decisions[self.active_decision_index]
            if row.get("result") == "PENDING":
                row["policy_decision_id"] = decision_id
                self._flush_decisions()
                return
        self._pending_policy_decision_id = decision_id

    def _on_goal_result_detail(self, msg: String) -> None:
        if self.closed:
            return
        try:
            payload = json.loads(msg.data)
            gx = float(payload["goal_x"])
            gy = float(payload["goal_y"])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Invalid navigation detail message: {exc}")
            return

        match = None
        for row in reversed(self.decisions):
            if row.get("goal_x", "") == "" or row.get("goal_y", "") == "":
                continue
            if math.hypot(float(row["goal_x"]) - gx, float(row["goal_y"]) - gy) < 0.05:
                match = row
                break
        if match is None:
            return

        detail = str(payload.get("detail", ""))
        succeeded = bool(payload.get("succeeded", False))
        match["navigation_detail"] = detail
        match["failure_reason"] = "" if succeeded else detail
        self._flush_decisions()

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
        try:
            self._save_recorder_snapshot("final")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Final map snapshot failed: {exc}")
            self.metadata["snapshot_final_error"] = str(exc)

        self.metadata["recorder_snapshot_count"] = self._periodic_index
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
