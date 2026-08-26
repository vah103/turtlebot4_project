#!/usr/bin/env python3
"""Official Hospital research recorder wrapper.

Adds benchmark-clock synchronization, exact execution-goal audit, and stronger
provenance to the replay-safe recorder.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

from research_recorder import REPO_ROOT
from research_recorder_safe import SafeResearchRecorder, _sha256_file


START_TOPIC = "/frontier_exploration_start"
SELECTED_CANDIDATE_TOPIC = "/frontier_policy_selected_candidate"


class OfficialResearchRecorder(SafeResearchRecorder):
    def __init__(self, method: str, run_id: str) -> None:
        super().__init__(method, run_id)
        self._pending_selected_candidate: dict[str, str] = {}

        self.create_subscription(String, START_TOPIC, self._on_exploration_start, 10)
        self.create_subscription(
            String,
            SELECTED_CANDIDATE_TOPIC,
            self._on_selected_candidate_id,
            10,
        )

        workspace = REPO_ROOT / "mapex_hospital_research"
        extra = {
            "nearest_hospital_adapted": workspace
            / "scripts/mapex_nearest_ros_hospital_adapted.py",
            "nearest_official_policy": workspace
            / "scripts/mapex_nearest_ros_official.py",
            "official_recorder": workspace / "scripts/research_recorder_official.py",
            "nearest_config": workspace / "config/nearest.yaml",
            "experiment_protocol": workspace / "EXPERIMENT_PROTOCOL.md",
            "data_schema": workspace / "docs/DATA_SCHEMA.md",
        }

        share = Path(get_package_share_directory("frontier_exploration"))
        installed = {
            "installed_hospital_flat_stack": share / "launch/hospital_flat_stack.launch.py",
            "installed_hospital_flat_simulation": share
            / "launch/hospital_flat_simulation.launch.py",
            "installed_hospital_nav2_launch": share / "launch/hospital_nav2.launch.py",
            "installed_frontier_config": share / "config/frontier.yaml",
            "installed_hospital_slam": share / "config/hospital_slam.yaml",
            "installed_hospital_nav2_override": share
            / "config/nav2_hospital_override.yaml",
            "installed_hospital_world": share / "worlds/hospital_aws_flat.sdf",
        }
        hashes = dict(self.metadata.get("config_sha256") or {})
        for name, path in {**extra, **installed}.items():
            hashes[name] = _sha256_file(path)
        self.metadata["config_sha256"] = hashes
        self.metadata["frontier_exploration_package_share"] = str(share)
        self.metadata["sim_seed"] = None
        self.metadata["sim_seed_policy"] = (
            "intentionally_uncontrolled_gazebo_default_multiple_run_statistics"
        )
        self.metadata["exploration_start_source"] = None
        self.metadata["execution_goal_semantics"] = "exact_frontier_center"
        self.metadata["planner_path_semantics"] = "reachability_evidence_only"
        self._write_metadata()

        self.get_logger().info(
            "Official recorder active: benchmark clock waits for first policy "
            "decision before computation; installed runtime files are hashed"
        )

    def _flush_decisions(self) -> None:
        """Write execution goal and planner endpoint as separate audited fields."""
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
            "planner_endpoint_x",
            "planner_endpoint_y",
            "planner_endpoint_to_frontier_m",
            "goal_x",
            "goal_y",
            "goal_source",
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

    def _on_exploration_start(self, msg: String) -> None:
        if self.closed or self.t0_sim is not None:
            return
        try:
            payload = json.loads(msg.data)
            t0 = float(payload["sim_time_s"])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Invalid exploration-start payload: {exc}")
            return

        self.t0_sim = t0
        self.metadata["exploration_start_sim_s"] = t0
        self.metadata["exploration_start_source"] = str(
            payload.get("event", "policy_start_topic")
        )
        self.metadata["exploration_start_map_stamp_s"] = payload.get("map_stamp_s")
        self.last_odom_xy = None
        self.last_trajectory_write_sim = None
        self.cumulative_distance_m = 0.0
        self._write_metadata()
        self.get_logger().info(
            f"Benchmark timer synchronized to policy pre-compute start: {t0:.6f}"
        )
        self._sample_metrics()

    def _on_selected_path(self, msg) -> None:
        """Keep the planner endpoint for audit; it is not the execution goal."""
        before = len(self.decisions)
        super()._on_selected_path(msg)
        if self.closed or len(self.decisions) == before:
            return

        row = self.decisions[-1]
        row["planner_endpoint_x"] = row.get("goal_x", "")
        row["planner_endpoint_y"] = row.get("goal_y", "")
        row["planner_endpoint_to_frontier_m"] = ""
        row["goal_source"] = "pending_exact_frontier"
        self._flush_decisions()

    def _on_selected_frontier(self, msg) -> None:
        """The exact policy frontier is the actual NavigateToPose goal."""
        super()._on_selected_frontier(msg)
        if self.closed or self.active_decision_index is None:
            return

        row = self.decisions[self.active_decision_index]
        fx = float(msg.point.x)
        fy = float(msg.point.y)
        row["goal_x"] = f"{fx:.5f}"
        row["goal_y"] = f"{fy:.5f}"
        row["goal_source"] = "exact_frontier_center"

        try:
            px = float(row.get("planner_endpoint_x", ""))
            py = float(row.get("planner_endpoint_y", ""))
            row["planner_endpoint_to_frontier_m"] = (
                f"{math.hypot(px - fx, py - fy):.5f}"
            )
        except Exception:  # noqa: BLE001
            row["planner_endpoint_to_frontier_m"] = ""
        self._flush_decisions()

    def _apply_candidate_id(self, policy_decision_id: str, candidate_id: str) -> bool:
        for row in reversed(self.decisions):
            if row.get("policy_decision_id") == policy_decision_id:
                row["selected_candidate_id"] = candidate_id
                self._flush_decisions()
                return True
        return False

    def _on_selected_candidate_id(self, msg: String) -> None:
        if self.closed:
            return
        try:
            payload = json.loads(msg.data)
            pid = str(payload["policy_decision_id"])
            cid = str(payload["candidate_id"])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Invalid selected-candidate payload: {exc}")
            return
        if not self._apply_candidate_id(pid, cid):
            self._pending_selected_candidate[pid] = cid

    def _on_policy_decision_id(self, msg: String) -> None:
        super()._on_policy_decision_id(msg)
        pid = msg.data.strip()
        cid = self._pending_selected_candidate.pop(pid, None)
        if cid:
            self._apply_candidate_id(pid, cid)

    def _on_goal_result_detail(self, msg: String) -> None:
        """Match result details to the exact frontier goal, even under DDS reordering."""
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
            coordinate_pairs = []
            if row.get("goal_x", "") != "" and row.get("goal_y", "") != "":
                coordinate_pairs.append((float(row["goal_x"]), float(row["goal_y"])))
            if row.get("frontier_x", "") != "" and row.get("frontier_y", "") != "":
                coordinate_pairs.append(
                    (float(row["frontier_x"]), float(row["frontier_y"]))
                )
            if any(math.hypot(x - gx, y - gy) < 0.05 for x, y in coordinate_pairs):
                match = row
                break
        if match is None:
            self.get_logger().warning(
                "Navigation detail could not be matched to an exact frontier decision"
            )
            return

        match["goal_x"] = f"{gx:.5f}"
        match["goal_y"] = f"{gy:.5f}"
        match["goal_source"] = str(
            payload.get("goal_source", "exact_frontier_center")
        )
        if "planner_endpoint_x" in payload:
            match["planner_endpoint_x"] = f"{float(payload['planner_endpoint_x']):.5f}"
        if "planner_endpoint_y" in payload:
            match["planner_endpoint_y"] = f"{float(payload['planner_endpoint_y']):.5f}"
        if "planner_endpoint_to_frontier_m" in payload:
            match["planner_endpoint_to_frontier_m"] = (
                f"{float(payload['planner_endpoint_to_frontier_m']):.5f}"
            )

        detail = str(payload.get("detail", ""))
        succeeded = bool(payload.get("succeeded", False))
        match["navigation_detail"] = detail
        match["failure_reason"] = "" if succeeded else detail
        self._flush_decisions()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=["nearest", "mapex"], required=True)
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    rclpy.init()
    node: OfficialResearchRecorder | None = None
    try:
        node = OfficialResearchRecorder(args.method, args.run_id)
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
