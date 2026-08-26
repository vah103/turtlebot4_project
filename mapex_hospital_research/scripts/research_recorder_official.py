#!/usr/bin/env python3
"""Official Hospital research recorder wrapper.

Adds benchmark-clock synchronization and stronger provenance to the replay-safe
recorder without changing metrics/navigation semantics.
"""

from __future__ import annotations

import argparse
import json
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
        self._write_metadata()

        self.get_logger().info(
            "Official recorder active: benchmark clock waits for first policy "
            "decision before computation; installed runtime files are hashed"
        )

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
