#!/usr/bin/env python3
"""Safe runtime wrapper for the Hospital research recorder.

Adds two runtime guarantees without changing metric semantics:
* callbacks become no-ops after files are finalized/closed;
* explicit exploration-policy failures are recorded in metadata.
"""

from __future__ import annotations

import argparse

import rclpy
from std_msgs.msg import String

from research_recorder import ResearchRecorder


class SafeResearchRecorder(ResearchRecorder):
    def __init__(self, method: str, run_id: str) -> None:
        super().__init__(method, run_id)
        self.create_subscription(
            String, "/exploration_failed", self._on_policy_failure, 10
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
        super()._on_odom(msg)

    def _on_selected_path(self, msg) -> None:
        if self.closed:
            return
        super()._on_selected_path(msg)

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
