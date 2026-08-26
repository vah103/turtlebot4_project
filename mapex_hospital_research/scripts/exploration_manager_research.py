#!/usr/bin/env python3
"""Research wrapper for frontier_exploration.ExplorationManager.

Navigation behaviour is unchanged. The only addition is a JSON detail topic so
research logs retain the concrete Nav2 execution outcome (success, rejection,
timeout, stall, status code, etc.) instead of collapsing every failure to the
single label FAILED.
"""

from __future__ import annotations

import json

import rclpy
from std_msgs.msg import String

from frontier_exploration.exploration_manager import ExplorationManager


DETAIL_TOPIC = "/frontier_goal_result_detail"


class ResearchExplorationManager(ExplorationManager):
    def __init__(self) -> None:
        super().__init__()
        self._result_detail_pub = self.create_publisher(String, DETAIL_TOPIC, 10)
        self.get_logger().info(
            f"Research navigation result details enabled on {DETAIL_TOPIC}"
        )

    def _finish_navigation(self, succeeded: bool, detail: str) -> None:
        goal = self._active_goal
        frame = self._active_goal_frame
        if goal is not None:
            msg = String()
            msg.data = json.dumps(
                {
                    "goal_x": float(goal[0]),
                    "goal_y": float(goal[1]),
                    "frame_id": frame or "map",
                    "succeeded": bool(succeeded),
                    "detail": str(detail),
                    "sim_time_s": float(self._now_sec()),
                },
                separators=(",", ":"),
            )
            self._result_detail_pub.publish(msg)
        super()._finish_navigation(succeeded, detail)


def main() -> None:
    rclpy.init()
    node = ResearchExplorationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
