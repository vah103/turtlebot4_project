#!/usr/bin/env python3
"""Research execution adapter for Hospital frontier exploration.

The planner path is used only as a reachability certificate.  The actual
NavigateToPose goal is the exact MapEx frontier center published on
``/frontier_selected``.  This avoids silently replacing a frontier with the
last pose of a tolerance-snapped ComputePathToPose result.

The wrapper also publishes detailed Nav2 execution outcomes for research logs.
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from frontier_exploration.exploration_manager import ExplorationManager


DETAIL_TOPIC = "/frontier_goal_result_detail"
FRONTIER_TOPIC = "/frontier_selected"


class ResearchExplorationManager(ExplorationManager):
    def __init__(self) -> None:
        # Base __init__ subscribes to /frontier_selected_path using self._on_path,
        # so Python dispatch binds that subscription to the override below.
        self._validated_path: Path | None = None
        self._exact_frontier: PointStamped | None = None
        self._active_planner_endpoint_xy: tuple[float, float] | None = None
        self._active_planner_endpoint_offset_m: float | None = None
        super().__init__()

        self._result_detail_pub = self.create_publisher(String, DETAIL_TOPIC, 10)
        self.create_subscription(
            PointStamped,
            FRONTIER_TOPIC,
            self._on_exact_frontier,
            10,
        )
        self.get_logger().info(
            f"Research navigation result details enabled on {DETAIL_TOPIC}"
        )
        self.get_logger().warning(
            "EXACT FRONTIER EXECUTION ACTIVE: ComputePathToPose path is reachability "
            "evidence only; NavigateToPose uses the exact /frontier_selected center"
        )

    def _on_path(self, path: Path) -> None:
        """Hold a validated path until its exact MapEx frontier center arrives."""
        if not path.poses:
            return

        if self._active_goal is not None:
            if not self._active_path_notice_shown:
                self.get_logger().info(
                    "Ignoring frontier path updates while navigation is active; "
                    "a fresh path/frontier pair will be required after completion"
                )
                self._active_path_notice_shown = True
            return

        self._validated_path = path
        self._try_queue_exact_frontier_goal()

    def _on_exact_frontier(self, msg: PointStamped) -> None:
        """Pair the policy's exact frontier center with its validated path."""
        if self._active_goal is not None:
            return
        self._exact_frontier = msg
        self._try_queue_exact_frontier_goal()

    def _try_queue_exact_frontier_goal(self) -> None:
        if self._validated_path is None or self._exact_frontier is None:
            return
        if self._active_goal is not None or self._pending_goal is not None:
            return

        path = self._validated_path
        frontier = self._exact_frontier
        endpoint_pose = path.poses[-1]

        path_frame = (
            path.header.frame_id
            or endpoint_pose.header.frame_id
            or frontier.header.frame_id
            or "map"
        )
        frontier_frame = frontier.header.frame_id or path_frame
        if path_frame != frontier_frame:
            self.get_logger().error(
                "Validated path/frontier frame mismatch: "
                f"path={path_frame!r}, frontier={frontier_frame!r}; dropping pair"
            )
            self._validated_path = None
            self._exact_frontier = None
            return

        endpoint_x = float(endpoint_pose.pose.position.x)
        endpoint_y = float(endpoint_pose.pose.position.y)
        frontier_x = float(frontier.point.x)
        frontier_y = float(frontier.point.y)
        endpoint_offset = math.hypot(
            endpoint_x - frontier_x,
            endpoint_y - frontier_y,
        )

        goal = PoseStamped()
        goal.header.frame_id = frontier_frame
        # Use latest TF, matching the existing manager's anti-extrapolation rule.
        goal.header.stamp.sec = 0
        goal.header.stamp.nanosec = 0
        goal.pose.position.x = frontier_x
        goal.pose.position.y = frontier_y
        goal.pose.orientation.w = 1.0

        self._active_planner_endpoint_xy = (endpoint_x, endpoint_y)
        self._active_planner_endpoint_offset_m = endpoint_offset
        self._validated_path = None
        self._exact_frontier = None

        if endpoint_offset > 0.05:
            self.get_logger().warning(
                "Planner endpoint differs from exact frontier center by "
                f"{endpoint_offset:.3f} m; executing exact frontier instead: "
                f"frontier=({frontier_x:.2f}, {frontier_y:.2f}), "
                f"planner_endpoint=({endpoint_x:.2f}, {endpoint_y:.2f})"
            )

        if self._awaiting_fresh_path:
            self._awaiting_fresh_path = False
            self.get_logger().info(
                "Fresh validated path + exact frontier pair received after "
                "navigation completion"
            )

        self._active_path_notice_shown = False
        self._pending_goal = goal
        self._process_pending_goal()

    def _finish_navigation(self, succeeded: bool, detail: str) -> None:
        goal = self._active_goal
        frame = self._active_goal_frame
        endpoint = self._active_planner_endpoint_xy
        endpoint_offset = self._active_planner_endpoint_offset_m
        if goal is not None:
            msg = String()
            payload = {
                "goal_x": float(goal[0]),
                "goal_y": float(goal[1]),
                "frame_id": frame or "map",
                "goal_source": "exact_frontier_center",
                "succeeded": bool(succeeded),
                "detail": str(detail),
                "sim_time_s": float(self._now_sec()),
            }
            if endpoint is not None:
                payload["planner_endpoint_x"] = float(endpoint[0])
                payload["planner_endpoint_y"] = float(endpoint[1])
            if endpoint_offset is not None:
                payload["planner_endpoint_to_frontier_m"] = float(endpoint_offset)

            msg.data = json.dumps(payload, separators=(",", ":"))
            self._result_detail_pub.publish(msg)

        super()._finish_navigation(succeeded, detail)
        self._active_planner_endpoint_xy = None
        self._active_planner_endpoint_offset_m = None


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
