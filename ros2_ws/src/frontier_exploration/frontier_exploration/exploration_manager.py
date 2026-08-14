"""Optional Nav2 execution manager for the frontier exploration baseline.

Navigation is disabled by default. When explicitly enabled, the manager consumes
planner-validated paths from the frontier detector and sends their final pose to
Nav2 NavigateToPose. It never publishes cmd_vel directly.
"""

from math import hypot

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node


class ExplorationManager(Node):
    """Execute planner-validated frontier goals only when explicitly enabled."""

    def __init__(self) -> None:
        super().__init__('exploration_manager')
        self.declare_parameter('path_topic', '/frontier_selected_path')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('enable_navigation', False)
        self.declare_parameter('goal_repeat_tolerance_m', 0.20)
        self.declare_parameter('retry_period_sec', 1.0)

        path_topic = str(self.get_parameter('path_topic').value)
        navigate_action = str(self.get_parameter('navigate_action').value)
        retry_period = max(
            0.2, float(self.get_parameter('retry_period_sec').value)
        )

        self._navigate_client = ActionClient(
            self, NavigateToPose, navigate_action
        )
        self.create_subscription(Path, path_topic, self._on_path, 10)
        self.create_timer(retry_period, self._process_pending_goal)

        self._pending_goal = None
        self._active_goal = None
        self._last_finished_goal: tuple[float, float] | None = None
        self._awaiting_fresh_path = False
        self._active_path_notice_shown = False
        self._navigate_warning_shown = False
        self._disabled_notice_shown = False

        enabled = bool(self.get_parameter('enable_navigation').value)
        self.get_logger().info(
            f'Listening for validated frontier path on {path_topic}'
        )
        self.get_logger().info(f'NavigateToPose action: {navigate_action}')
        if enabled:
            self.get_logger().warning(
                'AUTONOMOUS NAVIGATION IS ENABLED: validated frontier goals '
                'may move the simulation robot'
            )
        else:
            self.get_logger().info(
                'Autonomous navigation is disabled; set '
                'enable_navigation:=true explicitly to allow NavigateToPose goals'
            )

    def _on_path(self, path: Path) -> None:
        if not path.poses:
            return

        # A path selected while the robot is moving may already be stale by the
        # time the active navigation finishes. Never queue it for immediate use.
        if self._active_goal is not None:
            if not self._active_path_notice_shown:
                self.get_logger().info(
                    'Ignoring frontier path updates while navigation is active; '
                    'a fresh path will be required after completion'
                )
                self._active_path_notice_shown = True
            return

        self._active_path_notice_shown = False
        pose = path.poses[-1]
        pose.header.stamp = self.get_clock().now().to_msg()

        if self._awaiting_fresh_path:
            self._awaiting_fresh_path = False
            self.get_logger().info(
                'Fresh frontier path received after navigation completion'
            )

        self._pending_goal = pose
        self._process_pending_goal()

    def _same_goal(
        self,
        first: tuple[float, float],
        second: tuple[float, float],
    ) -> bool:
        tolerance = max(
            0.0, float(self.get_parameter('goal_repeat_tolerance_m').value)
        )
        return hypot(first[0] - second[0], first[1] - second[1]) <= tolerance

    def _process_pending_goal(self) -> None:
        if self._pending_goal is None or self._active_goal is not None:
            return

        if not bool(self.get_parameter('enable_navigation').value):
            if not self._disabled_notice_shown:
                self.get_logger().info(
                    'Validated frontier is ready, but navigation remains disabled'
                )
                self._disabled_notice_shown = True
            return

        self._disabled_notice_shown = False
        pose = self._pending_goal
        goal_xy = (pose.pose.position.x, pose.pose.position.y)
        if (
            self._last_finished_goal is not None
            and self._same_goal(goal_xy, self._last_finished_goal)
        ):
            self._pending_goal = None
            return

        if not self._navigate_client.server_is_ready():
            if not self._navigate_warning_shown:
                navigate_action = str(
                    self.get_parameter('navigate_action').value
                )
                self.get_logger().warning(
                    f'Waiting for Nav2 NavigateToPose action {navigate_action}'
                )
                self._navigate_warning_shown = True
            return

        self._navigate_warning_shown = False
        self._pending_goal = None
        self._active_goal = goal_xy
        self._active_path_notice_shown = False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        if (
            goal_msg.pose.pose.orientation.x == 0.0
            and goal_msg.pose.pose.orientation.y == 0.0
            and goal_msg.pose.pose.orientation.z == 0.0
            and goal_msg.pose.pose.orientation.w == 0.0
        ):
            goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().warning(
            'Sending autonomous frontier goal: '
            f'x={goal_xy[0]:.2f}, y={goal_xy[1]:.2f}'
        )
        future = self._navigate_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'NavigateToPose goal request failed: {exc}')
            self._finish_navigation(False, 'request failed')
            return

        if not goal_handle.accepted:
            self.get_logger().warning('NavigateToPose rejected the frontier goal')
            self._finish_navigation(False, 'rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_result(self, future) -> None:
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'NavigateToPose result failed: {exc}')
            self._finish_navigation(False, 'result error')
            return

        succeeded = wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
        if succeeded:
            self._finish_navigation(True, 'SUCCEEDED')
        else:
            self._finish_navigation(
                False, f'status={wrapped_result.status}'
            )

    def _finish_navigation(self, succeeded: bool, detail: str) -> None:
        finished_goal = self._active_goal
        self._active_goal = None
        if finished_goal is not None:
            self._last_finished_goal = finished_goal

        # Anything received while the robot was moving is deliberately discarded.
        # The next navigation goal must come from a Path message published after
        # this completion callback, i.e. from a fresh frontier/planner update.
        self._pending_goal = None
        self._awaiting_fresh_path = True
        self._active_path_notice_shown = False

        if succeeded:
            self.get_logger().info(
                f'Frontier navigation completed: {detail}; '
                'waiting for a fresh frontier path'
            )
        else:
            self.get_logger().warning(
                f'Frontier navigation did not succeed: {detail}; '
                'waiting for a fresh frontier path'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
