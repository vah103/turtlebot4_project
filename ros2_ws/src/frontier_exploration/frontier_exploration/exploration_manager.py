"""Optional Nav2 execution manager for the frontier exploration baseline.

Navigation is disabled by default. When explicitly enabled, the manager consumes
planner-validated paths from the frontier detector and sends their final pose to
Nav2 NavigateToPose. It never publishes cmd_vel directly.
"""

from math import hypot, isfinite

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
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
        self.declare_parameter('failed_goal_topic', '/frontier_failed_goal')
        self.declare_parameter('failed_goal_radius_m', 0.75)
        self.declare_parameter('failed_goal_cooldown_sec', 30.0)
        self.declare_parameter('stall_timeout_sec', 15.0)
        self.declare_parameter('stall_progress_epsilon_m', 0.10)
        self.declare_parameter('navigation_timeout_sec', 90.0)

        path_topic = str(self.get_parameter('path_topic').value)
        navigate_action = str(self.get_parameter('navigate_action').value)
        failed_goal_topic = str(self.get_parameter('failed_goal_topic').value)
        retry_period = max(
            0.2, float(self.get_parameter('retry_period_sec').value)
        )

        self._navigate_client = ActionClient(
            self, NavigateToPose, navigate_action
        )
        self._failed_goal_pub = self.create_publisher(
            PointStamped, failed_goal_topic, 10
        )
        self.create_subscription(Path, path_topic, self._on_path, 10)
        self.create_timer(retry_period, self._process_pending_goal)
        self.create_timer(1.0, self._watch_navigation)

        self._pending_goal = None
        self._active_goal: tuple[float, float] | None = None
        self._active_goal_frame = 'map'
        self._goal_handle = None
        self._goal_started_sec: float | None = None
        self._last_progress_sec: float | None = None
        self._best_distance_remaining: float | None = None
        self._cancel_requested = False
        self._cancel_reason: str | None = None

        self._last_finished_goal: tuple[float, float] | None = None
        self._recent_failed_goal: tuple[float, float, float] | None = None
        self._awaiting_fresh_path = False
        self._active_path_notice_shown = False
        self._failed_path_notice_shown = False
        self._navigate_warning_shown = False
        self._disabled_notice_shown = False

        enabled = bool(self.get_parameter('enable_navigation').value)
        self.get_logger().info(
            f'Listening for validated frontier path on {path_topic}'
        )
        self.get_logger().info(f'NavigateToPose action: {navigate_action}')
        self.get_logger().info(
            f'Failed frontier feedback: {failed_goal_topic}'
        )
        self.get_logger().info(
            'Navigation watchdog: '
            f'stall={float(self.get_parameter("stall_timeout_sec").value):.1f}s, '
            f'hard_timeout={float(self.get_parameter("navigation_timeout_sec").value):.1f}s'
        )
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

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _same_goal(
        self,
        first: tuple[float, float],
        second: tuple[float, float],
        tolerance: float | None = None,
    ) -> bool:
        if tolerance is None:
            tolerance = max(
                0.0, float(self.get_parameter('goal_repeat_tolerance_m').value)
            )
        return hypot(first[0] - second[0], first[1] - second[1]) <= tolerance

    def _is_recent_failed_goal(self, goal_xy: tuple[float, float]) -> bool:
        failed = self._recent_failed_goal
        if failed is None:
            return False
        if failed[2] <= self._now_sec():
            self._recent_failed_goal = None
            self._failed_path_notice_shown = False
            return False

        radius = max(
            0.0, float(self.get_parameter('failed_goal_radius_m').value)
        )
        return self._same_goal(goal_xy, (failed[0], failed[1]), radius)

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

        pose = path.poses[-1]
        goal_xy = (pose.pose.position.x, pose.pose.position.y)
        if self._is_recent_failed_goal(goal_xy):
            if not self._failed_path_notice_shown:
                self.get_logger().info(
                    'Ignoring path inside recent failed-frontier region; '
                    'waiting for detector to choose another candidate'
                )
                self._failed_path_notice_shown = True
            return

        self._failed_path_notice_shown = False
        self._active_path_notice_shown = False
        pose.header.stamp = self.get_clock().now().to_msg()

        if self._awaiting_fresh_path:
            self._awaiting_fresh_path = False
            self.get_logger().info(
                'Fresh frontier path received after navigation completion'
            )

        self._pending_goal = pose
        self._process_pending_goal()

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
        if self._is_recent_failed_goal(goal_xy):
            self._pending_goal = None
            return

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
        self._active_goal_frame = pose.header.frame_id or 'map'
        self._active_path_notice_shown = False
        self._goal_handle = None
        now = self._now_sec()
        self._goal_started_sec = now
        self._last_progress_sec = now
        self._best_distance_remaining = None
        self._cancel_requested = False
        self._cancel_reason = None

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
        future = self._navigate_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_navigation_feedback,
        )
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

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigation_result)

    def _on_navigation_feedback(self, feedback_msg) -> None:
        if self._active_goal is None or self._cancel_requested:
            return

        distance = float(feedback_msg.feedback.distance_remaining)
        if not isfinite(distance):
            return

        epsilon = max(
            0.0, float(self.get_parameter('stall_progress_epsilon_m').value)
        )
        if (
            self._best_distance_remaining is None
            or distance <= self._best_distance_remaining - epsilon
        ):
            self._best_distance_remaining = distance
            self._last_progress_sec = self._now_sec()

    def _watch_navigation(self) -> None:
        if self._active_goal is None or self._cancel_requested:
            return
        if self._goal_handle is None:
            return

        now = self._now_sec()
        stall_timeout = max(
            0.0, float(self.get_parameter('stall_timeout_sec').value)
        )
        navigation_timeout = max(
            0.0, float(self.get_parameter('navigation_timeout_sec').value)
        )

        reason = None
        if (
            navigation_timeout > 0.0
            and self._goal_started_sec is not None
            and now - self._goal_started_sec >= navigation_timeout
        ):
            reason = f'navigation timeout after {navigation_timeout:.1f}s'
        elif (
            stall_timeout > 0.0
            and self._last_progress_sec is not None
            and now - self._last_progress_sec >= stall_timeout
        ):
            reason = f'no meaningful progress for {stall_timeout:.1f}s'

        if reason is None:
            return

        self._cancel_requested = True
        self._cancel_reason = reason
        self.get_logger().warning(
            f'Navigation watchdog triggered: {reason}; canceling frontier goal'
        )
        cancel_future = self._goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to request goal cancellation: {exc}')
            self._cancel_requested = False
            self._last_progress_sec = self._now_sec()
            return

        if not response.goals_canceling:
            self.get_logger().warning(
                'Nav2 did not accept watchdog cancellation request'
            )
            self._cancel_requested = False
            self._last_progress_sec = self._now_sec()
            return

        self.get_logger().info(
            'Nav2 accepted watchdog cancellation; waiting for result'
        )

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
            return

        detail = self._cancel_reason or f'status={wrapped_result.status}'
        self._finish_navigation(False, detail)

    def _publish_failed_goal(
        self,
        goal_xy: tuple[float, float],
        frame_id: str,
        detail: str,
    ) -> None:
        cooldown = max(
            0.0, float(self.get_parameter('failed_goal_cooldown_sec').value)
        )
        if cooldown <= 0.0:
            return

        msg = PointStamped()
        msg.header.frame_id = frame_id or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = goal_xy[0]
        msg.point.y = goal_xy[1]
        msg.point.z = 0.0
        self._failed_goal_pub.publish(msg)

        self._recent_failed_goal = (
            goal_xy[0],
            goal_xy[1],
            self._now_sec() + cooldown,
        )
        self._failed_path_notice_shown = False
        self.get_logger().warning(
            'Reported failed frontier for temporary blacklist: '
            f'x={goal_xy[0]:.2f}, y={goal_xy[1]:.2f}, '
            f'cooldown={cooldown:.1f}s ({detail})'
        )

    def _finish_navigation(self, succeeded: bool, detail: str) -> None:
        finished_goal = self._active_goal
        finished_frame = self._active_goal_frame

        if succeeded and finished_goal is not None:
            self._last_finished_goal = finished_goal
        elif not succeeded and finished_goal is not None:
            self._publish_failed_goal(finished_goal, finished_frame, detail)

        self._active_goal = None
        self._goal_handle = None
        self._goal_started_sec = None
        self._last_progress_sec = None
        self._best_distance_remaining = None
        self._cancel_requested = False
        self._cancel_reason = None

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
                'failed region blacklisted temporarily'
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
