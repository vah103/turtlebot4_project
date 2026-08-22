"""Nav2 execution manager for the frontier exploration baseline.

The manager executes planner-validated paths only when navigation is explicitly
enabled. Frontier selection, safety filtering, and failed-region suppression stay
inside the detector so navigation state and exploration policy remain separated.
"""

from math import isfinite

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node


class ExplorationManager(Node):
    """Execute one validated frontier goal at a time through Nav2."""

    def __init__(self) -> None:
        super().__init__('exploration_manager')
        self.declare_parameter('path_topic', '/frontier_selected_path')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('enable_navigation', False)
        self.declare_parameter('retry_period_sec', 1.0)
        self.declare_parameter('failed_goal_topic', '/frontier_failed_goal')
        self.declare_parameter('completed_goal_topic', '/frontier_completed_goal')
        self.declare_parameter('stall_timeout_sec', 30.0)
        self.declare_parameter('stall_progress_epsilon_m', 0.02)
        self.declare_parameter('navigation_timeout_sec', 180.0)

        path_topic = str(self.get_parameter('path_topic').value)
        navigate_action = str(self.get_parameter('navigate_action').value)
        failed_goal_topic = str(self.get_parameter('failed_goal_topic').value)
        completed_goal_topic = str(
            self.get_parameter('completed_goal_topic').value
        )
        retry_period = max(
            0.2, float(self.get_parameter('retry_period_sec').value)
        )

        self._navigate_client = ActionClient(
            self, NavigateToPose, navigate_action
        )
        self._failed_goal_pub = self.create_publisher(
            PointStamped, failed_goal_topic, 10
        )
        self._completed_goal_pub = self.create_publisher(
            PointStamped, completed_goal_topic, 10
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
        self._awaiting_fresh_path = False
        self._active_path_notice_shown = False
        self._navigate_warning_shown = False
        self._disabled_notice_shown = False

        self.get_logger().info(
            f'Listening for validated frontier path on {path_topic}'
        )
        self.get_logger().info(f'NavigateToPose action: {navigate_action}')
        self.get_logger().info(
            f'Failed frontier feedback: {failed_goal_topic}'
        )
        self.get_logger().info(
            f'Completed frontier feedback: {completed_goal_topic}'
        )
        self.get_logger().info(
            'Navigation progress watchdog: '
            f'timeout={float(self.get_parameter("stall_timeout_sec").value):.1f}s, '
            f'epsilon={float(self.get_parameter("stall_progress_epsilon_m").value):.2f}m'
        )

        if bool(self.get_parameter('enable_navigation').value):
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

    def _on_path(self, path: Path) -> None:
        if not path.poses:
            return

        # Replanning while moving can produce a path that is stale by the time the
        # active Nav2 goal finishes. Require a detector refresh after completion.
        if self._active_goal is not None:
            if not self._active_path_notice_shown:
                self.get_logger().info(
                    'Ignoring frontier path updates while navigation is active; '
                    'a fresh path will be required after completion'
                )
                self._active_path_notice_shown = True
            return

        pose = path.poses[-1]
        # Use TF's "latest available transform" convention for the navigation
        # goal. Stamping the goal with the current simulated time can put it a few
        # milliseconds ahead of slam_toolbox's map->odom transform and Nav2 then
        # rejects an otherwise valid goal with future-extrapolation errors.
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        if self._awaiting_fresh_path:
            self._awaiting_fresh_path = False
            self.get_logger().info(
                'Fresh frontier path received after navigation completion'
            )

        self._active_path_notice_shown = False
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

        if not self._navigate_client.server_is_ready():
            if not self._navigate_warning_shown:
                action_name = str(self.get_parameter('navigate_action').value)
                self.get_logger().warning(
                    f'Waiting for Nav2 NavigateToPose action {action_name}'
                )
                self._navigate_warning_shown = True
            return

        self._disabled_notice_shown = False
        self._navigate_warning_shown = False
        pose = self._pending_goal
        self._pending_goal = None
        goal_xy = (pose.pose.position.x, pose.pose.position.y)
        self._active_goal = goal_xy
        self._active_goal_frame = pose.header.frame_id or 'map'
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
        if self._best_distance_remaining is None:
            self._best_distance_remaining = distance
            self._last_progress_sec = self._now_sec()
            return

        if distance < self._best_distance_remaining - epsilon:
            self._best_distance_remaining = distance
            self._last_progress_sec = self._now_sec()

    def _watch_navigation(self) -> None:
        if (
            self._active_goal is None
            or self._cancel_requested
            or self._goal_handle is None
        ):
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
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_response)

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

        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self._finish_navigation(True, 'SUCCEEDED')
            return

        detail = self._cancel_reason or f'status={wrapped_result.status}'
        self._finish_navigation(False, detail)

    def _publish_goal_event(
        self,
        publisher,
        goal_xy: tuple[float, float],
        frame_id: str,
    ) -> None:
        msg = PointStamped()
        msg.header.frame_id = frame_id or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = goal_xy[0]
        msg.point.y = goal_xy[1]
        msg.point.z = 0.0
        publisher.publish(msg)

    def _finish_navigation(self, succeeded: bool, detail: str) -> None:
        finished_goal = self._active_goal
        finished_frame = self._active_goal_frame

        self._active_goal = None
        self._goal_handle = None
        self._goal_started_sec = None
        self._last_progress_sec = None
        self._best_distance_remaining = None
        self._cancel_requested = False
        self._cancel_reason = None
        self._pending_goal = None
        self._awaiting_fresh_path = True
        self._active_path_notice_shown = False

        # Publish only after clearing active state so the detector can refresh and
        # the resulting fresh path can be accepted immediately.
        if finished_goal is not None:
            if succeeded:
                self._publish_goal_event(
                    self._completed_goal_pub,
                    finished_goal,
                    finished_frame,
                )
            else:
                self._publish_goal_event(
                    self._failed_goal_pub,
                    finished_goal,
                    finished_frame,
                )

        if succeeded:
            self.get_logger().info(
                f'Frontier navigation completed: {detail}; '
                'waiting for detector settle and a fresh frontier path'
            )
        else:
            self.get_logger().warning(
                f'Frontier navigation did not succeed: {detail}; '
                'detector will suppress this frontier temporarily'
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
