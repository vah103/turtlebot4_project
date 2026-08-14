"""Retry-capable wrapper for the frontier detector's Nav2 planner checks."""

from math import hypot

import rclpy
from geometry_msgs.msg import PointStamped

from frontier_exploration.frontier_detector import FrontierDetector, _cell_to_world


class RetryFrontierDetector(FrontierDetector):
    """Keep frontier candidates alive while Nav2 planner is still starting."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('planner_retry_period_sec', 1.0)
        self.declare_parameter('planner_rejection_retry_limit', 15)
        self.declare_parameter('failed_goal_topic', '/frontier_failed_goal')
        self.declare_parameter('failed_goal_radius_m', 0.75)
        self.declare_parameter('failed_goal_cooldown_sec', 30.0)

        retry_period = max(
            0.1, float(self.get_parameter('planner_retry_period_sec').value)
        )
        failed_goal_topic = str(self.get_parameter('failed_goal_topic').value)

        self._planner_retry_counts: dict[tuple[int, int], int] = {}
        self._retry_not_before_sec = 0.0
        self._failed_goals: list[tuple[float, float, float]] = []
        self._planner_retry_timer = self.create_timer(
            retry_period, self._retry_pending_planner_check
        )
        self.create_subscription(
            PointStamped, failed_goal_topic, self._on_failed_goal, 10
        )

        self.get_logger().info(
            'Planner startup retry enabled: '
            f'period={retry_period:.2f}s, '
            f'limit={int(self.get_parameter("planner_rejection_retry_limit").value)}'
        )
        self.get_logger().info(
            f'Failed frontier blacklist listening on {failed_goal_topic}'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _prune_failed_goals(self) -> bool:
        now = self._now_sec()
        before = len(self._failed_goals)
        self._failed_goals = [
            item for item in self._failed_goals if item[2] > now
        ]
        return len(self._failed_goals) != before

    def _is_blacklisted_xy(self, x: float, y: float) -> bool:
        self._prune_failed_goals()
        radius = max(
            0.0, float(self.get_parameter('failed_goal_radius_m').value)
        )
        return any(
            hypot(x - failed_x, y - failed_y) <= radius
            for failed_x, failed_y, _ in self._failed_goals
        )

    def _on_failed_goal(self, msg: PointStamped) -> None:
        cooldown = max(
            0.0, float(self.get_parameter('failed_goal_cooldown_sec').value)
        )
        if cooldown <= 0.0:
            return

        x = float(msg.point.x)
        y = float(msg.point.y)
        expiry = self._now_sec() + cooldown
        self._failed_goals.append((x, y, expiry))
        self._prune_failed_goals()

        self.get_logger().warning(
            'Blacklisting failed frontier region: '
            f'x={x:.2f}, y={y:.2f}, cooldown={cooldown:.1f}s'
        )

        # Invalidate any selection or in-flight planner check based on the old
        # candidate set. A fresh ordering will skip the blacklisted region.
        self._generation += 1
        self._candidate_signature = None
        self._candidate_queue = []
        self._planning_candidate = None
        self._selected_index = None
        self._selected_distance = None
        self._selected_path = None

        latest_map = self._latest_map
        if latest_map is not None:
            self._publish_empty_path(latest_map.header.frame_id or 'map')
            super()._on_map(latest_map)

    def _retry_pending_planner_check(self) -> None:
        """Retry planner work and wake candidates when blacklist cooldown expires."""
        expired = self._prune_failed_goals()
        if (
            expired
            and self._selected_index is None
            and self._planning_candidate is None
            and self._latest_map is not None
        ):
            self.get_logger().info(
                'Failed-frontier cooldown expired; re-evaluating candidates'
            )
            self._candidate_signature = None
            super()._on_map(self._latest_map)
            return
        self._start_next_path_check()

    def _start_next_path_check(self) -> None:
        now = self._now_sec()
        if now < self._retry_not_before_sec:
            return

        skipped = 0
        if self._latest_map is not None:
            while self._candidate_queue:
                index, _ = self._candidate_queue[0]
                point = _cell_to_world(index, self._latest_map)
                if not self._is_blacklisted_xy(point.x, point.y):
                    break
                self._candidate_queue.pop(0)
                skipped += 1

        if skipped:
            self.get_logger().info(
                f'Skipped {skipped} candidate(s) inside failed-frontier blacklist'
            )

        super()._start_next_path_check()

    def _requeue_transient_planner_failure(
        self,
        candidate: tuple[int, int, float],
        reason: str,
    ) -> None:
        if self._planning_candidate == candidate:
            self._planning_candidate = None

        generation, index, distance = candidate
        if generation != self._generation:
            self._start_next_path_check()
            return

        retry_limit = max(
            0, int(self.get_parameter('planner_rejection_retry_limit').value)
        )
        key = (generation, index)
        retry_count = self._planner_retry_counts.get(key, 0) + 1

        if retry_count <= retry_limit:
            self._planner_retry_counts[key] = retry_count
            self._candidate_queue.insert(0, (index, distance))
            retry_period = max(
                0.1, float(self.get_parameter('planner_retry_period_sec').value)
            )
            self._retry_not_before_sec = self._now_sec() + retry_period
            self.get_logger().warning(
                f'{reason}; keeping candidate for retry '
                f'{retry_count}/{retry_limit} in about {retry_period:.1f}s'
            )
            self._publish_markers()
            return

        self._planner_retry_counts.pop(key, None)
        self._retry_not_before_sec = 0.0
        self.get_logger().warning(
            f'{reason}; retry limit reached, skipping this candidate'
        )
        self._publish_markers()
        self._start_next_path_check()

    def _on_plan_goal_response(
        self,
        future,
        candidate: tuple[int, int, float],
    ) -> None:
        if self._planning_candidate != candidate:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._requeue_transient_planner_failure(
                candidate,
                f'Planner goal request failed: {exc}',
            )
            return

        if not goal_handle.accepted:
            self._requeue_transient_planner_failure(
                candidate,
                'Nav2 planner rejected candidate while starting',
            )
            return

        self._retry_not_before_sec = 0.0
        self._planner_retry_counts.pop((candidate[0], candidate[1]), None)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda finished_future, expected=candidate:
            self._on_plan_result(finished_future, expected)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RetryFrontierDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
