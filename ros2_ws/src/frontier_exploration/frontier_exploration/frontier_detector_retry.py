"""Retry, settle, and temporary-suppression wrapper for the frontier detector."""

from math import hypot

import rclpy
from geometry_msgs.msg import PointStamped

from frontier_exploration.frontier_detector import (
    Candidate,
    FrontierDetector,
    PlanningCandidate,
    _cell_to_world,
)


class RetryFrontierDetector(FrontierDetector):
    """Keep candidates alive across Nav2 startup and navigation outcomes."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('planner_retry_period_sec', 1.0)
        self.declare_parameter('planner_rejection_retry_limit', 15)
        self.declare_parameter('failed_goal_topic', '/frontier_failed_goal')
        self.declare_parameter('completed_goal_topic', '/frontier_completed_goal')
        self.declare_parameter('failed_goal_radius_m', 0.40)
        self.declare_parameter('failed_goal_cooldown_sec', 60.0)
        self.declare_parameter('post_goal_settle_sec', 1.0)

        retry_period = max(
            0.1, float(self.get_parameter('planner_retry_period_sec').value)
        )
        failed_goal_topic = str(self.get_parameter('failed_goal_topic').value)
        completed_goal_topic = str(
            self.get_parameter('completed_goal_topic').value
        )

        self._planner_retry_counts: dict[tuple[int, int], int] = {}
        self._retry_not_before_sec = 0.0
        self._replan_not_before_sec = 0.0
        self._failed_frontiers: list[tuple[float, float, float]] = []
        self._planner_retry_timer = self.create_timer(
            retry_period, self._retry_pending_planner_check
        )
        self.create_subscription(
            PointStamped, failed_goal_topic, self._on_failed_goal, 10
        )
        self.create_subscription(
            PointStamped, completed_goal_topic, self._on_completed_goal, 10
        )

        self.get_logger().info(
            'Planner startup retry enabled: '
            f'period={retry_period:.2f}s, '
            f'limit={int(self.get_parameter("planner_rejection_retry_limit").value)}'
        )
        self.get_logger().info(
            f'Failed frontier suppression listening on {failed_goal_topic}'
        )
        self.get_logger().info(
            f'Completed frontier refresh listening on {completed_goal_topic}'
        )
        self.get_logger().info(
            'Post-goal map settle: '
            f'{float(self.get_parameter("post_goal_settle_sec").value):.2f}s'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _prune_failed_frontiers(self) -> bool:
        now = self._now_sec()
        before = len(self._failed_frontiers)
        self._failed_frontiers = [
            item for item in self._failed_frontiers if item[2] > now
        ]
        return len(self._failed_frontiers) != before

    def _selected_frontier_xy(self) -> tuple[float, float] | None:
        if self._latest_map is None or self._selected_index is None:
            return None
        point = _cell_to_world(self._selected_index, self._latest_map)
        return point.x, point.y

    def _is_blacklisted_candidate(self, candidate: Candidate) -> bool:
        self._prune_failed_frontiers()
        if self._latest_map is None:
            return False
        representative = candidate[0]
        point = _cell_to_world(representative, self._latest_map)
        radius = max(
            0.0, float(self.get_parameter('failed_goal_radius_m').value)
        )
        return any(
            hypot(point.x - failed_x, point.y - failed_y) <= radius
            for failed_x, failed_y, _ in self._failed_frontiers
        )

    def _reset_selection_and_replan(
        self,
        reason: str,
        *,
        settle_sec: float = 0.0,
    ) -> None:
        """Invalidate cached selection and rebuild candidates from the latest map."""
        self._generation += 1
        self._candidate_signature = None
        self._candidate_queue = []
        self._planning_candidate = None
        self._selected_index = None
        self._selected_goal_index = None
        self._selected_distance = None
        self._selected_path = None
        self._planner_retry_counts.clear()
        self._retry_not_before_sec = 0.0
        self._replan_not_before_sec = self._now_sec() + max(0.0, settle_sec)

        latest_map = self._latest_map
        if latest_map is None:
            return

        self.get_logger().info(reason)
        self._publish_empty_path(latest_map.header.frame_id or 'map')
        super()._on_map(latest_map)

    def _on_completed_goal(self, msg: PointStamped) -> None:
        settle_sec = max(
            0.0, float(self.get_parameter('post_goal_settle_sec').value)
        )
        self._reset_selection_and_replan(
            'Completed frontier reached; rebuilding WFD candidates after '
            f'{settle_sec:.2f}s map settle at '
            f'x={float(msg.point.x):.2f}, y={float(msg.point.y):.2f}',
            settle_sec=settle_sec,
        )

    def _on_failed_goal(self, msg: PointStamped) -> None:
        cooldown = max(
            0.0, float(self.get_parameter('failed_goal_cooldown_sec').value)
        )
        if cooldown <= 0.0:
            return

        frontier_xy = self._selected_frontier_xy()
        if frontier_xy is None:
            frontier_xy = (float(msg.point.x), float(msg.point.y))
        expiry = self._now_sec() + cooldown
        self._failed_frontiers.append(
            (frontier_xy[0], frontier_xy[1], expiry)
        )
        self._prune_failed_frontiers()

        self.get_logger().warning(
            'Temporarily suppressing failed frontier region: '
            f'x={frontier_xy[0]:.2f}, y={frontier_xy[1]:.2f}, '
            f'cooldown={cooldown:.1f}s'
        )
        self._reset_selection_and_replan(
            'Failed frontier feedback received; selecting another safe candidate'
        )

    def _retry_pending_planner_check(self) -> None:
        expired = self._prune_failed_frontiers()
        if (
            expired
            and self._selected_index is None
            and self._planning_candidate is None
            and self._latest_map is not None
        ):
            self.get_logger().info(
                'Failed-frontier cooldown expired; rebuilding candidates'
            )
            self._candidate_signature = None
            super()._on_map(self._latest_map)
            return
        self._start_next_path_check()

    def _start_next_path_check(self) -> None:
        now = self._now_sec()
        if now < self._replan_not_before_sec or now < self._retry_not_before_sec:
            return

        skipped = 0
        while self._candidate_queue and self._is_blacklisted_candidate(
            self._candidate_queue[0]
        ):
            self._candidate_queue.pop(0)
            skipped += 1

        if skipped:
            self.get_logger().info(
                f'Skipped {skipped} candidate(s) in temporary failed-frontier regions'
            )

        super()._start_next_path_check()

    def _requeue_transient_planner_failure(
        self,
        candidate: PlanningCandidate,
        reason: str,
    ) -> None:
        if self._planning_candidate == candidate:
            self._planning_candidate = None

        generation, representative, goal_index, frontier_distance, goal_distance = candidate
        if generation != self._generation:
            self._start_next_path_check()
            return

        retry_limit = max(
            0, int(self.get_parameter('planner_rejection_retry_limit').value)
        )
        key = (generation, representative)
        retry_count = self._planner_retry_counts.get(key, 0) + 1

        if retry_count <= retry_limit:
            self._planner_retry_counts[key] = retry_count
            self._candidate_queue.insert(
                0,
                (
                    representative,
                    goal_index,
                    frontier_distance,
                    goal_distance,
                ),
            )
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
        candidate: PlanningCandidate,
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
