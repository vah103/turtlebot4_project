"""Retry-capable wrapper for the frontier detector's Nav2 planner checks."""

import rclpy

from frontier_exploration.frontier_detector import FrontierDetector


class RetryFrontierDetector(FrontierDetector):
    """Keep frontier candidates alive while Nav2 planner is still starting."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('planner_retry_period_sec', 1.0)
        self.declare_parameter('planner_rejection_retry_limit', 15)

        retry_period = max(
            0.1, float(self.get_parameter('planner_retry_period_sec').value)
        )
        self._planner_retry_counts: dict[tuple[int, int], int] = {}
        self._planner_retry_timer = self.create_timer(
            retry_period, self._retry_pending_planner_check
        )

        self.get_logger().info(
            'Planner startup retry enabled: '
            f'period={retry_period:.2f}s, '
            f'limit={int(self.get_parameter("planner_rejection_retry_limit").value)}'
        )

    def _retry_pending_planner_check(self) -> None:
        """Periodically retry queued candidates once the planner becomes usable."""
        self._start_next_path_check()

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
            self.get_logger().warning(
                f'{reason}; keeping candidate for retry '
                f'{retry_count}/{retry_limit} in about {retry_period:.1f}s'
            )
            self._publish_markers()
            return

        self._planner_retry_counts.pop(key, None)
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
