"""Retry, settle, and temporary-suppression wrapper for the frontier detector."""

from math import hypot

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from frontier_exploration.frontier_completion import CompletionTracker
from frontier_exploration.frontier_core import candidate_goal_cells
from frontier_exploration.frontier_detector import (
    Candidate,
    FrontierDetector,
    PlanningCandidate,
    _cell_to_world,
)


class RetryFrontierDetector(FrontierDetector):
    """Keep candidates stable across planning and one active navigation goal."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('planner_retry_period_sec', 1.0)
        self.declare_parameter('planner_rejection_retry_limit', 15)
        self.declare_parameter('failed_goal_topic', '/frontier_failed_goal')
        self.declare_parameter('completed_goal_topic', '/frontier_completed_goal')
        self.declare_parameter('failed_goal_radius_m', 0.40)
        self.declare_parameter('failed_goal_cooldown_sec', 60.0)
        self.declare_parameter('completed_frontier_radius_m', 0.40)
        self.declare_parameter('completed_frontier_cooldown_sec', 15.0)
        self.declare_parameter('post_goal_settle_sec', 1.0)
        self.declare_parameter('completion_topic', '/exploration_complete')
        self.declare_parameter('completion_stable_cycles', 5)
        self.declare_parameter('completion_min_idle_sec', 10.0)
        self.declare_parameter('completion_check_period_sec', 2.0)

        retry_period = max(
            0.1, float(self.get_parameter('planner_retry_period_sec').value)
        )
        failed_goal_topic = str(self.get_parameter('failed_goal_topic').value)
        completed_goal_topic = str(
            self.get_parameter('completed_goal_topic').value
        )
        completion_topic = str(self.get_parameter('completion_topic').value)

        self._planner_retry_counts: dict[tuple[int, int], int] = {}
        self._retry_not_before_sec = 0.0
        self._replan_not_before_sec = 0.0
        self._failed_frontiers: list[tuple[float, float, float]] = []
        self._completed_frontiers: list[tuple[float, float, float]] = []
        self._deferred_map: OccupancyGrid | None = None
        self._exploration_complete = False
        self._completion_tracker = CompletionTracker(
            required_cycles=int(
                self.get_parameter('completion_stable_cycles').value
            ),
            min_idle_sec=float(
                self.get_parameter('completion_min_idle_sec').value
            ),
            check_period_sec=float(
                self.get_parameter('completion_check_period_sec').value
            ),
        )

        completion_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._completion_pub = self.create_publisher(
            Bool, completion_topic, completion_qos
        )

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
        self.get_logger().info(
            'Completion policy: no safe reachable frontier for '
            f'{self._completion_tracker.required_cycles} checks and at least '
            f'{self._completion_tracker.min_idle_sec:.1f}s; '
            f'publishing on {completion_topic}'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _prune_regions(
        self,
        regions: list[tuple[float, float, float]],
    ) -> tuple[list[tuple[float, float, float]], bool]:
        now = self._now_sec()
        filtered = [item for item in regions if item[2] > now]
        return filtered, len(filtered) != len(regions)

    def _prune_suppression_regions(self) -> bool:
        self._failed_frontiers, failed_changed = self._prune_regions(
            self._failed_frontiers
        )
        self._completed_frontiers, completed_changed = self._prune_regions(
            self._completed_frontiers
        )
        return failed_changed or completed_changed

    def _selected_frontier_xy(self) -> tuple[float, float] | None:
        if self._latest_map is None or self._selected_index is None:
            return None
        point = _cell_to_world(self._selected_index, self._latest_map)
        return point.x, point.y

    def _candidate_in_regions(
        self,
        candidate: Candidate,
        regions: list[tuple[float, float, float]],
        radius: float,
    ) -> bool:
        if self._latest_map is None:
            return False
        representative = candidate[0]
        point = _cell_to_world(representative, self._latest_map)
        return any(
            hypot(point.x - x, point.y - y) <= radius
            for x, y, _ in regions
        )

    def _is_temporarily_suppressed_candidate(self, candidate: Candidate) -> bool:
        self._prune_suppression_regions()
        failed_radius = max(
            0.0, float(self.get_parameter('failed_goal_radius_m').value)
        )
        completed_radius = max(
            0.0,
            float(self.get_parameter('completed_frontier_radius_m').value),
        )
        return (
            self._candidate_in_regions(
                candidate,
                self._failed_frontiers,
                failed_radius,
            )
            or self._candidate_in_regions(
                candidate,
                self._completed_frontiers,
                completed_radius,
            )
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        """Freeze candidate indices against one map snapshot until goal outcome."""
        if self._exploration_complete:
            return
        if self._selected_index is not None or self._planning_candidate is not None:
            self._deferred_map = msg
            return

        # The occupancy grid may resize or shift origin while SLAM grows. Rebuild
        # candidates from each accepted idle snapshot instead of trusting raw grid
        # indices to keep the same world meaning across map geometry changes.
        self._candidate_signature = None
        super()._on_map(msg)

    def _goal_index_for_segment(
        self,
        segment: list[int],
        msg: OccupancyGrid,
        robot_x: float,
        robot_y: float,
        min_distance_m: float,
    ) -> tuple[int, float] | None:
        """Choose a costmap-safe free goal near the frontier centroid.

        Prefer a free goal outside the minimum-distance radius. If the frontier
        itself is eligible but its inward free-side goal lies just inside that
        radius, fall back to the best safe free neighbor rather than discarding
        the whole frontier.
        """
        source_frame = msg.header.frame_id or 'map'
        centroid_x, centroid_y = self._segment_centroid_world(segment, msg)
        best_any: tuple[float, int, float] | None = None
        best_far: tuple[float, int, float] | None = None

        for index in candidate_goal_cells(
            segment,
            self._latest_frontier_cells,
            list(msg.data),
            msg.info.width,
            msg.info.height,
        ):
            point = _cell_to_world(index, msg)
            goal_distance = hypot(point.x - robot_x, point.y - robot_y)

            if self._global_costmap is not None and self._point_blocked(
                self._global_costmap,
                point.x,
                point.y,
                source_frame,
                outside_is_blocked=True,
            ):
                continue

            if self._local_costmap is not None and self._point_blocked(
                self._local_costmap,
                point.x,
                point.y,
                source_frame,
                outside_is_blocked=False,
            ):
                continue

            centroid_distance_sq = (
                (point.x - centroid_x) ** 2 + (point.y - centroid_y) ** 2
            )
            score = (centroid_distance_sq, index, goal_distance)
            if best_any is None or score < best_any:
                best_any = score
            if goal_distance >= min_distance_m and (
                best_far is None or score < best_far
            ):
                best_far = score

        best = best_far if best_far is not None else best_any
        if best is None:
            return None
        return best[1], best[2]

    def _reset_selection_and_replan(
        self,
        reason: str,
        *,
        settle_sec: float = 0.0,
    ) -> None:
        """Invalidate cached selection and rebuild from the newest map snapshot."""
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

        latest_map = self._deferred_map or self._latest_map
        self._deferred_map = None
        if latest_map is None:
            return

        self.get_logger().info(reason)
        self._publish_empty_path(latest_map.header.frame_id or 'map')
        super()._on_map(latest_map)

    def _on_completed_goal(self, msg: PointStamped) -> None:
        frontier_xy = self._selected_frontier_xy()
        if frontier_xy is not None:
            cooldown = max(
                0.0,
                float(
                    self.get_parameter('completed_frontier_cooldown_sec').value
                ),
            )
            if cooldown > 0.0:
                self._completed_frontiers.append(
                    (frontier_xy[0], frontier_xy[1], self._now_sec() + cooldown)
                )

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
        self._failed_frontiers.append(
            (frontier_xy[0], frontier_xy[1], self._now_sec() + cooldown)
        )
        self._prune_suppression_regions()

        self.get_logger().warning(
            'Temporarily suppressing failed frontier region: '
            f'x={frontier_xy[0]:.2f}, y={frontier_xy[1]:.2f}, '
            f'cooldown={cooldown:.1f}s'
        )
        self._reset_selection_and_replan(
            'Failed frontier feedback received; selecting another safe candidate'
        )

    def _retry_pending_planner_check(self) -> None:
        if self._exploration_complete:
            return
        suppression_expired = self._prune_suppression_regions()

        # If every candidate from the frozen snapshot was exhausted, consume the
        # newest deferred map instead of staying idle on obsolete candidates.
        if (
            self._selected_index is None
            and self._planning_candidate is None
            and not self._candidate_queue
            and self._deferred_map is not None
        ):
            latest_map = self._deferred_map
            self._deferred_map = None
            self._candidate_signature = None
            super()._on_map(latest_map)
            return

        if (
            suppression_expired
            and self._selected_index is None
            and self._planning_candidate is None
            and self._latest_map is not None
        ):
            self.get_logger().info(
                'Temporary frontier suppression expired; rebuilding candidates'
            )
            self._candidate_signature = None
            super()._on_map(self._latest_map)
            return

        self._start_next_path_check()

    def _start_next_path_check(self) -> None:
        if self._exploration_complete:
            return
        now = self._now_sec()
        if now < self._replan_not_before_sec or now < self._retry_not_before_sec:
            return

        skipped = 0
        while self._candidate_queue and self._is_temporarily_suppressed_candidate(
            self._candidate_queue[0]
        ):
            self._candidate_queue.pop(0)
            skipped += 1

        if skipped:
            self.get_logger().info(
                f'Skipped {skipped} temporarily suppressed frontier candidate(s)'
            )

        super()._start_next_path_check()
        self._check_exploration_completion()

    def _check_exploration_completion(self) -> None:
        """Finish after the planner repeatedly exhausts all usable frontiers."""
        if self._exploration_complete or self._latest_map is None:
            return
        if (
            self._selected_index is not None
            or self._planning_candidate is not None
            or self._candidate_queue
            or self._deferred_map is not None
        ):
            return
        if (
            bool(self.get_parameter('require_global_costmap').value)
            and self._global_costmap is None
        ):
            return

        now = self._now_sec()
        previous_streak = self._completion_tracker.streak
        newly_complete = self._completion_tracker.observe_exhausted(now)
        if self._completion_tracker.streak != previous_streak:
            self.get_logger().info(
                'No safe reachable frontier remains: '
                f'check {self._completion_tracker.streak}/'
                f'{self._completion_tracker.required_cycles}, '
                f'idle={self._completion_tracker.idle_elapsed_sec(now):.1f}s'
            )
        if not newly_complete:
            return

        self._exploration_complete = True
        self._publish_empty_path(self._latest_map.header.frame_id or 'map')
        completion = Bool()
        completion.data = True
        self._completion_pub.publish(completion)
        self.get_logger().info(
            'Exploration complete: no safe reachable frontier remained after '
            'stable planner checks'
        )

    def _on_plan_result(self, future, candidate: PlanningCandidate) -> None:
        super()._on_plan_result(future, candidate)
        if self._selected_index is not None:
            self._completion_tracker.reset()

    def _requeue_transient_planner_failure(
        self,
        candidate: PlanningCandidate,
        reason: str,
    ) -> None:
        if self._planning_candidate == candidate:
            self._planning_candidate = None

        (
            generation,
            representative,
            goal_index,
            frontier_distance,
            goal_distance,
        ) = candidate
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
