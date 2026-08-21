"""Strict frontier completion semantics for the exploration baseline.

A navigation failure is only temporary evidence. A frontier becomes ignored only
when every costmap-safe free goal around that frontier segment has been checked by
ComputePathToPose and none yields a path, or after repeated costmap-blocked checks
stay unresolved across a retry window. Confirmed-unreachable evidence is
revalidated after successful exploration because SLAM and costmaps may have changed
enough to make an old conclusion stale.
"""

from math import hypot

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray

from frontier_exploration.frontier_completion import (
    AdaptiveFailureCooldownTracker,
    ConfirmedUnreachableTracker,
    DeferredRegionTracker,
)
from frontier_exploration.frontier_core import (
    candidate_goal_cells,
    candidate_goal_cells_with_standoff,
    representative_cell,
)
from frontier_exploration.frontier_detector import (
    Candidate,
    PlanningCandidate,
    _cell_to_world,
    path_length,
)
from frontier_exploration.frontier_detector_retry import RetryFrontierDetector


class StrictCompletionFrontierDetector(RetryFrontierDetector):
    """Explore until every remaining frontier is planner-confirmed unreachable."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('unreachable_frontier_radius_m', 0.50)
        self.declare_parameter('frontier_goal_standoff_m', 0.30)
        self.declare_parameter('costmap_blocked_retry_period_sec', 10.0)
        self.declare_parameter('costmap_blocked_max_checks', 4)
        self.declare_parameter('failed_goal_cooldown_multiplier', 2.0)
        self.declare_parameter('failed_goal_cooldown_max_sec', 240.0)
        self.declare_parameter('failed_goal_repeat_window_sec', 600.0)

        unreachable_radius = float(
            self.get_parameter('unreachable_frontier_radius_m').value
        )
        self._unreachable_tracker = ConfirmedUnreachableTracker(
            radius_m=unreachable_radius
        )
        self._costmap_deferred_tracker = DeferredRegionTracker(
            radius_m=unreachable_radius,
            max_checks=int(
                self.get_parameter('costmap_blocked_max_checks').value
            ),
            retry_period_sec=float(
                self.get_parameter('costmap_blocked_retry_period_sec').value
            ),
        )
        self._failure_cooldown_tracker = AdaptiveFailureCooldownTracker(
            radius_m=float(self.get_parameter('failed_goal_radius_m').value),
            base_cooldown_sec=float(
                self.get_parameter('failed_goal_cooldown_sec').value
            ),
            max_cooldown_sec=float(
                self.get_parameter('failed_goal_cooldown_max_sec').value
            ),
            multiplier=float(
                self.get_parameter('failed_goal_cooldown_multiplier').value
            ),
            repeat_window_sec=float(
                self.get_parameter('failed_goal_repeat_window_sec').value
            ),
        )
        self.get_logger().info(
            'Strict completion enabled: navigation failure is temporary; '
            'a frontier is ignored only after all safe free goals fail '
            'ComputePathToPose'
        )
        self.get_logger().info(
            'Frontier navigation goal standoff: '
            f'{float(self.get_parameter("frontier_goal_standoff_m").value):.2f} m '
            'into known free space before adjacent-cell fallback'
        )
        self.get_logger().info(
            'Costmap-blocked frontier retry: '
            f'{self._costmap_deferred_tracker.max_checks} checks spaced by '
            f'{self._costmap_deferred_tracker.retry_period_sec:.1f}s before '
            'confirming unreachable'
        )
        self.get_logger().info(
            'Navigation-failure region cooldown: '
            f'radius={self._failure_cooldown_tracker.radius_m:.2f}m, '
            f'base={self._failure_cooldown_tracker.base_cooldown_sec:.1f}s, '
            f'max={self._failure_cooldown_tracker.max_cooldown_sec:.1f}s, '
            f'multiplier={self._failure_cooldown_tracker.multiplier:.1f}x'
        )

    def _frontier_is_confirmed_unreachable(
        self,
        representative: int,
        msg: OccupancyGrid,
    ) -> bool:
        point = _cell_to_world(representative, msg)
        return self._unreachable_tracker.contains(point.x, point.y)

    def _mark_confirmed_unreachable(
        self,
        representative: int,
        msg: OccupancyGrid,
        reason: str,
    ) -> None:
        point = _cell_to_world(representative, msg)
        self._costmap_deferred_tracker.clear_near(point.x, point.y)
        if self._unreachable_tracker.mark(point.x, point.y):
            self.get_logger().warning(
                'Confirmed unreachable frontier; ignoring this region until '
                'successful exploration triggers revalidation: '
                f'x={point.x:.2f}, y={point.y:.2f}; {reason}'
            )

    def _goal_candidates_for_segment(
        self,
        segment: list[int],
        msg: OccupancyGrid,
    ) -> tuple[list[int], set[int]]:
        """Return standoff goals first, followed by adjacent free-side fallback."""
        data = list(msg.data)
        adjacent = candidate_goal_cells(
            segment,
            self._latest_frontier_cells,
            data,
            msg.info.width,
            msg.info.height,
        )

        resolution = max(float(msg.info.resolution), 1e-6)
        standoff_m = max(
            0.0, float(self.get_parameter('frontier_goal_standoff_m').value)
        )
        standoff_cells = max(1, round(standoff_m / resolution))
        standoff = candidate_goal_cells_with_standoff(
            segment,
            self._latest_frontier_cells,
            data,
            msg.info.width,
            msg.info.height,
            standoff_cells,
        )
        standoff_set = set(standoff)
        ordered = standoff + [
            index for index in adjacent if index not in standoff_set
        ]
        return ordered, standoff_set

    def _build_candidates(
        self,
        msg: OccupancyGrid,
        robot_x: float,
        robot_y: float,
        min_distance_m: float,
    ) -> list[Candidate]:
        """Materialize every costmap-safe free goal for each frontier segment."""
        candidates: list[Candidate] = []
        source_frame = msg.header.frame_id or 'map'
        now = self._now_sec()
        current_frontier_points: list[tuple[float, float]] = []

        for segment in self._latest_segments:
            representative = representative_cell(segment, msg.info.width)
            representative_point = _cell_to_world(representative, msg)
            current_frontier_points.append(
                (representative_point.x, representative_point.y)
            )

            if self._frontier_is_confirmed_unreachable(representative, msg):
                continue
            if self._costmap_deferred_tracker.is_waiting(
                representative_point.x,
                representative_point.y,
                now,
            ):
                continue

            frontier_distance = hypot(
                representative_point.x - robot_x,
                representative_point.y - robot_y,
            )
            centroid_x, centroid_y = self._segment_centroid_world(segment, msg)
            safe_goals: list[tuple[bool, bool, float, float, int]] = []
            goal_indices, standoff_set = self._goal_candidates_for_segment(
                segment, msg
            )

            for index in goal_indices:
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
                # Prefer deeper standoff goals, then goals far enough to avoid a
                # no-op command, then the point nearest the segment centroid.
                safe_goals.append(
                    (
                        index not in standoff_set,
                        goal_distance < min_distance_m,
                        centroid_distance_sq,
                        goal_distance,
                        index,
                    )
                )

            if not safe_goals:
                checks, exhausted = self._costmap_deferred_tracker.record_blocked(
                    representative_point.x,
                    representative_point.y,
                    now,
                )
                if exhausted:
                    self._mark_confirmed_unreachable(
                        representative,
                        msg,
                        'no costmap-safe standoff or adjacent free goal after '
                        f'{checks} spaced checks',
                    )
                else:
                    self.get_logger().warning(
                        'Deferring costmap-blocked frontier instead of declaring '
                        'it unreachable: '
                        f'x={representative_point.x:.2f}, '
                        f'y={representative_point.y:.2f}; '
                        f'check {checks}/'
                        f'{self._costmap_deferred_tracker.max_checks}; retry in '
                        f'{self._costmap_deferred_tracker.retry_period_sec:.1f}s'
                    )
                continue

            self._costmap_deferred_tracker.clear_near(
                representative_point.x,
                representative_point.y,
            )
            safe_goals.sort()
            for _, _, _, goal_distance, goal_index in safe_goals:
                candidates.append(
                    (
                        representative,
                        goal_index,
                        frontier_distance,
                        goal_distance,
                    )
                )

        self._costmap_deferred_tracker.retain_near(current_frontier_points)

        # Stable sort keeps every segment's ordered safe goals together while
        # still preferring the nearest frontier segment first.
        return sorted(candidates, key=lambda candidate: candidate[2])

    def _is_unavailable_candidate(self, candidate: Candidate) -> bool:
        """Skip only temporary cooldowns or planner-confirmed unreachable regions."""
        if self._is_temporarily_suppressed_candidate(candidate):
            return True
        if self._latest_map is None:
            return False
        point = _cell_to_world(candidate[0], self._latest_map)
        return self._unreachable_tracker.contains(point.x, point.y)

    def _record_no_path_candidate(self, candidate: PlanningCandidate) -> None:
        """Confirm a frontier unreachable only after its final safe goal fails."""
        if self._latest_map is None or candidate[0] != self._generation:
            return

        representative = candidate[1]
        same_frontier_remains = any(
            queued[0] == representative for queued in self._candidate_queue
        )
        if same_frontier_remains:
            remaining = sum(
                1
                for queued in self._candidate_queue
                if queued[0] == representative
            )
            self.get_logger().info(
                'Safe goal has no Nav2 path; trying another goal for the same '
                f'frontier ({remaining} remaining)'
            )
            return

        self._mark_confirmed_unreachable(
            representative,
            self._latest_map,
            'all costmap-safe free goals failed ComputePathToPose',
        )

    def _on_plan_result(self, future, candidate: PlanningCandidate) -> None:
        """Classify genuine no-path results separately from transient failures."""
        if self._planning_candidate != candidate:
            return

        generation, representative, goal_index, frontier_distance, _ = candidate
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._requeue_transient_planner_failure(
                candidate,
                f'Planner result request failed: {exc}',
            )
            return

        result = wrapped_result.result
        path = result.path if result is not None else Path()
        succeeded = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and len(path.poses) > 0
        )
        self._planning_candidate = None

        if generation != self._generation:
            self._start_next_path_check()
            return

        if not succeeded:
            # A missing result object is treated as transport/internal uncertainty,
            # not proof that the frontier itself is unreachable.
            if result is None:
                self._requeue_transient_planner_failure(
                    candidate,
                    'Planner returned no result object',
                )
                return

            error_code = getattr(result, 'error_code', -1)
            error_msg = getattr(result, 'error_msg', '')
            suffix = f' code={error_code}'
            if error_msg:
                suffix += f' ({error_msg})'
            self.get_logger().info(
                f'No path to this frontier-safe goal:{suffix}'
            )
            self._record_no_path_candidate(candidate)
            self._publish_markers()
            self._start_next_path_check()
            return

        self._selected_index = representative
        self._selected_goal_index = goal_index
        self._selected_distance = frontier_distance
        self._selected_path = path
        planned_length = path_length(path)
        frontier_point = _cell_to_world(representative, self._latest_map)
        goal_point = _cell_to_world(goal_index, self._latest_map)

        self.get_logger().info(
            'Reachable frontier selected: '
            f'frontier=({frontier_point.x:.2f}, {frontier_point.y:.2f}), '
            f'goal=({goal_point.x:.2f}, {goal_point.y:.2f}), '
            f'euclidean={frontier_distance:.2f} m, path={planned_length:.2f} m'
        )
        self._completion_tracker.mark_started()
        self._completion_tracker.observe_busy()
        self._path_pub.publish(path)
        self._publish_markers()

    def _requeue_transient_planner_failure(
        self,
        candidate: PlanningCandidate,
        reason: str,
    ) -> None:
        """Never convert planner transport/startup failures into unreachable."""
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
        key = (generation, representative, goal_index)
        retry_count = self._planner_retry_counts.get(key, 0) + 1
        retry_period = max(
            0.1, float(self.get_parameter('planner_retry_period_sec').value)
        )
        queued = (
            representative,
            goal_index,
            frontier_distance,
            goal_distance,
        )

        if retry_count <= retry_limit:
            self._planner_retry_counts[key] = retry_count
            self._candidate_queue.insert(0, queued)
            message = f'retry {retry_count}/{retry_limit}'
        else:
            # Rotate rather than discard. A communication/server failure cannot
            # prove that no geometric path exists.
            self._planner_retry_counts.pop(key, None)
            self._candidate_queue.append(queued)
            message = 'retry window exhausted; rotating candidate, not ignoring it'

        self._retry_not_before_sec = self._now_sec() + retry_period
        self.get_logger().warning(
            f'{reason}; {message}; retrying in about {retry_period:.1f}s'
        )
        self._publish_markers()

    def _retry_pending_planner_check(self) -> None:
        """Rebuild candidates when a deferred costmap-blocked frontier is due."""
        if (
            not self._exploration_complete
            and self._latest_map is not None
            and self._selected_index is None
            and self._planning_candidate is None
            and not self._candidate_queue
            and self._costmap_deferred_tracker.has_ready(self._now_sec())
        ):
            # Reuse the newest frozen map through RetryFrontierDetector's normal
            # deferred-map path so candidate generation remains generation-safe.
            self._deferred_map = self._latest_map
        super()._retry_pending_planner_check()

    def _check_exploration_completion(self) -> None:
        """Do not finish while a costmap-blocked frontier still awaits rechecking."""
        if self._costmap_deferred_tracker.has_pending():
            self._completion_tracker.observe_busy()
            return
        super()._check_exploration_completion()

    def _on_completed_goal(self, msg: PointStamped) -> None:
        """Successful exploration invalidates stale blocked/unreachable evidence."""
        frontier_xy = self._selected_frontier_xy()
        if frontier_xy is not None:
            self._failure_cooldown_tracker.clear_near(
                frontier_xy[0], frontier_xy[1]
            )

        cleared_unreachable = self._unreachable_tracker.clear_all()
        cleared_deferred = self._costmap_deferred_tracker.clear_all()
        if cleared_unreachable or cleared_deferred:
            self.get_logger().info(
                'Revalidating stale frontier evidence after successful '
                'exploration changed the map/costmap: '
                f'unreachable={cleared_unreachable}, deferred={cleared_deferred}'
            )
        super()._on_completed_goal(msg)

    def _on_failed_goal(self, msg: PointStamped) -> None:
        """Navigation execution failure causes cooldown only, never exhaustion."""
        self._completion_tracker.observe_busy()
        frontier_xy = self._selected_frontier_xy()
        if frontier_xy is None:
            frontier_xy = (float(msg.point.x), float(msg.point.y))

        now = self._now_sec()
        attempts, cooldown = self._failure_cooldown_tracker.record_failure(
            frontier_xy[0], frontier_xy[1], now
        )
        radius = self._failure_cooldown_tracker.radius_m

        # Replace any nearby active suppression with the new adaptive expiry.
        self._failed_frontiers = [
            region
            for region in self._failed_frontiers
            if hypot(frontier_xy[0] - region[0], frontier_xy[1] - region[1])
            > radius
        ]
        if cooldown > 0.0:
            self._failed_frontiers.append(
                (frontier_xy[0], frontier_xy[1], now + cooldown)
            )

        self.get_logger().warning(
            'Navigation to reachable frontier failed; applying temporary '
            f'region cooldown={cooldown:.1f}s, repeat={attempts}, '
            f'radius={radius:.2f}m at x={frontier_xy[0]:.2f}, '
            f'y={frontier_xy[1]:.2f}. This does NOT mark it unreachable.'
        )
        self._reset_selection_and_replan(
            'Navigation failure received; selecting another frontier while this '
            'region cools down'
        )

    def _publish_markers(self) -> None:
        """Publish normal frontier markers plus confirmed-unreachable regions."""
        super()._publish_markers()
        msg = self._latest_map
        if msg is None:
            return

        marker = Marker()
        marker.header.stamp = msg.header.stamp
        marker.header.frame_id = msg.header.frame_id or 'map'
        marker.ns = 'confirmed_unreachable_frontiers'
        marker.id = 20
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30
        marker.color.r = 0.45
        marker.color.g = 0.45
        marker.color.b = 0.45
        marker.color.a = 1.0

        for x, y in self._unreachable_tracker.regions:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.20
            marker.points.append(point)

        if not marker.points:
            marker.action = Marker.DELETE

        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self._marker_pub.publish(marker_array)
