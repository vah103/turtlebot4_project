"""Retry-capable wrapper for the frontier detector's Nav2 planner checks."""

from math import cos, floor, hypot, sin

import rclpy
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import ComputePathToPose

from frontier_exploration.frontier_detector import (
    FREE,
    FrontierDetector,
    _cell_to_world,
    _origin_yaw,
)


class RetryFrontierDetector(FrontierDetector):
    """Keep frontier candidates alive while Nav2 planner is still starting."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('planner_retry_period_sec', 1.0)
        self.declare_parameter('planner_rejection_retry_limit', 15)
        self.declare_parameter('failed_goal_topic', '/frontier_failed_goal')
        self.declare_parameter('completed_goal_topic', '/frontier_completed_goal')
        self.declare_parameter('failed_goal_radius_m', 0.75)
        self.declare_parameter('failed_goal_cooldown_sec', 30.0)
        self.declare_parameter('safe_goal_standoff_m', 0.35)
        self.declare_parameter('safe_goal_search_radius_m', 0.25)
        self.declare_parameter('safe_goal_clearance_m', 0.10)

        retry_period = max(
            0.1, float(self.get_parameter('planner_retry_period_sec').value)
        )
        failed_goal_topic = str(self.get_parameter('failed_goal_topic').value)
        completed_goal_topic = str(
            self.get_parameter('completed_goal_topic').value
        )

        self._planner_retry_counts: dict[tuple[int, int], int] = {}
        self._retry_not_before_sec = 0.0
        self._failed_goals: list[tuple[float, float, float]] = []
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
            f'Failed frontier blacklist listening on {failed_goal_topic}'
        )
        self.get_logger().info(
            f'Completed frontier refresh listening on {completed_goal_topic}'
        )
        self.get_logger().info(
            'Safe frontier goal enabled: '
            f'standoff={float(self.get_parameter("safe_goal_standoff_m").value):.2f}m, '
            f'clearance={float(self.get_parameter("safe_goal_clearance_m").value):.2f}m'
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

    def _reset_selection_and_replan(self, reason: str) -> None:
        """Invalidate cached selection and immediately re-evaluate the latest map."""
        self._generation += 1
        self._candidate_signature = None
        self._candidate_queue = []
        self._planning_candidate = None
        self._selected_index = None
        self._selected_distance = None
        self._selected_path = None
        self._planner_retry_counts.clear()
        self._retry_not_before_sec = 0.0

        latest_map = self._latest_map
        if latest_map is None:
            return

        self.get_logger().info(reason)
        self._publish_empty_path(latest_map.header.frame_id or 'map')
        super()._on_map(latest_map)

    def _on_completed_goal(self, msg: PointStamped) -> None:
        self._reset_selection_and_replan(
            'Completed frontier reached; refreshing candidates from latest map '
            f'at x={float(msg.point.x):.2f}, y={float(msg.point.y):.2f}'
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

        self._reset_selection_and_replan(
            'Failed frontier feedback received; refreshing candidate ordering'
        )

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

    def _world_to_cell(self, x: float, y: float) -> int | None:
        msg = self._latest_map
        if msg is None:
            return None

        resolution = max(float(msg.info.resolution), 1e-6)
        dx = x - msg.info.origin.position.x
        dy = y - msg.info.origin.position.y
        yaw = _origin_yaw(msg)
        local_x = cos(yaw) * dx + sin(yaw) * dy
        local_y = -sin(yaw) * dx + cos(yaw) * dy
        cell_x = int(floor(local_x / resolution))
        cell_y = int(floor(local_y / resolution))

        if not (0 <= cell_x < msg.info.width and 0 <= cell_y < msg.info.height):
            return None
        return cell_y * msg.info.width + cell_x

    def _cell_has_clearance(self, index: int, clearance_cells: int) -> bool:
        msg = self._latest_map
        if msg is None or msg.data[index] != FREE:
            return False

        width = msg.info.width
        height = msg.info.height
        center_x = index % width
        center_y = index // width
        radius_sq = clearance_cells * clearance_cells

        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if dx * dx + dy * dy > radius_sq:
                    continue
                x = center_x + dx
                y = center_y + dy
                if not (0 <= x < width and 0 <= y < height):
                    return False
                if msg.data[y * width + x] != FREE:
                    return False
        return True

    def _safe_goal_for_frontier(
        self,
        frontier_index: int,
    ) -> tuple[float, float] | None:
        msg = self._latest_map
        if msg is None:
            return None

        map_frame = msg.header.frame_id or 'map'
        robot_position = self._robot_position(map_frame)
        if robot_position is None:
            return None

        frontier = _cell_to_world(frontier_index, msg)
        robot_x, robot_y = robot_position
        dx = robot_x - frontier.x
        dy = robot_y - frontier.y
        distance = hypot(dx, dy)
        if distance <= 1e-6:
            return None

        standoff = max(
            0.0, float(self.get_parameter('safe_goal_standoff_m').value)
        )
        # Keep the goal on the explored side of the frontier and avoid placing
        # it directly on the free/unknown boundary.
        retreat = min(standoff, max(0.0, distance - 0.20))
        desired_x = frontier.x + dx / distance * retreat
        desired_y = frontier.y + dy / distance * retreat

        resolution = max(float(msg.info.resolution), 1e-6)
        search_radius = max(
            0.0, float(self.get_parameter('safe_goal_search_radius_m').value)
        )
        clearance = max(
            0.0, float(self.get_parameter('safe_goal_clearance_m').value)
        )
        search_cells = max(0, round(search_radius / resolution))
        clearance_cells = max(0, round(clearance / resolution))

        desired_index = self._world_to_cell(desired_x, desired_y)
        if desired_index is None:
            return None

        width = msg.info.width
        height = msg.info.height
        desired_cell_x = desired_index % width
        desired_cell_y = desired_index // width
        candidates: list[tuple[float, int]] = []

        for offset_y in range(-search_cells, search_cells + 1):
            for offset_x in range(-search_cells, search_cells + 1):
                if offset_x * offset_x + offset_y * offset_y > search_cells * search_cells:
                    continue
                cell_x = desired_cell_x + offset_x
                cell_y = desired_cell_y + offset_y
                if not (0 <= cell_x < width and 0 <= cell_y < height):
                    continue
                index = cell_y * width + cell_x
                if not self._cell_has_clearance(index, clearance_cells):
                    continue
                point = _cell_to_world(index, msg)
                score = (point.x - desired_x) ** 2 + (point.y - desired_y) ** 2
                candidates.append((score, index))

        if not candidates:
            return None

        _, best_index = min(candidates, key=lambda item: item[0])
        best = _cell_to_world(best_index, msg)
        return best.x, best.y

    def _start_next_path_check(self) -> None:
        now = self._now_sec()
        if now < self._retry_not_before_sec:
            return
        if self._selected_index is not None:
            return
        if self._planning_candidate is not None:
            return
        if self._latest_map is None:
            return
        if not self._candidate_queue:
            return

        if not self._planner_client.server_is_ready():
            if not self._planner_warning_shown:
                planner_action = str(self.get_parameter('planner_action').value)
                self.get_logger().warning(
                    f'Waiting for Nav2 planner action {planner_action}'
                )
                self._planner_warning_shown = True
            return

        self._planner_warning_shown = False
        skipped_blacklist = 0
        skipped_unsafe = 0

        while self._candidate_queue:
            index, distance = self._candidate_queue.pop(0)
            frontier = _cell_to_world(index, self._latest_map)
            if self._is_blacklisted_xy(frontier.x, frontier.y):
                skipped_blacklist += 1
                continue

            safe_goal = self._safe_goal_for_frontier(index)
            if safe_goal is None:
                skipped_unsafe += 1
                continue

            candidate = (self._generation, index, distance)
            self._planning_candidate = candidate
            goal_x, goal_y = safe_goal

            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal.header.frame_id = self._latest_map.header.frame_id or 'map'
            goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
            goal_msg.goal.pose.position.x = goal_x
            goal_msg.goal.pose.position.y = goal_y
            goal_msg.goal.pose.orientation.w = 1.0
            goal_msg.planner_id = str(self.get_parameter('planner_id').value)
            goal_msg.use_start = False

            self.get_logger().info(
                'Checking Nav2 path to safe frontier goal: '
                f'frontier=({frontier.x:.2f}, {frontier.y:.2f}), '
                f'goal=({goal_x:.2f}, {goal_y:.2f}), '
                f'euclidean={distance:.2f} m'
            )
            self._publish_markers()

            future = self._planner_client.send_goal_async(goal_msg)
            future.add_done_callback(
                lambda response_future, expected=candidate:
                self._on_plan_goal_response(response_future, expected)
            )
            break

        if skipped_blacklist:
            self.get_logger().info(
                f'Skipped {skipped_blacklist} candidate(s) inside failed-frontier blacklist'
            )
        if skipped_unsafe:
            self.get_logger().info(
                f'Skipped {skipped_unsafe} candidate(s) without a safe standoff goal'
            )

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
