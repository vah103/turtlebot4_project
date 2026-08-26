#!/usr/bin/env python3
"""ROS 2 adapter for the official MapEx `nearest` frontier policy.

Policy semantics are ported from castacks/MapEx commit
53636bd1c79153acc3c74a532837d78c926bae5e:

* frontier cell = observed free cell with at least one 8-neighbour unknown cell;
* frontier regions use 8-connectivity;
* keep regions with strictly more than 10 frontier cells;
* representative = region cell closest to the arithmetic mean row/column;
* score = Euclidean distance from the current robot pose;
* choose the minimum-score frontier;
* if the local planner cannot reach it, try the next-lowest-cost frontier.

The original MapEx implementation executes the policy in a grid simulator with
pyastar2d.  This adapter intentionally replaces only that execution layer:
Nav2 ComputePathToPose is used as the reachability/local-planning test and the
existing TurtleBot4 exploration_manager executes the validated goal.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Int32
from tf2_ros import Buffer, TransformException, TransformListener


MAPEX_SOURCE_COMMIT = "53636bd1c79153acc3c74a532837d78c926bae5e"
REGION_SIZE_THRESHOLD = 10  # MapEx keeps region_size > 10, not >= 10.
MIN_FRONTIER_DISTANCE_M = 1.0
COMPLETION_STABLE_CYCLES = 5
COMPLETION_MIN_IDLE_S = 10.0
COMPLETION_CHECK_PERIOD_S = 2.0
COMPLETION_STARTUP_GRACE_S = 20.0
EXECUTION_FAILURE_SUPPRESSION_M = 0.25


@dataclass(frozen=True)
class FrontierCandidate:
    row: int
    col: int
    distance_cells: float
    distance_m: float


class MapExNearestROS(Node):
    """Apply MapEx nearest-frontier policy to live ROS OccupancyGrid maps."""

    def __init__(self) -> None:
        super().__init__(
            "mapex_nearest_frontier",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("path_topic", "/frontier_selected_path")
        self.declare_parameter("selected_frontier_topic", "/frontier_selected")
        self.declare_parameter("candidate_count_topic", "/frontier_candidate_count")
        self.declare_parameter("completed_goal_topic", "/frontier_completed_goal")
        self.declare_parameter("failed_goal_topic", "/frontier_failed_goal")
        self.declare_parameter("completion_topic", "/exploration_complete")

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.planner = ActionClient(
            self,
            ComputePathToPose,
            str(self.get_parameter("planner_action").value),
        )

        self.path_pub = self.create_publisher(
            Path, str(self.get_parameter("path_topic").value), 10
        )
        self.frontier_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter("selected_frontier_topic").value),
            latched_qos,
        )
        self.candidate_count_pub = self.create_publisher(
            Int32, str(self.get_parameter("candidate_count_topic").value), 10
        )
        self.completion_pub = self.create_publisher(
            Bool, str(self.get_parameter("completion_topic").value), latched_qos
        )

        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("completed_goal_topic").value),
            self._on_goal_success,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("failed_goal_topic").value),
            self._on_goal_failure,
            10,
        )

        self.latest_map: OccupancyGrid | None = None
        self.active_frontier_xy: tuple[float, float] | None = None
        self.active = False
        self.planning = False
        self.current_candidates: list[FrontierCandidate] = []
        self.current_candidate_signature: tuple[tuple[int, int], ...] = ()
        self.planning_index = 0
        self.exhausted_signature: tuple[tuple[int, int], ...] | None = None
        self.failed_execution_xy: list[tuple[float, float]] = []

        self.first_ready_sec: float | None = None
        self.started = False
        self.idle_first_sec: float | None = None
        self.idle_last_check_sec: float | None = None
        self.idle_streak = 0
        self.complete = False
        self.shutdown_timer = None
        self.planner_wait_notice = False

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            "Official MapEx nearest policy adapter active: "
            f"source_commit={MAPEX_SOURCE_COMMIT}, region_size>10, "
            "8-connected regions, Euclidean frontier score, min_distance=1.0 m"
        )
        self.get_logger().info(
            "Execution adapter: Nav2 ComputePathToPose replaces MapEx pyastar2d A*; "
            "the selected goal remains the MapEx frontier-center cell"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_map(self, msg: OccupancyGrid) -> None:
        if msg.info.width <= 0 or msg.info.height <= 0:
            return
        if len(msg.data) != int(msg.info.width) * int(msg.info.height):
            return
        self.latest_map = msg

    @staticmethod
    def _origin_yaw(msg: OccupancyGrid) -> float:
        q = msg.info.origin.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _cell_to_world(
        self, row: int, col: int, msg: OccupancyGrid
    ) -> tuple[float, float]:
        res = float(msg.info.resolution)
        local_x = (col + 0.5) * res
        local_y = (row + 0.5) * res
        yaw = self._origin_yaw(msg)
        c, s = math.cos(yaw), math.sin(yaw)
        return (
            float(msg.info.origin.position.x) + c * local_x - s * local_y,
            float(msg.info.origin.position.y) + s * local_x + c * local_y,
        )

    def _world_to_grid_float(
        self, x: float, y: float, msg: OccupancyGrid
    ) -> tuple[float, float]:
        dx = x - float(msg.info.origin.position.x)
        dy = y - float(msg.info.origin.position.y)
        yaw = self._origin_yaw(msg)
        c, s = math.cos(yaw), math.sin(yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        res = max(float(msg.info.resolution), 1e-9)
        return local_y / res - 0.5, local_x / res - 0.5

    def _robot_xy(self, frame: str) -> tuple[float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                frame,
                str(self.get_parameter("robot_frame").value),
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(f"Waiting for robot TF: {exc}")
            return None
        return (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
        )

    @staticmethod
    def _frontier_mask(grid: np.ndarray) -> np.ndarray:
        """Exact MapEx edge rule: free cell with any 8-neighbour unknown."""
        unknown = grid < 0
        free = grid == 0
        h, w = grid.shape
        unknown_neighbour = np.zeros((h, w), dtype=bool)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                src_r0 = max(0, -dr)
                src_r1 = min(h, h - dr)
                src_c0 = max(0, -dc)
                src_c1 = min(w, w - dc)
                dst_r0 = src_r0 + dr
                dst_r1 = src_r1 + dr
                dst_c0 = src_c0 + dc
                dst_c1 = src_c1 + dc
                unknown_neighbour[dst_r0:dst_r1, dst_c0:dst_c1] |= unknown[
                    src_r0:src_r1, src_c0:src_c1
                ]
        return free & unknown_neighbour

    @staticmethod
    def _region_representatives(frontier: np.ndarray) -> list[tuple[int, int]]:
        """8-connected components; representative matches MapEx mean/argmin rule."""
        h, w = frontier.shape
        visited = np.zeros((h, w), dtype=bool)
        representatives: list[tuple[int, int]] = []
        neighbours = (
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),             (0, 1),
            (1, -1),  (1, 0),   (1, 1),
        )

        for row, col in np.argwhere(frontier):
            row = int(row)
            col = int(col)
            if visited[row, col]:
                continue
            queue = deque([(row, col)])
            visited[row, col] = True
            region: list[tuple[int, int]] = []
            while queue:
                rr, cc = queue.popleft()
                region.append((rr, cc))
                for dr, dc in neighbours:
                    nr, nc = rr + dr, cc + dc
                    if (
                        0 <= nr < h
                        and 0 <= nc < w
                        and frontier[nr, nc]
                        and not visited[nr, nc]
                    ):
                        visited[nr, nc] = True
                        queue.append((nr, nc))

            # MapEx: large_regions = region_sizes > 10.
            if len(region) <= REGION_SIZE_THRESHOLD:
                continue
            arr = np.asarray(region, dtype=np.float64)
            mean = arr.mean(axis=0)
            distances = np.linalg.norm(arr - mean, axis=1)
            best = region[int(np.argmin(distances))]
            representatives.append(best)

        return representatives

    def _is_execution_suppressed(self, x: float, y: float) -> bool:
        return any(
            math.hypot(x - fx, y - fy) <= EXECUTION_FAILURE_SUPPRESSION_M
            for fx, fy in self.failed_execution_xy
        )

    def _compute_candidates(
        self, msg: OccupancyGrid, robot_xy: tuple[float, float]
    ) -> list[FrontierCandidate]:
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        centers = self._region_representatives(self._frontier_mask(grid))
        robot_row, robot_col = self._world_to_grid_float(robot_xy[0], robot_xy[1], msg)
        res = float(msg.info.resolution)
        candidates: list[FrontierCandidate] = []
        for row, col in centers:
            distance_cells = math.hypot(row - robot_row, col - robot_col)
            distance_m = distance_cells * res
            if distance_m < MIN_FRONTIER_DISTANCE_M:
                continue
            x, y = self._cell_to_world(row, col, msg)
            if self._is_execution_suppressed(x, y):
                continue
            candidates.append(
                FrontierCandidate(row, col, distance_cells, distance_m)
            )
        # MapEx nearest: total_cost = Euclidean cost_dist; choose argmin.
        candidates.sort(key=lambda item: item.distance_cells)
        return candidates

    def _publish_candidate_count(self, count: int) -> None:
        msg = Int32()
        msg.data = int(count)
        self.candidate_count_pub.publish(msg)

    def _reset_idle(self) -> None:
        self.idle_first_sec = None
        self.idle_last_check_sec = None
        self.idle_streak = 0

    def _observe_exhausted(self) -> None:
        now = self._now_sec()
        if not self.started:
            if self.first_ready_sec is None:
                self.first_ready_sec = now
                return
            if now - self.first_ready_sec < COMPLETION_STARTUP_GRACE_S:
                return
            self.started = True

        if (
            self.idle_last_check_sec is not None
            and now - self.idle_last_check_sec < COMPLETION_CHECK_PERIOD_S
        ):
            return
        self.idle_last_check_sec = now
        if self.idle_first_sec is None:
            self.idle_first_sec = now
        self.idle_streak += 1
        if (
            self.idle_streak >= COMPLETION_STABLE_CYCLES
            and now - self.idle_first_sec >= COMPLETION_MIN_IDLE_S
        ):
            self._finish_exploration()

    def _finish_exploration(self) -> None:
        if self.complete:
            return
        self.complete = True
        msg = Bool()
        msg.data = True
        self.completion_pub.publish(msg)
        self.get_logger().warning(
            "MapEx-nearest exploration complete: no planner-reachable MapEx "
            "frontier remained for the frozen completion window"
        )
        self.shutdown_timer = self.create_timer(1.0, self._shutdown)

    def _shutdown(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()

    def _tick(self) -> None:
        if self.complete or self.active or self.planning or self.latest_map is None:
            return
        if not self.planner.server_is_ready():
            if not self.planner_wait_notice:
                self.get_logger().info("Waiting for Nav2 ComputePathToPose action server")
                self.planner_wait_notice = True
            return
        self.planner_wait_notice = False

        msg = self.latest_map
        frame = msg.header.frame_id or "map"
        robot_xy = self._robot_xy(frame)
        if robot_xy is None:
            return
        candidates = self._compute_candidates(msg, robot_xy)
        signature = tuple((c.row, c.col) for c in candidates)
        self._publish_candidate_count(len(candidates))

        if self.exhausted_signature == signature:
            self._observe_exhausted()
            return

        self.current_candidates = candidates
        self.current_candidate_signature = signature
        self.planning_index = 0
        if not candidates:
            self.exhausted_signature = signature
            self._observe_exhausted()
            return

        self._reset_idle()
        self.first_ready_sec = None
        self._plan_next_candidate()

    def _plan_next_candidate(self) -> None:
        if self.latest_map is None:
            self.planning = False
            return
        if self.planning_index >= len(self.current_candidates):
            self.planning = False
            self.exhausted_signature = self.current_candidate_signature
            self.get_logger().warning(
                "All current MapEx frontier centers failed Nav2 path validation; "
                "waiting for a changed frontier set"
            )
            self._observe_exhausted()
            return

        candidate = self.current_candidates[self.planning_index]
        self.planning_index += 1
        msg = self.latest_map
        frame = msg.header.frame_id or "map"
        x, y = self._cell_to_world(candidate.row, candidate.col, msg)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame
        goal_pose.header.stamp.sec = 0
        goal_pose.header.stamp.nanosec = 0
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.planner_id = ""
        goal.use_start = False

        self.planning = True
        future = self.planner.send_goal_async(goal)
        future.add_done_callback(
            lambda done, cand=candidate, xy=(x, y): self._on_plan_goal_response(
                done, cand, xy
            )
        )

    def _on_plan_goal_response(
        self,
        future,
        candidate: FrontierCandidate,
        frontier_xy: tuple[float, float],
    ) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"ComputePathToPose request failed: {exc}")
            self.planning = False
            self._plan_next_candidate()
            return
        if not handle.accepted:
            self.planning = False
            self._plan_next_candidate()
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda done, cand=candidate, xy=frontier_xy: self._on_plan_result(
                done, cand, xy
            )
        )

    def _on_plan_result(
        self,
        future,
        candidate: FrontierCandidate,
        frontier_xy: tuple[float, float],
    ) -> None:
        self.planning = False
        try:
            wrapped = future.result()
            path = wrapped.result.path
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"ComputePathToPose result failed: {exc}")
            self._plan_next_candidate()
            return

        if path is None or not path.poses:
            self.get_logger().info(
                "MapEx candidate has no Nav2 path; trying next nearest frontier: "
                f"distance={candidate.distance_m:.2f} m"
            )
            self._plan_next_candidate()
            return

        # A viable path resets exhausted evidence. Keep this frontier locked until
        # the manager reports success/failure, matching MapEx's locked frontier.
        self.exhausted_signature = None
        self._reset_idle()
        self.started = True
        self.active = True
        self.active_frontier_xy = frontier_xy

        # Publish path first because the recorder opens a decision on this topic;
        # publish the exact MapEx frontier center immediately after it.
        self.path_pub.publish(path)
        point = PointStamped()
        point.header.frame_id = self.latest_map.header.frame_id or "map"
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = frontier_xy[0]
        point.point.y = frontier_xy[1]
        self.frontier_pub.publish(point)

        self.get_logger().warning(
            "Selected MapEx nearest frontier: "
            f"row={candidate.row}, col={candidate.col}, "
            f"euclidean_distance={candidate.distance_m:.2f} m"
        )

    def _on_goal_success(self, _msg: PointStamped) -> None:
        if not self.active:
            return
        self.active = False
        self.active_frontier_xy = None
        # MapEx recomputes frontier scores after reaching the locked frontier.
        self.failed_execution_xy.clear()
        self.exhausted_signature = None

    def _on_goal_failure(self, _msg: PointStamped) -> None:
        if not self.active:
            return
        if self.active_frontier_xy is not None:
            self.failed_execution_xy.append(self.active_frontier_xy)
            self.get_logger().warning(
                "Nav2 execution failed for a planner-valid MapEx frontier; "
                "temporarily excluding that exact local region and reselecting"
            )
        self.active = False
        self.active_frontier_xy = None
        self.exhausted_signature = None


def main() -> int:
    rclpy.init()
    node = MapExNearestROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
