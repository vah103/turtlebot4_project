#!/usr/bin/env python3
"""Standalone nearest-frontier explorer ported from MapEx for Hospital/TurtleBot4.

MapEx source pinned for frontier semantics:
  castacks/MapEx @ 53636bd1c79153acc3c74a532837d78c926bae5e
  scripts/sim_utils.py -> FrontierPlanner(score_mode='nearest')

MapEx logic kept exactly at the selection layer:
  - free: 0
  - unknown: 0.5
  - frontier: free cell touching unknown in the 8-neighbourhood
  - frontier regions: 8-connected components
  - keep regions with size > 10 cells
  - representative: real frontier cell nearest arithmetic mean(row, col)
  - score: Euclidean robot-to-representative distance
  - rank nearest -> farthest

Hospital execution adaptation:
  1. Try every ranked exact MapEx frontier with ComputePathToPose.
  2. If ANY exact frontier is reachable, execute that exact validated path.
  3. Only after ALL exact candidates fail planning, enable fallback.
  4. Fallback keeps the same MapEx ranking but moves the execution point
     inward toward the robot, starting at 0.30 m and increasing to 0.60 m.
  5. Execute the validated path directly with FollowPath.

RViz output:
  /frontier_markers       : all MapEx representative frontier cells,
                            selected frontier, and fallback point when used.
  /frontier_selected_path : exact path sent to FollowPath.

This file intentionally has no LaMa, prediction, variance, information gain,
benchmark recorder, validator, or MapEx post-ranking <1 m rejection.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.ndimage import convolve, generate_binary_structure, label
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MAPEX_REGION_SIZE_THRESHOLD = 10
RETRY_PERIOD_S = 1.0
POST_GOAL_SETTLE_S = 1.0


@dataclass(frozen=True)
class FrontierCandidate:
    row: int
    col: int
    distance_cells: float


class MapExNearestExplorer(Node):
    def __init__(self) -> None:
        super().__init__('mapex_nearest_mapex')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('planner_action', '/compute_path_to_pose')
        self.declare_parameter('follow_path_action', '/follow_path')
        self.declare_parameter('marker_topic', '/frontier_markers')
        self.declare_parameter('path_topic', '/frontier_selected_path')
        self.declare_parameter('fallback_standoff_m', 0.30)
        self.declare_parameter('fallback_max_standoff_m', 0.60)
        self.declare_parameter('fallback_step_m', 0.05)

        map_topic = str(self.get_parameter('map_topic').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        planner_action = str(self.get_parameter('planner_action').value)
        follow_path_action = str(self.get_parameter('follow_path_action').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        self.fallback_standoff_m = float(
            self.get_parameter('fallback_standoff_m').value
        )
        self.fallback_max_standoff_m = float(
            self.get_parameter('fallback_max_standoff_m').value
        )
        self.fallback_step_m = float(self.get_parameter('fallback_step_m').value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.planner = ActionClient(self, ComputePathToPose, planner_action)
        self.follower = ActionClient(self, FollowPath, follow_path_action)

        self.latest_map: OccupancyGrid | None = None
        self.busy = False
        self.current_candidates: list[FrontierCandidate] = []
        self.current_candidate: FrontierCandidate | None = None
        self.candidate_index = 0
        self.mode = 'exact'
        self.next_fallback_standoff_m = self.fallback_standoff_m
        self.current_execution_xy: tuple[float, float] | None = None
        self.execution_failed_cells: set[tuple[int, int]] = set()
        self.next_allowed_time_s = 0.0
        self.wait_notice_shown = False
        self.no_frontier_notice_shown = False

        self.create_timer(RETRY_PERIOD_S, self._tick)

        self.get_logger().warning(
            'MAPEX NEAREST MAPEX ACTIVE: exact ranked frontiers first; '
            'standoff fallback only after every exact candidate fails planning.'
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _origin_yaw(self, msg: OccupancyGrid) -> float:
        return self._yaw_from_quaternion(msg.info.origin.orientation)

    def _robot_pose_in_map(
        self, msg: OccupancyGrid
    ) -> tuple[float, float, float] | None:
        frame = msg.header.frame_id or 'map'
        try:
            tf = self.tf_buffer.lookup_transform(
                frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for TF {frame}->{self.robot_frame}: {exc}'
            )
            return None

        t = tf.transform.translation
        yaw = self._yaw_from_quaternion(tf.transform.rotation)
        return float(t.x), float(t.y), yaw

    def _world_to_grid(
        self, x: float, y: float, msg: OccupancyGrid
    ) -> tuple[float, float]:
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        yaw = self._origin_yaw(msg)
        c = math.cos(yaw)
        s = math.sin(yaw)

        dx = x - ox
        dy = y - oy
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        res = float(msg.info.resolution)

        col = local_x / res - 0.5
        row = local_y / res - 0.5
        return row, col

    def _grid_to_world(
        self, row: int, col: int, msg: OccupancyGrid
    ) -> tuple[float, float]:
        res = float(msg.info.resolution)
        local_x = (float(col) + 0.5) * res
        local_y = (float(row) + 0.5) * res

        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        yaw = self._origin_yaw(msg)
        c = math.cos(yaw)
        s = math.sin(yaw)

        x = ox + c * local_x - s * local_y
        y = oy + s * local_x + c * local_y
        return x, y

    def _world_cell_is_free(
        self, x: float, y: float, msg: OccupancyGrid
    ) -> bool:
        row_f, col_f = self._world_to_grid(x, y, msg)
        row = int(round(row_f))
        col = int(round(col_f))
        h = int(msg.info.height)
        w = int(msg.info.width)
        if row < 0 or row >= h or col < 0 or col >= w:
            return False
        raw = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        return bool(raw[row, col] == 0)

    @staticmethod
    def _mapex_observed_map(msg: OccupancyGrid) -> np.ndarray:
        """Convert ROS OccupancyGrid to MapEx observed-map labels."""
        raw = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        obs = np.ones(raw.shape, dtype=np.float32)
        obs[raw == 0] = 0.0
        obs[raw < 0] = 0.5
        obs[raw > 0] = 1.0
        return obs

    @staticmethod
    def _mapex_frontier_centers(obs_map: np.ndarray) -> list[np.ndarray]:
        """Direct port of MapEx FrontierPlanner.get_frontier_centers_given_obs_map."""
        kernel = np.array(
            [
                [1, 1, 1],
                [1, 0, 1],
                [1, 1, 1],
            ]
        )

        edge_cells = convolve((obs_map == 0.5).astype(int), kernel) > 0
        edge_cells = edge_cells & (obs_map == 0)

        structure = generate_binary_structure(2, 2)
        frontier_regions, num_regions = label(edge_cells, structure)

        region_sizes = np.bincount(frontier_regions.ravel())
        large_regions = region_sizes > MAPEX_REGION_SIZE_THRESHOLD
        if len(large_regions) > 0:
            large_regions[0] = False

        frontier_region_centers: list[np.ndarray] = []
        for region_id in range(1, num_regions + 1):
            if not large_regions[region_id]:
                continue
            region = np.argwhere(frontier_regions == region_id)
            region_center = np.mean(region, axis=0)
            dist_to_center = np.linalg.norm(region - region_center, axis=1)
            closest_point_to_center = region[np.argmin(dist_to_center)]
            frontier_region_centers.append(closest_point_to_center)

        return frontier_region_centers

    def _rank_candidates(
        self, msg: OccupancyGrid, robot_x: float, robot_y: float
    ) -> list[FrontierCandidate]:
        obs_map = self._mapex_observed_map(msg)
        centers = self._mapex_frontier_centers(obs_map)
        if not centers:
            return []

        robot_row, robot_col = self._world_to_grid(robot_x, robot_y, msg)
        robot_rc = np.array([robot_row, robot_col], dtype=np.float64)

        center_array = np.asarray(centers, dtype=np.float64)
        distances = np.linalg.norm(center_array - robot_rc, axis=1)
        order = np.argsort(distances)

        ranked: list[FrontierCandidate] = []
        for index in order:
            ranked.append(
                FrontierCandidate(
                    row=int(center_array[index, 0]),
                    col=int(center_array[index, 1]),
                    distance_cells=float(distances[index]),
                )
            )
        return ranked

    def _publish_markers(
        self,
        candidates: list[FrontierCandidate],
        selected: FrontierCandidate | None = None,
        fallback_xy: tuple[float, float] | None = None,
    ) -> None:
        if self.latest_map is None:
            return

        msg = self.latest_map
        frame = msg.header.frame_id or 'map'
        now = self.get_clock().now().to_msg()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers = MarkerArray(markers=[delete_all])

        cells = Marker()
        cells.header.frame_id = frame
        cells.header.stamp = now
        cells.ns = 'mapex_frontier_cells'
        cells.id = 0
        cells.type = Marker.CUBE_LIST
        cells.action = Marker.ADD
        cells.pose.orientation.w = 1.0
        cells.scale.x = max(float(msg.info.resolution), 0.05)
        cells.scale.y = max(float(msg.info.resolution), 0.05)
        cells.scale.z = 0.05
        cells.color.r = 0.1
        cells.color.g = 1.0
        cells.color.b = 0.1
        cells.color.a = 0.9
        for candidate in candidates:
            x, y = self._grid_to_world(candidate.row, candidate.col, msg)
            cells.points.append(Point(x=x, y=y, z=0.05))
        markers.markers.append(cells)

        if selected is not None:
            x, y = self._grid_to_world(selected.row, selected.col, msg)
            chosen = Marker()
            chosen.header.frame_id = frame
            chosen.header.stamp = now
            chosen.ns = 'mapex_selected_frontier'
            chosen.id = 1
            chosen.type = Marker.SPHERE
            chosen.action = Marker.ADD
            chosen.pose.position.x = x
            chosen.pose.position.y = y
            chosen.pose.position.z = 0.12
            chosen.pose.orientation.w = 1.0
            chosen.scale.x = 0.22
            chosen.scale.y = 0.22
            chosen.scale.z = 0.22
            chosen.color.r = 1.0
            chosen.color.g = 0.1
            chosen.color.b = 0.1
            chosen.color.a = 1.0
            markers.markers.append(chosen)

        if fallback_xy is not None:
            fallback = Marker()
            fallback.header.frame_id = frame
            fallback.header.stamp = now
            fallback.ns = 'mapex_fallback_goal'
            fallback.id = 2
            fallback.type = Marker.SPHERE
            fallback.action = Marker.ADD
            fallback.pose.position.x = fallback_xy[0]
            fallback.pose.position.y = fallback_xy[1]
            fallback.pose.position.z = 0.12
            fallback.pose.orientation.w = 1.0
            fallback.scale.x = 0.18
            fallback.scale.y = 0.18
            fallback.scale.z = 0.18
            fallback.color.r = 1.0
            fallback.color.g = 0.55
            fallback.color.b = 0.0
            fallback.color.a = 1.0
            markers.markers.append(fallback)

        self.marker_pub.publish(markers)

    def _tick(self) -> None:
        if self.busy or self.latest_map is None:
            return
        if self._now_s() < self.next_allowed_time_s:
            return

        if not self.planner.server_is_ready() or not self.follower.server_is_ready():
            if not self.wait_notice_shown:
                self.wait_notice_shown = True
                self.get_logger().info(
                    'Waiting for Nav2 /compute_path_to_pose and /follow_path actions'
                )
            return
        self.wait_notice_shown = False

        msg = self.latest_map
        robot_pose = self._robot_pose_in_map(msg)
        if robot_pose is None:
            return

        all_candidates = self._rank_candidates(msg, robot_pose[0], robot_pose[1])
        self._publish_markers(all_candidates)

        if not all_candidates:
            if not self.no_frontier_notice_shown:
                self.no_frontier_notice_shown = True
                self.get_logger().warning(
                    'No MapEx frontier region >10 cells is currently available.'
                )
            return

        self.no_frontier_notice_shown = False

        candidates = [
            c
            for c in all_candidates
            if (c.row, c.col) not in self.execution_failed_cells
        ]
        if not candidates:
            self.get_logger().warning(
                'All frontier cells previously failed execution; clearing temporary '
                'execution-failure memory and trying them again.'
            )
            self.execution_failed_cells.clear()
            candidates = all_candidates

        self.current_candidates = candidates
        self.current_candidate = None
        self.candidate_index = 0
        self.mode = 'exact'
        self.next_fallback_standoff_m = self.fallback_standoff_m
        self.busy = True

        nearest = candidates[0]
        distance_m = nearest.distance_cells * float(msg.info.resolution)
        self.get_logger().warning(
            f'MapEx nearest cycle: candidates={len(candidates)}, '
            f'nearest row={nearest.row}, col={nearest.col}, '
            f'distance={distance_m:.2f} m'
        )
        self._try_next_exact_candidate()

    def _make_goal_pose(self, x: float, y: float) -> PoseStamped | None:
        if self.latest_map is None:
            return None
        robot_pose = self._robot_pose_in_map(self.latest_map)
        if robot_pose is None:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.latest_map.header.frame_id or 'map'
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(robot_pose[2] * 0.5)
        pose.pose.orientation.w = math.cos(robot_pose[2] * 0.5)
        return pose

    def _send_plan(
        self,
        candidate: FrontierCandidate,
        execution_xy: tuple[float, float],
        mode: str,
    ) -> None:
        pose = self._make_goal_pose(execution_xy[0], execution_xy[1])
        if pose is None:
            self.busy = False
            return

        self.current_candidate = candidate
        self.current_execution_xy = execution_xy
        self.mode = mode

        if mode == 'fallback':
            self._publish_markers(
                self.current_candidates,
                selected=candidate,
                fallback_xy=execution_xy,
            )
        else:
            self._publish_markers(self.current_candidates, selected=candidate)

        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.planner_id = ''
        goal.use_start = False

        future = self.planner.send_goal_async(goal)
        future.add_done_callback(self._on_plan_goal_response)

    def _try_next_exact_candidate(self) -> None:
        if self.latest_map is None:
            self.busy = False
            return

        if self.candidate_index >= len(self.current_candidates):
            self.get_logger().warning(
                'All exact MapEx frontier candidates failed planning. '
                'Starting standoff fallback.'
            )
            self.mode = 'fallback'
            self.candidate_index = 0
            self.next_fallback_standoff_m = self.fallback_standoff_m
            self._try_next_fallback_point()
            return

        candidate = self.current_candidates[self.candidate_index]
        self.candidate_index += 1
        x, y = self._grid_to_world(candidate.row, candidate.col, self.latest_map)
        self._send_plan(candidate, (x, y), 'exact')

    def _fallback_xy(
        self,
        candidate: FrontierCandidate,
        standoff_m: float,
    ) -> tuple[float, float] | None:
        if self.latest_map is None:
            return None
        robot_pose = self._robot_pose_in_map(self.latest_map)
        if robot_pose is None:
            return None

        fx, fy = self._grid_to_world(
            candidate.row, candidate.col, self.latest_map
        )
        rx, ry = robot_pose[0], robot_pose[1]
        dx = rx - fx
        dy = ry - fy
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            return None

        use_dist = min(standoff_m, max(0.0, length - 0.05))
        x = fx + dx / length * use_dist
        y = fy + dy / length * use_dist
        return x, y

    def _try_next_fallback_point(self) -> None:
        if self.latest_map is None:
            self.busy = False
            return

        while self.candidate_index < len(self.current_candidates):
            candidate = self.current_candidates[self.candidate_index]

            while (
                self.next_fallback_standoff_m
                <= self.fallback_max_standoff_m + 1e-9
            ):
                standoff = self.next_fallback_standoff_m
                self.next_fallback_standoff_m += self.fallback_step_m
                xy = self._fallback_xy(candidate, standoff)
                if xy is None:
                    continue
                if not self._world_cell_is_free(xy[0], xy[1], self.latest_map):
                    continue

                self.get_logger().warning(
                    f'Fallback candidate rank={self.candidate_index + 1}: '
                    f'standoff={standoff:.2f} m'
                )
                self._send_plan(candidate, xy, 'fallback')
                return

            self.candidate_index += 1
            self.next_fallback_standoff_m = self.fallback_standoff_m

        self.get_logger().warning(
            'No exact or fallback MapEx frontier is planner-reachable right now; '
            'retrying after map/TF update.'
        )
        self.current_candidate = None
        self.current_execution_xy = None
        self.busy = False
        self.next_allowed_time_s = self._now_s() + RETRY_PERIOD_S

    def _on_plan_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'ComputePathToPose request failed: {exc}')
            self._advance_after_plan_failure()
            return

        if not handle.accepted:
            self.get_logger().info('Planner rejected candidate.')
            self._advance_after_plan_failure()
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'ComputePathToPose result failed: {exc}')
            self._advance_after_plan_failure()
            return

        path = wrapped.result.path
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            if self.mode == 'exact':
                self.get_logger().info(
                    'Exact candidate has no successful Nav2 path; trying next exact.'
                )
            else:
                self.get_logger().info(
                    'Fallback point has no successful Nav2 path; trying next fallback.'
                )
            self._advance_after_plan_failure()
            return

        clean_path = Path()
        clean_path.header = path.header
        clean_path.header.stamp.sec = 0
        clean_path.header.stamp.nanosec = 0
        clean_path.poses = path.poses
        for pose in clean_path.poses:
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0

        self.path_pub.publish(clean_path)

        if self.current_candidate is None or self.current_execution_xy is None:
            self.busy = False
            return

        distance_m = self.current_candidate.distance_cells * float(
            self.latest_map.info.resolution
        )
        self.get_logger().warning(
            f'Executing {self.mode} MapEx target: '
            f'x={self.current_execution_xy[0]:.2f}, '
            f'y={self.current_execution_xy[1]:.2f}, '
            f'frontier_score={distance_m:.2f} m'
        )

        goal = FollowPath.Goal()
        goal.path = clean_path
        goal.controller_id = ''
        goal.goal_checker_id = ''
        goal.progress_checker_id = ''

        follow_future = self.follower.send_goal_async(goal)
        follow_future.add_done_callback(self._on_follow_goal_response)

    def _advance_after_plan_failure(self) -> None:
        if self.mode == 'exact':
            self._try_next_exact_candidate()
        else:
            self._try_next_fallback_point()

    def _on_follow_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'FollowPath request failed: {exc}')
            self._mark_execution_failure()
            return

        if not handle.accepted:
            self.get_logger().warning('FollowPath rejected the validated path.')
            self._mark_execution_failure()
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_follow_result)

    def _on_follow_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'FollowPath result failed: {exc}')
            self._mark_execution_failure()
            return

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                f'{self.mode} target reached. Recomputing MapEx frontiers.'
            )
            self.execution_failed_cells.clear()
            self.current_candidate = None
            self.current_execution_xy = None
            self.busy = False
            self.next_allowed_time_s = self._now_s() + POST_GOAL_SETTLE_S
            return

        self.get_logger().warning(
            f'FollowPath failed with status={int(wrapped.status)}; '
            'temporarily skipping this frontier.'
        )
        self._mark_execution_failure()

    def _mark_execution_failure(self) -> None:
        if self.current_candidate is not None:
            self.execution_failed_cells.add(
                (self.current_candidate.row, self.current_candidate.col)
            )
        self.current_candidate = None
        self.current_execution_xy = None
        self.busy = False
        self.next_allowed_time_s = self._now_s() + RETRY_PERIOD_S


def main() -> None:
    rclpy.init()
    node = MapExNearestExplorer()
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
