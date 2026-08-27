#!/usr/bin/env python3
"""Minimal MapEx nearest-frontier exploration node for Hospital.

Frontier selection is a direct ROS adaptation of the pinned MapEx implementation:
  castacks/MapEx @ 53636bd1c79153acc3c74a532837d78c926bae5e
  scripts/sim_utils.py -> FrontierPlanner(score_mode="nearest")

MapEx selection semantics kept here:
  1. free = 0, unknown = 0.5, occupied = 1
  2. frontier = free cell adjacent to unknown in the 8-neighbourhood
  3. frontier regions are 8-connected
  4. keep only regions with size > 10 cells
  5. representative = real frontier cell nearest the arithmetic mean row/col
  6. score = Euclidean robot-to-representative distance
  7. rank from nearest to farthest

Hospital execution adaptation:
  - first try every ranked exact MapEx frontier with ComputePathToPose
  - only if ALL exact candidates fail planning, enable a standoff fallback
  - fallback keeps the same MapEx ranking and moves the execution goal inward
    toward the robot by about 0.30 m (searching farther inward only if needed)
  - execute the already validated path with FollowPath
  - zero path timestamps to use latest-TF semantics

RViz:
  - /frontier_markers: all MapEx representative cells, selected frontier,
    and fallback execution point when active
  - /frontier_selected_path: the exact path sent to FollowPath

The original MapEx post-ranking <1 m rejection is intentionally not included.
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


class MapExNearestSimple(Node):
    def __init__(self) -> None:
        super().__init__("mapex_nearest_simple")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("planner_action", "/compute_path_to_pose")
        self.declare_parameter("follow_path_action", "/follow_path")
        self.declare_parameter("marker_topic", "/frontier_markers")
        self.declare_parameter("path_topic", "/frontier_selected_path")
        self.declare_parameter("fallback_standoff_m", 0.30)
        self.declare_parameter("fallback_max_standoff_m", 0.60)
        self.declare_parameter("fallback_step_m", 0.05)

        map_topic = str(self.get_parameter("map_topic").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        planner_action = str(self.get_parameter("planner_action").value)
        follow_path_action = str(self.get_parameter("follow_path_action").value)
        marker_topic = str(self.get_parameter("marker_topic").value)
        path_topic = str(self.get_parameter("path_topic").value)
        self.fallback_standoff_m = float(self.get_parameter("fallback_standoff_m").value)
        self.fallback_max_standoff_m = float(self.get_parameter("fallback_max_standoff_m").value)
        self.fallback_step_m = float(self.get_parameter("fallback_step_m").value)

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
        self.current_candidate_index = 0
        self.current_candidate: FrontierCandidate | None = None
        self.current_mode = "exact"
        self.current_execution_xy: tuple[float, float] | None = None
        self.execution_failed_cells: set[tuple[int, int]] = set()
        self.next_allowed_time_s = 0.0
        self.no_frontier_notice_shown = False
        self.wait_notice_shown = False

        self.create_timer(RETRY_PERIOD_S, self._tick)

        self.get_logger().warning(
            "MAPEX NEAREST SIMPLE ACTIVE: exact MapEx nearest first; "
            "standoff fallback only after every exact candidate fails planning."
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    @staticmethod
    def _origin_yaw(msg: OccupancyGrid) -> float:
        q = msg.info.origin.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _robot_pose_in_map(self, msg: OccupancyGrid) -> tuple[float, float, float] | None:
        frame = msg.header.frame_id or "map"
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.robot_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warning(f"Waiting for TF {frame}->{self.robot_frame}: {exc}")
            return None

        t = tf.transform.translation
        yaw = self._yaw_from_quaternion(tf.transform.rotation)
        return float(t.x), float(t.y), yaw

    def _world_to_grid(self, x: float, y: float, msg: OccupancyGrid) -> tuple[float, float]:
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

    def _grid_to_world(self, row: int, col: int, msg: OccupancyGrid) -> tuple[float, float]:
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

    @staticmethod
    def _raw_map(msg: OccupancyGrid) -> np.ndarray:
        return np.asarray(msg.data, dtype=np.int16).reshape(int(msg.info.height), int(msg.info.width))

    @classmethod
    def _mapex_observed_map(cls, msg: OccupancyGrid) -> np.ndarray:
        raw = cls._raw_map(msg)
        obs = np.ones(raw.shape, dtype=np.float32)
        obs[raw == 0] = 0.0
        obs[raw < 0] = 0.5
        obs[raw > 0] = 1.0
        return obs

    @staticmethod
    def _mapex_frontier_centers(obs_map: np.ndarray) -> list[np.ndarray]:
        """Direct port of MapEx FrontierPlanner.get_frontier_centers_given_obs_map."""
        kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]])

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

    def _rank_candidates(self, msg: OccupancyGrid, robot_x: float, robot_y: float) -> list[FrontierCandidate]:
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
            row = int(center_array[index, 0])
            col = int(center_array[index, 1])
            if (row, col) in self.execution_failed_cells:
                continue
            ranked.append(FrontierCandidate(row=row, col=col, distance_cells=float(distances[index])))
        return ranked

    def _publish_frontier_markers(
        self,
        candidates: list[FrontierCandidate],
        selected: FrontierCandidate | None = None,
        execution_xy: tuple[float, float] | None = None,
        mode: str = "exact",
    ) -> None:
        if self.latest_map is None:
            return

        msg = self.latest_map
        frame = msg.header.frame_id or "map"
        res = float(msg.info.resolution)

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        cells = Marker()
        cells.header.frame_id = frame
        cells.header.stamp = self.get_clock().now().to_msg()
        cells.ns = "mapex_frontier_cells"
        cells.id = 0
        cells.type = Marker.CUBE_LIST
        cells.action = Marker.ADD
        cells.pose.orientation.w = 1.0
        cells.scale.x = res
        cells.scale.y = res
        cells.scale.z = max(0.03, res * 0.5)
        cells.color.r = 0.0
        cells.color.g = 0.8
        cells.color.b = 1.0
        cells.color.a = 0.95
        for candidate in candidates:
            x, y = self._grid_to_world(candidate.row, candidate.col, msg)
            p = Point(x=x, y=y, z=0.06)
            cells.points.append(p)
        markers.markers.append(cells)

        if selected is not None:
            sx, sy = self._grid_to_world(selected.row, selected.col, msg)
            chosen = Marker()
            chosen.header.frame_id = frame
            chosen.header.stamp = self.get_clock().now().to_msg()
            chosen.ns = "mapex_selected_frontier"
            chosen.id = 1
            chosen.type = Marker.CUBE
            chosen.action = Marker.ADD
            chosen.pose.position.x = sx
            chosen.pose.position.y = sy
            chosen.pose.position.z = 0.11
            chosen.pose.orientation.w = 1.0
            chosen.scale.x = max(0.18, res * 2.0)
            chosen.scale.y = max(0.18, res * 2.0)
            chosen.scale.z = 0.12
            chosen.color.r = 1.0
            chosen.color.g = 0.2
            chosen.color.b = 0.0
            chosen.color.a = 1.0
            markers.markers.append(chosen)

        if execution_xy is not None and mode == "standoff":
            target = Marker()
            target.header.frame_id = frame
            target.header.stamp = self.get_clock().now().to_msg()
            target.ns = "mapex_standoff_goal"
            target.id = 2
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose.position.x = execution_xy[0]
            target.pose.position.y = execution_xy[1]
            target.pose.position.z = 0.12
            target.pose.orientation.w = 1.0
            target.scale.x = 0.20
            target.scale.y = 0.20
            target.scale.z = 0.20
            target.color.r = 0.1
            target.color.g = 1.0
            target.color.b = 0.1
            target.color.a = 1.0
            markers.markers.append(target)

        self.marker_pub.publish(markers)

    def _fallback_goal_for_candidate(
        self, candidate: FrontierCandidate, robot_x: float, robot_y: float
    ) -> tuple[float, float, float] | None:
        """Find a known-free cell about 0.30 m inward from the exact frontier."""
        if self.latest_map is None:
            return None

        msg = self.latest_map
        raw = self._raw_map(msg)
        fx, fy = self._grid_to_world(candidate.row, candidate.col, msg)
        dx = robot_x - fx
        dy = robot_y - fy
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None

        ux = dx / norm
        uy = dy / norm
        standoff = self.fallback_standoff_m

        while standoff <= self.fallback_max_standoff_m + 1e-9:
            x = fx + ux * standoff
            y = fy + uy * standoff
            row_f, col_f = self._world_to_grid(x, y, msg)
            row = int(round(row_f))
            col = int(round(col_f))
            if 0 <= row < raw.shape[0] and 0 <= col < raw.shape[1] and raw[row, col] == 0:
                gx, gy = self._grid_to_world(row, col, msg)
                actual = math.hypot(gx - fx, gy - fy)
                return gx, gy, actual
            standoff += self.fallback_step_m

        return None

    def _tick(self) -> None:
        if self.busy or self.latest_map is None:
            return
        if self._now_s() < self.next_allowed_time_s:
            return

        if not self.planner.server_is_ready() or not self.follower.server_is_ready():
            if not self.wait_notice_shown:
                self.wait_notice_shown = True
                self.get_logger().info("Waiting for Nav2 /compute_path_to_pose and /follow_path actions")
            return
        self.wait_notice_shown = False

        msg = self.latest_map
        robot_pose = self._robot_pose_in_map(msg)
        if robot_pose is None:
            return

        robot_x, robot_y, _robot_yaw = robot_pose
        candidates = self._rank_candidates(msg, robot_x, robot_y)
        self._publish_frontier_markers(candidates)

        if not candidates:
            if not self.no_frontier_notice_shown:
                self.no_frontier_notice_shown = True
                self.get_logger().warning("No MapEx frontier region >10 cells is currently available.")
            return

        self.no_frontier_notice_shown = False
        self.current_candidates = candidates
        self.current_candidate_index = 0
        self.current_candidate = None
        self.current_mode = "exact"
        self.current_execution_xy = None
        self.busy = True

        nearest = candidates[0]
        distance_m = nearest.distance_cells * float(msg.info.resolution)
        self.get_logger().warning(
            f"MapEx nearest frontier: candidates={len(candidates)}, row={nearest.row}, "
            f"col={nearest.col}, distance={distance_m:.2f} m"
        )
        self._plan_current_candidate()

    def _plan_current_candidate(self) -> None:
        if self.latest_map is None:
            self.busy = False
            return

        if self.current_candidate_index >= len(self.current_candidates):
            if self.current_mode == "exact":
                self.current_mode = "standoff"
                self.current_candidate_index = 0
                self.current_candidate = None
                self.current_execution_xy = None
                self.get_logger().warning(
                    "All exact MapEx frontier candidates failed planning. "
                    "Starting ~0.30 m standoff fallback in the SAME MapEx rank order."
                )
            else:
                self.get_logger().warning(
                    "No exact or standoff MapEx frontier has a Nav2 path right now; "
                    "retrying after map/TF updates."
                )
                self.busy = False
                self.next_allowed_time_s = self._now_s() + RETRY_PERIOD_S
                return

        candidate = self.current_candidates[self.current_candidate_index]
        self.current_candidate_index += 1
        self.current_candidate = candidate

        msg = self.latest_map
        robot_pose = self._robot_pose_in_map(msg)
        if robot_pose is None:
            self.busy = False
            return

        robot_x, robot_y, robot_yaw = robot_pose
        fx, fy = self._grid_to_world(candidate.row, candidate.col, msg)

        if self.current_mode == "exact":
            x, y = fx, fy
            self.current_execution_xy = (x, y)
        else:
            fallback = self._fallback_goal_for_candidate(candidate, robot_x, robot_y)
            if fallback is None:
                self.get_logger().info("No known-free standoff cell for candidate; trying next candidate.")
                self._plan_current_candidate()
                return
            x, y, actual_standoff = fallback
            self.current_execution_xy = (x, y)
            self.get_logger().info(
                f"Standoff candidate for frontier ({fx:.2f}, {fy:.2f}) -> "
                f"goal ({x:.2f}, {y:.2f}), offset={actual_standoff:.2f} m"
            )

        self._publish_frontier_markers(
            self.current_candidates,
            selected=candidate,
            execution_xy=self.current_execution_xy,
            mode=self.current_mode,
        )

        pose = PoseStamped()
        pose.header.frame_id = msg.header.frame_id or "map"
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(robot_yaw * 0.5)
        pose.pose.orientation.w = math.cos(robot_yaw * 0.5)

        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.planner_id = ""
        goal.use_start = False

        future = self.planner.send_goal_async(goal)
        future.add_done_callback(self._on_plan_goal_response)

    def _on_plan_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f"ComputePathToPose request failed: {exc}")
            self._plan_current_candidate()
            return

        if not handle.accepted:
            self.get_logger().info(f"Planner rejected {self.current_mode} candidate; trying next.")
            self._plan_current_candidate()
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(f"ComputePathToPose result failed: {exc}")
            self._plan_current_candidate()
            return

        path = wrapped.result.path
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            self.get_logger().info(
                f"{self.current_mode.capitalize()} candidate has no successful Nav2 path; trying next."
            )
            self._plan_current_candidate()
            return

        candidate = self.current_candidate
        if candidate is None or self.latest_map is None:
            self.busy = False
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

        fx, fy = self._grid_to_world(candidate.row, candidate.col, self.latest_map)
        distance_m = candidate.distance_cells * float(self.latest_map.info.resolution)
        ex, ey = self.current_execution_xy or (fx, fy)

        self._publish_frontier_markers(
            self.current_candidates,
            selected=candidate,
            execution_xy=(ex, ey),
            mode=self.current_mode,
        )

        if self.current_mode == "exact":
            self.get_logger().warning(
                f"Executing EXACT MapEx frontier: x={fx:.2f}, y={fy:.2f}, euclidean={distance_m:.2f} m"
            )
        else:
            self.get_logger().warning(
                f"Executing STANDOFF for MapEx frontier: frontier=({fx:.2f}, {fy:.2f}), "
                f"goal=({ex:.2f}, {ey:.2f}), euclidean_rank={distance_m:.2f} m"
            )

        goal = FollowPath.Goal()
        goal.path = clean_path
        goal.controller_id = ""
        goal.goal_checker_id = ""
        goal.progress_checker_id = ""

        future = self.follower.send_goal_async(goal)
        future.add_done_callback(self._on_follow_goal_response)

    def _on_follow_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().warning(f"FollowPath request failed: {exc}")
            self._mark_execution_failure_and_continue()
            return

        if not handle.accepted:
            self.get_logger().warning("FollowPath rejected the validated path.")
            self._mark_execution_failure_and_continue()
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_follow_result)

    def _on_follow_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self.get_logger().warning(f"FollowPath result failed: {exc}")
            self._mark_execution_failure_and_continue()
            return

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning(
                f"{self.current_mode.upper()} goal reached. Recomputing nearest MapEx frontier."
            )
            self.execution_failed_cells.clear()
            self.current_candidate = None
            self.current_execution_xy = None
            self.busy = False
            self.next_allowed_time_s = self._now_s() + POST_GOAL_SETTLE_S
            return

        self.get_logger().warning(
            f"FollowPath failed with status={int(wrapped.status)} for {self.current_mode} goal; recomputing."
        )
        self._mark_execution_failure_and_continue()

    def _mark_execution_failure_and_continue(self) -> None:
        if self.current_candidate is not None:
            self.execution_failed_cells.add((self.current_candidate.row, self.current_candidate.col))
        self.current_candidate = None
        self.current_execution_xy = None
        self.busy = False
        self.next_allowed_time_s = self._now_s() + RETRY_PERIOD_S


def main() -> None:
    rclpy.init()
    node = MapExNearestSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
