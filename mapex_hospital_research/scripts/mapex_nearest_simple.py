#!/usr/bin/env python3
"""Minimal MapEx nearest-frontier exploration node for Hospital.

This file intentionally keeps only the frontier-selection logic used by the
pinned MapEx implementation:

  source: castacks/MapEx
  commit: 53636bd1c79153acc3c74a532837d78c926bae5e
  source file: scripts/sim_utils.py, FrontierPlanner(score_mode='nearest')

Selection semantics reproduced here:
  1. free cell: observed-map value == 0
  2. unknown cell: observed-map value == 0.5
  3. frontier: free cell adjacent to unknown in the 8-neighbourhood
  4. frontier regions: 8-connected components
  5. keep a region only when size > 10 cells
  6. representative: actual region cell closest to arithmetic mean row/col
  7. score: Euclidean grid distance from robot to representative
  8. choose the minimum-distance representative

ROS occupancy mapping used to construct the MapEx-style observed map:
  /map value == 0  -> free (0.0)
  /map value < 0   -> unknown (0.5)
  /map value > 0   -> occupied (1.0)

The original MapEx post-ranking <1 m rejection is intentionally NOT included
because this node is meant to isolate the nearest-frontier selection rule itself.

Execution is deliberately thin and separate from scoring:
  - candidates remain ordered only by MapEx Euclidean score;
  - Nav2 ComputePathToPose is used only as a reachability check, in rank order;
  - the exact validated path is then executed with FollowPath;
  - path timestamps are zeroed before execution to use latest TF and avoid the
    future-extrapolation issue seen in previous Hospital pilots.

No recorder, benchmark clock, completion window, prediction, variance, IG,
MapEx <1 m filter, cooldown model, or research validator is used here.
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
FALLBACK_STANDOFF_M = 0.30


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

        map_topic = str(self.get_parameter("map_topic").value)
        self.robot_frame = str(self.get_parameter("robot_frame").value)
        planner_action = str(self.get_parameter("planner_action").value)
        follow_path_action = str(self.get_parameter("follow_path_action").value)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.frontier_marker_pub = self.create_publisher(
            MarkerArray, "/frontier_markers", 10
        )
        self.path_pub = self.create_publisher(Path, "/frontier_selected_path", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.planner = ActionClient(self, ComputePathToPose, planner_action)
        self.follower = ActionClient(self, FollowPath, follow_path_action)

        self.latest_map: OccupancyGrid | None = None
        self.busy = False
        self.current_candidates: list[FrontierCandidate] = []
        self.current_candidate_index = 0
        self.current_candidate: FrontierCandidate | None = None
        self.using_standoff_fallback = False
        self.execution_failed_cells: set[tuple[int, int]] = set()
        self.next_allowed_time_s = 0.0
        self.no_frontier_notice_shown = False
        self.wait_notice_shown = False

        self.create_timer(RETRY_PERIOD_S, self._tick)

        self.get_logger().warning(
            "MAPEX NEAREST SIMPLE ACTIVE: official MapEx frontier geometry + "
            "Euclidean nearest score only; no <1m rejection, no IG/prediction."
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

    @staticmethod
    def _mapex_observed_map(msg: OccupancyGrid) -> np.ndarray:
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
        self,
        msg: OccupancyGrid,
        robot_x: float,
        robot_y: float,
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
            row = int(center_array[index, 0])
            col = int(center_array[index, 1])
            if (row, col) in self.execution_failed_cells:
                continue
            ranked.append(
                FrontierCandidate(
                    row=row,
                    col=col,
                    distance_cells=float(distances[index]),
                )
            )
        return ranked

    def _publish_frontier_markers(
        self, candidates: list[FrontierCandidate]
    ) -> None:
        if self.latest_map is None:
            return

        marker_array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        if candidates:
            msg = self.latest_map
            res = float(msg.info.resolution)

            cells = Marker()
            cells.header.frame_id = msg.header.frame_id or "map"
            cells.header.stamp = self.get_clock().now().to_msg()
            cells.ns = "mapex_frontier_candidates"
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
            cells.color.a = 1.0

            for candidate in candidates:
                x, y = self._grid_to_world(candidate.row, candidate.col, msg)
                point = Point()
                point.x = x
                point.y = y
                point.z = 0.05
                cells.points.append(point)

            marker_array.markers.append(cells)

        self.frontier_marker_pub.publish(marker_array)

    def _tick(self) -> None:
        if self.busy or self.latest_map is None:
            return
        if self._now_s() < self.next_allowed_time_s:
            return

        if not self.planner.server_is_ready() or not self.follower.server_is_ready():
            if not self.wait_notice_shown:
                self.wait_notice_shown = True
                self.get_logger().info(
                    "Waiting for Nav2 /compute_path_to_pose and /follow_path actions"
                )
            return
        self.wait_notice_shown = False

        msg = self.latest_map
        robot_pose = self._robot_pose_in_map(msg)
        if robot_pose is None:
            return

        robot_x, robot_y, _robot_yaw = robot_pose
        candidates = self._rank_candidates(msg, robot_x, robot_y)

        if not candidates:
            self._publish_frontier_markers([])
            if not self.no_frontier_notice_shown:
                self.no_frontier_notice_shown = True
                self.get_logger().warning(
                    "No MapEx frontier region >10 cells is currently available."
                )
            return

        self.no_frontier_notice_shown = False
        self.current_candidates = candidates
        self.current_candidate_index = 0
        self.using_standoff_fallback = False
        self._publish_frontier_markers(candidates)
        self.busy = True

        nearest = candidates[0]
        distance_m = nearest.distance_cells * float(msg.info.resolution)
        self.get_logger().warning(
            f"MapEx nearest frontier: candidates={len(candidates)}, "
            f"row={nearest.row}, col={nearest.col}, distance={distance_m:.2f} m"
        )
        self._plan_current_candidate()

    def _plan_current_candidate(self) -> None:
        if self.latest_map is None:
            self.busy = False
            return

        if self.current_candidate_index >= len(self.current_candidates):
            if not self.using_standoff_fallback:
                self.using_standoff_fallback = True
                self.current_candidate_index = 0
                self.get_logger().warning(
                    "No exact MapEx frontier has a Nav2 path; retrying ranked "
                    "frontiers with a 0.30 m inward execution goal."
                )
            else:
                self.get_logger().warning(
                    "No ranked MapEx frontier has a Nav2 path right now; retrying later."
                )
                self.busy = False
                self.next_allowed_time_s = self._now_s() + RETRY_PERIOD_S
                return

        candidate = self.current_candidates[self.current_candidate_index]
        self.current_candidate = candidate
        self.current_candidate_index += 1

        msg = self.latest_map
        robot_pose = self._robot_pose_in_map(msg)
        if robot_pose is None:
            self.busy = False
            return

        x, y = self._grid_to_world(candidate.row, candidate.col, msg)
        robot_x, robot_y, robot_yaw = robot_pose

        if self.using_standoff_fallback:
            dx = robot_x - x
            dy = robot_y - y
            norm = math.hypot(dx, dy)
            if norm > FALLBACK_STANDOFF_M:
                x += FALLBACK_STANDOFF_M * dx / norm
                y += FALLBACK_STANDOFF_M * dy / norm

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
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"ComputePathToPose request failed: {exc}")
            self._plan_current_candidate()
            return

        if not handle.accepted:
            self.get_logger().info("Planner rejected candidate; trying next nearest.")
            self._plan_current_candidate()
            return

        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_plan_result)

    def _on_plan_result(self, future) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"ComputePathToPose result failed: {exc}")
            self._plan_current_candidate()
            return

        path = wrapped.result.path
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not path.poses:
            self.get_logger().info("Candidate has no successful Nav2 path; trying next nearest.")
            self._plan_current_candidate()
            return

        candidate = self.current_candidate
        if candidate is None or self.latest_map is None:
            self.busy = False
            return

        # Execute exactly the path that passed reachability validation.  Using zero
        # timestamps asks TF for the latest available transform instead of a pose a
        # few milliseconds ahead of slam_toolbox's map->odom publication.
        clean_path = Path()
        clean_path.header = path.header
        clean_path.header.stamp.sec = 0
        clean_path.header.stamp.nanosec = 0
        clean_path.poses = path.poses
        for pose in clean_path.poses:
            pose.header.stamp.sec = 0
            pose.header.stamp.nanosec = 0

        self.path_pub.publish(clean_path)

        x, y = self._grid_to_world(candidate.row, candidate.col, self.latest_map)
        distance_m = candidate.distance_cells * float(self.latest_map.info.resolution)
        if self.using_standoff_fallback:
            goal_pose = clean_path.poses[-1].pose.position
            self.get_logger().warning(
                f"Executing 0.30 m fallback for MapEx frontier: "
                f"frontier=({x:.2f}, {y:.2f}), "
                f"goal=({goal_pose.x:.2f}, {goal_pose.y:.2f}), "
                f"euclidean={distance_m:.2f} m"
            )
        else:
            self.get_logger().warning(
                f"Executing MapEx nearest reachable frontier: "
                f"x={x:.2f}, y={y:.2f}, euclidean={distance_m:.2f} m"
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
        except Exception as exc:  # noqa: BLE001
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
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"FollowPath result failed: {exc}")
            self._mark_execution_failure_and_continue()
            return

        if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warning("Frontier reached. Recomputing nearest MapEx frontier.")
            self.execution_failed_cells.clear()
            self.current_candidate = None
            self.busy = False
            self.next_allowed_time_s = self._now_s() + POST_GOAL_SETTLE_S
            return

        self.get_logger().warning(
            f"FollowPath failed with status={int(wrapped.status)}; trying another frontier."
        )
        self._mark_execution_failure_and_continue()

    def _mark_execution_failure_and_continue(self) -> None:
        if self.current_candidate is not None:
            self.execution_failed_cells.add(
                (self.current_candidate.row, self.current_candidate.col)
            )
        self.current_candidate = None
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
