#!/usr/bin/env python3
"""Minimal nearest-frontier exploration using Euclidean distance.

Pipeline:
    /map -> frontier cells -> 8-connected frontier regions
    -> one representative per region -> reject representatives < 0.5 m
    -> sort remaining representatives by Euclidean distance
    -> Nav2 ComputePathToPose in nearest-first order
    -> skip candidates with no valid path
    -> Nav2 NavigateToPose to the nearest planner-reachable frontier

RViz visualization uses the Hospital RViz config's existing displays:
- /frontier/goals_markers : eligible frontier representatives + selected goal
- /frontier_selected_path : selected Nav2 validation/global path

No LaMa, information gain, path-length ranking, recorder, or MapEx prediction.
"""

from collections import deque
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MIN_REGION_SIZE = 10
MIN_DISTANCE_THRESHOLD = 0.5
MAP_FRAME = "map"
ROBOT_FRAME = "base_link"


class NearestEuclideanFrontier(Node):
    def __init__(self):
        super().__init__("nearest_euclidean_frontier")

        self.map_msg = None
        self.goal_active = False
        self.planning_active = False
        self.pending_candidates = []
        self.pending_index = 0

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(Path, "/plan", self.plan_callback, 10)

        self.goals_pub = self.create_publisher(
            MarkerArray,
            "/frontier/goals_markers",
            10,
        )
        self.path_pub = self.create_publisher(
            Path,
            "/frontier_selected_path",
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
        )
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(1.0, self.exploration_step)

        self.get_logger().info(
            "Nearest Euclidean Frontier started "
            f"(min distance={MIN_DISTANCE_THRESHOLD:.2f} m, "
            "planner reachability check=enabled)"
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def plan_callback(self, msg: Path):
        self.path_pub.publish(msg)

    @staticmethod
    def frontier_mask(grid: np.ndarray) -> np.ndarray:
        height, width = grid.shape
        mask = np.zeros((height, width), dtype=bool)

        for row in range(height):
            for col in range(width):
                if grid[row, col] != 0:
                    continue

                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue

                        nr = row + dr
                        nc = col + dc
                        if 0 <= nr < height and 0 <= nc < width and grid[nr, nc] < 0:
                            mask[row, col] = True
                            break
                    if mask[row, col]:
                        break

        return mask

    @staticmethod
    def frontier_regions(mask: np.ndarray):
        height, width = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        regions = []

        for row in range(height):
            for col in range(width):
                if not mask[row, col] or visited[row, col]:
                    continue

                queue = deque([(row, col)])
                visited[row, col] = True
                region = []

                while queue:
                    r, c = queue.popleft()
                    region.append((r, c))

                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue

                            nr = r + dr
                            nc = c + dc
                            if (
                                0 <= nr < height
                                and 0 <= nc < width
                                and mask[nr, nc]
                                and not visited[nr, nc]
                            ):
                                visited[nr, nc] = True
                                queue.append((nr, nc))

                if len(region) > MIN_REGION_SIZE:
                    regions.append(region)

        return regions

    @staticmethod
    def representative(region):
        mean_row = sum(cell[0] for cell in region) / len(region)
        mean_col = sum(cell[1] for cell in region) / len(region)

        return min(
            region,
            key=lambda cell: (cell[0] - mean_row) ** 2 + (cell[1] - mean_col) ** 2,
        )

    def cell_to_world(self, row: int, col: int):
        info = self.map_msg.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                ROBOT_FRAME,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Waiting for TF map -> base_link: {exc}")
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def publish_goal_markers(self, candidates, selected=None):
        """Green = eligible candidates, small red = planner-reachable selected."""
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        now = self.get_clock().now().to_msg()

        for index, candidate in enumerate(candidates):
            _distance, x, y, _row, _col, _region_size = candidate

            marker = Marker()
            marker.header.frame_id = MAP_FRAME
            marker.header.stamp = now
            marker.ns = "frontier_candidates"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.10
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.18
            marker.scale.y = 0.18
            marker.scale.z = 0.18
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 0.95
            markers.markers.append(marker)

        if selected is not None:
            _distance, x, y, _row, _col, _region_size = selected

            chosen = Marker()
            chosen.header.frame_id = MAP_FRAME
            chosen.header.stamp = now
            chosen.ns = "selected_frontier"
            chosen.id = 0
            chosen.type = Marker.SPHERE
            chosen.action = Marker.ADD
            chosen.pose.position.x = x
            chosen.pose.position.y = y
            chosen.pose.position.z = 0.12
            chosen.pose.orientation.w = 1.0
            chosen.scale.x = 0.24
            chosen.scale.y = 0.24
            chosen.scale.z = 0.24
            chosen.color.r = 1.0
            chosen.color.g = 0.1
            chosen.color.b = 0.1
            chosen.color.a = 1.0
            markers.markers.append(chosen)

        self.goals_pub.publish(markers)

    def clear_goal_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self.goals_pub.publish(markers)

    def exploration_step(self):
        if self.map_msg is None or self.goal_active or self.planning_active:
            return

        robot = self.robot_position()
        if robot is None:
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        grid = np.asarray(self.map_msg.data, dtype=np.int16).reshape(height, width)

        mask = self.frontier_mask(grid)
        regions = self.frontier_regions(mask)

        if not regions:
            self.clear_goal_markers()
            self.get_logger().info("No frontier left. Exploration complete.")
            return

        robot_x, robot_y = robot
        candidates = []

        for region in regions:
            row, col = self.representative(region)
            x, y = self.cell_to_world(row, col)
            distance = math.hypot(x - robot_x, y - robot_y)

            if distance < MIN_DISTANCE_THRESHOLD:
                continue

            candidates.append((distance, x, y, row, col, len(region)))

        if not candidates:
            self.clear_goal_markers()
            self.get_logger().info(
                f"No frontier at least {MIN_DISTANCE_THRESHOLD:.2f} m from robot."
            )
            return

        candidates.sort(key=lambda item: item[0])
        self.publish_goal_markers(candidates)

        self.pending_candidates = candidates
        self.pending_index = 0
        self.planning_active = True
        self.validate_next_candidate()

    def validate_next_candidate(self):
        if not self.planning_active:
            return

        if self.pending_index >= len(self.pending_candidates):
            self.get_logger().warn(
                "No Nav2-reachable frontier in the current candidate sweep."
            )
            self.clear_planning_state()
            return

        if not self.planner_client.server_is_ready():
            self.get_logger().warn("ComputePathToPose action server is not ready")
            self.clear_planning_state()
            return

        distance, x, y, _row, _col, region_size = self.pending_candidates[
            self.pending_index
        ]

        self.get_logger().info(
            f"Checking frontier {self.pending_index + 1}/{len(self.pending_candidates)}: "
            f"x={x:.2f}, y={y:.2f}, distance={distance:.2f} m, "
            f"region={region_size} cells"
        )

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = MAP_FRAME
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = ""
        goal.use_start = False

        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(self.plan_goal_response_callback)

    def plan_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f"ComputePathToPose send failed: {exc}")
            self.try_next_candidate()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Planner rejected frontier rank {self.pending_index + 1}; skipping it."
            )
            self.try_next_candidate()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.plan_result_callback)

    def plan_result_callback(self, future):
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = wrapped.result
            path = result.path
            path_ok = status == GoalStatus.STATUS_SUCCEEDED and bool(path.poses)
        except Exception as exc:
            self.get_logger().warn(f"ComputePathToPose result failed: {exc}")
            self.try_next_candidate()
            return

        candidate = self.pending_candidates[self.pending_index]
        distance, x, y, _row, _col, region_size = candidate

        if not path_ok:
            error_code = getattr(result, "error_code", None)
            detail = f"status={status}, poses={len(path.poses)}"
            if error_code is not None:
                detail += f", error_code={error_code}"
            self.get_logger().warn(
                f"No valid path to frontier x={x:.2f}, y={y:.2f} "
                f"({detail}); dropping this goal and checking the next frontier."
            )
            self.try_next_candidate()
            return

        self.path_pub.publish(path)
        self.publish_goal_markers(self.pending_candidates, candidate)
        self.get_logger().info(
            f"Selected nearest reachable frontier: x={x:.2f}, y={y:.2f}, "
            f"distance={distance:.2f} m, region={region_size} cells"
        )

        self.clear_planning_state()
        self.send_goal(x, y)

    def try_next_candidate(self):
        if not self.planning_active:
            return
        self.pending_index += 1
        self.validate_next_candidate()

    def clear_planning_state(self):
        self.planning_active = False
        self.pending_candidates = []
        self.pending_index = 0

    def send_goal(self, x: float, y: float):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateToPose action server is not ready")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        self.goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f"NavigateToPose send failed: {exc}")
            self.goal_active = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.goal_active = False
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            result = wrapped.result
            error_code = getattr(result, "error_code", None)
        except Exception as exc:
            self.get_logger().warn(f"NavigateToPose result failed: {exc}")
            self.goal_active = False
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached")
        else:
            detail = f"status={status}"
            if error_code is not None:
                detail += f", error_code={error_code}"
            self.get_logger().warn(f"Goal finished with {detail}")

        self.goal_active = False


def main(args=None):
    rclpy.init(args=args)
    node = NearestEuclideanFrontier()

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
