#!/usr/bin/env python3
"""Minimal nearest-frontier exploration using Euclidean distance.

Pipeline:
    /map -> frontier cells -> 8-connected frontier regions
    -> one representative per region -> nearest by Euclidean distance
    -> Nav2 NavigateToPose

RViz visualization uses the Hospital RViz config's existing displays:
- /frontier/goals_markers : all frontier representatives + selected goal
- /frontier_selected_path : current Nav2 global path

This file intentionally keeps the policy simple:
- free cell: occupancy == 0
- unknown cell: occupancy < 0
- frontier cell: free cell adjacent to unknown in the 8-neighborhood
- frontier regions: 8-connected
- keep regions with size > 10 cells
- representative: frontier cell closest to the arithmetic mean of the region
- ranking: straight-line Euclidean distance from robot to representative

No LaMa, information gain, path-length ranking, recorder, or MapEx prediction.
"""

from collections import deque
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MIN_REGION_SIZE = 10
MAP_FRAME = "map"
ROBOT_FRAME = "base_link"


class NearestEuclideanFrontier(Node):
    def __init__(self):
        super().__init__("nearest_euclidean_frontier")

        self.map_msg = None
        self.goal_active = False

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(Path, "/plan", self.plan_callback, 10)

        # These topics are already enabled automatically by
        # ros2_ws/src/frontier_exploration/rviz/hospital_exploration.rviz.
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

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(1.0, self.exploration_step)

        self.get_logger().info("Nearest Euclidean Frontier started")
        self.get_logger().info(
            "RViz auto topics: /frontier/goals_markers, /frontier_selected_path"
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def plan_callback(self, msg: Path):
        """Republish Nav2's current global plan to the RViz frontier path topic."""
        self.path_pub.publish(msg)

    @staticmethod
    def frontier_mask(grid: np.ndarray) -> np.ndarray:
        """Return mask of free cells touching unknown cells in 8-neighborhood."""
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
                        if 0 <= nr < height and 0 <= nc < width:
                            if grid[nr, nc] < 0:
                                mask[row, col] = True
                                break
                    if mask[row, col]:
                        break

        return mask

    @staticmethod
    def frontier_regions(mask: np.ndarray):
        """Group frontier cells with 8-connectivity."""
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
        """Choose the frontier cell closest to the region arithmetic mean."""
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

    def publish_goal_markers(self, candidates, selected):
        """Publish green candidate goals and the selected goal in red."""
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        now = self.get_clock().now().to_msg()

        for index, candidate in enumerate(candidates):
            distance, x, y, _row, _col, region_size = candidate

            marker = Marker()
            marker.header.frame_id = MAP_FRAME
            marker.header.stamp = now
            marker.ns = "frontier_candidates"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.15
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.28
            marker.scale.y = 0.28
            marker.scale.z = 0.28
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 0.95
            markers.markers.append(marker)

            text = Marker()
            text.header.frame_id = MAP_FRAME
            text.header.stamp = now
            text.ns = "frontier_candidate_labels"
            text.id = index
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.55
            text.pose.orientation.w = 1.0
            text.scale.z = 0.22
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"F{index}  d={distance:.2f}m  n={region_size}"
            markers.markers.append(text)

        distance, x, y, _row, _col, _region_size = selected

        chosen = Marker()
        chosen.header.frame_id = MAP_FRAME
        chosen.header.stamp = now
        chosen.ns = "selected_frontier"
        chosen.id = 0
        chosen.type = Marker.SPHERE
        chosen.action = Marker.ADD
        chosen.pose.position.x = x
        chosen.pose.position.y = y
        chosen.pose.position.z = 0.20
        chosen.pose.orientation.w = 1.0
        chosen.scale.x = 0.50
        chosen.scale.y = 0.50
        chosen.scale.z = 0.50
        chosen.color.r = 1.0
        chosen.color.g = 0.1
        chosen.color.b = 0.1
        chosen.color.a = 1.0
        markers.markers.append(chosen)

        chosen_text = Marker()
        chosen_text.header.frame_id = MAP_FRAME
        chosen_text.header.stamp = now
        chosen_text.ns = "selected_frontier_label"
        chosen_text.id = 0
        chosen_text.type = Marker.TEXT_VIEW_FACING
        chosen_text.action = Marker.ADD
        chosen_text.pose.position.x = x
        chosen_text.pose.position.y = y
        chosen_text.pose.position.z = 0.85
        chosen_text.pose.orientation.w = 1.0
        chosen_text.scale.z = 0.28
        chosen_text.color.r = 1.0
        chosen_text.color.g = 0.2
        chosen_text.color.b = 0.2
        chosen_text.color.a = 1.0
        chosen_text.text = f"SELECTED  d={distance:.2f}m"
        markers.markers.append(chosen_text)

        self.goals_pub.publish(markers)

    def clear_goal_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self.goals_pub.publish(markers)

    def exploration_step(self):
        if self.map_msg is None or self.goal_active:
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
            candidates.append((distance, x, y, row, col, len(region)))

        # Nearest Frontier = minimum straight-line Euclidean distance.
        selected = min(candidates, key=lambda item: item[0])
        distance, x, y, row, col, region_size = selected

        self.publish_goal_markers(candidates, selected)

        self.get_logger().info(
            f"Selected nearest frontier: x={x:.2f}, y={y:.2f}, "
            f"distance={distance:.2f} m, region={region_size} cells"
        )

        self.send_goal(x, y)

    def send_goal(self, x: float, y: float):
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateToPose action server is not ready")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y

        # Neutral orientation. The frontier policy itself only selects position.
        goal.pose.pose.orientation.w = 1.0

        self.goal_active = True
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.goal_active = False
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Goal reached")
        else:
            self.get_logger().warn(f"Goal finished with status={status}")

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
