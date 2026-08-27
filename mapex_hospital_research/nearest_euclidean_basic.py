#!/usr/bin/env python3
"""Minimal nearest-frontier exploration using Euclidean distance.

Pipeline:
    /map -> frontier cells -> 8-connected frontier regions
    -> one representative per region -> nearest by Euclidean distance
    -> Nav2 NavigateToPose

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
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


MIN_REGION_SIZE = 10
MAP_FRAME = "map"
ROBOT_FRAME = "base_link"


class NearestEuclideanFrontier(Node):
    def __init__(self):
        super().__init__("nearest_euclidean_frontier")

        self.map_msg = None
        self.goal_active = False

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(1.0, self.exploration_step)

        self.get_logger().info("Nearest Euclidean Frontier started")

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

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
        distance, x, y, row, col, region_size = min(candidates, key=lambda item: item[0])

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
