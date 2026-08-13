"""Read-only frontier detector for the TurtleBot4 exploration baseline."""

from collections import deque
from typing import Iterable

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


FREE = 0
UNKNOWN = -1


def _neighbors(index: int, width: int, height: int) -> Iterable[int]:
    x = index % width
    y = index // width
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            nx = x + dx
            ny = y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield ny * width + nx


def detect_frontier_cells(data: list[int], width: int, height: int) -> set[int]:
    """Return free cells that touch at least one unknown cell."""
    frontiers: set[int] = set()
    for index, value in enumerate(data):
        if value != FREE:
            continue
        if any(data[n] == UNKNOWN for n in _neighbors(index, width, height)):
            frontiers.add(index)
    return frontiers


def cluster_frontiers(
    frontier_cells: set[int], width: int, height: int, min_cluster_size: int
) -> list[list[int]]:
    """Group connected frontier cells using 8-connectivity."""
    remaining = set(frontier_cells)
    clusters: list[list[int]] = []

    while remaining:
        seed = remaining.pop()
        queue = deque([seed])
        cluster = [seed]

        while queue:
            current = queue.popleft()
            for neighbor in _neighbors(current, width, height):
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    queue.append(neighbor)
                    cluster.append(neighbor)

        if len(cluster) >= min_cluster_size:
            clusters.append(cluster)

    return clusters


class FrontierDetector(Node):
    """Subscribe to an occupancy grid and report frontier statistics only."""

    def __init__(self) -> None:
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('min_cluster_size', 5)

        map_topic = self.get_parameter('map_topic').value
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._last_summary: tuple[int, int] | None = None
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, qos)
        self.get_logger().info(f'Listening for occupancy grids on {map_topic}')

    def _on_map(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0 or len(msg.data) != width * height:
            self.get_logger().warning('Ignoring invalid OccupancyGrid dimensions')
            return

        min_cluster_size = int(self.get_parameter('min_cluster_size').value)
        frontier_cells = detect_frontier_cells(list(msg.data), width, height)
        clusters = cluster_frontiers(
            frontier_cells, width, height, max(1, min_cluster_size)
        )
        summary = (len(frontier_cells), len(clusters))
        if summary != self._last_summary:
            self.get_logger().info(
                f'Frontier cells: {summary[0]} | valid clusters: {summary[1]}'
            )
            self._last_summary = summary


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
