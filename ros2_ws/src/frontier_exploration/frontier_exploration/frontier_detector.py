"""Read-only frontier detector with RViz visualization for TurtleBot4."""

from collections import deque
from math import atan2, cos, sin
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


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


def _origin_yaw(msg: OccupancyGrid) -> float:
    q = msg.info.origin.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def _cell_to_world(index: int, msg: OccupancyGrid) -> Point:
    width = msg.info.width
    resolution = msg.info.resolution
    cell_x = index % width
    cell_y = index // width

    local_x = (cell_x + 0.5) * resolution
    local_y = (cell_y + 0.5) * resolution
    yaw = _origin_yaw(msg)

    point = Point()
    point.x = (
        msg.info.origin.position.x
        + cos(yaw) * local_x
        - sin(yaw) * local_y
    )
    point.y = (
        msg.info.origin.position.y
        + sin(yaw) * local_x
        + cos(yaw) * local_y
    )
    point.z = 0.05
    return point


class FrontierDetector(Node):
    """Subscribe to an occupancy grid, detect frontiers, and publish RViz markers."""

    def __init__(self) -> None:
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/frontier_markers')
        self.declare_parameter('min_cluster_size', 5)

        map_topic = self.get_parameter('map_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._last_summary: tuple[int, int] | None = None
        self._marker_pub = self.create_publisher(
            MarkerArray, marker_topic, marker_qos
        )
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)

        self.get_logger().info(f'Listening for occupancy grids on {map_topic}')
        self.get_logger().info(f'Publishing RViz markers on {marker_topic}')

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

        self._publish_markers(msg, frontier_cells, clusters)

        summary = (len(frontier_cells), len(clusters))
        if summary != self._last_summary:
            self.get_logger().info(
                f'Frontier cells: {summary[0]} | valid clusters: {summary[1]}'
            )
            self._last_summary = summary

    def _publish_markers(
        self,
        msg: OccupancyGrid,
        frontier_cells: set[int],
        clusters: list[list[int]],
    ) -> None:
        frame_id = msg.header.frame_id or 'map'
        resolution = max(float(msg.info.resolution), 0.01)

        cells_marker = Marker()
        cells_marker.header.stamp = msg.header.stamp
        cells_marker.header.frame_id = frame_id
        cells_marker.ns = 'frontier_cells'
        cells_marker.id = 0
        cells_marker.type = Marker.POINTS
        cells_marker.action = Marker.ADD
        cells_marker.pose.orientation.w = 1.0
        cells_marker.scale.x = max(resolution * 0.8, 0.03)
        cells_marker.scale.y = max(resolution * 0.8, 0.03)
        cells_marker.color.r = 0.0
        cells_marker.color.g = 0.8
        cells_marker.color.b = 1.0
        cells_marker.color.a = 0.9
        cells_marker.points = [
            _cell_to_world(index, msg) for index in sorted(frontier_cells)
        ]

        centers_marker = Marker()
        centers_marker.header.stamp = msg.header.stamp
        centers_marker.header.frame_id = frame_id
        centers_marker.ns = 'frontier_cluster_centers'
        centers_marker.id = 1
        centers_marker.type = Marker.SPHERE_LIST
        centers_marker.action = Marker.ADD
        centers_marker.pose.orientation.w = 1.0
        center_scale = max(resolution * 3.0, 0.15)
        centers_marker.scale.x = center_scale
        centers_marker.scale.y = center_scale
        centers_marker.scale.z = center_scale
        centers_marker.color.r = 1.0
        centers_marker.color.g = 0.5
        centers_marker.color.b = 0.0
        centers_marker.color.a = 1.0

        for cluster in clusters:
            points = [_cell_to_world(index, msg) for index in cluster]
            center = Point()
            center.x = sum(point.x for point in points) / len(points)
            center.y = sum(point.y for point in points) / len(points)
            center.z = 0.10
            centers_marker.points.append(center)

        marker_array = MarkerArray()
        marker_array.markers = [cells_marker, centers_marker]
        self._marker_pub.publish(marker_array)


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
