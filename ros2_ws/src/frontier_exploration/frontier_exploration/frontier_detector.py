"""Read-only frontier detector and nearest-frontier selector for TurtleBot4."""

from collections import deque
from math import atan2, cos, hypot, sin
from typing import Iterable

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
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
        seed = min(remaining)
        remaining.remove(seed)
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


def split_frontier_cluster(
    cluster: list[int],
    width: int,
    height: int,
    segment_radius_cells: int,
    min_segment_size: int,
) -> list[list[int]]:
    """Split one long connected frontier into local connected segments."""
    remaining = set(cluster)
    segments: list[list[int]] = []
    radius_sq = max(1, segment_radius_cells) ** 2

    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        seed_x = seed % width
        seed_y = seed // width
        queue = deque([seed])
        segment = [seed]

        while queue:
            current = queue.popleft()
            for neighbor in _neighbors(current, width, height):
                if neighbor not in remaining:
                    continue

                neighbor_x = neighbor % width
                neighbor_y = neighbor // width
                distance_sq = (
                    (neighbor_x - seed_x) ** 2 + (neighbor_y - seed_y) ** 2
                )
                if distance_sq > radius_sq:
                    continue

                remaining.remove(neighbor)
                queue.append(neighbor)
                segment.append(neighbor)

        if len(segment) >= min_segment_size:
            segments.append(segment)

    return segments


def split_frontier_clusters(
    clusters: list[list[int]],
    width: int,
    height: int,
    resolution: float,
    segment_radius_m: float,
    min_segment_size: int,
) -> list[list[int]]:
    """Split every connected frontier cluster into local candidate segments."""
    safe_resolution = max(float(resolution), 1e-6)
    radius_cells = max(1, round(segment_radius_m / safe_resolution))
    segments: list[list[int]] = []

    for cluster in clusters:
        segments.extend(
            split_frontier_cluster(
                cluster,
                width,
                height,
                radius_cells,
                min_segment_size,
            )
        )

    return segments


def representative_cell(segment: list[int], width: int) -> int:
    """Return the frontier cell nearest the segment centroid."""
    if not segment:
        raise ValueError('segment must not be empty')

    xs = [index % width for index in segment]
    ys = [index // width for index in segment]
    centroid_x = sum(xs) / len(xs)
    centroid_y = sum(ys) / len(ys)

    return min(
        segment,
        key=lambda index: (
            (index % width - centroid_x) ** 2
            + (index // width - centroid_y) ** 2
        ),
    )


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


def nearest_representative(
    representatives: list[int],
    msg: OccupancyGrid,
    robot_x: float,
    robot_y: float,
) -> tuple[int, float] | None:
    """Return the representative with the smallest Euclidean robot distance."""
    if not representatives:
        return None

    best_index = representatives[0]
    best_point = _cell_to_world(best_index, msg)
    best_distance = hypot(best_point.x - robot_x, best_point.y - robot_y)

    for index in representatives[1:]:
        point = _cell_to_world(index, msg)
        distance = hypot(point.x - robot_x, point.y - robot_y)
        if distance < best_distance:
            best_index = index
            best_distance = distance

    return best_index, best_distance


class FrontierDetector(Node):
    """Detect frontier candidates and visualize nearest-frontier selection."""

    def __init__(self) -> None:
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/frontier_markers')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('segment_radius_m', 0.75)
        self.declare_parameter('min_segment_size', 5)

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

        self._last_summary: tuple[int, int, int, int, int] | None = None
        self._last_selected: tuple[int, int] | None = None
        self._tf_warning_shown = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._marker_pub = self.create_publisher(
            MarkerArray, marker_topic, marker_qos
        )
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)

        self.get_logger().info(f'Listening for occupancy grids on {map_topic}')
        self.get_logger().info(f'Publishing RViz markers on {marker_topic}')
        self.get_logger().info('Nearest-frontier selection is visualization only')

    def _robot_position(self, map_frame: str) -> tuple[float, float] | None:
        robot_frame = str(self.get_parameter('robot_frame').value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                robot_frame,
                Time(),
            )
        except TransformException as exc:
            if not self._tf_warning_shown:
                self.get_logger().warning(
                    f'Waiting for TF {map_frame} -> {robot_frame}: {exc}'
                )
                self._tf_warning_shown = True
            return None

        self._tf_warning_shown = False
        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0 or len(msg.data) != width * height:
            self.get_logger().warning('Ignoring invalid OccupancyGrid dimensions')
            return

        min_cluster_size = max(
            1, int(self.get_parameter('min_cluster_size').value)
        )
        segment_radius_m = max(
            0.05, float(self.get_parameter('segment_radius_m').value)
        )
        min_segment_size = max(
            1, int(self.get_parameter('min_segment_size').value)
        )

        frontier_cells = detect_frontier_cells(list(msg.data), width, height)
        clusters = cluster_frontiers(
            frontier_cells, width, height, min_cluster_size
        )
        segments = split_frontier_clusters(
            clusters,
            width,
            height,
            msg.info.resolution,
            segment_radius_m,
            min_segment_size,
        )
        representatives = [
            representative_cell(segment, width) for segment in segments
        ]

        map_frame = msg.header.frame_id or 'map'
        robot_position = self._robot_position(map_frame)
        selected: tuple[int, float] | None = None
        if robot_position is not None:
            selected = nearest_representative(
                representatives,
                msg,
                robot_position[0],
                robot_position[1],
            )

        selected_index = selected[0] if selected is not None else None
        self._publish_markers(
            msg,
            frontier_cells,
            segments,
            representatives,
            selected_index,
        )

        summary = (
            len(frontier_cells),
            len(clusters),
            len(segments),
            len(representatives),
            1 if selected is not None else 0,
        )
        if summary != self._last_summary:
            self.get_logger().info(
                'Frontier cells: '
                f'{summary[0]} | connected clusters: {summary[1]} '
                f'| segments: {summary[2]} | representatives: {summary[3]} '
                f'| selected: {summary[4]}'
            )
            self._last_summary = summary

        if selected is not None:
            selected_key = (selected[0], round(selected[1] * 100))
            if selected_key != self._last_selected:
                point = _cell_to_world(selected[0], msg)
                self.get_logger().info(
                    'Nearest frontier: '
                    f'x={point.x:.2f}, y={point.y:.2f}, '
                    f'distance={selected[1]:.2f} m'
                )
                self._last_selected = selected_key

    def _publish_markers(
        self,
        msg: OccupancyGrid,
        frontier_cells: set[int],
        segments: list[list[int]],
        representatives: list[int],
        selected_index: int | None,
    ) -> None:
        frame_id = msg.header.frame_id or 'map'
        resolution = max(float(msg.info.resolution), 0.01)

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL

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

        segment_centers_marker = Marker()
        segment_centers_marker.header.stamp = msg.header.stamp
        segment_centers_marker.header.frame_id = frame_id
        segment_centers_marker.ns = 'frontier_segment_centers'
        segment_centers_marker.id = 1
        segment_centers_marker.type = Marker.SPHERE_LIST
        segment_centers_marker.action = Marker.ADD
        segment_centers_marker.pose.orientation.w = 1.0
        center_scale = max(resolution * 2.2, 0.11)
        segment_centers_marker.scale.x = center_scale
        segment_centers_marker.scale.y = center_scale
        segment_centers_marker.scale.z = center_scale
        segment_centers_marker.color.r = 1.0
        segment_centers_marker.color.g = 0.5
        segment_centers_marker.color.b = 0.0
        segment_centers_marker.color.a = 0.8

        for segment in segments:
            points = [_cell_to_world(index, msg) for index in segment]
            center = Point()
            center.x = sum(point.x for point in points) / len(points)
            center.y = sum(point.y for point in points) / len(points)
            center.z = 0.08
            segment_centers_marker.points.append(center)

        representative_marker = Marker()
        representative_marker.header.stamp = msg.header.stamp
        representative_marker.header.frame_id = frame_id
        representative_marker.ns = 'frontier_representatives'
        representative_marker.id = 2
        representative_marker.type = Marker.SPHERE_LIST
        representative_marker.action = Marker.ADD
        representative_marker.pose.orientation.w = 1.0
        representative_scale = max(resolution * 3.0, 0.15)
        representative_marker.scale.x = representative_scale
        representative_marker.scale.y = representative_scale
        representative_marker.scale.z = representative_scale
        representative_marker.color.r = 0.2
        representative_marker.color.g = 1.0
        representative_marker.color.b = 0.2
        representative_marker.color.a = 1.0
        representative_marker.points = [
            _cell_to_world(index, msg) for index in representatives
        ]
        for point in representative_marker.points:
            point.z = 0.12

        selected_marker = Marker()
        selected_marker.header.stamp = msg.header.stamp
        selected_marker.header.frame_id = frame_id
        selected_marker.ns = 'nearest_frontier_selected'
        selected_marker.id = 3
        selected_marker.type = Marker.SPHERE
        selected_marker.action = Marker.ADD
        selected_marker.pose.orientation.w = 1.0
        selected_scale = max(resolution * 5.0, 0.25)
        selected_marker.scale.x = selected_scale
        selected_marker.scale.y = selected_scale
        selected_marker.scale.z = selected_scale
        selected_marker.color.r = 1.0
        selected_marker.color.g = 0.1
        selected_marker.color.b = 0.1
        selected_marker.color.a = 1.0
        if selected_index is not None:
            selected_point = _cell_to_world(selected_index, msg)
            selected_marker.pose.position.x = selected_point.x
            selected_marker.pose.position.y = selected_point.y
            selected_marker.pose.position.z = 0.18
        else:
            selected_marker.action = Marker.DELETE

        marker_array = MarkerArray()
        marker_array.markers = [
            delete_all,
            cells_marker,
            segment_centers_marker,
            representative_marker,
            selected_marker,
        ]
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
