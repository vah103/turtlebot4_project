"""Frontier detector with planning-only Nav2 reachability checks for TurtleBot4."""

from collections import deque
from math import atan2, cos, hypot, sin
from typing import Iterable

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
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


def ordered_representatives(
    representatives: list[int],
    msg: OccupancyGrid,
    robot_x: float,
    robot_y: float,
    min_distance_m: float,
) -> list[tuple[int, float]]:
    """Sort eligible representatives by Euclidean robot distance."""
    eligible: list[tuple[int, float]] = []
    for index in representatives:
        point = _cell_to_world(index, msg)
        distance = hypot(point.x - robot_x, point.y - robot_y)
        if distance >= min_distance_m:
            eligible.append((index, distance))
    return sorted(eligible, key=lambda candidate: candidate[1])


def path_length(path: Path) -> float:
    """Return geometric length of a nav_msgs/Path."""
    total = 0.0
    for first, second in zip(path.poses, path.poses[1:]):
        total += hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y,
        )
    return total


class FrontierDetector(Node):
    """Detect frontier candidates and select the nearest Nav2-reachable one."""

    def __init__(self) -> None:
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('marker_topic', '/frontier_markers')
        self.declare_parameter('path_topic', '/frontier_selected_path')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('planner_action', '/compute_path_to_pose')
        self.declare_parameter('planner_id', '')
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('segment_radius_m', 0.75)
        self.declare_parameter('min_segment_size', 5)
        self.declare_parameter('min_selection_distance_m', 0.60)

        map_topic = str(self.get_parameter('map_topic').value)
        marker_topic = str(self.get_parameter('marker_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        planner_action = str(self.get_parameter('planner_action').value)

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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._planner_client = ActionClient(
            self, ComputePathToPose, planner_action
        )

        self._marker_pub = self.create_publisher(
            MarkerArray, marker_topic, marker_qos
        )
        self._path_pub = self.create_publisher(Path, path_topic, 10)
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)

        self._tf_warning_shown = False
        self._planner_warning_shown = False
        self._last_summary: tuple[int, int, int, int, int] | None = None

        self._latest_map: OccupancyGrid | None = None
        self._latest_frontier_cells: set[int] = set()
        self._latest_segments: list[list[int]] = []
        self._latest_representatives: list[int] = []

        self._candidate_signature: tuple[int, ...] | None = None
        self._candidate_queue: list[tuple[int, float]] = []
        self._generation = 0
        self._planning_candidate: tuple[int, int, float] | None = None
        self._selected_index: int | None = None
        self._selected_distance: float | None = None
        self._selected_path: Path | None = None

        self.get_logger().info(f'Listening for occupancy grids on {map_topic}')
        self.get_logger().info(f'Publishing RViz markers on {marker_topic}')
        self.get_logger().info(f'Publishing selected path on {path_topic}')
        self.get_logger().info(
            f'Planning-only Nav2 reachability check on {planner_action}; '
            'no navigation goal is sent'
        )

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
        min_selection_distance_m = max(
            0.0, float(self.get_parameter('min_selection_distance_m').value)
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

        self._latest_map = msg
        self._latest_frontier_cells = frontier_cells
        self._latest_segments = segments
        self._latest_representatives = representatives

        map_frame = msg.header.frame_id or 'map'
        robot_position = self._robot_position(map_frame)
        candidates: list[tuple[int, float]] = []
        if robot_position is not None:
            candidates = ordered_representatives(
                representatives,
                msg,
                robot_position[0],
                robot_position[1],
                min_selection_distance_m,
            )

        signature = tuple(index for index, _ in candidates)
        if signature != self._candidate_signature:
            self._candidate_signature = signature
            self._candidate_queue = list(candidates)
            self._generation += 1
            self._selected_index = None
            self._selected_distance = None
            self._selected_path = None
            self._publish_empty_path(map_frame)

        self._publish_markers()

        summary = (
            len(frontier_cells),
            len(clusters),
            len(segments),
            len(representatives),
            len(candidates),
        )
        if summary != self._last_summary:
            self.get_logger().info(
                'Frontier cells: '
                f'{summary[0]} | connected clusters: {summary[1]} '
                f'| segments: {summary[2]} | representatives: {summary[3]} '
                f'| eligible: {summary[4]}'
            )
            self._last_summary = summary

        self._start_next_path_check()

    def _start_next_path_check(self) -> None:
        if self._selected_index is not None:
            return
        if self._planning_candidate is not None:
            return
        if self._latest_map is None:
            return
        if not self._candidate_queue:
            return

        if not self._planner_client.server_is_ready():
            if not self._planner_warning_shown:
                planner_action = str(self.get_parameter('planner_action').value)
                self.get_logger().warning(
                    f'Waiting for Nav2 planner action {planner_action}'
                )
                self._planner_warning_shown = True
            return

        self._planner_warning_shown = False
        index, distance = self._candidate_queue.pop(0)
        candidate = (self._generation, index, distance)
        self._planning_candidate = candidate

        point = _cell_to_world(index, self._latest_map)
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = self._latest_map.header.frame_id or 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = point.x
        goal_msg.goal.pose.position.y = point.y
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.planner_id = str(self.get_parameter('planner_id').value)
        goal_msg.use_start = False

        self.get_logger().info(
            'Checking Nav2 path to candidate: '
            f'x={point.x:.2f}, y={point.y:.2f}, euclidean={distance:.2f} m'
        )
        self._publish_markers()

        future = self._planner_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda response_future, expected=candidate:
            self._on_plan_goal_response(response_future, expected)
        )

    def _on_plan_goal_response(self, future, candidate: tuple[int, int, float]) -> None:
        if self._planning_candidate != candidate:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Planner goal request failed: {exc}')
            self._finish_path_failure(candidate)
            return

        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 planner rejected candidate')
            self._finish_path_failure(candidate)
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda finished_future, expected=candidate:
            self._on_plan_result(finished_future, expected)
        )

    def _on_plan_result(self, future, candidate: tuple[int, int, float]) -> None:
        if self._planning_candidate != candidate:
            return

        generation, index, distance = candidate
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Planner result failed: {exc}')
            self._finish_path_failure(candidate)
            return

        result = wrapped_result.result
        path = result.path if result is not None else Path()
        succeeded = (
            wrapped_result.status == GoalStatus.STATUS_SUCCEEDED
            and len(path.poses) > 0
        )

        self._planning_candidate = None

        if generation != self._generation:
            self._start_next_path_check()
            return

        if not succeeded:
            error_code = getattr(result, 'error_code', -1)
            error_msg = getattr(result, 'error_msg', '')
            suffix = f' code={error_code}'
            if error_msg:
                suffix += f' ({error_msg})'
            self.get_logger().info(
                f'Candidate unreachable by Nav2 planner:{suffix}'
            )
            self._publish_markers()
            self._start_next_path_check()
            return

        self._selected_index = index
        self._selected_distance = distance
        self._selected_path = path
        planned_length = path_length(path)
        point = _cell_to_world(index, self._latest_map)

        self.get_logger().info(
            'Reachable frontier selected: '
            f'x={point.x:.2f}, y={point.y:.2f}, '
            f'euclidean={distance:.2f} m, path={planned_length:.2f} m'
        )
        self._path_pub.publish(path)
        self._publish_markers()

    def _finish_path_failure(self, candidate: tuple[int, int, float]) -> None:
        if self._planning_candidate == candidate:
            self._planning_candidate = None
        if candidate[0] == self._generation:
            self._publish_markers()
            self._start_next_path_check()

    def _publish_empty_path(self, frame_id: str) -> None:
        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_pub.publish(path)

    def _publish_markers(self) -> None:
        msg = self._latest_map
        if msg is None:
            return

        frontier_cells = self._latest_frontier_cells
        segments = self._latest_segments
        representatives = self._latest_representatives
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
        selected_marker.ns = 'reachable_frontier_selected'
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
        if self._selected_index is not None:
            selected_point = _cell_to_world(self._selected_index, msg)
            selected_marker.pose.position.x = selected_point.x
            selected_marker.pose.position.y = selected_point.y
            selected_marker.pose.position.z = 0.18
        else:
            selected_marker.action = Marker.DELETE

        checking_marker = Marker()
        checking_marker.header.stamp = msg.header.stamp
        checking_marker.header.frame_id = frame_id
        checking_marker.ns = 'planner_candidate_checking'
        checking_marker.id = 4
        checking_marker.type = Marker.SPHERE
        checking_marker.action = Marker.ADD
        checking_marker.pose.orientation.w = 1.0
        checking_scale = max(resolution * 4.0, 0.20)
        checking_marker.scale.x = checking_scale
        checking_marker.scale.y = checking_scale
        checking_marker.scale.z = checking_scale
        checking_marker.color.r = 1.0
        checking_marker.color.g = 1.0
        checking_marker.color.b = 0.1
        checking_marker.color.a = 1.0
        if (
            self._planning_candidate is not None
            and self._planning_candidate[0] == self._generation
        ):
            checking_point = _cell_to_world(
                self._planning_candidate[1], msg
            )
            checking_marker.pose.position.x = checking_point.x
            checking_marker.pose.position.y = checking_point.y
            checking_marker.pose.position.z = 0.16
        else:
            checking_marker.action = Marker.DELETE

        marker_array = MarkerArray()
        marker_array.markers = [
            delete_all,
            cells_marker,
            segment_centers_marker,
            representative_marker,
            selected_marker,
            checking_marker,
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
