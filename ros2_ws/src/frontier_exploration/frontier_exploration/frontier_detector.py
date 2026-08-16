"""WFD-style frontier detector with costmap-aware Nav2 goal materialization."""

from math import atan2, cos, floor, hypot, sin

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

from frontier_exploration.frontier_core import (
    FREE,
    UNKNOWN,
    candidate_goal_cells,
    cluster_frontiers,
    detect_frontier_cells,
    reachable_free_cells,
    representative_cell,
    segment_centroid_xy,
    split_frontier_clusters,
)

# Candidate tuple:
# representative_index, navigation_goal_index, frontier_distance_m, goal_distance_m
Candidate = tuple[int, int, float, float]
# Planner candidate adds a generation number in front of Candidate.
PlanningCandidate = tuple[int, int, int, float, float]


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


def _world_to_cell(msg: OccupancyGrid, x: float, y: float) -> int | None:
    resolution = max(float(msg.info.resolution), 1e-6)
    dx = x - msg.info.origin.position.x
    dy = y - msg.info.origin.position.y
    yaw = _origin_yaw(msg)
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    cell_x = int(floor(local_x / resolution))
    cell_y = int(floor(local_y / resolution))
    if not (0 <= cell_x < msg.info.width and 0 <= cell_y < msg.info.height):
        return None
    return cell_y * msg.info.width + cell_x


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
    """Detect reachable frontiers and select the nearest safe reachable one."""

    def __init__(self) -> None:
        super().__init__('frontier_detector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('marker_topic', '/frontier_markers')
        self.declare_parameter('path_topic', '/frontier_selected_path')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('planner_action', '/compute_path_to_pose')
        self.declare_parameter('planner_id', '')
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('segment_radius_m', 0.75)
        self.declare_parameter('min_segment_size', 5)
        self.declare_parameter('min_selection_distance_m', 0.60)
        self.declare_parameter('costmap_occ_threshold', 65)
        self.declare_parameter('costmap_unknown_is_blocked', True)
        self.declare_parameter('require_global_costmap', True)

        map_topic = str(self.get_parameter('map_topic').value)
        global_costmap_topic = str(
            self.get_parameter('global_costmap_topic').value
        )
        local_costmap_topic = str(
            self.get_parameter('local_costmap_topic').value
        )
        marker_topic = str(self.get_parameter('marker_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        planner_action = str(self.get_parameter('planner_action').value)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        costmap_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
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
        self.create_subscription(
            OccupancyGrid,
            global_costmap_topic,
            self._on_global_costmap,
            costmap_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            local_costmap_topic,
            self._on_local_costmap,
            costmap_qos,
        )

        self._tf_warning_shown = False
        self._costmap_warning_shown = False
        self._planner_warning_shown = False
        self._last_summary: tuple[int, int, int, int, int] | None = None

        self._latest_map: OccupancyGrid | None = None
        self._global_costmap: OccupancyGrid | None = None
        self._local_costmap: OccupancyGrid | None = None
        self._latest_reachable_free: set[int] = set()
        self._latest_frontier_cells: set[int] = set()
        self._latest_segments: list[list[int]] = []
        self._latest_representatives: list[int] = []

        self._candidate_signature: tuple[tuple[int, int], ...] | None = None
        self._candidate_queue: list[Candidate] = []
        self._generation = 0
        self._planning_candidate: PlanningCandidate | None = None
        self._selected_index: int | None = None
        self._selected_goal_index: int | None = None
        self._selected_distance: float | None = None
        self._selected_path: Path | None = None

        self.get_logger().info(f'Listening for occupancy grids on {map_topic}')
        self.get_logger().info(
            f'Global costmap safety filter: {global_costmap_topic}'
        )
        self.get_logger().info(
            f'Local costmap safety filter: {local_costmap_topic}'
        )
        self.get_logger().info(f'Publishing RViz markers on {marker_topic}')
        self.get_logger().info(f'Publishing selected path on {path_topic}')
        self.get_logger().info(
            'Frontier extraction: WFD-style reachable-space BFS + '
            '8-connected frontier clustering'
        )
        self.get_logger().info(
            f'Planning-only Nav2 reachability check on {planner_action}; '
            'no navigation goal is sent by the detector'
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

    def _transform_xy(
        self,
        x: float,
        y: float,
        source_frame: str,
        target_frame: str,
    ) -> tuple[float, float] | None:
        if not target_frame or target_frame == source_frame:
            return x, y
        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        except TransformException:
            return None

        q = transform.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        return (
            tx + cos(yaw) * x - sin(yaw) * y,
            ty + sin(yaw) * x + cos(yaw) * y,
        )

    def _point_cost(
        self,
        costmap: OccupancyGrid,
        x: float,
        y: float,
        source_frame: str,
    ) -> int | None:
        target_frame = costmap.header.frame_id or source_frame
        transformed = self._transform_xy(x, y, source_frame, target_frame)
        if transformed is None:
            return None
        index = _world_to_cell(costmap, transformed[0], transformed[1])
        if index is None or index >= len(costmap.data):
            return None
        return int(costmap.data[index])

    def _point_blocked(
        self,
        costmap: OccupancyGrid,
        x: float,
        y: float,
        source_frame: str,
        *,
        outside_is_blocked: bool,
    ) -> bool:
        cost = self._point_cost(costmap, x, y, source_frame)
        if cost is None:
            return outside_is_blocked
        if cost < 0:
            return bool(self.get_parameter('costmap_unknown_is_blocked').value)
        threshold = int(self.get_parameter('costmap_occ_threshold').value)
        return cost >= threshold

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self._global_costmap = msg
        self._costmap_warning_shown = False
        if (
            self._latest_map is not None
            and self._selected_index is None
            and self._planning_candidate is None
        ):
            self._candidate_signature = None
            self._on_map(self._latest_map)

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self._local_costmap = msg
        if (
            self._latest_map is not None
            and self._selected_index is None
            and self._planning_candidate is None
        ):
            self._candidate_signature = None
            self._on_map(self._latest_map)

    def _segment_centroid_world(
        self,
        segment: list[int],
        msg: OccupancyGrid,
    ) -> tuple[float, float]:
        cell_x, cell_y = segment_centroid_xy(segment, msg.info.width)
        resolution = msg.info.resolution
        local_x = (cell_x + 0.5) * resolution
        local_y = (cell_y + 0.5) * resolution
        yaw = _origin_yaw(msg)
        return (
            msg.info.origin.position.x
            + cos(yaw) * local_x
            - sin(yaw) * local_y,
            msg.info.origin.position.y
            + sin(yaw) * local_x
            + cos(yaw) * local_y,
        )

    def _goal_index_for_segment(
        self,
        segment: list[int],
        msg: OccupancyGrid,
        robot_x: float,
        robot_y: float,
        min_distance_m: float,
    ) -> tuple[int, float] | None:
        source_frame = msg.header.frame_id or 'map'
        centroid_x, centroid_y = self._segment_centroid_world(segment, msg)
        best: tuple[float, int, float] | None = None

        for index in candidate_goal_cells(
            segment,
            self._latest_frontier_cells,
            list(msg.data),
            msg.info.width,
            msg.info.height,
        ):
            point = _cell_to_world(index, msg)
            goal_distance = hypot(point.x - robot_x, point.y - robot_y)
            if goal_distance < min_distance_m:
                continue

            if self._global_costmap is not None and self._point_blocked(
                self._global_costmap,
                point.x,
                point.y,
                source_frame,
                outside_is_blocked=True,
            ):
                continue

            if self._local_costmap is not None and self._point_blocked(
                self._local_costmap,
                point.x,
                point.y,
                source_frame,
                outside_is_blocked=False,
            ):
                continue

            centroid_distance_sq = (
                (point.x - centroid_x) ** 2 + (point.y - centroid_y) ** 2
            )
            score = (centroid_distance_sq, index, goal_distance)
            if best is None or score < best:
                best = score

        if best is None:
            return None
        return best[1], best[2]

    def _build_candidates(
        self,
        msg: OccupancyGrid,
        robot_x: float,
        robot_y: float,
        min_distance_m: float,
    ) -> list[Candidate]:
        candidates: list[Candidate] = []
        for segment in self._latest_segments:
            representative = representative_cell(segment, msg.info.width)
            representative_point = _cell_to_world(representative, msg)
            frontier_distance = hypot(
                representative_point.x - robot_x,
                representative_point.y - robot_y,
            )
            if frontier_distance < min_distance_m:
                continue

            goal = self._goal_index_for_segment(
                segment,
                msg,
                robot_x,
                robot_y,
                min_distance_m,
            )
            if goal is None:
                continue
            goal_index, goal_distance = goal
            candidates.append(
                (representative, goal_index, frontier_distance, goal_distance)
            )

        return sorted(candidates, key=lambda candidate: candidate[2])

    def _on_map(self, msg: OccupancyGrid) -> None:
        width = msg.info.width
        height = msg.info.height
        if width == 0 or height == 0 or len(msg.data) != width * height:
            self.get_logger().warning('Ignoring invalid OccupancyGrid dimensions')
            return

        map_frame = msg.header.frame_id or 'map'
        robot_position = self._robot_position(map_frame)
        if robot_position is None:
            return
        robot_index = _world_to_cell(msg, robot_position[0], robot_position[1])
        if robot_index is None:
            self.get_logger().warning('Robot pose is outside the frontier map bounds')
            return

        data = list(msg.data)
        reachable = reachable_free_cells(data, width, height, robot_index)
        frontier_cells = detect_frontier_cells(
            data,
            width,
            height,
            reachable,
        )
        clusters = cluster_frontiers(
            frontier_cells,
            width,
            height,
            max(1, int(self.get_parameter('min_cluster_size').value)),
        )
        segments = split_frontier_clusters(
            clusters,
            width,
            height,
            msg.info.resolution,
            max(0.05, float(self.get_parameter('segment_radius_m').value)),
            max(1, int(self.get_parameter('min_segment_size').value)),
        )
        representatives = [
            representative_cell(segment, width) for segment in segments
        ]

        self._latest_map = msg
        self._latest_reachable_free = reachable
        self._latest_frontier_cells = frontier_cells
        self._latest_segments = segments
        self._latest_representatives = representatives

        require_costmap = bool(
            self.get_parameter('require_global_costmap').value
        )
        candidates: list[Candidate] = []
        if require_costmap and self._global_costmap is None:
            if not self._costmap_warning_shown:
                self.get_logger().warning(
                    'Waiting for global Nav2 costmap before materializing '
                    'frontier navigation goals'
                )
                self._costmap_warning_shown = True
        else:
            candidates = self._build_candidates(
                msg,
                robot_position[0],
                robot_position[1],
                max(
                    0.0,
                    float(
                        self.get_parameter('min_selection_distance_m').value
                    ),
                ),
            )

        signature = tuple((item[0], item[1]) for item in candidates)
        if signature != self._candidate_signature:
            self._candidate_signature = signature
            self._candidate_queue = list(candidates)
            self._generation += 1
            self._planning_candidate = None
            self._selected_index = None
            self._selected_goal_index = None
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
                f'| safe eligible: {summary[4]}'
            )
            self._last_summary = summary

        self._start_next_path_check()

    def _start_next_path_check(self) -> None:
        if self._selected_index is not None:
            return
        if self._planning_candidate is not None:
            return
        if self._latest_map is None or not self._candidate_queue:
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
        representative, goal_index, frontier_distance, goal_distance = (
            self._candidate_queue.pop(0)
        )
        candidate: PlanningCandidate = (
            self._generation,
            representative,
            goal_index,
            frontier_distance,
            goal_distance,
        )
        self._planning_candidate = candidate

        frontier_point = _cell_to_world(representative, self._latest_map)
        goal_point = _cell_to_world(goal_index, self._latest_map)
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = self._latest_map.header.frame_id or 'map'
        goal_msg.goal.header.stamp = self.get_clock().now().to_msg()
        goal_msg.goal.pose.position.x = goal_point.x
        goal_msg.goal.pose.position.y = goal_point.y
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.planner_id = str(self.get_parameter('planner_id').value)
        goal_msg.use_start = False

        self.get_logger().info(
            'Checking Nav2 path to frontier-safe free goal: '
            f'frontier=({frontier_point.x:.2f}, {frontier_point.y:.2f}), '
            f'goal=({goal_point.x:.2f}, {goal_point.y:.2f}), '
            f'frontier_distance={frontier_distance:.2f} m'
        )
        self._publish_markers()

        future = self._planner_client.send_goal_async(goal_msg)
        future.add_done_callback(
            lambda response_future, expected=candidate:
            self._on_plan_goal_response(response_future, expected)
        )

    def _on_plan_goal_response(
        self,
        future,
        candidate: PlanningCandidate,
    ) -> None:
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

    def _on_plan_result(self, future, candidate: PlanningCandidate) -> None:
        if self._planning_candidate != candidate:
            return

        generation, representative, goal_index, frontier_distance, _ = candidate
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

        self._selected_index = representative
        self._selected_goal_index = goal_index
        self._selected_distance = frontier_distance
        self._selected_path = path
        planned_length = path_length(path)
        frontier_point = _cell_to_world(representative, self._latest_map)
        goal_point = _cell_to_world(goal_index, self._latest_map)

        self.get_logger().info(
            'Reachable frontier selected: '
            f'frontier=({frontier_point.x:.2f}, {frontier_point.y:.2f}), '
            f'goal=({goal_point.x:.2f}, {goal_point.y:.2f}), '
            f'euclidean={frontier_distance:.2f} m, path={planned_length:.2f} m'
        )
        self._path_pub.publish(path)
        self._publish_markers()

    def _finish_path_failure(self, candidate: PlanningCandidate) -> None:
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
            _cell_to_world(index, msg)
            for index in sorted(self._latest_frontier_cells)
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
        for segment in self._latest_segments:
            cx, cy = self._segment_centroid_world(segment, msg)
            center = Point()
            center.x = cx
            center.y = cy
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
            _cell_to_world(index, msg)
            for index in self._latest_representatives
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
            point = _cell_to_world(self._selected_index, msg)
            selected_marker.pose.position.x = point.x
            selected_marker.pose.position.y = point.y
            selected_marker.pose.position.z = 0.18
        else:
            selected_marker.action = Marker.DELETE

        navigation_goal_marker = Marker()
        navigation_goal_marker.header.stamp = msg.header.stamp
        navigation_goal_marker.header.frame_id = frame_id
        navigation_goal_marker.ns = 'frontier_navigation_goal'
        navigation_goal_marker.id = 4
        navigation_goal_marker.type = Marker.SPHERE
        navigation_goal_marker.action = Marker.ADD
        navigation_goal_marker.pose.orientation.w = 1.0
        goal_scale = max(resolution * 4.0, 0.20)
        navigation_goal_marker.scale.x = goal_scale
        navigation_goal_marker.scale.y = goal_scale
        navigation_goal_marker.scale.z = goal_scale
        navigation_goal_marker.color.r = 0.8
        navigation_goal_marker.color.g = 0.2
        navigation_goal_marker.color.b = 1.0
        navigation_goal_marker.color.a = 1.0
        if self._selected_goal_index is not None:
            point = _cell_to_world(self._selected_goal_index, msg)
            navigation_goal_marker.pose.position.x = point.x
            navigation_goal_marker.pose.position.y = point.y
            navigation_goal_marker.pose.position.z = 0.16
        else:
            navigation_goal_marker.action = Marker.DELETE

        checking_marker = Marker()
        checking_marker.header.stamp = msg.header.stamp
        checking_marker.header.frame_id = frame_id
        checking_marker.ns = 'planner_candidate_checking'
        checking_marker.id = 5
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
            point = _cell_to_world(self._planning_candidate[2], msg)
            checking_marker.pose.position.x = point.x
            checking_marker.pose.position.y = point.y
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
            navigation_goal_marker,
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
