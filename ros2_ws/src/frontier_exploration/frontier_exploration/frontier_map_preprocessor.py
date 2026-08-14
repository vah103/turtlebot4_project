"""Prepare a detector-only occupancy grid for robust frontier extraction.

The SLAM /map is never modified. This node publishes a filtered copy that:
1. closes narrow unknown gaps inside observed free space, and
2. keeps only the free-space component reachable from the robot.

The existing frontier detector can then operate on /frontier_map while Nav2
continues to use the original /map.
"""

from collections import deque
from math import atan2, cos, sin

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


FREE = 0
UNKNOWN = -1
BLOCKED_FOR_DETECTOR = 100


def _neighbors(index: int, width: int, height: int):
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


def _integral_image(mask: list[bool], width: int, height: int) -> tuple[list[int], int]:
    stride = width + 1
    integral = [0] * (stride * (height + 1))
    for y in range(height):
        row_sum = 0
        src = y * width
        dst = (y + 1) * stride
        prev = y * stride
        for x in range(width):
            row_sum += 1 if mask[src + x] else 0
            integral[dst + x + 1] = integral[prev + x + 1] + row_sum
    return integral, stride


def _box_sum(
    integral: list[int], stride: int, x0: int, y0: int, x1: int, y1: int
) -> int:
    return (
        integral[y1 * stride + x1]
        - integral[y0 * stride + x1]
        - integral[y1 * stride + x0]
        + integral[y0 * stride + x0]
    )


def _dilate(mask: list[bool], width: int, height: int, radius: int) -> list[bool]:
    if radius <= 0:
        return list(mask)
    integral, stride = _integral_image(mask, width, height)
    result = [False] * (width * height)
    for y in range(height):
        y0 = max(0, y - radius)
        y1 = min(height, y + radius + 1)
        for x in range(width):
            x0 = max(0, x - radius)
            x1 = min(width, x + radius + 1)
            result[y * width + x] = _box_sum(
                integral, stride, x0, y0, x1, y1
            ) > 0
    return result


def _erode(mask: list[bool], width: int, height: int, radius: int) -> list[bool]:
    if radius <= 0:
        return list(mask)
    integral, stride = _integral_image(mask, width, height)
    result = [False] * (width * height)
    full_span = 2 * radius + 1
    full_area = full_span * full_span
    for y in range(radius, height - radius):
        y0 = y - radius
        y1 = y + radius + 1
        for x in range(radius, width - radius):
            x0 = x - radius
            x1 = x + radius + 1
            result[y * width + x] = (
                _box_sum(integral, stride, x0, y0, x1, y1) == full_area
            )
    return result


def close_unknown_gaps(
    data: list[int], width: int, height: int, radius_cells: int
) -> tuple[list[int], int]:
    """Fill narrow unknown slivers that are enclosed by observed free space."""
    if radius_cells <= 0:
        return list(data), 0

    free_mask = [value == FREE for value in data]
    closed_free = _erode(
        _dilate(free_mask, width, height, radius_cells),
        width,
        height,
        radius_cells,
    )

    result = list(data)
    filled = 0
    for index, value in enumerate(data):
        if value == UNKNOWN and closed_free[index]:
            result[index] = FREE
            filled += 1
    return result, filled


def _origin_yaw(msg: OccupancyGrid) -> float:
    q = msg.info.origin.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return atan2(siny_cosp, cosy_cosp)


def world_to_cell(x: float, y: float, msg: OccupancyGrid) -> int | None:
    resolution = float(msg.info.resolution)
    if resolution <= 0.0:
        return None

    dx = x - msg.info.origin.position.x
    dy = y - msg.info.origin.position.y
    yaw = _origin_yaw(msg)
    local_x = cos(yaw) * dx + sin(yaw) * dy
    local_y = -sin(yaw) * dx + cos(yaw) * dy
    cell_x = int(local_x / resolution)
    cell_y = int(local_y / resolution)

    width = int(msg.info.width)
    height = int(msg.info.height)
    if not (0 <= cell_x < width and 0 <= cell_y < height):
        return None
    return cell_y * width + cell_x


def nearest_free_seed(
    data: list[int], width: int, height: int, start: int, radius_cells: int
) -> int | None:
    if 0 <= start < len(data) and data[start] == FREE:
        return start

    sx = start % width
    sy = start // width
    best: tuple[int, int] | None = None
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            x = sx + dx
            y = sy + dy
            if not (0 <= x < width and 0 <= y < height):
                continue
            index = y * width + x
            if data[index] != FREE:
                continue
            distance_sq = dx * dx + dy * dy
            if best is None or distance_sq < best[0]:
                best = (distance_sq, index)
    return None if best is None else best[1]


def reachable_free_cells(
    data: list[int], width: int, height: int, seed: int
) -> set[int]:
    """WFD outer search: return the free component reachable from the robot."""
    if not (0 <= seed < len(data)) or data[seed] != FREE:
        return set()

    visited = {seed}
    queue = deque([seed])
    while queue:
        current = queue.popleft()
        for neighbor in _neighbors(current, width, height):
            if neighbor in visited or data[neighbor] != FREE:
                continue
            visited.add(neighbor)
            queue.append(neighbor)
    return visited


def build_frontier_map(
    msg: OccupancyGrid,
    robot_x: float,
    robot_y: float,
    gap_radius_m: float,
    seed_search_radius_m: float,
) -> tuple[OccupancyGrid | None, int, int]:
    width = int(msg.info.width)
    height = int(msg.info.height)
    resolution = max(float(msg.info.resolution), 1e-6)
    data = list(msg.data)

    radius_cells = max(0, round(gap_radius_m / resolution))
    processed, filled_unknown = close_unknown_gaps(
        data, width, height, radius_cells
    )

    robot_index = world_to_cell(robot_x, robot_y, msg)
    if robot_index is None:
        return None, filled_unknown, 0

    seed_radius_cells = max(1, round(seed_search_radius_m / resolution))
    seed = nearest_free_seed(
        processed, width, height, robot_index, seed_radius_cells
    )
    if seed is None:
        return None, filled_unknown, 0

    reachable = reachable_free_cells(processed, width, height, seed)
    filtered = list(processed)
    for index, value in enumerate(filtered):
        if value == FREE and index not in reachable:
            filtered[index] = BLOCKED_FOR_DETECTOR

    output = OccupancyGrid()
    output.header = msg.header
    output.info = msg.info
    output.data = filtered
    return output, filled_unknown, len(reachable)


class FrontierMapPreprocessor(Node):
    """Publish a cleaned, WFD-reachable map for frontier detection only."""

    def __init__(self) -> None:
        super().__init__('frontier_map_preprocessor')
        self.declare_parameter('input_map_topic', '/map')
        self.declare_parameter('output_map_topic', '/frontier_map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('gap_fill_radius_m', 0.10)
        self.declare_parameter('seed_search_radius_m', 0.50)

        input_topic = str(self.get_parameter('input_map_topic').value)
        output_topic = str(self.get_parameter('output_map_topic').value)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            OccupancyGrid, output_topic, map_qos
        )
        self.create_subscription(
            OccupancyGrid, input_topic, self._on_map, map_qos
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_warning_shown = False
        self._last_summary: tuple[int, int] | None = None

        self.get_logger().info(
            f'Frontier map preprocessing: {input_topic} -> {output_topic}'
        )

    def _robot_position(self, map_frame: str) -> tuple[float, float] | None:
        robot_frame = str(self.get_parameter('robot_frame').value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame, robot_frame, Time()
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
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0 or len(msg.data) != width * height:
            self.get_logger().warning('Ignoring invalid OccupancyGrid dimensions')
            return

        map_frame = msg.header.frame_id or 'map'
        robot_position = self._robot_position(map_frame)
        if robot_position is None:
            return

        gap_radius_m = max(
            0.0, float(self.get_parameter('gap_fill_radius_m').value)
        )
        seed_search_radius_m = max(
            0.05, float(self.get_parameter('seed_search_radius_m').value)
        )
        output, filled_unknown, reachable_count = build_frontier_map(
            msg,
            robot_position[0],
            robot_position[1],
            gap_radius_m,
            seed_search_radius_m,
        )
        if output is None:
            self.get_logger().warning(
                'Could not seed WFD reachable-free search from the robot pose'
            )
            return

        self._publisher.publish(output)
        summary = (filled_unknown, reachable_count)
        if summary != self._last_summary:
            radius_cells = round(
                gap_radius_m / max(float(msg.info.resolution), 1e-6)
            )
            self.get_logger().info(
                'Frontier map ready: '
                f'filled_unknown={filled_unknown} | '
                f'reachable_free={reachable_count} | '
                f'gap_radius={radius_cells} cells'
            )
            self._last_summary = summary


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierMapPreprocessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
