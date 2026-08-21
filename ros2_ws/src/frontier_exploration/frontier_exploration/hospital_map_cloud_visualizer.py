"""Publish the SLAM occupancy grid as PointCloud2 for shader-safe RViz display."""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class HospitalMapCloudVisualizer(Node):
    """Convert known OccupancyGrid cells to one RGB PointCloud2 message."""

    def __init__(self) -> None:
        super().__init__('hospital_map_cloud_visualizer')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('cloud_topic', '/map_cloud')
        self.declare_parameter('z_offset', 0.02)

        map_topic = str(self.get_parameter('map_topic').value)
        cloud_topic = str(self.get_parameter('cloud_topic').value)
        self._z_offset = float(self.get_parameter('z_offset').value)

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._publisher = self.create_publisher(
            PointCloud2,
            cloud_topic,
            latched_qos,
        )
        self._subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._on_map,
            latched_qos,
        )
        self._reported_first_cloud = False

        self.get_logger().info(
            f'OccupancyGrid RViz fallback active: {map_topic} -> {cloud_topic}'
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        width = int(msg.info.width)
        height = int(msg.info.height)
        expected = width * height
        if width <= 0 or height <= 0 or len(msg.data) != expected:
            self.get_logger().warning(
                'Ignoring malformed OccupancyGrid: '
                f'{width}x{height}, cells={len(msg.data)}, expected={expected}'
            )
            return

        occupancy = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        rows, cols = np.nonzero(occupancy >= 0)
        values = occupancy[rows, cols].astype(np.float32, copy=False)

        resolution = float(msg.info.resolution)
        local_x = (cols.astype(np.float32) + 0.5) * resolution
        local_y = (rows.astype(np.float32) + 0.5) * resolution

        origin = msg.info.origin
        q = origin.orientation
        yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)

        world_x = float(origin.position.x) + cosine * local_x - sine * local_y
        world_y = float(origin.position.y) + sine * local_x + cosine * local_y

        # Match RViz's familiar occupancy-map appearance without MapDisplay:
        # free cells are light, occupied cells dark, and unknown cells are omitted.
        shade = np.clip(240.0 - 2.2 * values, 20.0, 240.0).astype(np.uint32)
        packed_rgb = (shade << 16) | (shade << 8) | shade

        points = np.empty((rows.size, 4), dtype=np.float32)
        points[:, 0] = world_x
        points[:, 1] = world_y
        points[:, 2] = self._z_offset
        points[:, 3].view(np.uint32)[:] = packed_rgb

        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height = 1
        cloud.width = int(rows.size)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        cloud.data = points.tobytes()
        self._publisher.publish(cloud)

        if not self._reported_first_cloud:
            self.get_logger().info(
                f'Published /map cloud with {cloud.width} known cells '
                f'at {resolution:.3f} m/cell.'
            )
            self._reported_first_cloud = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HospitalMapCloudVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
