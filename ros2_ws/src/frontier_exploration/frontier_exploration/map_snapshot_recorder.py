"""Automatically record raw SLAM map snapshots for LaMa evaluation."""

import json
import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_exploration.map_snapshot_core import (
    SnapshotPolicy,
    occupancy_to_pgm,
    occupancy_to_signed_bytes,
)


class MapSnapshotRecorder(Node):
    """Save one exact map snapshot whenever the robot moves or time elapses."""

    def __init__(self) -> None:
        super().__init__('map_snapshot_recorder')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('completion_topic', '/exploration_complete')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter(
            'output_dir', '~/turtlebot4_lama_snapshots'
        )
        self.declare_parameter('run_name', '')
        self.declare_parameter('distance_interval_m', 0.5)
        self.declare_parameter('time_interval_sec', 5.0)
        self.declare_parameter('min_interval_sec', 1.0)
        self.declare_parameter('check_period_sec', 0.5)
        self.declare_parameter('shutdown_on_completion', True)

        output_dir = Path(
            os.path.expanduser(str(self.get_parameter('output_dir').value))
        )
        run_name = str(self.get_parameter('run_name').value).strip()
        if not run_name:
            run_name = datetime.now(timezone.utc).strftime(
                'run_%Y%m%d_%H%M%S_utc'
            )
        self._run_dir = output_dir / run_name
        self._run_dir.mkdir(parents=True, exist_ok=False)
        self._manifest_path = self._run_dir / 'manifest.jsonl'

        self._policy = SnapshotPolicy(
            distance_interval_m=float(
                self.get_parameter('distance_interval_m').value
            ),
            time_interval_sec=float(
                self.get_parameter('time_interval_sec').value
            ),
            min_interval_sec=float(
                self.get_parameter('min_interval_sec').value
            ),
        )
        self._latest_map: OccupancyGrid | None = None
        self._latest_odom: Odometry | None = None
        self._sequence = 0
        self._recording_stopped = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        completion_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('completion_topic').value),
            self._on_completion,
            completion_qos,
        )
        check_period = max(
            0.1, float(self.get_parameter('check_period_sec').value)
        )
        self._timer = self.create_timer(check_period, self._check_snapshot)

        self._write_run_metadata()
        self.get_logger().info(
            f'Recording LaMa map snapshots in {self._run_dir}'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_odom = msg

    def _write_json(self, path: Path, payload: dict) -> None:
        temporary = path.with_suffix(path.suffix + '.tmp')
        with temporary.open('w', encoding='utf-8') as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
        os.replace(temporary, path)

    def _write_bytes(self, path: Path, payload: bytes) -> None:
        temporary = path.with_suffix(path.suffix + '.tmp')
        with temporary.open('wb') as stream:
            stream.write(payload)
        os.replace(temporary, path)

    def _write_run_metadata(self) -> None:
        payload = {
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'map_topic': str(self.get_parameter('map_topic').value),
            'odom_topic': str(self.get_parameter('odom_topic').value),
            'completion_topic': str(
                self.get_parameter('completion_topic').value
            ),
            'distance_interval_m': self._policy.distance_interval_m,
            'time_interval_sec': self._policy.time_interval_sec,
            'min_interval_sec': self._policy.min_interval_sec,
            'pgm_unknown_value': 127,
            'raw_encoding': 'signed int8, ROS row-major order',
        }
        self._write_json(self._run_dir / 'run.json', payload)

    def _map_robot_pose(self, map_frame: str) -> dict | None:
        robot_frame = str(self.get_parameter('robot_frame').value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                robot_frame,
                Time(),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'frame_id': map_frame,
            'child_frame_id': robot_frame,
            'position': {
                'x': translation.x,
                'y': translation.y,
                'z': translation.z,
            },
            'orientation': {
                'x': rotation.x,
                'y': rotation.y,
                'z': rotation.z,
                'w': rotation.w,
            },
        }

    def _snapshot_metadata(
        self,
        map_msg: OccupancyGrid,
        odom_msg: Odometry,
        trigger: str,
    ) -> dict:
        origin = map_msg.info.origin
        odom_pose = odom_msg.pose.pose
        map_frame = map_msg.header.frame_id or 'map'
        return {
            'sequence': self._sequence,
            'trigger': trigger,
            'saved_utc': datetime.now(timezone.utc).isoformat(),
            'node_time_sec': self._now_sec(),
            'map_stamp': {
                'sec': map_msg.header.stamp.sec,
                'nanosec': map_msg.header.stamp.nanosec,
            },
            'odom_stamp': {
                'sec': odom_msg.header.stamp.sec,
                'nanosec': odom_msg.header.stamp.nanosec,
            },
            'map': {
                'frame_id': map_frame,
                'width': map_msg.info.width,
                'height': map_msg.info.height,
                'resolution': map_msg.info.resolution,
                'origin': {
                    'position': {
                        'x': origin.position.x,
                        'y': origin.position.y,
                        'z': origin.position.z,
                    },
                    'orientation': {
                        'x': origin.orientation.x,
                        'y': origin.orientation.y,
                        'z': origin.orientation.z,
                        'w': origin.orientation.w,
                    },
                },
                'pgm_y_axis': 'top-down; ROS grid rows are vertically flipped',
            },
            'odom_pose': {
                'frame_id': odom_msg.header.frame_id or 'odom',
                'child_frame_id': odom_msg.child_frame_id,
                'position': {
                    'x': odom_pose.position.x,
                    'y': odom_pose.position.y,
                    'z': odom_pose.position.z,
                },
                'orientation': {
                    'x': odom_pose.orientation.x,
                    'y': odom_pose.orientation.y,
                    'z': odom_pose.orientation.z,
                    'w': odom_pose.orientation.w,
                },
            },
            'map_robot_pose': self._map_robot_pose(map_frame),
            'files': {
                'pgm': f'{self._sequence:06d}_map.pgm',
                'raw': f'{self._sequence:06d}_occupancy.bin',
                'metadata': f'{self._sequence:06d}_metadata.json',
            },
        }

    def _capture(self, trigger: str, *, force: bool = False) -> bool:
        if self._latest_map is None or self._latest_odom is None:
            return False

        map_msg = self._latest_map
        odom_msg = self._latest_odom
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        data = list(map_msg.data)
        if width <= 0 or height <= 0 or len(data) != width * height:
            self.get_logger().warning('Ignoring invalid OccupancyGrid snapshot')
            return False

        now = self._now_sec()
        x = float(odom_msg.pose.pose.position.x)
        y = float(odom_msg.pose.pose.position.y)
        if not force and not self._policy.should_capture(now, x, y):
            return False

        prefix = f'{self._sequence:06d}'
        metadata = self._snapshot_metadata(map_msg, odom_msg, trigger)
        try:
            self._write_bytes(
                self._run_dir / f'{prefix}_map.pgm',
                occupancy_to_pgm(data, width, height),
            )
            self._write_bytes(
                self._run_dir / f'{prefix}_occupancy.bin',
                occupancy_to_signed_bytes(data),
            )
            self._write_json(
                self._run_dir / f'{prefix}_metadata.json',
                metadata,
            )
            with self._manifest_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(metadata, ensure_ascii=False) + '\n')
        except OSError as exc:
            self.get_logger().error(f'Failed to save map snapshot: {exc}')
            return False

        self._policy.record_capture(now, x, y)
        self._sequence += 1
        self.get_logger().info(
            f'Saved snapshot {prefix}: trigger={trigger}, '
            f'map={width}x{height}'
        )
        return True

    def _check_snapshot(self) -> None:
        if self._recording_stopped:
            return
        self._capture('distance_or_time')

    def _on_completion(self, msg: Bool) -> None:
        if not msg.data or self._recording_stopped:
            return
        saved = self._capture('exploration_complete', force=True)
        self._recording_stopped = True
        self._timer.cancel()
        if saved:
            self.get_logger().info('Final snapshot saved; recording complete')
        else:
            self.get_logger().warning(
                'Exploration completed before a final map/odom pair was ready'
            )
        if bool(self.get_parameter('shutdown_on_completion').value):
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapSnapshotRecorder()
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
