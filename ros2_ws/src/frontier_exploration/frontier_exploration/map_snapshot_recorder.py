"""Automatically record raw SLAM map snapshots for LaMa evaluation."""

import os
from collections import deque
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
    create_unique_run_dir,
    nearest_timestamp_index,
    occupancy_to_pgm,
    occupancy_to_signed_bytes,
    write_json_atomic,
    write_snapshot_files,
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
        self.declare_parameter('sync_tolerance_sec', 0.25)
        self.declare_parameter('odom_buffer_size', 400)
        self.declare_parameter('max_odom_step_m', 1.0)
        self.declare_parameter('shutdown_on_completion', True)

        output_dir = Path(
            os.path.expanduser(str(self.get_parameter('output_dir').value))
        )
        run_name = str(self.get_parameter('run_name').value).strip()
        if not run_name:
            run_name = datetime.now(timezone.utc).strftime(
                'run_%Y%m%d_%H%M%S_utc'
            )
        self._run_dir = create_unique_run_dir(output_dir, run_name)
        if self._run_dir.name != run_name:
            self.get_logger().warning(
                f'Run {run_name} already exists; using {self._run_dir.name}'
            )

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
            max_odom_step_m=float(
                self.get_parameter('max_odom_step_m').value
            ),
        )
        buffer_size = max(
            10, int(self.get_parameter('odom_buffer_size').value)
        )
        self._odom_buffer: deque[Odometry] = deque(maxlen=buffer_size)
        self._pending_map: OccupancyGrid | None = None
        self._latest_pair: tuple[
            OccupancyGrid,
            Odometry,
            float | None,
        ] | None = None
        self._sequence = 0
        self._recording_stopped = False
        self._completion_pending = False

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
        self._write_run_metadata()
        self.get_logger().info(
            f'Recording LaMa map snapshots in {self._run_dir}'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_map(self, msg: OccupancyGrid) -> None:
        if self._recording_stopped:
            return
        self._pending_map = msg
        self._try_sync_pending_map()

    def _on_odom(self, msg: Odometry) -> None:
        if self._recording_stopped:
            return
        self._odom_buffer.append(msg)
        position = msg.pose.pose.position
        self._policy.observe_motion(float(position.x), float(position.y))
        self._try_sync_pending_map()

    @staticmethod
    def _stamp_sec(msg) -> float:
        return float(msg.header.stamp.sec) + (
            float(msg.header.stamp.nanosec) / 1e9
        )

    def _try_sync_pending_map(self) -> None:
        if self._pending_map is None or not self._odom_buffer:
            return

        map_msg = self._pending_map
        map_stamp = self._stamp_sec(map_msg)
        odom_messages = list(self._odom_buffer)
        odom_stamps = [self._stamp_sec(msg) for msg in odom_messages]
        sync_delta: float | None
        if map_stamp <= 0.0:
            odom_msg = odom_messages[-1]
            sync_delta = None
        else:
            tolerance = max(
                0.0,
                float(self.get_parameter('sync_tolerance_sec').value),
            )
            nearest = nearest_timestamp_index(
                map_stamp,
                odom_stamps,
                tolerance,
            )
            if nearest is None:
                # Once odometry has moved beyond this map's tolerance window,
                # no future sample can make the pair valid. Keep the last valid
                # pair as the completion fallback instead of waiting forever.
                if (
                    self._completion_pending
                    and max(odom_stamps) > map_stamp + tolerance
                ):
                    self._pending_map = None
                    self._finalize_completion()
                return
            index, sync_delta = nearest
            odom_msg = odom_messages[index]

        self._pending_map = None
        self._latest_pair = (map_msg, odom_msg, sync_delta)
        if self._completion_pending:
            self._finalize_completion()
        else:
            self._capture_pair(
                map_msg,
                odom_msg,
                sync_delta,
                'distance_or_time',
            )

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
            'max_odom_step_m': self._policy.max_odom_step_m,
            'pgm_unknown_value': 127,
            'raw_encoding': 'signed int8, ROS row-major order',
        }
        payload['sync_tolerance_sec'] = float(
            self.get_parameter('sync_tolerance_sec').value
        )
        write_json_atomic(self._run_dir / 'run.json', payload)

    def _map_robot_pose(self, map_msg: OccupancyGrid) -> dict | None:
        map_frame = map_msg.header.frame_id or 'map'
        robot_frame = str(self.get_parameter('robot_frame').value)
        try:
            transform = self._tf_buffer.lookup_transform(
                map_frame,
                robot_frame,
                Time.from_msg(map_msg.header.stamp),
            )
        except TransformException:
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'frame_id': map_frame,
            'child_frame_id': robot_frame,
            'stamp': {
                'sec': transform.header.stamp.sec,
                'nanosec': transform.header.stamp.nanosec,
            },
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
        sync_delta_sec: float | None,
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
            'map_odom_sync_delta_sec': sync_delta_sec,
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
            'map_robot_pose': self._map_robot_pose(map_msg),
            'files': {
                'pgm': f'{self._sequence:06d}_map.pgm',
                'raw': f'{self._sequence:06d}_occupancy.bin',
                'metadata': f'{self._sequence:06d}_metadata.json',
            },
        }

    def _capture_pair(
        self,
        map_msg: OccupancyGrid,
        odom_msg: Odometry,
        sync_delta_sec: float | None,
        trigger: str,
        *,
        force: bool = False,
    ) -> bool:
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        data = list(map_msg.data)
        if width <= 0 or height <= 0 or len(data) != width * height:
            self.get_logger().warning('Ignoring invalid OccupancyGrid snapshot')
            return False

        now = self._now_sec()
        if not force and not self._policy.should_capture(now):
            return False

        prefix = f'{self._sequence:06d}'
        metadata = self._snapshot_metadata(
            map_msg,
            odom_msg,
            sync_delta_sec,
            trigger,
        )
        try:
            write_snapshot_files(
                self._run_dir,
                self._sequence,
                occupancy_to_pgm(data, width, height),
                occupancy_to_signed_bytes(data),
                metadata,
            )
        except OSError as exc:
            self.get_logger().error(f'Failed to save map snapshot: {exc}')
            return False

        self._policy.record_capture(now)
        self._sequence += 1
        self.get_logger().info(
            f'Saved snapshot {prefix}: trigger={trigger}, '
            f'map={width}x{height}'
        )
        return True

    def _on_completion(self, msg: Bool) -> None:
        if not msg.data or self._recording_stopped:
            return
        self._completion_pending = True
        self._try_sync_pending_map()
        if self._pending_map is not None:
            self.get_logger().info(
                'Waiting for odometry synchronized with the newest map'
            )
            return
        if self._latest_pair is None:
            self.get_logger().warning(
                'Exploration complete received; waiting for a synchronized '
                'map/odom pair before shutdown'
            )
            return
        self._finalize_completion()

    def _finalize_completion(self) -> None:
        if self._latest_pair is None or self._recording_stopped:
            return
        map_msg, odom_msg, sync_delta = self._latest_pair
        saved = self._capture_pair(
            map_msg,
            odom_msg,
            sync_delta,
            'exploration_complete',
            force=True,
        )
        if saved:
            self._recording_stopped = True
            self.get_logger().info('Final snapshot saved; recording complete')
        else:
            self.get_logger().error(
                'Final snapshot write failed; waiting for the next synchronized '
                'pair to retry'
            )
            return
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
