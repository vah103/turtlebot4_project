"""Read-only live status dashboard for TurtleBot4 exploration runs."""

from __future__ import annotations

import json
import math
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from frontier_exploration.project_paths import resolve_project_path
from frontier_exploration.robot_status_core import (
    HealthThresholds,
    STATUS_NOT_OK,
    STATUS_OK,
    STATUS_WARNING,
    classify_health,
    topic_rate_hz,
)


_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
_ACTIVE_NAV_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def _yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _stamp_key(status: GoalStatus) -> tuple[int, int]:
    stamp = status.goal_info.stamp
    return int(stamp.sec), int(stamp.nanosec)


class RobotStatusMonitor(Node):
    """Subscribe to robot state only and render an operator-friendly dashboard."""

    def __init__(self) -> None:
        super().__init__('robot_status_monitor')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('path_topic', '/frontier_selected_path')
        self.declare_parameter('failed_goal_topic', '/frontier_failed_goal')
        self.declare_parameter('completed_goal_topic', '/frontier_completed_goal')
        self.declare_parameter('completion_topic', '/exploration_complete')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('display_period_sec', 2.0)
        self.declare_parameter('rate_window_sec', 5.0)
        self.declare_parameter('startup_grace_sec', 15.0)
        self.declare_parameter('clear_screen', True)
        self.declare_parameter('moving_linear_threshold_mps', 0.02)
        self.declare_parameter('moving_angular_threshold_rps', 0.05)

        self.declare_parameter('odom_warning_sec', 2.0)
        self.declare_parameter('odom_critical_sec', 5.0)
        self.declare_parameter('scan_warning_sec', 2.0)
        self.declare_parameter('scan_critical_sec', 5.0)
        self.declare_parameter('map_warning_sec', 8.0)
        self.declare_parameter('map_critical_sec', 20.0)
        self.declare_parameter('tf_critical_sec', 5.0)
        self.declare_parameter('navigation_stall_warning_sec', 20.0)
        self.declare_parameter('consecutive_nav_failures_warning', 3)

        self.declare_parameter('run_name', '')
        self.declare_parameter('lama_runs_dir', 'data/lama_runs')
        self.declare_parameter('status_output_dir', 'data/robot_status')
        self.declare_parameter('write_jsonl', True)

        self._started_monotonic = time.monotonic()
        self._receipt_times: dict[str, deque[float]] = {
            'map': deque(maxlen=200),
            'odom': deque(maxlen=1000),
            'scan': deque(maxlen=500),
        }
        self._last_receipt: dict[str, float | None] = {
            'map': None,
            'odom': None,
            'scan': None,
        }

        self._latest_odom: Odometry | None = None
        self._latest_map: OccupancyGrid | None = None
        self._latest_scan: LaserScan | None = None
        self._selected_goal: tuple[float, float] | None = None
        self._selected_goal_frame = 'map'
        self._selected_path_length_m: float | None = None
        self._nav_active = False
        self._nav_action_status = 'NO DATA'
        self._last_nav_result = 'NONE'
        self._last_nav_event_monotonic: float | None = None
        self._successful_goals = 0
        self._failed_goals = 0
        self._consecutive_nav_failures = 0
        self._exploration_complete = False
        self._stationary_since: float | None = None
        self._tf_missing_since: dict[str, float] = {}
        self._last_tf_status: dict[str, dict] = {}
        self._map_pose: tuple[float, float, float] | None = None

        self._run_name = str(self.get_parameter('run_name').value).strip()
        self._run_json_path: Path | None = None
        if self._run_name:
            runs_dir = resolve_project_path(
                str(self.get_parameter('lama_runs_dir').value)
            )
            self._run_json_path = runs_dir / self._run_name / 'run.json'

        output_dir = resolve_project_path(
            str(self.get_parameter('status_output_dir').value)
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        output_stem = self._run_name or datetime.now(timezone.utc).strftime(
            'monitor_%Y%m%d_%H%M%S_utc'
        )
        self._jsonl_path = output_dir / f'{output_stem}_robot_status.jsonl'
        self._jsonl_error: str | None = None

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

        map_topic = str(self.get_parameter('map_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        scan_topic = str(self.get_parameter('scan_topic').value)
        path_topic = str(self.get_parameter('path_topic').value)
        failed_topic = str(self.get_parameter('failed_goal_topic').value)
        completed_topic = str(self.get_parameter('completed_goal_topic').value)
        completion_topic = str(self.get_parameter('completion_topic').value)
        navigate_action = str(self.get_parameter('navigate_action').value).rstrip('/')
        nav_status_topic = f'{navigate_action}/_action/status'

        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(NavPath, path_topic, self._on_path, 10)
        self.create_subscription(PointStamped, failed_topic, self._on_failed_goal, 10)
        self.create_subscription(
            PointStamped, completed_topic, self._on_completed_goal, 10
        )
        self.create_subscription(
            Bool, completion_topic, self._on_completion, completion_qos
        )
        self.create_subscription(
            GoalStatusArray, nav_status_topic, self._on_nav_status, 10
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        period = max(
            0.5, float(self.get_parameter('display_period_sec').value)
        )
        self.create_timer(period, self._render_and_record)

        self.get_logger().info(
            'Read-only robot status monitor started; no publishers, actions, '
            'services, or motion commands are used.'
        )
        self.get_logger().info(f'Status timeline: {self._jsonl_path}')

    def _record_receipt(self, key: str) -> float:
        now = time.monotonic()
        self._last_receipt[key] = now
        self._receipt_times[key].append(now)
        return now

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._record_receipt('map')
        self._latest_map = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._record_receipt('scan')
        self._latest_scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        now = self._record_receipt('odom')
        self._latest_odom = msg
        linear_threshold = max(
            0.0,
            float(self.get_parameter('moving_linear_threshold_mps').value),
        )
        angular_threshold = max(
            0.0,
            float(self.get_parameter('moving_angular_threshold_rps').value),
        )
        linear = abs(float(msg.twist.twist.linear.x))
        angular = abs(float(msg.twist.twist.angular.z))
        if linear >= linear_threshold or angular >= angular_threshold:
            self._stationary_since = None
        elif self._stationary_since is None:
            self._stationary_since = now

    def _on_path(self, msg: NavPath) -> None:
        if not msg.poses:
            return
        goal = msg.poses[-1]
        self._selected_goal = (
            float(goal.pose.position.x),
            float(goal.pose.position.y),
        )
        self._selected_goal_frame = goal.header.frame_id or msg.header.frame_id or 'map'
        length = 0.0
        for first, second in zip(msg.poses, msg.poses[1:]):
            dx = float(second.pose.position.x - first.pose.position.x)
            dy = float(second.pose.position.y - first.pose.position.y)
            length += math.hypot(dx, dy)
        self._selected_path_length_m = length

    def _on_failed_goal(self, _msg: PointStamped) -> None:
        self._failed_goals += 1
        self._consecutive_nav_failures += 1
        self._last_nav_result = 'FAILED'
        self._last_nav_event_monotonic = time.monotonic()

    def _on_completed_goal(self, _msg: PointStamped) -> None:
        self._successful_goals += 1
        self._consecutive_nav_failures = 0
        self._last_nav_result = 'SUCCEEDED'
        self._last_nav_event_monotonic = time.monotonic()

    def _on_completion(self, msg: Bool) -> None:
        self._exploration_complete = bool(msg.data)

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        was_active = self._nav_active
        self._nav_active = any(
            item.status in _ACTIVE_NAV_STATUSES for item in msg.status_list
        )
        if msg.status_list:
            latest = max(msg.status_list, key=_stamp_key)
            self._nav_action_status = _STATUS_NAMES.get(
                latest.status, f'STATUS_{latest.status}'
            )
        else:
            self._nav_action_status = 'IDLE'

        if self._nav_active and not was_active and self._latest_odom is not None:
            linear = abs(float(self._latest_odom.twist.twist.linear.x))
            angular = abs(float(self._latest_odom.twist.twist.angular.z))
            if linear < float(self.get_parameter('moving_linear_threshold_mps').value) and angular < float(
                self.get_parameter('moving_angular_threshold_rps').value
            ):
                self._stationary_since = time.monotonic()

    def _topic_age(self, key: str, now: float) -> float | None:
        stamp = self._last_receipt[key]
        return None if stamp is None else max(0.0, now - stamp)

    def _thresholds(self) -> HealthThresholds:
        return HealthThresholds(
            odom_warning_sec=float(self.get_parameter('odom_warning_sec').value),
            odom_critical_sec=float(self.get_parameter('odom_critical_sec').value),
            scan_warning_sec=float(self.get_parameter('scan_warning_sec').value),
            scan_critical_sec=float(self.get_parameter('scan_critical_sec').value),
            map_warning_sec=float(self.get_parameter('map_warning_sec').value),
            map_critical_sec=float(self.get_parameter('map_critical_sec').value),
            tf_critical_sec=float(self.get_parameter('tf_critical_sec').value),
            navigation_stall_warning_sec=float(
                self.get_parameter('navigation_stall_warning_sec').value
            ),
            consecutive_nav_failures_warning=int(
                self.get_parameter('consecutive_nav_failures_warning').value
            ),
        )

    def _read_run_integrity(self) -> dict:
        result = {
            'configured': self._run_json_path is not None,
            'available': False,
            'run_invalid': False,
            'invalid_reason': None,
            'first_invalid_sequence': None,
            'map_jump_event_count': 0,
        }
        if self._run_json_path is None:
            return result
        try:
            payload = json.loads(self._run_json_path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return result

        events = payload.get('map_jump_events', [])
        result.update(
            {
                'available': True,
                'run_invalid': bool(payload.get('run_invalid', False)),
                'invalid_reason': payload.get('invalid_reason'),
                'first_invalid_sequence': payload.get('first_invalid_sequence'),
                'map_jump_event_count': len(events) if isinstance(events, list) else 0,
            }
        )
        if isinstance(events, list) and events:
            result['latest_map_jump_event'] = events[-1]
        return result

    def _lookup_tf(self, label: str, target: str, source: str, now: float):
        try:
            transform = self._tf_buffer.lookup_transform(target, source, Time())
        except TransformException as exc:
            since = self._tf_missing_since.setdefault(label, now)
            missing = max(0.0, now - since)
            status = {
                'ok': False,
                'target': target,
                'source': source,
                'missing_sec': missing,
                'error': str(exc),
            }
            self._last_tf_status[label] = status
            return None, status

        self._tf_missing_since.pop(label, None)
        status = {
            'ok': True,
            'target': target,
            'source': source,
            'missing_sec': 0.0,
        }
        self._last_tf_status[label] = status
        return transform, status

    def _collect_tf(self, now: float) -> dict[str, dict]:
        map_frame = str(self.get_parameter('map_frame').value)
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)

        result: dict[str, dict] = {}
        map_odom, result['map -> odom'] = self._lookup_tf(
            'map -> odom', map_frame, odom_frame, now
        )
        _ = map_odom
        odom_base, result['odom -> base'] = self._lookup_tf(
            'odom -> base', odom_frame, base_frame, now
        )
        _ = odom_base

        if self._latest_scan is not None and self._latest_scan.header.frame_id:
            scan_frame = self._latest_scan.header.frame_id
            _scan_tf, result[f'base -> {scan_frame}'] = self._lookup_tf(
                f'base -> {scan_frame}', base_frame, scan_frame, now
            )

        map_base, _map_base_status = self._lookup_tf(
            'map -> base', map_frame, base_frame, now
        )
        if map_base is not None:
            translation = map_base.transform.translation
            rotation = map_base.transform.rotation
            self._map_pose = (
                float(translation.x),
                float(translation.y),
                _yaw_from_quaternion(rotation),
            )
        else:
            self._map_pose = None
        return result

    def _map_statistics(self) -> dict:
        if self._latest_map is None:
            return {}
        msg = self._latest_map
        total = len(msg.data)
        known = sum(1 for value in msg.data if int(value) >= 0)
        unknown = total - known
        return {
            'width': int(msg.info.width),
            'height': int(msg.info.height),
            'resolution_m': float(msg.info.resolution),
            'known_cells': known,
            'unknown_cells': unknown,
            'known_fraction': (float(known) / float(total)) if total else 0.0,
            'source_origin_x': float(msg.info.origin.position.x),
            'source_origin_y': float(msg.info.origin.position.y),
        }

    def _odom_statistics(self) -> dict:
        if self._latest_odom is None:
            return {}
        msg = self._latest_odom
        pose = msg.pose.pose
        twist = msg.twist.twist
        return {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw_rad': _yaw_from_quaternion(pose.orientation),
            'linear_mps': float(twist.linear.x),
            'angular_rps': float(twist.angular.z),
        }

    @staticmethod
    def _fmt_float(value: float | None, digits: int = 2, suffix: str = '') -> str:
        if value is None:
            return 'N/A'
        return f'{value:.{digits}f}{suffix}'

    @staticmethod
    def _fmt_rate(value: float | None) -> str:
        return 'N/A' if value is None else f'{value:.2f} Hz'

    @staticmethod
    def _fmt_age(value: float | None) -> str:
        return 'N/A' if value is None else f'{value:.2f} s'

    def _stream_marker(self, age: float | None, warning: float, critical: float) -> str:
        if age is None:
            return '⏳ WAITING'
        if age >= critical:
            return '🔴 LOST'
        if age >= warning:
            return '⚠️ STALE'
        return '✅ OK'

    def _render_block(self, snapshot: dict) -> str:
        health = snapshot['health']
        thresholds: HealthThresholds = snapshot['thresholds']
        odom = snapshot['odom']
        map_stats = snapshot['map']
        integrity = snapshot['run_integrity']
        rates = snapshot['rates_hz']
        ages = snapshot['ages_sec']
        tf_status = snapshot['tf']

        lines = [
            '=' * 72,
            ' TURTLEBOT4 SYSTEM STATUS',
            '=' * 72,
            f"OVERALL STATUS: {health['headline']}",
            f"Kết luận      : {health['summary']}",
        ]
        for reason in health['critical_reasons']:
            lines.append(f'  - CRITICAL: {reason}')
        for warning in health['warnings']:
            lines.append(f'  - WARNING : {warning}')

        motion_state = 'UNKNOWN'
        stationary = snapshot['stationary_duration_sec']
        if odom:
            motion_state = 'STATIONARY' if stationary is not None else 'MOVING'
        nav_state = 'COMPLETE' if snapshot['exploration_complete'] else (
            'NAVIGATING' if snapshot['nav_active'] else 'IDLE'
        )
        run_state = 'UNKNOWN'
        if integrity['available']:
            run_state = 'INVALID' if integrity['run_invalid'] else 'VALID'

        lines.extend(
            [
                '-' * 72,
                f'Robot: {motion_state} | Navigation: {nav_state} | Dataset/run: {run_state}',
                '-' * 72,
                'ROBOT',
            ]
        )
        if snapshot['map_pose'] is None:
            lines.append('  Pose map         : N/A')
        else:
            mx, my, myaw = snapshot['map_pose']
            lines.append(
                f'  Pose map         : x={mx:.2f} m | y={my:.2f} m | yaw={math.degrees(myaw):.1f}°'
            )
        if odom:
            lines.append(
                f"  Pose odom        : x={odom['x']:.2f} m | y={odom['y']:.2f} m | "
                f"yaw={math.degrees(odom['yaw_rad']):.1f}°"
            )
            lines.append(
                f"  Velocity         : linear={odom['linear_mps']:.3f} m/s | "
                f"angular={odom['angular_rps']:.3f} rad/s"
            )
        else:
            lines.extend(['  Pose odom        : N/A', '  Velocity         : N/A'])
        lines.append(
            f"  Stationary time  : {self._fmt_float(stationary, 1, ' s') if stationary is not None else '0.0 s / moving'}"
        )

        lines.extend(['', 'NAVIGATION / FRONTIER'])
        lines.append(f"  State            : {nav_state}")
        lines.append(f"  Nav2 action      : {snapshot['nav_action_status']}")
        if snapshot['selected_goal'] is None:
            lines.append('  Selected goal    : N/A')
        else:
            gx, gy = snapshot['selected_goal']
            lines.append(
                f"  Selected goal    : x={gx:.2f} m | y={gy:.2f} m | frame={snapshot['selected_goal_frame']}"
            )
        lines.append(
            f"  Path length      : {self._fmt_float(snapshot['selected_path_length_m'], 2, ' m')}"
        )
        lines.append(
            f"  Goal distance    : {self._fmt_float(snapshot['goal_distance_m'], 2, ' m')}"
        )
        lines.append(f"  Last result      : {snapshot['last_nav_result']}")
        lines.append(
            f"  Goal results     : success={snapshot['successful_goals']} | failed={snapshot['failed_goals']} | "
            f"consecutive_failed={snapshot['consecutive_nav_failures']}"
        )

        lines.extend(['', 'SLAM / MAP'])
        if map_stats:
            lines.append(
                f"  Map size         : {map_stats['width']} x {map_stats['height']} cells"
            )
            lines.append(
                f"  Resolution       : {map_stats['resolution_m']:.3f} m/cell"
            )
            lines.append(
                f"  Known cells      : {map_stats['known_cells']:,} / "
                f"{map_stats['known_cells'] + map_stats['unknown_cells']:,} "
                f"({100.0 * map_stats['known_fraction']:.2f}%)"
            )
            lines.append(
                f"  Source origin    : x={map_stats['source_origin_x']:.2f} | y={map_stats['source_origin_y']:.2f}"
            )
        else:
            lines.append('  Map              : N/A')
        lines.append(
            f"  /map             : {self._stream_marker(ages['map'], thresholds.map_warning_sec, thresholds.map_critical_sec)} | "
            f"rate={self._fmt_rate(rates['map'])} | age={self._fmt_age(ages['map'])}"
        )
        lines.append(
            f"  Run integrity    : {'🔴 INVALID' if integrity['run_invalid'] else ('✅ VALID' if integrity['available'] else '❔ UNKNOWN')}"
        )
        if integrity['run_invalid']:
            lines.append(
                f"  Invalid reason   : {integrity['invalid_reason']} | first_sequence={integrity['first_invalid_sequence']} | "
                f"map_jump_events={integrity['map_jump_event_count']}"
            )

        lines.extend(['', 'SENSORS'])
        lines.append(
            f"  /scan            : {self._stream_marker(ages['scan'], thresholds.scan_warning_sec, thresholds.scan_critical_sec)} | "
            f"rate={self._fmt_rate(rates['scan'])} | age={self._fmt_age(ages['scan'])}"
        )
        lines.append(
            f"  /odom            : {self._stream_marker(ages['odom'], thresholds.odom_warning_sec, thresholds.odom_critical_sec)} | "
            f"rate={self._fmt_rate(rates['odom'])} | age={self._fmt_age(ages['odom'])}"
        )
        if self._latest_scan is not None:
            lines.append(f'  LiDAR frame      : {self._latest_scan.header.frame_id or "N/A"}')

        lines.extend(['', 'TF'])
        if not tf_status:
            lines.append('  TF               : N/A')
        else:
            for label, status in tf_status.items():
                if status['ok']:
                    lines.append(f'  {label:<17}: ✅ OK')
                else:
                    marker = (
                        '🔴 LOST'
                        if status['missing_sec'] >= thresholds.tf_critical_sec
                        else '⚠️ MISSING'
                    )
                    lines.append(
                        f"  {label:<17}: {marker} | missing={status['missing_sec']:.1f} s"
                    )

        health_icon = {
            STATUS_OK: '✅ NORMAL',
            STATUS_WARNING: '⚠️ CHECK',
            STATUS_NOT_OK: '🔴 NOT OK',
        }
        lines.extend(
            [
                '',
                'HEALTH CHECK',
                f"  Overall          : {health_icon[health['status']]}",
                f"  Map/run integrity: {'🔴 NOT OK' if integrity['run_invalid'] else ('✅ NORMAL' if integrity['available'] else '❔ UNKNOWN')}",
                f"  Navigation       : {'⚠️ CHECK' if snapshot['nav_warning'] else '✅ NORMAL'}",
                f"  JSONL timeline   : {self._jsonl_path}",
            ]
        )
        if self._jsonl_error:
            lines.append(f'  JSONL write      : 🔴 ERROR - {self._jsonl_error}')
        lines.extend(['=' * 72, ''])
        return '\n'.join(lines)

    def _write_jsonl(self, payload: dict) -> None:
        if not bool(self.get_parameter('write_jsonl').value):
            return
        try:
            with self._jsonl_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(payload, ensure_ascii=False) + '\n')
        except OSError as exc:
            self._jsonl_error = str(exc)

    def _render_and_record(self) -> None:
        now = time.monotonic()
        rate_window = max(0.5, float(self.get_parameter('rate_window_sec').value))
        ages = {key: self._topic_age(key, now) for key in self._last_receipt}
        rates = {
            key: topic_rate_hz(list(values), now, rate_window)
            for key, values in self._receipt_times.items()
        }
        thresholds = self._thresholds()
        tf_status = self._collect_tf(now)
        integrity = self._read_run_integrity()
        stationary_duration = (
            None
            if self._stationary_since is None
            else max(0.0, now - self._stationary_since)
        )
        startup_complete = (
            now - self._started_monotonic
            >= max(0.0, float(self.get_parameter('startup_grace_sec').value))
        )
        tf_missing = {
            label: float(status['missing_sec'])
            for label, status in tf_status.items()
            if not status['ok']
        }
        health = classify_health(
            startup_complete=startup_complete,
            run_invalid=bool(integrity['run_invalid']),
            run_invalid_reason=integrity['invalid_reason'],
            odom_age_sec=ages['odom'],
            scan_age_sec=ages['scan'],
            map_age_sec=ages['map'],
            tf_missing_durations_sec=tf_missing,
            nav_active=self._nav_active,
            stationary_duration_sec=stationary_duration,
            consecutive_nav_failures=self._consecutive_nav_failures,
            exploration_complete=self._exploration_complete,
            thresholds=thresholds,
        )

        goal_distance = None
        if (
            self._selected_goal is not None
            and self._map_pose is not None
            and self._selected_goal_frame == str(self.get_parameter('map_frame').value)
        ):
            goal_distance = math.hypot(
                self._selected_goal[0] - self._map_pose[0],
                self._selected_goal[1] - self._map_pose[1],
            )

        nav_warning = (
            self._consecutive_nav_failures
            >= thresholds.consecutive_nav_failures_warning
            or (
                self._nav_active
                and stationary_duration is not None
                and stationary_duration >= thresholds.navigation_stall_warning_sec
            )
        )

        snapshot = {
            'utc': datetime.now(timezone.utc).isoformat(),
            'ros_time_sec': self.get_clock().now().nanoseconds / 1e9,
            'run_name': self._run_name or None,
            'health': health,
            'thresholds': thresholds,
            'ages_sec': ages,
            'rates_hz': rates,
            'odom': self._odom_statistics(),
            'map_pose': self._map_pose,
            'map': self._map_statistics(),
            'tf': tf_status,
            'nav_active': self._nav_active,
            'nav_action_status': self._nav_action_status,
            'selected_goal': self._selected_goal,
            'selected_goal_frame': self._selected_goal_frame,
            'selected_path_length_m': self._selected_path_length_m,
            'goal_distance_m': goal_distance,
            'last_nav_result': self._last_nav_result,
            'successful_goals': self._successful_goals,
            'failed_goals': self._failed_goals,
            'consecutive_nav_failures': self._consecutive_nav_failures,
            'stationary_duration_sec': stationary_duration,
            'exploration_complete': self._exploration_complete,
            'run_integrity': integrity,
            'nav_warning': nav_warning,
        }

        # Dataclasses are not JSON serializable; keep the file payload plain.
        json_payload = dict(snapshot)
        json_payload['thresholds'] = thresholds.__dict__
        self._write_jsonl(json_payload)

        block = self._render_block(snapshot)
        if bool(self.get_parameter('clear_screen').value):
            sys.stdout.write('\033[2J\033[H')
        sys.stdout.write(block)
        sys.stdout.flush()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStatusMonitor()
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
