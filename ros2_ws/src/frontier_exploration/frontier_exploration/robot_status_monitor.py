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
_ACTIVE_NAV = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}


def _yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _status_stamp(item: GoalStatus) -> tuple[int, int]:
    stamp = item.goal_info.stamp
    return int(stamp.sec), int(stamp.nanosec)


class RobotStatusMonitor(Node):
    """Subscribe only, summarize health, and write a JSONL status timeline."""

    def __init__(self) -> None:
        super().__init__('robot_status_monitor')
        self._declare_parameters()
        self._start_mono = time.monotonic()

        self._times = {
            'map': deque(maxlen=200),
            'odom': deque(maxlen=1000),
            'scan': deque(maxlen=500),
        }
        self._last = {'map': None, 'odom': None, 'scan': None}
        self._map: OccupancyGrid | None = None
        self._odom: Odometry | None = None
        self._scan: LaserScan | None = None

        self._goal: tuple[float, float] | None = None
        self._goal_frame = 'map'
        self._path_length_m: float | None = None
        self._nav_active = False
        self._nav_status = 'NO DATA'
        self._last_nav_result = 'NONE'
        self._success_count = 0
        self._failure_count = 0
        self._consecutive_failures = 0
        self._exploration_complete = False
        self._stationary_since: float | None = None

        self._tf_missing_since: dict[str, float] = {}
        self._map_pose: tuple[float, float, float] | None = None

        self._run_name = str(self.get_parameter('run_name').value).strip()
        self._run_json = self._resolve_run_json()
        self._jsonl_path = self._resolve_jsonl_path()
        self._jsonl_error: str | None = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._create_subscriptions()

        period = max(
            0.5,
            float(self.get_parameter('display_period_sec').value),
        )
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'Read-only status monitor started. It does not publish, call '
            'services/actions, or send motion commands.'
        )
        self.get_logger().info(f'Status timeline: {self._jsonl_path}')

    def _declare_parameters(self) -> None:
        for name, value in (
            ('map_topic', '/map'),
            ('odom_topic', '/odom'),
            ('scan_topic', '/scan'),
            ('path_topic', '/frontier_selected_path'),
            ('failed_goal_topic', '/frontier_failed_goal'),
            ('completed_goal_topic', '/frontier_completed_goal'),
            ('completion_topic', '/exploration_complete'),
            ('navigate_action', '/navigate_to_pose'),
            ('map_frame', 'map'),
            ('odom_frame', 'odom'),
            ('base_frame', 'base_link'),
            ('display_period_sec', 2.0),
            ('rate_window_sec', 5.0),
            ('startup_grace_sec', 15.0),
            ('clear_screen', True),
            ('moving_linear_threshold_mps', 0.02),
            ('moving_angular_threshold_rps', 0.05),
            ('odom_warning_sec', 2.0),
            ('odom_critical_sec', 5.0),
            ('scan_warning_sec', 2.0),
            ('scan_critical_sec', 5.0),
            ('map_warning_sec', 8.0),
            ('map_critical_sec', 20.0),
            ('tf_critical_sec', 5.0),
            ('navigation_stall_warning_sec', 20.0),
            ('consecutive_nav_failures_warning', 3),
            ('run_name', ''),
            ('lama_runs_dir', 'data/lama_runs'),
            ('status_output_dir', 'data/robot_status'),
            ('write_jsonl', True),
        ):
            self.declare_parameter(name, value)

    def _create_subscriptions(self) -> None:
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        transient_qos = QoSProfile(
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
            LaserScan,
            str(self.get_parameter('scan_topic').value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            NavPath,
            str(self.get_parameter('path_topic').value),
            self._on_path,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('failed_goal_topic').value),
            self._on_failed_goal,
            10,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('completed_goal_topic').value),
            self._on_completed_goal,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('completion_topic').value),
            self._on_completion,
            transient_qos,
        )
        action_name = str(
            self.get_parameter('navigate_action').value
        ).rstrip('/')
        self.create_subscription(
            GoalStatusArray,
            f'{action_name}/_action/status',
            self._on_nav_status,
            10,
        )

    def _resolve_run_json(self) -> Path | None:
        if not self._run_name:
            return None
        runs_dir = resolve_project_path(
            str(self.get_parameter('lama_runs_dir').value)
        )
        return runs_dir / self._run_name / 'run.json'

    def _resolve_jsonl_path(self) -> Path:
        output_dir = resolve_project_path(
            str(self.get_parameter('status_output_dir').value)
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        stem = self._run_name
        if not stem:
            stem = datetime.now(timezone.utc).strftime(
                'monitor_%Y%m%d_%H%M%S_utc'
            )
        return output_dir / f'{stem}_robot_status.jsonl'

    def _receipt(self, key: str) -> float:
        now = time.monotonic()
        self._last[key] = now
        self._times[key].append(now)
        return now

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._receipt('map')
        self._map = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self._receipt('scan')
        self._scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        now = self._receipt('odom')
        self._odom = msg
        linear = abs(float(msg.twist.twist.linear.x))
        angular = abs(float(msg.twist.twist.angular.z))
        linear_limit = float(
            self.get_parameter('moving_linear_threshold_mps').value
        )
        angular_limit = float(
            self.get_parameter('moving_angular_threshold_rps').value
        )
        if linear >= linear_limit or angular >= angular_limit:
            self._stationary_since = None
        elif self._stationary_since is None:
            self._stationary_since = now

    def _on_path(self, msg: NavPath) -> None:
        if not msg.poses:
            return
        goal = msg.poses[-1]
        self._goal = (
            float(goal.pose.position.x),
            float(goal.pose.position.y),
        )
        self._goal_frame = (
            goal.header.frame_id or msg.header.frame_id or 'map'
        )
        length = 0.0
        for first, second in zip(msg.poses, msg.poses[1:]):
            dx = second.pose.position.x - first.pose.position.x
            dy = second.pose.position.y - first.pose.position.y
            length += math.hypot(float(dx), float(dy))
        self._path_length_m = length

    def _on_failed_goal(self, _msg: PointStamped) -> None:
        self._failure_count += 1
        self._consecutive_failures += 1
        self._last_nav_result = 'FAILED'

    def _on_completed_goal(self, _msg: PointStamped) -> None:
        self._success_count += 1
        self._consecutive_failures = 0
        self._last_nav_result = 'SUCCEEDED'

    def _on_completion(self, msg: Bool) -> None:
        self._exploration_complete = bool(msg.data)

    def _on_nav_status(self, msg: GoalStatusArray) -> None:
        was_active = self._nav_active
        self._nav_active = any(
            item.status in _ACTIVE_NAV for item in msg.status_list
        )
        if msg.status_list:
            latest = max(msg.status_list, key=_status_stamp)
            self._nav_status = _STATUS_NAMES.get(
                latest.status,
                f'STATUS_{latest.status}',
            )
        else:
            self._nav_status = 'IDLE'
        if self._nav_active and not was_active:
            self._stationary_since = time.monotonic()

    def _age(self, key: str, now: float) -> float | None:
        stamp = self._last[key]
        if stamp is None:
            return None
        return max(0.0, now - stamp)

    def _thresholds(self) -> HealthThresholds:
        return HealthThresholds(
            odom_warning_sec=float(
                self.get_parameter('odom_warning_sec').value
            ),
            odom_critical_sec=float(
                self.get_parameter('odom_critical_sec').value
            ),
            scan_warning_sec=float(
                self.get_parameter('scan_warning_sec').value
            ),
            scan_critical_sec=float(
                self.get_parameter('scan_critical_sec').value
            ),
            map_warning_sec=float(
                self.get_parameter('map_warning_sec').value
            ),
            map_critical_sec=float(
                self.get_parameter('map_critical_sec').value
            ),
            tf_critical_sec=float(
                self.get_parameter('tf_critical_sec').value
            ),
            navigation_stall_warning_sec=float(
                self.get_parameter('navigation_stall_warning_sec').value
            ),
            consecutive_nav_failures_warning=int(
                self.get_parameter(
                    'consecutive_nav_failures_warning'
                ).value
            ),
        )

    def _read_integrity(self) -> dict:
        result = {
            'available': False,
            'run_invalid': False,
            'invalid_reason': None,
            'first_invalid_sequence': None,
            'map_jump_event_count': 0,
        }
        if self._run_json is None:
            return result
        try:
            payload = json.loads(
                self._run_json.read_text(encoding='utf-8')
            )
        except (OSError, json.JSONDecodeError):
            return result
        events = payload.get('map_jump_events', [])
        result.update(
            {
                'available': True,
                'run_invalid': bool(payload.get('run_invalid', False)),
                'invalid_reason': payload.get('invalid_reason'),
                'first_invalid_sequence': payload.get(
                    'first_invalid_sequence'
                ),
                'map_jump_event_count': (
                    len(events) if isinstance(events, list) else 0
                ),
            }
        )
        return result

    def _lookup_tf(
        self,
        label: str,
        target: str,
        source: str,
        now: float,
    ) -> tuple[object | None, dict]:
        try:
            transform = self._tf_buffer.lookup_transform(
                target,
                source,
                Time(),
            )
        except TransformException as exc:
            since = self._tf_missing_since.setdefault(label, now)
            return None, {
                'ok': False,
                'missing_sec': max(0.0, now - since),
                'error': str(exc),
            }
        self._tf_missing_since.pop(label, None)
        return transform, {'ok': True, 'missing_sec': 0.0}

    def _tf_state(self, now: float) -> dict[str, dict]:
        map_frame = str(self.get_parameter('map_frame').value)
        odom_frame = str(self.get_parameter('odom_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        result = {}

        _, result['map -> odom'] = self._lookup_tf(
            'map -> odom',
            map_frame,
            odom_frame,
            now,
        )
        _, result['odom -> base'] = self._lookup_tf(
            'odom -> base',
            odom_frame,
            base_frame,
            now,
        )
        if self._scan is not None and self._scan.header.frame_id:
            scan_frame = self._scan.header.frame_id
            label = f'base -> {scan_frame}'
            _, result[label] = self._lookup_tf(
                label,
                base_frame,
                scan_frame,
                now,
            )

        map_base, _ = self._lookup_tf(
            'map -> base',
            map_frame,
            base_frame,
            now,
        )
        if map_base is None:
            self._map_pose = None
        else:
            trans = map_base.transform.translation
            rot = map_base.transform.rotation
            self._map_pose = (
                float(trans.x),
                float(trans.y),
                _yaw(rot),
            )
        return result

    def _odom_stats(self) -> dict:
        if self._odom is None:
            return {}
        pose = self._odom.pose.pose
        twist = self._odom.twist.twist
        return {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw_rad': _yaw(pose.orientation),
            'linear_mps': float(twist.linear.x),
            'angular_rps': float(twist.angular.z),
        }

    def _map_stats(self) -> dict:
        if self._map is None:
            return {}
        total = len(self._map.data)
        known = sum(1 for value in self._map.data if value >= 0)
        return {
            'width': int(self._map.info.width),
            'height': int(self._map.info.height),
            'resolution_m': float(self._map.info.resolution),
            'known_cells': known,
            'unknown_cells': total - known,
            'known_fraction': (
                float(known) / float(total) if total else 0.0
            ),
            'origin_x': float(self._map.info.origin.position.x),
            'origin_y': float(self._map.info.origin.position.y),
        }

    @staticmethod
    def _fmt(value: float | None, digits: int = 2) -> str:
        if value is None:
            return 'N/A'
        return f'{value:.{digits}f}'

    @staticmethod
    def _rate(value: float | None) -> str:
        return 'N/A' if value is None else f'{value:.2f} Hz'

    @staticmethod
    def _age_text(value: float | None) -> str:
        return 'N/A' if value is None else f'{value:.2f} s'

    @staticmethod
    def _stream_marker(
        age: float | None,
        warning: float,
        critical: float,
    ) -> str:
        if age is None:
            return '⏳ WAITING'
        if age >= critical:
            return '🔴 LOST'
        if age >= warning:
            return '⚠️ STALE'
        return '✅ OK'

    def _dashboard(self, state: dict) -> str:
        health = state['health']
        th = state['thresholds']
        ages = state['ages_sec']
        rates = state['rates_hz']
        odom = state['odom']
        map_stats = state['map']
        integrity = state['integrity']
        tf_state = state['tf']

        nav_state = 'COMPLETE' if self._exploration_complete else (
            'NAVIGATING' if self._nav_active else 'IDLE'
        )
        moving = 'UNKNOWN'
        if odom:
            moving = (
                'STATIONARY'
                if state['stationary_sec'] is not None
                else 'MOVING'
            )
        run_state = 'UNKNOWN'
        if integrity['available']:
            run_state = (
                'INVALID' if integrity['run_invalid'] else 'VALID'
            )

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
        lines.extend(
            [
                '-' * 72,
                (
                    f'Robot: {moving} | Navigation: {nav_state} | '
                    f'Dataset/run: {run_state}'
                ),
                '-' * 72,
                'ROBOT',
            ]
        )

        if self._map_pose is None:
            lines.append('  Pose map         : N/A')
        else:
            x, y, yaw = self._map_pose
            lines.append(
                f'  Pose map         : x={x:.2f} m | y={y:.2f} m | '
                f'yaw={math.degrees(yaw):.1f}°'
            )
        if odom:
            lines.append(
                f"  Pose odom        : x={odom['x']:.2f} m | "
                f"y={odom['y']:.2f} m | "
                f"yaw={math.degrees(odom['yaw_rad']):.1f}°"
            )
            lines.append(
                f"  Velocity         : linear={odom['linear_mps']:.3f} m/s | "
                f"angular={odom['angular_rps']:.3f} rad/s"
            )
        else:
            lines.append('  Pose odom        : N/A')
            lines.append('  Velocity         : N/A')
        stationary = state['stationary_sec']
        lines.append(
            '  Stationary time  : '
            + ('0.0 s / moving' if stationary is None else f'{stationary:.1f} s')
        )

        lines.extend(['', 'NAVIGATION / FRONTIER'])
        lines.append(f'  State            : {nav_state}')
        lines.append(f'  Nav2 action      : {self._nav_status}')
        if self._goal is None:
            lines.append('  Selected goal    : N/A')
        else:
            lines.append(
                f'  Selected goal    : x={self._goal[0]:.2f} m | '
                f'y={self._goal[1]:.2f} m | frame={self._goal_frame}'
            )
        lines.append(
            f"  Path length      : {self._fmt(self._path_length_m)} m"
        )
        lines.append(
            f"  Goal distance    : {self._fmt(state['goal_distance_m'])} m"
        )
        lines.append(f'  Last result      : {self._last_nav_result}')
        lines.append(
            f'  Goal results     : success={self._success_count} | '
            f'failed={self._failure_count} | '
            f'consecutive_failed={self._consecutive_failures}'
        )

        lines.extend(['', 'SLAM / MAP'])
        if map_stats:
            total = map_stats['known_cells'] + map_stats['unknown_cells']
            lines.append(
                f"  Map size         : {map_stats['width']} x "
                f"{map_stats['height']} cells"
            )
            lines.append(
                f"  Resolution       : {map_stats['resolution_m']:.3f} m/cell"
            )
            lines.append(
                f"  Known cells      : {map_stats['known_cells']:,} / "
                f"{total:,} ({map_stats['known_fraction'] * 100.0:.2f}%)"
            )
            lines.append(
                f"  Source origin    : x={map_stats['origin_x']:.2f} | "
                f"y={map_stats['origin_y']:.2f}"
            )
        else:
            lines.append('  Map              : N/A')
        map_mark = self._stream_marker(
            ages['map'], th.map_warning_sec, th.map_critical_sec
        )
        lines.append(
            f"  /map             : {map_mark} | rate={self._rate(rates['map'])} | "
            f"age={self._age_text(ages['map'])}"
        )
        integrity_mark = '❔ UNKNOWN'
        if integrity['available']:
            integrity_mark = (
                '🔴 INVALID' if integrity['run_invalid'] else '✅ VALID'
            )
        lines.append(f'  Run integrity    : {integrity_mark}')
        if integrity['run_invalid']:
            lines.append(
                f"  Invalid reason   : {integrity['invalid_reason']} | "
                f"first_sequence={integrity['first_invalid_sequence']} | "
                f"map_jump_events={integrity['map_jump_event_count']}"
            )

        lines.extend(['', 'SENSORS'])
        scan_mark = self._stream_marker(
            ages['scan'], th.scan_warning_sec, th.scan_critical_sec
        )
        odom_mark = self._stream_marker(
            ages['odom'], th.odom_warning_sec, th.odom_critical_sec
        )
        lines.append(
            f"  /scan            : {scan_mark} | rate={self._rate(rates['scan'])} | "
            f"age={self._age_text(ages['scan'])}"
        )
        lines.append(
            f"  /odom            : {odom_mark} | rate={self._rate(rates['odom'])} | "
            f"age={self._age_text(ages['odom'])}"
        )
        if self._scan is not None:
            lines.append(
                f'  LiDAR frame      : {self._scan.header.frame_id or "N/A"}'
            )

        lines.extend(['', 'TF'])
        for label, item in tf_state.items():
            if item['ok']:
                lines.append(f'  {label:<17}: ✅ OK')
            else:
                marker = (
                    '🔴 LOST'
                    if item['missing_sec'] >= th.tf_critical_sec
                    else '⚠️ MISSING'
                )
                lines.append(
                    f"  {label:<17}: {marker} | "
                    f"missing={item['missing_sec']:.1f} s"
                )

        overall_mark = {
            STATUS_OK: '✅ NORMAL',
            STATUS_WARNING: '⚠️ CHECK',
            STATUS_NOT_OK: '🔴 NOT OK',
        }[health['status']]
        lines.extend(['', 'HEALTH CHECK'])
        lines.append(f'  Overall          : {overall_mark}')
        lines.append(
            '  Map/run integrity: '
            + (
                '🔴 NOT OK'
                if integrity['run_invalid']
                else ('✅ NORMAL' if integrity['available'] else '❔ UNKNOWN')
            )
        )
        lines.append(
            '  Navigation       : '
            + ('⚠️ CHECK' if state['nav_warning'] else '✅ NORMAL')
        )
        lines.append(f'  JSONL timeline   : {self._jsonl_path}')
        if self._jsonl_error:
            lines.append(f'  JSONL write      : 🔴 ERROR - {self._jsonl_error}')
        lines.extend(['=' * 72, ''])
        return '\n'.join(lines)

    def _write_jsonl(self, state: dict) -> None:
        if not bool(self.get_parameter('write_jsonl').value):
            return
        payload = dict(state)
        payload['thresholds'] = state['thresholds'].__dict__
        try:
            with self._jsonl_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(payload, ensure_ascii=False) + '\n')
        except OSError as exc:
            self._jsonl_error = str(exc)

    def _tick(self) -> None:
        now = time.monotonic()
        window = max(
            0.5,
            float(self.get_parameter('rate_window_sec').value),
        )
        ages = {name: self._age(name, now) for name in self._last}
        rates = {
            name: topic_rate_hz(list(stamps), now, window)
            for name, stamps in self._times.items()
        }
        thresholds = self._thresholds()
        integrity = self._read_integrity()
        tf_state = self._tf_state(now)
        tf_missing = {
            name: item['missing_sec']
            for name, item in tf_state.items()
            if not item['ok']
        }
        stationary = (
            None
            if self._stationary_since is None
            else max(0.0, now - self._stationary_since)
        )
        startup_complete = (
            now - self._start_mono
            >= float(self.get_parameter('startup_grace_sec').value)
        )
        health = classify_health(
            startup_complete=startup_complete,
            run_invalid=integrity['run_invalid'],
            run_invalid_reason=integrity['invalid_reason'],
            odom_age_sec=ages['odom'],
            scan_age_sec=ages['scan'],
            map_age_sec=ages['map'],
            tf_missing_durations_sec=tf_missing,
            nav_active=self._nav_active,
            stationary_duration_sec=stationary,
            consecutive_nav_failures=self._consecutive_failures,
            exploration_complete=self._exploration_complete,
            thresholds=thresholds,
        )

        goal_distance = None
        map_frame = str(self.get_parameter('map_frame').value)
        if (
            self._goal is not None
            and self._map_pose is not None
            and self._goal_frame == map_frame
        ):
            goal_distance = math.hypot(
                self._goal[0] - self._map_pose[0],
                self._goal[1] - self._map_pose[1],
            )
        nav_warning = (
            self._consecutive_failures
            >= thresholds.consecutive_nav_failures_warning
            or (
                self._nav_active
                and stationary is not None
                and stationary
                >= thresholds.navigation_stall_warning_sec
            )
        )

        state = {
            'utc': datetime.now(timezone.utc).isoformat(),
            'ros_time_sec': self.get_clock().now().nanoseconds / 1e9,
            'run_name': self._run_name or None,
            'health': health,
            'thresholds': thresholds,
            'ages_sec': ages,
            'rates_hz': rates,
            'odom': self._odom_stats(),
            'map_pose': self._map_pose,
            'map': self._map_stats(),
            'tf': tf_state,
            'nav_active': self._nav_active,
            'nav_action_status': self._nav_status,
            'selected_goal': self._goal,
            'selected_goal_frame': self._goal_frame,
            'selected_path_length_m': self._path_length_m,
            'goal_distance_m': goal_distance,
            'last_nav_result': self._last_nav_result,
            'successful_goals': self._success_count,
            'failed_goals': self._failure_count,
            'consecutive_nav_failures': self._consecutive_failures,
            'stationary_sec': stationary,
            'exploration_complete': self._exploration_complete,
            'integrity': integrity,
            'nav_warning': nav_warning,
        }
        self._write_jsonl(state)
        block = self._dashboard(state)
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
