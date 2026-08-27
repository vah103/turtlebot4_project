"""Operator-first live view for the read-only TurtleBot4 status monitor."""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32

from frontier_exploration.robot_status_core import (
    STATUS_NOT_OK,
    STATUS_OK,
    STATUS_WARNING,
)
from frontier_exploration.robot_status_monitor import RobotStatusMonitor


class CompactRobotStatusMonitor(RobotStatusMonitor):
    """Show operator-relevant state while full diagnostics stay in JSONL."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('selected_frontier_topic', '/frontier_selected')
        self.declare_parameter('trapped_topic', '/robot_trapped')
        self.declare_parameter('frontier_cell_count_topic', '/frontier_cell_count')
        self.declare_parameter('frontier_group_count_topic', '/frontier_group_count')
        self.declare_parameter(
            'frontier_blacklist_count_topic', '/frontier_blacklist_count'
        )
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_smoothed_topic', '/cmd_vel_smoothed')
        self.declare_parameter('cmd_vel_final_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_fresh_sec', 3.0)
        self.declare_parameter('cmd_linear_motion_threshold_mps', 0.03)
        self.declare_parameter('cmd_angular_motion_threshold_rps', 0.05)

        self._frontier: tuple[float, float] | None = None
        self._frontier_frame = 'map'
        self._frontier_cells: int | None = None
        self._frontier_groups: int | None = None
        self._blacklist_count: int | None = None
        self._last_dashboard_known_cells: int | None = None
        self._robot_trapped = False
        self._cmd_vel_chain: dict[str, tuple[float, float, float] | None] = {
            'nav': None,
            'smoothed': None,
            'final': None,
        }

        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('selected_frontier_topic').value),
            self._on_selected_frontier,
            transient_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter('trapped_topic').value),
            self._on_robot_trapped,
            transient_qos,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('frontier_cell_count_topic').value),
            lambda msg: self._on_frontier_count('cells', msg),
            transient_qos,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('frontier_group_count_topic').value),
            lambda msg: self._on_frontier_count('groups', msg),
            transient_qos,
        )
        self.create_subscription(
            Int32,
            str(self.get_parameter('frontier_blacklist_count_topic').value),
            lambda msg: self._on_frontier_count('blacklist', msg),
            transient_qos,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_nav_topic').value),
            lambda msg: self._on_cmd_vel('nav', msg),
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_smoothed_topic').value),
            lambda msg: self._on_cmd_vel('smoothed', msg),
            20,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter('cmd_vel_final_topic').value),
            lambda msg: self._on_cmd_vel('final', msg),
            20,
        )

    def _on_selected_frontier(self, msg: PointStamped) -> None:
        self._frontier = (float(msg.point.x), float(msg.point.y))
        self._frontier_frame = msg.header.frame_id or 'map'

    def _on_robot_trapped(self, msg: Bool) -> None:
        self._robot_trapped = bool(msg.data)

    def _on_frontier_count(self, key: str, msg: Int32) -> None:
        value = max(0, int(msg.data))
        if key == 'cells':
            self._frontier_cells = value
        elif key == 'groups':
            self._frontier_groups = value
        elif key == 'blacklist':
            self._blacklist_count = value

    def _on_cmd_vel(self, key: str, msg: Twist) -> None:
        self._cmd_vel_chain[key] = (
            time.monotonic(),
            float(msg.linear.x),
            float(msg.angular.z),
        )

    def _on_failed_goal(self, msg: PointStamped) -> None:
        super()._on_failed_goal(msg)
        self._frontier = None

    def _on_completed_goal(self, msg: PointStamped) -> None:
        super()._on_completed_goal(msg)
        self._frontier = None

    @staticmethod
    def _clip(value: str, limit: int = 92) -> str:
        text = str(value).replace('\n', ' ').strip()
        if len(text) <= limit:
            return text
        return text[: max(1, limit - 3)] + '...'

    @staticmethod
    def _count_text(value: int | None) -> str:
        return 'N/A' if value is None else str(value)

    def _cmd_snapshot(self) -> dict[str, dict | None]:
        now = time.monotonic()
        result: dict[str, dict | None] = {}
        for key, sample in getattr(self, '_cmd_vel_chain', {}).items():
            if sample is None:
                result[key] = None
                continue
            stamp, linear, angular = sample
            result[key] = {
                'linear_mps': linear,
                'angular_rps': angular,
                'age_sec': max(0.0, now - stamp),
            }
        return result

    def _cmd_is_fresh(self, sample: dict | None) -> bool:
        if not sample:
            return False
        fresh = max(0.0, float(self.get_parameter('cmd_vel_fresh_sec').value))
        return sample['age_sec'] <= fresh

    def _cmd_is_motion(self, sample: dict | None) -> bool:
        if not self._cmd_is_fresh(sample):
            return False
        linear_threshold = float(
            self.get_parameter('cmd_linear_motion_threshold_mps').value
        )
        angular_threshold = float(
            self.get_parameter('cmd_angular_motion_threshold_rps').value
        )
        return (
            abs(sample['linear_mps']) >= linear_threshold
            or abs(sample['angular_rps']) >= angular_threshold
        )

    def _operator_thought(self, state: dict) -> str:
        integrity = state['integrity']
        stationary = state.get('stationary_sec')
        stall_threshold = state['thresholds'].navigation_stall_warning_sec
        if getattr(self, '_robot_trapped', False):
            return 'Đã xác nhận robot bị kẹt; không nhận thêm frontier goal mới.'
        if integrity['run_invalid']:
            return 'Phát hiện map/run không còn đáng tin; nên dừng run.'
        if self._exploration_complete:
            return 'Đã hoàn tất exploration.'
        if self._nav_active:
            if stationary is not None and stationary >= stall_threshold:
                return (
                    f'Nav2 vẫn active nhưng robot đã đứng yên {stationary:.1f}s; '
                    'đang nghi kẹt/recovery.'
                )
            if self._frontier is not None and self._path_length_m is not None:
                return 'Đang đi tới frontier đã chọn; Nav2 đã tạo được global path.'
            if self._frontier is not None:
                return 'Đã chọn frontier; Nav2 đang cố tạo/duy trì đường tới goal.'
            return 'Đang thực thi goal Nav2.'
        if self._consecutive_failures > 0:
            return 'Goal trước thất bại; đang bỏ goal đó và tìm frontier khác.'
        if self._frontier is not None or self._goal is not None:
            return 'Đã chọn frontier; đang chờ Nav2 nhận goal/tạo path.'
        return 'Đang quét map, tìm frontier và chọn goal gần nhất tiếp theo.'

    def _cmd_obstruction_hint(self, state: dict) -> str | None:
        snapshot = self._cmd_snapshot()
        nav = snapshot.get('nav')
        smoothed = snapshot.get('smoothed')
        final = snapshot.get('final')
        nav_fresh = self._cmd_is_fresh(nav)
        smoothed_fresh = self._cmd_is_fresh(smoothed)
        final_fresh = self._cmd_is_fresh(final)
        nav_motion = self._cmd_is_motion(nav)
        smoothed_motion = self._cmd_is_motion(smoothed)
        final_motion = self._cmd_is_motion(final)

        if nav_motion and smoothed_fresh and not smoothed_motion:
            return '⚠️ Nav2 đang ra lệnh nhưng velocity_smoother đưa vận tốc về gần 0.'
        if (nav_motion or smoothed_motion) and final_fresh and not final_motion:
            return '⚠️ Lệnh chuyển động bị chặn sau controller/smoother trước khi tới robot.'
        if final_motion:
            odom = state.get('odom') or {}
            moving = (
                abs(float(odom.get('linear_mps', 0.0))) >= 0.02
                or abs(float(odom.get('angular_rps', 0.0))) >= 0.05
            )
            if not moving:
                return '🔴 /cmd_vel có lệnh nhưng odometry không chuyển động: nghi va chạm/physics.'
        if nav_fresh and not nav_motion:
            return '⚠️ Controller đang tạo vận tốc gần 0 tại goal/path hiện tại.'
        if smoothed_fresh and not smoothed_motion and final_fresh and not final_motion:
            return '⚠️ Chuỗi vận tốc đang ở gần 0; controller/recovery chưa tạo chuyển động.'
        return None

    def _obstruction_text(self, state: dict) -> str:
        health = state['health']
        stationary = state['stationary_sec']
        threshold = state['thresholds'].navigation_stall_warning_sec

        if getattr(self, '_robot_trapped', False):
            return '🔴 ROBOT_TRAPPED: nhiều goal fail tại cùng pose; đã chặn goal mới.'
        if health['critical_reasons']:
            return '🔴 ' + self._clip(health['critical_reasons'][0], 84)
        if (
            self._nav_active
            and stationary is not None
            and stationary >= threshold
        ):
            hint = self._cmd_obstruction_hint(state)
            if hint is not None:
                return hint
            return (
                '⚠️ Robot đứng yên quá lâu khi có goal; có thể đang kẹt '
                'hoặc recovery.'
            )
        if self._consecutive_failures > 0:
            return (
                f'⚠️ Có {self._consecutive_failures} goal thất bại liên tiếp; '
                'đang chọn lại.'
            )
        if health['warnings']:
            return '⚠️ ' + self._clip(health['warnings'][0], 84)
        return '✅ Không phát hiện cản trở bất thường.'

    def _frontier_text(self, state: dict) -> str:
        if self._frontier is None:
            return 'Chưa có goal frontier active.'
        x, y = self._frontier
        distance = None
        map_pose = state['map_pose']
        map_frame = str(self.get_parameter('map_frame').value)
        if map_pose is not None and self._frontier_frame == map_frame:
            distance = math.hypot(x - map_pose[0], y - map_pose[1])
        suffix = '' if distance is None else f' | còn {distance:.2f} m'
        return f'({x:.2f}, {y:.2f}){suffix}'

    def _speed_text(self, state: dict) -> str:
        odom = state['odom']
        if not odom:
            return 'N/A'
        return (
            f"tiến {odom['linear_mps']:.3f} m/s | "
            f"quay {odom['angular_rps']:.3f} rad/s"
        )

    def _navigation_text(self, state: dict) -> str:
        path = self._fmt(self._path_length_m)
        goal_distance = self._fmt(state['goal_distance_m'])
        path_state = 'có path' if self._path_length_m is not None else 'chưa có path'
        return (
            f'{self._nav_status} | {path_state} {path} m | '
            f'goal còn {goal_distance} m | last={self._last_nav_result}'
        )

    def _scan_text(self, state: dict) -> str:
        rate = state.get('rates_hz', {}).get('scan')
        rate_text = 'N/A' if rate is None else f'{rate:.2f} Hz'
        if self._scan is None:
            return f'{rate_text} | chưa có scan'
        ranges = [
            float(value)
            for value in self._scan.ranges
            if math.isfinite(float(value))
            and float(value) >= float(self._scan.range_min)
            and float(value) <= float(self._scan.range_max)
        ]
        nearest = 'N/A' if not ranges else f'{min(ranges):.2f} m'
        return (
            f'{rate_text} | {len(self._scan.ranges)} rays | '
            f'valid {len(ranges)} | gần nhất {nearest}'
        )

    def _map_progress_text(self, state: dict) -> str:
        stats = state.get('map') or {}
        if not stats:
            return 'N/A'
        known = int(stats.get('known_cells', 0))
        if self._last_dashboard_known_cells is None:
            delta_text = 'mốc đầu'
        else:
            delta = known - self._last_dashboard_known_cells
            sign = '+' if delta >= 0 else ''
            delta_text = f'{sign}{delta:,} từ báo cáo trước'
        self._last_dashboard_known_cells = known
        fraction = float(stats.get('known_fraction', 0.0)) * 100.0
        map_rate = state.get('rates_hz', {}).get('map')
        rate_text = 'N/A' if map_rate is None else f'{map_rate:.2f} Hz'
        return f'known {known:,} ({fraction:.2f}%) | {delta_text} | /map {rate_text}'

    def _stationary_text(self, state: dict) -> str:
        value = state.get('stationary_sec')
        if value is None:
            return 'đang chuyển động'
        return f'{value:.1f} s'

    def _tf_text(self, state: dict) -> str:
        items = []
        for label, value in state.get('tf', {}).items():
            if value.get('ok'):
                items.append(f'{label}=OK')
            else:
                items.append(f"{label}=MISSING {value.get('missing_sec', 0.0):.1f}s")
        return ' | '.join(items) if items else 'N/A'

    def _dashboard(self, state: dict) -> str:
        health = state['health']
        if getattr(self, '_robot_trapped', False):
            health_mark = '🔴 KHÔNG ỔN - ROBOT BỊ KẸT, NÊN DỪNG RUN'
        else:
            health_mark = {
                STATUS_OK: '✅ HOẠT ĐỘNG BÌNH THƯỜNG',
                STATUS_WARNING: '⚠️ VẪN CHẠY NHƯNG CẦN THEO DÕI',
                STATUS_NOT_OK: '🔴 KHÔNG ỔN - NÊN DỪNG RUN',
            }[health['status']]

        map_pose = state.get('map_pose')
        if map_pose is None:
            pose_text = 'N/A'
        else:
            pose_text = (
                f'x={map_pose[0]:.2f}, y={map_pose[1]:.2f}, '
                f'yaw={math.degrees(map_pose[2]):.1f}°'
            )

        frontier_stats = (
            f'{self._count_text(self._frontier_cells)} cells | '
            f'{self._count_text(self._frontier_groups)} groups | '
            f'blacklist {self._count_text(self._blacklist_count)}'
        )

        lines = [
            '=' * 82,
            f'ROBOT          : {health_mark}',
            f'SUY NGHĨ       : {self._operator_thought(state)}',
            '-' * 82,
            f'VỊ TRÍ         : {pose_text}',
            f'TỐC ĐỘ         : {self._speed_text(state)}',
            f'ĐỨNG YÊN       : {self._stationary_text(state)}',
            f'FRONTIER       : {frontier_stats}',
            f'GOAL ĐANG CHỌN : {self._frontier_text(state)}',
            f'NAV2           : {self._navigation_text(state)}',
            (
                'KẾT QUẢ GOAL   : '
                f'success={self._success_count} | failed={self._failure_count} | '
                f'fail liên tiếp={self._consecutive_failures}'
            ),
            f'ĐANG QUÉT      : {self._scan_text(state)}',
            f'MAP MỞ RỘNG    : {self._map_progress_text(state)}',
            f'TF             : {self._tf_text(state)}',
            f'CẢN TRỞ        : {self._obstruction_text(state)}',
            '=' * 82,
        ]
        return '\n'.join(lines) + '\n'

    def _write_jsonl(self, state: dict) -> None:
        enriched = dict(state)
        enriched['selected_frontier'] = self._frontier
        enriched['selected_frontier_frame'] = self._frontier_frame
        enriched['frontier_cells'] = self._frontier_cells
        enriched['frontier_groups'] = self._frontier_groups
        enriched['frontier_blacklist_count'] = self._blacklist_count
        enriched['robot_trapped'] = getattr(self, '_robot_trapped', False)
        enriched['cmd_vel_chain'] = self._cmd_snapshot()
        super()._write_jsonl(enriched)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompactRobotStatusMonitor()
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
