"""Operator-first live view for the read-only TurtleBot4 status monitor."""

import math
import time

import rclpy
from geometry_msgs.msg import PointStamped, Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

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
        self.declare_parameter('cmd_vel_nav_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_smoothed_topic', '/cmd_vel_smoothed')
        self.declare_parameter('cmd_vel_final_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_fresh_sec', 3.0)
        self.declare_parameter('cmd_linear_motion_threshold_mps', 0.03)
        self.declare_parameter('cmd_angular_motion_threshold_rps', 0.05)

        self._frontier: tuple[float, float] | None = None
        self._frontier_frame = 'map'
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
        if getattr(self, '_robot_trapped', False):
            return 'Đã xác nhận robot bị kẹt; không nhận thêm frontier goal mới.'
        if integrity['run_invalid']:
            return 'Phát hiện map/run không còn đáng tin; nên dừng run.'
        if self._exploration_complete:
            return 'Đã hoàn tất exploration.'
        if self._nav_active:
            if self._frontier is not None:
                return 'Đang di chuyển tới frontier đã chọn.'
            return 'Đang thực thi goal Nav2.'
        if self._consecutive_failures > 0:
            return 'Goal trước thất bại; đang tìm frontier/đường khác.'
        if self._frontier is not None or self._goal is not None:
            return 'Đã chọn frontier; đang chờ/kiểm tra đường đi với Nav2.'
        return 'Đang quét map và tìm/kiểm tra frontier tiếp theo.'

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
            return 'Chưa có frontier đang active / đang tìm.'
        x, y = self._frontier
        distance = None
        map_pose = state['map_pose']
        map_frame = str(self.get_parameter('map_frame').value)
        if map_pose is not None and self._frontier_frame == map_frame:
            distance = math.hypot(x - map_pose[0], y - map_pose[1])
        suffix = '' if distance is None else f' | cách robot {distance:.2f} m'
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
        return (
            f'{self._nav_status} | goal còn {goal_distance} m | '
            f'path {path} m | last={self._last_nav_result}'
        )

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

        lines = [
            '=' * 72,
            f'ROBOT         : {health_mark}',
            f'ĐANG LÀM GÌ  : {self._operator_thought(state)}',
            f'FRONTIER      : {self._frontier_text(state)}',
            f'TỐC ĐỘ        : {self._speed_text(state)}',
            f'NAVIGATION    : {self._navigation_text(state)}',
            f'CẢN TRỞ       : {self._obstruction_text(state)}',
            (
                'MAP / DATA    : '
                + (
                    '🔴 INVALID'
                    if state['integrity']['run_invalid']
                    else '✅ Bình thường'
                )
            ),
            '=' * 72,
        ]
        return '\n'.join(lines) + '\n'

    def _write_jsonl(self, state: dict) -> None:
        enriched = dict(state)
        enriched['selected_frontier'] = self._frontier
        enriched['selected_frontier_frame'] = self._frontier_frame
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
