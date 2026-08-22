"""Operator-first live view for the read-only TurtleBot4 status monitor."""

import math

import rclpy
from geometry_msgs.msg import PointStamped

from frontier_exploration.robot_status_core import (
    STATUS_NOT_OK,
    STATUS_OK,
    STATUS_WARNING,
)
from frontier_exploration.robot_status_monitor import RobotStatusMonitor


class CompactRobotStatusMonitor(RobotStatusMonitor):
    """Show only operator-relevant state while full diagnostics stay in JSONL."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('selected_frontier_topic', '/frontier_selected')
        self._frontier: tuple[float, float] | None = None
        self._frontier_frame = 'map'
        self.create_subscription(
            PointStamped,
            str(self.get_parameter('selected_frontier_topic').value),
            self._on_selected_frontier,
            10,
        )

    def _on_selected_frontier(self, msg: PointStamped) -> None:
        self._frontier = (float(msg.point.x), float(msg.point.y))
        self._frontier_frame = msg.header.frame_id or 'map'

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

    def _operator_thought(self, state: dict) -> str:
        integrity = state['integrity']
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

    def _obstruction_text(self, state: dict) -> str:
        health = state['health']
        stationary = state['stationary_sec']
        threshold = state['thresholds'].navigation_stall_warning_sec

        if health['critical_reasons']:
            return '🔴 ' + self._clip(health['critical_reasons'][0], 84)
        if (
            self._nav_active
            and stationary is not None
            and stationary >= threshold
        ):
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
