"""One-screen operator view for the read-only TurtleBot4 status monitor."""

import math

import rclpy

from frontier_exploration.robot_status_core import (
    STATUS_NOT_OK,
    STATUS_OK,
    STATUS_WARNING,
)
from frontier_exploration.robot_status_monitor import RobotStatusMonitor


class CompactRobotStatusMonitor(RobotStatusMonitor):
    """Render the same diagnostics in a compact terminal-friendly layout."""

    @staticmethod
    def _clip(value: str, limit: int = 104) -> str:
        text = str(value).replace('\n', ' ').strip()
        if len(text) <= limit:
            return text
        return text[: max(1, limit - 3)] + '...'

    @staticmethod
    def _tf_mark(item: dict) -> str:
        return '✅' if item.get('ok') else '🔴'

    def _dashboard(self, state: dict) -> str:
        health = state['health']
        th = state['thresholds']
        ages = state['ages_sec']
        rates = state['rates_hz']
        odom = state['odom']
        map_pose = state['map_pose']
        map_stats = state['map']
        integrity = state['integrity']
        tf_state = state['tf']

        nav_state = 'COMPLETE' if self._exploration_complete else (
            'NAVIGATING' if self._nav_active else 'IDLE'
        )
        health_mark = {
            STATUS_OK: '✅ OK',
            STATUS_WARNING: '⚠️ WARNING',
            STATUS_NOT_OK: '🔴 NOT OK',
        }[health['status']]

        lines = [
            '=' * 70,
            ' TURTLEBOT4 LIVE STATUS  |  ONE-SCREEN VIEW',
            '=' * 70,
            f"OVERALL: {health_mark}  |  {health['headline']}",
            f"KẾT LUẬN: {self._clip(health['summary'])}",
        ]

        alerts = [
            ('CRITICAL', text)
            for text in health['critical_reasons']
        ] + [
            ('WARNING', text)
            for text in health['warnings']
        ]
        for level, text in alerts[:3]:
            lines.append(f'! {level}: {self._clip(text, 96)}')
        if len(alerts) > 3:
            lines.append(f'! +{len(alerts) - 3} cảnh báo khác trong JSONL.')

        lines.append('-' * 70)

        if map_pose is None:
            pose_map = 'map=N/A'
        else:
            pose_map = (
                f'map=({map_pose[0]:.2f}, {map_pose[1]:.2f}, '
                f'{math.degrees(map_pose[2]):.1f}°)'
            )

        if odom:
            pose_odom = (
                f"odom=({odom['x']:.2f}, {odom['y']:.2f}, "
                f"{math.degrees(odom['yaw_rad']):.1f}°)"
            )
            velocity = (
                f"v={odom['linear_mps']:.3f} m/s  "
                f"w={odom['angular_rps']:.3f} rad/s"
            )
        else:
            pose_odom = 'odom=N/A'
            velocity = 'v=N/A  w=N/A'

        stationary = state['stationary_sec']
        motion = (
            'MOVING'
            if stationary is None
            else f'STATIONARY {stationary:.1f}s'
        )
        lines.append(f'ROBOT : {pose_map}  |  {motion}')
        lines.append(f'ODOM  : {pose_odom}  |  {velocity}')

        goal_text = 'N/A'
        if self._goal is not None:
            goal_text = f'({self._goal[0]:.2f}, {self._goal[1]:.2f})'
        lines.append(
            f'NAV   : {nav_state} | Nav2={self._nav_status} | '
            f'last={self._last_nav_result}'
        )
        lines.append(
            f'GOAL  : {goal_text} | dist={self._fmt(state["goal_distance_m"])} m | '
            f'path={self._fmt(self._path_length_m)} m'
        )
        lines.append(
            f'RESULT: success={self._success_count} | failed={self._failure_count} | '
            f'consecutive_failed={self._consecutive_failures}'
        )

        if map_stats:
            total = map_stats['known_cells'] + map_stats['unknown_cells']
            map_line = (
                f"{map_stats['width']}x{map_stats['height']} @ "
                f"{map_stats['resolution_m']:.3f}m | known="
                f"{map_stats['known_cells']:,}/{total:,} "
                f"({map_stats['known_fraction'] * 100.0:.2f}%)"
            )
        else:
            map_line = 'N/A'
        run_state = 'UNKNOWN'
        if integrity['available']:
            run_state = 'INVALID' if integrity['run_invalid'] else 'VALID'
        lines.append(f'MAP   : {map_line} | run={run_state}')

        map_mark = self._stream_marker(
            ages['map'], th.map_warning_sec, th.map_critical_sec
        )
        scan_mark = self._stream_marker(
            ages['scan'], th.scan_warning_sec, th.scan_critical_sec
        )
        odom_mark = self._stream_marker(
            ages['odom'], th.odom_warning_sec, th.odom_critical_sec
        )
        lines.append(
            f'/map : {map_mark} {self._rate(rates["map"])} '
            f'age={self._age_text(ages["map"])}'
        )
        lines.append(
            f'/scan: {scan_mark} {self._rate(rates["scan"])} '
            f'age={self._age_text(ages["scan"])} | '
            f'/odom: {odom_mark} {self._rate(rates["odom"])} '
            f'age={self._age_text(ages["odom"])}'
        )

        tf_parts = []
        for label, item in tf_state.items():
            tf_parts.append(f'{label}={self._tf_mark(item)}')
        lines.append('TF    : ' + ' | '.join(tf_parts))

        map_health = (
            '🔴 INVALID'
            if integrity['run_invalid']
            else ('✅ NORMAL' if integrity['available'] else '❔ UNKNOWN')
        )
        nav_health = '⚠️ CHECK' if state['nav_warning'] else '✅ NORMAL'
        lines.append(
            f'HEALTH: overall={health_mark} | map/run={map_health} | '
            f'navigation={nav_health}'
        )
        lines.append(f'JSONL : data/robot_status/{self._jsonl_path.name}')
        lines.append('=' * 70)
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
