"""Exploration manager with repeated-failure trapped-robot protection."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from frontier_exploration.exploration_manager import ExplorationManager
from frontier_exploration.navigation_safety import TrappedFailureTracker


class ResilientExplorationManager(ExplorationManager):
    """Stop cycling frontier goals after repeated failures at the same pose."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('trapped_topic', '/robot_trapped')
        self.declare_parameter('trapped_failure_limit', 3)
        self.declare_parameter('trapped_radius_m', 0.20)

        trapped_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._trapped_pub = self.create_publisher(
            Bool,
            str(self.get_parameter('trapped_topic').value),
            trapped_qos,
        )
        self._trap_tracker = TrappedFailureTracker(
            radius_m=float(self.get_parameter('trapped_radius_m').value),
            failure_limit=int(self.get_parameter('trapped_failure_limit').value),
        )
        self._robot_trapped = False
        self._trapped_path_notice_shown = False
        self._publish_trapped(False)
        self.get_logger().info(
            'Trapped-robot guard: stop accepting new frontier paths after '
            f'{self._trap_tracker.failure_limit} execution failures within '
            f'{self._trap_tracker.radius_m:.2f} m'
        )

    @staticmethod
    def _counts_toward_trapped(detail: str) -> bool:
        text = str(detail)
        return (
            text.startswith('no meaningful progress')
            or text.startswith('navigation timeout')
            or text.startswith('status=')
        )

    def _publish_trapped(self, trapped: bool) -> None:
        msg = Bool()
        msg.data = bool(trapped)
        self._trapped_pub.publish(msg)

    def _set_trapped(self, trapped: bool) -> None:
        trapped = bool(trapped)
        changed = trapped != self._robot_trapped
        self._robot_trapped = trapped
        if changed or trapped:
            self._publish_trapped(trapped)
        if not trapped:
            self._trapped_path_notice_shown = False

    def _on_path(self, path: Path) -> None:
        if self._robot_trapped:
            if not self._trapped_path_notice_shown:
                self.get_logger().error(
                    'ROBOT_TRAPPED: ignoring new frontier paths to prevent an '
                    'endless fail/cooldown/retry loop. Restart the exploration '
                    'manager after the robot pose/clearance issue is resolved.'
                )
                self._trapped_path_notice_shown = True
            return
        super()._on_path(path)

    def _finish_navigation(self, succeeded: bool, detail: str) -> None:
        if succeeded:
            self._trap_tracker.reset()
            self._set_trapped(False)
        elif self._counts_toward_trapped(detail):
            pose = self._latest_odom_pose
            if pose is not None:
                count, trapped = self._trap_tracker.record_failure(
                    pose[0], pose[1]
                )
                anchor = self._trap_tracker.anchor_xy
                self.get_logger().warning(
                    'Repeated-failure trap check: '
                    f'{count}/{self._trap_tracker.failure_limit} failures near '
                    f'odom=({pose[0]:.2f}, {pose[1]:.2f}); '
                    f'anchor=({anchor[0]:.2f}, {anchor[1]:.2f})'
                )
                if trapped:
                    self._set_trapped(True)
                    self.get_logger().error(
                        'ROBOT_TRAPPED: repeated navigation failures occurred '
                        f'within {self._trap_tracker.radius_m:.2f} m of the same '
                        'physical odometry pose. New frontier goals are blocked.'
                    )

        super()._finish_navigation(succeeded, detail)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResilientExplorationManager()
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
