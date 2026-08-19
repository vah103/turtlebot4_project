"""TF-resilient entry point for the frontier detector."""

import rclpy

from frontier_exploration.frontier_detector_strict_completion import (
    StrictCompletionFrontierDetector,
)
from frontier_exploration.tf_pose import lookup_robot_xy


class ResilientFrontierDetector(StrictCompletionFrontierDetector):
    """Use strict completion plus split-chain TF fallback when needed."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('odom_frame', 'odom')
        self._split_tf_notice_shown = False

    def _robot_position(self, map_frame: str) -> tuple[float, float] | None:
        robot_frame = str(self.get_parameter('robot_frame').value)
        odom_frame = str(self.get_parameter('odom_frame').value)
        position, used_split_chain, error = lookup_robot_xy(
            self._tf_buffer,
            map_frame,
            robot_frame,
            odom_frame,
        )

        if position is None:
            if not self._tf_warning_shown:
                self.get_logger().warning(
                    f'Waiting for TF {map_frame} -> {robot_frame}: {error}'
                )
                self._tf_warning_shown = True
            return None

        self._tf_warning_shown = False
        if used_split_chain:
            if not self._split_tf_notice_shown:
                self.get_logger().warning(
                    'Using split-chain TF fallback: '
                    f'{map_frame}->{odom_frame} + {odom_frame}->{robot_frame}'
                )
                self._split_tf_notice_shown = True
        else:
            self._split_tf_notice_shown = False

        return position


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ResilientFrontierDetector()
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
