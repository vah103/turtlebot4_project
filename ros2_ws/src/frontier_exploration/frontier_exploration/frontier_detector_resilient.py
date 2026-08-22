"""TF-resilient entry point for the frontier detector."""

from math import atan2, cos, hypot, sin

import rclpy

from frontier_exploration.frontier_detector import _cell_to_world
from frontier_exploration.frontier_detector_strict_completion import (
    StrictCompletionFrontierDetector,
)
from frontier_exploration.tf_pose import lookup_robot_xy


class _FrontierFacingPlannerClient:
    """Orient planner goals from the safe goal toward the selected frontier."""

    def __init__(self, detector, client) -> None:
        self._detector = detector
        self._client = client

    def server_is_ready(self) -> bool:
        return self._client.server_is_ready()

    def send_goal_async(self, goal_msg):
        candidate = self._detector._planning_candidate
        map_msg = self._detector._latest_map
        if candidate is not None and map_msg is not None:
            _, representative, goal_index, _, _ = candidate
            frontier = _cell_to_world(representative, map_msg)
            goal = _cell_to_world(goal_index, map_msg)
            dx = frontier.x - goal.x
            dy = frontier.y - goal.y
            if hypot(dx, dy) > 1e-6:
                yaw = atan2(dy, dx)
                orientation = goal_msg.goal.pose.orientation
                orientation.x = 0.0
                orientation.y = 0.0
                orientation.z = sin(yaw / 2.0)
                orientation.w = cos(yaw / 2.0)
        return self._client.send_goal_async(goal_msg)


class ResilientFrontierDetector(StrictCompletionFrontierDetector):
    """Use strict completion plus split-chain TF fallback when needed."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('odom_frame', 'odom')
        self._split_tf_notice_shown = False
        self._planner_client = _FrontierFacingPlannerClient(
            self,
            self._planner_client,
        )
        self.get_logger().info(
            'Frontier planner goals face from the safe navigation goal toward '
            'the selected frontier'
        )

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
