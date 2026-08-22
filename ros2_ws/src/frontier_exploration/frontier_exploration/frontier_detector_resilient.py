"""TF-resilient entry point for the frontier detector."""

from math import atan2, cos, hypot, sin

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from frontier_exploration.frontier_detector import _cell_to_world, _world_to_cell
from frontier_exploration.frontier_detector_strict_completion import (
    StrictCompletionFrontierDetector,
)
from frontier_exploration.navigation_safety import circle_is_clear
from frontier_exploration.tf_pose import lookup_robot_xy


class _ApproachFacingPlannerClient:
    """Keep the terminal goal yaw aligned with the approach to the safe goal."""

    def __init__(self, detector, client) -> None:
        self._detector = detector
        self._client = client

    def server_is_ready(self) -> bool:
        return self._client.server_is_ready()

    def send_goal_async(self, goal_msg):
        candidate = self._detector._planning_candidate
        map_msg = self._detector._latest_map
        if candidate is not None and map_msg is not None:
            _, _representative, goal_index, _, _ = candidate
            goal = _cell_to_world(goal_index, map_msg)
            map_frame = map_msg.header.frame_id or 'map'
            robot = self._detector._robot_position(map_frame)
            if robot is not None:
                dx = goal.x - robot[0]
                dy = goal.y - robot[1]
                if hypot(dx, dy) > 1e-6:
                    yaw = atan2(dy, dx)
                    orientation = goal_msg.goal.pose.orientation
                    orientation.x = 0.0
                    orientation.y = 0.0
                    orientation.z = sin(yaw / 2.0)
                    orientation.w = cos(yaw / 2.0)
        return self._client.send_goal_async(goal_msg)


class ResilientFrontierDetector(StrictCompletionFrontierDetector):
    """Use strict completion, clearance checks, and split-chain TF fallback."""

    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('selected_frontier_topic', '/frontier_selected')
        self.declare_parameter('frontier_goal_clearance_m', 0.35)
        self._split_tf_notice_shown = False
        selected_frontier_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._selected_frontier_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter('selected_frontier_topic').value),
            selected_frontier_qos,
        )
        self._planner_client = _ApproachFacingPlannerClient(
            self,
            self._planner_client,
        )
        self.get_logger().info(
            'Frontier planner terminal yaw follows the approach direction to '
            'the safe goal; it no longer forces the robot to face the frontier'
        )
        self.get_logger().info(
            'Frontier goal turning-clearance radius: '
            f'{float(self.get_parameter("frontier_goal_clearance_m").value):.2f} m'
        )

    def _point_blocked(
        self,
        costmap: OccupancyGrid,
        x: float,
        y: float,
        source_frame: str,
        *,
        outside_is_blocked: bool,
    ) -> bool:
        if super()._point_blocked(
            costmap,
            x,
            y,
            source_frame,
            outside_is_blocked=outside_is_blocked,
        ):
            return True

        clearance = max(
            0.0, float(self.get_parameter('frontier_goal_clearance_m').value)
        )
        if clearance <= 0.0:
            return False

        target_frame = costmap.header.frame_id or source_frame
        transformed = self._transform_xy(x, y, source_frame, target_frame)
        if transformed is None:
            return outside_is_blocked
        center_index = _world_to_cell(costmap, transformed[0], transformed[1])
        if center_index is None:
            return outside_is_blocked

        clear = circle_is_clear(
            costmap.data,
            int(costmap.info.width),
            int(costmap.info.height),
            center_index,
            float(costmap.info.resolution),
            clearance,
            int(self.get_parameter('costmap_occ_threshold').value),
            unknown_is_blocked=bool(
                self.get_parameter('costmap_unknown_is_blocked').value
            ),
            outside_is_blocked=outside_is_blocked,
        )
        return not clear

    def _publish_selected_frontier(self) -> None:
        if self._selected_index is None or self._latest_map is None:
            return

        point = _cell_to_world(self._selected_index, self._latest_map)
        msg = PointStamped()
        msg.header = self._latest_map.header
        msg.point.x = float(point.x)
        msg.point.y = float(point.y)
        msg.point.z = 0.0
        self._selected_frontier_pub.publish(msg)

    def _on_plan_result(self, future, candidate) -> None:
        super()._on_plan_result(future, candidate)
        self._publish_selected_frontier()

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
