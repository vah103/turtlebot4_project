"""Simulation-only TwistStamped -> Twist adapter for TurtleBot4 Gazebo.

This node is disabled unless explicitly launched. It bridges Nav2's stamped
/cmd_vel output to the unstamped Twist type expected by the Gazebo bridge.
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class SimTwistAdapter(Node):
    """Convert stamped velocity commands to unstamped Twist in simulation."""

    def __init__(self) -> None:
        super().__init__('sim_twist_adapter')
        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(TwistStamped, input_topic, self._on_twist, 10)

        self.get_logger().warning(
            'SIMULATION TWIST ADAPTER ACTIVE: '
            f'{input_topic} TwistStamped -> {output_topic} Twist'
        )

    def _on_twist(self, msg: TwistStamped) -> None:
        twist = Twist()
        twist.linear.x = msg.twist.linear.x
        twist.linear.y = msg.twist.linear.y
        twist.linear.z = msg.twist.linear.z
        twist.angular.x = msg.twist.angular.x
        twist.angular.y = msg.twist.angular.y
        twist.angular.z = msg.twist.angular.z
        self._publisher.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimTwistAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
