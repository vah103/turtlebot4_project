"""Run the existing step debug mode with the minimum frontier-distance filter disabled."""

import rclpy

from control_tb4 import logger
from step import StepNavigationControl


def main(args=None):
    rclpy.init(args=args)
    node = StepNavigationControl()
    node.min_distance_threshold = 0.0
    logger.info("[STEP] Minimum frontier-distance filter DISABLED")
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
