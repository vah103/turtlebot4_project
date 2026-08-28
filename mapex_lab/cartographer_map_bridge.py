#!/usr/bin/env python3
"""Normalize Cartographer OccupancyGrid values for MapEx frontier semantics.

Cartographer publishes observed cells as probabilities in [0, 100], whereas the
MapEx/Nearest frontier code expects the discrete convention used by
slam_toolbox: unknown=-1, free=0, occupied=100.

This bridge subscribes to /cartographer_map and republishes /map using a 50%
occupancy threshold while preserving geometry, timestamps and unknown cells.
"""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


FREE_OCCUPANCY_THRESHOLD = 50


class CartographerMapBridge(Node):
    def __init__(self):
        super().__init__("cartographer_map_bridge")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(OccupancyGrid, "/map", qos)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            "/cartographer_map",
            self.map_callback,
            qos,
        )
        self.map_count = 0
        self.get_logger().info(
            "Cartographer map bridge started: /cartographer_map -> /map "
            "(-1 unknown, <50 free=0, >=50 occupied=100)"
        )

    def map_callback(self, msg: OccupancyGrid):
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.data = [
            -1 if value < 0 else (0 if value < FREE_OCCUPANCY_THRESHOLD else 100)
            for value in msg.data
        ]
        self.publisher.publish(out)

        self.map_count += 1
        if self.map_count == 1:
            unknown = sum(value < 0 for value in msg.data)
            free = sum(0 <= value < FREE_OCCUPANCY_THRESHOLD for value in msg.data)
            occupied = len(msg.data) - unknown - free
            self.get_logger().info(
                "First normalized map: "
                f"{msg.info.width}x{msg.info.height}, "
                f"resolution={msg.info.resolution:.3f}, "
                f"unknown={unknown}, free={free}, occupied={occupied}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CartographerMapBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
