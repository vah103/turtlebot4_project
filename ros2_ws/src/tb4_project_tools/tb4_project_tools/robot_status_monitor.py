"""A passive ROS 2 monitor for selected robot status topics."""

from dataclasses import dataclass
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import BatteryState, LaserScan


@dataclass
class TopicStatus:
    """Tracks message count and the most recent receipt timestamp."""

    label: str
    topic: str
    last_received_ns: Optional[int] = None
    message_count: int = 0

    def display(self, now_ns: int) -> str:
        if self.last_received_ns is None:
            return "0 message; đang chờ dữ liệu"
        age_sec = max(0, now_ns - self.last_received_ns) / 1_000_000_000
        return f"{self.message_count} message; message gần nhất cách đây {age_sec:.3f} s"


def make_battery_qos(reliability: str, durability: str) -> QoSProfile:
    """Build the explicitly configured BatteryState QoS profile."""
    reliability_values = {
        "reliable": ReliabilityPolicy.RELIABLE,
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
    }
    durability_values = {
        "transient_local": DurabilityPolicy.TRANSIENT_LOCAL,
        "volatile": DurabilityPolicy.VOLATILE,
    }
    try:
        reliability_policy = reliability_values[reliability.lower()]
    except (AttributeError, KeyError) as error:
        raise ValueError(
            "battery_reliability phải là reliable hoặc best_effort"
        ) from error
    try:
        durability_policy = durability_values[durability.lower()]
    except (AttributeError, KeyError) as error:
        raise ValueError(
            "battery_durability phải là transient_local hoặc volatile"
        ) from error
    return QoSProfile(
        depth=10,
        reliability=reliability_policy,
        durability=durability_policy,
    )


class RobotStatusMonitor(Node):
    """Subscribes to robot status data without publishing or controlling it."""

    def __init__(self) -> None:
        super().__init__("robot_status_monitor")

        self.declare_parameter("battery_topic", "battery_state")
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("odom_topic", "odom")
        self.declare_parameter("status_interval_sec", 5.0)
        self.declare_parameter("battery_reliability", "reliable")
        self.declare_parameter("battery_durability", "transient_local")

        battery_topic = self.get_parameter("battery_topic").value
        scan_topic = self.get_parameter("scan_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        status_interval = float(self.get_parameter("status_interval_sec").value)
        battery_qos = make_battery_qos(
            self.get_parameter("battery_reliability").value,
            self.get_parameter("battery_durability").value,
        )

        if status_interval <= 0.0:
            raise ValueError("status_interval_sec phải lớn hơn 0")

        self._statuses = {
            "battery": TopicStatus("BatteryState", battery_topic),
            "scan": TopicStatus("LaserScan", scan_topic),
            "odom": TopicStatus("Odometry", odom_topic),
        }

        # Keep subscription objects alive for the lifetime of the node.
        self._subscriptions = [
            self.create_subscription(
                BatteryState,
                battery_topic,
                lambda message: self._record_message("battery", message),
                battery_qos,
            ),
            self.create_subscription(
                LaserScan,
                scan_topic,
                lambda message: self._record_message("scan", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                Odometry,
                odom_topic,
                lambda message: self._record_message("odom", message),
                qos_profile_sensor_data,
            ),
        ]
        self._status_timer = self.create_timer(status_interval, self._log_status)

        self.get_logger().info("Robot status monitor đã khởi động ở chế độ chỉ subscribe.")
        for status in self._statuses.values():
            self.get_logger().info(f"Đang chờ {status.label} trên topic '{status.topic}'.")

    def _record_message(self, key: str, _message: object) -> None:
        status = self._statuses[key]
        status.last_received_ns = self.get_clock().now().nanoseconds
        status.message_count += 1
        if status.message_count == 1:
            self.get_logger().info(f"Đã nhận {status.label} lần đầu.")

    def _log_status(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        for status in self._statuses.values():
            self.get_logger().info(
                f"{status.label} [{status.topic}]: {status.display(now_ns)}."
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
