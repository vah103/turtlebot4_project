from pathlib import Path

import pytest
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy

from tb4_project_tools.robot_status_monitor import TopicStatus, make_battery_qos


def test_topic_status_waiting_without_message():
    status = TopicStatus("BatteryState", "battery_state")
    assert status.display(2_000_000_000) == "0 message; đang chờ dữ liệu"


def test_topic_status_displays_count_and_age():
    status = TopicStatus(
        "LaserScan", "scan", last_received_ns=1_500_000_000, message_count=4
    )
    assert status.display(2_000_000_000) == (
        "4 message; message gần nhất cách đây 0.500 s"
    )


def test_battery_qos_defaults():
    qos = make_battery_qos("reliable", "transient_local")
    assert qos.reliability == ReliabilityPolicy.RELIABLE
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL


def test_battery_qos_can_be_configured():
    qos = make_battery_qos("best_effort", "volatile")
    assert qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == DurabilityPolicy.VOLATILE


@pytest.mark.parametrize(
    ("reliability", "durability"),
    [("unknown", "volatile"), ("reliable", "unknown")],
)
def test_battery_qos_rejects_unknown_values(reliability, durability):
    with pytest.raises(ValueError):
        make_battery_qos(reliability, durability)


def test_monitor_source_does_not_create_publishers():
    source_path = (
        Path(__file__).parents[1]
        / "tb4_project_tools"
        / "robot_status_monitor.py"
    )
    source = source_path.read_text(encoding="utf-8")
    forbidden_fragments = (
        "create_publisher",
        "create_service",
        "create_client",
        "create_action",
        "cmd_vel",
        "Twist",
    )
    assert all(fragment not in source for fragment in forbidden_fragments)
