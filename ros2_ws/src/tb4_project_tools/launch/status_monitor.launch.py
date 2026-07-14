"""Launch the passive robot status monitor with configurable topic names."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration("namespace")
    battery_topic = LaunchConfiguration("battery_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    battery_reliability = LaunchConfiguration("battery_reliability")
    battery_durability = LaunchConfiguration("battery_durability")
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_config = PathJoinSubstitution(
        [FindPackageShare("tb4_project_tools"), "config", "status_monitor.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("battery_topic", default_value="battery_state"),
            DeclareLaunchArgument("scan_topic", default_value="scan"),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("battery_reliability", default_value="reliable"),
            DeclareLaunchArgument(
                "battery_durability", default_value="transient_local"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="tb4_project_tools",
                executable="robot_status_monitor",
                name="robot_status_monitor",
                namespace=namespace,
                output="screen",
                parameters=[
                    default_config,
                    {
                        "battery_topic": battery_topic,
                        "scan_topic": scan_topic,
                        "odom_topic": odom_topic,
                        "battery_reliability": battery_reliability,
                        "battery_durability": battery_durability,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
