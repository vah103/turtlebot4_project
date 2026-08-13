from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        get_package_share_directory('frontier_exploration'),
        'config',
        'frontier.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='frontier_exploration',
                executable='frontier_detector',
                name='frontier_detector',
                output='screen',
                parameters=[config_path],
            )
        ]
    )
