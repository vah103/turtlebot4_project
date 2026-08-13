import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        get_package_share_directory('frontier_exploration'),
        'config',
        'frontier.yaml',
    )
    enable_navigation = LaunchConfiguration('enable_navigation')

    manager = Node(
        package='frontier_exploration',
        executable='exploration_manager',
        name='exploration_manager',
        output='screen',
        parameters=[
            config_path,
            {
                'enable_navigation': ParameterValue(
                    enable_navigation, value_type=bool
                )
            },
        ],
    )

    detector = Node(
        package='frontier_exploration',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'enable_navigation',
                default_value='false',
                description='Explicitly allow NavigateToPose frontier goals.',
            ),
            manager,
            TimerAction(period=1.0, actions=[detector]),
        ]
    )
