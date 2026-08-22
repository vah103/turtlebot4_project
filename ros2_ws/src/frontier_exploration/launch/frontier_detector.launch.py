import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        get_package_share_directory('frontier_exploration'),
        'config',
        'frontier.yaml',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    common_time_parameter = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool)
    }

    detector = Node(
        package='frontier_exploration',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[config_path, common_time_parameter],
    )
    shutdown_when_detector_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=detector,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason='Frontier detector finished or stopped'
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the /clock simulation time source.',
            ),
            shutdown_when_detector_exits,
            TimerAction(period=0.5, actions=[detector]),
        ]
    )
