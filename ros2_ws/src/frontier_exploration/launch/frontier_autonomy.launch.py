import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
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
    enable_sim_twist_adapter = LaunchConfiguration('enable_sim_twist_adapter')

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

    adapter = Node(
        package='frontier_exploration',
        executable='sim_twist_adapter',
        name='sim_twist_adapter',
        output='screen',
        condition=IfCondition(enable_sim_twist_adapter),
        parameters=[
            {
                'input_topic': '/cmd_vel_stamped',
                'output_topic': '/cmd_vel',
            }
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
            DeclareLaunchArgument(
                'enable_sim_twist_adapter',
                default_value='false',
                description=(
                    'Simulation only: convert /cmd_vel_stamped TwistStamped '
                    'to /cmd_vel Twist for the Gazebo bridge.'
                ),
            ),
            manager,
            adapter,
            TimerAction(period=1.0, actions=[detector]),
        ]
    )
