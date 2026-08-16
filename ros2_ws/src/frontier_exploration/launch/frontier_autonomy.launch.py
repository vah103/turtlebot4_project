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
    use_sim_time = LaunchConfiguration('use_sim_time')

    common_time_parameter = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool)
    }

    manager = Node(
        package='frontier_exploration',
        executable='exploration_manager',
        name='exploration_manager',
        output='screen',
        parameters=[
            config_path,
            common_time_parameter,
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
            common_time_parameter,
            {
                'input_topic': '/cmd_vel_stamped',
                'output_topic': '/cmd_vel',
            },
        ],
    )

    detector = Node(
        package='frontier_exploration',
        executable='frontier_detector',
        name='frontier_detector',
        output='screen',
        parameters=[config_path, common_time_parameter],
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
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the /clock simulation time source.',
            ),
            manager,
            adapter,
            # Give Nav2/SLAM/TF a short head start. The detector then runs WFD
            # directly on /map; detector-only map preprocessing is not part of
            # the basic baseline anymore.
            TimerAction(period=1.0, actions=[detector]),
        ]
    )
