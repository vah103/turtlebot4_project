import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    config_path = os.path.join(
        package_dir,
        'config',
        'robot_status_hospital.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_name = LaunchConfiguration('run_name')
    clear_screen = LaunchConfiguration('clear_screen')

    monitor = Node(
        package='frontier_exploration',
        executable='robot_status_monitor',
        name='robot_status_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[
            config_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'run_name': run_name},
            {'clear_screen': ParameterValue(clear_screen, value_type=bool)},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use Gazebo simulation time for ROS timestamps.',
            ),
            DeclareLaunchArgument(
                'run_name',
                default_value='',
                description=(
                    'Recorder run name used to read run.json integrity state.'
                ),
            ),
            DeclareLaunchArgument(
                'clear_screen',
                default_value='true',
                description='Refresh the terminal as a live dashboard.',
            ),
            monitor,
        ]
    )
