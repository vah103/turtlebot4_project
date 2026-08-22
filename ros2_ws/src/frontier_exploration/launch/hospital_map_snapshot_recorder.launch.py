import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        get_package_share_directory('frontier_exploration'),
        'config',
        'map_snapshot_hospital.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_dir = LaunchConfiguration('output_dir')
    run_name = LaunchConfiguration('run_name')

    recorder = Node(
        package='frontier_exploration',
        executable='hospital_map_snapshot_recorder',
        name='map_snapshot_recorder',
        output='screen',
        parameters=[
            config_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'output_dir': output_dir, 'run_name': run_name},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo /clock source.',
            ),
            DeclareLaunchArgument(
                'output_dir',
                default_value='data/lama_runs',
                description=(
                    'Parent directory for recorded runs. Relative paths are '
                    'resolved from the TurtleBot4 project root.'
                ),
            ),
            DeclareLaunchArgument(
                'run_name',
                default_value='hospital_lama_fixed_canvas',
                description='Snapshot run folder name.',
            ),
            recorder,
        ]
    )
