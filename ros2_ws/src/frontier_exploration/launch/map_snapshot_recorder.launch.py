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
        'map_snapshot.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_dir = LaunchConfiguration('output_dir')
    run_name = LaunchConfiguration('run_name')
    fixed_canvas_configured = LaunchConfiguration('fixed_canvas_configured')
    canvas_width_cells = LaunchConfiguration('canvas_width_cells')
    canvas_height_cells = LaunchConfiguration('canvas_height_cells')
    canvas_resolution = LaunchConfiguration('canvas_resolution')
    canvas_origin_x = LaunchConfiguration('canvas_origin_x')
    canvas_origin_y = LaunchConfiguration('canvas_origin_y')

    recorder = Node(
        package='frontier_exploration',
        executable='map_snapshot_recorder',
        name='map_snapshot_recorder',
        output='screen',
        parameters=[
            config_path,
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
            {'output_dir': output_dir, 'run_name': run_name},
            {
                'fixed_canvas_configured': ParameterValue(
                    fixed_canvas_configured,
                    value_type=bool,
                ),
                'canvas_width_cells': ParameterValue(
                    canvas_width_cells,
                    value_type=int,
                ),
                'canvas_height_cells': ParameterValue(
                    canvas_height_cells,
                    value_type=int,
                ),
                'canvas_resolution': ParameterValue(
                    canvas_resolution,
                    value_type=float,
                ),
                'canvas_origin_x': ParameterValue(
                    canvas_origin_x,
                    value_type=float,
                ),
                'canvas_origin_y': ParameterValue(
                    canvas_origin_y,
                    value_type=float,
                ),
            },
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
                default_value='',
                description='Optional run folder name.',
            ),
            DeclareLaunchArgument(
                'fixed_canvas_configured',
                default_value='false',
                description='Confirm fixed canvas geometry was chosen before run.',
            ),
            DeclareLaunchArgument('canvas_width_cells', default_value='0'),
            DeclareLaunchArgument('canvas_height_cells', default_value='0'),
            DeclareLaunchArgument('canvas_resolution', default_value='0.0'),
            DeclareLaunchArgument('canvas_origin_x', default_value='0.0'),
            DeclareLaunchArgument('canvas_origin_y', default_value='0.0'),
            recorder,
        ]
    )
