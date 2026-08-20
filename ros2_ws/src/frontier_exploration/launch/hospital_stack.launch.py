"""Launch Hospital simulation, then SLAM, then Nav2 with fixed delays."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    slam_dir = get_package_share_directory('slam_toolbox')
    default_rviz_config = os.path.join(
        package_dir, 'rviz', 'hospital_exploration.rviz'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_delay_sec = LaunchConfiguration('slam_delay_sec')
    nav2_delay_sec = LaunchConfiguration('nav2_delay_sec')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    keepout_enabled = LaunchConfiguration('keepout_enabled')
    keepout_resolution = LaunchConfiguration('keepout_resolution')
    keepout_safety_margin_m = LaunchConfiguration('keepout_safety_margin_m')
    keepout_outer_padding_m = LaunchConfiguration('keepout_outer_padding_m')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'hospital_simulation.launch.py')
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file,
            'headless': headless,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    map_visualizer = Node(
        package='frontier_exploration',
        executable='hospital_map_cloud_visualizer',
        name='hospital_map_cloud_visualizer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'hospital_nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_composition': 'false',
            'keepout_enabled': keepout_enabled,
            'keepout_resolution': keepout_resolution,
            'keepout_safety_margin_m': keepout_safety_margin_m,
            'keepout_outer_padding_m': keepout_outer_padding_m,
            'start_x': x_pose,
            'start_y': y_pose,
            'start_yaw': yaw,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='True'),
            DeclareLaunchArgument(
                'rviz_config_file',
                default_value=default_rviz_config,
                description='RViz config used by the Hospital stack.',
            ),
            DeclareLaunchArgument('headless', default_value='False'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'x_pose',
                default_value='0.0',
                description='Initial TurtleBot4 x in the Hospital SDF world.',
            ),
            DeclareLaunchArgument(
                'y_pose',
                default_value='12.0',
                description='Initial TurtleBot4 y in the Hospital SDF world.',
            ),
            DeclareLaunchArgument(
                'yaw',
                default_value='-1.57',
                description='Initial TurtleBot4 yaw in the Hospital SDF world.',
            ),
            DeclareLaunchArgument(
                'keepout_enabled',
                default_value='true',
                description=(
                    'Protect unsupported Hospital floor edges with a Nav2 '
                    'collision-mesh-derived keepout mask.'
                ),
            ),
            DeclareLaunchArgument(
                'keepout_resolution',
                default_value='0.05',
                description='Hospital keepout mask resolution in m/cell.',
            ),
            DeclareLaunchArgument(
                'keepout_safety_margin_m',
                default_value='0.40',
                description='Safety margin kept inside physical floor edges.',
            ),
            DeclareLaunchArgument(
                'keepout_outer_padding_m',
                default_value='2.0',
                description='Blocked padding outside Hospital floor bounds.',
            ),
            DeclareLaunchArgument(
                'slam_delay_sec',
                default_value='10.0',
                description='Seconds after simulation start before SLAM starts.',
            ),
            DeclareLaunchArgument(
                'nav2_delay_sec',
                default_value='20.0',
                description='Seconds after simulation start before Nav2 starts.',
            ),
            simulation,
            map_visualizer,
            TimerAction(period=slam_delay_sec, actions=[slam]),
            TimerAction(period=nav2_delay_sec, actions=[nav2]),
        ]
    )
