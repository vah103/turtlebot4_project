"""Launch flat Hospital simulation, then SLAM, then Nav2 with fixed delays."""

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

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'hospital_flat_simulation.launch.py')
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

    # The flat world has no negative obstacles or unsupported floor edges, so
    # explicitly disable the Hospital keepout filter and run normal Nav2.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'hospital_nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_composition': 'false',
            'keepout_enabled': 'false',
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
                description='RViz config used by the flat Hospital stack.',
            ),
            DeclareLaunchArgument('headless', default_value='False'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'x_pose',
                default_value='0.0',
                description='Initial TurtleBot4 x in the flat Hospital world.',
            ),
            DeclareLaunchArgument(
                'y_pose',
                default_value='12.0',
                description='Initial TurtleBot4 y in the flat Hospital world.',
            ),
            DeclareLaunchArgument(
                'yaw',
                default_value='-1.57',
                description='Initial TurtleBot4 yaw in the flat Hospital world.',
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
