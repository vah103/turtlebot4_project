"""Run only Nav2 map_server + planner_server for Hospital reachability tests.

No Gazebo, SLAM, lidar, controller_server, or behavior_server is launched.
A fixed map->base_link transform keeps the global costmap valid while test
clients submit an explicit start pose through ComputePathToPose(use_start=True).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    nav2_dir = get_package_share_directory('nav2_bringup')

    base_params = os.path.join(nav2_dir, 'params', 'nav2_params.yaml')
    hospital_override = os.path.join(
        package_dir, 'config', 'nav2_hospital_override.yaml'
    )
    planner_only_override = os.path.join(
        package_dir, 'config', 'hospital_planner_only_override.yaml'
    )

    map_yaml = LaunchConfiguration('map')
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            base_params,
            {'use_sim_time': False, 'yaml_filename': map_yaml},
        ],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            base_params,
            hospital_override,
            planner_only_override,
            {'use_sim_time': False},
        ],
    )

    # Global costmap asks for the robot pose even though ComputePathToPose will
    # receive an explicit start. Keep that pose fixed during this offline test.
    static_robot_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='hospital_planner_test_tf',
        output='screen',
        arguments=[
            '--x', start_x,
            '--y', start_y,
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', start_yaw,
            '--frame-id', 'map',
            '--child-frame-id', 'base_link',
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner_test',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'planner_server'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Absolute path to a saved Hospital Nav2 map YAML.',
        ),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='12.0'),
        DeclareLaunchArgument('start_yaw', default_value='-1.57'),
        map_server,
        static_robot_tf,
        planner_server,
        lifecycle_manager,
    ])
