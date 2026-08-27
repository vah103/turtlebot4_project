"""Launch only Nav2 planner_server for replaying saved Hospital snapshots.

The replay client publishes /map and map->base_link for each saved snapshot.
No Gazebo, SLAM, map_server, lidar, controller_server, or behavior_server runs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
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

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_planner_replay',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['planner_server'],
        }],
    )

    return LaunchDescription([
        planner_server,
        lifecycle_manager,
    ])
