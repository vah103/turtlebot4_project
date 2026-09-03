"""Launch the self-contained mapex_lab new_room world with TurtleBot4.

This is a lightweight geometry / spawn preview launch. It intentionally starts
only the simulation stack (Gazebo Sim, TurtleBot4, optional RViz) and does not
start SLAM, Nav2, or exploration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    frontier_pkg = get_package_share_directory('frontier_exploration')
    research_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    world = os.path.join(research_root, 'map', 'new_room.sdf')
    simulation_launch = os.path.join(
        frontier_pkg, 'launch', 'tb4_simulation_safe.launch.py'
    )
    robot_sdf = os.path.join(
        frontier_pkg, 'urdf', 'hospital_turtlebot4.urdf.xacro'
    )
    rviz_config = os.path.join(
        frontier_pkg, 'rviz', 'hospital_exploration.rviz'
    )

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            'world': world,
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config,
            'robot_sdf': robot_sdf,
            'headless': headless,
            'x_pose': '0.0',
            'y_pose': '3.0',
            'yaw': '0.0',
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='False'),
            DeclareLaunchArgument('headless', default_value='False'),
            simulation,
        ]
    )
