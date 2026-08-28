#!/usr/bin/env python3
"""Launch Hospital simulation + SLAM + Nav2 + MapEx nearest-frontier node.

This research launch file includes ``hospital_simulation.launch.py`` and then
starts ``scripts/mapex_nearest_simple.py`` as a separate ROS 2 process.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    research_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    common_launch = os.path.join(
        research_dir,
        'launch',
        'hospital_simulation.launch.py',
    )
    nearest_script = os.path.join(
        research_dir,
        'scripts',
        'mapex_nearest_simple.py',
    )

    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    slam_delay_sec = LaunchConfiguration('slam_delay_sec')
    nav2_delay_sec = LaunchConfiguration('nav2_delay_sec')
    nearest_delay_sec = LaunchConfiguration('nearest_delay_sec')

    hospital_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(common_launch),
        launch_arguments={
            'use_rviz': use_rviz,
            'headless': headless,
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'slam_delay_sec': slam_delay_sec,
            'nav2_delay_sec': nav2_delay_sec,
        }.items(),
    )

    nearest_frontier = ExecuteProcess(
        cmd=[
            '/usr/bin/python3',
            nearest_script,
            '--ros-args',
            '-p',
            ['use_sim_time:=', use_sim_time],
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='True'),
            DeclareLaunchArgument('headless', default_value='False'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('x_pose', default_value='0.0'),
            DeclareLaunchArgument('y_pose', default_value='12.0'),
            DeclareLaunchArgument('yaw', default_value='-1.57'),
            DeclareLaunchArgument('slam_delay_sec', default_value='10.0'),
            DeclareLaunchArgument('nav2_delay_sec', default_value='20.0'),
            DeclareLaunchArgument(
                'nearest_delay_sec',
                default_value='25.0',
                description='Seconds after startup before nearest-frontier node starts.',
            ),
            hospital_stack,
            TimerAction(period=nearest_delay_sec, actions=[nearest_frontier]),
        ]
    )
