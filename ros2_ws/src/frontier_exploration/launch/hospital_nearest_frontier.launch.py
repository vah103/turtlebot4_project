"""Launch Hospital simulation, SLAM, Nav2, then nearest-frontier controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    slam_dir = get_package_share_directory('slam_toolbox')

    default_rviz_config = os.path.join(
        package_dir, 'rviz', 'hospital_exploration.rviz'
    )
    default_slam_params = os.path.join(
        package_dir, 'config', 'hospital_slam.yaml'
    )

    # This repository layout is:
    # turtlebot4_project/ros2_ws/install/frontier_exploration/share/frontier_exploration
    # Go back to turtlebot4_project and run the Hospital-safe wrapper. The wrapper
    # reuses control_tb4.py frontier/selection logic but gets robot pose from TF.
    repo_root = os.path.abspath(
        os.path.join(package_dir, '..', '..', '..', '..', '..')
    )
    control_dir = os.path.join(repo_root, 'mapex_hospital_research')
    control_script = os.path.join(control_dir, 'control_tb4_hospital.py')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    slam_delay_sec = LaunchConfiguration('slam_delay_sec')
    nav2_delay_sec = LaunchConfiguration('nav2_delay_sec')
    frontier_delay_sec = LaunchConfiguration('frontier_delay_sec')
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
        condition=IfCondition(use_rviz),
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
            'slam_params_file': slam_params_file,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_dir, 'launch', 'hospital_nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_composition': 'false',
        }.items(),
    )

    nearest_frontier = ExecuteProcess(
        cmd=[
            '/usr/bin/python3',
            control_script,
            '--ros-args',
            '-p',
            'use_sim_time:=true',
        ],
        cwd=control_dir,
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='True'),
            DeclareLaunchArgument(
                'rviz_config_file',
                default_value=default_rviz_config,
                description='RViz config used by the Hospital exploration stack.',
            ),
            DeclareLaunchArgument('headless', default_value='False'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'slam_params_file',
                default_value=default_slam_params,
                description='slam_toolbox parameters tuned for Hospital mapping.',
            ),
            DeclareLaunchArgument('x_pose', default_value='0.0'),
            DeclareLaunchArgument('y_pose', default_value='12.0'),
            DeclareLaunchArgument('yaw', default_value='-1.57'),
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
            DeclareLaunchArgument(
                'frontier_delay_sec',
                default_value='30.0',
                description='Seconds after simulation start before control_tb4_hospital.py starts.',
            ),
            simulation,
            map_visualizer,
            TimerAction(period=slam_delay_sec, actions=[slam]),
            TimerAction(period=nav2_delay_sec, actions=[nav2]),
            TimerAction(period=frontier_delay_sec, actions=[nearest_frontier]),
        ]
    )
