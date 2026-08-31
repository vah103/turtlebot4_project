"""Launch New Room simulation with stock TurtleBot4 Nav2 and near-stock SLAM.

This is the New Room counterpart of ``stock.launch.py``:
- simulation world: ``mapex_lab/map/new_room.sdf``;
- TurtleBot4 simulation adapter: ``tb4_simulation_safe.launch.py``;
- SLAM: ``mapex_lab/config/slam.yaml``;
- Nav2: installed TurtleBot4 stock config plus ``mapex_lab/config/nav2.yaml``;
- no segmented/submap scan frontend.

This launch is auxiliary cross-environment testing and is not part of the
formal ``hospital_v2`` protocol.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override values into a copy of base."""
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _cleanup_temp_file(_context, path: str):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
    return []


def generate_launch_description() -> LaunchDescription:
    frontier_pkg = get_package_share_directory('frontier_exploration')
    tb4_nav_pkg = get_package_share_directory('turtlebot4_navigation')
    research_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    slam_params = os.path.join(research_root, 'config', 'slam.yaml')

    stock_nav2_params = os.path.join(tb4_nav_pkg, 'config', 'nav2.yaml')
    xy_only_override = os.path.join(research_root, 'config', 'nav2.yaml')

    with open(stock_nav2_params, 'r', encoding='utf-8') as stream:
        nav2_base = yaml.safe_load(stream) or {}
    with open(xy_only_override, 'r', encoding='utf-8') as stream:
        nav2_override = yaml.safe_load(stream) or {}

    nav2_merged = _deep_merge(nav2_base, nav2_override)
    temporary = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='tb4_new_room_stock_xy_only_',
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    try:
        yaml.safe_dump(nav2_merged, temporary, sort_keys=False)
        nav2_params = temporary.name
    finally:
        temporary.close()

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    slam_delay_sec = LaunchConfiguration('slam_delay_sec')
    nav2_delay_sec = LaunchConfiguration('nav2_delay_sec')

    new_room_world = os.path.join(research_root, 'map', 'new_room.sdf')
    simulation_launch = os.path.join(
        frontier_pkg,
        'launch',
        'tb4_simulation_safe.launch.py',
    )
    robot_sdf = os.path.join(
        frontier_pkg,
        'urdf',
        'hospital_turtlebot4.urdf.xacro',
    )
    rviz_config = os.path.join(
        frontier_pkg,
        'rviz',
        'hospital_exploration.rviz',
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation_launch),
        launch_arguments={
            'world': new_room_world,
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config,
            'robot_sdf': robot_sdf,
            'headless': headless,
            'x_pose': '0.0',
            'y_pose': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_pkg, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params': slam_params,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_nav_pkg, 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items(),
    )

    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda context: _cleanup_temp_file(context, nav2_params)
                )
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('use_rviz', default_value='True'),
            DeclareLaunchArgument('headless', default_value='False'),
            DeclareLaunchArgument('slam_delay_sec', default_value='10.0'),
            DeclareLaunchArgument('nav2_delay_sec', default_value='20.0'),
            cleanup,
            simulation,
            TimerAction(period=slam_delay_sec, actions=[slam]),
            TimerAction(period=nav2_delay_sec, actions=[nav2]),
        ]
    )
