"""Launch Hospital + rolling local scan frontend + SLAM Toolbox + stock Nav2.

This is a debug A/B alternative to stock.launch.py. The SLAM Toolbox backend
remains unchanged; only its LaserScan input is replaced by /scan_local_window,
produced by local_scan_window.py from recent scan-to-local-window registration.

The experiment is intentionally not part of the official hospital_v2 protocol.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _deep_merge(base: dict, override: dict) -> dict:
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


def _frontend_process(context, script_path: str):
    def value(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    return [
        ExecuteProcess(
            cmd=[
                '/usr/bin/python3',
                script_path,
                '--ros-args',
                '-p',
                f'use_sim_time:={value("use_sim_time")}',
                '-p',
                'input_scan_topic:=/scan',
                '-p',
                'output_scan_topic:=/scan_local_window',
                '-p',
                'odom_frame:=odom',
                '-p',
                f'window_scans:={value("window_scans")}',
                '-p',
                f'scan_stride:={value("scan_stride")}',
                '-p',
                f'voxel_size_m:={value("voxel_size_m")}',
                '-p',
                f'max_correspondence_distance_m:={value("max_correspondence_distance_m")}',
                '-p',
                f'max_rmse_m:={value("max_rmse_m")}',
                '-p',
                f'max_translation_correction_m:={value("max_translation_correction_m")}',
                '-p',
                f'max_rotation_correction_rad:={value("max_rotation_correction_rad")}',
            ],
            output='screen',
        )
    ]


def generate_launch_description() -> LaunchDescription:
    frontier_pkg = get_package_share_directory('frontier_exploration')
    tb4_nav_pkg = get_package_share_directory('turtlebot4_navigation')
    research_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    slam_params = os.path.join(research_root, 'config', 'slam_local_window.yaml')
    frontend_script = os.path.join(research_root, 'local_scan_window.py')

    # Same stock Nav2 + project override path as stock.launch.py.
    stock_nav2_params = os.path.join(tb4_nav_pkg, 'config', 'nav2.yaml')
    xy_only_override = os.path.join(
        research_root,
        'config',
        'nav2_stock_xy_only.yaml',
    )

    with open(stock_nav2_params, 'r', encoding='utf-8') as stream:
        nav2_base = yaml.safe_load(stream) or {}
    with open(xy_only_override, 'r', encoding='utf-8') as stream:
        nav2_override = yaml.safe_load(stream) or {}

    nav2_merged = _deep_merge(nav2_base, nav2_override)
    temporary = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='tb4_local_window_stock_xy_only_',
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
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    slam_delay_sec = LaunchConfiguration('slam_delay_sec')
    nav2_delay_sec = LaunchConfiguration('nav2_delay_sec')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                frontier_pkg,
                'launch',
                'hospital_flat_simulation.launch.py',
            )
        ),
        launch_arguments={
            'use_rviz': use_rviz,
            'headless': headless,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
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
            DeclareLaunchArgument('x_pose', default_value='0.0'),
            DeclareLaunchArgument('y_pose', default_value='12.0'),
            DeclareLaunchArgument('yaw', default_value='-1.57'),
            DeclareLaunchArgument('slam_delay_sec', default_value='10.0'),
            DeclareLaunchArgument('nav2_delay_sec', default_value='20.0'),
            DeclareLaunchArgument('window_scans', default_value='20'),
            DeclareLaunchArgument('scan_stride', default_value='3'),
            DeclareLaunchArgument('voxel_size_m', default_value='0.07'),
            DeclareLaunchArgument(
                'max_correspondence_distance_m', default_value='0.25'
            ),
            DeclareLaunchArgument('max_rmse_m', default_value='0.12'),
            DeclareLaunchArgument(
                'max_translation_correction_m', default_value='0.20'
            ),
            DeclareLaunchArgument(
                'max_rotation_correction_rad', default_value='0.13962634'
            ),
            cleanup,
            simulation,
            OpaqueFunction(
                function=lambda context: _frontend_process(context, frontend_script)
            ),
            TimerAction(period=slam_delay_sec, actions=[slam]),
            TimerAction(period=nav2_delay_sec, actions=[nav2]),
        ]
    )
