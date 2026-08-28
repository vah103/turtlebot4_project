"""Launch Hospital simulation + Cartographer 2D + stock TurtleBot4 Nav2.

This is the Cartographer alternative to mapex_lab/launch/stock.launch.py.
It keeps the same Hospital simulation and stock-style Nav2 debug setup, but
replaces slam_toolbox with Cartographer 2D using wheel odometry, IMU, local
submaps, and pose-graph optimization.

Cartographer publishes a probabilistic OccupancyGrid on /cartographer_map. A
small compatibility bridge thresholds that grid to the discrete MapEx/Nearest
convention on /map: unknown=-1, free=0, occupied=100. The published /map
resolution remains 0.10 m/cell so nf_basic.py can be used without changing its
frontier semantics. Cartographer's internal submaps stay at 0.05 m/cell for more
precise scan matching.
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
from launch_ros.actions import Node


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

    cartographer_config_dir = os.path.join(research_root, 'config')
    cartographer_config_basename = 'cartographer_hospital_2d.lua'
    cartographer_map_bridge = os.path.join(
        research_root,
        'cartographer_map_bridge.py',
    )

    # Keep the same stock-style Nav2 debug profile used by stock.launch.py.
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
        prefix='tb4_cartographer_stock_xy_only_',
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
    cartographer_delay_sec = LaunchConfiguration('cartographer_delay_sec')
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

    # The Gazebo IMU bridge currently stamps /imu with the scoped sensor frame
    # "turtlebot4/imu_link/imu" while robot_state_publisher exposes the physical
    # link as "imu_link". The sensor itself has zero pose relative to imu_link,
    # so publish the missing identity transform instead of discarding IMU data.
    imu_sensor_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cartographer_imu_sensor_frame_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'imu_link',
            '--child-frame-id', 'turtlebot4/imu_link/imu',
        ],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory',
            cartographer_config_dir,
            '-configuration_basename',
            cartographer_config_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution',
            '0.10',
            '-publish_period_sec',
            '1.0',
        ],
        remappings=[('map', '/cartographer_map')],
    )

    map_bridge = ExecuteProcess(
        cmd=['/usr/bin/python3', cartographer_map_bridge],
        output='screen',
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
            DeclareLaunchArgument('cartographer_delay_sec', default_value='10.0'),
            DeclareLaunchArgument('nav2_delay_sec', default_value='20.0'),
            cleanup,
            simulation,
            imu_sensor_frame_tf,
            TimerAction(
                period=cartographer_delay_sec,
                actions=[cartographer_node, occupancy_grid_node, map_bridge],
            ),
            TimerAction(period=nav2_delay_sec, actions=[nav2]),
        ]
    )
