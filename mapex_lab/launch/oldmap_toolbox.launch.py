"""Launch TurtleBot4 with Old-Map-First V2.

This diagnostic profile keeps the normal SLAM Toolbox loop-closure detector,
whole-graph Ceres optimization, New Room simulation and Nav2 behavior.
Old-map-first confidence is applied at two layers:

1. Local scan matching:
   - the initial sequential/local match keeps the normal recent running buffer
     (30 scans) and adds sparse historical keyframes near the predicted pose;
   - the first scan is retained as a trusted historical keyframe;
   - earlier reference scans contribute more strongly than later scans using
     c(i) = c_min + (1-c_min) * exp(-i / 70), c_min=0.25;
   - active confidences are normalized by the strongest active reference so
     relative order S1>S2>S3... is preserved without collapsing matcher
     response when only late scans are locally available;
   - OldMapMapper mirrors Karto AddEdges topology, but the local near-chain
     MatchScan step also uses the same old>new weighted correlation rule;
   - loop-closure coarse/fine matching itself stays upstream/unweighted.

2. Pose-graph optimization:
   - the first scan pose remains hard-fixed exactly as upstream Ceres does;
   - local constraints with node gap <= 5 receive
     w(n) = 1 + 4 * exp(-n / 70), i.e. early constraints start near 5x and
     monotonically decay toward ordinary Toolbox 1x;
   - long-gap / loop constraints remain at 1x;
   - adaptive release is disabled (1.0 -> 1.0) for this diagnostic, so loop
     evidence cannot weaken the old-map temporal preference.

The first scan is not forced into every local match. It participates directly
only when it is spatially relevant to the predicted current pose; otherwise its
confidence is transmitted progressively through trusted local history and the
pose graph.
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
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _write_yaml_temp(data: dict, prefix: str) -> str:
    temporary = tempfile.NamedTemporaryFile(
        mode='w',
        prefix=prefix,
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    try:
        yaml.safe_dump(data, temporary, sort_keys=False)
        return temporary.name
    finally:
        temporary.close()


def _cleanup_temp_files(_context, paths):
    for path in paths:
        try:
            os.remove(path)
        except FileNotFoundError:
            pass
    return []


def generate_launch_description() -> LaunchDescription:
    frontier_pkg = get_package_share_directory('frontier_exploration')
    tb4_nav_pkg = get_package_share_directory('turtlebot4_navigation')
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')
    research_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    upstream_slam_params = os.path.join(
        slam_toolbox_pkg,
        'config',
        'mapper_params_online_async.yaml',
    )
    with open(upstream_slam_params, 'r', encoding='utf-8') as stream:
        slam_config = yaml.safe_load(stream) or {}

    slam_ros_params = slam_config.setdefault('slam_toolbox', {}).setdefault(
        'ros__parameters', {}
    )
    slam_ros_params.update(
        {
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'map_update_interval': 1.0,
            'resolution': 0.10,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.20,
            'minimum_travel_distance': 0.10,
            'minimum_travel_heading': 0.10,
            'scan_buffer_size': 30,
            'do_loop_closing': True,
            'loop_match_minimum_response_coarse': 0.55,
            'loop_match_minimum_response_fine': 0.65,
            'loop_match_minimum_chain_size': 30,
            'loop_search_maximum_distance': 2.0,
            'loop_search_space_dimension': 4.0,
            'loop_match_maximum_variance_coarse': 2.0,

            # Old-Map-First V2 sequential/local scan matcher.
            'oldmap_scan_weighting_enabled': False,
            'oldmap_scan_min_confidence': 0.25,
            'oldmap_scan_decay_nodes': 70.0,
            'oldmap_keep_first_scan': True,
            'oldmap_keyframe_distance': 0.5,
            'oldmap_history_search_radius': 3.0,
            'oldmap_history_max_keyframes': 40,

            # Old-Map-First pose-graph hierarchy.
            'adaptive_anchor_enabled': True,
            'adaptive_anchor_min_weight': 1.0,
            'adaptive_anchor_max_weight': 5.0,
            'adaptive_anchor_decay_nodes': 70.0,
            'adaptive_anchor_local_edge_max_gap': 5,
            'adaptive_anchor_loop_min_node_gap': 30,
            'adaptive_anchor_loop_evidence_min_edges': 3,
            'adaptive_anchor_loop_evidence_window_nodes': 120,
            'adaptive_anchor_loop_evidence_min_new_node_separation': 2,
            'adaptive_anchor_loop_consistency_translation_m': 0.20,
            'adaptive_anchor_loop_consistency_yaw_deg': 3.0,
            'adaptive_anchor_loop_min_mahalanobis_sq': 9.0,
            # V2 old-map-first: do not release historical temporal weighting.
            'adaptive_anchor_release_stage1_factor': 1.0,
            'adaptive_anchor_release_stage2_factor': 1.0,
        }
    )
    slam_params = _write_yaml_temp(
        slam_config,
        prefix='tb4_mapex_slam_toolbox_oldmap_v2_',
    )

    stock_nav2_params = os.path.join(tb4_nav_pkg, 'config', 'nav2.yaml')
    nav2_override_path = os.path.join(research_root, 'config', 'nav2.yaml')

    with open(stock_nav2_params, 'r', encoding='utf-8') as stream:
        nav2_base = yaml.safe_load(stream) or {}
    with open(nav2_override_path, 'r', encoding='utf-8') as stream:
        nav2_override = yaml.safe_load(stream) or {}

    nav2_merged = _deep_merge(nav2_base, nav2_override)
    nav2_params = _write_yaml_temp(
        nav2_merged,
        prefix='tb4_toolbox_oldmap_v2_nav2_',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
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
            'world': world,
            'use_rviz': use_rviz,
            'headless': headless,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_pkg, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
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
                    function=lambda context: _cleanup_temp_files(
                        context,
                        [slam_params, nav2_params],
                    )
                )
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('world', default_value='new_room'),
            DeclareLaunchArgument('x_pose', default_value='auto'),
            DeclareLaunchArgument('y_pose', default_value='auto'),
            DeclareLaunchArgument('yaw', default_value='auto'),
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
