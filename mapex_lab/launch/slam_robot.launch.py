"""SLAM launch for a real TurtleBot 4.

Keeps the SLAM configuration used by mapex_lab/launch/slam.launch.py, but
removes simulation and Nav2. The real robot is expected to provide /scan,
/odom and TF. Intended for manual teleoperation while mapping.
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
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    slam_toolbox_pkg = get_package_share_directory('slam_toolbox')

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

    # Keep this block aligned with mapex_lab/launch/slam.launch.py so that
    # simulation and real-robot mapping use the same SLAM algorithm/settings.
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

            'oldmap_scan_weighting_enabled': False,
            'oldmap_scan_min_confidence': 0.25,
            'oldmap_scan_decay_nodes': 70.0,
            'oldmap_keep_first_scan': True,
            'oldmap_keyframe_distance': 1.0,
            'oldmap_history_search_radius': 1.5,
            'oldmap_history_max_keyframes': 6,

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
            'adaptive_anchor_release_stage1_factor': 1.0,
            'adaptive_anchor_release_stage2_factor': 1.0,
        }
    )

    slam_params = _write_yaml_temp(
        slam_config,
        prefix='tb4_real_slam_',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

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

    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda context: _cleanup_temp_files(
                        context,
                        [slam_params],
                    )
                )
            ]
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            cleanup,
            slam,
        ]
    )
