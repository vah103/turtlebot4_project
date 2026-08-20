"""Launch Nav2 for the Hospital experiment with project-local overrides."""

import os
from pathlib import Path
import shutil
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
from launch_ros.actions import Node

from frontier_exploration.hospital_keepout import generate_hospital_keepout_mask


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override values into a copy of base."""
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _launch_bool_value(context, name: str) -> bool:
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    if raw in {'1', 'true', 'yes', 'on'}:
        return True
    if raw in {'0', 'false', 'no', 'off'}:
        return False
    raise RuntimeError(f'Invalid boolean launch value for {name}: {raw!r}')


def _launch_bool(context, name: str) -> str:
    """Return a PythonExpression-safe boolean literal for nested launches."""
    return 'True' if _launch_bool_value(context, name) else 'False'


def _cleanup_temp_paths(_context, paths: list[str]):
    for path in paths:
        if os.path.isdir(path):
            shutil.rmtree(path, ignore_errors=True)
            continue
        try:
            os.remove(path)
        except FileNotFoundError:
            pass
    return []


def _disable_keepout_filter(merged: dict) -> None:
    """Remove the optional keepout filter cleanly when launch disables it."""
    for costmap_name in ('global_costmap', 'local_costmap'):
        params = (
            merged.get(costmap_name, {})
            .get(costmap_name, {})
            .get('ros__parameters', {})
        )
        filters = params.get('filters', [])
        if isinstance(filters, list):
            params['filters'] = [
                item for item in filters if item != 'keepout_filter'
            ]
        params.pop('keepout_filter', None)


def _launch_nav2(context):
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    package_dir = get_package_share_directory('frontier_exploration')

    base_params = LaunchConfiguration('base_params_file').perform(context)
    override_params = os.path.join(
        package_dir,
        'config',
        'nav2_hospital_override.yaml',
    )

    with open(base_params, 'r', encoding='utf-8') as stream:
        base = yaml.safe_load(stream) or {}
    with open(override_params, 'r', encoding='utf-8') as stream:
        override = yaml.safe_load(stream) or {}

    merged = _deep_merge(base, override)
    keepout_enabled = _launch_bool_value(context, 'keepout_enabled')
    if not keepout_enabled:
        _disable_keepout_filter(merged)

    temporary = tempfile.NamedTemporaryFile(
        mode='w',
        prefix='hospital_nav2_',
        suffix='.yaml',
        delete=False,
        encoding='utf-8',
    )
    try:
        yaml.safe_dump(merged, temporary, sort_keys=False)
        merged_params_path = temporary.name
    finally:
        temporary.close()

    cleanup_paths = [merged_params_path]
    keepout_actions = []

    if keepout_enabled:
        keepout_dir = tempfile.mkdtemp(prefix='hospital_keepout_')
        cleanup_paths.append(keepout_dir)
        world_path = Path(package_dir) / 'worlds' / 'hospital_aws.sdf'
        models_dir = (
            Path.home()
            / '.cache'
            / 'turtlebot4_project'
            / 'hospital_world'
            / 'models'
        )
        result = generate_hospital_keepout_mask(
            world_path,
            models_dir,
            Path(keepout_dir),
            resolution=float(
                LaunchConfiguration('keepout_resolution').perform(context)
            ),
            safety_margin_m=float(
                LaunchConfiguration('keepout_safety_margin_m').perform(context)
            ),
            outer_padding_m=float(
                LaunchConfiguration('keepout_outer_padding_m').perform(context)
            ),
            start_x=float(LaunchConfiguration('start_x').perform(context)),
            start_y=float(LaunchConfiguration('start_y').perform(context)),
            start_yaw=float(LaunchConfiguration('start_yaw').perform(context)),
        )
        print(
            '[hospital_nav2] Generated collision-mesh keepout mask: '
            f'{result.width}x{result.height} cells, '
            f'{result.resolution:.3f} m/cell, '
            f'safety margin='
            f'{LaunchConfiguration("keepout_safety_margin_m").perform(context)} m'
        )

        use_sim_time = _launch_bool_value(context, 'use_sim_time')
        filter_mask_server = Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': str(result.yaml_path),
                    'topic_name': 'keepout_filter_mask',
                    'frame_id': 'map',
                }
            ],
        )
        filter_info_server = Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'type': 0,
                    'filter_info_topic': 'costmap_filter_info',
                    'mask_topic': 'keepout_filter_mask',
                    'base': 0.0,
                    'multiplier': 1.0,
                }
            ],
        )
        filter_lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_keepout',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'filter_mask_server',
                        'costmap_filter_info_server',
                    ],
                }
            ],
        )
        keepout_actions = [
            filter_mask_server,
            filter_info_server,
            filter_lifecycle_manager,
        ]

    # On ROS 2 Jazzy, keep cmd_vel as geometry_msgs/msg/Twist end-to-end.
    # The Hospital Gazebo bridge already subscribes to Twist on /cmd_vel, so a
    # global /cmd_vel -> /cmd_vel_stamped remap would mix Twist and TwistStamped
    # publishers on the same topic and prevent the robot from receiving commands.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': _launch_bool(context, 'use_sim_time'),
            'params_file': merged_params_path,
            'autostart': _launch_bool(context, 'autostart'),
            'use_composition': _launch_bool(context, 'use_composition'),
        }.items(),
    )

    cleanup = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda shutdown_context: _cleanup_temp_paths(
                        shutdown_context,
                        cleanup_paths,
                    )
                )
            ]
        )
    )
    return [cleanup, *keepout_actions, nav2]


def generate_launch_description() -> LaunchDescription:
    default_base_params = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'params',
        'nav2_params.yaml',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'base_params_file',
                default_value=default_base_params,
                description='Base Nav2 parameters to overlay for Hospital.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo /clock source.',
            ),
            DeclareLaunchArgument(
                'autostart',
                default_value='true',
                description='Automatically activate the Nav2 lifecycle nodes.',
            ),
            DeclareLaunchArgument(
                'use_composition',
                default_value='false',
                description=(
                    'Use Nav2 component composition. Hospital defaults to false '
                    'because no external nav2_container is launched.'
                ),
            ),
            DeclareLaunchArgument(
                'keepout_enabled',
                default_value='true',
                description=(
                    'Generate a keepout mask from the Hospital floor collision '
                    'mesh and apply it to both Nav2 costmaps.'
                ),
            ),
            DeclareLaunchArgument(
                'keepout_resolution',
                default_value='0.05',
                description='Keepout-mask resolution in metres per cell.',
            ),
            DeclareLaunchArgument(
                'keepout_safety_margin_m',
                default_value='0.40',
                description='Clearance kept inside supported floor edges.',
            ),
            DeclareLaunchArgument(
                'keepout_outer_padding_m',
                default_value='2.0',
                description='Blocked border outside the floor mesh bounds.',
            ),
            DeclareLaunchArgument(
                'start_x',
                default_value='0.0',
                description='Robot initial SDF-world x used by the SLAM frame.',
            ),
            DeclareLaunchArgument(
                'start_y',
                default_value='12.0',
                description='Robot initial SDF-world y used by the SLAM frame.',
            ),
            DeclareLaunchArgument(
                'start_yaw',
                default_value='-1.57',
                description='Robot initial SDF-world yaw used by the SLAM frame.',
            ),
            OpaqueFunction(function=_launch_nav2),
        ]
    )
