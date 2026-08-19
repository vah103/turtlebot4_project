"""Launch Nav2 for the Hospital experiment with project-local overrides."""

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


def _deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override values into a copy of base."""
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def _launch_bool(context, name: str) -> str:
    """Return a PythonExpression-safe boolean literal for nested launches."""
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    if raw in {'1', 'true', 'yes', 'on'}:
        return 'True'
    if raw in {'0', 'false', 'no', 'off'}:
        return 'False'
    raise RuntimeError(f'Invalid boolean launch value for {name}: {raw!r}')


def _cleanup_temp_file(_context, path: str):
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
    return []


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
                    function=lambda shutdown_context: _cleanup_temp_file(
                        shutdown_context,
                        merged_params_path,
                    )
                )
            ]
        )
    )
    return [cleanup, nav2]


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
            OpaqueFunction(function=_launch_nav2),
        ]
    )
