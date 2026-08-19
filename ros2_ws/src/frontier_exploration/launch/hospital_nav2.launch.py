"""Launch Nav2 for the Hospital experiment with project-local speed overrides."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


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

    nav2 = GroupAction(
        [
            # Nav2 Jazzy publishes TwistStamped while the Gazebo TB4 bridge
            # expects an unstamped Twist on /cmd_vel. Keep Nav2 on a dedicated
            # stamped topic; frontier_autonomy's sim_twist_adapter converts it.
            SetRemap(src='/cmd_vel', dst='/cmd_vel_stamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                    'params_file': merged_params_path,
                    'autostart': LaunchConfiguration('autostart').perform(context),
                    'use_composition': LaunchConfiguration('use_composition').perform(context),
                }.items(),
            ),
        ]
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
                default_value='true',
                description='Use Nav2 component composition.',
            ),
            OpaqueFunction(function=_launch_nav2),
        ]
    )
