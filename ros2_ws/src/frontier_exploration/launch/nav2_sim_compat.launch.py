import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')

    nav2_launch = os.path.join(
        get_package_share_directory('turtlebot4_navigation'),
        'launch',
        'nav2.launch.py',
    )

    nav2_group = GroupAction(
        [
            SetRemap(src='/cmd_vel', dst='/cmd_vel_stamped'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'namespace': '',
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation clock for Nav2.',
            ),
            nav2_group,
        ]
    )
