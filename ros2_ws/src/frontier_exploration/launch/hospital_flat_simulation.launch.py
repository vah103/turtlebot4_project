import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from frontier_exploration.hospital_scale import HOSPITAL_SCALE, prepare_scaled_hospital
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    source_world = os.path.join(package_dir, 'worlds', 'hospital_aws_flat.sdf')
    base_launch = os.path.join(package_dir, 'launch', 'tb4_simulation_safe.launch.py')
    default_rviz_config = os.path.join(
        package_dir, 'rviz', 'hospital_exploration.rviz'
    )
    default_robot_sdf = os.path.join(
        package_dir, 'urdf', 'hospital_turtlebot4.urdf.xacro'
    )

    source_models = (
        Path.home()
        / '.cache'
        / 'turtlebot4_project'
        / 'hospital_world'
        / 'models'
    )
    hospital = prepare_scaled_hospital(
        source_world=source_world,
        source_models_dir=source_models,
        scale=HOSPITAL_SCALE,
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    robot_sdf = LaunchConfiguration('robot_sdf')
    headless = LaunchConfiguration('headless')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'world': str(hospital.world),
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file,
            'robot_sdf': robot_sdf,
            'headless': headless,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='RViz is off by default for lower-load Hospital runs.',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='RViz config used by the flat Hospital simulation.',
        ),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=default_robot_sdf,
            description='TurtleBot4 xacro used by the flat Hospital simulation.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='True',
            description='Skip the Gazebo graphical client by default.',
        ),
        DeclareLaunchArgument('x_pose', default_value=str(hospital.spawn_x)),
        DeclareLaunchArgument('y_pose', default_value=str(hospital.spawn_y)),
        DeclareLaunchArgument('yaw', default_value=str(hospital.spawn_yaw)),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(hospital.models_dir)),
        LogInfo(
            msg=(
                f'Hospital geometry scale: x{hospital.scale:g}; '
                f'world={hospital.world}; '
                f'default_spawn=({hospital.spawn_x:g}, {hospital.spawn_y:g})'
            )
        ),
        simulation,
    ])
