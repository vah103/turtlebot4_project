import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    world = os.path.join(package_dir, 'worlds', 'hospital_aws_flat.sdf')
    base_launch = os.path.join(package_dir, 'launch', 'tb4_simulation_safe.launch.py')
    default_rviz_config = os.path.join(
        package_dir, 'rviz', 'hospital_exploration.rviz'
    )
    default_robot_sdf = os.path.join(
        package_dir, 'urdf', 'hospital_turtlebot4.urdf.xacro'
    )

    model_path = PathJoinSubstitution([
        EnvironmentVariable('HOME'),
        '.cache',
        'turtlebot4_project',
        'hospital_world',
        'models',
    ])

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
            'world': world,
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
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='12.0'),
        DeclareLaunchArgument('yaw', default_value='-1.57'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        simulation,
    ])
