import importlib.util
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _find_project_root(package_dir: str) -> Path:
    env_root = os.environ.get('TURTLEBOT4_PROJECT_ROOT')
    if env_root:
        candidate = Path(env_root).expanduser().resolve()
        if (candidate / 'mapex_lab/scripts/hospital_scale.py').is_file():
            return candidate

    package_path = Path(package_dir).resolve()
    for parent in (package_path, *package_path.parents):
        if (parent / 'mapex_lab/scripts/hospital_scale.py').is_file():
            return parent

    fallback = Path.home() / 'turtlebot4_project'
    if (fallback / 'mapex_lab/scripts/hospital_scale.py').is_file():
        return fallback

    raise FileNotFoundError(
        'Could not locate turtlebot4_project/mapex_lab. '
        'Set TURTLEBOT4_PROJECT_ROOT to the repository root.'
    )


def _load_hospital_scale(project_root: Path):
    module_path = project_root / 'mapex_lab/scripts/hospital_scale.py'
    spec = importlib.util.spec_from_file_location('mapex_lab_hospital_scale', module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Could not load Hospital scale module: {module_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    project_root = _find_project_root(package_dir)
    hospital_scale = _load_hospital_scale(project_root)
    hospital = hospital_scale.prepare_scaled_hospital()

    base_launch = os.path.join(package_dir, 'launch', 'tb4_simulation_safe.launch.py')
    default_rviz_config = os.path.join(
        package_dir, 'rviz', 'hospital_exploration.rviz'
    )
    default_robot_sdf = os.path.join(
        package_dir, 'urdf', 'hospital_turtlebot4.urdf.xacro'
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
                f'models={hospital.models_dir}; '
                f'default_spawn=({hospital.spawn_x:g}, {hospital.spawn_y:g})'
            )
        ),
        simulation,
    ])
