import hashlib
import importlib.util
import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


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
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _launch_simulation(context, package_dir: str, project_root: Path, hospital):
    world_arg = LaunchConfiguration('world').perform(context).strip()
    x_arg = LaunchConfiguration('x_pose').perform(context).strip()
    y_arg = LaunchConfiguration('y_pose').perform(context).strip()
    yaw_arg = LaunchConfiguration('yaw').perform(context).strip()
    expected_world_sha = LaunchConfiguration('mx048_world_sha256').perform(context).strip()
    layout_config_arg = LaunchConfiguration('mx048_layout_config').perform(context).strip()
    expected_layout_sha = LaunchConfiguration('mx048_layout_config_sha256').perform(context).strip()
    dry_launch = LaunchConfiguration('mx048_dry_launch').perform(context).strip().lower() == 'true'

    if world_arg in ('', 'hospital'):
        world_path = str(hospital.world)
        default_spawn = (hospital.spawn_x, hospital.spawn_y, hospital.spawn_yaw)
        world_label = 'hospital'
    elif world_arg == 'new_room':
        world_path = str(project_root / 'mapex_lab' / 'map' / 'new_room.sdf')
        default_spawn = (0.0, 3.0, 0.0)
        world_label = 'new_room'
    else:
        world_path = str(Path(world_arg).expanduser().resolve())
        default_spawn = (0.0, 0.0, 0.0)
        world_label = world_path

    spawn_x = str(default_spawn[0]) if x_arg in ('', 'auto') else x_arg
    spawn_y = str(default_spawn[1]) if y_arg in ('', 'auto') else y_arg
    spawn_yaw = str(default_spawn[2]) if yaw_arg in ('', 'auto') else yaw_arg

    world_file = Path(world_path)
    if not world_file.is_file():
        raise FileNotFoundError(f'Simulation world does not exist: {world_path}')

    binding_msg = (
        f'Simulation world: {world_label}; path={world_path}; '
        f'spawn=({spawn_x}, {spawn_y}, {spawn_yaw})'
    )
    if expected_world_sha or layout_config_arg or expected_layout_sha:
        if not (expected_world_sha and layout_config_arg and expected_layout_sha):
            raise RuntimeError('MX048 launch binding requires world SHA, layout-config path, and layout-config SHA together')
        actual_world_sha = _sha256(world_file)
        layout_config_path = Path(layout_config_arg).expanduser().resolve()
        if not layout_config_path.is_file():
            raise FileNotFoundError(f'MX048 layout config does not exist: {layout_config_path}')
        actual_layout_sha = _sha256(layout_config_path)
        if actual_world_sha != expected_world_sha:
            raise RuntimeError(f'MX048 world hash mismatch: {actual_world_sha} != {expected_world_sha}')
        if actual_layout_sha != expected_layout_sha:
            raise RuntimeError(f'MX048 layout-config hash mismatch: {actual_layout_sha} != {expected_layout_sha}')
        binding_msg = (
            'MX048_LAUNCH_BINDING_PASS '
            f'world_path={world_path} world_sha256={actual_world_sha} '
            f'layout_config_path={layout_config_path} layout_config_sha256={actual_layout_sha} '
            f'spawn=({spawn_x},{spawn_y},{spawn_yaw})'
        )

    if dry_launch:
        return [LogInfo(msg=binding_msg)]

    base_launch = os.path.join(package_dir, 'launch', 'tb4_simulation_safe.launch.py')
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'world': world_path,
            'gz_seed': LaunchConfiguration('gz_seed'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
            'robot_sdf': LaunchConfiguration('robot_sdf'),
            'headless': LaunchConfiguration('headless'),
            'x_pose': spawn_x,
            'y_pose': spawn_y,
            'yaw': spawn_yaw,
        }.items(),
    )

    return [LogInfo(msg=binding_msg), simulation]


def generate_launch_description() -> LaunchDescription:
    package_dir = get_package_share_directory('frontier_exploration')
    project_root = _find_project_root(package_dir)
    hospital_scale = _load_hospital_scale(project_root)
    hospital = hospital_scale.prepare_scaled_hospital()

    default_rviz_config = os.path.join(
        package_dir, 'rviz', 'hospital_exploration.rviz'
    )
    default_robot_sdf = os.path.join(
        package_dir, 'urdf', 'hospital_turtlebot4.urdf.xacro'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='hospital',
            description='World selector: hospital, new_room, or an explicit SDF path.',
        ),
        DeclareLaunchArgument('gz_seed', default_value='0'),
        DeclareLaunchArgument('mx048_world_sha256', default_value=''),
        DeclareLaunchArgument('mx048_layout_config', default_value=''),
        DeclareLaunchArgument('mx048_layout_config_sha256', default_value=''),
        DeclareLaunchArgument('mx048_dry_launch', default_value='false'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='False',
            description='RViz is off by default for lower-load runs.',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='RViz config used by the simulation.',
        ),
        DeclareLaunchArgument(
            'robot_sdf',
            default_value=default_robot_sdf,
            description='TurtleBot4 xacro used by the simulation.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='True',
            description='Skip the Gazebo graphical client by default.',
        ),
        DeclareLaunchArgument('x_pose', default_value='auto'),
        DeclareLaunchArgument('y_pose', default_value='auto'),
        DeclareLaunchArgument('yaw', default_value='auto'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(hospital.models_dir)),
        OpaqueFunction(
            function=lambda context: _launch_simulation(
                context, package_dir, project_root, hospital
            )
        ),
    ])
