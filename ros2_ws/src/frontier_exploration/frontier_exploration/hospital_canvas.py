"""Compute a fixed LaMa canvas from the known Hospital World geometry."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


FLOOR_MODEL = 'aws_robomaker_hospital_floor_01_floor'
WALLS_MODEL = 'aws_robomaker_hospital_floor_01_walls'
MODEL_COLLISION_FILES = {
    FLOOR_MODEL: 'aws_robomaker_hospital_floor_01_floor_collision.dae',
    WALLS_MODEL: 'aws_robomaker_hospital_floor_01_walls_collision.dae',
}

# These defaults must match hospital_simulation.launch.py. Gazebo places the
# robot in the world at this pose, while DiffDrive odometry / SLAM begin in the
# robot's local start frame. The fixed LaMa canvas therefore has to be expressed
# in that start frame rather than directly in SDF world coordinates.
DEFAULT_START_X = 0.0
DEFAULT_START_Y = 12.0
DEFAULT_START_YAW = -1.57


@dataclass(frozen=True)
class Bounds2D:
    """Axis-aligned bounds in metres."""

    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class ModelPlacement:
    """Planar placement of one model in the SDF world."""

    model_name: str
    x: float
    y: float
    yaw: float


def _local_name(tag: str) -> str:
    return tag.rsplit('}', 1)[-1]


def collada_ground_bounds(path: Path) -> Bounds2D:
    """Return the X/Y footprint of a Y-up COLLADA mesh in metres.

    AWS Hospital meshes are authored with Y as the vertical axis. Gazebo
    converts that to its Z-up frame, so COLLADA X/Z form the ground plane.
    Only POSITION sources referenced by <vertices> are considered.
    """
    root = ET.parse(path).getroot()

    unit_scale = 1.0
    up_axis = 'Y_UP'
    asset = next((node for node in root if _local_name(node.tag) == 'asset'), None)
    if asset is not None:
        for child in asset:
            name = _local_name(child.tag)
            if name == 'unit':
                unit_scale = float(child.attrib.get('meter', '1.0'))
            elif name == 'up_axis' and child.text:
                up_axis = child.text.strip()

    if up_axis != 'Y_UP':
        raise ValueError(
            f'Expected a Y_UP COLLADA mesh, got {up_axis!r}: {path}'
        )
    if unit_scale <= 0.0:
        raise ValueError(f'Invalid COLLADA unit scale {unit_scale}: {path}')

    points: list[tuple[float, float]] = []

    for geometry in root.iter():
        if _local_name(geometry.tag) != 'geometry':
            continue
        mesh = next(
            (child for child in geometry if _local_name(child.tag) == 'mesh'),
            None,
        )
        if mesh is None:
            continue

        sources: dict[str, tuple[list[float], int]] = {}
        for source in mesh:
            if _local_name(source.tag) != 'source':
                continue
            source_id = source.attrib.get('id')
            if not source_id:
                continue
            float_array = next(
                (
                    child
                    for child in source
                    if _local_name(child.tag) == 'float_array'
                ),
                None,
            )
            if float_array is None or not float_array.text:
                continue
            values = [float(value) for value in float_array.text.split()]
            stride = 1
            for descendant in source.iter():
                if _local_name(descendant.tag) == 'accessor':
                    stride = int(descendant.attrib.get('stride', '1'))
                    break
            sources[source_id] = (values, stride)

        position_source_ids: set[str] = set()
        for vertices in mesh:
            if _local_name(vertices.tag) != 'vertices':
                continue
            for input_node in vertices:
                if (
                    _local_name(input_node.tag) == 'input'
                    and input_node.attrib.get('semantic') == 'POSITION'
                ):
                    source_ref = input_node.attrib.get('source', '')
                    if source_ref.startswith('#'):
                        position_source_ids.add(source_ref[1:])

        for source_id in position_source_ids:
            if source_id not in sources:
                raise ValueError(
                    f'COLLADA POSITION source {source_id!r} not found: {path}'
                )
            values, stride = sources[source_id]
            if stride < 3 or len(values) % stride != 0:
                raise ValueError(
                    f'Invalid COLLADA POSITION source {source_id!r}: {path}'
                )
            for index in range(0, len(values), stride):
                local_x = values[index] * unit_scale
                # Y_UP -> Gazebo Z_UP is a +90 degree rotation about X,
                # therefore Gazebo planar Y is -COLLADA Z.
                local_y = -values[index + 2] * unit_scale
                points.append((local_x, local_y))

    if not points:
        raise ValueError(f'No COLLADA POSITION vertices found: {path}')

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return Bounds2D(min(xs), max(xs), min(ys), max(ys))


def read_model_placements(
    world_path: Path,
    model_names: set[str],
) -> dict[str, ModelPlacement]:
    """Read planar poses of selected model:// includes from an SDF world."""
    root = ET.parse(world_path).getroot()
    placements: dict[str, ModelPlacement] = {}

    for include in root.iter('include'):
        uri_node = include.find('uri')
        if uri_node is None or not uri_node.text:
            continue
        uri = uri_node.text.strip()
        if not uri.startswith('model://'):
            continue
        model_name = uri[len('model://'):].strip('/')
        if model_name not in model_names:
            continue

        pose_node = include.find('pose')
        pose = [0.0] * 6
        if pose_node is not None and pose_node.text:
            values = [float(value) for value in pose_node.text.split()]
            if len(values) != 6:
                raise ValueError(
                    f'Expected 6 values in pose for {model_name}: {world_path}'
                )
            pose = values

        roll, pitch, yaw = pose[3], pose[4], pose[5]
        if abs(roll) > 1e-6 or abs(pitch) > 1e-6:
            raise ValueError(
                f'{model_name} is tilted; planar bounds tool only supports '
                'zero roll/pitch'
            )

        placements[model_name] = ModelPlacement(
            model_name=model_name,
            x=pose[0],
            y=pose[1],
            yaw=yaw,
        )

    missing = model_names - placements.keys()
    if missing:
        raise ValueError(
            'World is missing required Hospital model(s): '
            + ', '.join(sorted(missing))
        )
    return placements


def transform_bounds(bounds: Bounds2D, placement: ModelPlacement) -> Bounds2D:
    """Transform the four corners of local AABB into the world frame."""
    cosine = math.cos(placement.yaw)
    sine = math.sin(placement.yaw)
    transformed: list[tuple[float, float]] = []

    for local_x in (bounds.min_x, bounds.max_x):
        for local_y in (bounds.min_y, bounds.max_y):
            world_x = placement.x + cosine * local_x - sine * local_y
            world_y = placement.y + sine * local_x + cosine * local_y
            transformed.append((world_x, world_y))

    xs = [point[0] for point in transformed]
    ys = [point[1] for point in transformed]
    return Bounds2D(min(xs), max(xs), min(ys), max(ys))


def transform_world_bounds_to_start_frame(
    bounds: Bounds2D,
    *,
    start_x: float,
    start_y: float,
    start_yaw: float,
) -> Bounds2D:
    """Express SDF-world bounds in the robot/SLAM frame at simulation start."""
    cosine = math.cos(-start_yaw)
    sine = math.sin(-start_yaw)
    transformed: list[tuple[float, float]] = []

    for world_x in (bounds.min_x, bounds.max_x):
        for world_y in (bounds.min_y, bounds.max_y):
            dx = world_x - start_x
            dy = world_y - start_y
            frame_x = cosine * dx - sine * dy
            frame_y = sine * dx + cosine * dy
            transformed.append((frame_x, frame_y))

    xs = [point[0] for point in transformed]
    ys = [point[1] for point in transformed]
    return Bounds2D(min(xs), max(xs), min(ys), max(ys))


def union_bounds(bounds: list[Bounds2D]) -> Bounds2D:
    if not bounds:
        raise ValueError('At least one bounds object is required')
    return Bounds2D(
        min(item.min_x for item in bounds),
        max(item.max_x for item in bounds),
        min(item.min_y for item in bounds),
        max(item.max_y for item in bounds),
    )


def canvas_from_bounds(
    bounds: Bounds2D,
    resolution: float,
    margin_m: float,
) -> dict:
    """Snap padded metric bounds outward to whole occupancy-grid cells."""
    if resolution <= 0.0:
        raise ValueError('resolution must be positive')
    if margin_m < 0.0:
        raise ValueError('margin must be non-negative')

    origin_x = math.floor((bounds.min_x - margin_m) / resolution) * resolution
    origin_y = math.floor((bounds.min_y - margin_m) / resolution) * resolution
    max_x = math.ceil((bounds.max_x + margin_m) / resolution) * resolution
    max_y = math.ceil((bounds.max_y + margin_m) / resolution) * resolution

    width = int(round((max_x - origin_x) / resolution))
    height = int(round((max_y - origin_y) / resolution))

    return {
        'fixed_canvas_configured': True,
        'canvas_width_cells': width,
        'canvas_height_cells': height,
        'canvas_resolution': resolution,
        'canvas_origin_x': origin_x,
        'canvas_origin_y': origin_y,
        'canvas_max_x': max_x,
        'canvas_max_y': max_y,
    }


def compute_hospital_canvas(
    world_path: Path,
    models_dir: Path,
    resolution: float,
    margin_m: float,
    *,
    start_x: float = DEFAULT_START_X,
    start_y: float = DEFAULT_START_Y,
    start_yaw: float = DEFAULT_START_YAW,
) -> tuple[Bounds2D, dict]:
    """Compute Hospital bounds/canvas in the SLAM frame fixed at robot start."""
    model_names = set(MODEL_COLLISION_FILES)
    placements = read_model_placements(world_path, model_names)
    world_bounds: list[Bounds2D] = []

    for model_name, collision_filename in MODEL_COLLISION_FILES.items():
        mesh_path = models_dir / model_name / 'meshes' / collision_filename
        if not mesh_path.is_file():
            raise FileNotFoundError(
                f'Hospital asset not found: {mesh_path}\n'
                'Run scripts/setup_hospital_world_assets.sh first.'
            )
        local_bounds = collada_ground_bounds(mesh_path)
        world_bounds.append(
            transform_bounds(local_bounds, placements[model_name])
        )

    geometry_world = union_bounds(world_bounds)
    geometry_start_frame = transform_world_bounds_to_start_frame(
        geometry_world,
        start_x=start_x,
        start_y=start_y,
        start_yaw=start_yaw,
    )
    return geometry_start_frame, canvas_from_bounds(
        geometry_start_frame,
        resolution=resolution,
        margin_m=margin_m,
    )


def _default_world_path() -> Path:
    return (
        Path(get_package_share_directory('frontier_exploration'))
        / 'worlds'
        / 'hospital_aws.sdf'
    )


def _default_models_dir() -> Path:
    return (
        Path.home()
        / '.cache'
        / 'turtlebot4_project'
        / 'hospital_world'
        / 'models'
    )


def _format_number(value: float) -> str:
    text = f'{value:.6f}'.rstrip('0').rstrip('.')
    return '0' if text in {'-0', ''} else text


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Compute a fixed occupancy-grid canvas from Hospital World floor '
            'and wall geometry in the robot/SLAM start frame before SLAM starts.'
        )
    )
    parser.add_argument('--world', type=Path, default=_default_world_path())
    parser.add_argument('--models-dir', type=Path, default=_default_models_dir())
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--margin', type=float, default=2.0)
    parser.add_argument('--start-x', type=float, default=DEFAULT_START_X)
    parser.add_argument('--start-y', type=float, default=DEFAULT_START_Y)
    parser.add_argument('--start-yaw', type=float, default=DEFAULT_START_YAW)
    parsed = parser.parse_args(args)

    bounds, canvas = compute_hospital_canvas(
        parsed.world.expanduser(),
        parsed.models_dir.expanduser(),
        resolution=parsed.resolution,
        margin_m=parsed.margin,
        start_x=parsed.start_x,
        start_y=parsed.start_y,
        start_yaw=parsed.start_yaw,
    )

    print('Hospital geometry bounds in SLAM-start frame (m):')
    print(f'  xmin: {_format_number(bounds.min_x)}')
    print(f'  xmax: {_format_number(bounds.max_x)}')
    print(f'  ymin: {_format_number(bounds.min_y)}')
    print(f'  ymax: {_format_number(bounds.max_y)}')
    print()
    print(
        'Robot start pose in SDF world: '
        f'x={parsed.start_x:.3f}, y={parsed.start_y:.3f}, '
        f'yaw={parsed.start_yaw:.3f}'
    )
    print(
        'Fixed canvas '
        f'(margin={parsed.margin:.2f} m, '
        f'resolution={parsed.resolution:.3f} m/cell):'
    )
    print('  fixed_canvas_configured: true')
    print(f"  canvas_width_cells: {canvas['canvas_width_cells']}")
    print(f"  canvas_height_cells: {canvas['canvas_height_cells']}")
    print(
        '  canvas_resolution: '
        f"{_format_number(canvas['canvas_resolution'])}"
    )
    print(f"  canvas_origin_x: {_format_number(canvas['canvas_origin_x'])}")
    print(f"  canvas_origin_y: {_format_number(canvas['canvas_origin_y'])}")
    print(f"  canvas_max_x: {_format_number(canvas['canvas_max_x'])}")
    print(f"  canvas_max_y: {_format_number(canvas['canvas_max_y'])}")


if __name__ == '__main__':
    main()
