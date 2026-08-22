"""Generate a Nav2 keepout mask from the Hospital floor collision mesh."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from frontier_exploration.hospital_canvas import (
    DEFAULT_START_X,
    DEFAULT_START_Y,
    DEFAULT_START_YAW,
    FLOOR_MODEL,
    MODEL_COLLISION_FILES,
    ModelPlacement,
    read_model_placements,
)


@dataclass(frozen=True)
class KeepoutMask:
    """Generated keepout-mask metadata."""

    image_path: Path
    yaml_path: Path
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int
    safe_cells: int


def _local_name(tag: str) -> str:
    return tag.rsplit('}', 1)[-1]


def _collada_unit_scale(root: ET.Element, path: Path) -> float:
    asset = next(
        (node for node in root if _local_name(node.tag) == 'asset'),
        None,
    )
    unit_scale = 1.0
    up_axis = 'Y_UP'
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
    return unit_scale


def _mesh_sources(mesh: ET.Element) -> dict[str, tuple[list[float], int]]:
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
    return sources


def _vertices_position_sources(mesh: ET.Element) -> dict[str, str]:
    result: dict[str, str] = {}
    for vertices in mesh:
        if _local_name(vertices.tag) != 'vertices':
            continue
        vertices_id = vertices.attrib.get('id')
        if not vertices_id:
            continue
        for input_node in vertices:
            if (
                _local_name(input_node.tag) == 'input'
                and input_node.attrib.get('semantic') == 'POSITION'
            ):
                source = input_node.attrib.get('source', '')
                if source.startswith('#'):
                    result[vertices_id] = source[1:]
    return result


def _source_points(
    source: tuple[list[float], int],
    unit_scale: float,
) -> list[tuple[float, float, float]]:
    values, stride = source
    if stride < 3 or len(values) % stride != 0:
        raise ValueError('Invalid COLLADA POSITION source')

    points: list[tuple[float, float, float]] = []
    for index in range(0, len(values), stride):
        collada_x = values[index] * unit_scale
        collada_y = values[index + 1] * unit_scale
        collada_z = values[index + 2] * unit_scale
        # Keep the same AWS Hospital Y_UP -> Gazebo Z_UP convention used by
        # hospital_canvas.py: planar X is COLLADA X and planar Y is -COLLADA Z.
        points.append((collada_x, -collada_z, collada_y))
    return points


def _polygon_indices(primitive: ET.Element) -> tuple[str, list[list[int]]]:
    inputs = [
        child
        for child in primitive
        if _local_name(child.tag) == 'input'
    ]
    if not inputs:
        return '', []

    vertex_input = next(
        (node for node in inputs if node.attrib.get('semantic') == 'VERTEX'),
        None,
    )
    if vertex_input is None:
        return '', []

    source_ref = vertex_input.attrib.get('source', '')
    if not source_ref.startswith('#'):
        return '', []
    vertices_id = source_ref[1:]
    vertex_offset = int(vertex_input.attrib.get('offset', '0'))
    input_stride = max(
        int(node.attrib.get('offset', '0')) for node in inputs
    ) + 1

    p_node = next(
        (child for child in primitive if _local_name(child.tag) == 'p'),
        None,
    )
    if p_node is None or not p_node.text:
        return vertices_id, []
    packed = [int(value) for value in p_node.text.split()]

    primitive_name = _local_name(primitive.tag)
    if primitive_name == 'polylist':
        vcount_node = next(
            (
                child
                for child in primitive
                if _local_name(child.tag) == 'vcount'
            ),
            None,
        )
        if vcount_node is None or not vcount_node.text:
            return vertices_id, []
        counts = [int(value) for value in vcount_node.text.split()]
    elif primitive_name == 'triangles':
        count = int(primitive.attrib.get('count', '0'))
        counts = [3] * count
    else:
        return vertices_id, []

    polygons: list[list[int]] = []
    cursor = 0
    for count in counts:
        required = count * input_stride
        if cursor + required > len(packed):
            raise ValueError('Malformed COLLADA primitive index buffer')
        polygon = [
            packed[cursor + vertex * input_stride + vertex_offset]
            for vertex in range(count)
        ]
        polygons.append(polygon)
        cursor += required
    return vertices_id, polygons


def collada_support_polygons(
    path: Path,
    *,
    max_support_height_m: float = 0.03,
    min_up_normal_z: float = 0.90,
) -> list[list[tuple[float, float]]]:
    """Return near-ground, upward-facing support polygons from a floor mesh."""
    if max_support_height_m < 0.0:
        raise ValueError('max_support_height_m must be non-negative')
    if not 0.0 < min_up_normal_z <= 1.0:
        raise ValueError('min_up_normal_z must be in (0, 1]')

    root = ET.parse(path).getroot()
    unit_scale = _collada_unit_scale(root, path)
    support: list[list[tuple[float, float]]] = []

    for geometry in root.iter():
        if _local_name(geometry.tag) != 'geometry':
            continue
        mesh = next(
            (child for child in geometry if _local_name(child.tag) == 'mesh'),
            None,
        )
        if mesh is None:
            continue

        sources = _mesh_sources(mesh)
        vertices_sources = _vertices_position_sources(mesh)

        for primitive in mesh:
            if _local_name(primitive.tag) not in {'polylist', 'triangles'}:
                continue
            vertices_id, polygons = _polygon_indices(primitive)
            position_source_id = vertices_sources.get(vertices_id)
            if not position_source_id or position_source_id not in sources:
                continue
            points = _source_points(
                sources[position_source_id],
                unit_scale,
            )

            for indices in polygons:
                if len(indices) < 3:
                    continue
                face = np.asarray([points[index] for index in indices])
                edge_a = face[1] - face[0]
                edge_b = face[2] - face[0]
                normal = np.cross(edge_a, edge_b)
                normal_length = float(np.linalg.norm(normal))
                if normal_length <= 1e-9:
                    continue
                up_component = abs(float(normal[2])) / normal_length
                average_height = float(np.mean(face[:, 2]))
                if up_component < min_up_normal_z:
                    continue
                if abs(average_height) > max_support_height_m:
                    continue
                support.append(
                    [(float(point[0]), float(point[1])) for point in face]
                )

    if not support:
        raise ValueError(f'No near-ground support polygons found: {path}')
    return support


def _transform_model_polygon(
    polygon: list[tuple[float, float]],
    placement: ModelPlacement,
) -> list[tuple[float, float]]:
    cosine = math.cos(placement.yaw)
    sine = math.sin(placement.yaw)
    transformed: list[tuple[float, float]] = []
    for local_x, local_y in polygon:
        transformed.append(
            (
                placement.x + cosine * local_x - sine * local_y,
                placement.y + sine * local_x + cosine * local_y,
            )
        )
    return transformed


def _transform_world_polygon_to_start_frame(
    polygon: list[tuple[float, float]],
    *,
    start_x: float,
    start_y: float,
    start_yaw: float,
) -> list[tuple[float, float]]:
    cosine = math.cos(-start_yaw)
    sine = math.sin(-start_yaw)
    transformed: list[tuple[float, float]] = []
    for world_x, world_y in polygon:
        dx = world_x - start_x
        dy = world_y - start_y
        transformed.append(
            (
                cosine * dx - sine * dy,
                sine * dx + cosine * dy,
            )
        )
    return transformed


def hospital_support_polygons(
    world_path: Path,
    models_dir: Path,
    *,
    start_x: float = DEFAULT_START_X,
    start_y: float = DEFAULT_START_Y,
    start_yaw: float = DEFAULT_START_YAW,
    max_support_height_m: float = 0.03,
) -> list[list[tuple[float, float]]]:
    """Return traversable Hospital floor support in the SLAM start frame."""
    placements = read_model_placements(world_path, {FLOOR_MODEL})
    collision_filename = MODEL_COLLISION_FILES[FLOOR_MODEL]
    mesh_path = (
        models_dir
        / FLOOR_MODEL
        / 'meshes'
        / collision_filename
    )
    if not mesh_path.is_file():
        raise FileNotFoundError(
            f'Hospital floor collision mesh not found: {mesh_path}\n'
            'Run scripts/setup_hospital_world_assets.sh first.'
        )

    local_polygons = collada_support_polygons(
        mesh_path,
        max_support_height_m=max_support_height_m,
    )
    world_polygons = [
        _transform_model_polygon(polygon, placements[FLOOR_MODEL])
        for polygon in local_polygons
    ]
    return [
        _transform_world_polygon_to_start_frame(
            polygon,
            start_x=start_x,
            start_y=start_y,
            start_yaw=start_yaw,
        )
        for polygon in world_polygons
    ]


def _points_inside_polygon(
    polygon: list[tuple[float, float]],
    xs: np.ndarray,
    ys: np.ndarray,
) -> np.ndarray:
    """Vectorized ray-casting test for cell-center coordinate grids."""
    inside = np.zeros((ys.size, xs.size), dtype=bool)
    grid_x = xs[np.newaxis, :]
    grid_y = ys[:, np.newaxis]

    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        crosses = (current_y > grid_y) != (previous_y > grid_y)
        denominator = previous_y - current_y
        if abs(denominator) < 1e-12:
            previous_x, previous_y = current_x, current_y
            continue
        intersection_x = (
            (previous_x - current_x)
            * (grid_y - current_y)
            / denominator
            + current_x
        )
        inside ^= crosses & (grid_x < intersection_x)
        previous_x, previous_y = current_x, current_y
    return inside


def _erode_safe_mask(mask: np.ndarray, cells: int) -> np.ndarray:
    """Conservatively erode safe floor with an 8-neighbour square kernel."""
    eroded = mask.copy()
    for _ in range(max(0, cells)):
        source = eroded
        padded = np.pad(source, 1, constant_values=False)
        eroded = np.ones_like(source)
        for row_offset in range(3):
            for column_offset in range(3):
                eroded &= padded[
                    row_offset:row_offset + source.shape[0],
                    column_offset:column_offset + source.shape[1],
                ]
    return eroded


def rasterize_safe_floor(
    polygons: list[list[tuple[float, float]]],
    *,
    resolution: float,
    safety_margin_m: float,
    outer_padding_m: float,
) -> tuple[np.ndarray, float, float]:
    """Rasterize support polygons and shrink them away from every cliff edge."""
    if not polygons:
        raise ValueError('At least one support polygon is required')
    if resolution <= 0.0:
        raise ValueError('resolution must be positive')
    if safety_margin_m < 0.0:
        raise ValueError('safety_margin_m must be non-negative')
    if outer_padding_m < 0.0:
        raise ValueError('outer_padding_m must be non-negative')

    all_points = [point for polygon in polygons for point in polygon]
    min_x = min(point[0] for point in all_points)
    max_x = max(point[0] for point in all_points)
    min_y = min(point[1] for point in all_points)
    max_y = max(point[1] for point in all_points)

    origin_x = (
        math.floor((min_x - outer_padding_m) / resolution) * resolution
    )
    origin_y = (
        math.floor((min_y - outer_padding_m) / resolution) * resolution
    )
    limit_x = math.ceil((max_x + outer_padding_m) / resolution) * resolution
    limit_y = math.ceil((max_y + outer_padding_m) / resolution) * resolution
    width = int(round((limit_x - origin_x) / resolution))
    height = int(round((limit_y - origin_y) / resolution))
    safe = np.zeros((height, width), dtype=bool)

    for polygon in polygons:
        polygon_min_x = min(point[0] for point in polygon)
        polygon_max_x = max(point[0] for point in polygon)
        polygon_min_y = min(point[1] for point in polygon)
        polygon_max_y = max(point[1] for point in polygon)

        x0 = max(
            0,
            int(math.floor((polygon_min_x - origin_x) / resolution)) - 1,
        )
        x1 = min(
            width,
            int(math.ceil((polygon_max_x - origin_x) / resolution)) + 1,
        )
        y0 = max(
            0,
            int(math.floor((polygon_min_y - origin_y) / resolution)) - 1,
        )
        y1 = min(
            height,
            int(math.ceil((polygon_max_y - origin_y) / resolution)) + 1,
        )
        if x0 >= x1 or y0 >= y1:
            continue

        xs = origin_x + (np.arange(x0, x1) + 0.5) * resolution
        ys = origin_y + (np.arange(y0, y1) + 0.5) * resolution
        safe[y0:y1, x0:x1] |= _points_inside_polygon(
            polygon,
            xs,
            ys,
        )

    margin_cells = int(math.ceil(safety_margin_m / resolution))
    return _erode_safe_mask(safe, margin_cells), origin_x, origin_y


def _write_pgm(path: Path, safe: np.ndarray) -> None:
    # Nav2 map_server uses black as occupancy 100 and white as occupancy 0 when
    # negate=0. Flip vertically because OccupancyGrid origin is bottom-left.
    pixels = np.where(safe, 254, 0).astype(np.uint8)
    image = np.flipud(pixels)
    with path.open('wb') as stream:
        stream.write(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode())
        stream.write(image.tobytes())


def generate_hospital_keepout_mask(
    world_path: Path,
    models_dir: Path,
    output_dir: Path,
    *,
    resolution: float = 0.05,
    safety_margin_m: float = 0.40,
    outer_padding_m: float = 2.0,
    start_x: float = DEFAULT_START_X,
    start_y: float = DEFAULT_START_Y,
    start_yaw: float = DEFAULT_START_YAW,
    max_support_height_m: float = 0.03,
) -> KeepoutMask:
    """Generate PGM/YAML where every non-safe floor cell is a keepout cell."""
    polygons = hospital_support_polygons(
        world_path,
        models_dir,
        start_x=start_x,
        start_y=start_y,
        start_yaw=start_yaw,
        max_support_height_m=max_support_height_m,
    )
    safe, origin_x, origin_y = rasterize_safe_floor(
        polygons,
        resolution=resolution,
        safety_margin_m=safety_margin_m,
        outer_padding_m=outer_padding_m,
    )

    output_dir.mkdir(parents=True, exist_ok=True)
    image_path = output_dir / 'hospital_keepout.pgm'
    yaml_path = output_dir / 'hospital_keepout.yaml'
    _write_pgm(image_path, safe)
    yaml_path.write_text(
        yaml.safe_dump(
            {
                'image': image_path.name,
                'mode': 'trinary',
                'resolution': float(resolution),
                'origin': [float(origin_x), float(origin_y), 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
            },
            sort_keys=False,
        ),
        encoding='utf-8',
    )

    return KeepoutMask(
        image_path=image_path,
        yaml_path=yaml_path,
        origin_x=origin_x,
        origin_y=origin_y,
        resolution=resolution,
        width=int(safe.shape[1]),
        height=int(safe.shape[0]),
        safe_cells=int(np.count_nonzero(safe)),
    )


def _default_world_path() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / 'share'
        / 'frontier_exploration'
        / 'worlds'
        / 'hospital_aws.sdf'
    )


def _installed_world_path() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory('frontier_exploration'))
            / 'worlds'
            / 'hospital_aws.sdf'
        )
    except Exception:  # noqa: BLE001
        return _default_world_path()


def _default_models_dir() -> Path:
    return (
        Path.home()
        / '.cache'
        / 'turtlebot4_project'
        / 'hospital_world'
        / 'models'
    )


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Generate a Nav2 keepout mask directly from the AWS Hospital floor '
            'collision mesh, with a safety margin around unsupported edges.'
        )
    )
    parser.add_argument('--world', type=Path, default=_installed_world_path())
    parser.add_argument('--models-dir', type=Path, default=_default_models_dir())
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--safety-margin', type=float, default=0.40)
    parser.add_argument('--outer-padding', type=float, default=2.0)
    parser.add_argument('--start-x', type=float, default=DEFAULT_START_X)
    parser.add_argument('--start-y', type=float, default=DEFAULT_START_Y)
    parser.add_argument('--start-yaw', type=float, default=DEFAULT_START_YAW)
    parsed = parser.parse_args(args)

    result = generate_hospital_keepout_mask(
        parsed.world.expanduser(),
        parsed.models_dir.expanduser(),
        parsed.output_dir.expanduser(),
        resolution=parsed.resolution,
        safety_margin_m=parsed.safety_margin,
        outer_padding_m=parsed.outer_padding,
        start_x=parsed.start_x,
        start_y=parsed.start_y,
        start_yaw=parsed.start_yaw,
    )
    print(f'Keepout mask: {result.yaml_path}')
    print(
        f'  size={result.width}x{result.height}, '
        f'resolution={result.resolution:.3f} m/cell'
    )
    print(
        f'  origin=({result.origin_x:.3f}, {result.origin_y:.3f}), '
        f'safe_cells={result.safe_cells}'
    )


if __name__ == '__main__':
    main()
