#!/usr/bin/env python3
"""Generate canonical 1.0x Hospital structural GT and connected-free ROI.

The canonical Hospital collision wall is a COLLADA mesh. This generator slices
that mesh at a fixed physical height, rasterizes the resulting wall segments on
``hospital_canvas_v1``, adds the flat-world elevator blockers, then flood-fills
the free component containing the SLAM start pose.

Important coordinate contract for the AWS Hospital collision DAE:
- COLLADA asset units are honored (the checked-in mesh uses centimeters).
- Raw DAE X is Hospital world X.
- Raw DAE Z is Hospital world Y.
- Raw DAE Y is physical height.

The world is finally expressed in the ``slam_start_map`` frame by applying the
inverse of the canonical robot start pose (0, 12, -1.57). The generated wall
slice bounds are checked against the frozen Hospital ROI spec before artifacts
are accepted, which catches axis / scale / frame mistakes.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

MAPEX_LAB_ROOT = Path(__file__).resolve().parents[1]
SCRIPTS_ROOT = Path(__file__).resolve().parent
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

import hospital_scale  # noqa: E402

CANVAS_ID = "hospital_canvas_v1"
CANVAS_RES = 0.05
CANVAS_W = 1504
CANVAS_H = 2123
CANVAS_X = -25.6
CANVAS_Y = -60.1

GT_ID = "hospital_structural_gt_v1"
ROI_ID = "hospital_connected_free_v1"
WALL_MODEL_NAME = "aws_robomaker_hospital_floor_01_walls"
WALL_INCLUDE_NAME = "hospital_walls"

DEFAULT_WALL_SLICE_Z_M = 0.30
DEFAULT_WALL_THICKNESS_CELLS = 2
DEFAULT_CRACK_DILATION_CELLS = 1
DEFAULT_BOUNDS_TOLERANCE_M = 0.15

# Frozen roi_v1.yaml contract.
SPEC_BOUNDS = {
    "xmin": -0.572445,
    "xmax": 24.588833,
    "ymin": -35.091079,
    "ymax": 21.044604,
}
SPEC_ROI_CELLS = 215435
SPEC_ROI_SHA256 = "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _pose6(element: ET.Element) -> tuple[float, float, float, float, float, float]:
    text = element.findtext("pose", default="0 0 0 0 0 0")
    values = [float(value) for value in text.split()]
    if len(values) < 6:
        values.extend([0.0] * (6 - len(values)))
    if len(values) != 6:
        raise ValueError(f"invalid SDF pose: {text!r}")
    return tuple(values)  # type: ignore[return-value]


def _world_to_slam_xy(
    x: np.ndarray | float,
    y: np.ndarray | float,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
) -> tuple[np.ndarray | float, np.ndarray | float]:
    """Express world XY in the robot-start / SLAM-start frame."""
    dx = x - spawn_x
    dy = y - spawn_y
    c = math.cos(spawn_yaw)
    s = math.sin(spawn_yaw)
    map_x = c * dx + s * dy
    map_y = -s * dx + c * dy
    return map_x, map_y


def _cell_center_x(cols: np.ndarray) -> np.ndarray:
    return CANVAS_X + (cols.astype(np.float64) + 0.5) * CANVAS_RES


def _cell_center_y(rows: np.ndarray) -> np.ndarray:
    return CANVAS_Y + (rows.astype(np.float64) + 0.5) * CANVAS_RES


def _world_to_cell(x: float, y: float) -> tuple[int, int]:
    col = int(math.floor((x - CANVAS_X) / CANVAS_RES))
    row = int(math.floor((y - CANVAS_Y) / CANVAS_RES))
    return row, col


def _xml_namespace(root: ET.Element) -> str:
    if root.tag.startswith("{"):
        return root.tag[1:].split("}", 1)[0]
    return ""


def _q(ns: str, tag: str) -> str:
    return f"{{{ns}}}{tag}" if ns else tag


def _load_dae_polygons(dae_path: Path) -> tuple[list[np.ndarray], float, str]:
    """Return COLLADA polygons in raw DAE XYZ, plus metres-per-unit and up axis."""
    root = ET.parse(dae_path).getroot()
    ns = _xml_namespace(root)

    asset = root.find(_q(ns, "asset"))
    unit_m = 1.0
    up_axis = "Y_UP"
    if asset is not None:
        unit = asset.find(_q(ns, "unit"))
        if unit is not None and unit.get("meter"):
            unit_m = float(unit.get("meter"))
        up = asset.findtext(_q(ns, "up_axis"))
        if up:
            up_axis = up.strip()

    polygons: list[np.ndarray] = []
    geometries = root.find(_q(ns, "library_geometries"))
    if geometries is None:
        raise ValueError(f"COLLADA has no library_geometries: {dae_path}")

    for geometry in geometries.findall(_q(ns, "geometry")):
        mesh = geometry.find(_q(ns, "mesh"))
        if mesh is None:
            continue

        sources: dict[str, np.ndarray] = {}
        for source in mesh.findall(_q(ns, "source")):
            sid = source.get("id")
            floats = source.find(_q(ns, "float_array"))
            accessor = source.find(f"{_q(ns, 'technique_common')}/{_q(ns, 'accessor')}")
            if not sid or floats is None or not floats.text or accessor is None:
                continue
            stride = int(accessor.get("stride", "1"))
            values = np.fromstring(floats.text, sep=" ", dtype=np.float64)
            if stride <= 0 or values.size % stride:
                raise ValueError(f"invalid COLLADA source {sid!r} in {dae_path}")
            sources[sid] = values.reshape(-1, stride)

        vertices_to_positions: dict[str, str] = {}
        for vertices in mesh.findall(_q(ns, "vertices")):
            vid = vertices.get("id")
            if not vid:
                continue
            for inp in vertices.findall(_q(ns, "input")):
                if inp.get("semantic") == "POSITION" and inp.get("source"):
                    vertices_to_positions[vid] = inp.get("source").lstrip("#")

        for primitive in list(mesh):
            kind = primitive.tag.split("}")[-1]
            if kind not in {"polylist", "triangles", "polygons"}:
                continue

            inputs = primitive.findall(_q(ns, "input"))
            if not inputs:
                continue
            max_offset = max(int(item.get("offset", "0")) for item in inputs)
            input_stride = max_offset + 1
            vertex_input = next(
                (item for item in inputs if item.get("semantic") == "VERTEX"),
                None,
            )
            if vertex_input is None or not vertex_input.get("source"):
                continue
            vertex_offset = int(vertex_input.get("offset", "0"))
            vertices_id = vertex_input.get("source").lstrip("#")
            position_source = vertices_to_positions.get(vertices_id)
            if position_source is None or position_source not in sources:
                raise ValueError(
                    f"missing POSITION source for COLLADA vertices {vertices_id!r}"
                )
            positions = sources[position_source]
            if positions.shape[1] < 3:
                raise ValueError(f"POSITION source {position_source!r} has stride < 3")

            def add_polygon(flat: np.ndarray, nverts: int) -> None:
                if flat.size != nverts * input_stride:
                    raise ValueError("COLLADA primitive index length mismatch")
                records = flat.reshape(nverts, input_stride)
                indices = records[:, vertex_offset]
                if np.any(indices < 0) or np.any(indices >= positions.shape[0]):
                    raise ValueError("COLLADA vertex index out of range")
                polygons.append(np.asarray(positions[indices, :3], dtype=np.float64))

            if kind == "polylist":
                vcount_text = primitive.findtext(_q(ns, "vcount"), default="")
                p_text = primitive.findtext(_q(ns, "p"), default="")
                vcounts = np.fromstring(vcount_text, sep=" ", dtype=np.int64)
                flat = np.fromstring(p_text, sep=" ", dtype=np.int64)
                cursor = 0
                for count in vcounts.tolist():
                    take = int(count) * input_stride
                    add_polygon(flat[cursor : cursor + take], int(count))
                    cursor += take
                if cursor != flat.size:
                    raise ValueError("COLLADA polylist has trailing indices")
            elif kind == "triangles":
                p_text = primitive.findtext(_q(ns, "p"), default="")
                flat = np.fromstring(p_text, sep=" ", dtype=np.int64)
                block = 3 * input_stride
                if block == 0 or flat.size % block:
                    raise ValueError("COLLADA triangle index length mismatch")
                for cursor in range(0, flat.size, block):
                    add_polygon(flat[cursor : cursor + block], 3)
            else:
                for p in primitive.findall(_q(ns, "p")):
                    flat = np.fromstring(p.text or "", sep=" ", dtype=np.int64)
                    if flat.size % input_stride:
                        raise ValueError("COLLADA polygon index length mismatch")
                    add_polygon(flat, flat.size // input_stride)

    if not polygons:
        raise ValueError(f"no polygons parsed from {dae_path}")
    return polygons, unit_m, up_axis


def _find_walls_include(world_path: Path) -> tuple[Path, tuple[float, ...], Path]:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"no <world> in {world_path}")

    target = None
    for include in world.findall("include"):
        name = include.findtext("name", default="").strip()
        uri = include.findtext("uri", default="").strip()
        if name == WALL_INCLUDE_NAME or uri == f"model://{WALL_MODEL_NAME}":
            target = include
            break
    if target is None:
        raise ValueError(f"Hospital wall include not found in {world_path}")

    uri = target.findtext("uri", default="").strip()
    if not uri.startswith("model://"):
        raise ValueError(f"Hospital wall URI must be model://, got {uri!r}")
    model_name = uri.removeprefix("model://").split("/", 1)[0]
    if model_name != WALL_MODEL_NAME:
        raise ValueError(f"unexpected Hospital wall model: {model_name!r}")

    models_root = MAPEX_LAB_ROOT / "map" / "models"
    model_sdf = models_root / model_name / "model.sdf"
    if not model_sdf.is_file():
        raise FileNotFoundError(f"missing Hospital wall model: {model_sdf}")

    model_root = ET.parse(model_sdf).getroot()
    model = model_root.find("model")
    if model is None:
        raise ValueError(f"no <model> in {model_sdf}")
    collision = model.find("link/collision")
    if collision is None:
        raise ValueError(f"no wall collision in {model_sdf}")
    mesh = collision.find("geometry/mesh")
    if mesh is None:
        raise ValueError(f"wall collision is not a mesh in {model_sdf}")
    mesh_uri = mesh.findtext("uri", default="").strip()
    if not mesh_uri.startswith(f"model://{model_name}/"):
        raise ValueError(f"unexpected wall collision URI: {mesh_uri!r}")
    relative = mesh_uri[len(f"model://{model_name}/") :]
    dae_path = models_root / model_name / relative
    if not dae_path.is_file():
        raise FileNotFoundError(f"missing wall collision DAE: {dae_path}")

    return dae_path, _pose6(target), model_sdf


def _dedupe_points(points: list[np.ndarray], tol: float = 1e-8) -> list[np.ndarray]:
    out: list[np.ndarray] = []
    for point in points:
        if not any(np.linalg.norm(point - existing) <= tol for existing in out):
            out.append(point)
    return out


def _slice_polygon(
    polygon_xyz: np.ndarray,
    z_slice_m: float,
    tol: float = 1e-9,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Intersect one 3D polygon with horizontal z=z_slice_m."""
    points: list[np.ndarray] = []
    n = polygon_xyz.shape[0]
    for i in range(n):
        a = polygon_xyz[i]
        b = polygon_xyz[(i + 1) % n]
        da = float(a[2] - z_slice_m)
        db = float(b[2] - z_slice_m)
        if abs(da) <= tol:
            points.append(a[:2].copy())
        if da * db < -tol * tol:
            t = (z_slice_m - float(a[2])) / float(b[2] - a[2])
            points.append(a[:2] + t * (b[:2] - a[:2]))
        elif abs(da) <= tol and abs(db) <= tol:
            points.append(b[:2].copy())

    points = _dedupe_points(points)
    if len(points) < 2:
        return []
    if len(points) == 2:
        return [(points[0], points[1])]

    center = np.mean(np.asarray(points), axis=0)
    ordered = sorted(points, key=lambda p: math.atan2(p[1] - center[1], p[0] - center[0]))
    return [
        (ordered[i], ordered[(i + 1) % len(ordered)])
        for i in range(len(ordered))
    ]


def _wall_slice_segments(
    dae_path: Path,
    include_pose: tuple[float, ...],
    z_slice_m: float,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
) -> tuple[list[tuple[np.ndarray, np.ndarray]], dict]:
    polygons, unit_m, up_axis = _load_dae_polygons(dae_path)
    inc_x, inc_y, inc_z, inc_roll, inc_pitch, inc_yaw = include_pose
    if abs(inc_roll) > 1e-9 or abs(inc_pitch) > 1e-9:
        raise ValueError("Hospital wall include must stay planar")

    c = math.cos(inc_yaw)
    s = math.sin(inc_yaw)
    segments: list[tuple[np.ndarray, np.ndarray]] = []

    for raw in polygons:
        local_x = raw[:, 0] * unit_m
        local_y = raw[:, 2] * unit_m
        local_z = raw[:, 1] * unit_m
        world_x = inc_x + c * local_x - s * local_y
        world_y = inc_y + s * local_x + c * local_y
        world_z = inc_z + local_z
        world = np.column_stack((world_x, world_y, world_z))

        for a_world, b_world in _slice_polygon(world, z_slice_m):
            ax, ay = _world_to_slam_xy(
                float(a_world[0]), float(a_world[1]), spawn_x, spawn_y, spawn_yaw
            )
            bx, by = _world_to_slam_xy(
                float(b_world[0]), float(b_world[1]), spawn_x, spawn_y, spawn_yaw
            )
            a = np.asarray([ax, ay], dtype=np.float64)
            b = np.asarray([bx, by], dtype=np.float64)
            if np.linalg.norm(a - b) > 1e-7:
                segments.append((a, b))

    if not segments:
        raise ValueError(f"wall mesh has no slice segments at z={z_slice_m:.3f} m")

    endpoints = np.concatenate(
        [np.stack((a, b), axis=0) for a, b in segments], axis=0
    )
    bounds = {
        "xmin": float(np.min(endpoints[:, 0])),
        "xmax": float(np.max(endpoints[:, 0])),
        "ymin": float(np.min(endpoints[:, 1])),
        "ymax": float(np.max(endpoints[:, 1])),
    }
    return segments, {
        "dae_unit_meter": unit_m,
        "dae_up_axis": up_axis,
        "polygon_count": len(polygons),
        "slice_segment_count": len(segments),
        "slice_bounds_slam_start_m": bounds,
    }


def _check_bounds(actual: dict, tolerance_m: float) -> dict:
    errors = {key: float(actual[key] - SPEC_BOUNDS[key]) for key in SPEC_BOUNDS}
    max_abs_error = max(abs(value) for value in errors.values())
    if max_abs_error > tolerance_m:
        raise ValueError(
            "Hospital wall slice does not match roi_v1 bounds; likely scale/axis/frame "
            f"mismatch. actual={actual}, expected={SPEC_BOUNDS}, errors={errors}, "
            f"tolerance={tolerance_m}"
        )
    return {"errors_m": errors, "max_abs_error_m": max_abs_error}


def _rasterize_segments(segments: list[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    mask = np.zeros((CANVAS_H, CANVAS_W), dtype=bool)
    step = CANVAS_RES / 3.0
    for a, b in segments:
        delta = b - a
        length = float(np.linalg.norm(delta))
        count = max(2, int(math.ceil(length / step)) + 1)
        t = np.linspace(0.0, 1.0, count)
        xs = a[0] + t * delta[0]
        ys = a[1] + t * delta[1]
        cols = np.floor((xs - CANVAS_X) / CANVAS_RES).astype(np.int64)
        rows = np.floor((ys - CANVAS_Y) / CANVAS_RES).astype(np.int64)
        valid = (
            (rows >= 0)
            & (rows < CANVAS_H)
            & (cols >= 0)
            & (cols < CANVAS_W)
        )
        mask[rows[valid], cols[valid]] = True
    return mask


def _dilate(mask: np.ndarray, iterations: int) -> np.ndarray:
    out = mask.astype(bool, copy=True)
    for _ in range(max(0, iterations)):
        src = out
        dst = src.copy()
        h, w = src.shape
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                r_src0 = max(0, -dr)
                r_src1 = min(h, h - dr)
                c_src0 = max(0, -dc)
                c_src1 = min(w, w - dc)
                r_dst0 = r_src0 + dr
                r_dst1 = r_src1 + dr
                c_dst0 = c_src0 + dc
                c_dst1 = c_src1 + dc
                dst[r_dst0:r_dst1, c_dst0:c_dst1] |= src[
                    r_src0:r_src1, c_src0:c_src1
                ]
        out = dst
    return out


def _box_corners_map(
    x_world: float,
    y_world: float,
    yaw_world: float,
    sx: float,
    sy: float,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
) -> np.ndarray:
    local = np.asarray(
        [
            [-sx / 2.0, -sy / 2.0],
            [-sx / 2.0, sy / 2.0],
            [sx / 2.0, sy / 2.0],
            [sx / 2.0, -sy / 2.0],
        ],
        dtype=np.float64,
    )
    c = math.cos(yaw_world)
    s = math.sin(yaw_world)
    rot = np.asarray([[c, -s], [s, c]], dtype=np.float64)
    world = local @ rot.T + np.asarray([x_world, y_world], dtype=np.float64)
    mx, my = _world_to_slam_xy(
        world[:, 0], world[:, 1], spawn_x, spawn_y, spawn_yaw
    )
    return np.column_stack((mx, my))


def _point_in_convex_polygon(xx: np.ndarray, yy: np.ndarray, poly: np.ndarray) -> np.ndarray:
    eps = 1e-10
    has_pos = np.zeros(xx.shape, dtype=bool)
    has_neg = np.zeros(xx.shape, dtype=bool)
    for i in range(len(poly)):
        a = poly[i]
        b = poly[(i + 1) % len(poly)]
        cross = (b[0] - a[0]) * (yy - a[1]) - (b[1] - a[1]) * (xx - a[0])
        has_pos |= cross > eps
        has_neg |= cross < -eps
    return ~(has_pos & has_neg)


def _rasterize_polygon(mask: np.ndarray, polygon: np.ndarray) -> None:
    xmin, ymin = np.min(polygon, axis=0)
    xmax, ymax = np.max(polygon, axis=0)
    c0 = max(0, int(math.floor((xmin - CANVAS_X) / CANVAS_RES)) - 1)
    c1 = min(CANVAS_W - 1, int(math.floor((xmax - CANVAS_X) / CANVAS_RES)) + 1)
    r0 = max(0, int(math.floor((ymin - CANVAS_Y) / CANVAS_RES)) - 1)
    r1 = min(CANVAS_H - 1, int(math.floor((ymax - CANVAS_Y) / CANVAS_RES)) + 1)
    if c1 < c0 or r1 < r0:
        return
    cols = np.arange(c0, c1 + 1, dtype=np.int32)
    rows = np.arange(r0, r1 + 1, dtype=np.int32)
    xx, yy = np.meshgrid(_cell_center_x(cols), _cell_center_y(rows))
    inside = _point_in_convex_polygon(xx, yy, polygon)
    mask[r0 : r1 + 1, c0 : c1 + 1] |= inside


def _rasterize_elevator_blockers(
    world_path: Path,
    z_slice_m: float,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
) -> tuple[np.ndarray, list[dict]]:
    root = ET.parse(world_path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"no <world> in {world_path}")
    mask = np.zeros((CANVAS_H, CANVAS_W), dtype=bool)
    metadata: list[dict] = []

    for model in world.findall("model"):
        name = model.get("name", "")
        if "elevator_blocker" not in name:
            continue
        mx, my, mz, mroll, mpitch, myaw = _pose6(model)
        if abs(mroll) > 1e-9 or abs(mpitch) > 1e-9:
            raise ValueError(f"blocker {name!r} is not planar")
        for link in model.findall("link"):
            lx, ly, lz, lroll, lpitch, lyaw = _pose6(link)
            if any(abs(value) > 1e-9 for value in (lroll, lpitch, lyaw)):
                raise ValueError(f"blocker link transform unsupported for {name!r}")
            for collision in link.findall("collision"):
                box = collision.find("geometry/box")
                if box is None:
                    continue
                size_text = box.findtext("size", default="")
                size = [float(v) for v in size_text.split()]
                if len(size) != 3:
                    raise ValueError(f"invalid blocker size in {name!r}")
                cx, cy, cz, croll, cpitch, cyaw = _pose6(collision)
                if any(abs(value) > 1e-9 for value in (croll, cpitch, cyaw)):
                    raise ValueError(f"blocker collision transform unsupported for {name!r}")
                x = mx + lx + cx
                y = my + ly + cy
                z = mz + lz + cz
                sx, sy, sz = size
                if not (z - sz / 2.0 - 1e-9 <= z_slice_m <= z + sz / 2.0 + 1e-9):
                    continue
                polygon = _box_corners_map(
                    x, y, myaw, sx, sy, spawn_x, spawn_y, spawn_yaw
                )
                _rasterize_polygon(mask, polygon)
                metadata.append(
                    {
                        "model": name,
                        "collision": collision.get("name", ""),
                        "world_xy": [x, y],
                        "world_yaw": myaw,
                        "size_xyz": size,
                    }
                )
    if not metadata:
        raise ValueError("no elevator_blocker collisions found in Hospital world")
    return mask, metadata


def _bounds_mask() -> np.ndarray:
    cols = np.arange(CANVAS_W, dtype=np.int32)
    rows = np.arange(CANVAS_H, dtype=np.int32)
    xs = _cell_center_x(cols)
    ys = _cell_center_y(rows)
    cmask = (xs >= SPEC_BOUNDS["xmin"]) & (xs <= SPEC_BOUNDS["xmax"])
    rmask = (ys >= SPEC_BOUNDS["ymin"]) & (ys <= SPEC_BOUNDS["ymax"])
    return rmask[:, None] & cmask[None, :]


def _connected_free_8(free: np.ndarray, start_xy=(0.0, 0.0)) -> tuple[np.ndarray, tuple[int, int]]:
    start = _world_to_cell(float(start_xy[0]), float(start_xy[1]))
    row, col = start
    if not (0 <= row < free.shape[0] and 0 <= col < free.shape[1]):
        raise ValueError("SLAM start is outside canonical canvas")
    if not free[start]:
        raise ValueError("SLAM start (0,0) is occupied in generated Hospital GT")

    connected = np.zeros(free.shape, dtype=bool)
    connected[start] = True
    q = deque([start])
    h, w = free.shape
    neighbors = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),             (0, 1),
        (1, -1),  (1, 0),   (1, 1),
    ]
    while q:
        r, c = q.popleft()
        for dr, dc in neighbors:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and free[nr, nc] and not connected[nr, nc]:
                connected[nr, nc] = True
                q.append((nr, nc))
    return connected, start


def _write_pgm(path: Path, data: np.ndarray) -> None:
    image = np.full(data.shape, 205, dtype=np.uint8)
    image[data == 0] = 254
    image[data > 50] = 0
    with path.open("wb") as stream:
        stream.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii"))
        stream.write(np.flipud(image).tobytes())


def generate(
    output_dir: Path,
    wall_slice_z_m: float = DEFAULT_WALL_SLICE_Z_M,
    wall_thickness_cells: int = DEFAULT_WALL_THICKNESS_CELLS,
    crack_dilation_cells: int = DEFAULT_CRACK_DILATION_CELLS,
    bounds_tolerance_m: float = DEFAULT_BOUNDS_TOLERANCE_M,
    allow_spec_mismatch: bool = False,
) -> dict:
    hospital = hospital_scale.prepare_scaled_hospital()
    if not math.isclose(hospital.scale, 1.0, abs_tol=1e-12):
        raise ValueError(
            f"Hospital GT is canonical 1.0x only; HOSPITAL_SCALE={hospital.scale}"
        )

    world_path = hospital.world.resolve()
    dae_path, include_pose, model_sdf = _find_walls_include(world_path)
    segments, mesh_meta = _wall_slice_segments(
        dae_path,
        include_pose,
        wall_slice_z_m,
        hospital.spawn_x,
        hospital.spawn_y,
        hospital.spawn_yaw,
    )
    bounds_check = _check_bounds(
        mesh_meta["slice_bounds_slam_start_m"], bounds_tolerance_m
    )

    wall_lines = _rasterize_segments(segments)
    thickness_iterations = max(0, int(wall_thickness_cells) // 2)
    walls = _dilate(wall_lines, thickness_iterations)

    blockers, blocker_meta = _rasterize_elevator_blockers(
        world_path,
        wall_slice_z_m,
        hospital.spawn_x,
        hospital.spawn_y,
        hospital.spawn_yaw,
    )
    occupied = walls | blockers
    occupied = _dilate(occupied, crack_dilation_cells)

    bounds_mask = _bounds_mask()
    occupied &= bounds_mask
    free_candidate = bounds_mask & ~occupied
    roi, start_cell = _connected_free_8(free_candidate, start_xy=(0.0, 0.0))

    evaluation_mask = roi | occupied
    data = np.full((CANVAS_H, CANVAS_W), -1, dtype=np.int16)
    data[roi] = 0
    data[occupied & evaluation_mask] = 100

    output_dir.mkdir(parents=True, exist_ok=True)
    gt_path = output_dir / f"{GT_ID}.npz"
    roi_path = output_dir / f"{ROI_ID}.npy"
    roi_meta_path = output_dir / f"{ROI_ID}_metadata.json"
    summary_path = output_dir / f"{GT_ID}_summary.json"
    preview_path = output_dir / f"{GT_ID}.pgm"

    np.save(roi_path, roi)
    roi_sha = _sha256(roi_path)
    roi_cells = int(np.count_nonzero(roi))
    spec_match = roi_cells == SPEC_ROI_CELLS and roi_sha == SPEC_ROI_SHA256

    np.savez_compressed(
        gt_path,
        data=data,
        evaluation_mask=evaluation_mask,
        resolution=np.float64(CANVAS_RES),
        width=np.int64(CANVAS_W),
        height=np.int64(CANVAS_H),
        origin_x=np.float64(CANVAS_X),
        origin_y=np.float64(CANVAS_Y),
        canvas_id=np.asarray(CANVAS_ID),
        ground_truth_id=np.asarray(GT_ID),
        roi_id=np.asarray(ROI_ID),
        source_world=np.asarray(str(world_path)),
        source_world_sha256=np.asarray(_sha256(world_path)),
        source_wall_model_sdf=np.asarray(str(model_sdf)),
        source_wall_model_sdf_sha256=np.asarray(_sha256(model_sdf)),
        source_wall_mesh=np.asarray(str(dae_path)),
        source_wall_mesh_sha256=np.asarray(_sha256(dae_path)),
        hospital_scale=np.float64(hospital.scale),
        wall_slice_z_m=np.float64(wall_slice_z_m),
        wall_thickness_cells=np.int64(wall_thickness_cells),
        crack_dilation_cells=np.int64(crack_dilation_cells),
        spawn_world_x=np.float64(hospital.spawn_x),
        spawn_world_y=np.float64(hospital.spawn_y),
        spawn_world_yaw=np.float64(hospital.spawn_yaw),
    )
    _write_pgm(preview_path, data)

    summary = {
        "status": "ok" if spec_match else "generated_spec_mismatch",
        "ground_truth_id": GT_ID,
        "roi_id": ROI_ID,
        "canvas_id": CANVAS_ID,
        "hospital_scale": hospital.scale,
        "source_world": str(world_path),
        "source_world_sha256": _sha256(world_path),
        "source_wall_model_sdf": str(model_sdf),
        "source_wall_mesh": str(dae_path),
        "source_wall_mesh_sha256": _sha256(dae_path),
        "coordinate_contract": {
            "dae_raw_x": "hospital_world_x",
            "dae_raw_z": "hospital_world_y",
            "dae_raw_y": "physical_height",
            "world_to_slam_start": "inverse canonical robot start pose",
        },
        "spawn_world": {
            "x": hospital.spawn_x,
            "y": hospital.spawn_y,
            "yaw": hospital.spawn_yaw,
        },
        "spawn_slam_start_xy": [0.0, 0.0],
        "spawn_cell_row_col": [int(start_cell[0]), int(start_cell[1])],
        "wall_slice_z_m": wall_slice_z_m,
        "wall_thickness_cells": wall_thickness_cells,
        "crack_dilation_cells": crack_dilation_cells,
        "mesh": mesh_meta,
        "bounds_spec": SPEC_BOUNDS,
        "bounds_check": bounds_check,
        "elevator_blockers": blocker_meta,
        "excluded_dynamic_or_nonstructural_geometry": [
            "hospital_closed_curtain_left",
            "furniture/props not present in hospital_aws_flat.sdf structural wall source",
        ],
        "canvas": {
            "resolution_m": CANVAS_RES,
            "width_cells": CANVAS_W,
            "height_cells": CANVAS_H,
            "origin_x": CANVAS_X,
            "origin_y": CANVAS_Y,
        },
        "counts": {
            "bounds_cells": int(np.count_nonzero(bounds_mask)),
            "occupied_cells": int(np.count_nonzero(occupied)),
            "connected_free_roi_cells": roi_cells,
            "evaluation_mask_cells": int(np.count_nonzero(evaluation_mask)),
        },
        "canonical_roi_check": {
            "expected_cells": SPEC_ROI_CELLS,
            "actual_cells": roi_cells,
            "expected_sha256": SPEC_ROI_SHA256,
            "actual_sha256": roi_sha,
            "match": spec_match,
        },
        "outputs": {
            "ground_truth": str(gt_path),
            "connected_free_roi": str(roi_path),
            "roi_metadata": str(roi_meta_path),
            "preview_pgm": str(preview_path),
            "summary": str(summary_path),
        },
    }

    roi_metadata = {
        "id": ROI_ID,
        "canvas_id": CANVAS_ID,
        "hospital_scale": hospital.scale,
        "denominator_cells": roi_cells,
        "mask_sha256": roi_sha,
        "canonical_spec_match": spec_match,
        "source_wall_mesh_sha256": _sha256(dae_path),
        "wall_slice_z_m": wall_slice_z_m,
        "wall_thickness_cells": wall_thickness_cells,
        "close_raster_cracks_dilation_cells": crack_dilation_cells,
        "connectivity": 8,
        "seed_xy_in_slam_start_m": [0.0, 0.0],
    }
    roi_meta_path.write_text(json.dumps(roi_metadata, indent=2) + "\n", encoding="utf-8")
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    if not spec_match and not allow_spec_mismatch:
        raise RuntimeError(
            "Generated Hospital ROI does not match frozen roi_v1.yaml. "
            f"Expected {SPEC_ROI_CELLS} cells / {SPEC_ROI_SHA256}, got "
            f"{roi_cells} cells / {roi_sha}. Artifacts were written for inspection, "
            "but must not be used for official evaluation yet."
        )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate canonical 1.0x Hospital structural GT + connected ROI"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=MAPEX_LAB_ROOT / "ground_truth" / "hospital" / "generated",
    )
    parser.add_argument("--wall-slice-z-m", type=float, default=DEFAULT_WALL_SLICE_Z_M)
    parser.add_argument(
        "--wall-thickness-cells", type=int, default=DEFAULT_WALL_THICKNESS_CELLS
    )
    parser.add_argument(
        "--crack-dilation-cells", type=int, default=DEFAULT_CRACK_DILATION_CELLS
    )
    parser.add_argument(
        "--bounds-tolerance-m", type=float, default=DEFAULT_BOUNDS_TOLERANCE_M
    )
    parser.add_argument(
        "--allow-spec-mismatch",
        action="store_true",
        help="write/return candidate artifacts even when frozen ROI cells/SHA differ",
    )
    args = parser.parse_args()

    try:
        summary = generate(
            output_dir=args.output_dir.expanduser().resolve(),
            wall_slice_z_m=args.wall_slice_z_m,
            wall_thickness_cells=args.wall_thickness_cells,
            crack_dilation_cells=args.crack_dilation_cells,
            bounds_tolerance_m=args.bounds_tolerance_m,
            allow_spec_mismatch=args.allow_spec_mismatch,
        )
    except Exception as exc:
        print(json.dumps({"status": "error", "error": str(exc)}, indent=2))
        raise SystemExit(2) from exc
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
