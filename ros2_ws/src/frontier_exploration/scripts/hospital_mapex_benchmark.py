#!/usr/bin/env python3
"""Validated entry point for the Hospital MapEx benchmark.

The heavy benchmark implementation is kept in
``hospital_mapex_benchmark_legacy.py``.  This wrapper fixes Hospital-specific
structural-GT details while leaving the MapEx exploration/metric pipeline alone:

* normalize the AWS Hospital COLLADA mesh to Gazebo metres / Z-up;
* deduplicate raw plane/triangle intersections before rasterization;
* robustly parse optional flat-world elevator blockers for the benchmark map;
* validate alignment on a one-cell 0.10 m wall raster, matching the Hospital
  report's structural wall-mesh reference rather than full simulator collision.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

HERE = Path(__file__).resolve().parent
LEGACY_PATH = HERE / "hospital_mapex_benchmark_legacy.py"

_spec = importlib.util.spec_from_file_location(
    "hospital_mapex_benchmark_legacy", LEGACY_PATH
)
if _spec is None or _spec.loader is None:
    raise RuntimeError(f"Could not load legacy benchmark: {LEGACY_PATH}")
legacy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(legacy)


# ---------------------------------------------------------------------------
# AWS Hospital COLLADA normalization + unique section segments
# ---------------------------------------------------------------------------
def _local_xml_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _collada_asset_metadata(mesh_path: Path) -> Tuple[float, str]:
    root = ET.parse(mesh_path).getroot()
    unit_scale = 1.0
    up_axis = "Z_UP"
    asset = next(
        (node for node in root if _local_xml_name(node.tag) == "asset"), None
    )
    if asset is not None:
        for child in asset:
            name = _local_xml_name(child.tag)
            if name == "unit":
                unit_scale = float(child.attrib.get("meter", "1.0"))
            elif name == "up_axis" and child.text:
                up_axis = child.text.strip().upper()
    if unit_scale <= 0.0:
        raise RuntimeError(f"Invalid COLLADA unit scale {unit_scale}: {mesh_path}")
    return unit_scale, up_axis


def _normalized_wall_mesh(mesh_path: Path):
    np = legacy.np
    trimesh = legacy.require_import(
        "trimesh", "python3 -m pip install trimesh pycollada"
    )
    loaded = trimesh.load(str(mesh_path), force="scene")

    if isinstance(loaded, trimesh.Scene):
        meshes = []
        for node_name in loaded.graph.nodes_geometry:
            transform, geom_name = loaded.graph[node_name]
            geom = loaded.geometry[geom_name].copy()
            geom.apply_transform(transform)
            meshes.append(geom)
        if not meshes:
            raise RuntimeError("Hospital wall DAE contains no geometry.")
        mesh = trimesh.util.concatenate(meshes)
    else:
        mesh = loaded.copy()

    unit_scale, up_axis = _collada_asset_metadata(mesh_path)
    extents = np.asarray(mesh.extents, dtype=np.float64)
    if extents.shape != (3,) or not np.all(np.isfinite(extents)):
        raise RuntimeError(f"Invalid Hospital wall mesh extents: {extents}")

    applied_scale = False
    if unit_scale != 1.0 and float(np.max(extents)) > 200.0:
        mesh.apply_scale(unit_scale)
        extents = np.asarray(mesh.extents, dtype=np.float64)
        applied_scale = True

    applied_axis = False
    if up_axis == "Y_UP" and extents[1] < 0.5 * extents[2]:
        # COLLADA (X,Y,Z) -> Gazebo (X,-Z,Y)
        transform = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        mesh.apply_transform(transform)
        extents = np.asarray(mesh.extents, dtype=np.float64)
        applied_axis = True
    elif up_axis not in ("Y_UP", "Z_UP"):
        raise RuntimeError(f"Unsupported COLLADA up_axis {up_axis!r}: {mesh_path}")

    if float(np.max(extents[:2])) > 100.0 or float(extents[2]) > 10.0:
        raise RuntimeError(
            "Hospital wall mesh normalization produced implausible extents "
            f"{extents.tolist()} m (unit_scale={unit_scale}, up_axis={up_axis})."
        )

    legacy.log(
        "Normalized Hospital DAE: "
        f"unit_scale={'applied' if applied_scale else 'already metric'}, "
        f"Y_UP->Z_UP={'applied' if applied_axis else 'already applied'}, "
        f"extents_m={[round(float(v), 3) for v in extents]}"
    )
    return mesh, trimesh


def _deduplicate_segments(raw):
    """Remove duplicate undirected triangle/plane intersection segments.

    Adjacent/duplicated faces can emit the same geometric segment more than
    once.  The report's wall-segment count refers to unique geometric segments,
    not every duplicate emitted by ``mesh_plane``.
    """
    np = legacy.np
    unique = []
    seen = set()
    for segment in np.asarray(raw, dtype=np.float64).reshape((-1, 2, 3)):
        xy = segment[:, :2]
        if float(np.linalg.norm(xy[1] - xy[0])) < 1e-9:
            continue
        rounded = np.round(xy, decimals=6)
        a = (float(rounded[0, 0]), float(rounded[0, 1]))
        b = (float(rounded[1, 0]), float(rounded[1, 1]))
        key = (a, b) if a <= b else (b, a)
        if key in seen:
            continue
        seen.add(key)
        unique.append(xy.copy())
    return unique


def load_wall_section_polylines(mesh_path: Path, z_slice: float):
    np = legacy.np
    mesh, trimesh = _normalized_wall_mesh(mesh_path)
    origin = np.array([0.0, 0.0, z_slice], dtype=np.float64)
    normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    raw = trimesh.intersections.mesh_plane(
        mesh=mesh,
        plane_normal=normal,
        plane_origin=origin,
    )
    raw = np.asarray(raw, dtype=np.float64)
    if raw.size == 0:
        raise RuntimeError(f"No wall-mesh intersections found at z={z_slice:.3f} m.")

    raw_count = int(raw.reshape((-1, 2, 3)).shape[0])
    polylines = _deduplicate_segments(raw)
    section = mesh.section(plane_origin=origin, plane_normal=normal)
    entity_count = len(section.entities) if section is not None else 0
    segment_count = len(polylines)

    if raw_count != segment_count:
        legacy.log(
            f"Wall segment dedup: raw={raw_count}, unique={segment_count}"
        )
    return polylines, entity_count, segment_count


legacy.load_wall_section_polylines = load_wall_section_polylines


# ---------------------------------------------------------------------------
# Robust world parsing
# ---------------------------------------------------------------------------
def find_world_model_geometry(
    sdf_path: Path,
) -> Tuple[
    Tuple[float, float, float],
    List[Tuple[float, float, float, float, float]],
]:
    root = ET.parse(sdf_path).getroot()
    wall_pose: Optional[Tuple[float, float, float]] = None

    for include in root.findall(".//include"):
        uri = (include.findtext("uri") or "").strip()
        if uri.endswith("aws_robomaker_hospital_floor_01_walls"):
            x, y, _z, _r, _p, yaw = legacy.parse_pose(include.findtext("pose"))
            wall_pose = (x, y, yaw)
            break
    if wall_pose is None:
        raise RuntimeError(f"Could not find Hospital wall include in {sdf_path}")

    blockers: List[Tuple[float, float, float, float, float]] = []
    for model in root.findall(".//model"):
        name = model.attrib.get("name", "").lower()
        if not ("elevator" in name and "blocker" in name):
            continue
        x, y, _z, _r, _p, yaw = legacy.parse_pose(model.findtext("pose"))
        size_text = model.findtext(".//collision/geometry/box/size")
        if not size_text:
            continue
        size = [float(v) for v in size_text.split()]
        if len(size) >= 2:
            blockers.append((x, y, yaw, size[0], size[1]))
    return wall_pose, blockers


legacy.find_world_model_geometry = find_world_model_geometry


# ---------------------------------------------------------------------------
# Boundary-touch is diagnostic only
# ---------------------------------------------------------------------------
_original_log = legacy.log


def log(msg: str) -> None:
    if msg.startswith(
        "WARNING: connected free space touches the configured Hospital bounding box"
    ):
        _original_log(
            "NOTE: valid space touches the configured clipping bounds; "
            "this is not a failure by itself. Running structural-GT validation next."
        )
    else:
        _original_log(msg)


legacy.log = log


# ---------------------------------------------------------------------------
# Report-style structural wall validation
# ---------------------------------------------------------------------------
VALIDATION_DIR = (
    legacy.PROJECT_ROOT
    / "data"
    / "lama_runs"
    / "hospital_flat_lama_01_001"
    / "lama_eval_20"
    / "model_input"
)
VALIDATION_FRAMES = ("000014", "000632", "001415")
MIN_WALL_MATCH = 0.70
MAX_FREE_CONFLICT = 0.05
MATCH_TOLERANCE_PIXELS = 1
TARGET_RESOLUTION = 0.10


def _metric_polyline_to_lama_pixels(poly_slam, target_shape):
    """Map metric SLAM points to the exact 0.10 m LaMa image grid."""
    np = legacy.np
    raw_rc = legacy.metric_xy_to_raw_rc(poly_slam)
    reduced_rc = raw_rc // 2

    scaled_h = int(
        math.ceil(
            legacy.CANVAS_HEIGHT
            * legacy.CANVAS_RESOLUTION
            / TARGET_RESOLUTION
            - 1e-12
        )
    )
    scaled_w = int(
        math.ceil(
            legacy.CANVAS_WIDTH
            * legacy.CANVAS_RESOLUTION
            / TARGET_RESOLUTION
            - 1e-12
        )
    )
    target_h, target_w = target_shape
    if scaled_w > target_w or scaled_h > target_h:
        raise RuntimeError(
            f"0.10 m Hospital grid {(scaled_h, scaled_w)} exceeds "
            f"LaMa image {target_shape}."
        )
    pad_top = target_h - scaled_h

    rows = pad_top + (scaled_h - 1 - reduced_rc[:, 0])
    cols = reduced_rc[:, 1]
    return np.stack([rows, cols], axis=1).astype(np.int32)


def _build_report_wall_image(
    aws_root: Path,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
    z_slice: float,
    target_shape,
):
    """Rasterize wall mesh directly at one 0.10 m cell thickness.

    The Hospital report describes structural GT as a wall-collision-mesh
    reference.  Elevator blockers are additional flat-world collision geometry,
    so they are intentionally excluded from this validation image.
    """
    np = legacy.np
    cv2 = legacy.require_import("cv2", "python3 -m pip install opencv-python")
    mesh_path = (
        aws_root
        / "models"
        / "aws_robomaker_hospital_floor_01_walls"
        / "meshes"
        / "aws_robomaker_hospital_floor_01_walls_collision.dae"
    )
    if not mesh_path.exists():
        raise RuntimeError(f"Hospital collision mesh missing: {mesh_path}")

    (wall_x, wall_y, wall_yaw), _blockers = find_world_model_geometry(
        legacy.HOSPITAL_WORLD_SDF
    )
    polylines, _entity_count, _segment_count = load_wall_section_polylines(
        mesh_path, z_slice
    )

    wall = np.zeros(target_shape, dtype=np.uint8)
    h, w = target_shape
    for poly in polylines:
        poly_world = legacy.transform_xy(poly, wall_x, wall_y, wall_yaw)
        poly_slam = legacy.world_to_slam(poly_world, spawn_x, spawn_y, spawn_yaw)
        rc = _metric_polyline_to_lama_pixels(poly_slam, target_shape)
        a, b = rc[0], rc[-1]
        if not (
            (0 <= a[0] < h and 0 <= a[1] < w)
            or (0 <= b[0] < h and 0 <= b[1] < w)
        ):
            continue
        cv2.line(
            wall,
            (int(a[1]), int(a[0])),
            (int(b[1]), int(b[0])),
            color=255,
            thickness=1,
            lineType=cv2.LINE_8,
        )

    # Close one-cell corner/raster cracks without dilating every wall outward.
    kernel = np.ones((3, 3), dtype=np.uint8)
    wall = cv2.morphologyEx(wall, cv2.MORPH_CLOSE, kernel)
    return wall > 0


def _alignment_metrics(structural_occ, reference, cv2):
    np = legacy.np
    reference_occ = reference <= 64
    reference_free = reference >= 192
    reference_known = reference_occ | reference_free

    kernel = np.ones((3, 3), dtype=np.uint8)
    occ_near = cv2.dilate(
        reference_occ.astype(np.uint8),
        kernel,
        iterations=MATCH_TOLERANCE_PIXELS,
    ).astype(bool)
    known_near = cv2.dilate(
        reference_known.astype(np.uint8),
        kernel,
        iterations=MATCH_TOLERANCE_PIXELS,
    ).astype(bool)

    observed_gt_wall = structural_occ & known_near
    observed_gt_count = int(observed_gt_wall.sum())
    wall_match = (
        float((observed_gt_wall & occ_near).sum() / observed_gt_count)
        if observed_gt_count
        else float("nan")
    )

    free_count = int(reference_free.sum())
    free_conflict = (
        float((structural_occ & reference_free).sum() / free_count)
        if free_count
        else float("nan")
    )
    return wall_match, free_conflict, observed_gt_count, free_count


def validate_structural_gt(
    map_dir: Path,
    metadata: dict,
    *,
    aws_root: Path,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
    z_slice: float,
    wall_thickness_cells: int,
    include_flat_blockers: bool,
) -> dict:
    np = legacy.np
    cv2 = legacy.require_import("cv2", "python3 -m pip install opencv-python")

    references = []
    for frame in VALIDATION_FRAMES:
        path = VALIDATION_DIR / f"{frame}.png"
        if path.exists():
            image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if image is not None:
                references.append((frame, path, image))
    if not references:
        raise RuntimeError(
            f"No committed Hospital validation snapshots found in {VALIDATION_DIR}"
        )

    frame_results = []
    all_pass = True
    wall_cache = {}
    for frame, path, reference in references:
        shape = tuple(reference.shape)
        if shape not in wall_cache:
            wall_cache[shape] = _build_report_wall_image(
                aws_root=aws_root,
                spawn_x=spawn_x,
                spawn_y=spawn_y,
                spawn_yaw=spawn_yaw,
                z_slice=z_slice,
                target_shape=shape,
            )
        structural_occ = wall_cache[shape]
        wall_match, free_conflict, observed_gt_count, free_count = _alignment_metrics(
            structural_occ, reference, cv2
        )
        frame_pass = bool(
            np.isfinite(wall_match)
            and np.isfinite(free_conflict)
            and wall_match >= MIN_WALL_MATCH
            and free_conflict <= MAX_FREE_CONFLICT
        )
        all_pass = all_pass and frame_pass
        frame_results.append(
            {
                "frame": frame,
                "wall_match": wall_match,
                "free_space_conflict": free_conflict,
                "observed_gt_wall_cells": observed_gt_count,
                "observed_free_cells": free_count,
                "passed": frame_pass,
            }
        )
        legacy.log(
            f"GT validation {frame}: {'PASS' if frame_pass else 'FAIL'} | "
            f"wall_match={wall_match * 100:.1f}% | "
            f"free_conflict={free_conflict * 100:.2f}%"
        )

    # The benchmark map itself may optionally contain the flat-world blockers;
    # validation intentionally tests only the report's structural wall mesh.
    occ_raw = np.load(map_dir / "occ_map.npy")
    start_rc = legacy.metric_xy_to_raw_rc(
        np.array([[0.0, 0.0]], dtype=np.float64)
    )[0]
    sr, sc = int(start_rc[0]), int(start_rc[1])
    start_is_free = bool(
        0 <= sr < occ_raw.shape[0]
        and 0 <= sc < occ_raw.shape[1]
        and occ_raw[sr, sc] > 127
    )
    all_pass = bool(all_pass and start_is_free)

    validation = {
        "passed": all_pass,
        "frames": frame_results,
        "start_is_free": start_is_free,
        "thresholds": {
            "min_wall_match": MIN_WALL_MATCH,
            "max_free_space_conflict": MAX_FREE_CONFLICT,
            "match_tolerance_pixels": MATCH_TOLERANCE_PIXELS,
        },
        "metric_definition": (
            "wall_match = matched structural wall cells / structural wall cells "
            "inside observed SLAM region; free_space_conflict = structural-wall "
            "and SLAM-free cells / observed SLAM-free cells"
        ),
        "validation_gt": (
            "wall collision mesh only, one-cell raster at 0.10 m; flat-world "
            "elevator blockers intentionally excluded to match the report"
        ),
        "benchmark_map_includes_flat_blockers": bool(include_flat_blockers),
        "benchmark_wall_thickness_cells_0p05m": int(wall_thickness_cells),
    }
    metadata["validation"] = validation
    (map_dir / "hospital_map_metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )

    legacy.log(
        f"GT validation summary: {'PASS' if all_pass else 'FAIL'} | "
        f"start_free={start_is_free}"
    )
    if not all_pass:
        raise RuntimeError(
            "Hospital structural GT failed report-style validation. "
            f"Need wall_match >= {MIN_WALL_MATCH * 100:.0f}% and "
            f"free_conflict <= {MAX_FREE_CONFLICT * 100:.0f}% on each "
            "committed validation frame. Do not run the benchmark yet."
        )
    return metadata


_original_build_hospital_map = legacy.build_hospital_map
_build_signature = inspect.signature(_original_build_hospital_map)


def build_hospital_map(*args, **kwargs):
    bound = _build_signature.bind(*args, **kwargs)
    params = bound.arguments
    map_dir, start_pose, metadata = _original_build_hospital_map(*args, **kwargs)
    metadata = validate_structural_gt(
        map_dir,
        metadata,
        aws_root=Path(params["aws_root"]),
        spawn_x=float(params["spawn_x"]),
        spawn_y=float(params["spawn_y"]),
        spawn_yaw=float(params["spawn_yaw"]),
        z_slice=float(params["z_slice"]),
        wall_thickness_cells=int(params["wall_thickness_cells"]),
        include_flat_blockers=bool(params["include_flat_blockers"]),
    )
    return map_dir, start_pose, metadata


legacy.build_hospital_map = build_hospital_map


if __name__ == "__main__":
    try:
        raise SystemExit(legacy.main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise
