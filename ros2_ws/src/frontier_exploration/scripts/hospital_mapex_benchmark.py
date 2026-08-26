#!/usr/bin/env python3
"""Validated entry point for the Hospital MapEx benchmark.

The full benchmark implementation lives in ``hospital_mapex_benchmark_legacy.py``.
This wrapper fixes Hospital-specific structural-GT details without changing the
upstream MapEx exploration or metric pipeline:

1. normalize the AWS Hospital COLLADA mesh to Gazebo metres / Z-up;
2. rasterize every raw plane/triangle intersection segment (the report's 934
   wall segments), instead of only ``Path.discrete`` chains;
3. robustly include the flat-world elevator blockers; and
4. validate alignment using the same interpretation as the Hospital report:
   wall match is measured on GT wall cells inside the observed SLAM region,
   while free-space conflict is the fraction of observed SLAM free cells that
   structural GT marks occupied.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
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
# AWS Hospital COLLADA normalization + raw section segments
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

    # Source DAE is Y_UP. pycollada/trimesh commonly applies the up-axis
    # transform already. Only rotate ourselves when Y is still clearly the
    # short/vertical axis.
    applied_axis = False
    if up_axis == "Y_UP" and extents[1] < 0.5 * extents[2]:
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


def load_wall_section_polylines(mesh_path: Path, z_slice: float):
    """Return every raw wall-plane intersection as a two-point polyline.

    ``mesh.section(...).discrete`` is unsuitable here: it traverses/merges path
    entities and can omit open chains. The Hospital report counted and
    rasterized raw face/plane intersection segments; on the expected asset this
    is 112 section entities and 934 raw wall segments at z=0.30 m.
    """
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
    raw = raw.reshape((-1, 2, 3))
    polylines = [segment[:, :2].copy() for segment in raw]

    section = mesh.section(plane_origin=origin, plane_normal=normal)
    entity_count = len(section.entities) if section is not None else 0
    segment_count = int(len(polylines))
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
# Structural-GT validation matching the report interpretation
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
MATCH_TOLERANCE_PIXELS = 1  # ~one 0.10 m cell, as described in the report.


def _build_explicit_wall_mask(
    aws_root: Path,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
    z_slice: float,
    wall_thickness_cells: int,
    include_flat_blockers: bool,
):
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

    (wall_x, wall_y, wall_yaw), blockers = find_world_model_geometry(
        legacy.HOSPITAL_WORLD_SDF
    )
    polylines, _entity_count, _segment_count = load_wall_section_polylines(
        mesh_path, z_slice
    )

    obstacle = np.zeros(
        (legacy.CANVAS_HEIGHT, legacy.CANVAS_WIDTH), dtype=np.uint8
    )
    for poly in polylines:
        poly_world = legacy.transform_xy(poly, wall_x, wall_y, wall_yaw)
        poly_slam = legacy.world_to_slam(poly_world, spawn_x, spawn_y, spawn_yaw)
        rc = legacy.metric_xy_to_raw_rc(poly_slam)
        legacy.rasterize_polyline(obstacle, rc, wall_thickness_cells, cv2)

    if include_flat_blockers:
        for x, y, yaw, sx, sy in blockers:
            corners_world = legacy.rectangle_corners(x, y, yaw, sx, sy)
            corners_slam = legacy.world_to_slam(
                corners_world, spawn_x, spawn_y, spawn_yaw
            )
            rc = legacy.metric_xy_to_raw_rc(corners_slam)
            polygon = np.stack([rc[:, 1], rc[:, 0]], axis=1).astype(np.int32)
            cv2.fillPoly(obstacle, [polygon], 255)

    kernel = np.ones((3, 3), dtype=np.uint8)
    return cv2.dilate(obstacle, kernel, iterations=1) > 0


def _wall_mask_as_lama_image(wall_raw, target_shape):
    np = legacy.np
    source_h, source_w = wall_raw.shape
    block = int(round(0.10 / legacy.CANVAS_RESOLUTION))
    if block != 2:
        raise RuntimeError(f"Unexpected Hospital downsample factor: {block}")

    pad_h = (-source_h) % block
    pad_w = (-source_w) % block
    padded = np.pad(
        wall_raw.astype(np.uint8),
        ((0, pad_h), (0, pad_w)),
        mode="constant",
        constant_values=0,
    )
    reduced = padded.reshape(
        padded.shape[0] // block,
        block,
        padded.shape[1] // block,
        block,
    ).max(axis=(1, 3)).astype(bool)

    top_down = reduced[::-1, :]
    target_h, target_w = target_shape
    if top_down.shape[1] > target_w or top_down.shape[0] > target_h:
        raise RuntimeError(
            f"Structural wall map {top_down.shape} is larger than reference {target_shape}."
        )
    output = np.zeros((target_h, target_w), dtype=bool)
    pad_top = target_h - top_down.shape[0]
    output[pad_top : pad_top + top_down.shape[0], : top_down.shape[1]] = top_down
    return output


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

    # Report interpretation: only GT wall cells that lie in the observed SLAM
    # region are eligible for wall-match. Unknown/unexplored GT walls are not
    # counted as misses.
    observed_gt_wall = structural_occ & known_near
    observed_gt_count = int(observed_gt_wall.sum())
    wall_match = (
        float((observed_gt_wall & occ_near).sum() / observed_gt_count)
        if observed_gt_count
        else float("nan")
    )

    # Fraction of SLAM cells confidently observed FREE that structural GT marks
    # occupied. This is intentionally normalized by observed free area, not by
    # the number of GT wall cells.
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
        raise RuntimeError(f"No committed Hospital validation snapshots found in {VALIDATION_DIR}")

    wall_raw = _build_explicit_wall_mask(
        aws_root=aws_root,
        spawn_x=spawn_x,
        spawn_y=spawn_y,
        spawn_yaw=spawn_yaw,
        z_slice=z_slice,
        wall_thickness_cells=wall_thickness_cells,
        include_flat_blockers=include_flat_blockers,
    )

    frame_results = []
    all_pass = True
    for frame, path, reference in references:
        structural_occ = _wall_mask_as_lama_image(wall_raw, reference.shape)
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

    # In SLAM-start coordinates the initial robot pose is (0,0).
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
            "wall_match = matched GT wall cells / GT wall cells in observed SLAM region; "
            "free_space_conflict = GT-occupied & SLAM-free / observed SLAM-free cells"
        ),
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
            f"Each committed validation frame needs wall_match >= "
            f"{MIN_WALL_MATCH * 100:.0f}% and free_conflict <= "
            f"{MAX_FREE_CONFLICT * 100:.0f}%. Do not run the benchmark yet."
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
