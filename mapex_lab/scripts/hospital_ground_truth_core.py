#!/usr/bin/env python3
"""Canonical Hospital ROI/structural-GT generation helpers.

This restores the exact rasterization strategy used by the historical
``mapex_hospital_research/scripts/generate_hospital_roi.py`` that originally
froze Hospital ROI v1:

- load the COLLADA through trimesh so scene-graph transforms are honored;
- normalize COLLADA units/up-axis;
- intersect the wall mesh with a horizontal plane;
- rasterize segments with OpenCV using round-to-nearest metric->cell mapping;
- fill the physical elevator blockers from the active world;
- close one-cell raster cracks with OpenCV dilation;
- take the 8-connected free component containing the SLAM-start origin.

The active world is still authoritative.  Therefore a later geometry change
(e.g. the 1.60 m -> 1.80 m elevator-blocker widening) is allowed to produce a
new mask; callers can compare it with the frozen v1 cell count/SHA and decide
whether a new ROI version is required.
"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import scipy.ndimage
import trimesh
import yaml


MAPEX_LAB_ROOT = Path(__file__).resolve().parents[1]
WORLD_PATH = MAPEX_LAB_ROOT / "map" / "hospital_aws_flat.sdf"
MODELS_DIR = MAPEX_LAB_ROOT / "map" / "models"
ROI_SPEC_PATH = MAPEX_LAB_ROOT / "ground_truth" / "hospital" / "roi_v1.yaml"
OUTPUT_DIR = MAPEX_LAB_ROOT / "ground_truth" / "hospital" / "generated"

GT_ID = "hospital_structural_gt_v1"
ROI_ID = "hospital_connected_free_v1"
CANVAS_ID = "hospital_canvas_v1"

# Frozen v1 audit values.  These are not used to alter geometry; they only
# detect whether the active world still reproduces the historical v1 mask.
FROZEN_V1_CELLS = 215435
FROZEN_V1_SHA256 = "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"

SPAWN_X = 0.0
SPAWN_Y = 12.0
SPAWN_YAW = -1.57


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_spec() -> dict:
    with ROI_SPEC_PATH.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("roi"), dict):
        raise RuntimeError(f"Invalid Hospital ROI spec: {ROI_SPEC_PATH}")
    return data["roi"]


def parse_pose(text: str | None) -> tuple[float, float, float, float, float, float]:
    values = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])  # type: ignore[return-value]


def find_world_geometry(world_path: Path):
    root = ET.parse(world_path).getroot()
    wall_pose = None
    for include in root.findall(".//include"):
        uri = (include.findtext("uri") or "").strip()
        if uri.endswith("aws_robomaker_hospital_floor_01_walls"):
            x, y, _z, _r, _p, yaw = parse_pose(include.findtext("pose"))
            wall_pose = (x, y, yaw)
            break
    if wall_pose is None:
        raise RuntimeError(f"Hospital wall include not found: {world_path}")

    blockers = []
    for model in root.findall(".//model"):
        name = model.attrib.get("name", "")
        if not ("elevator" in name.lower() and "blocker" in name.lower()):
            continue
        x, y, z, _r, _p, yaw = parse_pose(model.findtext("pose"))
        size_text = model.findtext(".//collision/geometry/box/size")
        if not size_text:
            continue
        size = [float(v) for v in size_text.split()]
        if len(size) < 3:
            continue
        blockers.append(
            {
                "name": name,
                "x": x,
                "y": y,
                "z": z,
                "yaw": yaw,
                "sx": size[0],
                "sy": size[1],
                "sz": size[2],
            }
        )
    return wall_pose, blockers


def collada_metadata(mesh_path: Path) -> tuple[float, str]:
    root = ET.parse(mesh_path).getroot()

    def local_name(tag: str) -> str:
        return tag.rsplit("}", 1)[-1]

    unit_scale = 1.0
    up_axis = "Z_UP"
    asset = next((node for node in root if local_name(node.tag) == "asset"), None)
    if asset is not None:
        for child in asset:
            name = local_name(child.tag)
            if name == "unit":
                unit_scale = float(child.attrib.get("meter", "1.0"))
            elif name == "up_axis" and child.text:
                up_axis = child.text.strip().upper()
    return unit_scale, up_axis


def normalized_wall_mesh(mesh_path: Path):
    loaded = trimesh.load(str(mesh_path), force="scene")
    if isinstance(loaded, trimesh.Scene):
        meshes = []
        for node_name in loaded.graph.nodes_geometry:
            transform, geom_name = loaded.graph[node_name]
            geom = loaded.geometry[geom_name].copy()
            geom.apply_transform(transform)
            meshes.append(geom)
        if not meshes:
            raise RuntimeError("Hospital wall DAE contains no geometry")
        mesh = trimesh.util.concatenate(meshes)
    else:
        mesh = loaded.copy()

    unit_scale, up_axis = collada_metadata(mesh_path)
    extents = np.asarray(mesh.extents, dtype=np.float64)

    # Historical canonical generator behavior.
    if unit_scale != 1.0 and float(np.max(extents)) > 200.0:
        mesh.apply_scale(unit_scale)
        extents = np.asarray(mesh.extents, dtype=np.float64)

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
    elif up_axis not in {"Y_UP", "Z_UP"}:
        raise RuntimeError(f"Unsupported COLLADA up_axis {up_axis!r}")

    if float(np.max(extents[:2])) > 100.0 or float(extents[2]) > 10.0:
        raise RuntimeError(f"Implausible normalized wall-mesh extents: {extents.tolist()}")
    return mesh, unit_scale, up_axis, extents


def unique_wall_segments(mesh, z_slice: float) -> list[np.ndarray]:
    raw = trimesh.intersections.mesh_plane(
        mesh=mesh,
        plane_normal=np.array([0.0, 0.0, 1.0]),
        plane_origin=np.array([0.0, 0.0, z_slice]),
    )
    raw = np.asarray(raw, dtype=np.float64)
    if raw.size == 0:
        raise RuntimeError(f"No wall intersections found at z={z_slice}")

    unique = []
    seen = set()
    for segment in raw.reshape((-1, 2, 3)):
        xy = segment[:, :2]
        if float(np.linalg.norm(xy[1] - xy[0])) < 1e-9:
            continue
        rounded = np.round(xy, 6)
        a = (float(rounded[0, 0]), float(rounded[0, 1]))
        b = (float(rounded[1, 0]), float(rounded[1, 1]))
        key = (a, b) if a <= b else (b, a)
        if key in seen:
            continue
        seen.add(key)
        unique.append(xy.copy())
    return unique


def transform_xy(xy: np.ndarray, tx: float, ty: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]], dtype=np.float64)
    return xy @ rot.T + np.array([tx, ty], dtype=np.float64)


def world_to_slam(xy: np.ndarray) -> np.ndarray:
    shifted = xy - np.array([SPAWN_X, SPAWN_Y], dtype=np.float64)
    c, s = math.cos(-SPAWN_YAW), math.sin(-SPAWN_YAW)
    rot = np.array([[c, -s], [s, c]], dtype=np.float64)
    return shifted @ rot.T


def rectangle_corners(x: float, y: float, yaw: float, sx: float, sy: float):
    local = np.array(
        [
            [-sx / 2.0, -sy / 2.0],
            [sx / 2.0, -sy / 2.0],
            [sx / 2.0, sy / 2.0],
            [-sx / 2.0, sy / 2.0],
        ],
        dtype=np.float64,
    )
    return transform_xy(local, x, y, yaw)


def _wall_mesh_path(spec: dict) -> Path:
    structural = spec["structural_source"]
    mesh_name = structural["wall_collision_mesh"]
    return (
        MODELS_DIR
        / "aws_robomaker_hospital_floor_01_walls"
        / "meshes"
        / mesh_name
    )


def build_geometry(spec: dict):
    canvas = spec["canvas"]
    structural = spec["structural_source"]
    bounds = spec["bounds_slam_start_m"]

    width = int(canvas["width_cells"])
    height = int(canvas["height_cells"])
    resolution = float(spec["resolution_m"])
    origin_x = float(canvas["origin_x_m"])
    origin_y = float(canvas["origin_y_m"])

    if not WORLD_PATH.is_file():
        raise FileNotFoundError(WORLD_PATH)
    mesh_path = _wall_mesh_path(spec)
    if not mesh_path.is_file():
        raise FileNotFoundError(mesh_path)

    def metric_to_rc(xy: np.ndarray) -> np.ndarray:
        # Keep np.rint: the frozen v1 SHA depends on this exact convention.
        cols = np.rint((xy[:, 0] - origin_x) / resolution).astype(np.int32)
        rows = np.rint((xy[:, 1] - origin_y) / resolution).astype(np.int32)
        return np.stack([rows, cols], axis=1)

    obstacle = np.zeros((height, width), dtype=np.uint8)
    wall_pose, blockers = find_world_geometry(WORLD_PATH)
    wall_mesh, unit_scale, up_axis, mesh_extents = normalized_wall_mesh(mesh_path)
    z_slice = float(structural["wall_slice_z_m"])
    segments = unique_wall_segments(wall_mesh, z_slice)

    wall_x, wall_y, wall_yaw = wall_pose
    wall_thickness = int(structural["wall_thickness_cells"])
    transformed_endpoints = []
    for segment in segments:
        world_xy = transform_xy(segment, wall_x, wall_y, wall_yaw)
        slam_xy = world_to_slam(world_xy)
        transformed_endpoints.append(slam_xy)
        rc = metric_to_rc(slam_xy)
        a, b = rc[0], rc[1]
        cv2.line(
            obstacle,
            (int(a[1]), int(a[0])),
            (int(b[1]), int(b[0])),
            255,
            thickness=wall_thickness,
            lineType=cv2.LINE_8,
        )

    blocker_meta = []
    if bool(structural.get("include_flat_elevator_blockers", True)):
        for blocker in blockers:
            world_xy = rectangle_corners(
                blocker["x"], blocker["y"], blocker["yaw"], blocker["sx"], blocker["sy"]
            )
            slam_xy = world_to_slam(world_xy)
            rc = metric_to_rc(slam_xy)
            polygon = np.stack([rc[:, 1], rc[:, 0]], axis=1).astype(np.int32)
            cv2.fillPoly(obstacle, [polygon], 255)
            blocker_meta.append(blocker)

    dilation_cells = int(structural["close_raster_cracks_dilation_cells"])
    if dilation_cells > 0:
        kernel = np.ones((3, 3), dtype=np.uint8)
        obstacle = cv2.dilate(obstacle, kernel, iterations=dilation_cells)

    bbox_xy = np.array(
        [
            [float(bounds["xmin"]), float(bounds["ymin"])],
            [float(bounds["xmax"]), float(bounds["ymax"])],
        ],
        dtype=np.float64,
    )
    bbox_rc = metric_to_rc(bbox_xy)
    rmin, rmax = sorted([int(bbox_rc[0, 0]), int(bbox_rc[1, 0])])
    cmin, cmax = sorted([int(bbox_rc[0, 1]), int(bbox_rc[1, 1])])
    rmin, rmax = max(0, rmin), min(height - 1, rmax)
    cmin, cmax = max(0, cmin), min(width - 1, cmax)

    bbox_mask = np.zeros((height, width), dtype=bool)
    bbox_mask[rmin : rmax + 1, cmin : cmax + 1] = True
    walkable = bbox_mask & (obstacle == 0)

    seed_xy = np.asarray([spec["valid_cell_rule"]["seed_xy_in_slam_start_m"]], dtype=float)
    seed_rc = metric_to_rc(seed_xy)[0]
    sr, sc = int(seed_rc[0]), int(seed_rc[1])
    if not (0 <= sr < height and 0 <= sc < width and walkable[sr, sc]):
        coords = np.argwhere(walkable)
        if len(coords) == 0:
            raise RuntimeError("No walkable cells inside Hospital bounds")
        d2 = np.sum((coords - np.array([sr, sc])) ** 2, axis=1)
        sr, sc = [int(v) for v in coords[int(np.argmin(d2))]]

    structure = np.ones((3, 3), dtype=np.int8)
    labels, component_count = scipy.ndimage.label(walkable, structure=structure)
    start_label = int(labels[sr, sc])
    if start_label == 0:
        raise RuntimeError("ROI seed is outside the connected walkable region")
    roi = labels == start_label

    if transformed_endpoints:
        endpoints = np.concatenate(transformed_endpoints, axis=0)
        wall_bounds = {
            "xmin": float(endpoints[:, 0].min()),
            "xmax": float(endpoints[:, 0].max()),
            "ymin": float(endpoints[:, 1].min()),
            "ymax": float(endpoints[:, 1].max()),
        }
    else:
        wall_bounds = None

    audit = {
        "wall_segment_count": len(segments),
        "elevator_blocker_count": len(blocker_meta),
        "elevator_blockers": blocker_meta,
        "component_count": int(component_count),
        "seed_row": sr,
        "seed_col": sc,
        "bbox_rows": [rmin, rmax],
        "bbox_cols": [cmin, cmax],
        "obstacle_cells_after_dilation": int((obstacle > 0).sum()),
        "denominator_cells": int(roi.sum()),
        "dae_unit_meter": unit_scale,
        "dae_up_axis": up_axis,
        "normalized_mesh_extents_xyz_m": [float(v) for v in mesh_extents],
        "wall_slice_bounds_slam_start_m": wall_bounds,
        "wall_slice_z_m": z_slice,
        "wall_thickness_cells": wall_thickness,
        "crack_dilation_cells": dilation_cells,
    }
    return roi, obstacle, bbox_mask, audit, metric_to_rc, mesh_path


def generate(output_dir: Path = OUTPUT_DIR) -> dict:
    spec = load_spec()
    roi, obstacle, bbox_mask, audit, _metric_to_rc, mesh_path = build_geometry(spec)

    output_dir.mkdir(parents=True, exist_ok=True)
    roi_path = output_dir / f"{ROI_ID}.npy"
    gt_path = output_dir / f"{GT_ID}.npz"
    roi_meta_path = output_dir / f"{ROI_ID}_metadata.json"
    summary_path = output_dir / f"{GT_ID}_summary.json"
    preview_path = output_dir / f"{GT_ID}_preview.png"

    # Historical v1 used uint8 + allow_pickle=False.  Keep that representation
    # so the SHA comparison is meaningful.
    np.save(roi_path, roi.astype(np.uint8), allow_pickle=False)
    roi_sha = sha256_file(roi_path)
    roi_cells = int(roi.sum())

    occupied = (obstacle > 0) & bbox_mask
    evaluation_mask = roi | occupied
    data = np.full(roi.shape, -1, dtype=np.int16)
    data[roi] = 0
    data[occupied] = 100

    canvas = spec["canvas"]
    resolution = float(spec["resolution_m"])
    origin_x = float(canvas["origin_x_m"])
    origin_y = float(canvas["origin_y_m"])
    np.savez_compressed(
        gt_path,
        data=data,
        evaluation_mask=evaluation_mask,
        resolution=np.float64(resolution),
        width=np.int64(int(canvas["width_cells"])),
        height=np.int64(int(canvas["height_cells"])),
        origin_x=np.float64(origin_x),
        origin_y=np.float64(origin_y),
        canvas_id=np.asarray(CANVAS_ID),
        ground_truth_id=np.asarray(GT_ID),
        roi_id=np.asarray(ROI_ID),
        source_world=np.asarray(str(WORLD_PATH.resolve())),
        source_world_sha256=np.asarray(sha256_file(WORLD_PATH)),
        source_wall_mesh=np.asarray(str(mesh_path.resolve())),
        source_wall_mesh_sha256=np.asarray(sha256_file(mesh_path)),
        hospital_scale=np.float64(1.0),
        spawn_world_x=np.float64(SPAWN_X),
        spawn_world_y=np.float64(SPAWN_Y),
        spawn_world_yaw=np.float64(SPAWN_YAW),
    )

    preview = np.zeros(roi.shape, dtype=np.uint8)
    preview[roi] = 200
    preview[occupied] = 255
    cv2.imwrite(str(preview_path), np.flipud(preview))

    frozen_match = roi_cells == FROZEN_V1_CELLS and roi_sha == FROZEN_V1_SHA256
    current_blocker_widths = sorted({round(float(b["sx"]), 6) for b in audit["elevator_blockers"]})
    summary = {
        "status": "ok_frozen_v1_match" if frozen_match else "ok_active_world_differs_from_frozen_v1",
        "hospital_scale": 1.0,
        "world": str(WORLD_PATH.resolve()),
        "world_sha256": sha256_file(WORLD_PATH),
        "wall_mesh": str(mesh_path.resolve()),
        "wall_mesh_sha256": sha256_file(mesh_path),
        "roi": {
            "id": ROI_ID,
            "cells": roi_cells,
            "sha256": roi_sha,
            "frozen_v1_expected_cells": FROZEN_V1_CELLS,
            "frozen_v1_expected_sha256": FROZEN_V1_SHA256,
            "frozen_v1_match": frozen_match,
        },
        "active_world_elevator_blocker_widths_m": current_blocker_widths,
        "audit": audit,
        "structural_gt": {
            "id": GT_ID,
            "evaluation_mask_cells": int(evaluation_mask.sum()),
            "occupied_cells": int(occupied.sum()),
        },
        "outputs": {
            "roi": str(roi_path),
            "roi_metadata": str(roi_meta_path),
            "structural_gt": str(gt_path),
            "preview": str(preview_path),
            "summary": str(summary_path),
        },
    }
    roi_meta_path.write_text(
        json.dumps(
            {
                "roi_id": ROI_ID,
                "mask_sha256": roi_sha,
                "denominator_cells": roi_cells,
                "shape": [int(roi.shape[0]), int(roi.shape[1])],
                "resolution_m": resolution,
                "frozen_v1_match": frozen_match,
                "audit": audit,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary
