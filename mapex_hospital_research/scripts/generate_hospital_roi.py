#!/usr/bin/env python3
"""Generate and freeze the canonical Hospital exploration ROI.

Outputs are local-only under:
  mapex_hospital_research/ground_truth/hospital/generated/

The script also writes the resulting denominator and mask SHA-256 back into
roi_v1.yaml and config/hospital.yaml, and updates EXPERIMENT_PROTOCOL.md.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import scipy.ndimage
import trimesh
import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
REPO_ROOT = WORKSPACE.parent
ROI_SPEC_PATH = WORKSPACE / "ground_truth" / "hospital" / "roi_v1.yaml"
HOSPITAL_CONFIG_PATH = WORKSPACE / "config" / "hospital.yaml"
PROTOCOL_PATH = WORKSPACE / "EXPERIMENT_PROTOCOL.md"

DEFAULT_MODELS_DIR = (
    Path.home() / ".cache" / "turtlebot4_project" / "hospital_world" / "models"
)


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected mapping YAML: {path}")
    return data


def save_yaml(path: Path, data: dict) -> None:
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)


def parse_pose(text: str | None) -> tuple[float, float, float, float, float, float]:
    values = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


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
        name = model.attrib.get("name", "").lower()
        if not ("elevator" in name and "blocker" in name):
            continue
        x, y, _z, _r, _p, yaw = parse_pose(model.findtext("pose"))
        size_text = model.findtext(".//collision/geometry/box/size")
        if not size_text:
            continue
        size = [float(v) for v in size_text.split()]
        if len(size) >= 2:
            blockers.append((x, y, yaw, size[0], size[1]))
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
    return mesh


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


def world_to_slam(xy: np.ndarray, spawn_x: float, spawn_y: float, spawn_yaw: float):
    shifted = xy - np.array([spawn_x, spawn_y], dtype=np.float64)
    c, s = math.cos(-spawn_yaw), math.sin(-spawn_yaw)
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


def build_roi(roi_spec: dict, hospital_config: dict, models_dir: Path):
    roi = roi_spec["roi"]
    canvas = roi["canvas"]
    structural = roi["structural_source"]
    bounds = roi["bounds_slam_start_m"]

    width = int(canvas["width_cells"])
    height = int(canvas["height_cells"])
    resolution = float(roi["resolution_m"])
    origin_x = float(canvas["origin_x_m"])
    origin_y = float(canvas["origin_y_m"])

    spawn = hospital_config["experiment"]["spawn"]
    spawn_x = float(spawn["x"])
    spawn_y = float(spawn["y"])
    spawn_yaw = float(spawn["yaw"])

    world_path = REPO_ROOT / structural["world"]
    mesh_path = (
        models_dir
        / "aws_robomaker_hospital_floor_01_walls"
        / "meshes"
        / structural["wall_collision_mesh"]
    )
    if not world_path.is_file():
        raise FileNotFoundError(world_path)
    if not mesh_path.is_file():
        raise FileNotFoundError(
            f"{mesh_path}\nRun ros2_ws/src/frontier_exploration/scripts/"
            "setup_hospital_world_assets.sh first."
        )

    def metric_to_rc(xy: np.ndarray) -> np.ndarray:
        cols = np.rint((xy[:, 0] - origin_x) / resolution).astype(np.int32)
        rows = np.rint((xy[:, 1] - origin_y) / resolution).astype(np.int32)
        return np.stack([rows, cols], axis=1)

    obstacle = np.zeros((height, width), dtype=np.uint8)
    wall_pose, blockers = find_world_geometry(world_path)
    wall_mesh = normalized_wall_mesh(mesh_path)
    segments = unique_wall_segments(wall_mesh, float(structural["wall_slice_z_m"]))

    wall_x, wall_y, wall_yaw = wall_pose
    wall_thickness = int(structural["wall_thickness_cells"])
    for segment in segments:
        world_xy = transform_xy(segment, wall_x, wall_y, wall_yaw)
        slam_xy = world_to_slam(world_xy, spawn_x, spawn_y, spawn_yaw)
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

    blocker_count = 0
    if bool(structural.get("include_flat_elevator_blockers", True)):
        for x, y, yaw, sx, sy in blockers:
            world_xy = rectangle_corners(x, y, yaw, sx, sy)
            slam_xy = world_to_slam(world_xy, spawn_x, spawn_y, spawn_yaw)
            rc = metric_to_rc(slam_xy)
            polygon = np.stack([rc[:, 1], rc[:, 0]], axis=1).astype(np.int32)
            cv2.fillPoly(obstacle, [polygon], 255)
            blocker_count += 1

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

    seed_xy = np.asarray([roi["valid_cell_rule"]["seed_xy_in_slam_start_m"]], dtype=float)
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
    valid = labels == start_label

    audit = {
        "wall_segment_count": len(segments),
        "elevator_blocker_count": blocker_count,
        "component_count": int(component_count),
        "seed_row": sr,
        "seed_col": sc,
        "bbox_rows": [rmin, rmax],
        "bbox_cols": [cmin, cmax],
        "obstacle_cells_after_dilation": int((obstacle > 0).sum()),
        "denominator_cells": int(valid.sum()),
    }
    return valid, obstacle, audit


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def update_protocol(denominator: int, mask_sha256: str) -> None:
    text = PROTOCOL_PATH.read_text(encoding="utf-8")
    text = re.sub(
        r"- Total denominator cells: .*",
        f"- Total denominator cells: `{denominator}` (frozen by mask SHA-256 `{mask_sha256}`).",
        text,
        count=1,
    )
    PROTOCOL_PATH.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models-dir", type=Path, default=DEFAULT_MODELS_DIR)
    parser.add_argument(
        "--force",
        action="store_true",
        help="Allow replacing an already-generated mask. Use only before nearest_001.",
    )
    args = parser.parse_args()

    roi_spec = load_yaml(ROI_SPEC_PATH)
    hospital_config = load_yaml(HOSPITAL_CONFIG_PATH)
    roi = roi_spec["roi"]

    output_path = REPO_ROOT / roi["generated_mask"]
    output_dir = output_path.parent
    metadata_path = output_dir / "hospital_connected_free_v1_metadata.json"
    preview_path = output_dir / "hospital_connected_free_v1_preview.png"

    if output_path.exists() and not args.force:
        raise RuntimeError(
            f"ROI mask already exists: {output_path}\n"
            "Refusing to replace a frozen mask. Use --force only if no baseline run exists."
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    valid, obstacle, audit = build_roi(roi_spec, hospital_config, args.models_dir.expanduser())

    np.save(output_path, valid.astype(np.uint8), allow_pickle=False)
    mask_sha256 = sha256_file(output_path)
    denominator = int(valid.sum())

    preview = np.zeros(valid.shape, dtype=np.uint8)
    preview[valid] = 200
    preview[obstacle > 0] = 255
    cv2.imwrite(str(preview_path), np.flipud(preview))

    metadata = {
        "roi_id": roi["id"],
        "mask_path": str(output_path.relative_to(REPO_ROOT)),
        "mask_sha256": mask_sha256,
        "denominator_cells": denominator,
        "shape": [int(valid.shape[0]), int(valid.shape[1])],
        "resolution_m": float(roi["resolution_m"]),
        "audit": audit,
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    roi["denominator_cells"] = denominator
    roi["mask_sha256"] = mask_sha256
    roi["generated_metadata"] = str(metadata_path.relative_to(REPO_ROOT))
    save_yaml(ROI_SPEC_PATH, roi_spec)

    hospital_config["canonical_evaluation_roi"]["denominator_cells"] = denominator
    hospital_config["canonical_evaluation_roi"]["mask_sha256"] = mask_sha256
    save_yaml(HOSPITAL_CONFIG_PATH, hospital_config)
    update_protocol(denominator, mask_sha256)

    print(f"ROI ID: {roi['id']}")
    print(f"Mask: {output_path}")
    print(f"Denominator cells: {denominator}")
    print(f"SHA-256: {mask_sha256}")
    print(f"Wall segments: {audit['wall_segment_count']}")
    print(f"Elevator blockers: {audit['elevator_blocker_count']}")
    print("Updated: roi_v1.yaml, config/hospital.yaml, EXPERIMENT_PROTOCOL.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
