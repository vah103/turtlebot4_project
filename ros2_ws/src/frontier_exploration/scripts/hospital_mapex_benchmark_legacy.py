#!/usr/bin/env python3
"""
One-command Hospital benchmark for MapEx and baselines.

What it does
------------
1. Prepares an offline Hospital structural map from the AWS wall collision mesh.
2. Runs the ORIGINAL MapEx closed-loop simulator for:
      nearest, upen, hectoraug, visvarprob (MapEx)
3. Generates map predictions with the official MapEx Big-LaMa checkpoint.
4. Computes paper-style metrics:
      Coverage, occupied IoU, Topological Understanding (TU)
5. Writes per-method curves, normalized AUCs, and improvement vs Nearest.

Important
---------
- This is OFFLINE. Gazebo/Nav2 do not need to be running.
- Exact all-method reproduction uses the upstream MapEx code, not reimplemented
  approximations.
- UPEN in the upstream release contains CUDA-specific calls. For the default
  four-method comparison, run inside the MapEx/LAMA environment with CUDA.
- The Hospital structural ground truth is generated from the same floor-1 wall
  collision mesh used by the Hospital environment. It is a structural reference,
  not a sensor-perfect ground truth.

Typical use
-----------
    python3 hospital_mapex_benchmark.py --setup --run

If MapEx weights are already installed:
    python3 hospital_mapex_benchmark.py --run

Quick smoke test:
    python3 hospital_mapex_benchmark.py --setup --run --mission-time 100

Outputs
-------
    artifacts/hospital_mapex_benchmark/
        summary.csv
        improvement_vs_nearest.csv
        benchmark_metadata.json
        curves/<method>_coverage.csv
        curves/<method>_iou.csv
        curves/<method>_tu.csv
"""

from __future__ import annotations

import argparse
import csv
import importlib
import json
import math
import os
import random
import shutil
import subprocess
import sys
import time
import zipfile
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[4]
PACKAGE_ROOT = PROJECT_ROOT / "ros2_ws" / "src" / "frontier_exploration"
DEFAULT_MAPEX_ROOT = PROJECT_ROOT / "third_party" / "MapEx"
DEFAULT_AWS_ROOT = PROJECT_ROOT / "third_party" / "aws-hospital-world"
DEFAULT_OUTPUT_ROOT = PROJECT_ROOT / "artifacts" / "hospital_mapex_benchmark"
HOSPITAL_WORLD_SDF = PACKAGE_ROOT / "worlds" / "hospital_aws_flat.sdf"
SNAPSHOT_CONFIG = PACKAGE_ROOT / "config" / "map_snapshot_hospital.yaml"

MAPEX_URL = "https://github.com/castacks/MapEx.git"
AWS_URL = "https://github.com/TeamSOBITS/aws-hospital-world.git"
MAPEX_WEIGHTS_FOLDER_URL = (
    "https://drive.google.com/drive/folders/1u9WZ9ftwaMbP-RVySuNSVEdUDV_x4Dw6"
)

WORLD_ID = "HOSPITAL_FLAT"
DEFAULT_METHODS = ("nearest", "upen", "hectoraug", "visvarprob")
METHOD_LABELS = {
    "nearest": "Nearest",
    "upen": "UPEN",
    "hectoraug": "IG-Hector",
    "visvarprob": "MapEx",
}

# Values used by the existing Hospital recorder/report.
CANVAS_WIDTH = 1504
CANVAS_HEIGHT = 2123
CANVAS_RESOLUTION = 0.05
CANVAS_ORIGIN_X = -25.6
CANVAS_ORIGIN_Y = -60.1

# Approximate Hospital floor bounds in SLAM-start coordinates from the current
# recorder configuration. These bounds keep the flood-fill inside the building
# neighborhood while the fixed canvas retains the large 25 m safety margin.
HOSPITAL_BOUNDS = (-0.572445, 24.588833, -35.091079, 21.044604)

# Robot spawn used by hospital_flat_stack / map_snapshot_hospital.yaml.
DEFAULT_SPAWN_X = 0.0
DEFAULT_SPAWN_Y = 12.0
DEFAULT_SPAWN_YAW = -1.57

PADDING = 500
RAW_TO_MAPEX_BLOCK = 2  # 0.05 m/cell -> 0.10 m/pixel
PIXELS_PER_METER = 10

WEIGHT_RELATIVE_PATHS = (
    Path("weights/big_lama/models/best.ckpt"),
    Path("weights/lama_ensemble/train_1/models/best.ckpt"),
    Path("weights/lama_ensemble/train_2/models/best.ckpt"),
    Path("weights/lama_ensemble/train_3/models/best.ckpt"),
)


def log(msg: str) -> None:
    print(f"[hospital-mapex] {msg}", flush=True)


def run_cmd(
    cmd: Sequence[str],
    cwd: Optional[Path] = None,
    env: Optional[dict] = None,
    check: bool = True,
) -> subprocess.CompletedProcess:
    log("$ " + " ".join(str(x) for x in cmd))
    return subprocess.run(
        [str(x) for x in cmd],
        cwd=str(cwd) if cwd else None,
        env=env,
        check=check,
    )


def require_import(module: str, install_hint: Optional[str] = None):
    try:
        return importlib.import_module(module)
    except ImportError as exc:
        hint = install_hint or module
        raise RuntimeError(
            f"Missing Python dependency '{module}'. Install it with: {hint}"
        ) from exc


def clone_if_missing(target: Path, url: str, branch: Optional[str] = None, recurse=False):
    if (target / ".git").exists():
        log(f"Using existing checkout: {target}")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["git", "clone", "--depth", "1"]
    if recurse:
        cmd.append("--recurse-submodules")
    if branch:
        cmd += ["--branch", branch]
    cmd += [url, str(target)]
    run_cmd(cmd)
    if recurse:
        run_cmd(["git", "submodule", "update", "--init", "--recursive"], cwd=target)


def expected_weight_paths(mapex_root: Path) -> List[Path]:
    base = mapex_root / "pretrained_models"
    return [base / p for p in WEIGHT_RELATIVE_PATHS]


def weights_ready(mapex_root: Path) -> bool:
    return all(p.exists() for p in expected_weight_paths(mapex_root))


def print_weight_help(mapex_root: Path) -> None:
    expected = "\n".join(f"  - {p}" for p in expected_weight_paths(mapex_root))
    print(
        "\nOfficial MapEx weights are missing.\n"
        "Download the pretrained KTH weights from:\n"
        f"  {MAPEX_WEIGHTS_FOLDER_URL}\n"
        "and place/unzip them so these files exist:\n"
        f"{expected}\n"
        "\nOr install gdown and rerun with --download-weights:\n"
        "  python3 -m pip install gdown\n"
    )


def download_weights(mapex_root: Path) -> None:
    if weights_ready(mapex_root):
        log("MapEx weights already present.")
        return
    try:
        import gdown  # noqa: F401
    except ImportError as exc:
        print_weight_help(mapex_root)
        raise RuntimeError("gdown is not installed.") from exc

    dest = mapex_root / "pretrained_models"
    dest.mkdir(parents=True, exist_ok=True)
    log("Downloading official MapEx weights with gdown...")
    # gdown's folder CLI is more stable across releases than the Python API.
    run_cmd(
        [
            sys.executable,
            "-m",
            "gdown",
            "--folder",
            MAPEX_WEIGHTS_FOLDER_URL,
            "-O",
            str(dest),
        ]
    )

    # The Drive folder may contain weights.zip or an already-expanded weights dir.
    zip_candidates = list(dest.rglob("weights.zip"))
    for zpath in zip_candidates:
        log(f"Extracting {zpath}")
        with zipfile.ZipFile(zpath, "r") as zf:
            zf.extractall(dest)

    # Normalize common accidental nesting, e.g. pretrained_models/weights/weights/.
    nested = dest / "weights" / "weights"
    if nested.exists() and not (dest / "weights" / "big_lama").exists():
        for child in nested.iterdir():
            shutil.move(str(child), str(dest / "weights" / child.name))
        nested.rmdir()

    if not weights_ready(mapex_root):
        print_weight_help(mapex_root)
        raise RuntimeError("Download completed, but expected checkpoint layout was not found.")


def parse_pose(text: Optional[str]) -> Tuple[float, float, float, float, float, float]:
    values = [float(x) for x in (text or "0 0 0 0 0 0").split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


def find_world_model_geometry(
    sdf_path: Path,
) -> Tuple[Tuple[float, float, float], List[Tuple[float, float, float, float, float]]]:
    """Return wall model pose (x,y,yaw) and flat blocker boxes (x,y,yaw,sx,sy)."""
    root = ET.parse(sdf_path).getroot()
    wall_pose = None

    for include in root.findall(".//include"):
        uri = (include.findtext("uri") or "").strip()
        if uri.endswith("aws_robomaker_hospital_floor_01_walls"):
            x, y, _z, _r, _p, yaw = parse_pose(include.findtext("pose"))
            wall_pose = (x, y, yaw)
            break

    if wall_pose is None:
        raise RuntimeError(
            "Could not find floor-1 Hospital wall include in "
            f"{sdf_path}"
        )

    blockers: List[Tuple[float, float, float, float, float]] = []
    for model in root.findall(".//model"):
        name = model.attrib.get("name", "")
        if "elevator_blocker" not in name:
            continue
        x, y, _z, _r, _p, yaw = parse_pose(model.findtext("pose"))
        size_text = model.findtext(".//collision/geometry/box/size")
        if size_text:
            sx, sy, *_ = [float(v) for v in size_text.split()]
            blockers.append((x, y, yaw, sx, sy))
    return wall_pose, blockers


def transform_xy(
    xy: np.ndarray,
    tx: float,
    ty: float,
    yaw: float,
) -> np.ndarray:
    """Apply 2D rigid transform to N x 2 points."""
    c, s = math.cos(yaw), math.sin(yaw)
    r = np.array([[c, -s], [s, c]], dtype=np.float64)
    return xy @ r.T + np.array([tx, ty], dtype=np.float64)


def world_to_slam(
    xy_world: np.ndarray,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
) -> np.ndarray:
    """World coordinates -> robot start/SLAM frame."""
    shifted = xy_world - np.array([spawn_x, spawn_y], dtype=np.float64)
    c, s = math.cos(-spawn_yaw), math.sin(-spawn_yaw)
    r = np.array([[c, -s], [s, c]], dtype=np.float64)
    return shifted @ r.T


def metric_xy_to_raw_rc(xy: np.ndarray) -> np.ndarray:
    col = np.rint((xy[:, 0] - CANVAS_ORIGIN_X) / CANVAS_RESOLUTION).astype(int)
    row = np.rint((xy[:, 1] - CANVAS_ORIGIN_Y) / CANVAS_RESOLUTION).astype(int)
    return np.stack([row, col], axis=1)


def raw_rc_in_bounds(rc: np.ndarray) -> np.ndarray:
    return (
        (rc[:, 0] >= 0)
        & (rc[:, 0] < CANVAS_HEIGHT)
        & (rc[:, 1] >= 0)
        & (rc[:, 1] < CANVAS_WIDTH)
    )


def load_wall_section_polylines(mesh_path: Path, z_slice: float):
    trimesh = require_import("trimesh", "python3 -m pip install trimesh")
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
        mesh = loaded

    section = mesh.section(
        plane_origin=np.array([0.0, 0.0, z_slice]),
        plane_normal=np.array([0.0, 0.0, 1.0]),
    )
    if section is None:
        raise RuntimeError(f"No wall-mesh section found at z={z_slice:.3f} m.")

    polylines = [np.asarray(p)[:, :2] for p in section.discrete if len(p) >= 2]
    segment_count = int(sum(max(0, len(p) - 1) for p in polylines))
    entity_count = len(section.entities)
    return polylines, entity_count, segment_count


def rasterize_polyline(mask: np.ndarray, rc: np.ndarray, thickness: int, cv2) -> None:
    if len(rc) < 2:
        return
    for a, b in zip(rc[:-1], rc[1:]):
        if not (raw_rc_in_bounds(np.stack([a, b])).all()):
            # cv2 can clip, but huge invalid coordinates are better skipped.
            if not (raw_rc_in_bounds(np.stack([a, b])).any()):
                continue
        cv2.line(
            mask,
            (int(a[1]), int(a[0])),
            (int(b[1]), int(b[0])),
            color=255,
            thickness=thickness,
            lineType=cv2.LINE_8,
        )


def rectangle_corners(x: float, y: float, yaw: float, sx: float, sy: float) -> np.ndarray:
    local = np.array(
        [
            [-sx / 2, -sy / 2],
            [sx / 2, -sy / 2],
            [sx / 2, sy / 2],
            [-sx / 2, sy / 2],
        ],
        dtype=np.float64,
    )
    return transform_xy(local, x, y, yaw)


def nearest_true_cell(mask: np.ndarray, rc: Tuple[int, int]) -> Tuple[int, int]:
    r0, c0 = rc
    if 0 <= r0 < mask.shape[0] and 0 <= c0 < mask.shape[1] and mask[r0, c0]:
        return r0, c0
    coords = np.argwhere(mask)
    if len(coords) == 0:
        raise RuntimeError("No valid free cell exists in generated Hospital map.")
    d2 = np.sum((coords - np.array([r0, c0])) ** 2, axis=1)
    r, c = coords[np.argmin(d2)]
    return int(r), int(c)


def build_hospital_map(
    project_root: Path,
    mapex_root: Path,
    aws_root: Path,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
    z_slice: float,
    wall_thickness_cells: int,
    include_flat_blockers: bool,
) -> Tuple[Path, Tuple[int, int], dict]:
    cv2 = require_import("cv2", "python3 -m pip install opencv-python")
    scipy_ndimage = require_import("scipy.ndimage", "python3 -m pip install scipy")

    mesh_path = (
        aws_root
        / "models"
        / "aws_robomaker_hospital_floor_01_walls"
        / "meshes"
        / "aws_robomaker_hospital_floor_01_walls_collision.dae"
    )
    if not mesh_path.exists():
        raise RuntimeError(f"Hospital collision mesh missing: {mesh_path}")

    wall_pose, blockers = find_world_model_geometry(HOSPITAL_WORLD_SDF)
    wall_x, wall_y, wall_yaw = wall_pose

    polylines, entity_count, segment_count = load_wall_section_polylines(mesh_path, z_slice)
    log(
        f"Wall slice: {entity_count} entities, {segment_count} polyline segments "
        f"(report reference: 112 entities / 934 segments)."
    )

    obstacle = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH), dtype=np.uint8)
    all_slam_xy = []

    for poly in polylines:
        poly_world = transform_xy(poly, wall_x, wall_y, wall_yaw)
        poly_slam = world_to_slam(poly_world, spawn_x, spawn_y, spawn_yaw)
        all_slam_xy.append(poly_slam)
        rc = metric_xy_to_raw_rc(poly_slam)
        rasterize_polyline(obstacle, rc, wall_thickness_cells, cv2)

    blocker_count = 0
    if include_flat_blockers:
        for x, y, yaw, sx, sy in blockers:
            corners_world = rectangle_corners(x, y, yaw, sx, sy)
            corners_slam = world_to_slam(corners_world, spawn_x, spawn_y, spawn_yaw)
            rc = metric_xy_to_raw_rc(corners_slam)
            polygon = np.stack([rc[:, 1], rc[:, 0]], axis=1).astype(np.int32)
            cv2.fillPoly(obstacle, [polygon], 255)
            blocker_count += 1

    # Close single-cell rasterization cracks before connected-component flood-fill.
    kernel = np.ones((3, 3), dtype=np.uint8)
    obstacle_closed = cv2.dilate(obstacle, kernel, iterations=1)

    xmin, xmax, ymin, ymax = HOSPITAL_BOUNDS
    bbox_xy = np.array([[xmin, ymin], [xmax, ymax]], dtype=np.float64)
    bbox_rc = metric_xy_to_raw_rc(bbox_xy)
    rmin, rmax = sorted([int(bbox_rc[0, 0]), int(bbox_rc[1, 0])])
    cmin, cmax = sorted([int(bbox_rc[0, 1]), int(bbox_rc[1, 1])])
    rmin, rmax = max(0, rmin), min(CANVAS_HEIGHT - 1, rmax)
    cmin, cmax = max(0, cmin), min(CANVAS_WIDTH - 1, cmax)

    bbox_mask = np.zeros_like(obstacle_closed, dtype=bool)
    bbox_mask[rmin : rmax + 1, cmin : cmax + 1] = True
    walkable = bbox_mask & (obstacle_closed == 0)

    # In SLAM-start frame the initial robot pose is (0,0).
    start_raw = metric_xy_to_raw_rc(np.array([[0.0, 0.0]], dtype=np.float64))[0]
    sr, sc = nearest_true_cell(walkable, (int(start_raw[0]), int(start_raw[1])))

    structure = np.ones((3, 3), dtype=np.int8)
    labels, num_labels = scipy_ndimage.label(walkable, structure=structure)
    start_label = labels[sr, sc]
    if start_label == 0:
        raise RuntimeError("Robot start fell outside generated walkable Hospital area.")
    valid = labels == start_label

    # The MapEx KTH loader requires the raw occ file to contain ONLY 0 and 254.
    occ_raw = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH), dtype=np.uint8)
    occ_raw[valid] = 254
    valid_raw = np.zeros_like(occ_raw)
    valid_raw[valid] = 255

    map_dir = mapex_root / "kth_test_maps" / WORLD_ID
    map_dir.mkdir(parents=True, exist_ok=True)
    np.save(map_dir / "occ_map.npy", occ_raw)
    np.save(map_dir / "valid_space.npy", valid_raw)

    # Human-readable references for inspection.
    cv2.imwrite(str(map_dir / "hospital_structural_occ.png"), occ_raw)
    cv2.imwrite(str(map_dir / "hospital_valid_space.png"), valid_raw)

    # MapEx downsamples raw map by 2 and then pads 500 px.
    valid_reduced = valid.reshape(
        math.ceil(valid.shape[0] / 2), 2, math.ceil(valid.shape[1] / 2), 2
    ) if False else None
    # Use scipy/skimage-compatible semantics without requiring skimage here:
    # pad odd dimensions to even, then max over each 2x2 block.
    hpad = (-valid.shape[0]) % 2
    wpad = (-valid.shape[1]) % 2
    vr = np.pad(valid, ((0, hpad), (0, wpad)), constant_values=False)
    valid_01 = vr.reshape(vr.shape[0] // 2, 2, vr.shape[1] // 2, 2).max(axis=(1, 3))
    raw_start_rc = np.array([sr, sc])
    reduced_guess = (raw_start_rc // 2).astype(int)
    rr, cc = nearest_true_cell(
        valid_01,
        (int(reduced_guess[0]), int(reduced_guess[1])),
    )
    mapex_start = (rr + PADDING, cc + PADDING)

    touches_bbox = bool(
        valid[rmin, cmin : cmax + 1].any()
        or valid[rmax, cmin : cmax + 1].any()
        or valid[rmin : rmax + 1, cmin].any()
        or valid[rmin : rmax + 1, cmax].any()
    )
    if touches_bbox:
        log(
            "WARNING: connected free space touches the configured Hospital bounding box. "
            "Inspect hospital_valid_space.png if results look suspicious."
        )

    if all_slam_xy:
        pts = np.concatenate(all_slam_xy, axis=0)
        structural_bounds = {
            "x_min": float(pts[:, 0].min()),
            "x_max": float(pts[:, 0].max()),
            "y_min": float(pts[:, 1].min()),
            "y_max": float(pts[:, 1].max()),
        }
    else:
        structural_bounds = {}

    metadata = {
        "world_id": WORLD_ID,
        "canvas": {
            "width_cells": CANVAS_WIDTH,
            "height_cells": CANVAS_HEIGHT,
            "resolution_m_per_cell": CANVAS_RESOLUTION,
            "origin_x": CANVAS_ORIGIN_X,
            "origin_y": CANVAS_ORIGIN_Y,
        },
        "spawn_world": {"x": spawn_x, "y": spawn_y, "yaw": spawn_yaw},
        "wall_model_world_pose": {"x": wall_x, "y": wall_y, "yaw": wall_yaw},
        "slice_z_m": z_slice,
        "section_entities": entity_count,
        "section_segments": segment_count,
        "flat_blockers_included": blocker_count,
        "valid_free_cells_raw": int(valid.sum()),
        "valid_area_m2": float(valid.sum() * CANVAS_RESOLUTION**2),
        "structural_bounds_slam": structural_bounds,
        "mapex_start_pose_rc": [int(mapex_start[0]), int(mapex_start[1])],
        "connected_components_in_bbox": int(num_labels),
        "valid_touches_bbox": touches_bbox,
    }
    (map_dir / "hospital_map_metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )
    return map_dir, mapex_start, metadata


def torch_cuda_available() -> bool:
    try:
        import torch
        return bool(torch.cuda.is_available())
    except Exception:
        return False


def write_mapex_config(
    mapex_root: Path,
    methods: Sequence[str],
    start_pose: Tuple[int, int],
    mission_time: int,
    output_folder_name: str,
    device: str,
) -> str:
    """Return exact config text compatible with upstream scripts/explore.py."""
    methods_yaml = "[" + ", ".join(repr(m) for m in methods) + "]"
    return f"""# Auto-generated temporarily by hospital_mapex_benchmark.py
root_path: '{mapex_root.as_posix()}/'
num_data_per_world: 1
test_world_only: false
train_test_world_split_path: 'data_factory/train_test_worlds_split_1127.json'
collect_world_list: ['{WORLD_ID}']
output_folder_name: '{output_folder_name}'

start_pose: [{start_pose[0]}, {start_pose[1]}]
cur_pose_dist_threshold_m: 1

modes_to_test: {methods_yaml}

unknown_as_occ: true
use_distance_transform_for_planning: true
upen_config:
  rrt_max_iters: 2500
  expand_dis: 5
  goal_sample_rate: -1
  connect_circle_dist: 20
  rrt_num_path: 1000
  rrt_straight_line: false
  reach_horizon: 50
  goal_pose_freq: 10

lidar_sim_configs:
  laser_range_m: 20
  num_laser: 2500
  pixel_per_meter: 10
  dilate_diam_for_planning: 3

pred_vis_configs:
  laser_range_m: 20
  pixel_per_meter: 10
  num_laser: 250

mission_time: {mission_time}
show_plt_freq: 50
max_workers: 1

ensemble_folder_name: 'weights/lama_ensemble'
n_spatial_classes: 3
map_loss_scale: 1.0

big_lama_model_folder_name: 'weights/big_lama'
lama_transform_variant: 'default_map_eval'
lama_device: '{device}'
lama_out_size: (512, 512)
"""


def run_mapex_exploration(
    mapex_root: Path,
    methods: Sequence[str],
    start_pose: Tuple[int, int],
    mission_time: int,
    output_folder_name: str,
    device: str,
) -> Dict[str, Path]:
    config_path = mapex_root / "configs" / "base.yaml"
    if not config_path.exists():
        raise RuntimeError(f"MapEx config missing: {config_path}")
    original = config_path.read_text(encoding="utf-8")
    config_text = write_mapex_config(
        mapex_root, methods, start_pose, mission_time, output_folder_name, device
    )

    before = time.time()
    try:
        config_path.write_text(config_text, encoding="utf-8")
        run_cmd(
            [
                sys.executable,
                "explore.py",
                "--collect_world_list",
                WORLD_ID,
                "--start_pose",
                str(start_pose[0]),
                str(start_pose[1]),
            ],
            cwd=mapex_root / "scripts",
        )
    finally:
        config_path.write_text(original, encoding="utf-8")

    exp_root = mapex_root / output_folder_name
    candidates = []
    if exp_root.exists():
        for d in exp_root.rglob("*"):
            if d.is_dir() and d.stat().st_mtime >= before - 5:
                if WORLD_ID in d.name and (d / "global_obs").exists():
                    candidates.append(d)

    found: Dict[str, Path] = {}
    for method in methods:
        suffix = f"_{method}"
        matches = sorted(
            [d for d in candidates if d.name.endswith(suffix)],
            key=lambda p: p.stat().st_mtime,
        )
        if matches:
            found[method] = matches[-1]

    missing = [m for m in methods if m not in found]
    if missing:
        raise RuntimeError(
            "MapEx run finished but result folders were not found for: "
            + ", ".join(missing)
            + f". Search under {exp_root}"
        )
    return found


def import_mapex_lama_utils(mapex_root: Path):
    scripts_dir = str(mapex_root / "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    import lama_pred_utils  # type: ignore
    return lama_pred_utils


def center_match(arr: np.ndarray, target_hw: Tuple[int, int], fill=0) -> np.ndarray:
    """Center crop/pad first two dimensions to target H,W."""
    th, tw = target_hw
    h, w = arr.shape[:2]
    out_shape = (th, tw) + arr.shape[2:]
    out = np.full(out_shape, fill, dtype=arr.dtype)

    src_r0 = max(0, (h - th) // 2)
    src_c0 = max(0, (w - tw) // 2)
    dst_r0 = max(0, (th - h) // 2)
    dst_c0 = max(0, (tw - w) // 2)
    hh = min(h, th)
    ww = min(w, tw)
    out[dst_r0 : dst_r0 + hh, dst_c0 : dst_c0 + ww] = arr[
        src_r0 : src_r0 + hh, src_c0 : src_c0 + ww
    ]
    return out


def generate_predictions(
    mapex_root: Path,
    result_dirs: Dict[str, Path],
    prediction_stride: int,
    device: str,
) -> None:
    cv2 = require_import("cv2", "python3 -m pip install opencv-python")
    torch = require_import("torch", "Install the MapEx/LAMA conda environment.")
    utils = import_mapex_lama_utils(mapex_root)

    model_path = mapex_root / "pretrained_models" / "weights" / "big_lama"
    log(f"Loading Big-LaMa G from {model_path}")
    model = utils.load_lama_model(str(model_path), device=device)
    transform = utils.get_lama_transform("default_map_eval", (512, 512))

    with torch.no_grad():
        for method, exp_dir in result_dirs.items():
            input_dir = exp_dir / "global_obs"
            output_dir = exp_dir / "global_pred"
            output_dir.mkdir(exist_ok=True)
            obs_files = sorted(input_dir.glob("*.png"))[::prediction_stride]
            log(f"{method}: generating {len(obs_files)} G predictions.")
            for obs_path in obs_files:
                frame = int(obs_path.stem)
                pred_path = output_dir / f"{frame:08d}_pred.npy"
                if pred_path.exists():
                    continue
                obs_bgr = cv2.imread(str(obs_path), cv2.IMREAD_COLOR)
                if obs_bgr is None:
                    raise RuntimeError(f"Could not read {obs_path}")
                obs_one = cv2.cvtColor(obs_bgr, cv2.COLOR_BGR2RGB)[:, :, 0]
                three = np.stack([obs_one, obs_one, obs_one], axis=2)
                input_batch, lama_mask = utils.convert_obsimg_to_model_input(
                    three, transform, device
                )
                pred = model(input_batch)
                pred_viz = utils.visualize_prediction(pred, lama_mask)
                np.save(pred_path, pred_viz)


def block_reduce_max_2(a: np.ndarray) -> np.ndarray:
    hpad = (-a.shape[0]) % 2
    wpad = (-a.shape[1]) % 2
    p = np.pad(a, ((0, hpad), (0, wpad)), constant_values=0)
    return p.reshape(p.shape[0] // 2, 2, p.shape[1] // 2, 2).max(axis=(1, 3))


def load_valid_reduced(map_dir: Path) -> np.ndarray:
    valid = np.load(map_dir / "valid_space.npy") > 0
    return block_reduce_max_2(valid.astype(np.uint8)).astype(bool)


def load_gt_occupied_from_experiment(exp_dir: Path) -> np.ndarray:
    cv2 = require_import("cv2", "python3 -m pip install opencv-python")
    gt_path = exp_dir / "gt_map.png"
    if not gt_path.exists():
        raise RuntimeError(f"MapEx gt_map.png missing: {gt_path}")
    gt = cv2.imread(str(gt_path), cv2.IMREAD_COLOR)
    if gt is None:
        raise RuntimeError(f"Could not read {gt_path}")
    # Upstream gt_map visualization encodes occupied as bright/white after its mask utils.
    # Match the exact occupied-IoU convention used by calc_metrics_subdirectory.py:
    # prediction > 0.5 (after normalization) versus GT > 0.5.
    return gt[:, :, 0] > 128


def crop_padding(a: np.ndarray) -> np.ndarray:
    if a.shape[0] <= 2 * PADDING or a.shape[1] <= 2 * PADDING:
        raise RuntimeError(f"Array {a.shape} is too small for MapEx {PADDING}px padding.")
    return a[PADDING:-PADDING, PADDING:-PADDING]


def calculate_coverage_curve(
    exp_dir: Path,
    valid_reduced: np.ndarray,
    stride: int,
) -> np.ndarray:
    cv2 = require_import("cv2", "python3 -m pip install opencv-python")
    rows = []
    for obs_path in sorted((exp_dir / "global_obs").glob("*.png"))[::stride]:
        obs = cv2.imread(str(obs_path), cv2.IMREAD_COLOR)
        if obs is None:
            continue
        obs_crop = crop_padding(obs)
        valid = center_match(valid_reduced.astype(np.uint8), obs_crop.shape[:2]).astype(bool)
        known = obs_crop[:, :, 0] != 128
        denom = int(valid.sum())
        cov = float((known & valid).sum() / denom) if denom else float("nan")
        rows.append((int(obs_path.stem), cov))
    return np.asarray(rows, dtype=float)


def calculate_iou_curve(exp_dir: Path) -> np.ndarray:
    gt_occ_full = load_gt_occupied_from_experiment(exp_dir)
    gt_occ = crop_padding(gt_occ_full)
    rows = []
    pred_files = sorted((exp_dir / "global_pred").glob("*_pred.npy"))
    for pred_path in pred_files:
        frame = int(pred_path.name.split("_")[0])
        pred = np.load(pred_path)
        pred = center_match(pred, gt_occ_full.shape[:2], fill=0)
        pred_crop = crop_padding(pred)
        pred_occ = pred_crop[:, :, 0] > 128
        gt = center_match(gt_occ.astype(np.uint8), pred_occ.shape[:2]).astype(bool)
        tp = np.logical_and(pred_occ, gt).sum()
        union = np.logical_or(pred_occ, gt).sum()
        iou = float(tp / union) if union else 0.0
        rows.append((frame, iou))
    return np.asarray(rows, dtype=float)


def load_mapex_gt_utils(mapex_root: Path, map_dir: Path):
    scripts_dir = str(mapex_root / "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    import sim_utils  # type: ignore
    occ_map, valid_map = sim_utils.get_kth_occ_validspace_map(
        str(map_dir / "occ_map.npy"),
        str(map_dir / "valid_space.npy"),
    )
    return occ_map, valid_map


def sample_tu_goals(valid_map: np.ndarray, count: int, seed: int) -> np.ndarray:
    coords = np.argwhere(valid_map == 1)
    if len(coords) < count:
        count = len(coords)
    rng = np.random.default_rng(seed)
    inds = rng.choice(len(coords), size=count, replace=False)
    return coords[inds]


def calculate_tu_curve(
    mapex_root: Path,
    map_dir: Path,
    exp_dir: Path,
    start_pose: Tuple[int, int],
    tu_goals: int,
    seed: int,
    timestep_step: int,
) -> np.ndarray:
    pyastar2d = require_import("pyastar2d", "python3 -m pip install pyastar2d")
    gt_map, valid_map = load_mapex_gt_utils(mapex_root, map_dir)
    goals = sample_tu_goals(valid_map, tu_goals, seed)

    pred_by_frame = {}
    for p in (exp_dir / "global_pred").glob("*_pred.npy"):
        pred_by_frame[int(p.name.split("_")[0])] = p
    available = sorted(pred_by_frame)
    target_steps = [t for t in range(timestep_step, max(available, default=0) + 1, timestep_step)]
    rows = []

    for target in target_steps:
        # Use nearest generated prediction to the paper-style target timestep,
        # but only if no more than one prediction stride away.
        if not available:
            continue
        frame = min(available, key=lambda f: abs(f - target))
        pred = np.load(pred_by_frame[frame])
        pred = center_match(pred, gt_map.shape[:2], fill=0)
        pred_occ = pred[:, :, 0] > 128

        cost = np.ones(pred_occ.shape, dtype=np.float32)
        cost[pred_occ] = np.inf

        succeed = 0
        total = 0
        for goal in goals:
            try:
                path = pyastar2d.astar_path(
                    cost,
                    np.asarray(start_pose, dtype=np.int32),
                    np.asarray(goal, dtype=np.int32),
                    allow_diagonal=False,
                )
            except Exception:
                path = None
            total += 1
            if path is None:
                continue
            path = np.asarray(path, dtype=int)
            if np.any(gt_map[path[:, 0], path[:, 1]] == 1):
                continue
            succeed += 1
        rows.append((target, succeed / total if total else float("nan")))
    return np.asarray(rows, dtype=float)


def save_curve(path: Path, curve: np.ndarray, metric: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timestep", metric])
        for row in curve:
            w.writerow([int(row[0]), float(row[1])])


def normalized_auc(curve: np.ndarray, horizon: Optional[int] = None) -> float:
    if curve.size == 0:
        return float("nan")
    c = curve[np.argsort(curve[:, 0])]
    if horizon is not None:
        c = c[c[:, 0] <= horizon]
    if len(c) == 0:
        return float("nan")
    if len(c) == 1:
        return float(c[0, 1])
    x, y = c[:, 0], c[:, 1]
    span = float(x[-1] - x[0])
    if span <= 0:
        return float(np.nanmean(y))
    return float(np.trapz(y, x) / span)


def value_at_or_before(curve: np.ndarray, horizon: int) -> float:
    c = curve[curve[:, 0] <= horizon]
    if len(c) == 0:
        return float("nan")
    return float(c[np.argmax(c[:, 0]), 1])


def travel_distance_m(exp_dir: Path) -> float:
    odom_path = exp_dir / "odom.npy"
    if not odom_path.exists():
        return float("nan")
    odom = np.load(odom_path)
    if len(odom) < 2:
        return 0.0
    # MapEx simulation is 10 pixels/m after get_kth_occ_validspace_map.
    return float(np.linalg.norm(np.diff(odom[:, :2], axis=0), axis=1).sum() / PIXELS_PER_METER)


def summarize_metrics(
    curves: Dict[str, Dict[str, np.ndarray]],
    result_dirs: Dict[str, Path],
    methods: Sequence[str],
    output_root: Path,
) -> Tuple[List[dict], dict]:
    # Coverage and IoU should be compared over the same closed-loop horizon.
    per_method_max = {}
    for method in methods:
        maxima = []
        for metric in ("coverage", "iou"):
            c = curves[method][metric]
            if len(c):
                maxima.append(int(c[:, 0].max()))
        if maxima:
            per_method_max[method] = min(maxima)
    if not per_method_max:
        raise RuntimeError("No Coverage/IoU curves were produced.")
    common_horizon = min(per_method_max.values())

    tu_max = {
        m: int(curves[m]["tu"][:, 0].max())
        for m in methods
        if len(curves[m]["tu"])
    }
    common_tu_horizon = min(tu_max.values()) if tu_max else 0

    rows = []
    for method in methods:
        cov = curves[method]["coverage"]
        iou = curves[method]["iou"]
        tu = curves[method]["tu"]
        rows.append(
            {
                "method": METHOD_LABELS.get(method, method),
                "mode": method,
                "coverage_auc": normalized_auc(cov, common_horizon),
                "iou_auc": normalized_auc(iou, common_horizon),
                "tu_auc": normalized_auc(tu, common_tu_horizon) if common_tu_horizon else float("nan"),
                "coverage_at_common_horizon": value_at_or_before(cov, common_horizon),
                "iou_at_common_horizon": value_at_or_before(iou, common_horizon),
                "tu_at_common_horizon": value_at_or_before(tu, common_tu_horizon) if common_tu_horizon else float("nan"),
                "travel_distance_m": travel_distance_m(result_dirs[method]),
                "common_horizon": common_horizon,
                "common_tu_horizon": common_tu_horizon,
            }
        )

    output_root.mkdir(parents=True, exist_ok=True)
    summary_path = output_root / "summary.csv"
    fields = list(rows[0].keys())
    with summary_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    nearest = next((r for r in rows if r["mode"] == "nearest"), None)
    improvement_rows = []
    if nearest:
        for row in rows:
            imp = {"method": row["method"], "mode": row["mode"]}
            for key in ("coverage_auc", "iou_auc", "tu_auc"):
                base = float(nearest[key])
                val = float(row[key])
                imp[f"{key}_improvement_vs_nearest_pct"] = (
                    (val / base - 1.0) * 100.0
                    if np.isfinite(base) and base != 0 and np.isfinite(val)
                    else float("nan")
                )
            improvement_rows.append(imp)
        with (output_root / "improvement_vs_nearest.csv").open(
            "w", newline="", encoding="utf-8"
        ) as f:
            fields2 = list(improvement_rows[0].keys())
            w = csv.DictWriter(f, fieldnames=fields2)
            w.writeheader()
            w.writerows(improvement_rows)

    return rows, {
        "common_horizon": common_horizon,
        "common_tu_horizon": common_tu_horizon,
    }


def print_summary(rows: List[dict]) -> None:
    print("\n=== Hospital MapEx benchmark ===")
    print(
        f"{'Method':<12} {'Coverage AUC':>13} {'IoU AUC':>10} "
        f"{'TU AUC':>10} {'Travel(m)':>10}"
    )
    for r in rows:
        print(
            f"{r['method']:<12} "
            f"{r['coverage_auc']:>13.4f} "
            f"{r['iou_auc']:>10.4f} "
            f"{r['tu_auc']:>10.4f} "
            f"{r['travel_distance_m']:>10.2f}"
        )


def preflight_mapex(mapex_root: Path, methods: Sequence[str], device: str) -> None:
    required = [
        mapex_root / "scripts" / "explore.py",
        mapex_root / "scripts" / "sim_utils.py",
        mapex_root / "scripts" / "lama_pred_utils.py",
        mapex_root / "configs" / "base.yaml",
    ]
    missing = [p for p in required if not p.exists()]
    if missing:
        raise RuntimeError("Incomplete MapEx checkout:\n" + "\n".join(map(str, missing)))
    if not weights_ready(mapex_root):
        print_weight_help(mapex_root)
        raise RuntimeError("Official MapEx checkpoints are required.")

    # range_libc is imported by sim_utils before any planner runs.
    require_import(
        "range_libc",
        f"cd {mapex_root / 'range_libc' / 'pywrapper'} && python3 setup.py install",
    )
    require_import("pyastar2d", "python3 -m pip install pyastar2d")

    if "upen" in methods and not torch_cuda_available():
        raise RuntimeError(
            "UPEN in the upstream MapEx release uses CUDA-specific calls. "
            "For the faithful four-method benchmark, activate the MapEx/LAMA CUDA "
            "environment. Or temporarily omit UPEN with "
            "--methods nearest hectoraug visvarprob."
        )
    if device.startswith("cuda") and not torch_cuda_available():
        raise RuntimeError(
            f"Requested device '{device}', but torch.cuda.is_available() is False."
        )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Run paper-style MapEx/baseline benchmark on AWS Hospital."
    )
    p.add_argument(
        "--setup",
        action="store_true",
        help="Clone MapEx and AWS Hospital repositories if absent.",
    )
    p.add_argument(
        "--download-weights",
        action="store_true",
        help="Download official MapEx KTH weights using gdown.",
    )
    p.add_argument(
        "--run",
        action="store_true",
        help="Run exploration, predictions, and all metrics.",
    )
    p.add_argument(
        "--prepare-only",
        action="store_true",
        help="Only build the Hospital structural map and print the MapEx start pose.",
    )
    p.add_argument(
        "--mapex-root",
        type=Path,
        default=DEFAULT_MAPEX_ROOT,
    )
    p.add_argument(
        "--aws-root",
        type=Path,
        default=DEFAULT_AWS_ROOT,
    )
    p.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
    )
    p.add_argument(
        "--methods",
        nargs="+",
        default=list(DEFAULT_METHODS),
        choices=list(METHOD_LABELS),
    )
    p.add_argument("--mission-time", type=int, default=1000)
    p.add_argument("--prediction-stride", type=int, default=50)
    p.add_argument("--tu-step", type=int, default=100)
    p.add_argument("--tu-goals", type=int, default=100)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--spawn-x", type=float, default=DEFAULT_SPAWN_X)
    p.add_argument("--spawn-y", type=float, default=DEFAULT_SPAWN_Y)
    p.add_argument("--spawn-yaw", type=float, default=DEFAULT_SPAWN_YAW)
    p.add_argument("--slice-z", type=float, default=0.30)
    p.add_argument("--wall-thickness-cells", type=int, default=2)
    p.add_argument(
        "--no-flat-blockers",
        action="store_true",
        help="Do not rasterize the two elevator blocker boxes from hospital_aws_flat.sdf.",
    )
    p.add_argument(
        "--reuse-results",
        type=Path,
        default=None,
        help=(
            "Skip exploration and use an existing MapEx experiment parent directory. "
            "It must contain one result folder per requested method."
        ),
    )
    return p.parse_args()


def discover_reuse_results(parent: Path, methods: Sequence[str]) -> Dict[str, Path]:
    found = {}
    dirs = [p for p in parent.rglob("*") if p.is_dir() and (p / "global_obs").exists()]
    for method in methods:
        matches = [p for p in dirs if WORLD_ID in p.name and p.name.endswith(f"_{method}")]
        if matches:
            found[method] = sorted(matches, key=lambda p: p.stat().st_mtime)[-1]
    missing = [m for m in methods if m not in found]
    if missing:
        raise RuntimeError(
            f"--reuse-results could not find {missing} under {parent}"
        )
    return found


def main() -> int:
    args = parse_args()
    mapex_root = args.mapex_root.resolve()
    aws_root = args.aws_root.resolve()
    output_root = args.output_root.resolve()
    methods = tuple(args.methods)

    # If user gave no action flag, make the useful behavior explicit but safe:
    # setup + prepare. Full expensive benchmark still requires --run.
    if not (args.setup or args.download_weights or args.run or args.prepare_only):
        args.setup = True
        args.prepare_only = True

    if args.setup:
        clone_if_missing(mapex_root, MAPEX_URL, recurse=True)
        clone_if_missing(aws_root, AWS_URL, branch="ros2")

    if args.download_weights:
        if not (mapex_root / ".git").exists():
            raise RuntimeError("Run --setup first or provide --mapex-root.")
        download_weights(mapex_root)

    if not (aws_root / ".git").exists():
        raise RuntimeError(
            f"AWS Hospital checkout not found at {aws_root}. Run with --setup."
        )
    if not (mapex_root / ".git").exists():
        raise RuntimeError(f"MapEx checkout not found at {mapex_root}. Run with --setup.")

    map_dir, start_pose, gt_meta = build_hospital_map(
        PROJECT_ROOT,
        mapex_root,
        aws_root,
        args.spawn_x,
        args.spawn_y,
        args.spawn_yaw,
        args.slice_z,
        args.wall_thickness_cells,
        include_flat_blockers=not args.no_flat_blockers,
    )
    log(f"Generated Hospital MapEx input at: {map_dir}")
    log(f"MapEx start_pose [row,col] = {list(start_pose)}")

    output_root.mkdir(parents=True, exist_ok=True)
    metadata = {
        "script": str(Path(__file__).relative_to(PROJECT_ROOT)),
        "methods": list(methods),
        "method_labels": {m: METHOD_LABELS[m] for m in methods},
        "mission_time": args.mission_time,
        "prediction_stride": args.prediction_stride,
        "tu_step": args.tu_step,
        "tu_goals": args.tu_goals,
        "seed": args.seed,
        "device": args.device,
        "hospital_gt": gt_meta,
        "mapex_reference_settings": {
            "lidar_range_m": 20,
            "lidar_num_rays": 2500,
            "predicted_visibility_range_m": 20,
            "predicted_visibility_num_rays": 250,
            "map_resolution_m_per_pixel": 0.10,
        },
    }

    if args.prepare_only and not args.run:
        (output_root / "benchmark_metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
        log("Prepare-only complete.")
        return 0

    if not args.run:
        return 0

    preflight_mapex(mapex_root, methods, args.device)

    if args.reuse_results:
        result_dirs = discover_reuse_results(args.reuse_results.resolve(), methods)
    else:
        result_dirs = run_mapex_exploration(
            mapex_root,
            methods,
            start_pose,
            args.mission_time,
            "experiments_hospital",
            args.device,
        )

    metadata["result_dirs"] = {m: str(p) for m, p in result_dirs.items()}

    generate_predictions(
        mapex_root,
        result_dirs,
        args.prediction_stride,
        args.device,
    )

    valid_reduced = load_valid_reduced(map_dir)
    curves: Dict[str, Dict[str, np.ndarray]] = {}
    curve_dir = output_root / "curves"
    for method in methods:
        exp_dir = result_dirs[method]
        log(f"Computing metrics: {METHOD_LABELS[method]}")
        coverage = calculate_coverage_curve(
            exp_dir, valid_reduced, args.prediction_stride
        )
        iou = calculate_iou_curve(exp_dir)
        tu = calculate_tu_curve(
            mapex_root,
            map_dir,
            exp_dir,
            start_pose,
            args.tu_goals,
            args.seed,
            args.tu_step,
        )
        curves[method] = {"coverage": coverage, "iou": iou, "tu": tu}
        save_curve(curve_dir / f"{method}_coverage.csv", coverage, "coverage")
        save_curve(curve_dir / f"{method}_iou.csv", iou, "occupied_iou")
        save_curve(curve_dir / f"{method}_tu.csv", tu, "tu_success_rate")

    rows, horizon_meta = summarize_metrics(curves, result_dirs, methods, output_root)
    metadata.update(horizon_meta)
    metadata["metric_notes"] = {
        "coverage": "known valid-space fraction along each method's own closed-loop trajectory",
        "iou": "occupied-class IoU of Big-LaMa G prediction versus structural GT",
        "tu": "A* success rate over fixed random valid-space goals; predicted path must also be collision-free in GT",
        "auc": "trapezoidal area normalized by compared timestep span",
    }
    (output_root / "benchmark_metadata.json").write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )

    print_summary(rows)
    log(f"Saved benchmark outputs to: {output_root}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise
