#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
import platform
import shutil
import socket
import subprocess
import tempfile
import zipfile
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import generate_new_room_ground_truth as gtref

GENERATOR_ID = "MX048_ORTHO_NEW_ROOM_V1"
SCHEMA_VERSION = "MX048_LAYOUT_V1"
LAYOUT_SEEDS = tuple(range(49001, 49013))
PRIMARY_DEVELOPMENT = tuple(range(49001, 49007))
PRIMARY_CONFIRMATION = tuple(range(49007, 49011))
RESERVE_DEVELOPMENT = (49011,)
RESERVE_CONFIRMATION = (49012,)
RUN_SEEDS = (1, 2)
MAX_ATTEMPTS = 64

MX045_METHOD_COMMIT = "01fcdd328d106c084cedefbc05a9ec7478758c77"
MX045_METHOD_BLOB = "96286887e76676ffa2fd13c1fb19b974d2bf57de"
MX048_CONTRACT_COMMIT = "928be24d768f821f20155864d641e78bf9b31dcd"
MX048_CONTRACT_BLOB = "731c53555318efa079629485b3622d54a2660c84"
MX049_ADDENDUM_COMMIT = "bdaaceb10b7f4a102e6e1b5dc1a9bf3d27cdf0a8"
MX049_ADDENDUM_BLOB = "dcdffdbf4e3c6d62a845ea2c32e6a098cdc7bb0a"
COHORT_ID = "MX049_FRESH_DELL_V1"
NAMESPACE = "mx049_fresh_dell"

BASE_WORLD_REL = "mapex_lab/map/new_room.sdf"
GENERATED_WORLD_REL = "mapex_lab/map/generated/mx049_fresh_dell/layout_{seed}/world.sdf"
GEOMETRY_REL = "mapex_lab/map/generated/mx049_fresh_dell/layout_{seed}/geometry_params.json"
LAYOUT_CONFIG_REL = "mapex_lab/map/generated/mx049_fresh_dell/layout_{seed}/layout_config.json"
GEN_AUDIT_REL = "mapex_lab/map/generated/mx049_fresh_dell/layout_{seed}/generation_audit.json"
GEN_RUNTIME_REL = "mapex_lab/map/generated/mx049_fresh_dell/layout_{seed}/generation_runtime_provenance.json"
GT_REL = "mapex_lab/ground_truth/mx049_fresh_dell/layout_{seed}/sealed/structural_gt_v2.npz"
ROI_REL = "mapex_lab/ground_truth/mx049_fresh_dell/layout_{seed}/sealed/connected_free_roi_v2.npy"
GT_SUMMARY_REL = "mapex_lab/ground_truth/mx049_fresh_dell/layout_{seed}/sealed/structural_gt_v2_summary.json"
GT_BINDING_REL = "mapex_lab/ground_truth/mx049_fresh_dell/layout_{seed}/sealed/gt_binding.json"

NPY_VERSION = (2, 0)
ZIP_TIMESTAMP = (1980, 1, 1, 0, 0, 0)
ZIP_EXTERNAL_ATTR = (0o100644 << 16)

PARAMETER_ORDER = [
    "y_top_cm", "y_bottom_cm",
    "top_left_door_center_x_cm", "top_left_door_width_cm",
    "top_center_door_center_x_cm", "top_center_door_width_cm",
    "top_right_door_center_x_cm", "top_right_door_width_cm",
    "bottom_left_door_center_x_cm", "bottom_left_door_width_cm",
    "bottom_center_door_center_x_cm", "bottom_center_door_width_cm",
    "bottom_right_door_center_x_cm", "bottom_right_door_width_cm",
    "x_top_left_cm", "x_top_right_cm", "x_bottom_left_cm", "x_bottom_right_cm",
    "top_left_divider_door_center_y_cm", "top_left_divider_door_width_cm",
    "top_right_divider_door_center_y_cm", "top_right_divider_door_width_cm",
    "bottom_left_divider_door_center_y_cm", "bottom_left_divider_door_width_cm",
    "bottom_right_divider_door_center_y_cm", "bottom_right_divider_door_width_cm",
]
for i in range(1, 9):
    PARAMETER_ORDER.extend([
        f"obs{i:02d}_width_x_cm", f"obs{i:02d}_width_y_cm",
        f"obs{i:02d}_x_cm", f"obs{i:02d}_y_cm",
    ])

@dataclass(frozen=True)
class Rect:
    xmin: int
    xmax: int
    ymin: int
    ymax: int
    name: str
    kind: str

    def contained_in(self, bounds: tuple[int, int, int, int]) -> bool:
        x0, x1, y0, y1 = bounds
        return self.xmin >= x0 and self.xmax <= x1 and self.ymin >= y0 and self.ymax <= y1

@dataclass(frozen=True)
class Box:
    name: str
    cx2: int
    cy2: int
    sx: int
    sy: int
    sz_cm: int
    kind: str

    @property
    def rect(self) -> Rect:
        return Rect(
            (self.cx2 - self.sx) // 2,
            (self.cx2 + self.sx) // 2,
            (self.cy2 - self.sy) // 2,
            (self.cy2 + self.sy) // 2,
            self.name,
            self.kind,
        )

class SplitMix64:
    MASK = (1 << 64) - 1

    def __init__(self, state: int):
        self.state = state & self.MASK

    def next_u64(self) -> int:
        self.state = (self.state + 0x9E3779B97F4A7C15) & self.MASK
        z = self.state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & self.MASK
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & self.MASK
        return (z ^ (z >> 31)) & self.MASK

    def grid(self, lo: int, hi: int, step: int) -> int:
        if step <= 0 or hi < lo or (hi - lo) % step:
            raise ValueError(f"invalid grid {lo}..{hi} step={step}")
        n = (hi - lo) // step + 1
        return lo + step * (self.next_u64() % n)

def canonical_json_bytes(obj: object) -> bytes:
    return (json.dumps(
        obj, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False
    ) + "\n").encode("utf-8")

def write_canonical_json(path: Path, obj: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_json_bytes(obj))

def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()

def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()

def git_value(root: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()

def split_role(seed: int) -> tuple[str, str]:
    if seed in PRIMARY_DEVELOPMENT:
        return "development", "primary"
    if seed in PRIMARY_CONFIRMATION:
        return "confirmation", "primary"
    if seed in RESERVE_DEVELOPMENT:
        return "development", "reserve"
    if seed in RESERVE_CONFIRMATION:
        return "confirmation", "reserve"
    raise ValueError(f"unknown layout seed {seed}")

def candidate_key(seed: int, attempt: int) -> bytes:
    return f"MX048_LAYOUT_V1|{seed}|{attempt}".encode("ascii")

def candidate_rng(seed: int, attempt: int) -> SplitMix64:
    digest = hashlib.sha256(candidate_key(seed, attempt)).digest()
    return SplitMix64(int.from_bytes(digest[:8], "big", signed=False))

def ceil10(x: int) -> int:
    return 10 * math.ceil(x / 10)

def floor10(x: int) -> int:
    return 10 * math.floor(x / 10)

def draw_room_center(rng: SplitMix64, lo: int, hi: int) -> int:
    lo2 = ceil10(lo + 100)
    hi2 = floor10(hi - 100)
    if hi2 < lo2:
        raise ValueError("V1_PARAMETER_DOMAIN")
    return rng.grid(lo2, hi2, 10)

def draw_candidate(seed: int, attempt: int) -> dict[str, int]:
    rng = candidate_rng(seed, attempt)
    p: dict[str, int] = {}
    p["y_top_cm"] = rng.grid(380, 460, 10)
    p["y_bottom_cm"] = rng.grid(-460, -380, 10)
    for side, lo, hi in (("left", -900, -650), ("center", -150, 150), ("right", 650, 900)):
        p[f"top_{side}_door_center_x_cm"] = rng.grid(lo, hi, 10)
        p[f"top_{side}_door_width_cm"] = rng.grid(150, 190, 10)
    for side, lo, hi in (("left", -900, -650), ("center", -150, 150), ("right", 650, 900)):
        p[f"bottom_{side}_door_center_x_cm"] = rng.grid(lo, hi, 10)
        p[f"bottom_{side}_door_width_cm"] = rng.grid(150, 190, 10)

    p["x_top_left_cm"] = rng.grid(-750, -550, 10)
    p["x_top_right_cm"] = rng.grid(550, 750, 10)
    p["x_bottom_left_cm"] = rng.grid(-750, -550, 10)
    p["x_bottom_right_cm"] = rng.grid(550, 750, 10)

    for name, lo, hi in (
        ("top_left", 620, 820), ("top_right", 620, 820),
        ("bottom_left", -820, -620), ("bottom_right", -820, -620),
    ):
        p[f"{name}_divider_door_center_y_cm"] = rng.grid(lo, hi, 10)
        p[f"{name}_divider_door_width_cm"] = rng.grid(150, 190, 10)

    yt, yb = p["y_top_cm"], p["y_bottom_cm"]
    xtl, xtr, xbl, xbr = (
        p["x_top_left_cm"], p["x_top_right_cm"],
        p["x_bottom_left_cm"], p["x_bottom_right_cm"],
    )
    rooms = [
        (-1291, xtl - 9, yt + 9, 991),
        (xtl + 9, xtr - 9, yt + 9, 991),
        (xtr + 9, 1291, yt + 9, 991),
        (-1291, xbl - 9, -991, yb - 9),
        (xbl + 9, xbr - 9, -991, yb - 9),
        (xbr + 9, 1291, -991, yb - 9),
    ]
    for idx, (x0, x1, y0, y1) in enumerate(rooms, start=1):
        p[f"obs{idx:02d}_width_x_cm"] = rng.grid(60, 120, 10)
        p[f"obs{idx:02d}_width_y_cm"] = rng.grid(60, 120, 10)
        p[f"obs{idx:02d}_x_cm"] = draw_room_center(rng, x0, x1)
        p[f"obs{idx:02d}_y_cm"] = draw_room_center(rng, y0, y1)

    for idx, xlo, xhi in ((7, -1050, -450), (8, 450, 1050)):
        p[f"obs{idx:02d}_width_x_cm"] = rng.grid(60, 120, 10)
        p[f"obs{idx:02d}_width_y_cm"] = rng.grid(60, 120, 10)
        p[f"obs{idx:02d}_x_cm"] = rng.grid(xlo, xhi, 10)
        p[f"obs{idx:02d}_y_cm"] = rng.grid(-250, 100, 10)
    return p

def parameter_text(params: dict[str, int]) -> bytes:
    return "".join(f"{k}={int(params[k])}\n" for k in PARAMETER_ORDER).encode("ascii")

def geometry_digest(params: dict[str, int]) -> str:
    return sha256_bytes(parameter_text(params))

def wall_segments(parent_lo: int, parent_hi: int, gaps: list[tuple[int, int]]) -> list[tuple[int, int]]:
    ordered = sorted((c - w // 2, c + w // 2) for c, w in gaps)
    out: list[tuple[int, int]] = []
    cursor = parent_lo
    for g0, g1 in ordered:
        if g0 <= parent_lo or g1 >= parent_hi or g0 >= g1:
            raise ValueError("V2_DOOR_VALIDITY")
        if g0 < cursor:
            raise ValueError("V2_DOOR_VALIDITY")
        if g0 - cursor < 50:
            raise ValueError("V2_DOOR_VALIDITY")
        out.append((cursor, g0))
        cursor = g1
    if parent_hi - cursor < 50:
        raise ValueError("V2_DOOR_VALIDITY")
    out.append((cursor, parent_hi))
    return out

def build_boxes(params: dict[str, int]) -> tuple[list[Box], list[tuple[int, int, int, int]]]:
    boxes: list[Box] = []
    # cx2/cy2 are twice-centimeters so half-centimeter segment centers remain exact.
    boxes.extend([
        Box("outer_north", 0, 2000, 2600, 18, 250, "wall"),
        Box("outer_south", 0, -2000, 2600, 18, 250, "wall"),
        Box("outer_west", -2600, 0, 18, 2000, 250, "wall"),
        Box("outer_east", 2600, 0, 18, 2000, 250, "wall"),
    ])
    for label, y in (("top", params["y_top_cm"]), ("bottom", params["y_bottom_cm"])):
        gaps = [
            (params[f"{label}_left_door_center_x_cm"], params[f"{label}_left_door_width_cm"]),
            (params[f"{label}_center_door_center_x_cm"], params[f"{label}_center_door_width_cm"]),
            (params[f"{label}_right_door_center_x_cm"], params[f"{label}_right_door_width_cm"]),
        ]
        for j, (a, b) in enumerate(wall_segments(-1291, 1291, gaps), start=1):
            boxes.append(Box(f"{label}_hseg_{j}", a + b, 2 * y, b - a, 18, 250, "wall"))

    vertical_specs = [
        ("top_left", params["x_top_left_cm"], params["y_top_cm"], 991),
        ("top_right", params["x_top_right_cm"], params["y_top_cm"], 991),
        ("bottom_left", params["x_bottom_left_cm"], -991, params["y_bottom_cm"]),
        ("bottom_right", params["x_bottom_right_cm"], -991, params["y_bottom_cm"]),
    ]
    for label, x, lo, hi in vertical_specs:
        gap = [(params[f"{label}_divider_door_center_y_cm"], params[f"{label}_divider_door_width_cm"])]
        for j, (a, b) in enumerate(wall_segments(lo, hi, gap), start=1):
            boxes.append(Box(f"{label}_vseg_{j}", 2 * x, a + b, 18, b - a, 250, "wall"))

    yt, yb = params["y_top_cm"], params["y_bottom_cm"]
    xtl, xtr, xbl, xbr = (
        params["x_top_left_cm"], params["x_top_right_cm"],
        params["x_bottom_left_cm"], params["x_bottom_right_cm"],
    )
    bounds = [
        (-1291, xtl - 9, yt + 9, 991),
        (xtl + 9, xtr - 9, yt + 9, 991),
        (xtr + 9, 1291, yt + 9, 991),
        (-1291, xbl - 9, -991, yb - 9),
        (xbl + 9, xbr - 9, -991, yb - 9),
        (xbr + 9, 1291, -991, yb - 9),
        (-1291, 1291, yb + 9, yt - 9),
        (-1291, 1291, yb + 9, yt - 9),
    ]
    for i in range(1, 9):
        boxes.append(Box(
            f"obstacle_{i:02d}",
            2 * params[f"obs{i:02d}_x_cm"],
            2 * params[f"obs{i:02d}_y_cm"],
            params[f"obs{i:02d}_width_x_cm"],
            params[f"obs{i:02d}_width_y_cm"],
            120,
            "obstacle",
        ))
    return boxes, bounds

def rect_distance2(a: Rect, b: Rect) -> int:
    dx = max(b.xmin - a.xmax, a.xmin - b.xmax, 0)
    dy = max(b.ymin - a.ymax, a.ymin - b.ymax, 0)
    return dx * dx + dy * dy

def point_rect_distance2(x: int, y: int, r: Rect) -> int:
    dx = max(r.xmin - x, x - r.xmax, 0)
    dy = max(r.ymin - y, y - r.ymax, 0)
    return dx * dx + dy * dy

def validate_v1(params: dict[str, int]) -> None:
    if set(PARAMETER_ORDER) != set(params) or len(params) != len(PARAMETER_ORDER):
        raise ValueError("V1_PARAMETER_DOMAIN")
    # Exact regeneration is the domain oracle.
    for k, v in params.items():
        if not isinstance(v, int):
            raise ValueError("V1_PARAMETER_DOMAIN")

def validate_v2(params: dict[str, int]) -> None:
    for label in ("top", "bottom"):
        gaps = []
        for side in ("left", "center", "right"):
            c = params[f"{label}_{side}_door_center_x_cm"]
            w = params[f"{label}_{side}_door_width_cm"]
            if w < 150:
                raise ValueError("V2_DOOR_VALIDITY")
            gaps.append((c, w))
        wall_segments(-1291, 1291, gaps)
    for label, lo, hi in (
        ("top_left", params["y_top_cm"], 991),
        ("top_right", params["y_top_cm"], 991),
        ("bottom_left", -991, params["y_bottom_cm"]),
        ("bottom_right", -991, params["y_bottom_cm"]),
    ):
        w = params[f"{label}_divider_door_width_cm"]
        if w < 150:
            raise ValueError("V2_DOOR_VALIDITY")
        wall_segments(lo, hi, [(params[f"{label}_divider_door_center_y_cm"], w)])

def validate_v3_v4(params: dict[str, int]) -> tuple[list[Box], list[tuple[int, int, int, int]]]:
    boxes, bounds = build_boxes(params)
    walls = [b.rect for b in boxes if b.kind == "wall"]
    obstacles = [b.rect for b in boxes if b.kind == "obstacle"]
    for i, o in enumerate(obstacles):
        if not o.contained_in(bounds[i]):
            raise ValueError("V3_OBSTACLE_CLEARANCE")
        for w in walls:
            if rect_distance2(o, w) < 1600:
                raise ValueError("V3_OBSTACLE_CLEARANCE")
        for j, q in enumerate(obstacles):
            if i != j and rect_distance2(o, q) < 1600:
                raise ValueError("V3_OBSTACLE_CLEARANCE")
    for b in [x.rect for x in boxes]:
        if point_rect_distance2(0, 300, b) < 75 * 75:
            raise ValueError("V4_SPAWN_CLEARANCE")
    return boxes, bounds

def _fmt_cm2(v2: int) -> str:
    # twice-centimeters -> meters, exact decimal
    sign = "-" if v2 < 0 else ""
    n = abs(v2)
    whole = n // 200
    rem = n % 200
    if rem == 0:
        return f"{sign}{whole}"
    frac = f"{rem * 5:03d}".rstrip("0")
    return f"{sign}{whole}.{frac}"

def _fmt_cm(v: int) -> str:
    return _fmt_cm2(2 * v)

def _box_xml(box: Box, idx: int) -> str:
    z_cm = box.sz_cm // 2
    cx, cy = _fmt_cm2(box.cx2), _fmt_cm2(box.cy2)
    sx, sy, sz = _fmt_cm(box.sx), _fmt_cm(box.sy), _fmt_cm(box.sz_cm)
    z = _fmt_cm(z_cm)
    material = "0.58 0.60 0.64 1" if box.kind == "obstacle" else "0.78 0.78 0.80 1"
    return (
        f'<collision name="mx048_c{idx:03d}"><pose>{cx} {cy} {z} 0 0 0</pose>'
        f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
        f'<visual name="mx048_v{idx:03d}"><pose>{cx} {cy} {z} 0 0 0</pose>'
        f'<geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
        f'<material><ambient>{material}</ambient><diffuse>{material}</diffuse></material></visual>\n'
    )

def render_sdf(repo_root: Path, boxes: list[Box]) -> bytes:
    base = (repo_root / BASE_WORLD_REL).read_text(encoding="utf-8")
    marker = '<model name="mini_hospital_structure">'
    if marker not in base:
        raise ValueError("V9_SDF_IDENTITY")
    prefix = base.split(marker, 1)[0]
    structure = '<model name="mini_hospital_structure"><static>true</static><link name="structure">\n'
    for idx, box in enumerate(boxes):
        structure += _box_xml(box, idx)
    structure += "</link></model></world></sdf>\n"
    return (prefix + structure).encode("utf-8")

def _npy_bytes(value: np.ndarray) -> bytes:
    bio = io.BytesIO()
    np.lib.format.write_array(bio, np.ascontiguousarray(value), version=NPY_VERSION, allow_pickle=False)
    return bio.getvalue()

def _write_npy(path: Path, value: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_npy_bytes(value))

def _write_npz_deterministic(path: Path, ordered: list[tuple[str, np.ndarray]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_STORED, strict_timestamps=True) as zf:
        for name, value in ordered:
            info = zipfile.ZipInfo(f"{name}.npy", ZIP_TIMESTAMP)
            info.compress_type = zipfile.ZIP_STORED
            info.create_system = 3
            info.external_attr = ZIP_EXTERNAL_ATTR
            zf.writestr(info, _npy_bytes(np.asarray(value)))

def semantic_digest(ordered: list[tuple[str, np.ndarray]], source_sdf: str, world_sha: str) -> str:
    h = hashlib.sha256()
    for name, value in ordered:
        a = np.ascontiguousarray(np.asarray(value))
        h.update(name.encode("ascii") + b"\n")
        h.update(str(a.dtype).encode("ascii") + b"\n")
        h.update((",".join(str(x) for x in a.shape)).encode("ascii") + b"\n")
        h.update(a.tobytes(order="C"))
        h.update(b"\n")
    h.update(f"source_sdf={source_sdf}\nworld_sha256={world_sha}\n".encode("utf-8"))
    return h.hexdigest()

def build_gt(repo_root: Path, sdf_path: Path, canonical_sdf: str, out_dir: Path) -> dict:
    boxes_world = gtref._load_collision_boxes(sdf_path, gtref.STRUCTURE_MODEL, gtref.DEFAULT_Z_SLICE_M)
    boxes = gtref._boxes_world_to_slam(
        boxes_world, gtref.DEFAULT_SPAWN_X, gtref.DEFAULT_SPAWN_Y, gtref.DEFAULT_SPAWN_YAW
    )
    occupied = gtref._rasterize_boxes(boxes)
    evaluation_mask, bounds = gtref._building_mask(boxes)
    occupied &= evaluation_mask
    data = np.full((gtref.CANVAS_H, gtref.CANVAS_W), -1, dtype=np.int16)
    data[evaluation_mask] = 0
    data[occupied] = 100
    free = evaluation_mask & ~occupied
    connected, start_cell = gtref._connected_free(free, 0.0, 0.0)
    source_sha = sha256_file(sdf_path)

    fields = [
        ("data", data),
        ("evaluation_mask", evaluation_mask),
        ("resolution", np.asarray(gtref.CANVAS_RES, dtype=np.float64)),
        ("width", np.asarray(gtref.CANVAS_W, dtype=np.int64)),
        ("height", np.asarray(gtref.CANVAS_H, dtype=np.int64)),
        ("origin_x", np.asarray(gtref.CANVAS_X, dtype=np.float64)),
        ("origin_y", np.asarray(gtref.CANVAS_Y, dtype=np.float64)),
        ("canvas_id", np.asarray(gtref.CANVAS_ID)),
        ("ground_truth_id", np.asarray(gtref.GT_ID)),
        ("source_model", np.asarray(gtref.STRUCTURE_MODEL)),
        ("source_sdf", np.asarray(canonical_sdf)),
        ("source_sdf_sha256", np.asarray(source_sha)),
        ("z_slice_m", np.asarray(gtref.DEFAULT_Z_SLICE_M, dtype=np.float64)),
        ("spawn_world_x", np.asarray(gtref.DEFAULT_SPAWN_X, dtype=np.float64)),
        ("spawn_world_y", np.asarray(gtref.DEFAULT_SPAWN_Y, dtype=np.float64)),
        ("spawn_world_yaw", np.asarray(gtref.DEFAULT_SPAWN_YAW, dtype=np.float64)),
        ("roi_seed_slam_x", np.asarray(0.0, dtype=np.float64)),
        ("roi_seed_slam_y", np.asarray(0.0, dtype=np.float64)),
    ]
    gt_path = out_dir / "structural_gt_v2.npz"
    roi_path = out_dir / "connected_free_roi_v2.npy"
    summary_path = out_dir / "structural_gt_v2_summary.json"
    _write_npz_deterministic(gt_path, fields)
    _write_npy(roi_path, connected.astype(bool))

    xmin, xmax, ymin, ymax = bounds
    summary = {
        "canvas": {
            "height_cells": gtref.CANVAS_H, "origin_x": gtref.CANVAS_X,
            "origin_y": gtref.CANVAS_Y, "resolution_m": gtref.CANVAS_RES,
            "width_cells": gtref.CANVAS_W,
        },
        "canvas_id": gtref.CANVAS_ID,
        "coordinate_frame": "slam_start",
        "ground_truth_id": gtref.GT_ID,
        "outputs": {
            "connected_free_roi": canonical_sdf.replace("mapex_lab/map/generated/mx049_fresh_dell/", "mapex_lab/ground_truth/mx049_fresh_dell/").replace("/world.sdf", "/sealed/connected_free_roi_v2.npy"),
            "ground_truth": canonical_sdf.replace("mapex_lab/map/generated/mx049_fresh_dell/", "mapex_lab/ground_truth/mx049_fresh_dell/").replace("/world.sdf", "/sealed/structural_gt_v2.npz"),
        },
        "roi_id": gtref.ROI_ID,
        "roi_seed_slam_xy": [0.0, 0.0],
        "source_model": gtref.STRUCTURE_MODEL,
        "source_sdf": canonical_sdf,
        "source_sdf_sha256": source_sha,
        "spawn_cell_row_col": [int(start_cell[0]), int(start_cell[1])],
        "world_to_slam": {
            "rule": "R(-spawn_yaw) @ (p_world - p_spawn)",
            "spawn_world_x": 0.0, "spawn_world_y": 3.0, "spawn_world_yaw": 0.0,
        },
        "z_slice_m": gtref.DEFAULT_Z_SLICE_M,
        "building_bounds_m": {
            "x_min": xmin, "x_max": xmax, "y_min": ymin, "y_max": ymax,
            "width": xmax - xmin, "height": ymax - ymin,
        },
    }
    write_canonical_json(summary_path, summary)
    return {
        "data": data,
        "evaluation_mask": evaluation_mask,
        "roi": connected.astype(bool),
        "gt_path": gt_path,
        "roi_path": roi_path,
        "summary_path": summary_path,
        "gt_sha256": sha256_file(gt_path),
        "roi_sha256": sha256_file(roi_path),
        "summary_sha256": sha256_file(summary_path),
        "semantic_digest": semantic_digest(fields, canonical_sdf, source_sha),
        "source_sha256": source_sha,
    }

def validate_v5(gt: dict) -> None:
    data = gt["data"]
    row = math.floor((0.0 - gtref.CANVAS_Y) / gtref.CANVAS_RES)
    col = math.floor((0.0 - gtref.CANVAS_X) / gtref.CANVAS_RES)
    if int(data[row, col]) != 0:
        raise ValueError("V5_GT_RASTERIZATION")

def exact_inflated_reach(data: np.ndarray) -> tuple[np.ndarray, np.ndarray, int, int]:
    rows = np.arange(data.shape[0], dtype=np.int64)
    cols = np.arange(data.shape[1], dtype=np.int64)
    y2 = -11415 + 10 * rows
    x2 = -5115 + 10 * cols
    row_sel = np.flatnonzero((-1982 <= y2) & (y2 < 1982))
    col_sel = np.flatnonzero((-2582 <= x2) & (x2 < 2582))
    if row_sel.size == 0 or col_sel.size == 0:
        raise ValueError("V6_CONNECTIVITY")
    r0, r1 = int(row_sel[0]), int(row_sel[-1])
    c0, c1 = int(col_sel[0]), int(col_sel[-1])
    outer = np.zeros(data.shape, dtype=bool)
    outer[np.ix_(row_sel, col_sel)] = True
    occupied = data == 100
    inflated = np.zeros_like(occupied)
    offsets = [
        (dr, dc) for dr in range(-5, 6) for dc in range(-5, 6)
        if 2500 * (dr * dr + dc * dc) <= 83521
    ]
    for dr, dc in offsets:
        src_r0 = max(0, r0 - dr)
        src_r1 = min(data.shape[0] - 1, r1 - dr)
        src_c0 = max(0, c0 - dc)
        src_c1 = min(data.shape[1] - 1, c1 - dc)
        if src_r1 < src_r0 or src_c1 < src_c0:
            continue
        dst_r0, dst_r1 = src_r0 + dr, src_r1 + dr
        dst_c0, dst_c1 = src_c0 + dc, src_c1 + dc
        inflated[dst_r0:dst_r1+1, dst_c0:dst_c1+1] |= occupied[src_r0:src_r1+1, src_c0:src_c1+1]
    free = outer & (data == 0) & ~inflated
    sr = math.floor((0.0 - gtref.CANVAS_Y) / gtref.CANVAS_RES)
    sc = math.floor((0.0 - gtref.CANVAS_X) / gtref.CANVAS_RES)
    if not free[sr, sc]:
        raise ValueError("V6_CONNECTIVITY")
    reach = np.zeros_like(free)
    reach[sr, sc] = True
    q = deque([(sr, sc)])
    h, w = free.shape
    while q:
        r, c = q.popleft()
        for dr, dc in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            nr, nc = r + dr, c + dc
            if not (r0 <= nr <= r1 and c0 <= nc <= c1):
                continue
            if not free[nr, nc] or reach[nr, nc]:
                continue
            if dr and dc and (not free[r + dr, c] or not free[r, c + dc]):
                continue
            reach[nr, nc] = True
            q.append((nr, nc))
    return free, reach, int(np.count_nonzero(free)), int(np.count_nonzero(reach))

def validate_v6(data: np.ndarray) -> dict:
    free, reach, n_free, n_reach = exact_inflated_reach(data)
    if n_free == 0:
        raise ValueError("V6_CONNECTIVITY")
    ratio = n_reach / n_free
    area = n_reach * 0.05 * 0.05
    if ratio < 0.95 or area < 350.0:
        raise ValueError("V6_CONNECTIVITY")
    return {"inflated_free_cells": n_free, "reachable_cells": n_reach, "reachable_fraction": ratio, "reachable_area_m2": area, "reach": reach}

def _zone_mask(params: dict[str, int], zone: str, shape: tuple[int, int]) -> np.ndarray:
    rows = np.arange(shape[0], dtype=np.int64)
    cols = np.arange(shape[1], dtype=np.int64)
    y2 = -11415 + 10 * rows
    x2 = -5115 + 10 * cols
    xtl2 = 2 * params["x_top_left_cm"]
    xtr2 = 2 * params["x_top_right_cm"]
    xbl2 = 2 * params["x_bottom_left_cm"]
    xbr2 = 2 * params["x_bottom_right_cm"]
    yt2 = 2 * params["y_top_cm"]
    yb2 = 2 * params["y_bottom_cm"]
    if zone == "top_left":
        xm = (-2582 <= x2) & (x2 < xtl2 - 18); ym = (yt2 + 18 <= y2) & (y2 < 1982)
    elif zone == "top_center":
        xm = (xtl2 + 18 <= x2) & (x2 < xtr2 - 18); ym = (yt2 + 18 <= y2) & (y2 < 1982)
    elif zone == "top_right":
        xm = (xtr2 + 18 <= x2) & (x2 < 2582); ym = (yt2 + 18 <= y2) & (y2 < 1982)
    elif zone == "center":
        xm = (-2582 <= x2) & (x2 < 2582); ym = (yb2 + 18 <= y2) & (y2 < yt2 - 18)
    elif zone == "bottom_left":
        xm = (-2582 <= x2) & (x2 < xbl2 - 18); ym = (-1982 <= y2) & (y2 < yb2 - 18)
    elif zone == "bottom_center":
        xm = (xbl2 + 18 <= x2) & (x2 < xbr2 - 18); ym = (-1982 <= y2) & (y2 < yb2 - 18)
    elif zone == "bottom_right":
        xm = (xbr2 + 18 <= x2) & (x2 < 2582); ym = (-1982 <= y2) & (y2 < yb2 - 18)
    else:
        raise KeyError(zone)
    return ym[:, None] & xm[None, :]

def validate_v7(params: dict[str, int], reach: np.ndarray) -> dict[str, float]:
    areas: dict[str, float] = {}
    for zone in ("top_left","top_center","top_right","center","bottom_left","bottom_center","bottom_right"):
        area = float(np.count_nonzero(reach & _zone_mask(params, zone, reach.shape)) * 0.0025)
        areas[zone] = area
        threshold = 40.0 if zone == "center" else 8.0
        if area < threshold:
            raise ValueError("V7_ZONE_REACHABILITY")
    return areas

def validate_v9(sdf_path: Path, params: dict[str, int], gt: dict) -> None:
    root = ET.parse(sdf_path).getroot()
    models = [m for m in root.findall(".//model") if m.get("name") == "mini_hospital_structure"]
    if len(models) != 1:
        raise ValueError("V9_SDF_IDENTITY")
    if gt["source_sha256"] != sha256_file(sdf_path):
        raise ValueError("V9_SDF_IDENTITY")

def world_identity_text(generator_blob: str, seed: int, accepted_attempt: int, geom_digest: str, world_sha: str, layout_sha: str) -> bytes:
    fields = [
        ("generator_id", GENERATOR_ID),
        ("generator_blob", generator_blob),
        ("layout_seed", str(seed)),
        ("accepted_attempt", str(accepted_attempt)),
        ("geometry_parameter_digest", geom_digest),
        ("world_sha256", world_sha),
        ("layout_config_sha256", layout_sha),
        ("spawn_x_cm", "0"),
        ("spawn_y_cm", "300"),
        ("spawn_yaw_millirad", "0"),
    ]
    return "".join(f"{k}={v}\n" for k, v in fields).encode("utf-8")

def generate_one(
    repo_root: Path,
    seed: int,
    accepted_digests: set[str],
    generator_commit: str,
    generator_blob: str,
    gt_generator_blob: str,
    old_geometry_digests: set[str] | None = None,
) -> dict:
    split, role = split_role(seed)
    world_rel = GENERATED_WORLD_REL.format(seed=seed)
    geometry_rel = GEOMETRY_REL.format(seed=seed)
    layout_rel = LAYOUT_CONFIG_REL.format(seed=seed)
    audit_rel = GEN_AUDIT_REL.format(seed=seed)
    runtime_rel = GEN_RUNTIME_REL.format(seed=seed)
    gt_rel = GT_REL.format(seed=seed)
    roi_rel = ROI_REL.format(seed=seed)
    summary_rel = GT_SUMMARY_REL.format(seed=seed)
    binding_rel = GT_BINDING_REL.format(seed=seed)
    world_path = repo_root / world_rel
    gt_dir = (repo_root / gt_rel).parent

    attempts: list[dict] = []
    accepted = None
    for attempt in range(MAX_ATTEMPTS):
        rec = {
            "attempt": attempt,
            "candidate_key_sha256": sha256_bytes(candidate_key(seed, attempt)),
            "first_failing_validation_code": None,
        }
        try:
            params = draw_candidate(seed, attempt)
            validate_v1(params)
            validate_v2(params)
            boxes, _bounds = validate_v3_v4(params)
            sdf_bytes = render_sdf(repo_root, boxes)
            with tempfile.TemporaryDirectory(prefix=f"mx049_{seed}_{attempt}_") as tmp:
                tmp_root = Path(tmp)
                tmp_sdf = tmp_root / "world.sdf"
                tmp_sdf.write_bytes(sdf_bytes)
                gt = build_gt(repo_root, tmp_sdf, world_rel, tmp_root / "sealed")
                validate_v5(gt)
                v6 = validate_v6(gt["data"])
                v7 = validate_v7(params, v6["reach"])
                digest = geometry_digest(params)
                if digest in accepted_digests:
                    raise ValueError("V8_GEOMETRY_UNIQUENESS")
                validate_v9(tmp_sdf, params, gt)
                accepted = {
                    "attempt": attempt, "params": params, "boxes": boxes,
                    "sdf_bytes": sdf_bytes, "gt_temp": tmp_root / "sealed",
                    "gt": gt, "geometry_digest": digest,
                    "v6": {k:v for k,v in v6.items() if k != "reach"}, "v7": v7,
                    "tmp_bytes": {
                        "gt": gt["gt_path"].read_bytes(),
                        "roi": gt["roi_path"].read_bytes(),
                        "summary": gt["summary_path"].read_bytes(),
                    },
                }
            rec["validation_results"] = {f"V{i}": "PASS" for i in range(1, 10)}
            attempts.append(rec)
            break
        except ValueError as exc:
            code = str(exc)
            if not code.startswith("V"):
                code = "V1_PARAMETER_DOMAIN"
            rec["first_failing_validation_code"] = code
            attempts.append(rec)
    if accepted is None:
        return {
            "layout_seed": seed, "split": split, "primary_or_reserve": role,
            "status": "LAYOUT_SEED_EXHAUSTED", "accepted_attempt": None, "attempts": attempts,
        }

    geom_digest = accepted["geometry_digest"]
    if old_geometry_digests is not None and geom_digest in old_geometry_digests:
        return {
            "layout_seed": seed,
            "split": split,
            "primary_or_reserve": role,
            "status": "CROSS_COHORT_GEOMETRY_IDENTITY_COLLISION",
            "accepted_attempt": accepted["attempt"],
            "geometry_parameter_digest": geom_digest,
            "old_geometry_match": True,
            "attempts": attempts,
        }

    world_path.parent.mkdir(parents=True, exist_ok=True)
    gt_dir.mkdir(parents=True, exist_ok=True)
    world_path.write_bytes(accepted["sdf_bytes"])
    (repo_root / gt_rel).write_bytes(accepted["tmp_bytes"]["gt"])
    (repo_root / roi_rel).write_bytes(accepted["tmp_bytes"]["roi"])
    (repo_root / summary_rel).write_bytes(accepted["tmp_bytes"]["summary"])

    world_sha = sha256_file(world_path)
    gt_sha = sha256_file(repo_root / gt_rel)
    roi_sha = sha256_file(repo_root / roi_rel)
    geometry_obj = {
        "generator_id": GENERATOR_ID,
        "cohort_id": COHORT_ID,
        "namespace": NAMESPACE,
        "parameters_cm": {k: int(accepted["params"][k]) for k in PARAMETER_ORDER},
    }
    write_canonical_json(repo_root / geometry_rel, geometry_obj)

    layout_cfg = {
        "schema_version": SCHEMA_VERSION,
        "cohort_id": COHORT_ID,
        "namespace": NAMESPACE,
        "layout_seed": seed,
        "split": split,
        "primary_or_reserve": role,
        "accepted_attempt": accepted["attempt"],
        "generator_id": GENERATOR_ID,
        "generator_commit": generator_commit,
        "generator_blob": generator_blob,
        "geometry_parameter_digest": geom_digest,
        "world_path": world_rel,
        "world_sha256": world_sha,
        "gt_path": gt_rel,
        "roi_path": roi_rel,
        "gt_binding_path": binding_rel,
        "gt_sha256": gt_sha,
        "roi_sha256": roi_sha,
        "spawn": {"x_cm": 0, "y_cm": 300, "yaw_millirad": 0},
        "expected_run_seeds": [1, 2],
    }
    write_canonical_json(repo_root / layout_rel, layout_cfg)
    layout_sha = sha256_file(repo_root / layout_rel)
    world_identity = sha256_bytes(world_identity_text(
        generator_blob, seed, accepted["attempt"], geom_digest, world_sha, layout_sha
    ))

    gt_binding = {
        "cohort_id": COHORT_ID,
        "namespace": NAMESPACE,
        "layout_seed": seed,
        "world_path": world_rel,
        "world_sha256": world_sha,
        "generator_id": GENERATOR_ID,
        "generator_commit": generator_commit,
        "generator_blob": generator_blob,
        "gt_generator_commit": generator_commit,
        "gt_generator_blob": gt_generator_blob,
        "spawn": {"x_cm": 0, "y_cm": 300, "yaw_millirad": 0},
        "canvas": {
            "id": gtref.CANVAS_ID, "resolution_m": gtref.CANVAS_RES,
            "width": gtref.CANVAS_W, "height": gtref.CANVAS_H,
            "origin_x": gtref.CANVAS_X, "origin_y": gtref.CANVAS_Y,
        },
        "gt_path": gt_rel, "gt_sha256": gt_sha,
        "roi_path": roi_rel, "roi_sha256": roi_sha,
        "semantic_digest": accepted["gt"]["semantic_digest"],
        "world_identity_sha256": world_identity,
        "sealed": True,
    }
    write_canonical_json(repo_root / binding_rel, gt_binding)

    audit = {
        "cohort_id": COHORT_ID,
        "namespace": NAMESPACE,
        "layout_seed": seed,
        "split": split,
        "primary_or_reserve": role,
        "generator_id": GENERATOR_ID,
        "generator_implementation_commit": generator_commit,
        "generator_implementation_blob": generator_blob,
        "attempts": attempts,
        "accepted_attempt": accepted["attempt"],
        "canonical_parameter_text_sha256": sha256_bytes(parameter_text(accepted["params"])),
        "geometry_parameter_digest": geom_digest,
        "old_geometry_match": False,
        "world_path": world_rel,
        "world_sha256": world_sha,
        "layout_config_path": layout_rel,
        "layout_config_sha256": layout_sha,
        "no_outcome_flag": True,
        "v6": accepted["v6"],
        "v7_zone_area_m2": accepted["v7"],
    }
    write_canonical_json(repo_root / audit_rel, audit)
    runtime = {
        "generation_utc": datetime.now(timezone.utc).isoformat(),
        "hostname": socket.gethostname(),
        "absolute_repo_root": str(repo_root.resolve()),
        "python_version": platform.python_version(),
        "pid": os.getpid(),
    }
    write_canonical_json(repo_root / runtime_rel, runtime)
    accepted_digests.add(geom_digest)
    return {
        "layout_seed": seed, "split": split, "primary_or_reserve": role,
        "status": "ACCEPTED", "accepted_attempt": accepted["attempt"],
        "geometry_parameter_digest": geom_digest,
        "world_path": world_rel, "world_sha256": world_sha,
        "layout_config_path": layout_rel, "layout_config_sha256": layout_sha,
        "world_identity_sha256": world_identity,
        "gt_binding_path": binding_rel,
        "gt_binding_sha256": sha256_file(repo_root / binding_rel),
        "gt_sha256": gt_sha, "roi_sha256": roi_sha,
        "gt_semantic_digest": accepted["gt"]["semantic_digest"],
        "audit_path": audit_rel,
    }

def generate_set(
    repo_root: Path,
    seeds: tuple[int, ...] = LAYOUT_SEEDS,
    old_geometry_digests: set[str] | None = None,
) -> list[dict]:
    repo_root = repo_root.resolve()
    generator_path = repo_root / "mapex_lab/scripts/mx049_generator.py"
    gt_generator_path = repo_root / "mapex_lab/scripts/generate_new_room_ground_truth.py"
    generator_commit = git_value(repo_root, "rev-parse", "HEAD")
    generator_blob = git_value(repo_root, "hash-object", str(generator_path))
    gt_generator_blob = git_value(repo_root, "hash-object", str(gt_generator_path))
    accepted_digests: set[str] = set()
    out = []
    for seed in sorted(seeds):
        item = generate_one(
            repo_root,
            seed,
            accepted_digests,
            generator_commit,
            generator_blob,
            gt_generator_blob,
            old_geometry_digests,
        )
        out.append(item)
    return out

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--seed", type=int, action="append")
    parser.add_argument("--audit-csv", type=Path)
    parser.add_argument("--old-geometry-denylist", type=Path)
    args = parser.parse_args()
    seeds = tuple(args.seed) if args.seed else LAYOUT_SEEDS
    old_geometry_digests = None
    if args.old_geometry_denylist:
        denylist = json.loads(args.old_geometry_denylist.read_text(encoding="utf-8"))
        old_geometry_digests = {
            str(entry["geometry_parameter_digest"]) for entry in denylist["entries"]
        }
    results = generate_set(args.repo_root, seeds, old_geometry_digests)
    if args.audit_csv:
        args.audit_csv.parent.mkdir(parents=True, exist_ok=True)
        keys = sorted({k for r in results for k in r})
        with args.audit_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(results)
    print(json.dumps(results, indent=2, sort_keys=True))
    if any(r["status"] != "ACCEPTED" for r in results):
        raise SystemExit(2)

if __name__ == "__main__":
    main()
