#!/usr/bin/env python3
"""Frozen R003 New Room paper1000 construction and evaluation primitives.

This module is deliberately ROS-free.  The online recorders use the budget and
snapshot helpers, while the offline generator/evaluator uses the grid helpers.
Keeping the contract here prevents NF and MapEx from drifting apart.
"""
from __future__ import annotations

import hashlib
import heapq
import json
import math
import platform
import random
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np

from generate_new_room_ground_truth import (
    CANVAS_H,
    CANVAS_ID,
    CANVAS_RES,
    CANVAS_W,
    CANVAS_X,
    CANVAS_Y,
    DEFAULT_SPAWN_X,
    DEFAULT_SPAWN_Y,
    DEFAULT_SPAWN_YAW,
    DEFAULT_Z_SLICE_M,
    STRUCTURE_MODEL,
    _boxes_world_to_slam,
    _building_mask,
    _load_collision_boxes,
    _rasterize_boxes,
    _sha256,
    _world_to_cell,
)

PROFILE_ID = "new_room_mapex_paper1000_v1"
BUDGET_ID = "odom_progress_0p30m_v1"
STRUCTURAL_ID = "new_room_mapex_structural_v1"
VALID_SPACE_ID = "new_room_mapex_valid_space_v1"
EVAL_ID = "new_room_mapex_eval_010_v1"
PROTOCOL_RELATIVE = "docs/R003_PAPER1000_PROTOCOL_V1.md"
REDUCED_RES = 0.10
REDUCED_W = 752
REDUCED_H = 1062
STEP_METERS = 0.30
MAX_STEPS = 1000
BUDGET_METERS = STEP_METERS * MAX_STEPS
TU_GOAL_COUNT = 100
TU_SEED = 3001
MAX_ODOM_GAP_S = 1.0
MAX_ODOM_INCREMENT_M = 2.0
NEIGHBOURS = ((-1, 0), (0, -1), (0, 1), (1, 0))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def array_sha256(array: np.ndarray) -> str:
    contiguous = np.ascontiguousarray(array)
    digest = hashlib.sha256()
    digest.update(str(contiguous.dtype).encode("ascii"))
    digest.update(np.asarray(contiguous.shape, dtype=np.int64).tobytes())
    digest.update(contiguous.tobytes())
    return digest.hexdigest()


def _pad_2x2(array: np.ndarray, value) -> np.ndarray:
    if array.shape != (CANVAS_H, CANVAS_W):
        raise ValueError(f"expected source canvas {(CANVAS_H, CANVAS_W)}, got {array.shape}")
    return np.pad(array, ((0, CANVAS_H % 2), (0, CANVAS_W % 2)), constant_values=value)


def reduce_any(mask: np.ndarray, *, pad_value: bool = False) -> np.ndarray:
    padded = _pad_2x2(np.asarray(mask, dtype=bool), pad_value)
    return padded.reshape(REDUCED_H, 2, REDUCED_W, 2).any(axis=(1, 3))


def reduce_observed(source: np.ndarray) -> np.ndarray:
    """R003 observed reduction: occupied wins, then free, else unknown."""
    source = np.asarray(source)
    occupied = reduce_any(source > 0, pad_value=True)
    free = reduce_any(source == 0, pad_value=False)
    result = np.full((REDUCED_H, REDUCED_W), -1, dtype=np.int16)
    result[free] = 0
    result[occupied] = 100
    return result


def reduce_prediction(source_probability: np.ndarray) -> np.ndarray:
    source_probability = np.asarray(source_probability, dtype=np.float32)
    if source_probability.shape != (CANVAS_H, CANVAS_W):
        raise ValueError("prediction must be on the 0.05 m canonical canvas")
    if not np.isfinite(source_probability).all():
        raise ValueError("prediction contains nonfinite values")
    padded = _pad_2x2(source_probability, 0.0)
    return padded.reshape(REDUCED_H, 2, REDUCED_W, 2).max(axis=(1, 3))


def project_runtime_observed(
    data: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Project an immutable axis-aligned runtime grid onto 0.05 m cell centres."""
    data = np.asarray(data)
    if data.ndim != 2:
        raise ValueError("runtime map must be two-dimensional")
    if not math.isfinite(origin_yaw) or abs(origin_yaw) > 1e-6:
        raise ValueError("rotated runtime OccupancyGrid is unsupported")
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("invalid runtime resolution")
    xs = CANVAS_X + (np.arange(CANVAS_W) + 0.5) * CANVAS_RES
    ys = CANVAS_Y + (np.arange(CANVAS_H) + 0.5) * CANVAS_RES
    cols = np.floor((xs - origin_x) / resolution).astype(np.int64)
    rows = np.floor((ys - origin_y) / resolution).astype(np.int64)
    valid_cols = (cols >= 0) & (cols < data.shape[1])
    valid_rows = (rows >= 0) & (rows < data.shape[0])
    support = valid_rows[:, None] & valid_cols[None, :]
    result = np.full((CANVAS_H, CANVAS_W), -1, dtype=np.int16)
    rr = rows[valid_rows]
    cc = cols[valid_cols]
    result[np.ix_(valid_rows, valid_cols)] = data[np.ix_(rr, cc)].astype(np.int16)
    return result, support


def project_runtime_prediction(
    probability: np.ndarray,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    probability = np.asarray(probability, dtype=np.float32)
    if not np.isfinite(probability).all():
        raise ValueError("prediction contains nonfinite values")
    observed, support = project_runtime_observed(
        probability, resolution, origin_x, origin_y, origin_yaw
    )
    return np.where(support, observed.astype(np.float32), 0.0), support


def build_profile(sdf_path: Path, output_dir: Path, *, git_commit: str) -> dict:
    """Generate and freeze the R003 masks, TU goals and manifest."""
    boxes_world = _load_collision_boxes(sdf_path, STRUCTURE_MODEL, DEFAULT_Z_SLICE_M)
    boxes = _boxes_world_to_slam(
        boxes_world, DEFAULT_SPAWN_X, DEFAULT_SPAWN_Y, DEFAULT_SPAWN_YAW
    )
    occupied05 = _rasterize_boxes(boxes)
    footprint05, bounds = _building_mask(boxes)
    occupied05 &= footprint05
    valid05 = footprint05.copy()
    evaluation05 = footprint05.copy()

    occupied10 = reduce_any(occupied05, pad_value=True)
    valid10 = reduce_any(valid05, pad_value=False)
    evaluation10 = reduce_any(evaluation05, pad_value=False)
    free10 = evaluation10 & ~occupied10
    start05 = _world_to_cell(0.0, 0.0)
    start10 = (start05[0] // 2, start05[1] // 2)
    if not evaluation10[start10] or occupied10[start10]:
        raise ValueError(f"R003 start {start10} must be inside E10 and GT-free")
    if np.any(valid05[-1]) or np.any(evaluation05[-1]):
        raise ValueError("odd source row unexpectedly intersects valid/evaluation support")
    valid_cells = np.argwhere(valid10)
    if len(valid_cells) < TU_GOAL_COUNT:
        raise ValueError("valid-space contains fewer than 100 cells")
    rng = random.Random(TU_SEED)
    sampled = rng.sample(range(len(valid_cells)), TU_GOAL_COUNT)
    goals = valid_cells[np.asarray(sampled, dtype=np.int64)].astype(np.int32)

    output_dir.mkdir(parents=True, exist_ok=True)
    profile_path = output_dir / f"{EVAL_ID}.npz"
    goals_path = output_dir / f"{EVAL_ID}_tu_goals.npy"
    manifest_path = output_dir / f"{EVAL_ID}_manifest.json"
    np.savez_compressed(
        profile_path,
        occupied=occupied10,
        valid_space=valid10,
        evaluation_mask=evaluation10,
        resolution=np.float64(REDUCED_RES),
        width=np.int64(REDUCED_W),
        height=np.int64(REDUCED_H),
        origin_x=np.float64(CANVAS_X),
        origin_y=np.float64(CANVAS_Y),
        frame_id=np.asarray("slam_start"),
        profile_id=np.asarray(PROFILE_ID),
        structural_id=np.asarray(STRUCTURAL_ID),
        valid_space_id=np.asarray(VALID_SPACE_ID),
        evaluation_id=np.asarray(EVAL_ID),
        start_row=np.int64(start10[0]),
        start_col=np.int64(start10[1]),
    )
    np.save(goals_path, goals)
    counts = {
        "source": {
            "occupied": int(occupied05.sum()),
            "free": int((evaluation05 & ~occupied05).sum()),
            "valid": int(valid05.sum()),
            "valid_and_occupied": int((valid05 & occupied05).sum()),
        },
        "reduced": {
            "occupied": int(occupied10.sum()),
            "free": int(free10.sum()),
            "valid": int(valid10.sum()),
            "valid_and_occupied": int((valid10 & occupied10).sum()),
            "evaluation": int(evaluation10.sum()),
        },
    }
    manifest = {
        "profile_id": PROFILE_ID,
        "budget_id": BUDGET_ID,
        "structural_id": STRUCTURAL_ID,
        "valid_space_id": VALID_SPACE_ID,
        "evaluation_id": EVAL_ID,
        "git_commit": git_commit,
        "source_sdf": str(sdf_path),
        "source_sdf_sha256": _sha256(sdf_path),
        "protocol": PROTOCOL_RELATIVE,
        "coordinate_frame": "slam_start",
        "world_to_slam": {
            "spawn_world_xyz_yaw": [DEFAULT_SPAWN_X, DEFAULT_SPAWN_Y, 0.0, DEFAULT_SPAWN_YAW],
            "rule": "R(-spawn_yaw) @ (p_world - p_spawn)",
        },
        "source_canvas": {
            "id": CANVAS_ID,
            "shape": [CANVAS_H, CANVAS_W],
            "resolution_m": CANVAS_RES,
            "origin_xy": [CANVAS_X, CANVAS_Y],
        },
        "reduced_canvas": {
            "shape": [REDUCED_H, REDUCED_W],
            "resolution_m": REDUCED_RES,
            "origin_xy": [CANVAS_X, CANVAS_Y],
            "block_anchor": [0, 0],
            "odd_row_padding": "occupied=1, valid=0, evaluation=0",
        },
        "bounds_m": list(map(float, bounds)),
        "counts": counts,
        "start_row_col": list(map(int, start10)),
        "tu": {
            "goal_count": TU_GOAL_COUNT,
            "seed": TU_SEED,
            "rng": "python.random.Random.sample",
            "python_version": platform.python_version(),
            "neighbour_order": [list(item) for item in NEIGHBOURS],
            "goals_sha256": array_sha256(goals),
        },
        "hashes": {
            "profile_file_sha256": sha256(profile_path),
            "occupied": array_sha256(occupied10),
            "valid_space": array_sha256(valid10),
            "evaluation_mask": array_sha256(evaluation10),
            "tu_goals_file_sha256": sha256(goals_path),
        },
        "generation_command": (
            "python3 mapex_lab/scripts/generate_r003_paper1000_profile.py "
            "--output-dir mapex_lab/ground_truth/new_room/generated/r003_paper1000"
        ),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def occupied_iou(probability10: np.ndarray, occupied10: np.ndarray, evaluation10: np.ndarray) -> dict:
    probability10 = np.asarray(probability10, dtype=np.float32)
    if probability10.shape != (REDUCED_H, REDUCED_W) or not np.isfinite(probability10).all():
        raise ValueError("invalid reduced prediction")
    predicted = probability10 > 0.5
    gt = np.asarray(occupied10, dtype=bool)
    domain = np.asarray(evaluation10, dtype=bool)
    tp = int(np.count_nonzero(domain & predicted & gt))
    fp = int(np.count_nonzero(domain & predicted & ~gt))
    fn = int(np.count_nonzero(domain & ~predicted & gt))
    union = tp + fp + fn
    return {"value": float(tp / union) if union else 0.0, "tp": tp, "fp": fp, "fn": fn, "empty_union": union == 0}


def coverage(observed10: np.ndarray, valid10: np.ndarray) -> dict:
    known = np.asarray(observed10) >= 0
    valid = np.asarray(valid10, dtype=bool)
    denominator = int(valid.sum())
    return {
        "coverage": float(np.count_nonzero(known & valid) / denominator),
        "known_fraction": float(np.count_nonzero(known) / known.size),
        "coverage_denominator": denominator,
        "known_fraction_denominator": int(known.size),
    }


def _astar(predicted_free: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
    if not predicted_free[start] or not predicted_free[goal]:
        return None
    heap: list[tuple[int, int, int, int]] = [(0, 0, start[0], start[1])]
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {start: 0}
    serial = 0
    while heap:
        _f, _serial, row, col = heapq.heappop(heap)
        current = (row, col)
        if current == goal:
            path = [current]
            while current != start:
                current = parent[current]
                path.append(current)
            return list(reversed(path))
        current_cost = cost[current]
        for dr, dc in NEIGHBOURS:
            nxt = (row + dr, col + dc)
            if not (0 <= nxt[0] < predicted_free.shape[0] and 0 <= nxt[1] < predicted_free.shape[1]):
                continue
            if not predicted_free[nxt]:
                continue
            candidate = current_cost + 1
            if candidate >= cost.get(nxt, 1 << 60):
                continue
            cost[nxt] = candidate
            parent[nxt] = current
            serial += 1
            heuristic = abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1])
            heapq.heappush(heap, (candidate + heuristic, serial, nxt[0], nxt[1]))
    return None


def topological_understanding(
    probability10: np.ndarray,
    occupied10: np.ndarray,
    evaluation10: np.ndarray,
    start: tuple[int, int],
    goals: np.ndarray,
) -> dict:
    probability10 = np.asarray(probability10, dtype=np.float32)
    if not np.isfinite(probability10).all():
        return {"status": "evaluation_error", "error": "nonfinite_prediction", "denominator": len(goals)}
    evaluation = np.asarray(evaluation10, dtype=bool)
    occupied = np.asarray(occupied10, dtype=bool)
    predicted_free = evaluation & ~(probability10 > 0.5)
    counts = {"success": 0, "predicted_occupied": 0, "no_path": 0, "gt_collision": 0, "evaluation_error": 0}
    for raw_goal in np.asarray(goals):
        goal = (int(raw_goal[0]), int(raw_goal[1]))
        try:
            if not predicted_free[start] or not predicted_free[goal]:
                counts["predicted_occupied"] += 1
                continue
            path = _astar(predicted_free, start, goal)
            if path is None:
                counts["no_path"] += 1
            elif any(occupied[cell] for cell in path):
                counts["gt_collision"] += 1
            else:
                counts["success"] += 1
        except Exception:
            counts["evaluation_error"] += 1
    if counts["evaluation_error"]:
        return {"status": "evaluation_error", "denominator": len(goals), **counts}
    return {"status": "ok", "value": counts["success"] / len(goals), "denominator": len(goals), **counts}


@dataclass(frozen=True)
class BudgetUpdate:
    distance_m: float
    step: int
    residual_m: float
    crossed_steps: tuple[int, ...]
    cutoff_crossed: bool
    detection_overshoot_m: float


class OdomProgressBudget:
    """Thread-safe accumulated odometry budget with integrity fault latching."""
    def __init__(
        self, step_m: float = STEP_METERS, max_steps: int = MAX_STEPS,
        max_gap_s: float = MAX_ODOM_GAP_S,
        max_increment_m: float = MAX_ODOM_INCREMENT_M,
    ):
        self.step_m = float(step_m)
        self.max_steps = int(max_steps)
        self.limit_m = self.step_m * self.max_steps
        self.max_gap_s = float(max_gap_s)
        self.max_increment_m = float(max_increment_m)
        self._lock = threading.Lock()
        self.started = False
        self.last_xy: tuple[float, float] | None = None
        self.last_stamp: float | None = None
        self.frame_id: str | None = None
        self.distance_m = 0.0
        self.step = 0
        self.cutoff_latched = False
        self.integrity_faults: list[str] = []

    def start(self, x: float, y: float, stamp_s: float, frame_id: str) -> None:
        with self._lock:
            if self.started:
                return
            self._validate(x, y, stamp_s)
            self.started = True
            self.last_xy = (float(x), float(y))
            self.last_stamp = float(stamp_s)
            self.frame_id = str(frame_id)

    @staticmethod
    def _validate(x: float, y: float, stamp_s: float) -> None:
        if not all(math.isfinite(value) for value in (x, y, stamp_s)):
            raise ValueError("nonfinite odometry")

    def update(self, x: float, y: float, stamp_s: float, frame_id: str) -> BudgetUpdate:
        with self._lock:
            self._validate(x, y, stamp_s)
            if not self.started:
                return BudgetUpdate(self.distance_m, self.step, 0.0, (), False, 0.0)
            if self.frame_id != str(frame_id):
                fault = f"odom_frame_changed:{self.frame_id}->{frame_id}"
                self.integrity_faults.append(fault)
                raise ValueError(fault)
            if self.last_stamp is not None and stamp_s <= self.last_stamp:
                fault = f"odom_time_not_increasing:{stamp_s}<={self.last_stamp}"
                self.integrity_faults.append(fault)
                raise ValueError(fault)
            if self.last_stamp is not None and stamp_s - self.last_stamp > self.max_gap_s:
                fault = f"odom_gap_s:{stamp_s - self.last_stamp:.9f}>{self.max_gap_s:.9f}"
                self.integrity_faults.append(fault)
                raise ValueError(fault)
            assert self.last_xy is not None
            increment = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            if increment > self.max_increment_m:
                fault = f"odom_position_jump_m:{increment:.9f}>{self.max_increment_m:.9f}"
                self.integrity_faults.append(fault)
                raise ValueError(fault)
            self.distance_m += increment
            old_step = self.step
            self.step = min(self.max_steps, int(math.floor((self.distance_m + 1e-12) / self.step_m)))
            self.last_xy = (float(x), float(y))
            self.last_stamp = float(stamp_s)
            crossed = tuple(range(old_step + 1, self.step + 1))
            cutoff = not self.cutoff_latched and self.distance_m >= self.limit_m
            if cutoff:
                self.cutoff_latched = True
            return BudgetUpdate(
                self.distance_m,
                self.step,
                self.distance_m - self.step * self.step_m,
                crossed,
                cutoff,
                max(0.0, self.distance_m - self.limit_m) if cutoff else 0.0,
            )


def common_crossings(crossed_steps: Iterable[int]) -> tuple[int, ...]:
    return tuple(step for step in crossed_steps if step % 10 == 0 and 10 <= step <= MAX_STEPS)


def select_final_sample(records: list[dict]) -> dict:
    """Prefer the explicit cutoff/natural final record, never the last decision."""
    finals = [row for row in records if row.get("event") in {"budget_cutoff", "natural_completion"}]
    if len(finals) != 1:
        raise ValueError(f"expected exactly one authoritative final sample, got {len(finals)}")
    return finals[0]


def request_cutoff_during_scoring(
    budget: OdomProgressBudget,
    odom_updates: Iterable[tuple[float, float, float, str]],
    scoring: Callable[[], None],
) -> bool:
    """Test helper proving budget updates can latch while scoring is blocked."""
    scoring_started = threading.Event()
    scoring_release = threading.Event()

    def wrapped_scoring():
        scoring_started.set()
        scoring_release.wait(timeout=5.0)
        scoring()

    worker = threading.Thread(target=wrapped_scoring)
    worker.start()
    scoring_started.wait(timeout=2.0)
    crossed = False
    for update in odom_updates:
        crossed = budget.update(*update).cutoff_crossed or crossed
    scoring_release.set()
    worker.join(timeout=2.0)
    return crossed
