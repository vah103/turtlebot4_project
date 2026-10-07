"""A small grid simulator and policy-only geometry utilities.

World truth is private to GridWorld. Policies receive immutable observations
and predictions, never the world or an evaluation domain.
"""
from __future__ import annotations

import ast
from collections import deque
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

FOUR = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool)
EIGHT = np.ones((3, 3), dtype=bool)
FREE, UNKNOWN, OCCUPIED = 0.0, 0.5, 1.0


def array_hash(a):
    a = np.ascontiguousarray(a)
    return hashlib.sha256(str(a.shape).encode() + str(a.dtype).encode() + a.tobytes()).hexdigest()


@dataclass(frozen=True)
class PolicyInput:
    observed: np.ndarray
    mean: np.ndarray
    variance: np.ndarray
    predictions: np.ndarray
    pose: tuple[int, int]
    resolution: float
    remaining_m: float

    def __post_init__(self):
        shape = self.observed.shape
        if self.mean.shape != shape or self.variance.shape != shape:
            raise ValueError("Prediction and observation geometry must match")
        if self.predictions.shape != (3,) + shape:
            raise ValueError("Exactly three predictions are required")
        if not all(np.isfinite(a).all() for a in (self.observed, self.mean, self.variance, self.predictions)):
            raise ValueError("Non-finite policy input")
        if not np.isin(self.observed, [FREE, UNKNOWN, OCCUPIED]).all():
            raise ValueError("Observation labels must be 0, 0.5, 1")
        known = self.observed != UNKNOWN
        if not np.array_equal(self.mean[known], self.observed[known]):
            raise ValueError("Prediction must preserve observed cells")
        for a in (self.observed, self.mean, self.variance, self.predictions):
            a.setflags(write=False)


class GridWorld:
    """Ideal pose, static geometry, first-hit 360-degree range observations.

    Rays sample every half cell. Motion uses cardinal cell-centre steps, and
    physical collision checks include the robot's disk footprint. A policy
    cannot use these checks to rank candidates; they are execution outcomes.
    """

    def __init__(self, occupied, resolution, pose, sensor_range_m=20.0,
                 sensor_rays=2500, robot_radius_m=0.15, observed=None):
        self._occupied = np.asarray(occupied, dtype=bool).copy()
        self.resolution = float(resolution)
        self.pose = tuple(int(x) for x in pose)
        self._physical_free = clearance_free(~self._occupied, robot_radius_m / resolution)
        if not self._physical_free[self.pose]:
            raise ValueError("Initial robot footprint is not collision-free")
        self.observed = (np.full(self._occupied.shape, UNKNOWN, dtype=np.float32)
                         if observed is None else np.asarray(observed, dtype=np.float32).copy())
        self.distance_m = 0.0
        self.collisions = 0
        self.sensor_range_m = float(sensor_range_m)
        self.sensor_rays = int(sensor_rays)
        angles = np.linspace(0, 2 * np.pi, sensor_rays, endpoint=False)
        steps = np.arange(0, sensor_range_m / resolution + 0.25, 0.5)
        self._dr = np.floor(steps[:, None] * np.sin(angles)[None, :] + 0.5).astype(np.int32)
        self._dc = np.floor(steps[:, None] * np.cos(angles)[None, :] + 0.5).astype(np.int32)

    def sense(self):
        h, w = self._occupied.shape
        active = np.ones(self.sensor_rays, dtype=bool)
        visible = np.zeros((h, w), dtype=bool)
        r0, c0 = self.pose
        for dr, dc in zip(self._dr, self._dc):
            r, c = r0 + dr, c0 + dc
            active &= (r >= 0) & (r < h) & (c >= 0) & (c < w)
            if not active.any():
                break
            indices = np.flatnonzero(active)
            rr, cc = r[indices], c[indices]
            visible[rr, cc] = True
            active[indices[self._occupied[rr, cc]]] = False
        new = visible & (self.observed == UNKNOWN)
        self.observed[visible] = self._occupied[visible].astype(np.float32)
        return new

    def step(self, pose):
        pose = tuple(int(x) for x in pose)
        if abs(pose[0] - self.pose[0]) + abs(pose[1] - self.pose[1]) != 1:
            raise ValueError("Motion must be one cardinal cell; teleportation is forbidden")
        h, w = self._occupied.shape
        if not (0 <= pose[0] < h and 0 <= pose[1] < w) or not self._physical_free[pose]:
            self.collisions += 1
            self.sense()
            return False
        self.pose = pose
        self.distance_m += self.resolution
        return True


def clearance_free(free, radius_cells):
    # The padded occupied border prevents implicit free space outside the grid.
    padded = np.pad(np.asarray(free, dtype=bool), 1, constant_values=False)
    distance = ndi.distance_transform_edt(padded)[1:-1, 1:-1]
    return free & (distance > radius_cells + 1e-9)


def observed_traversable(observed, radius_cells):
    # Unknown centres cannot be traversed. Only measured obstacles are inflated;
    # this permits exact frontier goals. The simulator still checks the true
    # disk footprint and reports any collision rather than hiding it.
    obstacle = observed == OCCUPIED
    obstacle = np.pad(obstacle, 1, constant_values=True)
    clearance = ndi.distance_transform_edt(~obstacle)[1:-1, 1:-1]
    return (observed == FREE) & (clearance > radius_cells + 1e-9)


def shortest_paths(traversable, start):
    """Exact shortest paths on a unit-cost 4-neighbour grid (A* with h=0)."""
    h, w = traversable.shape
    dist = np.full((h, w), -1, dtype=np.int32)
    parent = np.full((h, w), -1, dtype=np.int32)
    if not traversable[start]:
        return dist, parent
    dist[start] = 0
    q = deque([start])
    while q:
        r, c = q.popleft()
        next_dist = dist[r, c] + 1
        for rr, cc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if 0 <= rr < h and 0 <= cc < w and traversable[rr, cc] and dist[rr, cc] < 0:
                dist[rr, cc] = next_dist
                parent[rr, cc] = r * w + c
                q.append((rr, cc))
    return dist, parent


def recover_path(parent, start, goal):
    if start == goal:
        return [start]
    if parent[goal] < 0:
        return None
    w = parent.shape[1]
    path = [goal]
    while path[-1] != start:
        p = int(parent[path[-1]])
        if p < 0:
            raise ValueError("Broken predecessor chain")
        path.append(divmod(p, w))
    return path[::-1]


def frontier_representatives(observed, min_size=10):
    mask = (observed == FREE) & ndi.binary_dilation(observed == UNKNOWN, structure=EIGHT)
    labels, n = ndi.label(mask, structure=EIGHT)
    representatives = []
    for label in range(1, n + 1):
        cells = np.argwhere(labels == label)
        if len(cells) <= min_size:
            continue
        centre = cells.mean(axis=0)
        p = cells[np.argmin(np.sum((cells - centre) ** 2, axis=1))]
        representatives.append(tuple(int(x) for x in p))
    return representatives


def reachable_component(free, seed):
    if not free[seed]:
        return np.zeros_like(free, dtype=bool)
    return ndi.binary_propagation(np.zeros_like(free, dtype=bool) | seed_mask(free.shape, seed),
                                  structure=FOUR, mask=free)


def seed_mask(shape, seed):
    a = np.zeros(shape, dtype=bool)
    a[seed] = True
    return a


def line_cells(start, end):
    r0, c0 = start
    r1, c1 = end
    dc, dr = abs(c1 - c0), -abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    error = dc + dr
    while True:
        yield r0, c0
        if (r0, c0) == (r1, c1):
            break
        e = 2 * error
        if e >= dr:
            error += dr
            c0 += sc
        if e <= dc:
            error += dc
            r0 += sr


class MapExNumerics:
    """Load only the already-reviewed numerical methods, without ROS imports.

    An explicit AST allowlist reuses the source instead of maintaining a second
    version of probabilistic raycasting/polygon/flood-fill calculations.
    """
    def __init__(self, source, resolution, sensor_range_m=20.0, rays=250):
        source = Path(source)
        text = source.read_text()
        tree = ast.parse(text)
        helpers = {"_bresenham", "_init_buffered_boundary", "_flood_fill_simple"}
        methods = {"cast_ray", "collect_ray_endpoints", "build_visibility_mask", "compute_visibility",
                   "compute_information_gain"}
        nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in helpers]
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MapExExplorer")
        selected = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
        if {n.name for n in nodes} != helpers or {n.name for n in selected} != methods:
            raise RuntimeError("MapEx numerical source changed; inspect adapter before running")
        pure = ast.ClassDef(name="NumericalExplorer", bases=[], keywords=[], body=selected, decorator_list=[])
        module = ast.Module(body=[ast.ImportFrom(module="__future__", names=[ast.alias(name="annotations")], level=0)]
                           + nodes + [pure], type_ignores=[])
        namespace = {"np": np, "math": math, "deque": deque}
        exec(compile(ast.fix_missing_locations(module), str(source), "exec"), namespace)
        self.worker = namespace["NumericalExplorer"]()
        self.worker.ray_range_cells = int(sensor_range_m / resolution)
        self.worker.mapex_num_rays = rays
        self.worker.mapex_epsilon = 0.8
        self.source_sha256 = hashlib.sha256(text.encode()).hexdigest()

    def visibility(self, pose, mean, observed):
        return self.worker.compute_visibility({"row": pose[0], "col": pose[1]}, mean, observed, 0, 0)


def evaluate(observed, mean, occupied, domain, seed, resolution, robot_radius_m=0.15):
    """Evaluation-only function. Its inputs/results never enter a policy."""
    completed = np.where(observed != UNKNOWN, observed, mean) >= 0.5
    evaluation = domain | (occupied & ndi.binary_dilation(domain, iterations=3))
    free_iou = _iou(~completed & evaluation, ~occupied & evaluation)
    occupied_iou = _iou(completed & evaluation, occupied & evaluation)
    true_free = clearance_free(~occupied, robot_radius_m / resolution)
    pred_free = clearance_free(~completed, robot_radius_m / resolution)
    actual = reachable_component(true_free, seed) & evaluation
    predicted = reachable_component(pred_free, seed) & evaluation
    labels, _ = ndi.label(pred_free, structure=FOUR)
    counts = np.bincount(labels.ravel())
    missed = actual & ~predicted
    false = predicted & ~actual
    return dict(macro_iou=(free_iou + occupied_iou) / 2,
                free_iou=free_iou, occupied_iou=occupied_iou,
                missed_reachable_free_m2=float(missed.sum() * resolution ** 2),
                false_reachable_m2=float(false.sum() * resolution ** 2),
                reachable_mismatch_m2=float((missed | false).sum() * resolution ** 2),
                reachable_retention=float((actual & predicted).sum() / max(1, actual.sum())),
                observed_free_coverage=float(((observed == FREE) & actual).sum() / max(1, actual.sum())),
                prediction_wrong_unknown_cells=int(((completed != occupied) & (observed == UNKNOWN) & evaluation).sum()),
                known_cells=int(((observed != UNKNOWN) & evaluation).sum()),
                predicted_components=int(len(counts) - 1))


def _iou(a, b):
    union = (a | b).sum()
    return float((a & b).sum() / union) if union else 1.0
