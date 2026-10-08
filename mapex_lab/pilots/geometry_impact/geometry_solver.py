"""Anytime action bounds for selective GT geometry corrections.

The renderer and score are reused from pinned R4 A1. A packed ray reader
accelerates endpoint collection; it must pass native parity before real use.
No mask inclusion/monotonicity assumption is made.
"""
from __future__ import annotations

import ast
from fractions import Fraction
import math
from pathlib import Path
import signal
import time

import numpy as np


class GeometryBudget(RuntimeError):
    pass


def exact_sum_upper(weights, unknown):
    if weights.dtype != np.float32:
        raise ValueError("Native sum requires float32 in this sealed adapter")
    v = np.ascontiguousarray(weights[unknown])
    if not np.all(np.isfinite(v)) or np.any(v < 0):
        raise ValueError("Nonnegative finite uncertainty required")
    bits = v[v > 0].view(np.uint32)
    n = len(bits)
    if not n:
        return 0.0
    if n - 1 >= 2**24:
        return math.inf
    exponents = (bits >> 23) & 255
    mantissas = (bits & ((1 << 23) - 1)).astype(np.uint64)
    mantissas += (exponents > 0).astype(np.uint64) << 23
    numerator = 0
    for e in np.unique(exponents):
        total = int(np.sum(mantissas[exponents == e], dtype=np.uint64))
        numerator += total << max(0, int(e) - 1)
    bound = Fraction(numerator, 1 << 149) / (1 - Fraction(n - 1, 1 << 24))
    if bound > Fraction.from_float(float(np.finfo(np.float32).max)):
        return math.inf
    rounded = float(bound)
    if Fraction.from_float(rounded) < bound:
        rounded = float(np.nextafter(rounded, math.inf))
    return rounded


def native_bresenham(source):
    tree = ast.parse(Path(source).read_text())
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_bresenham")
    namespace = {"np": np, "math": math}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(source), "exec"), namespace)
    return namespace["_bresenham"]


class RayTable:
    def __init__(self, native, shape, pose, bresenham):
        self.shape = shape
        h, w = shape
        paths = []
        for angle in np.linspace(0.0, 2.0 * math.pi, native.mapex_num_rays):
            end = (int(pose[0] + native.ray_range_cells * math.cos(angle)),
                   int(pose[1] + native.ray_range_cells * math.sin(angle)))
            path = []
            for r, c in bresenham(pose, end):
                if r < 0 or r >= h or c < 0 or c >= w:
                    break
                path.append(r * w + c)
            if not path:
                raise ValueError("Invalid viewpoint")
            paths.append(path)
        self.lengths = np.asarray([len(p) for p in paths], dtype=np.int64)
        self.indices = np.zeros((len(paths), int(self.lengths.max())), dtype=np.int64)
        for j, path in enumerate(paths):
            self.indices[j, :len(path)] = path
        self.valid = np.arange(self.indices.shape[1])[None, :] < self.lengths[:, None]
        self.epsilon = native.mapex_epsilon

    def endpoints(self, mean):
        values = np.clip(mean.ravel()[self.indices], 0.0, 1.0).astype(np.float64)
        values[~self.valid] = 0.0
        cumulative = np.cumsum(values, axis=1, dtype=np.float64)
        hits = (cumulative >= self.epsilon) & self.valid
        locations = np.where(hits.any(axis=1), hits.argmax(axis=1), self.lengths - 1)
        flat = self.indices[np.arange(len(locations)), locations]
        return np.column_stack(np.divmod(flat, self.shape[1])).astype(np.int64)


class GeometrySolver:
    def __init__(self, engine, world, mean, variance, truth, candidates):
        self.engine, self.world, self.candidates = engine, world, candidates
        self.mean, self.variance = mean, variance
        self.truth = truth.astype(mean.dtype)
        self.unknown = world.observed == 0.5
        if not np.all(np.isfinite(mean)) or mean.shape != truth.shape:
            raise ValueError("Mean/GT registration or finite-value check failed")
        if not np.array_equal(mean[~self.unknown], world.observed[~self.unknown]):
            raise ValueError("Known observations changed")
        self.worker = engine.native.worker
        source = Path(engine.cfg['repo']) / 'mapex_lab/scripts/mapex.py'
        line = native_bresenham(source)
        self.rays = [RayTable(self.worker, mean.shape, tuple(c['goal']), line) for c in candidates]
        used = np.zeros(mean.size, dtype=bool)
        frequency = np.zeros(mean.size, dtype=np.int64)
        for ray in self.rays:
            indices = ray.indices[ray.valid]
            used[indices] = True
            frequency += np.bincount(indices, minlength=mean.size)
        eligible = used & self.unknown.ravel() & (mean.ravel() != self.truth.ravel())
        cells = np.flatnonzero(eligible)
        importance = frequency[cells] * np.abs(mean.ravel()[cells] - self.truth.ravel()[cells])
        self.cells = cells[np.lexsort((cells, -importance))]
        self.low = mean.copy()
        self.high = mean.copy()
        self.low.ravel()[self.cells] = np.minimum(mean.ravel()[self.cells], self.truth.ravel()[self.cells])
        self.high.ravel()[self.cells] = np.maximum(mean.ravel()[self.cells], self.truth.ravel()[self.cells])
        self.gain_upper = exact_sum_upper(variance, self.unknown)
        self.cache = {}
        self.mask_evaluations = 0
        self.parity_checks = 0

    def tie(self, index):
        candidate = self.candidates[index]
        return candidate['cost'], tuple(candidate['goal']), index

    def winner(self, scores):
        return min(range(len(scores)), key=lambda j: (-scores[j], *self.tie(j)))

    def score(self, endpoints, index):
        key = (index, endpoints.tobytes())
        if key not in self.cache:
            mask = self.worker.build_visibility_mask(endpoints, tuple(self.candidates[index]['goal']), self.mean.shape)
            mask &= self.unknown
            gain = self.worker.compute_information_gain(self.variance, mask)
            self.cache[key] = gain / max(self.candidates[index]['cost'], 1e-9)
            self.mask_evaluations += 1
        return self.cache[key]

    def scores(self, mean):
        return [self.score(ray.endpoints(mean), j) for j, ray in enumerate(self.rays)]

    def native_parity(self):
        patched_gt = np.where(self.unknown, self.truth, self.mean).astype(self.mean.dtype)
        for candidate_mean in (self.mean, patched_gt, self.low, self.high):
            actual_scores = []
            for j, ray in enumerate(self.rays):
                native = self.worker.collect_ray_endpoints(candidate_mean, tuple(self.candidates[j]['goal']))
                packed = ray.endpoints(candidate_mean)
                if not np.array_equal(native, packed):
                    raise RuntimeError("Packed/native endpoint mismatch")
                mask = self.engine.native.visibility(tuple(self.candidates[j]['goal']), candidate_mean, self.world.observed)
                score = float(self.variance[mask].sum()) / max(self.candidates[j]['cost'], 1e-9)
                if score != self.score(packed, j):
                    raise RuntimeError("Native mask/score mismatch")
                self.parity_checks += 2
                actual_scores.append(score)
            native_scores, chosen, _ = self.engine.score(self.world, candidate_mean, self.variance, self.candidates)
            if native_scores['native_P_U'] != actual_scores or chosen['native_P_U'] != self.winner(actual_scores):
                raise RuntimeError("Frozen Engine score/tie mismatch")
            self.parity_checks += 1
        return self.parity_checks

    def bounds(self, assignments):
        low, high = self.low.copy(), self.high.copy()
        for j, bit in assignments.items():
            flat = self.cells[j]
            value = self.truth.ravel()[flat] if bit else self.mean.ravel()[flat]
            low.ravel()[flat] = high.ravel()[flat] = value
        result, fixed = [], []
        for j, ray in enumerate(self.rays):
            opaque, transparent = ray.endpoints(high), ray.endpoints(low)
            if np.array_equal(opaque, transparent):
                score = self.score(opaque, j)
                result.append((score, score))
                fixed.append(j)
            else:
                result.append((0.0, self.gain_upper / max(self.candidates[j]['cost'], 1e-9)))
        return result, fixed

    def possible(self, intervals):
        return {a for a, (_, upper) in enumerate(intervals)
                if not any(lower > upper or (lower == upper and self.tie(b) < self.tie(a))
                           for b, (lower, _) in enumerate(intervals) if b != a)}

    def completion(self, assignments):
        mean = self.mean.copy()
        corrected = []
        for j, bit in assignments.items():
            if bit:
                flat = int(self.cells[j])
                mean.ravel()[flat] = self.truth.ravel()[flat]
                corrected.append(flat)
        return mean, {'kind': 'sparse', 'corrected_flat': sorted(corrected)}

    def seed_witnesses(self):
        witness = {}
        for kind, mean in [('none', self.mean), ('all_unknown', np.where(self.unknown, self.truth, self.mean))]:
            action = self.winner(self.scores(mean))
            witness.setdefault(action, {'kind': kind})
        return witness

    def solve(self, max_nodes=4096, seconds=60.0):
        began = time.perf_counter()
        stack, witnesses = [{}], {}
        nodes, fixed_root, intervals_root = 0, [], []
        timed_out = False
        current_partial = None
        def alarm_handler(signum, frame):
            raise GeometryBudget('geometry time cap')
        previous = signal.signal(signal.SIGALRM, alarm_handler)
        signal.setitimer(signal.ITIMER_REAL, max(0.001, seconds))
        try:
            witnesses = self.seed_witnesses()
            while stack and nodes < max_nodes:
                partial = stack[-1]
                current_partial = dict(partial)
                intervals, fixed = self.bounds(partial)
                if not nodes:
                    intervals_root, fixed_root = intervals, fixed
                possible = self.possible(intervals)
                if not possible:
                    raise RuntimeError('Empty possible action set')
                if possible.issubset(witnesses):
                    stack.pop()
                elif len(possible) == 1:
                    mean, certificate = self.completion(partial)
                    action = self.winner(self.scores(mean))
                    if action not in possible:
                        raise RuntimeError('Invalid singleton action certificate')
                    witnesses[action] = certificate
                    stack.pop()
                else:
                    j = next((j for j in range(len(self.cells)) if j not in partial), None)
                    if j is None:
                        raise RuntimeError('Fully assigned node still has tied/uncertain actions')
                    children = []
                    for bit in (1, 0):
                        child = dict(partial)
                        child[j] = bit
                        children.append(child)
                    stack.pop()
                    stack.extend(children)
                nodes += 1
        except GeometryBudget:
            timed_out = True
            # A signal can arrive between pop/extend. Reinsert the parent;
            # overlapping subcubes are conservative and never erase a branch.
            if current_partial is not None:
                stack.append(current_partial)
        finally:
            signal.setitimer(signal.ITIMER_REAL, 0)
            signal.signal(signal.SIGALRM, previous)
        lower = set(witnesses)
        # If the timer interrupted seeds/root, retain all candidates. Otherwise
        # root score intervals safely enclose every unprocessed subcube.
        upper = (self.possible(intervals_root) if intervals_root else set(range(len(self.candidates)))) if (stack or timed_out) else lower
        upper |= lower
        return dict(reachable_lower=sorted(lower), reachable_upper=sorted(upper),
                    witnesses={str(k): v for k, v in witnesses.items()},
                    complete=not stack and not timed_out, timed_out=timed_out, nodes=nodes,
                    open_subcubes=len(stack), fixed_root=fixed_root,
                    correctable_relevant_cells=len(self.cells), candidates=len(self.candidates),
                    native_mask_evaluations=self.mask_evaluations, parity_checks=self.parity_checks,
                    wall_s=time.perf_counter()-began,
                    subcubes=[[[int(j), int(bit)] for j, bit in sorted(p.items())] for p in stack])

    def replay_witness(self, certificate):
        mean = self.mean.copy()
        kind = certificate['kind']
        if kind == 'all_unknown':
            mean[self.unknown] = self.truth[self.unknown]
        elif kind == 'sparse':
            cells = np.asarray(certificate['corrected_flat'], dtype=np.int64)
            if cells.size:
                if not np.all(self.unknown.ravel()[cells]):
                    raise RuntimeError('Correction touched a known cell')
                mean.ravel()[cells] = self.truth.ravel()[cells]
        elif kind != 'none':
            raise RuntimeError('Unknown geometry witness type')
        return self.winner(self.scores(mean))
