#!/usr/bin/env python3
"""A numerical preflight, not an exploration experiment.

Loads numerical methods from the existing MapEx source by AST. Verifies a
ray-endpoint certificate and a conservative action-reachability solver against
exhaustive selective corrections on small fixtures. No model inference or ROS.
"""
from __future__ import annotations

import ast
from collections import deque
from fractions import Fraction
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT / "deps"))
SOURCE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT.parent / "repo_read/main/mapex_lab/scripts/mapex.py"


def load_native():
    text = SOURCE.read_text()
    tree = ast.parse(text)
    helpers = {"_bresenham", "_init_buffered_boundary", "_flood_fill_simple"}
    methods = {"cast_ray", "collect_ray_endpoints", "build_visibility_mask",
               "compute_visibility", "compute_information_gain"}
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in helpers]
    source_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "MapExExplorer")
    selected = [n for n in source_class.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    assert {n.name for n in nodes} == helpers
    assert {n.name for n in selected} == methods
    cls = ast.ClassDef(name="NativeNumerics", bases=[], keywords=[], body=selected, decorator_list=[])
    module = ast.Module(body=nodes + [cls], type_ignores=[])
    namespace = {"np": np, "math": math, "deque": deque}
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace["NativeNumerics"], namespace["_bresenham"], hashlib.sha256(text.encode()).hexdigest()


Native, bresenham, source_hash = load_native()


def certified_sum_upper(weights, unknown):
    """Bound a float32 nonnegative native subset sum, including roundoff.

    Float32 values are accumulated exactly as integers in units of 2**-149.
    The gamma bound applies to at most n-1 nontrivial floating additions.
    This deliberately prioritizes validity over speed; real-grid cost untested.
    """
    assert weights.dtype == np.float32
    values = weights[unknown]
    assert np.all(np.isfinite(values)) and np.all(values >= 0)
    positive = values[values > 0]
    n = len(positive)
    if n == 0:
        return 0.0
    if (n - 1) >= 2**24:
        return math.inf
    numerator = 0
    for value in positive:
        a, denominator = float(value).as_integer_ratio()
        exponent = denominator.bit_length() - 1
        assert denominator == 1 << exponent and exponent <= 149
        numerator += a << (149 - exponent)
    exact = Fraction(numerator, 1 << 149)
    rounding_factor = Fraction(1, 1) / (1 - Fraction(n - 1, 1 << 24))
    bound = exact * rounding_factor
    if bound > Fraction.from_float(float(np.finfo(np.float32).max)):
        return math.inf
    rounded = float(bound)
    if Fraction.from_float(rounded) < bound:
        rounded = float(np.nextafter(rounded, math.inf))
    return rounded


class Fixture:
    def __init__(self, name, mean, truth, observed, cells, poses, rays=24, range_cells=12):
        self.name, self.mean, self.truth, self.observed = name, mean, truth, observed
        self.cells, self.poses = tuple(cells), tuple(poses)
        self.distances = [2.0, 1.0, 1.0][:len(poses)]
        self.native = Native()
        self.native.ray_range_cells = range_cells
        self.native.mapex_num_rays = rays
        self.native.mapex_epsilon = 0.8
        self.unknown = np.isclose(observed, 0.5)
        rng = np.random.default_rng(2026100827 + len(cells))
        self.weights = (rng.random(mean.shape) * self.unknown).astype(np.float32)
        self.gain_upper = certified_sum_upper(self.weights, self.unknown)
        self.cache = {}
        self.mask_calls = 0

    def patched(self, assignment):
        out = self.mean.copy()
        for j, bit in enumerate(assignment):
            if bit == 1:
                out[self.cells[j]] = self.truth[self.cells[j]]
        return out

    def envelope(self, assignment):
        lo, hi = self.patched(assignment), self.patched(assignment)
        for j, bit in enumerate(assignment):
            if bit == -1:
                cell = self.cells[j]
                lo[cell] = min(float(self.mean[cell]), float(self.truth[cell]))
                hi[cell] = max(float(self.mean[cell]), float(self.truth[cell]))
        return lo, hi

    def score_from_endpoints(self, endpoints, frontier):
        key = (frontier, endpoints.tobytes())
        if key not in self.cache:
            pose = self.poses[frontier]
            visible = self.native.build_visibility_mask(endpoints, pose, self.mean.shape) & self.unknown
            gain = self.native.compute_information_gain(self.weights, visible)
            self.cache[key] = (gain / self.distances[frontier], visible)
            self.mask_calls += 1
        return self.cache[key]

    def scores(self, assignment):
        mean = self.patched(assignment)
        return [self.score_from_endpoints(self.native.collect_ray_endpoints(mean, pose), j)[0]
                for j, pose in enumerate(self.poses)]

    def tie_key(self, j):
        return self.distances[j], self.poses[j], j

    def winner(self, scores):
        # Same score/cost/goal ordering as the inspected local 2D Engine.score.
        # Exact controller_v2 source parity still requires a separate check.
        return min(range(len(scores)), key=lambda j: (-scores[j], *self.tie_key(j)))

    def score_bounds(self, assignment):
        lo, hi = self.envelope(assignment)
        intervals = []
        fixed = []
        for j, pose in enumerate(self.poses):
            transparent = self.native.collect_ray_endpoints(lo, pose)
            opaque = self.native.collect_ray_endpoints(hi, pose)
            is_fixed = np.array_equal(transparent, opaque)
            fixed.append(is_fixed)
            if is_fixed:
                score = self.score_from_endpoints(transparent, j)[0]
                intervals.append((score, score))
            else:
                upper = self.gain_upper / self.distances[j]
                intervals.append((0.0, upper))
        return intervals, fixed

    def possible_winners(self, intervals):
        return {a for a, (_, upper) in enumerate(intervals)
                if not any(lower > upper or (lower == upper and self.tie_key(b) < self.tie_key(a))
                           for b, (lower, _) in enumerate(intervals) if b != a)}

    def solve_actions(self, max_nodes):
        stack = [tuple([-1] * len(self.cells))]
        reachable, witnesses, nodes = set(), {}, 0
        while stack and nodes < max_nodes:
            partial = stack.pop()
            intervals, _ = self.score_bounds(partial)
            possible = self.possible_winners(intervals)
            assert possible
            nodes += 1
            if possible.issubset(reachable):
                continue
            if len(possible) == 1:
                complete = tuple(0 if bit == -1 else bit for bit in partial)
                action = self.winner(self.scores(complete))
                assert action in possible
                reachable.add(action)
                witnesses[action] = complete
            else:
                bit_index = next(j for j, bit in enumerate(partial) if bit == -1)
                for bit in (1, 0):
                    child = list(partial)
                    child[bit_index] = bit
                    stack.append(tuple(child))
        # Unprocessed correction subcubes are retained conservatively.
        upper = set(range(len(self.poses))) if stack else set(reachable)
        return reachable, upper, witnesses, nodes, bool(stack)


def ray_steps(fixture, pose):
    height, width = fixture.mean.shape
    vr, vc = pose
    result = []
    for angle in np.linspace(0.0, 2.0 * math.pi, fixture.native.mapex_num_rays):
        end = (int(vr + fixture.native.ray_range_cells * math.cos(angle)),
               int(vc + fixture.native.ray_range_cells * math.sin(angle)))
        cells = []
        for row, col in bresenham(pose, end):
            if not (0 <= row < height and 0 <= col < width):
                break
            cells.append((row, col))
        result.append({cell: j for j, cell in enumerate(cells)})
    return result


def build_fixtures():
    out = []
    shape = (9, 9)
    truth = np.zeros(shape, dtype=np.float32)
    truth[2, 2:7] = truth[6, 2:7] = 1
    truth[2:7, 2] = truth[2:7, 6] = 1
    observed = truth.copy()
    cells = [(0, 1), (0, 7), (8, 1), (8, 7), (1, 0), (7, 8)]
    mean = truth.copy()
    for cell in cells:
        observed[cell], mean[cell] = 0.5, 0.7
    out.append(Fixture("known_wall_prefix_250_rays", mean, truth, observed, cells,
                       [(3, 3), (4, 4), (5, 5)], rays=250, range_cells=200))
    rng = np.random.default_rng(2026100828)
    for case in range(7):
        truth = (rng.random(shape) < 0.10).astype(np.float32)
        poses = [(2, 2), (6, 6), (2, 6)]
        for pose in poses:
            truth[pose] = 0
        observed = truth.copy()
        legal = [(r, c) for r in range(1, 8) for c in range(1, 8) if (r, c) not in poses]
        chosen = rng.choice(len(legal), size=6, replace=False)
        cells = [legal[int(j)] for j in chosen]
        mean = truth.copy()
        for j, cell in enumerate(cells):
            observed[cell] = 0.5
            mean[cell] = np.float32([0.05, 0.25, 0.45, 0.65, 0.9, 0.3][j])
        out.append(Fixture(f"mixed_corrections_{case}", mean, truth, observed, cells, poses))
    return out


def meets(counts, target):
    # Coverage: fixed denominator; Q: rational intersection / union; TU: fixed goals.
    covered, intersection, union, success = counts
    tc, ti, tu, ts = target
    qi, qu = (1, 1) if union == 0 else (intersection, union)
    tqi, tqu = (1, 1) if tu == 0 else (ti, tu)
    quality = qi * tqu >= tqi * qu
    return covered >= tc and quality and success >= ts


def test_task_semantics():
    target = (80, 9, 10, 90)
    assert meets((80, 9, 10, 90), target)
    assert not meets((85, 8, 10, 92), target)
    assert not meets((85, 9, 10, 89), target)
    assert not meets((79, 10, 10, 100), target)
    assert meets((80, 0, 0, 90), target)
    assert not meets((80, 9, 10, 90), (80, 0, 0, 90))
    timeline = [(30, (85, 8, 10, 95)), (50, (85, 9, 10, 90)), (150, target)]
    first = min(d for d, counts in timeline if meets(counts, target))
    assert first == 50
    return {"checks": 7, "first_valid_handoff_m": first,
            "note": "A faster map with worse IoU or route success is not a same-quality success."}


def test_stable_ties():
    # min() keeps input order when score, cost and goal all match.
    # Without the final index, duplicate tied actions could leave a fully
    # assigned geometry node with no bit to branch, despite a unique selection.
    mean = np.zeros((9, 9), dtype=np.float32)
    fixture = Fixture("duplicate_goal_zero_gain", mean, mean.copy(), mean.copy(),
                      [], [(4, 4), (4, 4), (4, 4)])
    scores = fixture.scores(())
    assert scores == [0.0, 0.0, 0.0]
    assert fixture.winner(scores) == 1
    assert fixture.possible_winners([(0.0, 0.0)] * 3) == {1}
    lower, upper, witnesses, _, incomplete = fixture.solve_actions(1024)
    assert lower == upper == {1} and not incomplete
    assert witnesses[1] == ()
    return {"checks": 5, "selected_index": 1,
            "note": "Zero scores and duplicate cost/goal retain stable input ordering."}


def main():
    start = time.perf_counter()
    fixtures = build_fixtures()
    results, checks, complete_maps, ray_checks, negative_monotonic_examples = [], 0, 0, 0, []
    for fixture in fixtures:
        began = time.perf_counter()
        assignments = list(itertools.product((0, 1), repeat=len(fixture.cells)))
        lo, hi = fixture.envelope([-1] * len(fixture.cells))
        all_scores, exact_actions = {}, set()
        extrema = []
        for j, pose in enumerate(fixture.poses):
            transparent = fixture.native.collect_ray_endpoints(lo, pose)
            opaque = fixture.native.collect_ray_endpoints(hi, pose)
            steps = ray_steps(fixture, pose)
            lo_index = [steps[k][tuple(point)] for k, point in enumerate(transparent)]
            hi_index = [steps[k][tuple(point)] for k, point in enumerate(opaque)]
            min_mask = fixture.score_from_endpoints(opaque, j)[1]
            max_mask = fixture.score_from_endpoints(transparent, j)[1]
            extrema.append((transparent, opaque, steps, lo_index, hi_index, min_mask, max_mask))
        intervals, fixed = fixture.score_bounds([-1] * len(fixture.cells))
        for assignment in assignments:
            complete_maps += 1
            patched = fixture.patched(assignment)
            scores = []
            for j, pose in enumerate(fixture.poses):
                transparent, opaque, steps, lo_idx, hi_idx, min_mask, max_mask = extrema[j]
                endpoints = fixture.native.collect_ray_endpoints(patched, pose)
                for k, point in enumerate(endpoints):
                    index = steps[k][tuple(point)]
                    assert hi_idx[k] <= index <= lo_idx[k]
                    ray_checks += 1
                score, mask = fixture.score_from_endpoints(endpoints, j)
                lower, upper = intervals[j]
                assert lower <= score <= upper
                checks += 1
                if fixed[j]:
                    assert np.array_equal(endpoints, transparent)
                    assert np.array_equal(mask, max_mask)
                    checks += 1
                if (np.any(min_mask & ~mask) or np.any(mask & ~max_mask)) and len(negative_monotonic_examples) < 3:
                    negative_monotonic_examples.append({"fixture": fixture.name, "frontier": j,
                                                        "assignment": assignment,
                                                        "min_mask_outside_actual": int(np.sum(min_mask & ~mask)),
                                                        "actual_outside_max_mask": int(np.sum(mask & ~max_mask))})
                scores.append(score)
            all_scores[assignment] = scores
            exact_actions.add(fixture.winner(scores))
        solver_rows = []
        for cap in (0, 1, 4, 16, 1024):
            lower, upper, witnesses, nodes, incomplete = fixture.solve_actions(cap)
            assert lower.issubset(exact_actions) and exact_actions.issubset(upper)
            if not incomplete:
                assert lower == exact_actions == upper
            for action, assignment in witnesses.items():
                assert fixture.winner(all_scores[assignment]) == action
            checks += 3 + len(witnesses)
            solver_rows.append({"cap": cap, "nodes": nodes, "incomplete": incomplete,
                                "reachable_lower": sorted(lower), "reachable_upper": sorted(upper)})
        results.append({"fixture": fixture.name, "correctable_cells": len(fixture.cells),
                        "all_corrections": len(assignments), "rays": fixture.native.mapex_num_rays,
                        "fixed_frontiers": sum(fixed), "frontiers": len(fixed),
                        "exact_reachable_actions": sorted(exact_actions), "solver": solver_rows,
                        "native_mask_evaluations": fixture.mask_calls,
                        "wall_s": time.perf_counter() - began})
        print(json.dumps({"done": fixture.name, "complete_maps": complete_maps}, ensure_ascii=False), flush=True)
    for weights in [np.array([[0.0, np.nextafter(np.float32(0), np.float32(1)), 0.3, 0.7]], dtype=np.float32),
                    np.array([[1.0, 1.0e-30, 1.0e-12, 0.0]], dtype=np.float32),
                    np.array([[3.0e38, 3.0e38, 0.0, 0.0]], dtype=np.float32)]:
        upper = certified_sum_upper(weights, np.ones(weights.shape, dtype=bool))
        for bits in itertools.product((False, True), repeat=weights.size):
            with np.errstate(over="ignore"):
                value = float(np.sum(weights.reshape(-1)[np.array(bits)]))
            assert value <= upper
            checks += 1
    import shapely
    output = {"status": "PASS_SMALL_FIXTURES_ONLY", "source": str(SOURCE), "source_sha256": source_hash,
              "source_scope": "Existing local repository snapshot; exact R4 execution-head parity not established.",
              "numpy": np.__version__, "shapely": shapely.__version__, "seed": 2026100828,
              "checks": checks, "ray_order_checks": ray_checks, "complete_correction_maps": complete_maps,
              "fixtures": results, "task_semantics": test_task_semantics(),
              "tie_semantics": test_stable_ties(),
              "mask_monotonic_counterexamples": negative_monotonic_examples,
              "wall_s": time.perf_counter() - start,
              "not_run": ["LaMa inference", "real saved decision snapshots", "whole-episode search",
                          "held-out building collection", "research impact estimation"]}
    (ROOT / "preflight_results.json").write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: output[k] for k in ["status", "checks", "ray_order_checks",
                                           "complete_correction_maps", "wall_s"]}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
