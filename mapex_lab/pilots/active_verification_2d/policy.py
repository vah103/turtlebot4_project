"""Geometry-based counterfactual gate verification, with no truth access."""
from __future__ import annotations

from dataclasses import dataclass, asdict
import math

import numpy as np
from scipy import ndimage as ndi

from .core import (UNKNOWN, FREE, OCCUPIED, PolicyInput, MapExNumerics,
                   observed_traversable, shortest_paths, frontier_representatives,
                   clearance_free, reachable_component, line_cells)


@dataclass
class Hypothesis:
    centre: tuple[int, int]
    orientation: str
    intervention: str
    patch_cells: np.ndarray
    impact_m2: float
    ensemble_variance_mean: float
    ensemble_agreement: float


def hypotheses(state, cfg):
    """Local open/closed gate alternatives with geometric supporting cues.

    Close alternatives span a narrow predicted passage bounded by obstacles.
    Open alternatives cross a thin predicted obstacle with free space on both
    sides. Only UNKNOWN cells can be changed, and consequence is measured by
    the changed reachable component under a disk footprint.
    """
    res = state.resolution
    free = state.mean < 0.5
    unknown = state.observed == UNKNOWN
    h, w = free.shape
    span = max(2, int(round(cfg["gate_span_m"] / (2 * res))))
    depth = max(1, int(round(cfg["gate_depth_m"] / (2 * res))))
    side = max(depth + 1, int(round(cfg["open_side_offset_m"] / res)))
    near_observed = ndi.distance_transform_edt(unknown) * res <= cfg["sensor_range_m"]
    narrow = ndi.distance_transform_edt(free) * res <= cfg["gate_span_m"] / 2
    eligible = unknown & near_observed & ((free & narrow) | ~free)
    coordinates = np.argwhere(eligible)
    stride = cfg["gate_sampling_stride_cells"]
    coordinates = coordinates[(coordinates[:, 0] % stride == 0) & (coordinates[:, 1] % stride == 0)]
    raw = []
    for r, c in coordinates:
        r, c = int(r), int(c)
        if min(r, c) <= span + side or r + span + side >= h or c + span + side >= w:
            continue
        for orientation in ("vertical", "horizontal"):
            if orientation == "vertical":
                patch_r, patch_c = np.mgrid[r-span:r+span+1, c-depth:c+depth+1]
                # A vertical gate cuts a left-to-right passage.
                ends = [(r, c-side), (r, c+side)]
                transverse = [free[r-span:r-depth, c].mean(), free[r+depth+1:r+span+1, c].mean()]
            else:
                patch_r, patch_c = np.mgrid[r-depth:r+depth+1, c-span:c+span+1]
                ends = [(r-side, c), (r+side, c)]
                transverse = [free[r, c-span:c-depth].mean(), free[r, c+depth+1:c+span+1].mean()]
            if not all(free[p] for p in ends):
                continue
            if free[r, c]:
                if not all(value < 0.5 for value in transverse):
                    continue
                intervention = "close"
            else:
                # Across a thin barrier, every point outside the local gate
                # along the crossing line must already be predicted free.
                crossed = [(r, x) for x in range(c-side, c+side+1)] if orientation == "vertical" else [(x, c) for x in range(r-side, r+side+1)]
                outside = [p for p in crossed if (abs(p[1]-c) > depth if orientation == "vertical" else abs(p[0]-r) > depth)]
                if not all(free[p] for p in outside):
                    continue
                intervention = "open"
            mask = unknown[patch_r, patch_c]
            rr, cc = patch_r[mask], patch_c[mask]
            if len(rr) < cfg["min_unknown_gate_cells"]:
                continue
            if any(kind == intervention and axis == orientation and math.hypot(r-r0, c-c0)*res < cfg["gate_nms_m"]
                   for r0, c0, axis, kind, _ in raw):
                continue
            raw.append((r, c, orientation, intervention, np.column_stack((rr, cc))))
    if len(raw) > cfg["max_geometry_proposals"]:
        indices = np.linspace(0, len(raw)-1, cfg["max_geometry_proposals"]).astype(int)
        raw = [raw[i] for i in indices]
    radius = cfg["robot_radius_m"] / res
    base = reachable_component(clearance_free(free, radius), state.pose)
    out = []
    for r, c, orientation, intervention, patch in raw:
        alternative = free.copy()
        alternative[patch[:, 0], patch[:, 1]] = intervention == "open"
        changed = reachable_component(clearance_free(alternative, radius), state.pose)
        impact = float(((base ^ changed) & unknown).sum() * res ** 2)
        if impact < cfg["min_structural_impact_m2"]:
            continue
        votes = state.predictions[:, patch[:, 0], patch[:, 1]] >= 0.5
        agreement = np.maximum(votes.sum(axis=0), 3-votes.sum(axis=0)) / 3
        out.append(Hypothesis((r, c), orientation, intervention, patch, impact,
                              float(state.variance[patch[:, 0], patch[:, 1]].mean()),
                              float(agreement.mean())))
    out.sort(key=lambda x: (-x.impact_m2, x.centre, x.orientation, x.intervention))
    return out[:cfg["max_hypotheses"]], dict(geometry_proposals=len(raw), consequence_proposals=len(out))


def observation_poses(state, dist, frontiers, hs, cfg):
    """Same broader observation set for structural and uncertainty policies."""
    points = set(p for p in frontiers if dist[p] > 0)
    near_unknown = ndi.distance_transform_edt(state.observed != UNKNOWN) * state.resolution <= cfg["view_band_m"]
    pool = np.argwhere((dist > 0) & near_unknown)
    if len(pool):
        queries = list(frontiers) + [h.centre for h in hs]
        for centre in queries:
            squared = np.sum((pool - centre) ** 2, axis=1)
            order = np.lexsort((pool[:, 1], pool[:, 0], squared))
            for index in order[:cfg["views_per_query"]]:
                points.add(tuple(int(x) for x in pool[index]))
        order = np.lexsort((pool[:, 1], pool[:, 0], dist[pool[:, 0], pool[:, 1]]))
        indices = np.linspace(0, len(order)-1, min(cfg["uniform_extra_views"], len(order))).astype(int)
        for index in indices:
            points.add(tuple(int(x) for x in pool[order[index]]))
    # Cap using proximity to a query and path cost, not truth or error labels.
    points = sorted(points, key=lambda p: (min([math.dist(p, h.centre) for h in hs] + [math.dist(p, state.pose)]), int(dist[p]), p))
    return points[:cfg["max_observation_poses"]]


def observable_fraction(state, pose, hypothesis, cfg):
    """Sparse patch visibility through the common predicted surroundings.

    Patch cells are the contested part: a first return there distinguishes the
    local alternatives. Before reaching the patch, the accumulated occupancy
    cutoff is the same 0.8 as MapEx. This is a geometric proxy, not a calibrated
    probability that a hypothesis is wrong.
    """
    patch = hypothesis.patch_cells
    selected = patch[np.linspace(0, len(patch)-1, min(len(patch), cfg["probe_target_samples"])).astype(int)]
    patch_set = set(map(tuple, patch.tolist()))
    visible = 0
    for target in selected:
        target = tuple(int(x) for x in target)
        if math.dist(pose, target) * state.resolution > cfg["sensor_range_m"]:
            continue
        occupancy = 0.0
        for p in line_cells(pose, target):
            if p in patch_set:
                visible += 1
                break
            occupancy += float(state.mean[p])
            if occupancy >= 0.8:
                break
    return visible / max(1, len(selected))


def choose(state, method, numerics, cfg):
    traversable = observed_traversable(state.observed, cfg["robot_radius_m"] / state.resolution)
    dist, parent = shortest_paths(traversable, state.pose)
    frontiers = frontier_representatives(state.observed, cfg["frontier_min_size_strict"])
    raw_distant = any(math.dist(p, state.pose)*state.resolution >= 1.0 for p in frontiers)
    ordinary = [p for p in frontiers if dist[p] > 0 and
                (not raw_distant or math.dist(p, state.pose)*state.resolution >= 1.0)]
    common = dict(raw_frontiers=len(frontiers), reachable_frontiers=len(ordinary), near_frontier_fallback=not raw_distant)
    hs, diagnostic = hypotheses(state, cfg) if method in {"structural", "uncertainty"} else ([], {})
    views = observation_poses(state, dist, frontiers, hs, cfg) if method in {"structural", "uncertainty"} else ordinary
    vis_cache = {}
    def gain(p):
        if p not in vis_cache:
            mask = numerics.visibility(p, state.mean, state.observed)
            vis_cache[p] = float(state.variance[mask].sum())
        return vis_cache[p]
    action, goal, score, selected_h = "EXPLORE", None, -math.inf, None
    records = []
    if method == "structural":
        for h in hs:
            for p in views:
                visible = observable_fraction(state, p, h, cfg)
                cost = max(state.resolution, dist[p]*state.resolution)
                utility = h.impact_m2 * visible / cost
                if utility > 0:
                    records.append(dict(pose=list(p), hypothesis=list(h.centre), impact_m2=h.impact_m2,
                                        observable_fraction=visible, path_m=cost, score=utility,
                                        intervention=h.intervention, orientation=h.orientation,
                                        agreement=h.ensemble_agreement, variance=h.ensemble_variance_mean))
                    if utility > score or (utility == score and (goal is None or p < goal)):
                        goal, score, selected_h = p, utility, h
        if goal is not None:
            action = "VERIFY"
    if method == "uncertainty":
        for p in views:
            value = gain(p) / max(state.resolution, dist[p]*state.resolution)
            if value > score or (value == score and (goal is None or p < goal)):
                goal, score = p, value
        action = "OBSERVE_UNCERTAINTY"
    if method in {"mapex", "nearest"} or goal is None:
        action = "EXPLORE" if method != "structural" else "EXPLORE_FALLBACK"
        score = -math.inf
        for p in ordinary:
            euclidean = max(state.resolution, math.dist(p, state.pose)*state.resolution)
            value = -euclidean if method == "nearest" else gain(p) / euclidean
            if value > score or (value == score and (goal is None or p < goal)):
                goal, score = p, value
    if goal is None:
        action, score = "NO_REACHABLE_CANDIDATE", 0.0
    hypothesis_info = None
    if selected_h is not None:
        hypothesis_info = dict(centre=list(selected_h.centre), intervention=selected_h.intervention,
                               orientation=selected_h.orientation, impact_m2=selected_h.impact_m2,
                               agreement=selected_h.ensemble_agreement, variance=selected_h.ensemble_variance_mean,
                               patch_cells=selected_h.patch_cells.tolist())
    records.sort(key=lambda x: (-x["score"], x["pose"], x["hypothesis"]))
    return goal, parent, dict(action=action, goal=None if goal is None else list(goal),
                              score=float(score), path_m=0.0 if goal is None else float(dist[goal]*state.resolution),
                              hypothesis=hypothesis_info, candidate_records=records[:12],
                              hypotheses=len(hs), observation_poses=len(views), **common, **diagnostic)
