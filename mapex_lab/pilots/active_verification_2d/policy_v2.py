"""Route-aware error correction and local structural-correction ablation.

No assets, evaluator, world or truth labels are accessible here. The structural
term is a weighted local counterfactual, not a Bayes value-of-information claim.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .core import (UNKNOWN, observed_traversable, shortest_paths, recover_path,
                   frontier_representatives, clearance_free, reachable_component)
from .policy import hypotheses, observation_poses, choose as original_choose


class PredictedSensor:
    """Vectorized first-hit scans on a predicted hard map, including the return."""
    def __init__(self, resolution, range_m, rays=360, step_cells=.5):
        angles = np.linspace(0, 2*np.pi, rays, endpoint=False)
        steps = np.arange(0, range_m/resolution+.25, step_cells)
        self.dr = np.floor(steps[:, None]*np.sin(angles)[None, :]+.5).astype(np.int32)
        self.dc = np.floor(steps[:, None]*np.cos(angles)[None, :]+.5).astype(np.int32)

    def visible(self, occupied, pose):
        h, w = occupied.shape
        rr, cc = pose[0]+self.dr, pose[1]+self.dc
        valid = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        blocked = ~valid | occupied[np.clip(rr, 0, h-1), np.clip(cc, 0, w-1)]
        # Stop after the first hit, but include the first occupied cell itself.
        prior_blocks = np.cumsum(blocked, axis=0)-blocked
        use = valid & (prior_blocks == 0)
        out = np.zeros((h, w), dtype=bool)
        out[rr[use], cc[use]] = True
        return out


def visible_targets(occupied, pose, targets, resolution, range_m):
    """Target-directed half-cell rays; include a first return at a target.

    This is a visibility approximation for a dense 360-degree sensor, not the
    sparse seven-probe fraction used in V1. Geometry is always hypothetical.
    """
    targets = np.asarray(targets, dtype=int)
    delta = targets-np.asarray(pose)
    distance = np.linalg.norm(delta, axis=1)
    out = np.zeros(len(targets), dtype=bool)
    eligible = np.flatnonzero(distance*resolution <= range_m+1e-9)
    if not len(eligible):
        return out
    delta = delta[eligible]
    n = max(1, int(math.ceil(float(distance[eligible].max())*2)))
    fractions = np.linspace(0, 1, n+1)
    rr = np.floor(pose[0]+fractions[:, None]*delta[None, :, 0]+.5).astype(int)
    cc = np.floor(pose[1]+fractions[:, None]*delta[None, :, 1]+.5).astype(int)
    h, w = occupied.shape
    valid = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
    blocked = ~valid | occupied[np.clip(rr, 0, h-1), np.clip(cc, 0, w-1)]
    first = np.argmax(blocked, axis=0)
    columns = np.arange(len(eligible))
    target = targets[eligible]
    at_target = (rr[first, columns] == target[:, 0]) & (cc[first, columns] == target[:, 1])
    out[eligible] = (~blocked.any(axis=0)) | at_target
    return out


def route_anchors(path, stride):
    # The current pose has already been sensed; do not count a zero-cost scan.
    indices = list(range(stride, len(path), stride))
    if len(path) > 1 and (not indices or indices[-1] != len(path)-1):
        indices.append(len(path)-1)
    return [path[i] for i in indices]


@dataclass
class Route:
    goal: tuple[int, int]
    original_target: tuple[int, int]
    path: list
    anchors: list
    cost_m: float


def routes_from_views(state, views, parent, cfg):
    max_steps = int(math.floor((state.remaining_m+1e-8)/state.resolution))
    routes = {}
    for target in views:
        path = recover_path(parent, state.pose, target)
        if path is None:
            continue
        path = path[:max_steps+1]
        if len(path) < 2:
            continue
        goal = path[-1]
        route = Route(goal, target, path, route_anchors(path, cfg["route_scan_stride_cells"]),
                      (len(path)-1)*state.resolution)
        # Different remote viewpoints can induce the same budget prefix.
        if goal not in routes or target < routes[goal].original_target:
            routes[goal] = route
    out = sorted(routes.values(), key=lambda x: x.goal)
    distant = [r for r in out if r.cost_m >= cfg["preferred_route_length_m"]-1e-8]
    return distant or out


def counterfactual_gain(base_component, alternative_component, partial_component, unknown, resolution):
    before = np.count_nonzero((base_component ^ alternative_component) & unknown)
    after = np.count_nonzero((partial_component ^ alternative_component) & unknown)
    return max(0., float(before-after)*resolution**2)


def hypothesis_info(h, record):
    return dict(centre=list(h.centre), intervention=h.intervention, orientation=h.orientation,
                impact_m2=h.impact_m2, agreement=h.ensemble_agreement,
                variance=h.ensemble_variance_mean, patch_cells=h.patch_cells.tolist(),
                error_weight=record["error_weight"],
                predicted_patch_revealed_cells=record["predicted_patch_revealed_cells"],
                counterfactual_correction_m2=record["counterfactual_correction_m2"])


def choose_v2(state, method, numerics, cfg, risk_model):
    if method in {"mapex", "uncertainty", "nearest", "structural"}:
        return original_choose(state, method, numerics, cfg)
    if method not in {"route_uncertainty", "route_error", "structural_v2"}:
        raise ValueError("Unknown V2 method: " + method)
    traversable = observed_traversable(state.observed, cfg["robot_radius_m"]/state.resolution)
    dist, parent = shortest_paths(traversable, state.pose)
    frontiers = frontier_representatives(state.observed, cfg["frontier_min_size_strict"])
    hs, stats = hypotheses(state, cfg)
    views = observation_poses(state, dist, frontiers, hs, cfg)
    routes = routes_from_views(state, views, parent, cfg)
    common = dict(raw_frontiers=len(frontiers), hypotheses=len(hs), observation_poses=len(views),
                  executable_routes=len(routes), variant="ROUTE_RISK_V2", **stats)
    if not routes:
        goal, parent, detail = original_choose(state, "mapex", numerics, cfg)
        detail.update(action="EXPLORE_V2_NO_ROUTE_FALLBACK", **common)
        return goal, parent, detail
    unknown = state.observed == UNKNOWN
    occupied = state.mean >= .5
    sensor = PredictedSensor(state.resolution, cfg["sensor_range_m"], cfg["route_scan_rays"], cfg["route_scan_step_cells"])
    visibility = {}
    for route in routes:
        for pose in route.anchors:
            if pose not in visibility:
                visibility[pose] = sensor.visible(occupied, pose) & unknown
    risk = risk_model.risk(state) if method != "route_uncertainty" else None
    radius = cfg["robot_radius_m"]/state.resolution
    base_component = reachable_component(clearance_free(~occupied, radius), state.pose) if method == "structural_v2" else None
    alternatives = []
    if method == "structural_v2":
        for h in hs:
            alternative = occupied.copy()
            rr, cc = h.patch_cells.T
            alternative[rr, cc] = h.intervention == "close"
            changed = occupied[rr, cc] != alternative[rr, cc]
            if not changed.any():
                continue
            component = reachable_component(clearance_free(~alternative, radius), state.pose)
            # Mean cell error is an uncalibrated whole-hypothesis weight.
            weight = float(risk[rr[changed], cc[changed]].mean())
            per_pose = {p: visible_targets(alternative, p, h.patch_cells, state.resolution,
                                           cfg["sensor_range_m"]) for p in visibility}
            alternatives.append((h, alternative, component, weight, per_pose))
    records, best = [], None
    for route in routes:
        mask = np.logical_or.reduce([visibility[p] for p in route.anchors])
        error_area = 0. if risk is None else float(risk[mask].sum()*state.resolution**2)
        ig = float(state.variance[mask].sum()*state.resolution**2)
        structure_gain, selected_h, selected_record = 0., None, None
        for h, alternative, component, weight, per_pose in alternatives:
            seen = np.logical_or.reduce([per_pose[p] for p in route.anchors])
            if not seen.any():
                continue
            rr, cc = h.patch_cells[seen].T
            partial = occupied.copy()
            partial[rr, cc] = alternative[rr, cc]
            corrected = reachable_component(clearance_free(~partial, radius), state.pose)
            gain = counterfactual_gain(base_component, component, corrected, unknown, state.resolution)
            value = weight*gain
            if value > structure_gain:
                structure_gain, selected_h = value, h
                selected_record = dict(error_weight=weight, predicted_patch_revealed_cells=int(seen.sum()),
                                       counterfactual_correction_m2=gain)
        benefit = ig if method == "route_uncertainty" else error_area + cfg["structural_term_weight"]*structure_gain
        score = benefit/max(route.cost_m, state.resolution)
        record = dict(pose=list(route.goal), original_target=list(route.original_target), path_m=route.cost_m,
                      score=score, visible_unknown_cells=int(mask.sum()), route_uncertainty_m2=ig,
                      estimated_wrong_area_m2=error_area, weighted_structural_correction_m2=structure_gain,
                      hypothetical_correction=selected_record,
                      hypothesis_centre=None if selected_h is None else list(selected_h.centre))
        records.append(record)
        key = (-score, route.cost_m, route.goal)
        if best is None or key < best[0]:
            best = key, route, selected_h, selected_record, record
    _, route, h, h_record, record = best
    records.sort(key=lambda r: (-r["score"], r["path_m"], r["pose"]))
    if record["score"] <= cfg["minimum_route_score"]:
        goal, parent, detail = original_choose(state, "mapex", numerics, cfg)
        detail.update(action="EXPLORE_V2_ZERO_GAIN_FALLBACK", candidate_records=records, **common)
        return goal, parent, detail
    return route.goal, parent, dict(action="VERIFY" if h is not None else "OBSERVE_"+method.upper(),
                                   goal=list(route.goal), score=record["score"], path_m=route.cost_m,
                                   hypothesis=None if h is None else hypothesis_info(h, h_record),
                                   selected_route=record, candidate_records=records, **common)
