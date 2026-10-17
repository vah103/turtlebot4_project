"""G2-R6: light room-local Watchman heuristic; no GT and no UNKNOWN traversal.

Architecture:
 * One LaMa/RoomNode epoch => known-free candidate routes + pathwise sensing.
 * Greedy marginal uncovered observation value / extra route distance.
 * Bounded waypoint-order improvement, then fast scan-time invalidation/pruning.
 * Only native confirmed observed FREE cells may be commanded.
 * All visibility values are heuristic predictions, NOT true coverage.
"""
from __future__ import annotations

import hashlib
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class R6Config:
    resolution_m: float = 0.10
    lidar_range_m: float = 20.0
    ray_beams: int = 80
    ray_step_cells: float = 2.0
    scan_stride_cells: int = 8
    candidate_tile_cells: int = 18
    max_candidates: int = 20
    max_goals: int = 5
    max_order_improvements: int = 10
    minimum_new_weight: float = 18.0
    minimum_value_per_meter: float = 12.0
    min_keep_goal_weight: float = 15.0
    heavy_new_cells: int = 200
    large_structural_change_fraction: float = 0.05
    nominal_weight: float = 0.7


def sha(a: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def room_connected(obs, semantic, robot):
    room = (semantic == 1) & (obs != 2)
    count, components = cv2.connectedComponents(room.astype(np.uint8), connectivity=4)
    rid = int(components[robot])
    return components == rid if rid > 0 else np.zeros_like(room, bool)


def _walkable(obs, semantic, robot, native_walkable):
    room = room_connected(obs, semantic, robot)
    legal = room & (obs == 0) & np.asarray(native_walkable, bool)
    if not legal[robot]:
        raise ValueError("robot not in native-certified reachable known-free room")
    # Native mask may contain other disconnected FREE components.
    _, labels = cv2.connectedComponents(legal.astype(np.uint8), connectivity=4)
    return labels == labels[robot], room


class _KnownFreePaths:
    """4-connected BFS; lazy caching by start, never traverses UNKNOWN."""
    def __init__(self, legal):
        self.legal = np.asarray(legal, bool)
        self.height, self.width = legal.shape
        self.flat = legal.ravel()
        self.cache = {}

    def _build(self, start):
        h, w = self.height, self.width
        s = int(start[0]) * w + int(start[1])
        parent = np.full(h * w, -2, dtype=np.int32)
        if not self.flat[s]:
            return parent
        parent[s] = -1
        queue = deque([s])
        while queue:
            p = queue.popleft()
            y, x = divmod(p, w)
            for q in ((p-w if y else -1), (p+w if y+1<h else -1),
                      (p-1 if x else -1), (p+1 if x+1<w else -1)):
                if q >= 0 and self.flat[q] and parent[q] == -2:
                    parent[q] = p
                    queue.append(q)
        return parent

    def path(self, start, end):
        start = tuple(map(int, start))
        end = tuple(map(int, end))
        if start not in self.cache:
            self.cache[start] = self._build(start)
        parent = self.cache[start]
        q = end[0]*self.width+end[1]
        if q < 0 or q >= parent.size or parent[q] == -2:
            return None
        p = []
        while q >= 0:
            p.append((int(q//self.width), int(q%self.width)))
            q = int(parent[q])
        p.reverse()
        return p


class _PredictedSight:
    """Vectorized 2D raycasting; caches each viewing pose within one LaMa epoch."""
    def __init__(self, obs, mean, var, room, target, cfg):
        self.obs = obs
        self.target = target
        self.cfg = cfg
        self.cache = {}
        self.shape = obs.shape
        ang = np.linspace(0, 2*np.pi, cfg.ray_beams, endpoint=False)
        radii = np.arange(0, cfg.lidar_range_m/cfg.resolution_m + 0.01,
                          cfg.ray_step_cells)
        self.offset_y = np.rint(np.cos(ang)[:, None]*radii[None, :]).astype(np.int16)
        self.offset_x = np.rint(np.sin(ang)[:, None]*radii[None, :]).astype(np.int16)
        nominal = np.clip(mean, 0, 1)
        stress = np.clip(mean + np.sqrt(np.maximum(0, var)), 0, 1)
        self.blockers = ((obs == 2) | (~room) | ((obs == 1) & (nominal >= .5)),
                         (obs == 2) | (~room) | ((obs == 1) & (stress >= .5)))

    def at(self, point):
        point = tuple(map(int, point))
        if point in self.cache:
            return self.cache[point]
        h, w = self.shape
        y = point[0] + self.offset_y.astype(np.int32)
        x = point[1] + self.offset_x.astype(np.int32)
        valid = (y >= 0) & (x >= 0) & (y < h) & (x < w)
        yc, xc = np.clip(y, 0, h-1), np.clip(x, 0, w-1)
        masks = []
        for blocker in self.blockers:
            hit = (~valid) | blocker[yc, xc]
            previous_hit = np.zeros_like(hit)
            previous_hit[:, 1:] = np.maximum.accumulate(hit[:, :-1], axis=1)
            visible = valid & (~previous_hit)
            m = np.zeros(self.shape, bool)
            m[yc[visible], xc[visible]] = True
            masks.append(m & self.target)
        self.cache[point] = (masks[0], masks[1])
        return self.cache[point]

    def route(self, path):
        first = np.zeros(self.shape, bool)
        second = first.copy()
        poses = path[::self.cfg.scan_stride_cells]
        if not poses or poses[-1] != path[-1]:
            poses = list(poses) + [path[-1]]
        for p in poses:
            a, b = self.at(p)
            first |= a
            second |= b
        return first, second


class RoomWatchman:
    def __init__(self, observed, mean, variance, semantic, robot,
                 native_walkable, cfg: Optional[R6Config] = None):
        self.cfg = cfg or R6Config()
        self.obs = np.asarray(observed, np.uint8).copy()
        self.mean = np.asarray(mean, np.float32).copy()
        self.var = np.asarray(variance, np.float32).copy()
        self.semantic = np.asarray(semantic, np.uint8).copy()
        self.robot = tuple(map(int, robot))
        if not all(x.shape == self.obs.shape for x in
                   (self.mean, self.var, self.semantic, native_walkable)):
            raise ValueError("all input maps must have the same shape")
        if not np.isin(self.obs, (0, 1, 2)).all() or self.obs[self.robot] != 0:
            raise ValueError("observed coding must be 0 FREE/1 UNKNOWN/2 WALL")
        if not np.isfinite(self.mean).all() or not np.isfinite(self.var).all():
            raise ValueError("non-finite predicted maps")
        self.legal, self.room = _walkable(self.obs, self.semantic, self.robot, native_walkable)
        self.mission = self.room & (self.obs == 1)
        if not self.mission.any():
            raise ValueError("no unresolved room cells; use LOCAL WFD exit gate")
        self.weight = np.clip(1-self.mean, .10, 1) * (1-.35*np.sqrt(np.maximum(self.var, 0)))
        self.weight = np.maximum(self.weight, .1)
        self.paths = _KnownFreePaths(self.legal)
        self.rays = _PredictedSight(self.obs, self.mean, self.var, self.room,
                                   self.mission, self.cfg)
        self.cache_cover = {}
        self.plan_order = []
        self.prediction_sha256 = sha(self.mean)
        self.input_sha256 = sha(self.obs)
        self.last_decision = None

    def candidates(self):
        """One strong FREE proposal per spatial tile; favor dense unresolved areas."""
        cfg = self.cfg
        unknown_score = (self.mission.astype(np.float32)*self.weight).astype(np.float32)
        radius = 23
        density = cv2.boxFilter(unknown_score, -1, (radius, radius), normalize=False)
        ys, xs = np.nonzero(self.legal)
        scores = density[ys, xs] / (1 + .008*(abs(ys-self.robot[0])+abs(xs-self.robot[1])))
        tiled = {}
        for y, x, score in zip(ys.tolist(), xs.tolist(), scores.tolist()):
            tile = (y//cfg.candidate_tile_cells, x//cfg.candidate_tile_cells)
            if tile not in tiled or score > tiled[tile][0]:
                tiled[tile] = (score, (y, x))
        top = sorted(tiled.values(), key=lambda v: (-v[0], v[1]))
        # Include diverse spatial candidates, not only nearest frontier centres.
        selected = [p for score, p in top[:cfg.max_candidates] if p != self.robot]
        return selected

    def _route_cover(self, path):
        key = (tuple(path[0]), tuple(path[-1]))
        if key not in self.cache_cover:
            self.cache_cover[key] = self.rays.route(path)
        return self.cache_cover[key]

    def _weighted(self, cover, unresolved=None):
        if unresolved is None:
            unresolved = self.mission
        n, s = cover
        cfg = self.cfg
        return float((self.weight * unresolved *
             (cfg.nominal_weight*n + (1-cfg.nominal_weight)*s)).sum())

    def _route_order(self, order, start=None):
        current = self.robot if start is None else tuple(start)
        a = np.zeros(self.obs.shape, bool)
        b = a.copy()
        route = []
        dist = 0
        legs = []
        for goal in order:
            seg = self.paths.path(current, goal)
            if not seg:
                return None
            ca, cb = self._route_cover(seg)
            a |= ca
            b |= cb
            dist += len(seg)-1
            route.extend(seg if not route else seg[1:])
            legs.append({"goal_rc": list(goal), "distance_leg_m": round((len(seg)-1)*self.cfg.resolution_m, 3)})
            current = goal
        return {"cover": (a, b), "route": route,
                "distance_m": dist*self.cfg.resolution_m, "legs": legs}

    def plan(self):
        began = time.perf_counter()
        cfg = self.cfg
        points = self.candidates()
        current = self.robot
        used = set()
        covered = (np.zeros(self.obs.shape, bool), np.zeros(self.obs.shape, bool))
        chosen = []
        trace = []
        for _ in range(cfg.max_goals):
            options = []
            for point in points:
                if point in used:
                    continue
                seg = self.paths.path(current, point)
                if seg is None or len(seg) <= 1:
                    continue
                vis = self._route_cover(seg)
                new = (vis[0] & ~covered[0], vis[1] & ~covered[1])
                value = self._weighted(new)
                length = (len(seg)-1)*cfg.resolution_m
                ratio = value/(length + .8)
                if value >= cfg.minimum_new_weight and ratio >= cfg.minimum_value_per_meter:
                    options.append((ratio, value, -length, point, vis, seg))
            if not options:
                break
            ratio, value, _, point, vis, seg = max(options, key=lambda x:(x[0],x[1],x[2],tuple(-z for z in x[3])))
            chosen.append(point)
            trace.append({"goal_rc": list(point), "new_value": round(value, 2),
                          "value_per_meter_regularized": round(ratio, 2),
                          "leg_m": round((len(seg)-1)*cfg.resolution_m, 3)})
            covered = (covered[0] | vis[0], covered[1] | vis[1])
            used.add(point)
            current = point
        initial = self._route_order(chosen)
        if initial is None:
            raise RuntimeError("selected route unexpectedly invalid")
        # Bounded 2-opt, accept only if projected visibility remains comparable.
        initial_length = initial["distance_m"]
        initial_value = self._weighted(initial["cover"])
        refinements = 0
        for _ in range(cfg.max_order_improvements):
            best = None
            if len(chosen) < 3:
                break
            for i in range(len(chosen)-1):
                for j in range(i+1,len(chosen)):
                    alt = chosen[:i] + list(reversed(chosen[i:j+1])) + chosen[j+1:]
                    trial = self._route_order(alt)
                    if trial is None:
                        continue
                    value = self._weighted(trial["cover"])
                    if value < .98*initial_value:
                        continue
                    # Fewer metres and no material visibility loss.
                    if trial["distance_m"] < initial["distance_m"] - .099:
                        key = (trial["distance_m"], -value, alt)
                        if best is None or key < best[0]:
                            best = (key, alt, trial, value)
            if best is None:
                break
            _, chosen, initial, initial_value = best
            refinements += 1
        self.plan_order = chosen[:]
        self.last_decision = {
            "version": "G2_R6_WATCHMAN_R1",
            "status": "CONDITIONAL_SAFE_ROUTE" if chosen else "DEFER_NO_POSITIVE_VIEWPOINT",
            "goal_sequence_core_rc": [list(p) for p in chosen],
            "legs": initial["legs"],
            "planned_grid_route_core_rc": [list(p) for p in initial["route"]],
            "projected_unique_unknown_nominal": int((initial["cover"][0] & self.mission).sum()),
            "projected_unique_unknown_stress": int((initial["cover"][1] & self.mission).sum()),
            "projected_weighted_gain": round(initial_value, 2),
            "initial_greedy_trace": trace,
            "order_improvements": refinements,
            "distance_before_order_improvements_m": round(initial_length, 3),
            "predicted_route_distance_m": round(initial["distance_m"], 3),
            "mission_unknown": int(self.mission.sum()),
            "candidate_count": len(points),
            "extra_G2_planning_wall_s": round(time.perf_counter()-began, 4),
            "input_observed_sha256": self.input_sha256,
            "LaMa_mean_sha256": self.prediction_sha256,
            "used_gt": False, "used_future_observations": False,
            "goals_only_native_known_free": all(self.legal[p] for p in chosen),
            "no_predicted_unknown_travel": True,
            "is_actual_coverage_certified": False,
        }
        return self.last_decision

    def scan_update(self, new_observed, current_robot, native_walkable=None,
                    semantic=None):
        """Cheap scan-time refresh. No LaMa and no full greedy beam search.

        Return REPLAN_REQUIRED when any committed route segment is unsafe or
        current structure has materially changed. No action commands emitted.
        """
        began = time.perf_counter()
        new = np.asarray(new_observed, dtype=np.uint8)
        robot = tuple(map(int, current_robot))
        if new.shape != self.obs.shape or not np.isin(new, (0, 1, 2)).all():
            raise ValueError("bad scan map")
        if new[robot] != 0:
            return {"action": "REPLAN_REQUIRED", "reason": "ROBOT_NOT_OBSERVED_FREE"}
        old_obs = self.obs.copy()
        newly_seen = (old_obs == 1) & (new != 1)
        wall_corrections = (old_obs == 0) & (new == 2)
        invalid = (old_obs != 1) & (old_obs != new) & ~wall_corrections
        if invalid.any():
            return {"action": "REPLAN_REQUIRED", "reason": "NON_MONOTONIC_OBSERVATIONS"}
        changed = int(newly_seen.sum())
        # New LiDAR walls contradicting a strongly FREE LaMa prediction may
        # invalidate cached ray visibility even when all routes remain safe.
        unexpected_wall = int(np.count_nonzero((old_obs == 1) & (new == 2) & (self.mean < .25)))
        self.obs = new.copy()
        remaining = self.mission & (new == 1)
        if unexpected_wall >= 32:
            return {"action": "REPLAN_REQUIRED", "reason": "PREDICTION_CONTRADICTED_BY_OBSERVED_WALL",
                    "unexpected_wall_cells": unexpected_wall, "new_observed_cells": changed}
        if semantic is not None:
            newer_room = room_connected(new, np.asarray(semantic, np.uint8), robot)
            room_delta = int(np.count_nonzero(self.room ^ newer_room))
            if not newer_room[robot] or room_delta / max(1, int(self.room.sum())) >= self.cfg.large_structural_change_fraction:
                return {"action": "REPLAN_REQUIRED", "reason": "MAJOR_ROOM_SEMANTIC_CHANGE",
                        "room_changed_cells": room_delta, "new_observed_cells": changed}
        if not self.plan_order:
            return {"action": "REPLAN_REQUIRED", "reason": "NO_COMMITTED_ROUTE",
                    "new_observed_cells": changed}
        legal = self.legal if native_walkable is None else (
            np.asarray(native_walkable, bool) & (new == 0) & self.room)
        if not legal[robot]:
            return {"action": "REPLAN_REQUIRED", "reason": "ROBOT_NO_LONGER_IN_FREE_SCOPE"}
        _, labels = cv2.connectedComponents(legal.astype(np.uint8), connectivity=4)
        reachable = labels == labels[robot]
        if any(not reachable[p] for p in self.plan_order):
            return {"action": "REPLAN_REQUIRED", "reason": "GOAL_OR_ROUTE_INVALIDATED",
                    "new_observed_cells": changed}
        # Previously planned paths must not cross an observed new wall.
        remaining_route = self.last_decision["planned_grid_route_core_rc"] if self.last_decision else []
        if any(new[tuple(p)] != 0 for p in remaining_route):
            return {"action": "REPLAN_REQUIRED", "reason": "COMMITTED_PATH_BLOCKED",
                    "new_observed_cells": changed}
        if changed >= self.cfg.heavy_new_cells and wall_corrections.any():
            return {"action": "REPLAN_REQUIRED", "reason": "MAJOR_OBSERVED_STRUCTURE_CHANGE",
                    "new_observed_cells": changed}
        # Gain of each remaining goal after existing LiDAR observations.
        # Prune from the END to avoid skipping the currently executed route leg.
        kept = list(self.plan_order)
        pruned = []
        if len(kept) > 1:
            full = self._route_order(kept)
            while len(kept) > 1:
                shorter = self._route_order(kept[:-1])
                if shorter is None or full is None:
                    break
                all_remaining = self._weighted(full["cover"], remaining)
                short_remaining = self._weighted(shorter["cover"], remaining)
                if all_remaining-short_remaining >= self.cfg.min_keep_goal_weight:
                    break
                pruned.insert(0, list(kept.pop()))
                full = shorter
        self.plan_order = kept
        return {"action": "PRUNE_TAIL" if pruned else "KEEP",
                "remaining_goals_rc": [list(p) for p in kept],
                "removed_goals_rc": pruned,
                "new_observed_cells": changed,
                "remaining_mission_unknown": int(remaining.sum()),
                "fast_update_wall_s": round(time.perf_counter()-began, 4),
                "lama_inference_called": False,
                "no_gt": True}

    def refine_remaining(self, current_robot, native_walkable=None):
        """Non-blocking-capable work unit: revise unexecuted route without LaMa.

        Call from a low-priority worker while robot navigates. The FIRST goal
        is held stable, so a new plan cannot abruptly redirect the robot.
        Output is an ADVISORY; native A* must validate it before adoption.
        """
        began = time.perf_counter()
        robot = tuple(map(int, current_robot))
        if self.obs[robot] != 0 or not self.room[robot]:
            return {"action": "REPLAN_REQUIRED", "reason": "ROBOT_OUTSIDE_SAFE_ROOM"}
        legal = self.legal if native_walkable is None else (
            np.asarray(native_walkable, bool) & (self.obs == 0) & self.room)
        if not legal[robot]:
            return {"action": "REPLAN_REQUIRED", "reason": "ROBOT_NOT_NATIVE_REACHABLE"}
        _, labels = cv2.connectedComponents(legal.astype(np.uint8), connectivity=4)
        legal = labels == labels[robot]
        if any(not legal[p] for p in self.plan_order):
            return {"action": "REPLAN_REQUIRED", "reason": "PENDING_GOAL_NO_LONGER_REACHABLE"}
        if not np.array_equal(legal, self.legal):
            self.legal = legal
            self.paths = _KnownFreePaths(legal)
            self.cache_cover.clear()
        pending = list(self.plan_order)
        if not pending:
            return {"action": "DEFER", "reason": "NO_GOALS"}
        remain = self.mission & (self.obs == 1)
        baseline = self._route_order(pending, start=robot)
        if baseline is None:
            return {"action": "REPLAN_REQUIRED", "reason": "PENDING_CHAIN_INVALID"}
        base_gain = self._weighted(baseline["cover"], remain)
        best = (pending, baseline, base_gain)
        # Fix pending[0]. Permute later goals by 2-opt, bounded and simple.
        for i in range(1, len(pending)-1):
            for j in range(i+1, len(pending)):
                test = pending[:i] + list(reversed(pending[i:j+1])) + pending[j+1:]
                trial = self._route_order(test, start=robot)
                if trial is None:
                    continue
                gain = self._weighted(trial["cover"], remain)
                if gain >= .98*base_gain and trial["distance_m"] < best[1]["distance_m"]-.099:
                    best = (test, trial, gain)
        # Drop a non-current goal if the lost visibility is negligible.
        test_order = best[0]
        for k in range(len(test_order)-1, 0, -1):
            reduced = test_order[:k] + test_order[k+1:]
            trial = self._route_order(reduced, start=robot)
            if trial is None:
                continue
            loss = best[2]-self._weighted(trial["cover"], remain)
            if loss <= self.cfg.min_keep_goal_weight and trial["distance_m"] < best[1]["distance_m"]-.099:
                best = (reduced, trial, self._weighted(trial["cover"], remain))
                test_order = reduced
        changed = best[0] != pending
        self.plan_order = list(best[0])
        return {"action": "PROPOSE_REVISED_ROUTE" if changed else "KEEP",
                "goal_sequence_core_rc": [list(p) for p in best[0]],
                "remaining_route_m": round(best[1]["distance_m"], 3),
                "projected_remaining_weighted_gain": round(best[2], 2),
                "last_goal_kept_safe": bool(best[0] and self.legal[best[0][-1]]),
                "first_goal_unchanged": bool(best[0] and best[0][0] == pending[0]),
                "native_commit_required": True,
                "LaMa_inference_called": False,
                "refine_wall_s": round(time.perf_counter()-began, 4)}