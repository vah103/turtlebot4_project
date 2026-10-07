"""Evaluation-only one-action oracle, with remaining predictions held fixed.

This enumerates the existing budget-feasible observation set. It diagnoses
direct sensing/correction headroom; it is neither an online policy nor a
global upper bound, and it does not predict later LaMa changes.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .core import (PolicyInput, observed_traversable, shortest_paths, recover_path,
                   frontier_representatives, evaluate, GridWorld)
from .policy import hypotheses, observation_poses
from .run import write_json, write_csv

BASE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--primary", type=Path, default=BASE/"results/pilot_v1")
    args = parser.parse_args()
    cfg = json.loads((args.primary/"provenance.json").read_text())["config"]
    rows, candidates = [], []
    for folder in sorted((args.primary/"raw").iterdir()):
        initial_path = folder/"initial.npz"
        if not initial_path.exists():
            continue
        case, layout = folder.name, folder.name.split("__")[0]
        with np.load(BASE/"assets"/(layout+".npz")) as z:
            asset = {k: z[k].copy() for k in z.files}
        with np.load(initial_path) as z:
            state = PolicyInput(z["observed"].copy(), z["mean"].copy(), z["variance"].copy(),
                                z["predictions"].copy(), tuple(z["pose"]), .1, cfg["branch_budget_m"])
        traversable = observed_traversable(state.observed, cfg["robot_radius_m"]/.1)
        dist, parent = shortest_paths(traversable, state.pose)
        frontiers = frontier_representatives(state.observed, cfg["frontier_min_size_strict"])
        hs, _ = hypotheses(state, cfg)
        views = observation_poses(state, dist, frontiers, hs, cfg)
        feasible = [p for p in views if dist[p]*.1 <= cfg["branch_budget_m"]+1e-8]
        before = evaluate(state.observed, state.mean, asset["occupied"], asset["domain"], state.pose, .1, cfg["robot_radius_m"])
        evaluation = asset["domain"] | (asset["occupied"] & __import__("scipy").ndimage.binary_dilation(asset["domain"], iterations=3))
        wrong = ((state.mean >= .5) != asset["occupied"]) & (state.observed == .5) & evaluation
        low_u_wrong = wrong & (state.variance <= cfg["low_variance_diagnostic_threshold"])
        local = []
        for goal in feasible:
            world = GridWorld(asset["occupied"], .1, state.pose, cfg["sensor_range_m"], cfg["sensor_rays"], cfg["robot_radius_m"], state.observed)
            path = recover_path(parent, state.pose, goal)
            for index, p in enumerate(path[1:], start=1):
                if not world.step(p):
                    break
                if index % cfg["sense_every_motion_steps"] == 0:
                    world.sense()
            world.sense()
            revealed = (state.observed == .5) & (world.observed != .5)
            after = evaluate(world.observed, state.mean, asset["occupied"], asset["domain"], state.pose, .1, cfg["robot_radius_m"])
            record = dict(case=case, row=goal[0], col=goal[1], distance_m=world.distance_m,
                          collisions=world.collisions, wrong_revealed=int((revealed & wrong).sum()),
                          low_u_wrong_revealed=int((revealed & low_u_wrong).sum()),
                          frozen_prediction_mismatch_gain_m2=before["reachable_mismatch_m2"]-after["reachable_mismatch_m2"],
                          frozen_prediction_macro_iou_gain=after["macro_iou"]-before["macro_iou"])
            local.append(record)
        candidates.extend(local)
        row = dict(case=case, candidate_views=len(views), budget_feasible_views=len(feasible),
                   max_direct_mismatch_gain_m2=max([r["frozen_prediction_mismatch_gain_m2"] for r in local]+[0.0]),
                   max_wrong_revealed=max([r["wrong_revealed"] for r in local]+[0]),
                   max_low_u_wrong_revealed=max([r["low_u_wrong_revealed"] for r in local]+[0]),
                   oracle_collision_candidates=sum(r["collisions"] > 0 for r in local))
        rows.append(row)
        print("HEADROOM", json.dumps(row), flush=True)
    write_csv(args.primary/"headroom_summary.csv", rows)
    write_csv(args.primary/"headroom_candidates.csv", candidates)
    write_json(args.primary/"headroom_scope.json", dict(
        status="POST_HOC_EVALUATION_ONLY_ONE_ACTION_FROZEN_PREDICTION", cases=len(rows),
        cases_with_positive_direct_structural_headroom=sum(r["max_direct_mismatch_gain_m2"]>1e-9 for r in rows),
        limitation="Enumerated existing budget-feasible views, one action, no LaMa rerun. This is a candidate-set oracle diagnostic, not an online algorithm or an upper bound on full closed-loop performance."))


if __name__ == "__main__":
    main()
