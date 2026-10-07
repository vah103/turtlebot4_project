"""Post-hoc resource-feasibility ablation; retain the frozen V1 result.

Only verification viewpoints whose entire shortest path fits the remaining
budget are eligible. No truth, fitted threshold, or new model is introduced.
The original 12 baseline branches are reused explicitly, not counted as new
runs. This is development diagnosis, not independent confirmation.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np

from . import run as runner
from .core import observed_traversable, shortest_paths, frontier_representatives
from .policy import choose as original_choose, hypotheses, observation_poses, observable_fraction
from .predictor import RealLamaPredictor, file_hash

BASE = Path(__file__).resolve().parent


def choose_budget_feasible(state, method, numerics, cfg):
    if method != "structural":
        return original_choose(state, method, numerics, cfg)
    traversable = observed_traversable(state.observed, cfg["robot_radius_m"] / state.resolution)
    dist, parent = shortest_paths(traversable, state.pose)
    frontiers = frontier_representatives(state.observed, cfg["frontier_min_size_strict"])
    hs, stats = hypotheses(state, cfg)
    views = observation_poses(state, dist, frontiers, hs, cfg)
    feasible = [p for p in views if dist[p]*state.resolution <= state.remaining_m+1e-8]
    best, records = None, []
    for h in hs:
        for p in feasible:
            visible = observable_fraction(state, p, h, cfg)
            cost = max(state.resolution, dist[p]*state.resolution)
            score = h.impact_m2*visible/cost
            if score <= 0:
                continue
            record = dict(pose=list(p), hypothesis=list(h.centre), impact_m2=h.impact_m2,
                          observable_fraction=visible, path_m=float(cost), score=score,
                          intervention=h.intervention, orientation=h.orientation,
                          agreement=h.ensemble_agreement, variance=h.ensemble_variance_mean)
            records.append(record)
            key = (-score, p, h.centre, h.orientation, h.intervention)
            if best is None or key < best[0]:
                best = key, p, h, record
    if best is None:
        goal, parent, detail = original_choose(state, "mapex", numerics, cfg)
        detail.update(action="EXPLORE_BUDGET_FALLBACK", hypotheses=len(hs),
                      observation_poses=len(views), budget_feasible_views=len(feasible),
                      variant="BUDGET_FEASIBILITY_ONLY", **stats)
        return goal, parent, detail
    _, goal, h, record = best
    records.sort(key=lambda r: (-r["score"], r["pose"], r["hypothesis"]))
    return goal, parent, dict(action="VERIFY", goal=list(goal), score=record["score"],
                              path_m=record["path_m"], hypothesis=dict(centre=list(h.centre),
                              intervention=h.intervention, orientation=h.orientation,
                              impact_m2=h.impact_m2, agreement=h.ensemble_agreement,
                              variance=h.ensemble_variance_mean, patch_cells=h.patch_cells.tolist()),
                              candidate_records=records[:12], hypotheses=len(hs),
                              observation_poses=len(views), budget_feasible_views=len(feasible),
                              raw_frontiers=len(frontiers), variant="BUDGET_FEASIBILITY_ONLY", **stats)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=BASE.parents[2])
    parser.add_argument("--mapex-root", type=Path, default=Path.home()/"MapEx")
    parser.add_argument("--worker-python", type=Path)
    parser.add_argument("--primary", type=Path, default=BASE/"results/pilot_v1")
    parser.add_argument("--output", type=Path, default=BASE/"results/budget_ablation")
    args = parser.parse_args()
    primary_provenance = json.loads((args.primary/"provenance.json").read_text())
    cfg = primary_provenance["config"]
    cfg = dict(cfg, identity="active_verification_2d_budget_feasibility_ablation",
               interpretation="Post-hoc development ablation; only verification goal budget eligibility changes; original baselines reused explicitly.")
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.primary/"branches.csv").open() as f:
        old_rows = list(csv.DictReader(f))
    original_rows = []
    for row in old_rows:
        if row["method"] != "structural":
            row_file = args.primary/"rows"/(row["case"]+"__"+row["method"]+".json")
            original_rows.append(json.loads(row_file.read_text()))
    predictor = RealLamaPredictor(args.repo_root, args.mapex_root, BASE/"cache", args.worker_python)
    try:
        if predictor.identity != hashlib.sha256(json.dumps(primary_provenance["prediction"], sort_keys=True).encode()).hexdigest():
            raise RuntimeError("Ablation must use identical prediction model/preprocessing")
        provenance = dict(status="POST_HOC_DEVELOPMENT_ABLATION", config=cfg,
                          parent_experiment_identity=primary_provenance["experiment_identity"],
                          parent_branch_csv_sha256=file_hash(args.primary/"branches.csv"),
                          variant_code_sha256=file_hash(__file__), frozen_execution_code_sha256=primary_provenance["code_sha256"],
                          prediction_identity=predictor.identity,
                          change="Filter verification viewpoints by shortest_path_m <= remaining_m. Ordinary exploration may still use partial paths.",
                          original_baselines_reused=12, new_structural_branches=6)
        identity = hashlib.sha256(json.dumps(provenance, sort_keys=True).encode()).hexdigest()
        provenance["experiment_identity"] = identity
        path = args.output/"provenance.json"
        if path.exists() and json.loads(path.read_text())["experiment_identity"] != identity:
            raise RuntimeError("Ablation output identity changed")
        runner.write_json(path, provenance)
        numerics = runner.MapExNumerics(args.repo_root/"mapex_lab/scripts/mapex.py", cfg["resolution_m"],
                                        cfg["sensor_range_m"], cfg["prediction_visibility_rays"])
        runner.choose = choose_budget_feasible
        new_rows = []
        for case in sorted({r["case"] for r in old_rows}):
            layout = case.split("__")[0]
            with np.load(BASE/"assets"/(layout+".npz")) as z:
                asset = {k: z[k].copy() for k in z.files}
            initial_path = args.primary/"raw"/case/"initial.npz"
            with np.load(initial_path) as z:
                observed, pose = z["observed"].copy(), tuple(z["pose"])
                initial_pred = tuple(z[k].copy() for k in ("predictions", "mean", "variance"))
            reference = next(r for r in old_rows if r["case"] == case)
            warm = dict(observed=observed, pose=pose, requested_distance_m=float(reference["warm_requested_m"]),
                        actual_distance_m=float(reference["warm_actual_m"]), observed_hash=reference["initial_observed_hash"])
            folder = args.output/"raw"/case
            folder.mkdir(parents=True, exist_ok=True)
            shutil.copy2(initial_path, folder/"initial.npz")
            new_rows.append(runner.run_branch(case, "structural", asset, warm, predictor, numerics,
                                              cfg, initial_pred, identity, args.output))
            runner.write_csv(args.output/"new_structural_branches.csv", new_rows)
        combined = original_rows + new_rows
        with (args.primary/"initial_diagnostics.csv").open() as f:
            diagnostics = list(csv.DictReader(f))
        summary = runner.summarize(combined, [dict(d, hypotheses=int(d["hypotheses"])) for d in diagnostics], cfg)
        summary.update(status="POST_HOC_DEVELOPMENT_ABLATION", new_branches=6, reused_baseline_branches=12,
                       inference=dict(calls=predictor.calls, cache_hits=predictor.cache_hits, inference_s=predictor.inference_s))
        runner.write_json(args.output/"summary.json", summary)
        runner.write_csv(args.output/"branches.csv", combined)
        runner.write_csv(args.output/"paired.csv", summary["paired"])
        print("ABLATION_COMPLETE", json.dumps(summary["comparisons"]), flush=True)
    finally:
        runner.choose = original_choose
        predictor.close()


if __name__ == "__main__":
    main()
