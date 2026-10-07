"""Run a paired 18-branch pilot with real LaMa on ideal 2D grids.

Run from this folder's parent using ``python -m active_verification_2d.run``.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time

import numpy as np

from .core import (GridWorld, PolicyInput, MapExNumerics, UNKNOWN, array_hash,
                   observed_traversable, shortest_paths, recover_path,
                   frontier_representatives, evaluate)
from .policy import choose, hypotheses
from .predictor import RealLamaPredictor, file_hash

BASE = Path(__file__).resolve().parent


def write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


def write_csv(path, rows):
    if not rows:
        return
    keys = list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def world_from_asset(asset, cfg, observed=None, pose=None):
    return GridWorld(asset["occupied"], float(asset["resolution"]),
                     tuple(asset["start"]) if pose is None else pose,
                     cfg["sensor_range_m"], cfg["sensor_rays"], cfg["robot_radius_m"], observed)


def warm_states(asset, cfg):
    """Prepare states by fixed travelled distance, without prediction/error selection."""
    world = world_from_asset(asset, cfg)
    world.sense()
    out = []
    res = world.resolution
    for target in cfg["warm_start_distance_m"]:
        moves, goals, reason = 0, 0, "DISTANCE_CHECKPOINT"
        while world.distance_m < target - res/2:
            traversable = observed_traversable(world.observed, cfg["robot_radius_m"] / res)
            dist, parent = shortest_paths(traversable, world.pose)
            candidates = [p for p in frontier_representatives(world.observed, cfg["frontier_min_size_strict"]) if dist[p] > 0]
            if not candidates:
                reason = "WARM_START_NO_REACHABLE_FRONTIER"
                break
            goal = min(candidates, key=lambda p: (np.hypot(p[0]-world.pose[0], p[1]-world.pose[1]), p))
            path = recover_path(parent, world.pose, goal)
            goals += 1
            for p in path[1:]:
                if world.distance_m >= target - res/2:
                    break
                if not world.step(p):
                    raise RuntimeError("Warm-start collision: investigate simulator/planning before scientific runs")
                moves += 1
                if moves % cfg["sense_every_motion_steps"] == 0:
                    world.sense()
            newly_observed = world.sense()
            if not newly_observed.any() and len(path) == 1:
                reason = "WARM_START_STAGNATION"
                break
            if goals > 100:
                reason = "WARM_START_GOAL_LIMIT"
                break
        out.append(dict(observed=world.observed.copy(), pose=world.pose,
                        requested_distance_m=target, actual_distance_m=world.distance_m,
                        reason=reason, observed_hash=array_hash(world.observed)))
    return out


def run_branch(case, method, asset, warm, predictor, numerics, cfg, initial_prediction, identity, output):
    folder = output / "raw" / case / method
    summary_path = output / "rows" / (case + "__" + method + ".json")
    if summary_path.exists():
        saved = json.loads(summary_path.read_text())
        if saved.get("experiment_identity") != identity:
            raise RuntimeError("Result identity changed; use a new output folder")
        print("RESUME", case, method, flush=True)
        return saved
    folder.mkdir(parents=True, exist_ok=True)
    world = world_from_asset(asset, cfg, warm["observed"], warm["pose"])
    predictions, mean, variance = tuple(x.copy() for x in initial_prediction)
    initial_eval = evaluate(world.observed, mean, asset["occupied"], asset["domain"],
                            warm["pose"], world.resolution, cfg["robot_radius_m"])
    actions, frames = [], []
    moves = 0
    budget = cfg["branch_budget_m"]
    start = time.perf_counter()
    initial_calls, initial_seconds = predictor.calls, predictor.inference_s
    termination = "DECISION_LIMIT"
    for decision in range(cfg["max_policy_decisions"]):
        if world.distance_m >= budget - world.resolution / 2:
            termination = "DISTANCE_BUDGET"
            break
        state = PolicyInput(world.observed.copy(), mean.copy(), variance.copy(), predictions.copy(),
                            world.pose, world.resolution, budget-world.distance_m)
        before_hash = array_hash(state.observed)
        t = time.perf_counter()
        goal, parent, detail = choose(state, method, numerics, cfg)
        detail.update(case=case, method=method, decision=decision, start_pose=list(world.pose),
                      distance_before_m=world.distance_m, policy_compute_s=time.perf_counter()-t,
                      observed_hash=before_hash)
        if array_hash(state.observed) != before_hash:
            raise AssertionError("Policy mutated observation")
        if goal is None:
            actions.append(detail)
            termination = "NO_REACHABLE_CANDIDATE"
            break
        path = recover_path(parent, world.pose, goal)
        if path is None or len(path) < 2:
            raise AssertionError("Policy selected an unreachable/current goal")
        before = world.observed.copy()
        old_mean, old_variance = mean.copy(), variance.copy()
        path_taken = [world.pose]
        collided = False
        for p in path[1:]:
            if world.distance_m >= budget - world.resolution/2:
                break
            if not world.step(p):
                collided = True
                break
            moves += 1
            path_taken.append(world.pose)
            if moves % cfg["sense_every_motion_steps"] == 0:
                world.sense()
        world.sense()
        revealed = (before == UNKNOWN) & (world.observed != UNKNOWN)
        # Truth appears only here, after action/execution, for offline scoring.
        wrong = revealed & ((old_mean >= 0.5) != asset["occupied"])
        low_u_wrong = wrong & (old_variance <= cfg["low_variance_diagnostic_threshold"])
        detail.update(distance_after_m=world.distance_m, path_taken=[list(p) for p in path_taken],
                      goal_reached=world.pose == goal, collision=collided,
                      revealed_cells=int(revealed.sum()), revealed_wrong_cells=int(wrong.sum()),
                      revealed_low_u_wrong_cells=int(low_u_wrong.sum()))
        if detail.get("hypothesis"):
            patch = np.asarray(detail["hypothesis"]["patch_cells"], dtype=int)
            rr, cc = patch[:, 0], patch[:, 1]
            detail["hypothesis"].update(patch_revealed_cells=int(revealed[rr, cc].sum()),
                                        patch_wrong_revealed_cells=int(wrong[rr, cc].sum()))
        actions.append(detail)
        predictions, mean, variance = predictor.predict(world.observed)
        metrics = evaluate(world.observed, mean, asset["occupied"], asset["domain"], warm["pose"],
                           world.resolution, cfg["robot_radius_m"])
        frames.append(dict(decision=decision, distance_m=world.distance_m, **metrics))
        np.savez_compressed(folder / ("frame_%03d.npz" % decision), observed=world.observed,
                            mean=mean, variance=variance, pose=world.pose)
        if collided:
            termination = "PHYSICAL_COLLISION"
            break
    else:
        if world.distance_m >= budget-world.resolution/2:
            termination = "DISTANCE_BUDGET"
    final = evaluate(world.observed, mean, asset["occupied"], asset["domain"], warm["pose"],
                     world.resolution, cfg["robot_radius_m"])
    if world.distance_m > budget + 1e-8:
        raise AssertionError("Motion budget exceeded")
    np.savez_compressed(folder / "final.npz", observed=world.observed, mean=mean,
                        variance=variance, pose=world.pose)
    write_json(folder / "actions.json", actions)
    write_csv(folder / "trajectory_metrics.csv", frames)
    record = dict(case=case, method=method, layout=case.split("__")[0],
                  experiment_identity=identity, warm_requested_m=warm["requested_distance_m"],
                  warm_actual_m=warm["actual_distance_m"], initial_observed_hash=warm["observed_hash"],
                  distance_m=world.distance_m, budget_m=budget, termination=termination,
                  collisions=world.collisions, policy_decisions=len(actions),
                  verify_decisions=sum(x["action"] == "VERIFY" for x in actions),
                  verify_patch_observed=sum(x.get("hypothesis", {}).get("patch_revealed_cells", 0) for x in actions),
                  verify_patch_wrong_observed=sum(x.get("hypothesis", {}).get("patch_wrong_revealed_cells", 0) for x in actions),
                  revealed_wrong_cells=sum(x.get("revealed_wrong_cells", 0) for x in actions),
                  revealed_low_u_wrong_cells=sum(x.get("revealed_low_u_wrong_cells", 0) for x in actions),
                  inference_calls=predictor.calls-initial_calls,
                  inference_s=predictor.inference_s-initial_seconds,
                  policy_compute_s=sum(x["policy_compute_s"] for x in actions),
                  wall_time_s=time.perf_counter()-start,
                  **final)
    for key, value in initial_eval.items():
        record["initial_"+key] = value
        record["delta_"+key] = final[key]-value
    write_json(summary_path, record)
    print("RESULT", json.dumps({k: record[k] for k in ["case", "method", "distance_m", "termination",
                                                     "reachable_mismatch_m2", "macro_iou", "verify_decisions"]}), flush=True)
    return record


def summarize(rows, diagnostics, cfg):
    paired = []
    for case in sorted({r["case"] for r in rows}):
        by_method = {r["method"]: r for r in rows if r["case"] == case}
        structural = by_method.get("structural")
        if not structural:
            continue
        for other in ("mapex", "uncertainty"):
            baseline = by_method[other]
            paired.append(dict(case=case, layout=structural["layout"], baseline=other,
                               equal_budget_completed=all(x["termination"] == "DISTANCE_BUDGET" for x in [structural, baseline]),
                               delta_mismatch_m2=structural["reachable_mismatch_m2"]-baseline["reachable_mismatch_m2"],
                               delta_macro_iou=structural["macro_iou"]-baseline["macro_iou"],
                               delta_distance_m=structural["distance_m"]-baseline["distance_m"]))
    comparisons = {}
    for other in ("mapex", "uncertainty"):
        comparisons[other] = {}
        for scope in ("all_executed", "equal_budget_only"):
            group = [p for p in paired if p["baseline"] == other and
                     (scope == "all_executed" or p["equal_budget_completed"])]
            if not group:
                comparisons[other][scope] = {"n_cases": 0}
                continue
            macro = []
            for layout in sorted({p["layout"] for p in group}):
                values = [p["delta_mismatch_m2"] for p in group if p["layout"] == layout]
                macro.append(float(np.mean(values)))
            comparisons[other][scope] = dict(n_cases=len(group), n_layouts=len(macro),
                                            structural_mismatch_wins=sum(p["delta_mismatch_m2"] < -1e-9 for p in group),
                                            structural_iou_wins=sum(p["delta_macro_iou"] > 1e-9 for p in group),
                                            layout_macro_mean_delta_mismatch_m2=float(np.mean(macro)),
                                            mean_delta_macro_iou=float(np.mean([p["delta_macro_iou"] for p in group])))
    return dict(status="EXPLORATORY_PILOT_NOT_CONFIRMATORY", branches=len(rows),
                cases=len({r["case"] for r in rows}), layouts=len({r["layout"] for r in rows}),
                budget_complete_branches=sum(r["termination"] == "DISTANCE_BUDGET" for r in rows),
                collision_branches=sum(r["collisions"] > 0 for r in rows),
                verify_actions=sum(r["verify_decisions"] for r in rows),
                cases_with_initial_structural_hypotheses=sum(d["hypotheses"] > 0 for d in diagnostics),
                comparisons=comparisons, paired=paired,
                limits=["Development pilot on three layouts; paired warm states are not independent environments.",
                        "No STOP algorithm or threshold is fitted or validated.",
                        "Ideal localization, static 2D geometry, first-hit noiseless sensing; no robot/Gazebo evidence.",
                        "Gate sensitivity is not a calibrated error probability or a guarantee of map correctness."])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=BASE.parents[2])
    parser.add_argument("--mapex-root", type=Path, default=Path.home()/"MapEx")
    parser.add_argument("--worker-python", type=Path)
    parser.add_argument("--output", type=Path, default=BASE/"results/pilot_v1")
    parser.add_argument("--config", type=Path, default=BASE/"protocol.json")
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    print("START", str(output), flush=True)
    predictor = RealLamaPredictor(args.repo_root, args.mapex_root, BASE/"cache", args.worker_python)
    try:
        source = args.repo_root / "mapex_lab/scripts/mapex.py"
        numerics = MapExNumerics(source, cfg["resolution_m"], cfg["sensor_range_m"], cfg["prediction_visibility_rays"])
        code_hashes = {p.name: file_hash(p) for p in [BASE/"core.py", BASE/"policy.py", BASE/"predictor.py", BASE/"run.py"]}
        source_hashes = {layout: file_hash(BASE/"assets"/(layout+".npz")) for layout in cfg["layouts"]}
        provenance = dict(config=cfg, config_sha256=file_hash(args.config), code_sha256=code_hashes,
                          asset_sha256=source_hashes, prediction=predictor.provenance,
                          mapex_numerics_sha256=numerics.source_sha256,
                          python=platform.python_version(), numpy=np.__version__,
                          base_technical_commit="88fb673b7a19d2973cfb1c78b4d0a083fdf66b50")
        identity = hashlib.sha256(json.dumps(provenance, sort_keys=True).encode()).hexdigest()
        provenance["experiment_identity"] = identity
        prior = output / "provenance.json"
        if prior.exists() and json.loads(prior.read_text())["experiment_identity"] != identity:
            raise RuntimeError("Existing output belongs to a different experiment")
        write_json(prior, provenance)
        rows, diagnostics = [], []
        for layout in cfg["layouts"]:
            with np.load(BASE/"assets"/(layout+".npz"), allow_pickle=False) as z:
                asset = {k: z[k].copy() for k in z.files}
            for index, warm in enumerate(warm_states(asset, cfg)):
                case = layout + "__warm" + str(int(warm["requested_distance_m"]))
                print("WARM", case, warm["actual_distance_m"], warm["reason"], flush=True)
                case_folder = output / "raw" / case
                case_folder.mkdir(parents=True, exist_ok=True)
                pred = predictor.predict(warm["observed"])
                predictions, mean, variance = pred
                np.savez_compressed(case_folder/"initial.npz", observed=warm["observed"], mean=mean,
                                    variance=variance, predictions=predictions, pose=warm["pose"])
                state = PolicyInput(warm["observed"].copy(), mean.copy(), variance.copy(), predictions.copy(),
                                    warm["pose"], float(asset["resolution"]), cfg["branch_budget_m"])
                hs, stats = hypotheses(state, cfg)
                initial_metrics = evaluate(state.observed, state.mean, asset["occupied"], asset["domain"],
                                           warm["pose"], state.resolution, cfg["robot_radius_m"])
                unknown = state.observed == UNKNOWN
                wrong = unknown & ((state.mean >= 0.5) != asset["occupied"]) & asset["domain"]
                diag = dict(case=case, layout=layout, hypotheses=len(hs),
                            unknown_wrong_cells=int(wrong.sum()),
                            low_variance_wrong_cells=int((wrong & (state.variance <= cfg["low_variance_diagnostic_threshold"])).sum()),
                            max_hypothesis_impact_m2=max([h.impact_m2 for h in hs] + [0.0]),
                            warm_reason=warm["reason"], **stats, **initial_metrics)
                diagnostics.append(diag)
                write_csv(output/"initial_diagnostics.csv", diagnostics)
                print("DIAGNOSTIC", json.dumps(diag), flush=True)
                for method in cfg["methods"]:
                    rows.append(run_branch(case, method, asset, warm, predictor, numerics, cfg, pred, identity, output))
                    write_csv(output/"branches.csv", rows)
        summary = summarize(rows, diagnostics, cfg)
        summary["inference"] = dict(calls=predictor.calls, cache_hits=predictor.cache_hits,
                                     inference_s=predictor.inference_s)
        write_json(output/"summary.json", summary)
        write_csv(output/"paired.csv", summary["paired"])
        print("COMPLETE", json.dumps({k: summary[k] for k in ["branches", "cases", "layouts", "budget_complete_branches", "collision_branches", "verify_actions", "comparisons"]}), flush=True)
    finally:
        predictor.close()


if __name__ == "__main__":
    main()
