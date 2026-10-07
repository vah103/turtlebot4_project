"""Evaluation-only error reliability, VERIFY outcomes and navigation audit.

Navigation is a supplementary diagnostic added during this development turn;
it does not replace the frozen reachability-area primary endpoint. Query goals
are selected by a fixed seed from true reachable footprint-free cells, never
from the methods' results. This is static plan checking, not robot navigation.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

from .core import (PolicyInput, clearance_free, shortest_paths, recover_path,
                   UNKNOWN, array_hash)
from .risk_model import ErrorRiskModel
from .calibrate_v2 import metrics
from .run import write_csv, write_json
from .predictor import file_hash

BASE = Path(__file__).resolve().parent
NAVIGATION_SEED = 6100703
NAVIGATION_QUERIES = 40


def make_navigation_queries(physical, domain, start, rng):
    start = tuple(int(v) for v in start)
    true_dist, _ = shortest_paths(physical, start)
    candidates = np.argwhere((true_dist > 0) & domain)
    selected = rng.choice(len(candidates), min(NAVIGATION_QUERIES, len(candidates)), replace=False)
    goals = [tuple(int(v) for v in candidates[i]) for i in selected]
    return dict(start=list(start), goals=[list(p) for p in goals], truth_physical_hash=array_hash(physical)), goals, true_dist


def navigation_scores(free, physical_free, start, goals, true_dist):
    dist, parent = shortest_paths(free, start)
    attempted, safe, lengths = 0, 0, []
    for goal in goals:
        path = recover_path(parent, start, goal)
        if path is None:
            continue
        attempted += 1
        valid = all(physical_free[p] for p in path)
        if valid:
            safe += 1
            lengths.append((len(path)-1)/max(1, true_dist[goal]))
    return dict(query_count=len(goals), plans_found=attempted, safe_plans=safe,
                unsafe_plans=attempted-safe, safe_plan_fraction=safe/max(1,len(goals)),
                unsafe_plan_fraction=(attempted-safe)/max(1,attempted),
                mean_safe_path_stretch=float(np.mean(lengths)) if lengths else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=BASE/"results/pilot_v2")
    parser.add_argument("--primary", type=Path, default=BASE/"results/pilot_v1")
    args = parser.parse_args()
    cfg = json.loads((args.output/"seal.json").read_text())["config"]
    model = ErrorRiskModel.load(args.output/"error_model.json")
    reliability, verify, navigation, queries = [], [], [], []
    for phase in ("development", "confirmation"):
        root = args.output/phase
        rng = np.random.default_rng(NAVIGATION_SEED)
        for folder in sorted((root/"raw").iterdir()):
            case = folder.name; layout = case.split("__")[0]
            with np.load(folder/"initial.npz") as z:
                state = PolicyInput(z["observed"].copy(),z["mean"].copy(),z["variance"].copy(),
                                    z["predictions"].copy(),tuple(z["pose"]),.1,8.)
            with np.load(BASE/"assets"/(layout+".npz")) as z:
                truth, domain = z["occupied"].copy(), z["domain"].copy()
            evaluation = domain | (truth & ndi.binary_dilation(domain, iterations=3))
            mask = (state.observed == UNKNOWN) & evaluation
            labels = ((state.mean >= .5) != truth)[mask].astype(float)
            risk = model.risk(state)[mask]
            scores = metrics(labels, risk, np.ones(len(labels)))
            constant = metrics(labels, np.full(len(labels),model.payload["prevalence"]),np.ones(len(labels)))
            reliability.append(dict(phase=phase,case=case,layout=layout,cells=len(labels),**scores,
                                    constant_prior_brier=constant["brier"],delta_brier=scores["brier"]-constant["brier"]))
            physical = clearance_free(~truth,cfg["robot_radius_m"]/.1)
            query, goals, true_dist = make_navigation_queries(physical,domain,state.pose,rng)
            queries.append(dict(phase=phase,case=case,**query))
            for method in cfg["confirmation_methods"]:
                branch = (args.primary/"raw"/case/method if phase == "development" and method in {"mapex","uncertainty"}
                          else folder/method)
                acts = json.loads((branch/"actions.json").read_text())
                for a in acts:
                    if a["action"] == "VERIFY":
                        h = a["hypothesis"]
                        verify.append(dict(phase=phase,case=case,method=method,decision=a["decision"],
                                           path_m=a["path_m"],goal_reached=a["goal_reached"],
                                           predicted_patch_cells=h.get("predicted_patch_revealed_cells"),
                                           actual_patch_cells=h["patch_revealed_cells"],
                                           actual_patch_wrong_cells=h["patch_wrong_revealed_cells"],
                                           error_weight=h.get("error_weight"),
                                           predicted_counterfactual_correction_m2=h.get("counterfactual_correction_m2")))
                with np.load(branch/"final.npz") as z:
                    complete = np.where(z["observed"] == UNKNOWN,z["mean"],z["observed"]) >= .5
                row = navigation_scores(clearance_free(~complete,cfg["robot_radius_m"]/.1),physical,state.pose,goals,true_dist)
                navigation.append(dict(phase=phase,case=case,layout=layout,method=method,**row))
    write_csv(args.output/"risk_reliability.csv",reliability)
    if verify:write_csv(args.output/"verify_outcomes.csv",verify)
    write_csv(args.output/"navigation_audit.csv",navigation)
    write_json(args.output/"navigation_queries.json",dict(status="SUPPLEMENTARY_DEVELOPMENT_DIAGNOSTIC",seed=NAVIGATION_SEED,
               queries_per_state=NAVIGATION_QUERIES,queries=queries,
               limits="Fixed-seed static plans from warm pose to truth-reachable goals. No execution or replanning. Added as secondary diagnostic during V2 development; does not replace frozen primary endpoint."))
    summary = {}
    for phase in ("development","confirmation"):
        rs=[r for r in reliability if r["phase"]==phase]
        vs=[r for r in verify if r["phase"]==phase]
        summary[phase]=dict(risk_brier_better_cases=sum(r["delta_brier"]<0 for r in rs),risk_cases=len(rs),
                           mean_risk_delta_brier=float(np.mean([r["delta_brier"] for r in rs])),
                           verify_actions=len(vs),verify_goals_reached=sum(r["goal_reached"] for r in vs),
                           verify_no_patch_observed=sum(r["actual_patch_cells"]==0 for r in vs),
                           verify_wrong_patch_observed=sum(r["actual_patch_wrong_cells"] for r in vs),
                           navigation={method:dict(mean_safe_plan_fraction=float(np.mean([r["safe_plan_fraction"] for r in navigation if r["phase"]==phase and r["method"]==method])),
                                       mean_unsafe_plan_fraction=float(np.mean([r["unsafe_plan_fraction"] for r in navigation if r["phase"]==phase and r["method"]==method])))
                                       for method in cfg["confirmation_methods"]})
    write_json(args.output/"diagnostic_summary.json",summary)
    write_json(args.output/"analysis_provenance.json",dict(status="SUPPLEMENTARY_EVALUATION_ONLY",
               analysis_code_sha256=file_hash(__file__),seal_sha256=file_hash(args.output/"seal.json"),
               output_sha256={name:file_hash(args.output/name) for name in
                              ("risk_reliability.csv","navigation_audit.csv","navigation_queries.json","diagnostic_summary.json")},
               navigation_seed=NAVIGATION_SEED,navigation_queries_per_state=NAVIGATION_QUERIES,
               interpretation="Model was frozen before confirmation; confirmation labels are used here only for evaluation. Navigation audit is supplementary, not the registered primary endpoint."))
    print("V2_DIAGNOSTICS",json.dumps(summary),flush=True)


if __name__ == "__main__":
    main()
