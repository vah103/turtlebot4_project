"""V2 paired experiment: six development states and six new-layout states.

V1 execution/policy code is not modified. Development baselines are explicitly
reused; every confirmation branch is newly executed after an input/code seal.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform
import shutil

import numpy as np

from . import run as runner
from .core import PolicyInput, MapExNumerics, array_hash
from .policy_v2 import choose_v2
from .predictor import RealLamaPredictor, file_hash
from .risk_model import ErrorRiskModel

BASE = Path(__file__).resolve().parent
EXECUTION_FILES = ["core.py", "policy.py", "predictor.py", "run.py",
                   "policy_v2.py", "risk_model.py", "run_v2.py", "calibrate_v2.py"]


def config():
    return dict(json.loads((BASE/"protocol.json").read_text()),
                **json.loads((BASE/"protocol_v2.json").read_text()))


def summarize(rows, phase, cfg):
    paired = []
    for case in sorted({r["case"] for r in rows}):
        group = {r["method"]: r for r in rows if r["case"] == case}
        proposed = group["structural_v2"]
        for other in ("mapex", "uncertainty", "route_uncertainty", "route_error"):
            baseline = group[other]
            paired.append(dict(case=case, layout=proposed["layout"], baseline=other,
                               delta_mismatch_m2=proposed["reachable_mismatch_m2"]-baseline["reachable_mismatch_m2"],
                               delta_macro_iou=proposed["macro_iou"]-baseline["macro_iou"],
                               delta_distance_m=proposed["distance_m"]-baseline["distance_m"],
                               equal_budget_completed=all(r["termination"] == "DISTANCE_BUDGET" for r in (proposed, baseline))))
    comparisons = {}
    for other in ("mapex", "uncertainty", "route_uncertainty", "route_error"):
        group = [p for p in paired if p["baseline"] == other]
        per_layout = [np.mean([r["delta_mismatch_m2"] for r in group if r["layout"] == layout])
                      for layout in sorted({r["layout"] for r in group})]
        comparisons[other] = dict(cases=len(group), layouts=len(per_layout),
                                  wins=sum(p["delta_mismatch_m2"] < -1e-9 for p in group),
                                  ties=sum(abs(p["delta_mismatch_m2"]) <= 1e-9 for p in group),
                                  losses=sum(p["delta_mismatch_m2"] > 1e-9 for p in group),
                                  layout_macro_delta_mismatch_m2=float(np.mean(per_layout)),
                                  mean_delta_macro_iou=float(np.mean([p["delta_macro_iou"] for p in group])),
                                  equal_budget_pairs=sum(p["equal_budget_completed"] for p in group))
    return dict(status="PRELIMINARY_NEW_LAYOUT_CHECK" if phase == "confirmation" else "DEVELOPMENT_REPLAY",
                phase=phase, branches=len(rows), new_branches=sum(not r.get("reused_from_v1", False) for r in rows),
                reused_baselines=sum(bool(r.get("reused_from_v1", False)) for r in rows),
                cases=len({r["case"] for r in rows}), layouts=len({r["layout"] for r in rows}),
                complete_branches=sum(r["termination"] == "DISTANCE_BUDGET" for r in rows),
                collision_branches=sum(r["collisions"] > 0 for r in rows),
                comparisons=comparisons, paired=paired, limits=cfg["limits"])


def seal_inputs(output, primary, cfg, predictor, numerics):
    old = json.loads((primary/"provenance.json").read_text())
    for name, sha in old["code_sha256"].items():
        if file_hash(BASE/name) != sha:
            raise RuntimeError("Frozen V1 code changed: "+name)
    if predictor.provenance != old["prediction"]:
        raise RuntimeError("V2 changed real LaMa models or preprocessing")
    model_path = output/"error_model.json"
    model = json.loads(model_path.read_text())
    if {s["case"].split("__")[0] for s in model["sources"]} != set(cfg["development_layouts"]):
        raise RuntimeError("Error-model training split mismatch")
    if model["protocol_sha256"] != file_hash(BASE/"protocol_v2.json"):
        raise RuntimeError("Protocol changed after error-model fit")
    payload = dict(status="SEALED_BEFORE_CONFIRMATION_INFERENCE", config=cfg,
                   config_v1_sha256=file_hash(BASE/"protocol.json"), config_v2_sha256=file_hash(BASE/"protocol_v2.json"),
                   execution_code_sha256={name: file_hash(BASE/name) for name in EXECUTION_FILES},
                   asset_sha256={layout: file_hash(BASE/"assets"/(layout+".npz"))
                                 for layout in cfg["development_layouts"]+cfg["confirmation_layouts"]},
                   risk_model_sha256=file_hash(model_path), parent_provenance_sha256=file_hash(primary/"provenance.json"),
                   parent_branch_csv_sha256=file_hash(primary/"branches.csv"), prediction=predictor.provenance,
                   mapex_numerics_sha256=numerics.source_sha256, python=platform.python_version())
    identity = hashlib.sha256(json.dumps(payload, sort_keys=True).encode()).hexdigest()
    payload["experiment_identity"] = identity
    path = output/"seal.json"
    if path.exists() and json.loads(path.read_text()) != payload:
        raise RuntimeError("Sealed inputs changed; use a new experiment")
    runner.write_json(path, payload)
    return identity


def initial_from_v1(primary, case):
    with np.load(primary/"raw"/case/"initial.npz") as z:
        observed, pose = z["observed"].copy(), tuple(z["pose"])
        prediction = tuple(z[k].copy() for k in ("predictions", "mean", "variance"))
    reference = json.loads((primary/"rows"/(case+"__mapex.json")).read_text())
    warm = dict(observed=observed, pose=pose, requested_distance_m=reference["warm_requested_m"],
                actual_distance_m=reference["warm_actual_m"], observed_hash=array_hash(observed), reason="V1_FROZEN_REUSE")
    return warm, prediction


def publish_phase(output, rows, phase, cfg):
    runner.write_csv(output/"branches.csv", rows)
    summary = summarize(rows, phase, cfg)
    runner.write_json(output/"summary.json", summary)
    runner.write_csv(output/"paired.csv", summary["paired"])
    logs = output/"action_logs"
    logs.mkdir(exist_ok=True)
    for f in (output/"raw").glob("*/*/actions.json"):
        shutil.copy2(f, logs/(f.parent.parent.name+"__"+f.parent.name+".json"))
    print("V2_PHASE_COMPLETE", phase, json.dumps({k: summary[k] for k in
          ("new_branches", "reused_baselines", "complete_branches", "collision_branches", "comparisons")}), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=BASE.parents[2])
    parser.add_argument("--mapex-root", type=Path, default=Path.home()/"MapEx")
    parser.add_argument("--worker-python", type=Path)
    parser.add_argument("--primary", type=Path, default=BASE/"results/pilot_v1")
    parser.add_argument("--output", type=Path, default=BASE/"results/pilot_v2")
    parser.add_argument("--phase", choices=["all", "development", "confirmation"], default="all")
    args = parser.parse_args()
    cfg = config()
    model = ErrorRiskModel.load(args.output/"error_model.json")
    predictor = RealLamaPredictor(args.repo_root, args.mapex_root, BASE/"cache", args.worker_python)
    original_choose = runner.choose
    try:
        numerics = MapExNumerics(args.repo_root/"mapex_lab/scripts/mapex.py", cfg["resolution_m"],
                                cfg["sensor_range_m"], cfg["prediction_visibility_rays"])
        identity = seal_inputs(args.output, args.primary, cfg, predictor, numerics)
        runner.choose = lambda state, method, num, parameters: choose_v2(state, method, num, parameters, model)
        for phase in ("development", "confirmation"):
            if args.phase not in ("all", phase):
                continue
            output = args.output/phase
            output.mkdir(parents=True, exist_ok=True)
            runner.write_json(output/"provenance.json", dict(phase=phase, experiment_identity=identity,
                                                            seal_sha256=file_hash(args.output/"seal.json"), config=cfg))
            rows = []
            for layout in cfg[phase+"_layouts"]:
                with np.load(BASE/"assets"/(layout+".npz")) as z:
                    asset = {k: z[k].copy() for k in z.files}
                warms = ([initial_from_v1(args.primary, layout+"__warm"+str(int(d)))
                          for d in cfg["warm_start_distance_m"]] if phase == "development" else
                         [(warm, None) for warm in runner.warm_states(asset, cfg)])
                for warm, initial in warms:
                    case = layout+"__warm"+str(int(warm["requested_distance_m"]))
                    print("V2_WARM", phase, case, warm["actual_distance_m"], warm["reason"], flush=True)
                    if initial is None:
                        initial = predictor.predict(warm["observed"])
                    folder = output/"raw"/case
                    folder.mkdir(parents=True, exist_ok=True)
                    predictions, mean, variance = initial
                    np.savez_compressed(folder/"initial.npz", observed=warm["observed"], pose=warm["pose"],
                                        predictions=predictions, mean=mean, variance=variance)
                    if phase == "development":
                        for method in ("mapex", "uncertainty"):
                            reference = json.loads((args.primary/"rows"/(case+"__"+method+".json")).read_text())
                            reference.update(reused_from_v1=True, parent_row_sha256=file_hash(args.primary/"rows"/(case+"__"+method+".json")))
                            rows.append(reference)
                        methods = cfg["route_methods"]
                    else:
                        methods = cfg["confirmation_methods"]
                    for method in methods:
                        rows.append(runner.run_branch(case, method, asset, warm, predictor, numerics, cfg, initial, identity, output))
                        runner.write_csv(output/"branches.csv", rows)
            publish_phase(output, rows, phase, cfg)
    finally:
        runner.choose = original_choose
        predictor.close()


if __name__ == "__main__":
    main()
