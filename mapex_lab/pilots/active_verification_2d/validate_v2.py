"""Recompute V2 artifacts, training/confirmation split and motion invariants."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .core import evaluate, array_hash, observed_traversable, clearance_free
from .predictor import file_hash
from .run import write_json
from .run_v2 import summarize

BASE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=BASE/"results/pilot_v2")
    parser.add_argument("--primary", type=Path, default=BASE/"results/pilot_v1")
    args = parser.parse_args()
    seal = json.loads((args.output/"seal.json").read_text()); cfg = seal["config"]
    checks = []
    def check(name, ok):
        checks.append(dict(name=name, passed=bool(ok)))
    for name, sha in seal["execution_code_sha256"].items():
        check("sealed source "+name, file_hash(BASE/name) == sha)
    for layout, sha in seal["asset_sha256"].items():
        check("sealed asset "+layout, file_hash(BASE/"assets"/(layout+".npz")) == sha)
    check("V1 protocol unchanged", file_hash(BASE/"protocol.json") == seal["config_v1_sha256"])
    check("V2 protocol frozen", file_hash(BASE/"protocol_v2.json") == seal["config_v2_sha256"])
    check("parent results unchanged", file_hash(args.primary/"branches.csv") == seal["parent_branch_csv_sha256"])
    check("risk estimator frozen", file_hash(args.output/"error_model.json") == seal["risk_model_sha256"])
    model = json.loads((args.output/"error_model.json").read_text())
    training = {s["case"].split("__")[0] for s in model["sources"]}
    check("training exclusively development layouts", training == set(cfg["development_layouts"]))
    check("confirmation excluded from training", not training.intersection(cfg["confirmation_layouts"]))
    for source in model["sources"]:
        check("training input hash "+source["case"], file_hash(args.primary/"raw"/source["case"]/"initial.npz") == source["initial_sha256"])
    for phase in ("development", "confirmation"):
        output = args.output/phase
        rows = []
        for case in sorted(p.name for p in (output/"raw").iterdir()):
            methods = cfg["confirmation_methods"]
            for method in methods:
                if phase == "development" and method in {"mapex", "uncertainty"}:
                    path = args.primary/"rows"/(case+"__"+method+".json")
                    row = json.loads(path.read_text()); row.update(reused_from_v1=True, parent_row_sha256=file_hash(path))
                else:
                    row = json.loads((output/"rows"/(case+"__"+method+".json")).read_text())
                rows.append(row)
        expected = 2*len(cfg[phase+"_layouts"])*len(cfg["confirmation_methods"])
        check(phase+" full declared cohort", len(rows) == expected and len({(r["case"], r["method"]) for r in rows}) == expected)
        with (output/"branches.csv").open() as f:
            csv_rows = list(csv.DictReader(f))
        by_key = {(r["case"],r["method"]):r for r in rows}
        csv_ok = len(csv_rows) == len(rows)
        for row in csv_rows:
            reference = by_key.get((row["case"],row["method"]))
            csv_ok &= reference is not None
            if reference is not None:
                csv_ok &= all(abs(float(row[k])-reference[k]) < 1e-9 for k in
                              ("distance_m","reachable_mismatch_m2","macro_iou","collisions"))
        check(phase+" published CSV matches branch records", csv_ok)
        stored = json.loads((output/"summary.json").read_text())
        check(phase+" paired summary recomputation", stored == summarize(rows, phase, cfg))
        check(phase+" phase provenance seal", json.loads((output/"provenance.json").read_text())["seal_sha256"] == file_hash(args.output/"seal.json"))
        for case in sorted({r["case"] for r in rows}):
            group = [r for r in rows if r["case"] == case]
            with np.load(output/"raw"/case/"initial.npz") as z:
                initial = {k: z[k].copy() for k in z.files}
            check(case+" common warm state", all(r["initial_observed_hash"] == array_hash(initial["observed"]) for r in group))
            layout = case.split("__")[0]
            with np.load(BASE/"assets"/(layout+".npz")) as z:
                asset = {k: z[k].copy() for k in z.files}
            res = float(asset["resolution"])
            physical = clearance_free(~asset["occupied"], cfg["robot_radius_m"]/res)
            for row in group:
                key = phase+"/"+case+"/"+row["method"]
                reused = bool(row.get("reused_from_v1", False))
                folder = (args.primary if reused else output)/"raw"/case/row["method"]
                check(key+" experiment identity", reused or row["experiment_identity"] == seal["experiment_identity"])
                with np.load(folder/"final.npz") as z:
                    observed, mean = z["observed"].copy(), z["mean"].copy()
                    known = observed != .5
                    endpoint = evaluate(observed, mean, asset["occupied"], asset["domain"], tuple(initial["pose"]), res, cfg["robot_radius_m"])
                    check(key+" endpoint recomputation", all(abs(row[k]-v)<1e-8 for k,v in endpoint.items()))
                    check(key+" measured labels are physical", np.array_equal(observed[known], asset["occupied"][known].astype(np.float32)))
                    check(key+" initial known labels preserved", np.array_equal(observed[initial["observed"] != .5], initial["observed"][initial["observed"] != .5]))
                    check(key+" prediction preserves observation", np.array_equal(mean[known], observed[known]))
                actions = json.loads((folder/"actions.json").read_text())
                before = initial["observed"].copy(); pose = tuple(initial["pose"])
                distance = 0.; valid_motion = True; measured_paths = True; physical_paths = True; ranks = True; goal_budget = True
                for a in actions:
                    check(key+" action input hash "+str(a["decision"]), array_hash(before) == a["observed_hash"])
                    path = [tuple(p) for p in a.get("path_taken", [])]
                    if path:
                        valid_motion &= path[0] == pose
                        valid_motion &= all(abs(p[0]-q[0])+abs(p[1]-q[1]) == 1 for p,q in zip(path[:-1],path[1:]))
                        measured = observed_traversable(before, cfg["robot_radius_m"]/res)
                        measured_paths &= all(measured[p] for p in path)
                        physical_paths &= all(physical[p] for p in path)
                        distance += (len(path)-1)*res; pose = path[-1]
                        if a.get("selected_route"):
                            ranks &= a["selected_route"] == a["candidate_records"][0]
                            ranks &= abs(a["score"]-max(r["score"] for r in a["candidate_records"])) < 1e-9
                            goal_budget &= a["path_m"] <= cfg["branch_budget_m"]-a["distance_before_m"]+1e-8
                            goal_budget &= bool(a["goal_reached"]) or bool(a["collision"])
                        with np.load(folder/("frame_%03d.npz" % a["decision"])) as z:
                            before = z["observed"].copy()
                check(key+" continuous cardinal motion", valid_motion)
                check(key+" paths use measured free cells", measured_paths)
                check(key+" paths physically footprint free", physical_paths)
                check(key+" distance recomputed from paths", abs(distance-row["distance_m"]) < 1e-8)
                check(key+" budget cap", row["distance_m"] <= cfg["branch_budget_m"]+1e-8)
                check(key+" selected highest logged route score", ranks)
                check(key+" selected route goal achievable in remaining budget", goal_budget)
    result = dict(status="SELF_VALIDATION_NOT_INDEPENDENT_QA", checks=len(checks),
                  passed=sum(c["passed"] for c in checks), failed=[c for c in checks if not c["passed"]])
    write_json(args.output/"validation.json", result)
    print(json.dumps(result))
    if result["failed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
