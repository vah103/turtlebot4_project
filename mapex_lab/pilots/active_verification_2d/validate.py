"""Validate saved paired results and recompute endpoints from final arrays."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .core import evaluate, array_hash
from .predictor import file_hash
from .run import write_json

BASE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=BASE/"results/pilot_v1")
    args = parser.parse_args()
    output = args.output
    provenance = json.loads((output/"provenance.json").read_text())
    cfg = provenance["config"]
    with (output/"branches.csv").open() as f:
        rows = list(csv.DictReader(f))
    checks = []
    def check(name, value):
        checks.append({"name": name, "pass": bool(value)})
    check("18 unique case-method branches", len(rows) == 18 and len({(r["case"], r["method"]) for r in rows}) == 18)
    for name, sha in provenance["code_sha256"].items():
        check("frozen execution code "+name, file_hash(BASE/name) == sha)
    check("frozen protocol", file_hash(BASE/"protocol.json") == provenance["config_sha256"])
    for layout, sha in provenance["asset_sha256"].items():
        check("frozen layout "+layout, file_hash(BASE/"assets"/(layout+".npz")) == sha)
    for case in sorted({r["case"] for r in rows}):
        group = [r for r in rows if r["case"] == case]
        check(case+" paired identical initial map", len({r["initial_observed_hash"] for r in group}) == 1)
        initial = np.load(output/"raw"/case/"initial.npz")
        check(case+" initial hash", array_hash(initial["observed"]) == group[0]["initial_observed_hash"])
        layout = case.split("__")[0]
        with np.load(BASE/"assets"/(layout+".npz")) as z:
            asset = {k: z[k].copy() for k in z.files}
        for row in group:
            key = case+"/"+row["method"]
            folder = output/"raw"/case/row["method"]
            with np.load(folder/"final.npz") as z:
                observed, mean = z["observed"], z["mean"]
                endpoint = evaluate(observed, mean, asset["occupied"], asset["domain"], tuple(initial["pose"]),
                                    float(asset["resolution"]), cfg["robot_radius_m"])
                known = observed != .5
                check(key+" observed labels agree with sensor world", np.array_equal(observed[known], asset["occupied"][known].astype(np.float32)))
                check(key+" initial observations preserved", np.array_equal(observed[initial["observed"] != .5], initial["observed"][initial["observed"] != .5]))
                check(key+" completion preserves observations", np.array_equal(mean[known], observed[known]))
            check(key+" endpoint recomputation", all(abs(float(row[k])-v) < 1e-8 for k, v in endpoint.items()))
            actions = json.loads((folder/"actions.json").read_text())
            distance = 0.0
            valid_motion = True
            continuity = tuple(initial["pose"])
            for a in actions:
                path = [tuple(p) for p in a.get("path_taken", [])]
                if path:
                    valid_motion &= path[0] == continuity
                    valid_motion &= all(abs(p[0]-q[0])+abs(p[1]-q[1]) == 1 for p, q in zip(path[:-1], path[1:]))
                    continuity = path[-1]
                    distance += (len(path)-1)*float(asset["resolution"])
            check(key+" cardinal continuous executed path", valid_motion)
            check(key+" executed distance from path", abs(distance-float(row["distance_m"])) < 1e-8)
            check(key+" budget cap", float(row["distance_m"]) <= cfg["branch_budget_m"]+1e-8)
    result = dict(status="SELF_VALIDATION_NOT_INDEPENDENT_QA", checks=len(checks),
                  passed=sum(c["pass"] for c in checks), failed=[c for c in checks if not c["pass"]])
    write_json(output/"validation.json", result)
    print(json.dumps(result))
    if result["failed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
