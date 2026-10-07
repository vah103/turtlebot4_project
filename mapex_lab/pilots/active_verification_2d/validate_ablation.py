"""Recompute the six secondary endpoints and check budget-feasible VERIFY."""
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
    parser.add_argument("--primary", type=Path, default=BASE/"results/pilot_v1")
    parser.add_argument("--output", type=Path, default=BASE/"results/budget_ablation")
    args = parser.parse_args()
    provenance = json.loads((args.output/"provenance.json").read_text())
    cfg = provenance["config"]
    checks = []
    def check(name, value):
        checks.append(dict(name=name, passed=bool(value)))
    check("original baseline result unchanged", file_hash(args.primary/"branches.csv") == provenance["parent_branch_csv_sha256"])
    check("ablation code unchanged", file_hash(BASE/"budget_ablation.py") == provenance["variant_code_sha256"])
    with (args.output/"new_structural_branches.csv").open() as f:
        rows = list(csv.DictReader(f))
    check("six new branches", len(rows) == 6)
    for row in rows:
        case, layout = row["case"], row["layout"]
        initial = np.load(args.output/"raw"/case/"initial.npz")
        check(case+" shared initial observation", array_hash(initial["observed"]) == row["initial_observed_hash"])
        with np.load(BASE/"assets"/(layout+".npz")) as z:
            asset = {k: z[k].copy() for k in z.files}
        final = np.load(args.output/"raw"/case/"structural/final.npz")
        metrics = evaluate(final["observed"], final["mean"], asset["occupied"], asset["domain"], tuple(initial["pose"]),
                           float(asset["resolution"]), cfg["robot_radius_m"])
        check(case+" recomputed endpoints", all(abs(float(row[k])-v) < 1e-8 for k, v in metrics.items()))
        known = final["observed"] != .5
        check(case+" observations agree with sensor world", np.array_equal(final["observed"][known], asset["occupied"][known].astype(np.float32)))
        actions = json.loads((args.output/"raw"/case/"structural/actions.json").read_text())
        check(case+" verification budget eligibility", all(a["path_m"] <= cfg["branch_budget_m"]-a["distance_before_m"]+1e-8 for a in actions if a["action"] == "VERIFY"))
        check(case+" feasible verification goal reached", all(a["goal_reached"] for a in actions if a["action"] == "VERIFY"))
        paths = [[tuple(p) for p in a.get("path_taken", [])] for a in actions]
        travelled = sum(max(0, len(path)-1)*float(asset["resolution"]) for path in paths)
        check(case+" exact travel accounting", abs(travelled-float(row["distance_m"])) < 1e-8)
        check(case+" motion budget", float(row["distance_m"]) <= cfg["branch_budget_m"]+1e-8)
        check(case+" zero collision", int(row["collisions"]) == 0)
    result = dict(status="SELF_VALIDATION_NOT_INDEPENDENT_QA", checks=len(checks),
                  passed=sum(x["passed"] for x in checks), failed=[x for x in checks if not x["passed"]])
    write_json(args.output/"validation.json", result)
    print(json.dumps(result))
    if result["failed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
