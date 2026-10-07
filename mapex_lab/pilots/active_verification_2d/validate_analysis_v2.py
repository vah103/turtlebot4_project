"""Integrity and query-domain checks for supplementary V2 evaluation files."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from .core import clearance_free, reachable_component, array_hash
from .predictor import file_hash
from .run import write_json

BASE=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=BASE/"results/pilot_v2")
    args=parser.parse_args();root=args.output
    provenance=json.loads((root/"analysis_provenance.json").read_text())
    checks=[]
    def check(name,ok):checks.append(dict(name=name,passed=bool(ok)))
    check("analysis code hash",file_hash(BASE/"analyze_v2.py")==provenance["analysis_code_sha256"])
    check("sealed experiment",file_hash(root/"seal.json")==provenance["seal_sha256"])
    for name,sha in provenance["output_sha256"].items():check("output hash "+name,file_hash(root/name)==sha)
    queries=json.loads((root/"navigation_queries.json").read_text())["queries"]
    check("12 unique query sets",len(queries)==12 and len({(q["phase"],q["case"]) for q in queries})==12)
    for q in queries:
        key=q["phase"]+"/"+q["case"];layout=q["case"].split("__")[0]
        with np.load(BASE/"assets"/(layout+".npz")) as z:truth=z["occupied"].copy();domain=z["domain"].copy()
        physical=clearance_free(~truth,1.5)
        actual=reachable_component(physical,tuple(q["start"]))
        check(key+" physical hash",array_hash(physical)==q["truth_physical_hash"])
        goals=[tuple(p) for p in q["goals"]]
        check(key+" 40 unique reachable domain goals",len(goals)==40 and len(set(goals))==40 and all(actual[p] and domain[p] and p!=tuple(q["start"]) for p in goals))
    with (root/"navigation_audit.csv").open() as f:rows=list(csv.DictReader(f))
    check("60 case-method navigation rows",len(rows)==60 and len({(r["phase"],r["case"],r["method"]) for r in rows})==60)
    valid=True
    for r in rows:
        n,found,safe,unsafe=[int(r[k]) for k in ("query_count","plans_found","safe_plans","unsafe_plans")]
        valid &= n==40 and found==safe+unsafe and 0<=safe<=found<=n and 0<=unsafe<=found
        valid &= abs(float(r["safe_plan_fraction"])-safe/n)<1e-10
        valid &= abs(float(r["unsafe_plan_fraction"])-unsafe/max(1,found))<1e-10
    check("navigation counts and denominators",valid)
    summary=json.loads((root/"diagnostic_summary.json").read_text())
    for phase in ("development","confirmation"):
        for method,values in summary[phase]["navigation"].items():
            group=[r for r in rows if r["phase"]==phase and r["method"]==method]
            check(phase+"/"+method+" navigation aggregate",len(group)==6 and all(abs(np.mean([float(r[key]) for r in group])-values["mean_"+key])<1e-10 for key in ("safe_plan_fraction","unsafe_plan_fraction")))
    result=dict(status="SELF_VALIDATION_NOT_INDEPENDENT_QA",checks=len(checks),passed=sum(c["passed"] for c in checks),failed=[c for c in checks if not c["passed"]])
    write_json(root/"analysis_validation.json",result);print(json.dumps(result))
    if result["failed"]:raise SystemExit(1)


if __name__=="__main__":main()
