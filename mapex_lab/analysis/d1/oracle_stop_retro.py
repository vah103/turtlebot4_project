#!/usr/bin/env python3
"""Frozen H080 retrospective structural-GT oracle primitives."""
from __future__ import annotations

import hashlib, importlib.util, json, math, sys
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from d1_gate_p import _structural_context, load_raw_grid, load_structural_gt

RUNS = tuple(f"mpx_{i:03d}" for i in range(1, 11))
EXPECTED_COUNTS = dict(zip(RUNS, (35,37,41,36,34,36,35,35,36,40)))
TOLERANCES = (1,5,10)
SPAWN = (1202,512)
RADIUS_M = 0.189
GT_RESOLUTION = 0.05
GT_BLOB = "a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a"
GT_SUMMARY_BLOB = "81a39bfd9484a6704a3a5f5f1893ae5699830866"
GATE_P_BLOB = "c5bf4e3e6f45c81afef135166fc96e088719658e"
SHARED_BLOB = "3e80124d545ea37b2791594dd775a7d953368bea"
R004_REFERENCE = "61b91640ca1d4fd608f716e152a91573ec072b5a"


def sha256_file(path: Path) -> str:
    h=hashlib.sha256()
    with path.open("rb") as s:
        for block in iter(lambda:s.read(1<<20),b""): h.update(block)
    return h.hexdigest()


def load_r004_helper(reference: Path):
    path=reference/"mapex_lab/analysis/r004/evaluate_topology_traversability.py"
    spec=importlib.util.spec_from_file_location("accepted_r004_topology",path)
    if spec is None or spec.loader is None: raise RuntimeError(f"cannot load {path}")
    module=importlib.util.module_from_spec(spec)
    helper_dir=str(path.parent); sys.path.insert(0,helper_dir)
    try: spec.loader.exec_module(module)
    finally: sys.path.remove(helper_dir)
    return module,path


def validate_universe(frame: pd.DataFrame) -> None:
    if len(frame)!=365 or frame[["run_id","decision_id"]].duplicated().any(): raise ValueError("expected 365 unique rows")
    if frame.groupby("run_id").size().to_dict()!=EXPECTED_COUNTS: raise ValueError("run universe mismatch")


def raw_input_fingerprint(frame:pd.DataFrame,data_root:Path,verify_recorded:bool=True)->str:
    h=hashlib.sha256()
    for row in frame.sort_values(["run_id","decision_id"]).to_dict("records"):
        path=data_root/row["run_id"]/str(row["raw_map"]); digest=sha256_file(path)
        if verify_recorded and str(row.get("raw_map_sha256",digest))!=digest: raise ValueError(f"raw map hash mismatch: {path}")
        h.update(f"{row['run_id']}:{row['decision_id']}:{digest}\n".encode())
    return h.hexdigest()


def structural_universe(gt, topo) -> np.ndarray:
    if not math.isclose(gt.resolution,GT_RESOLUTION,abs_tol=1e-12): raise ValueError("GT resolution mismatch")
    free=(gt.data==0)&gt.evaluation_mask
    safe=topo.cspace(free,gt.evaluation_mask,topo.collision_stencil(RADIUS_M,GT_RESOLUTION))
    if not safe[SPAWN]: raise ValueError("canonical spawn is not footprint-safe")
    universe=topo.reachable(safe,SPAWN)
    if not universe.any(): raise ValueError("empty structural universe")
    return universe


def project_known(raw,gt) -> tuple[np.ndarray,dict[str,Any]]:
    context=_structural_context(raw,gt)
    known_hi=np.repeat(np.repeat(raw.data>=0,context["ratio"],axis=0),context["ratio"],axis=1)
    projected=np.zeros(gt.shape,dtype=bool)
    projected[context["gt_slice"]]=known_hi[context["src_slice"]]&gt.evaluation_mask[context["gt_slice"]]
    return projected,context


def persistent_oracle(values:list[int|None], tolerance:int, denominator:int) -> dict[str,Any]:
    evaluable=[v is not None for v in values]
    naive=next((i for i,v in enumerate(values) if v is not None and 100*v<=tolerance*denominator),None)
    selected=None
    for i,v in enumerate(values):
        if v is None or not all(evaluable[i:]): continue
        if all(100*int(x)<=tolerance*denominator for x in values[i:] if x is not None): selected=i; break
    finite=[(i,int(v)) for i,v in enumerate(values) if v is not None]
    best=min((v for _,v in finite),default=None)
    best_i=next((i for i,v in finite if v==best),None)
    status="FOUND" if selected is not None else ("INSUFFICIENT_TRUTH_SUPPORT" if not all(evaluable) else "NO_ACCEPTABLE_ORACLE_STOP")
    return {"status":status,"index":selected,"naive_index":naive,"best_index":best_i,"best_remaining":best,
            "persistence_changed":naive!=selected}


def monotonicity(values:list[int|None]) -> dict[str,Any]:
    changes=[]
    for i,(a,b) in enumerate(zip(values,values[1:])):
        if a is not None and b is not None and b>a: changes.append((i+1,b-a))
    return {"upward_count":len(changes),"max_upward_cells":max((v for _,v in changes),default=0),
            "affected_next_indices":[i for i,_ in changes]}


def score_decisions(shared:pd.DataFrame,data_root:Path,gt,universe:np.ndarray) -> pd.DataFrame:
    n_gt=int(universe.sum()); rows=[]
    for record in shared.to_dict("records"):
        row=dict(record); run=data_root/record["run_id"]
        try:
            raw_path=run/str(record["raw_map"])
            raw=load_raw_grid(raw_path)
            known,ctx=project_known(raw,gt)
            remaining=int(np.count_nonzero(universe&~known))
            row.update({"N_GT":n_gt,"N_remaining":remaining,"A_true_remaining_m2":remaining*GT_RESOLUTION**2,
                        "OracleRemainingFraction_GT":remaining/n_gt,"oracle_truth_evaluable":True,"oracle_truth_reason":"",
                        "runtime_to_gt_ratio":ctx["ratio"],"origin_rounding_residual_x_m":ctx["origin_rounding_residual_x_m"],
                        "origin_rounding_residual_y_m":ctx["origin_rounding_residual_y_m"],"raw_map_resolved_path":str(raw_path)})
        except Exception as exc:
            row.update({"N_GT":n_gt,"N_remaining":pd.NA,"A_true_remaining_m2":np.nan,
                        "OracleRemainingFraction_GT":np.nan,"oracle_truth_evaluable":False,"oracle_truth_reason":f"{type(exc).__name__}:{exc}",
                        "runtime_to_gt_ratio":pd.NA,"origin_rounding_residual_x_m":np.nan,"origin_rounding_residual_y_m":np.nan,
                        "raw_map_resolved_path":""})
        rows.append(row)
    return pd.DataFrame(rows)


def summarize_runs(decisions:pd.DataFrame) -> tuple[pd.DataFrame,dict[str,Any]]:
    run_rows=[]
    for run_id,group in decisions.groupby("run_id",sort=True):
        group=group.sort_values("decision_index"); values=[None if pd.isna(v) else int(v) for v in group.N_remaining]
        base={"run_id":run_id,"decision_count":len(group),"N_GT":int(group.N_GT.iloc[0]),
              "truth_evaluable_rows":int(group.oracle_truth_evaluable.sum()),**monotonicity(values)}
        for p in TOLERANCES:
            out=persistent_oracle(values,p,base["N_GT"]); prefix=f"oracle_{p}"
            base[f"{prefix}_status"]=out["status"]; base[f"{prefix}_naive_decision"]=None if out["naive_index"] is None else int(group.iloc[out["naive_index"]].decision_id)
            base[f"{prefix}_decision"]=None if out["index"] is None else int(group.iloc[out["index"]].decision_id)
            base[f"{prefix}_persistence_changed"]=out["persistence_changed"]
            base[f"{prefix}_best_decision"]=None if out["best_index"] is None else int(group.iloc[out["best_index"]].decision_id)
            if out["index"] is not None:
                r=group.iloc[out["index"]]; base[f"{prefix}_progress"]=float(r.decision_progress); base[f"{prefix}_fraction_saved"]=1-float(r.decision_progress)
                base[f"{prefix}_decisions_saved"]=len(group)-out["index"]-1
                base[f"{prefix}_remaining_cells"]=int(r.N_remaining); base[f"{prefix}_remaining_m2"]=float(r.A_true_remaining_m2); base[f"{prefix}_remaining_fraction"]=float(r.OracleRemainingFraction_GT)
            else:
                for suffix in ("progress","fraction_saved","decisions_saved","remaining_cells","remaining_m2","remaining_fraction"): base[f"{prefix}_{suffix}"]=np.nan
        run_rows.append(base)
    runs=pd.DataFrame(run_rows)
    summary={"runs":len(runs),"decision_rows":len(decisions),"N_GT":int(runs.N_GT.iloc[0]),"tolerances":{}}
    for p in TOLERANCES:
        found=runs[runs[f"oracle_{p}_status"]=="FOUND"]
        entry={"found_count":len(found),"found_runs":found.run_id.tolist(),"not_found_runs":runs[runs[f"oracle_{p}_status"]!="FOUND"].run_id.tolist()}
        for field in ("progress","fraction_saved","remaining_m2","remaining_fraction"):
            x=pd.to_numeric(found[f"oracle_{p}_{field}"],errors="coerce").dropna()
            entry[field]={"support":len(x),"mean":float(x.mean()) if len(x) else None,"sample_std":float(x.std(ddof=1)) if len(x)>1 else None,"median":float(x.median()) if len(x) else None}
        summary["tolerances"][str(p)]=entry
    summary["sensitivity_offsets"]={
        "decision_1_minus_5": (pd.to_numeric(runs.oracle_1_decision,errors="coerce")-pd.to_numeric(runs.oracle_5_decision,errors="coerce")).dropna().astype(int).tolist(),
        "decision_10_minus_5": (pd.to_numeric(runs.oracle_10_decision,errors="coerce")-pd.to_numeric(runs.oracle_5_decision,errors="coerce")).dropna().astype(int).tolist(),
    }
    return runs,summary


def signal_around(decisions:pd.DataFrame,runs:pd.DataFrame) -> pd.DataFrame:
    fields=("U_p95","U_mean","U_disagreement","A_mean_m2","RemainingFraction","KnownReachableFree_m2",
            "u_r_union_evaluable","d1_runtime_region_evaluable","d1_source_available","broad_error_evaluable",
            "primary_topology_risk_evaluable","prediction_support_coverage","free_support_coverage","non_evaluable_reasons")
    out=[]
    for rr in runs.to_dict("records"):
        oracle=rr.get("oracle_5_decision")
        if pd.isna(oracle): continue
        group=decisions[decisions.run_id==rr["run_id"]].sort_values("decision_index").reset_index(drop=True)
        pos=int(group.index[group.decision_id==int(oracle)][0])
        for offset in range(-3,4):
            if 0<=pos+offset<len(group):
                row=group.iloc[pos+offset]; item={"run_id":rr["run_id"],"oracle_5_decision":int(oracle),"offset":offset,"decision_id":int(row.decision_id)}
                item.update({f:row.get(f,np.nan) for f in fields}); out.append(item)
    return pd.DataFrame(out)


def json_dump(value,path:Path):
    path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+"\n",encoding="utf-8")
