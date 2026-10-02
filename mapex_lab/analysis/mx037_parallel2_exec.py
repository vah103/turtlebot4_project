#!/usr/bin/env python3
"""Two-level parallel scheduler for MX038.

Scientific computation is delegated unchanged to mx037_exec.py.
This file only partitions independent decisions within each fixed run, merges
them deterministically, then delegates ten-run aggregation to
mx037_parallel_exec.py.
"""
from __future__ import annotations
import sys,json,math
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

SCRIPT=Path(__file__).resolve()
REPO=SCRIPT.parents[2]
if str(REPO) not in sys.path:sys.path.insert(0,str(REPO))

from mapex_lab.analysis import mx037_exec as core
from mapex_lab.analysis import mx037_parallel_exec as par
from mapex_lab.analysis.d1 import d1_gate_p as gatep
from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo

CHROOT=core.ANALYSIS/"mx037_chunk_partials"

def process_chunk(run_id, chunk_id, decision_indices):
    outdir=CHROOT/run_id/f"chunk_{chunk_id:02d}";outdir.mkdir(parents=True,exist_ok=True)
    accepted=core.idx(core.read_csv(core.MX026));mx031=core.idx(core.read_csv(core.MX031PRIMARY));gt=gatep.load_structural_gt(core.GT_PATH)
    run=core.EXPS/run_id;decisions=core.read_csv(run/"decisions.csv");trajectory=topo.read_trajectory(run/"trajectory.csv")
    decision_grids=[gatep.load_raw_grid(run/r["raw_map"]) for r in decisions]
    per=[];components=[];accaudit=[];structrows=[];laterrows=[];r004rows=[];shared=[];hard=[]
    for di in decision_indices:
        row=decisions[di-1]
        print(f"MX038 {run_id} chunk {chunk_id} decision {di}/{len(decisions)}",flush=True)
        key=(run_id,int(row["decision_id"]));a=accepted[key];x31=mx031[key]
        p=core.online_proxy(run,row,a,trajectory)
        if p["tvalid"] != core.bv(x31["TopoValid"]) or str(p["treason"]) != str(x31["TopoValid_reason"]):
            hard.append(f"{key}:TopoValid_parity:{p['tvalid']}/{p['treason']} != {x31['TopoValid']}/{x31['TopoValid_reason']}")
        if not p["parity"]:hard.append(f"{key}:accepted_R_parity")
        rawgrid=decision_grids[di-1];se=core.structural_eval(p,rawgrid,gt);le=core.later_eval(p,rawgrid,di,decisions,decision_grids);te=core.r004_truth(run,row,p,trajectory)
        rec={
          "run_id":run_id,"decision_id":int(row["decision_id"]),"decision_index":di,"decision_count":len(decisions),
          "normalized_progress":float(a["normalized_progress"]),"decision_time_s":float(row["time_s"]),
          "OracleStop_4":int(x31["OracleStop_4"]),"OracleRemainingFraction_GT":float(x31["OracleRemainingFraction_GT"]),
          "R_map":float(a["R_MapRemainingFraction"]),"A_map_mean_m2":float(a["R_A_map_mean_m2"]),"KnownFree_map_m2":float(a["R_KnownFree_map_m2"]),
          "H_ANY_mask_m2":p["Hany"],"Delta_ANY_m2":p["delta_any"],"A_ANY_adj_m2":p["Aany"],"R_ANY":p["Rany"],
          "H_TC_mask_m2":p["Htc"],"Delta_TC_m2":p["delta_tc"],"A_TC_adj_m2":p["Atc"],"R_TC":p["Rtc"],
          "TopoValid_mean":int(p["tvalid"]),"Topo_reason":p["treason"],"online_component_count":len(p["comprows"]),
          "critical_component_count":sum(c["critical"] for c in p["comprows"]),
          "max_UnlockedArea_m2":max([c["unlocked_area_m2"] for c in p["comprows"]],default=0) if p["tvalid"] else math.nan,
          "union_UnlockedArea_m2":float(p["unlocked_union"].sum()*p["canvas_meta"][1]**2) if p["tvalid"] else math.nan,
          "unsupported_online_area_m2":p["unsupported_online_area_m2"],"fixed_base_first_fire":int(int(row["decision_id"])==core.BASE_FIRE[run_id]),
          "STOP_BASE_condition":int(float(a["R_MapRemainingFraction"])<=core.R_TAU and p["tvalid"]),
          "STOP_ANY_condition":int(p["tvalid"] and core.finite(p["Rany"]) and p["Rany"]<=core.R_TAU),
          "STOP_TC_condition":int(p["tvalid"] and core.finite(p["Rtc"]) and p["Rtc"]<=core.R_TAU),
          **{k:v for k,v in p.items() if k.startswith("k_free_") or k.startswith("TC_k_") or k.startswith("ANY_k_") or k.endswith("_eq_0_5_count")},
          **{k:v for k,v in te.items() if not k.startswith("_")}
        }
        per.append(rec)
        for c in p["comprows"]:components.append({**c,"run_id":run_id,"decision_id":int(row["decision_id"]),"topology_evaluable":int(p["tvalid"]),"topology_reason":p["treason"]})
        maxadj=0.0
        if p["tvalid"] and p["tc_rt"].any():
            vals=(p["k_rt"][p["tc_rt"]]/3 + __import__("numpy").maximum(0,1-p["k_rt"][p["tc_rt"]]/3))*p["cell_area"];maxadj=float(vals.max()) if vals.size else 0.0
        accaudit.append({"run_id":run_id,"decision_id":int(row["decision_id"]),"A_map_1_recomputed":p["A"][0],"A_map_2_recomputed":p["A"][1],"A_map_3_recomputed":p["A"][2],
          "A_map_mean_recomputed":p["Are"],"A_map_mean_accepted":float(a["R_A_map_mean_m2"]),"KnownFree_recomputed":p["Ka"],"KnownFree_accepted":float(a["R_KnownFree_map_m2"]),
          "R_map_recomputed":p["Rre"],"R_map_accepted":float(a["R_MapRemainingFraction"]),"H_ANY_mask_m2":p["Hany"],"Delta_ANY_m2":p["delta_any"],
          "H_TC_mask_m2":p["Htc"],"Delta_TC_m2":p["delta_tc"],"max_selected_cell_adjusted_contribution_m2":maxadj,"runtime_cell_area_m2":p["cell_area"],
          "parity_pass":int(p["parity"]),"parity_reason":"" if p["parity"] else "ACCEPTED_R_RECONSTRUCTION_MISMATCH",
          **{k:v for k,v in p.items() if k.startswith("k_free_") or k.startswith("TC_k_") or k.startswith("ANY_k_")}})
        structrows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**se});laterrows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**le})
        r004rows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**{k:v for k,v in te.items() if not k.startswith("_")}})
        shared.append({"run_id":run_id,"decision_id":int(row["decision_id"]),"STRUCT_shared_bias_cells":se["STRUCT_target_shared_bias_cells"],
          "LATER_shared_bias_cells":le.get("LATER_target_shared_bias_cells",""),"R004_shared_bias_critical_cells":te.get("R004_shared_bias_critical_cells","")})
    for name,rows in [("per.csv",per),("components.csv",components),("accounting.csv",accaudit),("struct.csv",structrows),("later.csv",laterrows),("r004.csv",r004rows),("shared.csv",shared)]:
        core.write_csv(outdir/name,rows)
    core.write_json(outdir/"hard.json",{"hard":hard})
    return {"chunk":chunk_id,"n":len(per),"hard":len(hard)}

def merge_run(run_id):
    final=par.PARTIAL/run_id;final.mkdir(parents=True,exist_ok=True)
    merged={k:[] for k in ["per","components","accounting","struct","later","r004","shared"]};hard=[]
    chunks=sorted((CHROOT/run_id).glob("chunk_*"))
    for d in chunks:
        for k in merged:merged[k].extend(core.read_csv(d/f"{k}.csv"))
        hard.extend(json.loads((d/"hard.json").read_text())["hard"])
    merged["per"].sort(key=lambda r:int(r["decision_id"]))
    for k in ["accounting","struct","later","r004","shared"]:merged[k].sort(key=lambda r:int(r["decision_id"]))
    core.write_csv(final/"per.csv",merged["per"]);core.write_csv(final/"components.csv",merged["components"]);core.write_csv(final/"accounting.csv",merged["accounting"])
    core.write_csv(final/"struct.csv",merged["struct"]);core.write_csv(final/"later.csv",merged["later"]);core.write_csv(final/"r004.csv",merged["r004"]);core.write_csv(final/"shared.csv",merged["shared"])
    core.write_json(final/"hard.json",{"run_id":run_id,"hard":hard})
    if len(merged["per"])!=len(core.read_csv(core.EXPS/run_id/"decisions.csv")):raise RuntimeError("run merge row count mismatch")

def worker(run_id):
    decisions=core.read_csv(core.EXPS/run_id/"decisions.csv");N=len(decisions);workers=min(4,N)
    chunks=[list(range(i+1,N+1,workers)) for i in range(workers)]
    with ProcessPoolExecutor(max_workers=workers) as ex:
        fs=[ex.submit(process_chunk,run_id,i,ch) for i,ch in enumerate(chunks)]
        for f in as_completed(fs):print(json.dumps(f.result()),flush=True)
    merge_run(run_id)
    print(json.dumps({"run":run_id,"merged":N}),flush=True)

if __name__=="__main__":
    if sys.argv[1]=="worker":worker(sys.argv[2])
    elif sys.argv[1]=="aggregate":par.aggregate()
    else:raise SystemExit(sys.argv[1])
