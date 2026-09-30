#!/usr/bin/env python3
from __future__ import annotations
import csv, json, math, hashlib
from pathlib import Path
import numpy as np
from mapex_lab.analysis.d1 import d1_gate_p as gate
from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo

ROOT=Path(__file__).resolve().parents[2]
EXPERIMENTS=ROOT/"experiments"/"mapex"
OUT=ROOT/"analysis"/"mx023_exec_results"
GT_PATH=ROOT/"ground_truth"/"new_room"/"generated"/"new_room_structural_gt_v2.npz"
ORACLE={"mpx_001":21,"mpx_002":18,"mpx_003":21,"mpx_004":16,"mpx_005":16,"mpx_006":20,"mpx_007":18,"mpx_008":19,"mpx_009":18,"mpx_010":19}
FRAG_CASES=[("mpx_005",16),("mpx_005",17),("mpx_007",15),("mpx_009",16)]
BASE_SHA="4899ee95c85640965befeaf98c20f156c0d3181a"

def write_csv(path, rows):
    rows=list(rows); path.parent.mkdir(parents=True,exist_ok=True)
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields: fields.append(k)
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)

def div(a,b):
    return float(a/b) if b else math.nan

def metrics_strict(values, truth_occ):
    values=np.asarray(values,dtype=float); truth_occ=np.asarray(truth_occ,dtype=bool)
    po=values>0.5; pf=~po; to=truth_occ; tf=~to
    OO=int(np.sum(po&to)); FO=int(np.sum(po&tf)); OF=int(np.sum(pf&to)); FF=int(np.sum(pf&tf))
    return dict(
        strict_FreeRecall=div(FF,FF+FO), strict_FreePrecision=div(FF,FF+OF),
        strict_MissedFreeRate=div(FO,FF+FO), strict_FreeIoU=div(FF,FF+FO+OF),
        strict_OccupiedRecall=div(OO,OO+OF), strict_OccupiedPrecision=div(OO,OO+FO),
        strict_FalseOpenRate=div(OF,OO+OF), strict_OccupiedIoU=div(OO,OO+OF+FO)
    )

def boundary_audit():
    gt=gate.load_structural_gt(GT_PATH)
    rows=[]
    for run_id,o in ORACLE.items():
        run=EXPERIMENTS/run_id
        table=sorted(gate._read_csv(run/"decisions.csv"), key=lambda r:int(r["decision_id"]))
        grids=[gate.load_raw_grid(gate._resolve_run_path(run,r["raw_map"])) for r in table]
        N=len(table)
        for idx,row in enumerate(table,1):
            did=int(row["decision_id"]); dp=(did-o)/(N-1)
            if abs(dp)>0.100000000001: continue
            obs=grids[idx-1]
            pred=gate.load_runtime_prediction(gate._resolve_run_path(run,row["mean_map"]),obs,"mean")
            global_half=int(np.count_nonzero(pred==0.5))
            ur,uc=np.nonzero(obs.data<0)
            labels,_,_,_=gate.first_later_observation_targets(obs,ur,uc,idx,table,grids)
            keep=labels>=0
            later_vals=pred[ur[keep],uc[keep]]
            later_truth=labels[keep]>0
            ctx=gate._structural_context(obs,gt)
            struct_vals=gate._structural_prediction_values(pred,ctx)
            struct_truth=ctx["truth_occupied"]
            for domain,vals,truth in [
                ("Q1_STRUCTURAL_GT",struct_vals,struct_truth),
                ("Q1_LATER_OBSERVED",later_vals,later_truth),
            ]:
                half=int(np.count_nonzero(vals==0.5))
                rec=dict(run_id=run_id,decision_id=did,decision_count=N,delta_p=dp,
                         band="ANCHOR" if did==o else ("PRE" if did<o else "POST"),
                         domain=domain,global_exact_p_eq_0_5_count=global_half,
                         scoreable_count=int(vals.size),scoreable_exact_p_eq_0_5_count=half,
                         sensitivity_required=int(half>0))
                if half>0: rec.update(metrics_strict(vals,truth))
                rows.append(rec)
    write_csv(OUT/"MX023_P_BOUNDARY_AUDIT.csv",rows)
    return rows

def false_barrier_case(run_id, decision_id):
    run=EXPERIMENTS/run_id
    final,_,_=topo.base.final_snapshot(run)
    decisions=topo.base.read_csv(run/"decisions.csv")
    _,meta,_=topo.base.load_canvas(run,decisions[0]["canvas_map"])
    shape,resolution,origin_x,origin_y,_=meta
    topo.validate_resolution(resolution)
    trajectory=topo.read_trajectory(run/"trajectory.csv")
    stencil=topo.collision_stencil(topo.RADIUS_M,resolution)
    target=None
    for i,d in enumerate(decisions,1):
        if int(d["decision_id"])==decision_id: target=(i,d); break
    if target is None: raise RuntimeError(f"{run_id} missing decision {decision_id}")
    index,d=target
    obs,pred,support=topo.base.prediction_canvas(run,d,shape)
    known,future,scoreable,domain,final_free,ref_occ,pred_occ,missed=topo.completed_masks(obs,pred,final,support)
    offset,cropped=topo.crop_domain(domain,ref_occ,pred_occ,missed,scoreable,final_free,topo.occupied_clearance(final))
    dom,ro,po,miss,e,ffree,clearance=cropped
    ref_trav=topo.cspace(dom&~ro,dom,stencil); pred_trav=topo.cspace(dom&~po,dom,stencil)
    source=topo.align_source(float(d["time_s"]),trajectory)
    if not source["available"]: raise RuntimeError(f"{run_id}/{decision_id}: source unavailable {source['mode']}")
    sg=topo.world_to_cell(source["x"],source["y"],origin_x,origin_y,resolution)
    sl=(sg[0]-offset[0],sg[1]-offset[1])
    if not (0<=sl[0]<dom.shape[0] and 0<=sl[1]<dom.shape[1] and ref_trav[sl]):
        raise RuntimeError(f"{run_id}/{decision_id}: invalid source in reference C-space")
    ref_reach=topo.reachable(ref_trav,sl)
    labels,_=topo.components(pred_trav)
    inv=topo.component_inventory(labels,ref_reach,offset)
    if len(inv)<2: raise RuntimeError(f"{run_id}/{decision_id}: expected fragmented, components={len(inv)}")
    ref_clear=topo.distance_transform_edt(ref_trav)
    reps=[topo.representative(item["local_cells"],ref_clear) for item in inv[:2]]
    path=topo.shortest_path(ref_trav,reps[0],reps[1])
    if not path: raise RuntimeError(f"{run_id}/{decision_id}: no reference-valid path")
    barrier={"representatives":reps,"path":path}
    svg=OUT/"figures"/f"MX023_{run_id}_decision_{decision_id:03d}_FALSE_BARRIER.svg"
    topo.render_topology(svg,dom,ref_trav,pred_trav,miss,sl,offset,barrier)
    obstructed=[c for c in path if not pred_trav[c]]
    r004_missed=[c for c in path if miss[c]]
    return dict(run_id=run_id,decision_id=decision_id,domain="R004_TOPOLOGY_MISSEDFREE",
                component_count=len(inv),path_cells=len(path),obstructed_path_cells=len(obstructed),
                r004_missed_free_path_cells=len(r004_missed),
                representative_a_row=int(reps[0][0]+offset[0]),representative_a_col=int(reps[0][1]+offset[1]),
                representative_b_row=int(reps[1][0]+offset[0]),representative_b_col=int(reps[1][1]+offset[1]),
                overlay_file=str(svg.relative_to(OUT)))

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    boundary=boundary_audit()
    manifest=[false_barrier_case(*x) for x in FRAG_CASES]
    write_csv(OUT/"MX023_P_FALSE_BARRIER_MANIFEST.csv",manifest)
    summary={
      "schema":"mx023_bounded_execution_support_v1",
      "frozen_technical_base_sha":BASE_SHA,
      "w10_boundary_rows":len(boundary),
      "scoreable_exact_p_eq_0_5_total":sum(int(r["scoreable_exact_p_eq_0_5_count"]) for r in boundary),
      "rows_requiring_boundary_sensitivity":sum(int(r["sensitivity_required"]) for r in boundary),
      "false_barrier_cases":manifest,
      "method_notes":[
        "Q1 boundary audit reuses Gate-P saved mean prediction loader, first-later-observed target semantics, and structural-GT projection.",
        "False-barrier cases reuse accepted R004 footprint/C-space/source/components/representative/shortest-path/render semantics.",
        "No threshold, Oracle anchor, W10 frame, topology radius, or graph semantics changed."
      ]
    }
    (OUT/"MX023_EXECUTION_SUPPORT_SUMMARY.json").write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n",encoding="utf-8")
    print(json.dumps(summary,indent=2,sort_keys=True))

if __name__=="__main__": main()
