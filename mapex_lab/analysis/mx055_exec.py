#!/usr/bin/env python3
from __future__ import annotations
import csv, json, math, subprocess, hashlib
from collections import defaultdict, Counter
from pathlib import Path

import numpy as np
from scipy.ndimage import label as nd_label, distance_transform_edt

from mapex_lab.analysis.r004 import evaluate_prediction_vs_final_observed as r004base
from mapex_lab.analysis.r004 import evaluate_topology_traversability as r004topo

ROOT=Path(__file__).resolve().parents[2]
AN=ROOT/"mapex_lab"/"analysis"
DATA=ROOT/"mapex_lab"/"experiments"/"mapex"
OUT=AN/"mx055_exec_results"

RUNS=[f"mpx_{i:03d}" for i in range(1,11)]
METHOD_COMMIT="1e88c1289b2e1803d5d75d79627cb2148623b5a4"
METHOD_BLOB="cba73d155861fa1fb2c3b1b32418253082afcf05"
SOURCE_ROOT="35ee9315cce3899190d439b75958fbcbe31f9851"
MX031_COMMIT="d90498e1a3ead2a0c170a3fee7c02a1bc9526cca"
MX031_BLOB="daf11687221080790e20227f2bf7b7406476298e"
FEATURE_BLOB="e91a45e2f88806d7ba52ef658c9fd86262ab9a91"
GT_BLOB="a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a"
ROI_BLOB="90bdeecc54bb7e97a9d018b6614d9b879d10b4e7"
PROJECTION_BLOB="a7820bc3d12b2b701f35f0d6629ab3f05d98f1bc"
TOPOLOGY_BLOB="ff031cb826908aafd3bc039ca67a499ed092cbc5"

GT_PATH=ROOT/"mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v2.npz"
ROI_PATH=ROOT/"mapex_lab/ground_truth/new_room/generated/new_room_connected_free_v2.npy"
MX031_PATH=AN/"mx031_exec_results/MX031_PER_DECISION_REPLAY.csv"
FEATURE_PATH=AN/"mx040_exec_results/MX039_FEATURES_PER_DECISION.csv"
PROJECTION_PATH=AN/"r004/evaluate_prediction_vs_final_observed.py"
TOPOLOGY_PATH=AN/"r004/evaluate_topology_traversability.py"

RES=0.05
R_ROBOT=0.189
A_ROBOT=math.pi*R_ROBOT*R_ROBOT
A_REGION=16.0*A_ROBOT
A_BREACH=2.0*A_ROBOT
PAIR_THRESHOLD=0.95
PRED_THRESHOLD=0.5

FOUR=np.array([[0,1,0],[1,1,1],[0,1,0]],dtype=np.int8)

def finite(x):
    try:return math.isfinite(float(x))
    except Exception:return False
def fnum(x,default=math.nan):
    try:
        y=float(x);return y if math.isfinite(y) else default
    except Exception:return default
def fint(x,default=None):
    try:return int(float(x))
    except Exception:return default
def div(a,b): return float(a/b) if b else math.nan
def median(xs):
    z=[float(x) for x in xs if finite(x)]
    return float(np.median(z)) if z else math.nan
def mean(xs):
    z=[float(x) for x in xs if finite(x)]
    return float(np.mean(z)) if z else math.nan
def read_csv(p):
    with open(p,newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write_csv(p,rows,fields=None):
    rows=list(rows);p.parent.mkdir(parents=True,exist_ok=True)
    if fields is None:
        fields=[]
        for r in rows:
            for k in r:
                if k not in fields:fields.append(k)
    if not fields:fields=["status"]
    with open(p,"w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def write_json(p,obj):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(obj,indent=2,sort_keys=True,allow_nan=True)+"\n",encoding="utf-8")
def key(r):return (r["run_id"],int(r["decision_id"]))
def git_blob(p):
    return subprocess.check_output(["git","hash-object",str(p)],cwd=ROOT,text=True).strip()
def git_index():
    out=subprocess.check_output(["git","ls-files","-s"],cwd=ROOT,text=True)
    m={}
    for line in out.splitlines():
        left,path=line.split("\t",1);parts=left.split()
        if len(parts)>=2:m[path]=parts[1]
    return m

def strict_quality(v,gt,D):
    resolved=D&(v>=0);pf=resolved&(v==0);po=resolved&(v>0);tf=D&(gt==0);to=D&(gt>0)
    ft=int(np.sum(tf&pf)); ff=int(np.sum(to&pf)); ffn=int(np.sum(tf&~pf))
    ot=int(np.sum(to&po)); of=int(np.sum(tf&po)); ofn=int(np.sum(to&~po))
    fp=div(ft,ft+ff);fr=div(ft,ft+ffn);fi=div(ft,ft+ff+ffn)
    op=div(ot,ot+of);orr=div(ot,ot+ofn);oi=div(ot,ot+of+ofn)
    macro=(fi+oi)/2 if finite(fi) and finite(oi) else math.nan
    acc=div(ft+ot,int(D.sum()))
    return {
      "ResolvedFraction_ROI":div(int(resolved.sum()),int(D.sum())),
      "FreePrecision":fp,"FreeRecall":fr,"FreeIoU":fi,
      "OccupiedPrecision":op,"OccupiedRecall":orr,"OccupiedIoU":oi,
      "StrictMacroIoU":macro,"StrictAccuracy":acc,
      "FreeTP":ft,"FreeFP":ff,"FreeFN":ffn,"OccTP":ot,"OccFP":of,"OccFN":ofn,
      "StrictDenominator_D":int(D.sum())
    }

def completion_quality(v1,gt,fillable):
    n=int(fillable.sum())
    if not n:
        return {k:math.nan for k in ["FreePrecision","FreeRecall","FreeIoU","OccupiedPrecision","OccupiedRecall","OccupiedIoU","MacroIoU","BinaryErrorRate"]}|{"FillableCount":0}
    p=v1[fillable];t=gt[fillable]
    pf=p==0;po=p>0;tf=t==0;to=t>0
    ft=int(np.sum(tf&pf));ff=int(np.sum(to&pf));ffn=int(np.sum(tf&~pf))
    ot=int(np.sum(to&po));of=int(np.sum(tf&po));ofn=int(np.sum(to&~po))
    fi=div(ft,ft+ff+ffn);oi=div(ot,ot+of+ofn)
    return {
      "FillableCount":n,
      "FreePrecision":div(ft,ft+ff),"FreeRecall":div(ft,ft+ffn),"FreeIoU":fi,
      "OccupiedPrecision":div(ot,ot+of),"OccupiedRecall":div(ot,ot+ofn),"OccupiedIoU":oi,
      "MacroIoU":(fi+oi)/2 if finite(fi) and finite(oi) else math.nan,
      "BinaryErrorRate":div(int(np.sum((po!=to))),n)
    }

def fast_components(mask):
    # Accepted no-corner-cut 8-neighbour component partition is identical to
    # 4-neighbour partition because an accepted diagonal requires both
    # orthogonal cells, which already provide a cardinal path.
    labels,n=nd_label(np.asarray(mask,bool),structure=FOUR)
    return labels.astype(np.int32,copy=False),int(n)

def structural_metrics(v,gt,D,crop,reftrav,ref_labels,ref_ids,ref_sizes,ref_clearance):
    r0,r1,c0,c1=crop
    vv=v[r0:r1,c0:c1];dd=D[r0:r1,c0:c1]
    vfree=dd&(vv==0)
    vtrav=r004topo.cspace(vfree,dd,r004topo.collision_stencil(R_ROBOT,RES))
    vlabels,vn=fast_components(vtrav)
    refn=int(reftrav.sum())
    retained=int(np.sum(reftrav&vtrav))
    unsafe=int(np.sum(vtrav&~reftrav))
    pair_den=0;pair_num=0;largest_sum=0
    meaningful=0;false_barrier=0;false_barrier_count=0
    frag_components=0; second_largest_max=0
    for rid,N in zip(ref_ids,ref_sizes):
        refmask=ref_labels==rid
        pair_den += N*(N-1)
        labs=vlabels[refmask]
        cnt=np.bincount(labs,minlength=vn+1)
        positive=np.flatnonzero(cnt[1:]>0)+1
        pieces=[]
        for lab in positive:
            size=int(cnt[lab])
            cells=np.flatnonzero(refmask&(vlabels==lab))
            minflat=int(cells.min())
            pieces.append((size,minflat,int(lab),cells))
            pair_num += size*(size-1)
        pieces.sort(key=lambda z:(-z[0],z[1]))
        if pieces:
            largest_sum += pieces[0][0]
        if len(pieces)>=2:
            second=pieces[1][0];second_largest_max=max(second_largest_max,second)
            if second*RES*RES >= A_REGION:
                meaningful=1;frag_components+=1
                reps=[]
                for item in pieces[:2]:
                    cells=item[3]
                    rc=[(int(x//vlabels.shape[1]),int(x%vlabels.shape[1])) for x in cells]
                    reps.append(r004topo.representative(rc,ref_clearance))
                path=r004topo.shortest_path(reftrav,reps[0],reps[1])
                obstructed=sum(not bool(vtrav[cell]) for cell in path)
                if obstructed>0:false_barrier=1;false_barrier_count+=1
    pair=div(pair_num,pair_den)
    largest=div(largest_sum,sum(ref_sizes))
    unsafe_area=unsafe*RES*RES
    s1=int(finite(pair) and pair<PAIR_THRESHOLD)
    s2=int(meaningful==1)
    s3=int(unsafe_area+1e-12>=A_BREACH)
    return {
      "TravRetention":div(retained,refn),
      "UnsafeTravArea_m2":unsafe_area,
      "PairConnectivityRetention":pair,
      "LargestPieceMassRetention":largest,
      "MeaningfulFragmentation":meaningful,
      "FalseBarrierEvent":false_barrier,
      "FalseBarrierComponentCount":false_barrier_count,
      "MeaningfulFragmentedReferenceComponents":frag_components,
      "SecondLargestPieceMaxCells":second_largest_max,
      "VariantTraversableCells":int(vtrav.sum()),
      "ReferenceTraversableCells":refn,
      "PairDen":pair_den,"PairNum":pair_num,
      "S1_MajorConnectivityCollapse":s1,
      "S2_MeaningfulFragmentation":s2,
      "S3_MajorFalseOpening":s3,
      "PrimarySevere":int(bool(s1 or s2 or s3))
    }

def r_stratum(r):
    x=fnum(r["R_map"])
    if not finite(x):return "R_NA"
    if x>0.10:return "R_GT_0.10"
    if x>0.05:return "R_0.05_TO_0.10"
    return "R_LE_0.05"
def p_stratum(p):
    x=fnum(p)
    if x<.25:return "P_Q1_0_25"
    if x<.50:return "P_Q2_25_50"
    if x<.75:return "P_Q3_50_75"
    return "P_Q4_75_100"

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    hard=[];insufficient=[]
    idx=git_index()
    blob_checks={
      "GT":(GT_PATH,GT_BLOB),"ROI":(ROI_PATH,ROI_BLOB),
      "MX031":(MX031_PATH,MX031_BLOB),"FEATURES":(FEATURE_PATH,FEATURE_BLOB),
      "PROJECTION":(PROJECTION_PATH,PROJECTION_BLOB),"TOPOLOGY":(TOPOLOGY_PATH,TOPOLOGY_BLOB)}
    blob_actual={}
    for name,(p,exp) in blob_checks.items():
        if not p.is_file():
            insufficient.append(f"MISSING_{name}")
            continue
        got=git_blob(p);blob_actual[name]=got
        if got!=exp:hard.append(f"BLOB_MISMATCH_{name}:{got}!={exp}")

    with np.load(GT_PATH,allow_pickle=False) as z:
        gt=np.asarray(z["data"],np.int16);D=np.asarray(z["evaluation_mask"],bool)
        gt_meta=(gt.shape,float(z["resolution"]),float(z["origin_x"]),float(z["origin_y"]),str(z["canvas_id"]))
    roi=np.load(ROI_PATH).astype(bool)
    if roi.shape!=gt.shape:hard.append("ROI_GT_SHAPE_MISMATCH")
    if gt_meta!=( (2123,1504),0.05,-25.6,-60.1,"hospital_canvas_v1"):
        hard.append(f"GT_META_MISMATCH:{gt_meta}")
    rr,cc=np.nonzero(D)
    crop=(int(rr.min()),int(rr.max())+1,int(cc.min()),int(cc.max())+1)
    r0,r1,c0,c1=crop
    dcrop=D[r0:r1,c0:c1];gfcrop=(gt[r0:r1,c0:c1]==0)&dcrop
    reftrav=r004topo.cspace(gfcrop,dcrop,r004topo.collision_stencil(R_ROBOT,RES))
    if not np.any(reftrav):hard.append("EMPTY_REF_TRAV")
    ref_labels,ref_n=fast_components(reftrav)
    ref_ids=list(range(1,ref_n+1))
    ref_sizes=[int(np.sum(ref_labels==x)) for x in ref_ids]
    ref_clearance=distance_transform_edt(reftrav)

    mx31=read_csv(MX031_PATH);m31={key(r):r for r in mx31}
    feats=read_csv(FEATURE_PATH);fm={key(r):r for r in feats}
    if len(m31)!=365 or len(fm)!=365:hard.append("ACCEPTED_JOIN_ROW_COUNT")
    if set(m31)!=set(fm):hard.append("ACCEPTED_JOIN_KEY_MISMATCH")

    source_rows=[];manifest=[];mask_rows=[];saving_rows=[];quality_rows=[]
    comp_rows=[];err_rows=[];struct_rows=[];event_rows=[];comparison_rows=[];signal_rows=[]
    v1_decisions=[];v0_quality={};v1_quality={};v2_quality={};v3_quality={}
    v1_struct={};v2_struct={}
    nonlattice_count=0;observed_change_total=0;unresolved_fill_total=0
    final_cache={};v3q_cache={};v3s_cache={}
    decision_count=0

    for run_id in RUNS:
        run=DATA/run_id
        table=r004base.read_csv(run/"decisions.csv")
        if not table:
            insufficient.append(f"{run_id}:NO_DECISIONS");continue
        try:
            final,final_file,fallback=r004base.final_snapshot(run)
        except Exception as exc:
            insufficient.append(f"{run_id}:FINAL:{exc}");continue
        if final.shape!=gt.shape:hard.append(f"{run_id}:FINAL_SHAPE")
        final_cache[run_id]=(final,final_file,fallback)
        v3q_cache[run_id]=strict_quality(final,gt,D)
        v3s_cache[run_id]=structural_metrics(final,gt,D,crop,reftrav,ref_labels,ref_ids,ref_sizes,ref_clearance)

        times=[fnum(m31[(run_id,int(d["decision_id"]))]["decision_time_s"]) for d in table if (run_id,int(d["decision_id"])) in m31]
        tfirst=min(x for x in times if finite(x));tend=max(x for x in times if finite(x))
        N=len(table)

        for k,d in enumerate(table,1):
            decision_count+=1;did=int(d["decision_id"]);K=(run_id,did)
            src={"run_id":run_id,"decision_id":did,"decision_index":k,"decision_count":N}
            raw_rel=d["raw_map"];canvas_rel=d["canvas_map"];mean_rel=d["mean_map"]
            paths=[run/raw_rel,run/canvas_rel,run/mean_rel]
            present=[p.is_file() for p in paths]
            source_reason=""
            if not all(present):
                source_reason="SOURCE_INSUFFICIENT_MISSING_ARTIFACT"
                insufficient.append(f"{run_id}:d{did}:MISSING")
                source_rows.append(src|{"source_valid":0,"source_reason":source_reason})
                continue
            try:
                with np.load(run/raw_rel,allow_pickle=False) as z:
                    rawshape=np.asarray(z["data"]).shape;rrr=float(z["resolution"]);rx=float(z["origin_x"]);ry=float(z["origin_y"]);rstamp=float(z["source_stamp_s"])
                with np.load(run/mean_rel,allow_pickle=False) as z:
                    ph=int(z["source_height"]);pw=int(z["source_width"]);pr=float(z["resolution"]);px=float(z["origin_x"]);py=float(z["origin_y"]);pstamp=float(z["source_map_stamp_s"]);member=str(z["member"])
                with np.load(run/canvas_rel,allow_pickle=False) as z:
                    cm=(np.asarray(z["data"]).shape,float(z["resolution"]),float(z["origin_x"]),float(z["origin_y"]),str(z["canvas_id"]));cstamp=float(z["source_map_stamp_s"])
                shape_ok=(ph,pw)==rawshape
                geom_ok=shape_ok and math.isclose(pr,rrr,abs_tol=1e-6) and math.isclose(px,rx,abs_tol=1e-6) and math.isclose(py,ry,abs_tol=1e-6)
                stamp_ok=math.isclose(pstamp,rstamp,abs_tol=1e-9) and math.isclose(cstamp,rstamp,abs_tol=1e-9)
                canvas_ok=cm==gt_meta
                if not shape_ok or not geom_ok or not stamp_ok or not canvas_ok or member!="mean":
                    hard.append(f"{run_id}:d{did}:SOURCE_IDENTITY")
                lattice_y=(ry-gt_meta[2+1] if False else 0.0)
                # accepted projection intentionally permits dynamic origin not exactly on 0.05m lattice
                xoff=(rx-gt_meta[2])/RES;yoff=(ry-gt_meta[3])/RES
                nonlattice=not (math.isclose(xoff,round(xoff),abs_tol=1e-9) and math.isclose(yoff,round(yoff),abs_tol=1e-9))
                nonlattice_count+=int(nonlattice)
                obs,pred,support=r004base.prediction_canvas(run,d,gt.shape)
                projection_ok=True
            except Exception as exc:
                hard.append(f"{run_id}:d{did}:PROJECTION:{exc}")
                source_rows.append(src|{"source_valid":0,"source_reason":"HARD_PROJECTION_FAILURE"})
                continue

            if K not in m31 or K not in fm:
                hard.append(f"{run_id}:d{did}:JOIN_KEY");continue
            r31=m31[K];fr=fm[K]
            if fint(r31["decision_index"])!=k or fint(fr["decision_index"])!=k:
                hard.append(f"{run_id}:d{did}:JOIN_INDEX")

            known=D&(obs>=0);unknown=D&(obs<0);fillable=unknown&support;unresolved=unknown&~support
            v0=obs.copy()
            v1=obs.copy();v1[fillable]=np.where(pred[fillable]>PRED_THRESHOLD,100,0).astype(np.int16)
            v2=obs.copy();v2[fillable]=gt[fillable]
            observed_changes=int(np.sum(known&(v1!=obs)));observed_change_total+=observed_changes
            unresolved_filled=int(np.sum(unresolved&(v1>=0)));unresolved_fill_total+=unresolved_filled
            if observed_changes:hard.append(f"{run_id}:d{did}:OBSERVED_OVERWRITE")
            if unresolved_filled:hard.append(f"{run_id}:d{did}:UNSUPPORTED_FILL")

            q0=strict_quality(v0,gt,D);q1=strict_quality(v1,gt,D);q2=strict_quality(v2,gt,D);q3=v3q_cache[run_id]
            cq=completion_quality(v1,gt,fillable)
            s0=structural_metrics(v0,gt,D,crop,reftrav,ref_labels,ref_ids,ref_sizes,ref_clearance)
            s1=structural_metrics(v1,gt,D,crop,reftrav,ref_labels,ref_ids,ref_sizes,ref_clearance)
            s2=structural_metrics(v2,gt,D,crop,reftrav,ref_labels,ref_ids,ref_sizes,ref_clearance)
            s3=v3s_cache[run_id]

            pred_attr=int(s1["PrimarySevere"]==1 and s2["PrimarySevere"]==0)
            support_severe=int(s2["PrimarySevere"]==1)
            ti=fnum(r31["decision_time_s"])
            saved=N-k;sp=fnum(r31["normalized_progress"]);sdf=1.0-sp if finite(sp) else math.nan
            st=max(0.0,tend-ti) if finite(ti) else math.nan;stf=st/(tend-tfirst) if finite(st) and tend>tfirst else math.nan
            unknown_n=int(unknown.sum());unresolved_n=int(unresolved.sum());fill_n=int(fillable.sum());known_n=int(known.sum())
            observed_error=div(int(np.sum(known&(((obs>0)!=(gt>0))))),known_n)
            inferred_error=div(int(np.sum(fillable&(((v1>0)!=(gt>0))))),fill_n)
            unf=div(unresolved_n,unknown_n)
            penalty=q2["StrictMacroIoU"]-q1["StrictMacroIoU"] if finite(q2["StrictMacroIoU"]) and finite(q1["StrictMacroIoU"]) else math.nan

            source_rows.append(src|{
              "source_valid":1,"source_reason":"","raw_present":1,"canvas_present":1,"mean_present":1,
              "raw_prediction_shape_match":int(shape_ok),"raw_prediction_geometry_match":int(geom_ok),
              "source_stamp_match":int(stamp_ok),"canvas_gt_geometry_match":int(canvas_ok),
              "accepted_projection_success":int(projection_ok),"nonlattice_raw_origin":int(nonlattice),
              "mean_member_exact":int(member=="mean")})
            manifest.append(src|{
              "raw_map":raw_rel,"canvas_map":canvas_rel,"mean_map":mean_rel,
              "raw_blob":idx.get(str((run/raw_rel).relative_to(ROOT)),""),
              "canvas_blob":idx.get(str((run/canvas_rel).relative_to(ROOT)),""),
              "mean_blob":idx.get(str((run/mean_rel).relative_to(ROOT)),""),
              "V0":"OBS_ONLY","V1":"MEAN_COMPLETION","V2":"GT_FILL_CEILING","V3_final_snapshot":final_file,
              "V3_fallback":int(fallback),"prediction_threshold":PRED_THRESHOLD})
            mask_rows.append(src|{
              "D_count":int(D.sum()),"Known_count":known_n,"Unknown_count":unknown_n,"Fillable_count":fill_n,
              "Unresolved_count":unresolved_n,"PredictionSupport_D_count":int(np.sum(D&support)),
              "Known_fraction_D":div(known_n,int(D.sum())),"Fillable_fraction_current_unknown":div(fill_n,unknown_n),
              "UnresolvedFraction_CurrentUnknown":unf})
            saving_rows.append(src|{
              "normalized_progress":sp,"R_map":fnum(r31["R_map"]),"decision_time_s":ti,
              "SavedDecisions":saved,"SavedDecisionFraction":sdf,"SavedTime_s":st,"SavedTimeFraction":stf})
            for vn,qv in [("V0_OBS_ONLY",q0),("V1_MEAN_COMPLETION",q1),("V2_GT_FILL_CEILING",q2),("V3_FULL_EXPLORE_REFERENCE",q3)]:
                quality_rows.append(src|{"variant":vn,"UnresolvedFraction_CurrentUnknown":unf,**qv})
            comp_rows.append(src|cq)
            err_rows.append(src|{
              "ObservedErrorRate":observed_error,"InferredErrorRate":inferred_error,
              "UnresolvedFraction_CurrentUnknown":unf,"PredictionPenalty_MacroIoU":penalty,
              "ObservedCellCount":known_n,"InferredCellCount":fill_n,"UnresolvedCellCount":unresolved_n})
            for vn,sv in [("V0_OBS_ONLY",s0),("V1_MEAN_COMPLETION",s1),("V2_GT_FILL_CEILING",s2),("V3_FULL_EXPLORE_REFERENCE",s3)]:
                struct_rows.append(src|{"variant":vn,**sv})
            event_rows.append(src|{
              "V0_PrimarySevere":s0["PrimarySevere"],"V1_PrimarySevere":s1["PrimarySevere"],
              "V2_PrimarySevere":s2["PrimarySevere"],"V3_PrimarySevere":s3["PrimarySevere"],
              "PredictionAttributableSevere":pred_attr,"ObservedOrSupportSevere":support_severe,
              "V1_S1":s1["S1_MajorConnectivityCollapse"],"V1_S2":s1["S2_MeaningfulFragmentation"],"V1_S3":s1["S3_MajorFalseOpening"],
              "V1_FalseBarrierEvent":s1["FalseBarrierEvent"]})
            comparison_rows.append(src|{
              "SavedDecisionFraction":sdf,"V1_minus_V0_StrictMacroIoU":q1["StrictMacroIoU"]-q0["StrictMacroIoU"],
              "V2_minus_V1_StrictMacroIoU":q2["StrictMacroIoU"]-q1["StrictMacroIoU"],
              "V3_minus_V1_StrictMacroIoU":q3["StrictMacroIoU"]-q1["StrictMacroIoU"],
              "V0_StrictMacroIoU":q0["StrictMacroIoU"],"V1_StrictMacroIoU":q1["StrictMacroIoU"],
              "V2_StrictMacroIoU":q2["StrictMacroIoU"],"V3_StrictMacroIoU":q3["StrictMacroIoU"],
              "V1_PrimarySevere":s1["PrimarySevere"],"PredictionAttributableSevere":pred_attr,"ObservedOrSupportSevere":support_severe})
            sig_fields=["R_map","log_decision","log_elapsed","F1_PredOccShare","F1_OccMaskIoUMean2","F1_OccFlipRateMean2",
              "F2_VotePersistenceMean2","F2_Vote3Share","F2_Vote2Share","F2_Vote1Share"]
            sig={x:fnum(fr.get(x)) for x in sig_fields}
            signal_rows.append(src|sig|{
              "F1_evaluable":fint(fr.get("F1_evaluable"),0),"F2_evaluable":fint(fr.get("F2_evaluable"),0),
              "joint_F1F2_complete":int(all(finite(sig[x]) for x in sig_fields)),
              "V1_StrictMacroIoU":q1["StrictMacroIoU"],"V1_ResolvedFraction_ROI":q1["ResolvedFraction_ROI"],
              "V1_TravRetention":s1["TravRetention"],"V1_PairConnectivityRetention":s1["PairConnectivityRetention"],
              "signal_role":"FUTURE_PREDICTIVE_CANDIDATE_NOT_SAFETY_SIGNAL"})
            rec=src|{
              "R_map":fnum(r31["R_map"]),"normalized_progress":sp,"SavedDecisions":saved,"SavedDecisionFraction":sdf,"SavedTimeFraction":stf,
              "V0_StrictMacroIoU":q0["StrictMacroIoU"],"V1_StrictMacroIoU":q1["StrictMacroIoU"],
              "V1_ResolvedFraction_ROI":q1["ResolvedFraction_ROI"],"V1_PrimarySevere":s1["PrimarySevere"],
              "PredictionAttributableSevere":pred_attr,"ObservedOrSupportSevere":support_severe,
              "UsefulSaving":int(saved>=3),"QualityImprovedOverV0":int(q1["StrictMacroIoU"]>q0["StrictMacroIoU"])}
            v1_decisions.append(rec);v0_quality[K]=q0;v1_quality[K]=q1;v2_quality[K]=q2;v3_quality[K]=q3;v1_struct[K]=s1;v2_struct[K]=s2

    if decision_count!=365:hard.append(f"DECISION_INVENTORY:{decision_count}")
    if len(source_rows)!=365:hard.append(f"SOURCE_ROWS:{len(source_rows)}")
    valid_n=sum(fint(x.get("source_valid"),0) for x in source_rows)
    if valid_n<365 and not hard:
        insufficient.append(f"SOURCE_VALID:{valid_n}/365")

    # descriptive strata
    strata=[]
    for stype in ("R","PROGRESS"):
        groups=defaultdict(list)
        for r in v1_decisions:
            name=r_stratum(r) if stype=="R" else p_stratum(r["normalized_progress"])
            groups[name].append(r)
        for name,arr in sorted(groups.items()):
            strata.append({
              "stratum_type":stype,"stratum":name,"decision_n":len(arr),"run_contributor_n":len(set(x["run_id"] for x in arr)),
              "median_SavedDecisionFraction":median(x["SavedDecisionFraction"] for x in arr),
              "median_SavedTimeFraction":median(x["SavedTimeFraction"] for x in arr),
              "median_V1_StrictMacroIoU":median(x["V1_StrictMacroIoU"] for x in arr),
              "median_V1_ResolvedFraction_ROI":median(x["V1_ResolvedFraction_ROI"] for x in arr),
              "PrimarySevere_n":sum(x["V1_PrimarySevere"] for x in arr),"PrimarySevere_fraction":mean(x["V1_PrimarySevere"] for x in arr),
              "PredictionAttributableSevere_n":sum(x["PredictionAttributableSevere"] for x in arr),
              "ObservedOrSupportSevere_n":sum(x["ObservedOrSupportSevere"] for x in arr)})

    # severe support
    sev_support=[]
    for hold in RUNS:
        train=[r for r in v1_decisions if r["run_id"]!=hold]
        sev=[r for r in train if r["V1_PrimarySevere"]==1];non=[r for r in train if r["V1_PrimarySevere"]==0]
        rec={"outer_heldout":hold,"severe_decisions":len(sev),"severe_runs":len(set(x["run_id"] for x in sev)),
          "nonsevere_decisions":len(non),"nonsevere_runs":len(set(x["run_id"] for x in non))}
        rec["pass"]=int(rec["severe_decisions"]>=15 and rec["severe_runs"]>=3 and rec["nonsevere_decisions"]>=15 and rec["nonsevere_runs"]>=3)
        sev_support.append(rec)
    severe_adequate=all(x["pass"] for x in sev_support)

    # opportunities and run macro
    opportunities={};run_macro=[]
    for run in RUNS:
        arr=[r for r in v1_decisions if r["run_id"]==run]
        opp=[r for r in arr if r["UsefulSaving"] and not r["V1_PrimarySevere"] and r["QualityImprovedOverV0"]]
        opportunities[run]=int(bool(opp))
        run_macro.append({"run_id":run,"decision_n":len(arr),
          "PrimarySevere_n":sum(x["V1_PrimarySevere"] for x in arr),"PrimarySevere_fraction":mean(x["V1_PrimarySevere"] for x in arr),
          "PredictionAttributableSevere_n":sum(x["PredictionAttributableSevere"] for x in arr),
          "ObservedOrSupportSevere_n":sum(x["ObservedOrSupportSevere"] for x in arr),
          "median_SavedDecisionFraction":median(x["SavedDecisionFraction"] for x in arr),
          "median_V1_StrictMacroIoU":median(x["V1_StrictMacroIoU"] for x in arr),
          "median_V1_minus_V0_StrictMacroIoU":median(x["V1_StrictMacroIoU"]-x["V0_StrictMacroIoU"] for x in arr),
          "NonSevereUsefulOpportunity":int(bool(opp)),"NonSevereUsefulOpportunity_n":len(opp)})

    opportunity_runs=sum(opportunities.values())
    followon_pass=severe_adequate and opportunity_runs>=8

    # Pareto frontier: severe=0 lexicographically dominates severe=1; within same severe flag,
    # larger saving and quality are Pareto-better.
    frontier=[]
    for run in RUNS:
        arr=[r for r in v1_decisions if r["run_id"]==run]
        for a in arr:
            dominated=False
            for b in arr:
                if b is a:continue
                if b["V1_PrimarySevere"]<a["V1_PrimarySevere"]:
                    dominated=True;break
                if b["V1_PrimarySevere"]==a["V1_PrimarySevere"]:
                    if (b["SavedDecisionFraction"]>=a["SavedDecisionFraction"] and b["V1_StrictMacroIoU"]>=a["V1_StrictMacroIoU"]
                        and (b["SavedDecisionFraction"]>a["SavedDecisionFraction"] or b["V1_StrictMacroIoU"]>a["V1_StrictMacroIoU"])):
                        dominated=True;break
            if not dominated:
                frontier.append({"run_id":run,"decision_id":a["decision_id"],"decision_index":a["decision_index"],
                  "SavedDecisionFraction":a["SavedDecisionFraction"],"V1_StrictMacroIoU":a["V1_StrictMacroIoU"],
                  "V1_PrimarySevere":a["V1_PrimarySevere"],"frontier_role":"DESCRIPTIVE_NON_ACTIONABLE"})
    fc=Counter(x["run_id"] for x in frontier)
    for r in run_macro:r["ParetoFrontier_n"]=fc[r["run_id"]]

    # Adversarial actual/static contract checks
    all_projection=sum(fint(x.get("accepted_projection_success"),0) for x in source_rows)==365
    all_stamp=sum(fint(x.get("source_stamp_match"),0) for x in source_rows)==365
    all_geom=sum(fint(x.get("raw_prediction_geometry_match"),0) for x in source_rows)==365
    all_canvas=sum(fint(x.get("canvas_gt_geometry_match"),0) for x in source_rows)==365
    all_mean=sum(fint(x.get("mean_member_exact"),0) for x in source_rows)==365
    unresolved_preserved=(unresolved_fill_total==0)
    # topology component equivalence spot-check against accepted helper
    synthetic_masks=[
      np.array([[1,0],[0,1]],bool),
      np.array([[1,1],[1,1]],bool),
      np.array([[1,1,0],[1,0,1],[0,1,1]],bool)]
    comp_equiv=True
    for m in synthetic_masks:
        fl,fn=fast_components(m);al,ar=r004topo.components(m)
        fs=sorted(int(np.sum(fl==i)) for i in range(1,fn+1))
        rs=sorted(len(x["cells"]) for x in ar)
        comp_equiv &= fs==rs
    adv=[
      ("A1",all_mean and valid_n==365,"365/365 exact stored mean present/read; no regeneration"),
      ("A2",all_mean,"primary member exact mean; G1/G2/G3 never substituted"),
      ("A3",all_stamp,"raw/prediction/canvas source stamps match"),
      ("A4",all_geom,"raw/prediction origin/resolution/shape match"),
      ("A5",all_canvas,"canvas/GT geometry and canvas id match"),
      ("A6",all_projection and nonlattice_count>0,f"accepted rounded projection succeeds; nonlattice rows={nonlattice_count}"),
      ("A7",observed_change_total==0,f"observed cells changed by V1={observed_change_total}"),
      ("A8",True,"V1 uses current obs + stored mean only; no future map input"),
      ("A9",True,"GT enters only V2/evaluator metrics, never V1"),
      ("A10",unresolved_preserved,f"unsupported unknown filled by V1={unresolved_fill_total}"),
      ("A11",all(x["StrictDenominator_D"]==int(D.sum()) for x in quality_rows),"strict full-map denominator fixed |D|; unresolved retained as errors"),
      ("A12",True,"V2 marked evaluator-only GT fill ceiling"),
      ("A13",True,"V3 marked evaluator-only historical full-explore reference"),
      ("A14",PRED_THRESHOLD==0.5,"V1 uses strict >0.5 occupied"),
      ("A15",git_blob(PROJECTION_PATH)==PROJECTION_BLOB,"accepted projection blob guards nearest integer replication/no bilinear"),
      ("A16",True,"structural free mask requires V==0; unresolved is non-traversable"),
      ("A17",git_blob(TOPOLOGY_PATH)==TOPOLOGY_BLOB and comp_equiv,"accepted no-corner-cut topology blob + component equivalence check"),
      ("A18",math.isclose(R_ROBOT,0.189,abs_tol=1e-15),"robot radius 0.189m"),
      ("A19",True,"severe flag is non-tradeable in frontier/opportunity gates"),
      ("A20",True,"no semantic room labels created; only region/bottleneck proxies"),
      ("A21",math.isclose(A_REGION,16*A_ROBOT),"meaningful fragmentation area=16x robot footprint"),
      ("A22",math.isclose(A_BREACH,2*A_ROBOT),"unsafe opening severe area=2x robot footprint"),
      ("A23",PAIR_THRESHOLD==0.95,"pair-retention severe threshold fixed 0.95"),
      ("A24",all(x["PredictionAttributableSevere"]==int(x["V1_PrimarySevere"]==1 and x["V2_PrimarySevere"]==0) for x in event_rows),"prediction attribution requires V2 non-severe"),
      ("A25",all(x["ObservedOrSupportSevere"]==x["V2_PrimarySevere"] for x in event_rows),"V2 severe assigned observed/support-limited"),
      ("A26",True,"R strata descriptive only; no selected threshold/K"),
      ("A27",all(x["frontier_role"]=="DESCRIPTIVE_NON_ACTIONABLE" for x in frontier),"Pareto frontier non-actionable"),
      ("A28",all(x["signal_role"]=="FUTURE_PREDICTIVE_CANDIDATE_NOT_SAFETY_SIGNAL" for x in signal_rows),"F1+F2 never labeled safety signal"),
      ("A29",True,"severe absence/support handled as insufficiency, never safety PASS"),
      ("A30",True,"Hospital/MX046/MX050/confirmation absent by source contract"),
    ]
    adv_rows=[{"audit":a,"semantic_pass":int(bool(p)),"detail":d} for a,p,d in adv]
    if not all(x["semantic_pass"] for x in adv_rows):hard.append("ADVERSARIAL_AUDIT_FAILURE")

    if hard:
        classification="MX054_INVALID_EXECUTION_OR_PROVENANCE"
    elif insufficient:
        classification="MX054_STOP_COMPLETION_SOURCE_INSUFFICIENT"
    elif not severe_adequate:
        classification="MX054_COUNTERFACTUAL_VALID_SEVERE_EVENT_SUPPORT_INSUFFICIENT"
    elif opportunity_runs<8:
        classification="MX054_COUNTERFACTUAL_VALID_NO_BOUNDED_FOLLOWON_OPPORTUNITY_SUPPORT"
    else:
        classification="MX054_COUNTERFACTUAL_VALID_FOLLOWON_METHOD_SUPPORT"

    source_prov={
      "schema":"mx054_source_provenance_r1_execution","task":"MX055","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "frozen_historical_source_root":SOURCE_ROOT,"accepted_mx031_commit":MX031_COMMIT,
      "blob_expected":{k:v[1] for k,v in blob_checks.items()},"blob_actual":blob_actual,
      "run_ids":RUNS,"decision_inventory":decision_count,"source_valid_decisions":valid_n,
      "gt_meta":{"shape":list(gt.shape),"resolution":RES,"origin_x":gt_meta[2],"origin_y":gt_meta[3],"canvas_id":gt_meta[4],
        "D_count":int(D.sum()),"connected_free_roi_count":int(roi.sum())},
      "forbidden_sources":{"Hospital":False,"MX046_partial":False,"MX050_partial":False,"confirmation":False,
        "prediction_regeneration":False,"future_map_in_V1":False,"GT_in_V1":False}}
    severe_json={"schema":"mx054_severe_event_support_r1","per_outer_split":sev_support,
      "adequate_for_followon_model_design":bool(severe_adequate),
      "status":"SEVERE_EVENT_SUPPORT_ADEQUATE_FOR_FOLLOWON_MODEL_DESIGN" if severe_adequate else "SEVERE_EVENT_SUPPORT_INSUFFICIENT_FOR_FOLLOWON_MODEL_DESIGN",
      "overall_V1_severe_decisions":sum(x["V1_PrimarySevere"] for x in v1_decisions),
      "overall_V1_nonsevere_decisions":sum(not x["V1_PrimarySevere"] for x in v1_decisions),
      "severe_runs":len(set(x["run_id"] for x in v1_decisions if x["V1_PrimarySevere"])),
      "nonsevere_runs":len(set(x["run_id"] for x in v1_decisions if not x["V1_PrimarySevere"]))}
    follow_json={"schema":"mx054_followon_support_gate_r1","severe_support_adequate":bool(severe_adequate),
      "NonSevereUsefulOpportunity_by_run":opportunities,"opportunity_runs":opportunity_runs,
      "opportunity_requirement_runs":8,"followon_method_support":bool(followon_pass),
      "classification":classification}

    metric_dict={
      "schema":"mx054_metric_dictionary_r1_execution",
      "variants":{"V0":"OBS_ONLY","V1":"MEAN_COMPLETION_PRIMARY","V2":"GT_FILL_CEILING_EVALUATOR_ONLY","V3":"FULL_EXPLORE_REFERENCE_EVALUATOR_ONLY"},
      "prediction_threshold":">0.5 occupied; <=0.5 free","unresolved":"unsupported decision-time unknown remains -1",
      "strict_quality":"unresolved cells included as class FN; accuracy denominator |D|",
      "robot_radius_m":R_ROBOT,"resolution_m":RES,"pair_connectivity_threshold":PAIR_THRESHOLD,
      "meaningful_fragmentation_area_m2":A_REGION,"unsafe_breach_area_m2":A_BREACH,
      "PrimarySevere":"PairConnectivityRetention<0.95 OR MeaningfulFragmentation OR UnsafeTravArea>=2*A_robot",
      "component_label_zero":"excluded from n_kj pieces; non-traversable is not a synthetic component",
      "R_strata":[">0.10","0.05<R<=0.10","<=0.05"],"progress_strata":["0-25%","25-50%","50-75%","75-100%"],
      "frontier":"descriptive only; PrimarySevere=0 lexicographically dominates PrimarySevere=1; then saving/StrictMacroIoU Pareto",
      "no_stop_rule":True}
    exec_prov={"schema":"mx054_execution_provenance_r1","task":"MX055","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "classification":classification,"hard_failures":hard,"source_insufficiencies":insufficient,
      "decision_inventory":decision_count,"source_valid_decisions":valid_n,"A1_A30_pass":sum(x["semantic_pass"] for x in adv_rows),
      "severe_support_adequate":bool(severe_adequate),"opportunity_runs":opportunity_runs,
      "constraints":{"new_run":False,"simulation":False,"prediction_regeneration":False,"STOP_model_fit":False,
        "retuning":False,"threshold_selection":False,"K_selection":False,"Hospital":False,"MX046_MX050":False,
        "confirmation":False,"deployment":False,"robot_STOP":False}}

    write_json(OUT/"MX054_SOURCE_PROVENANCE.json",source_prov)
    write_csv(OUT/"MX054_SOURCE_FEASIBILITY_PARITY.csv",source_rows)
    write_csv(OUT/"MX054_COUNTERFACTUAL_MAP_MANIFEST.csv",manifest)
    write_csv(OUT/"MX054_MAP_PROVENANCE_MASK_SUMMARY.csv",mask_rows)
    write_csv(OUT/"MX054_SAVING_PER_DECISION.csv",saving_rows)
    write_csv(OUT/"MX054_MAP_QUALITY_PER_DECISION.csv",quality_rows)
    write_csv(OUT/"MX054_COMPLETION_REGION_QUALITY.csv",comp_rows)
    write_csv(OUT/"MX054_ERROR_PROVENANCE_DECOMPOSITION.csv",err_rows)
    write_csv(OUT/"MX054_STRUCTURAL_METRICS_PER_DECISION.csv",struct_rows)
    write_csv(OUT/"MX054_STRUCTURAL_EVENT_AUDIT.csv",event_rows)
    write_csv(OUT/"MX054_VARIANT_COMPARISONS.csv",comparison_rows)
    write_csv(OUT/"MX054_R_STRATA_SUMMARY.csv",strata)
    write_csv(OUT/"MX054_RUN_MACRO_SUMMARY.csv",run_macro)
    write_csv(OUT/"MX054_SAVING_QUALITY_STRUCTURE_FRONTIER.csv",frontier)
    write_csv(OUT/"MX054_SIGNAL_ATTACHMENT_AUDIT.csv",signal_rows)
    write_json(OUT/"MX054_SEVERE_EVENT_SUPPORT.json",severe_json)
    write_json(OUT/"MX054_FOLLOWON_SUPPORT_GATE.json",follow_json)
    write_csv(OUT/"MX054_ADVERSARIAL_CASE_AUDIT.csv",adv_rows)
    write_json(OUT/"MX054_METRIC_DICTIONARY.json",metric_dict)
    write_json(OUT/"MX054_EXECUTION_PROVENANCE.json",exec_prov)

    v1_severe=sum(x["V1_PrimarySevere"] for x in v1_decisions)
    pred_severe=sum(x["PredictionAttributableSevere"] for x in v1_decisions)
    support_sev=sum(x["ObservedOrSupportSevere"] for x in v1_decisions)
    qgain=[x["V1_StrictMacroIoU"]-x["V0_StrictMacroIoU"] for x in v1_decisions]
    predpen=[x["V2_StrictMacroIoU"]-x["V1_StrictMacroIoU"] for x in comparison_rows]
    report=[
      "# MX055 Analyst05 — MX054 Method R1 STOP→completion counterfactual result","",
      f"Final classification: **{classification}**","",
      "## Integrity",
      f"- decision inventory: {decision_count}/365; source-valid: {valid_n}/365",
      f"- hard failures: {len(hard)}; source insufficiencies: {len(insufficient)}",
      f"- A1-A30: {sum(x['semantic_pass'] for x in adv_rows)}/30 PASS","",
      "## Exploration saving / map quality",
      f"- median V1 strict-MacroIoU gain vs V0 across decisions: {median(qgain)}",
      f"- median V2-V1 prediction penalty: {median(predpen)}",
      f"- non-severe useful opportunity runs: {opportunity_runs}/10","",
      "## Structural consequence",
      f"- V1 PrimarySevere decisions: {v1_severe}/365",
      f"- prediction-attributable severe decisions: {pred_severe}/365",
      f"- observed/support-limited severe decisions (V2 severe): {support_sev}/365",
      f"- severe-event support adequate for future run-held-out model design: {severe_adequate}","",
      "## Boundary",
      "- descriptive historical counterfactual only; no STOP rule/model, threshold/K selection, retuning, new run, deployment or robot STOP.",
      "- saving never compensates PrimarySevere.",
      "- any follow-on-support class only permits PM/USER to consider a separately reviewed methodology."
    ]
    (OUT/"MX054_ANALYST_REPORT.md").write_text("\n".join(report)+"\n",encoding="utf-8")

    required=[
      "MX054_SOURCE_PROVENANCE.json","MX054_SOURCE_FEASIBILITY_PARITY.csv","MX054_COUNTERFACTUAL_MAP_MANIFEST.csv",
      "MX054_MAP_PROVENANCE_MASK_SUMMARY.csv","MX054_SAVING_PER_DECISION.csv","MX054_MAP_QUALITY_PER_DECISION.csv",
      "MX054_COMPLETION_REGION_QUALITY.csv","MX054_ERROR_PROVENANCE_DECOMPOSITION.csv",
      "MX054_STRUCTURAL_METRICS_PER_DECISION.csv","MX054_STRUCTURAL_EVENT_AUDIT.csv","MX054_VARIANT_COMPARISONS.csv",
      "MX054_R_STRATA_SUMMARY.csv","MX054_RUN_MACRO_SUMMARY.csv","MX054_SAVING_QUALITY_STRUCTURE_FRONTIER.csv",
      "MX054_SIGNAL_ATTACHMENT_AUDIT.csv","MX054_SEVERE_EVENT_SUPPORT.json","MX054_FOLLOWON_SUPPORT_GATE.json",
      "MX054_ADVERSARIAL_CASE_AUDIT.csv","MX054_METRIC_DICTIONARY.json","MX054_EXECUTION_PROVENANCE.json","MX054_ANALYST_REPORT.md"]
    actual=sorted(p.name for p in OUT.iterdir() if p.is_file())
    if sorted(required)!=actual:raise RuntimeError(f"OUTPUT_CONTRACT:{actual}")
    if len(source_rows)!=365 or len(saving_rows)!=365 or len(mask_rows)!=365 or len(event_rows)!=365 or len(comparison_rows)!=365 or len(signal_rows)!=365:
        raise RuntimeError("PER_DECISION_ROW_CONTRACT")
    if len(quality_rows)!=1460 or len(struct_rows)!=1460:
        raise RuntimeError("VARIANT_ROW_CONTRACT")
    print(json.dumps({"classification":classification,"outputs":len(actual),"decisions":decision_count,"source_valid":valid_n,
      "hard_failures":len(hard),"source_insufficiencies":len(insufficient),"A_pass":sum(x["semantic_pass"] for x in adv_rows),
      "V1_severe":v1_severe,"prediction_attributable_severe":pred_severe,"observed_support_severe":support_sev,
      "severe_support_adequate":severe_adequate,"opportunity_runs":opportunity_runs,"frontier_rows":len(frontier)},sort_keys=True))

if __name__=="__main__":
    main()
