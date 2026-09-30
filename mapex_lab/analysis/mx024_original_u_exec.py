#!/usr/bin/env python3
from __future__ import annotations
import csv, hashlib, json, math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from mapex_lab.analysis.d1 import d1_gate_p as gate

BASE_SHA="4899ee95c85640965befeaf98c20f156c0d3181a"
METHOD_SHA="6fe651e383b3d88d1a824558d6ec398fc97eef1c"
LAB=Path(__file__).resolve().parents[1]
EXP=LAB/"experiments"/"mapex"
GT_PATH=LAB/"ground_truth"/"new_room"/"generated"/"new_room_structural_gt_v2.npz"
SHARED=LAB/"analysis"/"d1"/"results"/"shared_phase0_evidence_v1"/"shared_phase0_evidence.csv"
OUT=LAB/"analysis"/"mx024_exec_results"
FIG=OUT/"figures"
ORACLE={"mpx_001":21,"mpx_002":18,"mpx_003":21,"mpx_004":16,"mpx_005":16,
        "mpx_006":20,"mpx_007":18,"mpx_008":19,"mpx_009":18,"mpx_010":19}
RUNS=list(ORACLE)

def read_csv(p):
    with Path(p).open(newline="",encoding="utf-8") as f: return list(csv.DictReader(f))

def write_csv(p, rows):
    rows=list(rows); Path(p).parent.mkdir(parents=True,exist_ok=True)
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields: fields.append(k)
    with Path(p).open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)

def digest(p):
    h=hashlib.sha256()
    with Path(p).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""): h.update(b)
    return h.hexdigest()

def scalar(z,k):
    v=np.asarray(z[k]).reshape(()).item()
    return v.decode() if isinstance(v,bytes) else v

def integral(v,name):
    x=float(v)
    if not math.isfinite(x) or int(x)!=x: raise ValueError(name+"_NONINTEGRAL")
    return int(x)

def finite(v):
    try: x=float(v)
    except Exception: return math.nan
    return x if math.isfinite(x) else math.nan

def quant(vals,q):
    a=np.asarray(list(vals),dtype=float); a=a[np.isfinite(a)]
    return float(np.quantile(a,q,method="linear")) if a.size else math.nan

def median(vals): return quant(vals,0.5)

def resolve(run,rel):
    p=(run/rel).resolve()
    if run.resolve() not in p.parents: raise ValueError("PATH_ESCAPES_RUN")
    return p

def load_raw(p,did):
    if p.name!=f"decision_{did:06d}_raw.npz": raise ValueError("RAW_IDENTITY_FAIL")
    with np.load(p,allow_pickle=False) as z:
        data=np.asarray(z["data"])
        keys=("resolution","width","height","origin_x","origin_y","origin_yaw","frame_id","source_stamp_s")
        meta={k:scalar(z,k) for k in keys}
    h,w=integral(meta["height"],"RAW_HEIGHT"),integral(meta["width"],"RAW_WIDTH")
    if data.ndim!=2 or data.shape!=(h,w): raise ValueError("RAW_SHAPE_FAIL")
    if not np.all(np.isfinite(data)): raise ValueError("RAW_NONFINITE")
    if not math.isfinite(float(meta["resolution"])) or float(meta["resolution"])<=0: raise ValueError("RAW_RESOLUTION_FAIL")
    for k in ("origin_x","origin_y","origin_yaw","source_stamp_s"):
        if not math.isfinite(float(meta[k])): raise ValueError("RAW_"+k.upper()+"_FAIL")
    if float(meta["origin_yaw"])!=0.0: raise ValueError("RAW_ORIGIN_YAW_FAIL")
    if str(meta["frame_id"])!="map": raise ValueError("RAW_FRAME_FAIL")
    return data,meta

def load_pred(p,did,member,raw,rawmeta,crop0=None):
    if p.name!=f"decision_{did:06d}_{member.lower()}.npz": raise ValueError(member+"_IDENTITY_FAIL")
    with np.load(p,allow_pickle=False) as z:
        need=("source_height","source_width","pad_top","pad_left","member","source_map_stamp_s","resolution","origin_x","origin_y")
        missing=[k for k in need if k not in z.files]
        if missing: raise ValueError(member+"_MISSING_META")
        data=np.asarray(z["data"]); m={k:scalar(z,k) for k in need}
    h,w=integral(m["source_height"],"SOURCE_HEIGHT"),integral(m["source_width"],"SOURCE_WIDTH")
    t,l=integral(m["pad_top"],"PAD_TOP"),integral(m["pad_left"],"PAD_LEFT")
    if data.ndim!=2 or (h,w)!=raw.shape: raise ValueError(member+"_SOURCE_SHAPE_FAIL")
    if min(t,l)<0 or t+h>data.shape[0] or l+w>data.shape[1]: raise ValueError(member+"_CROP_BOUNDS_FAIL")
    if str(m["member"])!=member: raise ValueError(member+"_MEMBER_FAIL")
    if m["source_map_stamp_s"]!=rawmeta["source_stamp_s"]: raise ValueError(member+"_STAMP_FAIL")
    if m["resolution"]!=rawmeta["resolution"] or m["origin_x"]!=rawmeta["origin_x"] or m["origin_y"]!=rawmeta["origin_y"]:
        raise ValueError(member+"_GEOMETRY_FAIL")
    crop=(h,w,t,l)
    if crop0 is not None and crop!=crop0: raise ValueError(member+"_CROP_METADATA_MISMATCH")
    a=np.asarray(data[t:t+h,l:l+w])
    if a.shape!=raw.shape or not np.all(np.isfinite(a)): raise ValueError(member+"_CROP_OR_FINITE_FAIL")
    if member=="variance":
        if np.any(a<0): raise ValueError("VARIANCE_NEGATIVE")
    elif np.any(a<0) or np.any(a>1):
        raise ValueError(member+"_RANGE_FAIL")
    return a,crop,m,data.shape

def q2_record(run_id,did,n,dp,band,domain,var,truth,pred,cell_area,reason=""):
    base={"run_id":run_id,"decision_id":did,"decision_count":n,"delta_p":dp,"band":band,
          "domain":domain,"evaluable":0,"reason":reason}
    if reason: return base
    v=np.asarray(var,dtype=float); t=np.asarray(truth,dtype=bool); po=np.asarray(pred,dtype=float)>=0.5
    if not (v.size==t.size==po.size): return dict(base,reason="Q2_SIZE_MISMATCH")
    if v.size==0: return dict(base,reason="EMPTY_Q2_TRUTH_DOMAIN")
    mf=(~t)&po; fo=t&(~po); err=mf|fo; cor=~err
    def pack(mask):
        x=v[mask]
        return {"count":int(x.size),"area_m2":float(x.size*cell_area),
                "mean":float(np.mean(x)) if x.size else math.nan,
                "median":median(x),"q95":quant(x,0.95)}
    out=dict(base,evaluable=1,reason="",truth_domain_count=int(v.size),truth_domain_area_m2=float(v.size*cell_area))
    for name,mask in (("Correct",cor),("AllError",err),("MissedFree",mf),("FalseOpen",fo)):
        for k,val in pack(mask).items(): out[name+"_"+k]=val
    q95=quant(v,0.95); high=v>=q95
    hn,en,mn,fn=map(int,(np.count_nonzero(high),np.count_nonzero(err),np.count_nonzero(mf),np.count_nonzero(fo)))
    he=int(np.count_nonzero(high&err)); base_rate=en/v.size; high_rate=he/hn if hn else math.nan
    out.update(q95_truth_domain=q95,high_u_count=hn,high_u_fraction=hn/v.size,
               HighUErrorRate=high_rate,BaselineErrorRate=base_rate,
               ErrorEnrichment95=(high_rate/base_rate if base_rate>0 and math.isfinite(high_rate) else math.nan),
               ErrorCapture95=(he/en if en else math.nan),
               MissedFreeCapture95=(int(np.count_nonzero(high&mf))/mn if mn else math.nan),
               FalseOpenCapture95=(int(np.count_nonzero(high&fo))/fn if fn else math.nan))
    return out

def structural_values(grid,mean,var,gt):
    c=gate._structural_context(grid,gt); ratio=int(c["ratio"])
    mh=np.repeat(np.repeat(mean,ratio,0),ratio,1)[c["src_slice"]]
    vh=np.repeat(np.repeat(var,ratio,0),ratio,1)[c["src_slice"]]
    valid=c["valid"]
    return vh[valid],np.asarray(c["truth_occupied"],dtype=bool),mh[valid],c,vh,mh

def later_values(grid,mean,var,index,table,grids):
    rr,cc=np.nonzero(grid.data<0)
    labels,_,_,_=gate.first_later_observation_targets(grid,rr,cc,index,table,grids)
    keep=labels>=0
    return var[rr[keep],cc[keep]],labels[keep]>0,mean[rr[keep],cc[keep]]

def make_panel(run_id,did,grid,var,up95,mean,gt):
    vals,truth,pred,c,vh,mh=structural_values(grid,mean,var,gt)
    valid=c["valid"]; tg=np.zeros(valid.shape,bool); tg[valid]=truth
    pg=np.zeros(valid.shape,bool); pg[valid]=pred>=0.5
    mf=valid&(~tg)&pg; fo=valid&tg&(~pg); err=mf|fo
    ratio=int(c["ratio"])
    unknown=np.repeat(np.repeat(grid.data<0,ratio,0),ratio,1)[c["src_slice"]]
    high=np.repeat(np.repeat((grid.data<0)&(var>=up95),ratio,0),ratio,1)[c["src_slice"]]
    fig,ax=plt.subplots(1,5,figsize=(18,4))
    ax[0].imshow(unknown,origin="lower"); ax[0].set_title("Unknown mask")
    im=ax[1].imshow(vh,origin="lower"); ax[1].set_title("MapEx variance"); fig.colorbar(im,ax=ax[1],fraction=0.046)
    ax[2].imshow(high,origin="lower"); ax[2].set_title("Primary U >= p95")
    ax[3].imshow(vh,origin="lower"); ax[3].imshow(np.ma.masked_where(~err,err),origin="lower",alpha=.55); ax[3].set_title("Structural errors")
    cat=np.zeros(valid.shape,int); cat[mf]=1; cat[fo]=2
    ax[4].imshow(cat,origin="lower"); ax[4].set_title("1 MissedFree / 2 FalseOpen")
    for a in ax: a.set_xticks([]); a.set_yticks([])
    fig.suptitle(f"MX024 {run_id} Oracle-4 decision {did}"); fig.tight_layout()
    p=FIG/f"MX024_{run_id}_ORACLE4_d{did:03d}.svg"; fig.savefig(p); plt.close(fig); return p

def main():
    OUT.mkdir(parents=True,exist_ok=True); FIG.mkdir(parents=True,exist_ok=True)
    gt=gate.load_structural_gt(GT_PATH)
    shared={(r["run_id"],int(r["decision_id"])):r for r in read_csv(SHARED)}
    per=[]; parity=[]; q2=[]; panel_input={}
    for run_id in RUNS:
        run=EXP/run_id
        table=sorted(read_csv(run/"decisions.csv"),key=lambda r:int(r["decision_id"]))
        grids=[gate.load_raw_grid(resolve(run,r["raw_map"])) for r in table]
        legacy={int(r["decision_id"]):r for r in read_csv(run/"early_stopping_analysis.csv")}
        n=len(table)
        for index,row in enumerate(table,1):
            did=int(row["decision_id"]); dp=(did-ORACLE[run_id])/(n-1)
            band="ANCHOR" if did==ORACLE[run_id] else ("PRE" if -0.10<=dp<0 else ("POST" if 0<dp<=0.10 else ""))
            paths={k:resolve(run,row[k]) for k in ("raw_map","g1_map","g2_map","g3_map","mean_map","variance_map")}
            pr={"run_id":run_id,"decision_id":did,"raw_identity":row["raw_map"],"g1_identity":row["g1_map"],
                "g2_identity":row["g2_map"],"g3_identity":row["g3_map"],"variance_identity":row["variance_map"],
                "raw_sha256":digest(paths["raw_map"]),"g1_sha256":digest(paths["g1_map"]),
                "g2_sha256":digest(paths["g2_map"]),"g3_sha256":digest(paths["g3_map"]),
                "variance_sha256":digest(paths["variance_map"]),
                "variance_estimator":"ddof=1 / correction=1 / denominator=2",
                "variance_parity_pass":0,"primary_Q1_evaluable":0,"q2_mean_evaluable":0,"reason":""}
            dr={"run_id":run_id,"decision_id":did,"decision_count":n,"progress":(did-1)/(n-1),"delta_p":dp,"band":band,
                "in_W10":int(abs(dp)<=0.10+1e-12),"in_W20":int(abs(dp)<=0.20+1e-12),
                "primary_Q1_evaluable":0,"reason":"","MAPEX_U_mean":math.nan,"MAPEX_U_p95":math.nan,
                "MAPEX_U_disagreement":math.nan,"fraction_split_vote_cells":math.nan,"MAPEX_UNKNOWN_VALID_count":0}
            mean=None
            try:
                raw,rm=load_raw(paths["raw_map"],did); crop=None; members=[]
                for key,member in (("g1_map","G1"),("g2_map","G2"),("g3_map","G3")):
                    a,c,m,shape=load_pred(paths[key],did,member,raw,rm,crop)
                    if crop is None: crop=c
                    members.append(a)
                saved,c,m,shape=load_pred(paths["variance_map"],did,"variance",raw,rm,crop)
                stack=np.stack(members,axis=0); recomputed=np.var(stack,axis=0,ddof=1); recomputed[raw>=0]=0
                diff=np.abs(np.asarray(saved,float)-np.asarray(recomputed,float))
                ok=bool(np.allclose(saved,recomputed,rtol=1e-5,atol=1e-7,equal_nan=False))
                pr.update(source_height=crop[0],source_width=crop[1],pad_top=crop[2],pad_left=crop[3],
                          source_stamp_s=rm["source_stamp_s"],source_map_stamp_s=m["source_map_stamp_s"],
                          resolution_check=1,origin_check=1,crop_shape_pass=1,finite_range_pass=1,
                          saved_finite_count=int(np.count_nonzero(np.isfinite(saved))),
                          recomputed_finite_count=int(np.count_nonzero(np.isfinite(recomputed))),
                          variance_parity_pass=int(ok),variance_max_abs_diff=float(np.max(diff)),
                          variance_mean_abs_diff=float(np.mean(diff)))
                if not ok: raise RuntimeError("VARIANCE_PARITY_FAIL")
                unknown=raw<0; un=int(np.count_nonzero(unknown))
                if un==0:
                    dr["reason"]="EMPTY_MAPEX_UNKNOWN_VALID_DOMAIN"; pr["reason"]=dr["reason"]
                else:
                    vals=np.asarray(saved[unknown],float); votes=np.sum(stack[:,unknown]<0.5,axis=0)
                    udis=float(np.mean(np.minimum(votes,3-votes)/3))
                    dr.update(primary_Q1_evaluable=1,MAPEX_U_mean=float(np.mean(vals)),MAPEX_U_p95=quant(vals,0.95),
                              MAPEX_U_disagreement=udis,fraction_split_vote_cells=3*udis,MAPEX_UNKNOWN_VALID_count=un)
                    pr["primary_Q1_evaluable"]=1
                lr=legacy.get(did,{})
                dr["legacy_unknown_mean_variance"]=finite(lr.get("unknown_mean_variance"))
                dr["legacy_unknown_variance_p95"]=finite(lr.get("unknown_variance_p95"))
                dr["legacy_mean_minus_direct"]=dr["legacy_unknown_mean_variance"]-dr["MAPEX_U_mean"] if math.isfinite(dr["legacy_unknown_mean_variance"]) and math.isfinite(dr["MAPEX_U_mean"]) else math.nan
                dr["legacy_p95_minus_direct"]=dr["legacy_unknown_variance_p95"]-dr["MAPEX_U_p95"] if math.isfinite(dr["legacy_unknown_variance_p95"]) and math.isfinite(dr["MAPEX_U_p95"]) else math.nan
                sr=shared.get((run_id,did),{})
                dr["R_union_U_mean_secondary"]=finite(sr.get("U_mean")); dr["R_union_U_p95_secondary"]=finite(sr.get("U_p95"))
                dr["R_union_U_disagreement_secondary"]=finite(sr.get("U_disagreement")); dr["R_union_evaluable_secondary"]=sr.get("u_r_union_evaluable","")
                try:
                    mean,mc,mm,ms=load_pred(paths["mean_map"],did,"mean",raw,rm,crop)
                    pr.update(q2_mean_evaluable=1,mean_identity=row["mean_map"],mean_sha256=digest(paths["mean_map"]),mean_reason="")
                except Exception as e:
                    pr["mean_reason"]=f"{type(e).__name__}:{e}"
                if not pr["primary_Q1_evaluable"]:
                    reason=dr["reason"] or "PRIMARY_Q1_NOT_EVALUABLE"
                    q2.append(q2_record(run_id,did,n,dp,band,"STRUCTURAL_GT",[],[],[],gt.resolution**2,reason))
                    q2.append(q2_record(run_id,did,n,dp,band,"LATER_OBSERVED",[],[],[],float(rm["resolution"])**2,reason))
                elif mean is None:
                    reason=pr["mean_reason"]
                    q2.append(q2_record(run_id,did,n,dp,band,"STRUCTURAL_GT",[],[],[],gt.resolution**2,reason))
                    q2.append(q2_record(run_id,did,n,dp,band,"LATER_OBSERVED",[],[],[],float(rm["resolution"])**2,reason))
                else:
                    sv,st,sp,_,_,_=structural_values(grids[index-1],mean,saved,gt)
                    q2.append(q2_record(run_id,did,n,dp,band,"STRUCTURAL_GT",sv,st,sp,gt.resolution**2))
                    lv,lt,lp=later_values(grids[index-1],mean,saved,index,table,grids)
                    q2.append(q2_record(run_id,did,n,dp,band,"LATER_OBSERVED",lv,lt,lp,float(rm["resolution"])**2))
                if did==ORACLE[run_id] and pr["primary_Q1_evaluable"] and mean is not None:
                    panel_input[run_id]=(did,grids[index-1],saved.copy(),dr["MAPEX_U_p95"],mean.copy())
            except Exception as e:
                reason="VARIANCE_PARITY_FAIL" if "VARIANCE_PARITY_FAIL" in str(e) else f"SOURCE_VALIDATION_FAIL:{type(e).__name__}:{e}"
                pr["reason"]=reason; dr["reason"]=reason
                sr=shared.get((run_id,did),{})
                dr["R_union_U_mean_secondary"]=finite(sr.get("U_mean")); dr["R_union_U_p95_secondary"]=finite(sr.get("U_p95"))
                dr["R_union_U_disagreement_secondary"]=finite(sr.get("U_disagreement")); dr["R_union_evaluable_secondary"]=sr.get("u_r_union_evaluable","")
                q2.append(q2_record(run_id,did,n,dp,band,"STRUCTURAL_GT",[],[],[],gt.resolution**2,reason))
                q2.append(q2_record(run_id,did,n,dp,band,"LATER_OBSERVED",[],[],[],0.01,reason))
            parity.append(pr); per.append(dr)

    if len(per)!=365 or len(parity)!=365 or len(q2)!=730: raise RuntimeError("ROW_MEMBERSHIP_FAIL")
    write_csv(OUT/"MX024_U_PER_DECISION.csv",per); write_csv(OUT/"MX024_U_SOURCE_PARITY.csv",parity); write_csv(OUT/"MX024_U_ERROR_ALIGNMENT_PER_DECISION.csv",q2)

    perrun=[]
    for metric in ("MAPEX_U_mean","MAPEX_U_p95","MAPEX_U_disagreement"):
        for run_id in RUNS:
            rows=[r for r in per if r["run_id"]==run_id and r["in_W10"] and r["primary_Q1_evaluable"]]
            pre=[float(r[metric]) for r in rows if r["band"]=="PRE" and math.isfinite(float(r[metric]))]
            anc=[float(r[metric]) for r in rows if r["band"]=="ANCHOR" and math.isfinite(float(r[metric]))]
            post=[float(r[metric]) for r in rows if r["band"]=="POST" and math.isfinite(float(r[metric]))]
            p,a,s=median(pre),(anc[0] if len(anc)==1 else math.nan),median(post)
            ap=a-p if math.isfinite(a) and math.isfinite(p) else math.nan; sp=s-p if math.isfinite(s) and math.isfinite(p) else math.nan
            direction="DECREASING" if math.isfinite(sp) and sp<0 else ("INCREASING" if math.isfinite(sp) and sp>0 else ("EXACT_ZERO" if math.isfinite(sp) else "NA"))
            vals=[float(r[metric]) for r in rows if math.isfinite(float(r[metric]))]
            perrun.append({"run_id":run_id,"metric":metric,"PRE_median":p,"ANCHOR":a,"POST_median":s,
                           "anchor_minus_PRE":ap,"POST_minus_PRE":sp,"W10_min":min(vals) if vals else math.nan,
                           "W10_max":max(vals) if vals else math.nan,"PRE_n":len(pre),"ANCHOR_n":len(anc),"POST_n":len(post),
                           "W10_valid_n":len(vals),"direction_POST_vs_PRE":direction})
    write_csv(OUT/"MX024_U_PER_RUN.csv",perrun)

    macro=[]; synth=[]
    for metric in ("MAPEX_U_mean","MAPEX_U_p95","MAPEX_U_disagreement"):
        rows=[r for r in perrun if r["metric"]==metric]
        for field in ("PRE_median","ANCHOR","POST_median","anchor_minus_PRE","POST_minus_PRE"):
            vals=[float(r[field]) for r in rows if math.isfinite(float(r[field]))]
            macro.append({"section":"Q1","domain":"MAPEX_UNKNOWN_VALID","metric":metric+":"+field,
                          "run_macro_median":median(vals),"run_macro_q25":quant(vals,.25),"run_macro_q75":quant(vals,.75),"contributor_n":len(vals)})
        dirs=[r["direction_POST_vs_PRE"] for r in rows]
        synth.append({"section":"A" if metric!="MAPEX_U_disagreement" else "B","domain":"MAPEX_UNKNOWN_VALID","metric":metric,
                      "PRE_run_macro_median":median(float(r["PRE_median"]) for r in rows if math.isfinite(float(r["PRE_median"]))),
                      "ANCHOR_run_macro_median":median(float(r["ANCHOR"]) for r in rows if math.isfinite(float(r["ANCHOR"]))),
                      "POST_run_macro_median":median(float(r["POST_median"]) for r in rows if math.isfinite(float(r["POST_median"]))),
                      "decreasing_runs":dirs.count("DECREASING"),"increasing_runs":dirs.count("INCREASING"),
                      "exact_zero_runs":dirs.count("EXACT_ZERO"),"direction_contributor_n":sum(d!="NA" for d in dirs)})
    q2metrics=("HighUErrorRate","BaselineErrorRate","ErrorEnrichment95","ErrorCapture95","MissedFreeCapture95","FalseOpenCapture95")
    for domain in ("STRUCTURAL_GT","LATER_OBSERVED"):
        wr=[r for r in q2 if r["domain"]==domain and r["evaluable"] and abs(float(r["delta_p"]))<=.10+1e-12]
        for metric in q2metrics:
            rv=[]
            for run_id in RUNS:
                vals=[float(r[metric]) for r in wr if r["run_id"]==run_id and metric in r and math.isfinite(float(r[metric]))]
                if vals: rv.append(median(vals))
            macro.append({"section":"Q2_W10","domain":domain,"metric":metric,"run_macro_median":median(rv),
                          "run_macro_q25":quant(rv,.25),"run_macro_q75":quant(rv,.75),"contributor_n":len(rv)})
            synth.append({"section":"C","domain":domain,"metric":metric,"W10_run_macro_median":median(rv),
                          "W10_run_macro_q25":quant(rv,.25),"W10_run_macro_q75":quant(rv,.75),"contributor_n":len(rv)})
    write_csv(OUT/"MX024_U_RUN_MACRO.csv",macro); write_csv(OUT/"MX024_U_ORACLE_SYNTHESIS.csv",synth)

    panels=[]
    for run_id in RUNS:
        if run_id not in panel_input: raise RuntimeError(run_id+"_ORACLE_PANEL_INPUT_NA")
        panels.append(make_panel(run_id,*panel_input[run_id],gt))
    if len(panels)!=10: raise RuntimeError("PANEL_COUNT_FAIL")

    dictionary={"method_revision":METHOD_SHA,"technical_base":BASE_SHA,
      "variance_semantics":{"ensemble_n":3,"ddof":1,"correction":1,"denominator":2},
      "q95_semantics":"numpy.quantile method=linear; HighU uses >= and includes ties",
      "truth_domains":["STRUCTURAL_GT","LATER_OBSERVED"],
      "metrics":[
        {"name":"MAPEX_U_mean","role":"PRIMARY_Q1_PROJECT_SUMMARY","domain":"MAPEX_UNKNOWN_VALID"},
        {"name":"MAPEX_U_p95","role":"PRIMARY_Q1_PROJECT_SUMMARY","domain":"MAPEX_UNKNOWN_VALID"},
        {"name":"MAPEX_U_disagreement","role":"SECONDARY_DIAGNOSTIC","domain":"MAPEX_UNKNOWN_VALID"},
        {"name":"HighUErrorRate","role":"Q2_DIAGNOSTIC","domain":"MATCHED_TRUTH_DOMAIN"},
        {"name":"BaselineErrorRate","role":"Q2_DIAGNOSTIC","domain":"MATCHED_TRUTH_DOMAIN"},
        {"name":"ErrorEnrichment95","role":"Q2_DIAGNOSTIC","domain":"MATCHED_TRUTH_DOMAIN"},
        {"name":"ErrorCapture95","role":"Q2_DIAGNOSTIC","domain":"MATCHED_TRUTH_DOMAIN"},
        {"name":"MissedFreeCapture95","role":"Q2_DIAGNOSTIC","domain":"MATCHED_TRUTH_DOMAIN"},
        {"name":"FalseOpenCapture95","role":"Q2_DIAGNOSTIC","domain":"MATCHED_TRUTH_DOMAIN"},
        {"name":"U_p95/U_mean/U_disagreement","role":"SECONDARY_R_UNION_CONTEXT","domain":"D1_R_union"}],
      "forbidden":["LaMa rerun","new simulation","prediction regeneration","retuning","winner/composite","online STOP"]}
    (OUT/"MX024_U_METRIC_DICTIONARY.json").write_text(json.dumps(dictionary,indent=2,sort_keys=True)+"\n")

    q1={r["metric"]:r for r in synth if r["section"] in ("A","B")}
    q2s={(r["domain"],r["metric"]):r for r in synth if r["section"]=="C"}
    pp=sum(int(r["variance_parity_pass"]) for r in parity); qe=sum(int(r["primary_Q1_evaluable"]) for r in parity); me=sum(int(r["q2_mean_evaluable"]) for r in parity)
    pindex={(r["run_id"],int(r["decision_id"])):r for r in parity}
    lines=["# MX024 Analyst Report — MapEx-original-first U around Oracle-4","","Method revision: "+METHOD_SHA,"Technical base: "+BASE_SHA,"","## A — MapEx U near Oracle-4"]
    for m in ("MAPEX_U_mean","MAPEX_U_p95"):
        r=q1[m]; lines.append(f"- {m}: PRE/ANCHOR/POST run-macro medians = {r['PRE_run_macro_median']:.9g} / {r['ANCHOR_run_macro_median']:.9g} / {r['POST_run_macro_median']:.9g}; POST-vs-PRE directions = {r['decreasing_runs']} decreasing, {r['increasing_runs']} increasing, {r['exact_zero_runs']} exact-zero of {r['direction_contributor_n']}.")
    lines+=["- Direction is descriptive only; no usefulness threshold or STOP rule is inferred.","","## B — Ensemble agreement"]
    r=q1["MAPEX_U_disagreement"]; lines.append(f"- MAPEX_U_disagreement PRE/ANCHOR/POST = {r['PRE_run_macro_median']:.9g} / {r['ANCHOR_run_macro_median']:.9g} / {r['POST_run_macro_median']:.9g}; directions = {r['decreasing_runs']} decreasing, {r['increasing_runs']} increasing, {r['exact_zero_runs']} exact-zero.")
    lines+=["- Disagreement is secondary diagnostic, not original MapEx U.","","## C — Does high U identify prediction errors?"]
    for d in ("STRUCTURAL_GT","LATER_OBSERVED"):
        e=q2s[(d,"ErrorEnrichment95")]; c=q2s[(d,"ErrorCapture95")]; mf=q2s[(d,"MissedFreeCapture95")]; fo=q2s[(d,"FalseOpenCapture95")]
        lines.append(f"- {d} W10 equal-run medians: ErrorEnrichment95={e['W10_run_macro_median']:.9g} (n={e['contributor_n']}), ErrorCapture95={c['W10_run_macro_median']:.9g} (n={c['contributor_n']}), MissedFreeCapture95={mf['W10_run_macro_median']:.9g} (n={mf['contributor_n']}), FalseOpenCapture95={fo['W10_run_macro_median']:.9g} (n={fo['contributor_n']}).")
    lines+=["- Structural-GT and later-observed denominators remain separate.","","## D — Where are high-U cells?",f"- Generated {len(panels)}/10 deterministic Oracle-4 spatial panels.","","## E — Planning context","- No new topology/path criterion was introduced. D1 R_union U remains secondary per-decision context.","","## F — Bounded synthesis / integrity",f"- Saved variance parity passed {pp}/365; primary Q1 evaluable {qe}/365; mean/Q2 source evaluable {me}/365.",f"- mpx_003 d18–20 primary Q1: "+", ".join(f"d{d}={'PASS' if pindex[('mpx_003',d)]['primary_Q1_evaluable'] else 'NA'}" for d in (18,19,20)),"- No LaMa rerun, new simulation, prediction regeneration, retuning, winner/composite, causal claim, or online STOP.","- Scope: New Room development cohort around retrospective Oracle-4 only."]
    (OUT/"MX024_ANALYST_REPORT.md").write_text("\n".join(lines)+"\n")

    artifacts=[OUT/"MX024_U_METRIC_DICTIONARY.json",OUT/"MX024_U_PER_DECISION.csv",OUT/"MX024_U_PER_RUN.csv",
               OUT/"MX024_U_ERROR_ALIGNMENT_PER_DECISION.csv",OUT/"MX024_U_RUN_MACRO.csv",OUT/"MX024_U_ORACLE_SYNTHESIS.csv",
               OUT/"MX024_U_SOURCE_PARITY.csv",OUT/"MX024_ANALYST_REPORT.md",*panels]
    manifest={"schema":"mx024_original_mapex_u_execution_v1","method_revision":METHOD_SHA,"technical_base":BASE_SHA,
              "status":"COMPLETE_PENDING_INDEPENDENT_QA","cohort":{"runs":RUNS,"decisions":365,"oracle_anchors":ORACLE},
              "variance_semantics":{"n":3,"ddof":1,"correction":1,"denominator":2,"rtol":1e-5,"atol":1e-7},
              "guards":{"model_rerun":False,"simulation_rerun":False,"prediction_regeneration":False,"retuning":False,
                        "winner_or_composite":False,"online_stop":False,"primary_u_requires_robot_source":False,"r_union_is_primary":False},
              "integrity":{"variance_parity_pass_count":pp,"primary_q1_evaluable_count":qe,"q2_mean_evaluable_count":me,
                           "oracle_panel_count":len(panels),"mpx_003_d18_20_primary_q1":{str(d):int(pindex[("mpx_003",d)]["primary_Q1_evaluable"]) for d in (18,19,20)}},
              "artifacts":[{"path":str(p.relative_to(OUT)),"sha256":digest(p),"size":p.stat().st_size} for p in artifacts]}
    (OUT/"MX024_U_ARTIFACT_MANIFEST.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n")
    print(json.dumps(manifest["integrity"],indent=2,sort_keys=True))

if __name__=="__main__": main()
