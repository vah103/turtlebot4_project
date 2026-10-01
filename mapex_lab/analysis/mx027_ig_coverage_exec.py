#!/usr/bin/env python3
from __future__ import annotations
import csv, hashlib, json, math
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

METHOD_SHA="6925611773ced0722ffc8bf43279084dd01fad75"
BASE_SHA="4899ee95c85640965befeaf98c20f156c0d3181a"
MX026_SHA="79ee3ba8e2f82a55cd6e2d68fdb3d2ed08569e55"
MX026_BLOB="8e73f4441b1c852f90d801c20f993216bbd1e78a"
LAB=Path(__file__).resolve().parents[1]
EXP=LAB/"experiments"/"mapex"
MX026=LAB/"analysis"/"mx027_inputs"/"MX026_PUR_PER_DECISION_ACCEPTED_R2.csv"
OUT=LAB/"analysis"/"mx027_exec_results"; FIG=OUT/"figures"
RUNS=[f"mpx_{i:03d}" for i in range(1,11)]
EXPECTED_DECISION_BLOBS={
"mpx_001":"07d1894548768a163a56ac3ba21c00a5cb800e05","mpx_002":"4e7e2ce94ccf2f64d11057efea899161702c1977",
"mpx_003":"62f8d7c150d203a34b77bdfc6cb5e6b96224da73","mpx_004":"529b70c58c776ae00e87e3997240561a8b5cc712",
"mpx_005":"b0a70584a691deb9f0f03f216aeac75ace49b0f9","mpx_006":"e5cdf2e65164368ff47f4283a5a05f7072bf73c7",
"mpx_007":"5c49fa764ba23f13b31922b50528ca9adc9ba269","mpx_008":"6dc7c1311615f47eb788db039f3140d25d4f1fd0",
"mpx_009":"f3949c07938300f41ef7d00d5b54d0ea6871ecce","mpx_010":"0cc77a1fd939a44db146de31a447334c4dac40bb"}
EXPECTED_NO_SELECTION={"mpx_001":7,"mpx_002":9,"mpx_003":9,"mpx_004":0,"mpx_005":8,"mpx_006":0,"mpx_007":0,"mpx_008":8,"mpx_009":9,"mpx_010":9}
BINS=[("B00_10",0,.1,0),("B10_20",.1,.2,0),("B20_30",.2,.3,0),("B30_40",.3,.4,0),("B40_50",.4,.5,0),("B50_60",.5,.6,0),("B60_70",.6,.7,0),("B70_80",.7,.8,0),("B80_90",.8,.9,0),("B90_100",.9,1,1)]
METRICS=["IG_selected","IG_visible_unknown_cells","IG_density","IG_policy_score","KnownArea_m2","DeltaKnownArea_m2","KnownAreaRate_m2_s"]

def read_csv(p):
    with Path(p).open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write_csv(p,rows):
    rows=list(rows);Path(p).parent.mkdir(parents=True,exist_ok=True);fields=[]
    for r in rows:
        for k in r:
            if k not in fields:fields.append(k)
    with Path(p).open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def sha256(p):
    h=hashlib.sha256()
    with Path(p).open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):h.update(b)
    return h.hexdigest()
def git_blob_sha(p):
    b=Path(p).read_bytes();return hashlib.sha1(b"blob "+str(len(b)).encode()+b"\0"+b).hexdigest()
def fin(v):
    try:x=float(v)
    except:return math.nan
    return x if math.isfinite(x) else math.nan
def exact_int(v):
    x=float(v)
    if not math.isfinite(x) or int(x)!=x:raise ValueError
    return int(x)
def quant(vals,q):
    a=np.asarray([float(v) for v in vals if math.isfinite(float(v))],dtype=np.float64)
    return float(np.quantile(a,q,method="linear")) if a.size else math.nan
def med(vals):return quant(vals,.5)
def pbin(p):
    for n,lo,hi,last in BINS:
        if (lo<=p<=hi) if last else (lo<=p<hi):return n
    raise RuntimeError("PROGRESS_BIN_FAIL")
def ranks(v):
    a=np.asarray(v,float);order=np.argsort(a,kind="mergesort");r=np.empty(len(a),float);i=0
    while i<len(a):
        j=i+1
        while j<len(a) and a[order[j]]==a[order[i]]:j+=1
        r[order[i:j]]=((i+1)+j)/2.0;i=j
    return r
def spearman(metric,rows):
    pairs=[(fin(r["normalized_progress"]),fin(r.get(metric))) for r in rows]
    pairs=[x for x in pairs if math.isfinite(x[0]) and math.isfinite(x[1])]
    n=len(pairs);out={"rho":math.nan,"rho_reason":"","paired_finite_n":n,"excluded_nonfinite_n":len(rows)-n}
    if n<5:out["rho_reason"]="INSUFFICIENT_FINITE_PAIRED_SUPPORT_LT5";return out
    x=np.asarray([a for a,b in pairs]);y=np.asarray([b for a,b in pairs]);xc=np.all(x==x[0]);yc=np.all(y==y[0])
    if xc and yc:out["rho_reason"]="CONSTANT_BOTH_VECTORS";return out
    if xc:out["rho_reason"]="CONSTANT_X_VECTOR";return out
    if yc:out["rho_reason"]="CONSTANT_Y_VECTOR";return out
    out["rho"]=float(np.corrcoef(ranks(x),ranks(y))[0,1]);return out
def blank(v):return v is None or str(v).strip()==""
def safe_run_path(run,rel):
    p=(run/str(rel)).resolve()
    if run.resolve() not in p.parents:raise ValueError("PATH_ESCAPE")
    return p

def inspect_raw(run,row,did):
    try:
        p=safe_run_path(run,row["raw_map"])
        if not p.is_file():return {"ok":False,"reason":"RAW_MAP_MISSING_OR_UNREADABLE","path":str(row["raw_map"])}
        with np.load(p,allow_pickle=False) as z:
            if "data" not in z.files:return {"ok":False,"reason":"RAW_MAP_DATA_INVALID","path":str(row["raw_map"]),"sha256":sha256(p)}
            data=np.asarray(z["data"],dtype=np.int16)
            if data.ndim!=2:return {"ok":False,"reason":"RAW_MAP_DATA_INVALID","path":str(row["raw_map"]),"sha256":sha256(p)}
            if "height" in z.files and exact_int(np.asarray(z["height"]).item())!=data.shape[0]:return {"ok":False,"reason":"RAW_MAP_DATA_INVALID","path":str(row["raw_map"]),"sha256":sha256(p)}
            if "width" in z.files and exact_int(np.asarray(z["width"]).item())!=data.shape[1]:return {"ok":False,"reason":"RAW_MAP_DATA_INVALID","path":str(row["raw_map"]),"sha256":sha256(p)}
            if "resolution" not in z.files:return {"ok":False,"reason":"RAW_MAP_RESOLUTION_INVALID","path":str(row["raw_map"]),"sha256":sha256(p)}
            res=float(np.asarray(z["resolution"]).item())
            if not math.isfinite(res) or res<=0:return {"ok":False,"reason":"RAW_MAP_RESOLUTION_INVALID","path":str(row["raw_map"]),"sha256":sha256(p)}
        return {"ok":True,"reason":"","path":str(row["raw_map"]),"sha256":sha256(p),"resolution":res,"known_count":int(np.count_nonzero(data>=0)),"shape":f"{data.shape[0]}x{data.shape[1]}"}
    except Exception:return {"ok":False,"reason":"RAW_MAP_MISSING_OR_UNREADABLE","path":str(row.get("raw_map",""))}

def inspect_ig(run,row,did):
    base={"IG_evaluable":0,"IG_reason":"","IG_selected":math.nan,"IG_visible_unknown_cells":math.nan,
          "IG_density":math.nan,"IG_density_reason":"","selected_distance_m":math.nan,
          "IG_policy_score":math.nan,"IG_policy_score_reason":"","IG_score_parity_pass":"","IG_score_abs_diff":math.nan,
          "candidate_table_path":"","candidate_table_sha256":"","candidate_lookup_attempted":0,"policy_decision_id":""}
    pr=row.get("mapex_policy_decision_id","");outcome=str(row.get("outcome","")).strip()
    if blank(pr):
        try:ct=exact_int(row.get("candidate_total",""))
        except Exception:
            base["IG_reason"]="BLANK_POLICY_ID_INCONSISTENT_RUNTIME_STATE";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
        if outcome=="no_selection" and ct==0:
            base["IG_reason"]="NO_RUNTIME_SELECTED_FRONTIER";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
        base["IG_reason"]="BLANK_POLICY_ID_INCONSISTENT_RUNTIME_STATE";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    try:p=exact_int(pr)
    except Exception:
        base["IG_reason"]="POLICY_DECISION_ID_INVALID";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    base["policy_decision_id"]=p;base["candidate_lookup_attempted"]=1
    rel=f"decisions/policy_decision_{p:06d}/candidates.csv";base["candidate_table_path"]=rel;path=run/rel
    try:rows=read_csv(path);base["candidate_table_sha256"]=sha256(path)
    except Exception:
        base["IG_reason"]="CANDIDATE_TABLE_MISSING_OR_UNREADABLE";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    req={"decision_id","mapex_policy_decision_id","policy_decision_id","selected","status","information_gain","visible_unknown_cells","distance_m","score"}
    if not rows or not req.issubset(rows[0].keys()):
        base["IG_reason"]="CANDIDATE_TABLE_SCHEMA_INVALID";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    for r in rows:
        try:ids=(exact_int(r["decision_id"]),exact_int(r["mapex_policy_decision_id"]),exact_int(r["policy_decision_id"]))
        except Exception:
            base["IG_reason"]="CANDIDATE_ROW_IDENTITY_MISMATCH";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
        if ids!=(did,p,p):
            base["IG_reason"]="CANDIDATE_ROW_IDENTITY_MISMATCH";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    try:selected=[r for r in rows if exact_int(r["selected"])==1]
    except Exception:selected=[]
    if len(selected)!=1:
        base["IG_reason"]="SELECTED_FRONTIER_CARDINALITY_NE_1";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    s=selected[0]
    if str(s["status"]).strip()!="selected":
        base["IG_reason"]="SELECTED_ROW_STATUS_MISMATCH";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    try:
        ig=float(s["information_gain"]);vu=exact_int(s["visible_unknown_cells"]);dist=float(s["distance_m"]);score=float(s["score"])
        if not all(map(math.isfinite,[ig,dist,score])) or vu<0 or dist<0:raise ValueError
    except Exception:
        base["IG_reason"]="SELECTED_ROW_VALUE_INVALID";base["IG_density_reason"]=base["IG_reason"];base["IG_policy_score_reason"]=base["IG_reason"];return base
    base.update(IG_evaluable=1,IG_reason="",IG_selected=ig,IG_visible_unknown_cells=vu,selected_distance_m=dist)
    if vu>0:base["IG_density"]=ig/vu;base["IG_density_reason"]=""
    else:base["IG_density_reason"]="ZERO_VISIBLE_UNKNOWN_DENOMINATOR"
    recomputed=ig/max(dist,1e-6);diff=abs(score-recomputed);ok=diff<=1e-8+1e-9*abs(recomputed)
    base["IG_score_parity_pass"]=int(ok);base["IG_score_abs_diff"]=diff
    if ok:base["IG_policy_score"]=score;base["IG_policy_score_reason"]=""
    else:base["IG_policy_score_reason"]="IG_POLICY_SCORE_PARITY_FAIL"
    return base

def plot_run(rid,rows):
    x=[float(r["normalized_progress"]) for r in rows];oracle=float([r["normalized_progress"] for r in rows if str(r["Oracle4_marker"])=="1"][0])
    fig,ax=plt.subplots(4,1,figsize=(12,14),sharex=True)
    ax[0].plot(x,[fin(r["IG_selected"]) for r in rows],label="IG_selected");ax[0].plot(x,[fin(r["IG_visible_unknown_cells"]) for r in rows],label="visible_unknown_cells")
    ax[0].axvline(oracle,ls="--");ax[0].legend(fontsize=8);ax[0].set_ylabel("IG / cells")
    ax[1].plot(x,[fin(r["IG_density"]) for r in rows],label="IG_density");b=ax[1].twinx();b.plot(x,[fin(r["IG_policy_score"]) for r in rows],label="policy_score")
    ax[1].axvline(oracle,ls="--");h1,l1=ax[1].get_legend_handles_labels();h2,l2=b.get_legend_handles_labels();ax[1].legend(h1+h2,l1+l2,fontsize=8)
    ax[2].plot(x,[fin(r["KnownArea_m2"]) for r in rows],label="KnownArea_m2");ax[2].axvline(oracle,ls="--");ax[2].legend(fontsize=8)
    ax[3].plot(x,[fin(r["DeltaKnownArea_m2"]) for r in rows],label="DeltaKnownArea_m2");b2=ax[3].twinx();b2.plot(x,[fin(r["KnownAreaRate_m2_s"]) for r in rows],label="KnownAreaRate_m2_s")
    ax[3].axhline(0,ls=":");ax[3].axvline(oracle,ls="--");h1,l1=ax[3].get_legend_handles_labels();h2,l2=b2.get_legend_handles_labels();ax[3].legend(h1+h2,l1+l2,fontsize=8);ax[3].set_xlabel("normalized progress")
    fig.suptitle("MX027 "+rid+" full trajectory");fig.tight_layout();p=FIG/f"MX027_{rid}_FULL_TRAJECTORY.svg";fig.savefig(p);plt.close(fig);return p

def main():
    OUT.mkdir(parents=True,exist_ok=True);FIG.mkdir(parents=True,exist_ok=True)
    mx=read_csv(MX026);mxi={(r["run_id"],int(r["decision_id"])):r for r in mx}
    if len(mx)!=365 or len(mxi)!=365:raise RuntimeError("MX026_FRAME_MEMBERSHIP_FAIL")
    per=[];source=[];tables={}
    # validate exact decisions tables first
    for rid in RUNS:
        p=EXP/rid/"decisions.csv"
        if not p.is_file() or git_blob_sha(p)!=EXPECTED_DECISION_BLOBS[rid]:raise RuntimeError("DECISIONS_TABLE_PIN_FAIL:"+rid)
        rows=read_csv(p);req={"decision_id","mapex_policy_decision_id","time_s","candidate_total","outcome","raw_map"}
        if not rows or not req.issubset(rows[0].keys()):raise RuntimeError("DECISIONS_TABLE_SCHEMA_INVALID:"+rid)
        ids=[exact_int(r["decision_id"]) for r in rows]
        if len(ids)!=len(set(ids)):raise RuntimeError("DECISION_ROW_DUPLICATE:"+rid)
        tables[rid]={i:r for i,r in zip(ids,rows)}
    # raw preinspection + resolution consistency
    rawinfo={}
    for rid in RUNS:
        run=EXP/rid
        for did in [int(r["decision_id"]) for r in mx if r["run_id"]==rid]:
            if did not in tables[rid]:raise RuntimeError("DECISION_ROW_MISSING:"+rid+":"+str(did))
            rawinfo[(rid,did)]=inspect_raw(run,tables[rid][did],did)
        valid_res=[z["resolution"] for k,z in rawinfo.items() if k[0]==rid and z["ok"]]
        inconsistent=len(set(valid_res))>1
        if inconsistent:
            for k,z in rawinfo.items():
                if k[0]==rid:z["run_resolution_inconsistent"]=True
        else:
            for k,z in rawinfo.items():
                if k[0]==rid:z["run_resolution_inconsistent"]=False
    for rid in RUNS:
        run=EXP/rid;rowsmx=sorted([r for r in mx if r["run_id"]==rid],key=lambda r:int(r["decision_index"]))
        N=len(rowsmx);prev=None
        for m in rowsmx:
            did=int(m["decision_id"]);d=tables[rid][did]
            if abs(fin(m["normalized_progress"])-((int(m["decision_index"])-1)/(N-1)))>1e-12:raise RuntimeError("MX026_PROGRESS_FRAME_FAIL")
            o={"run_id":rid,"decision_id":did,"decision_index":m["decision_index"],"decision_count":m["decision_count"],"normalized_progress":m["normalized_progress"],"progress_bin":m["progress_bin"],"Oracle4_marker":m["Oracle4_marker"]}
            ig=inspect_ig(run,d,did);o.update({k:ig[k] for k in ["IG_evaluable","IG_reason","IG_selected","IG_visible_unknown_cells","IG_density","IG_density_reason","selected_distance_m","IG_policy_score","IG_policy_score_reason","IG_score_parity_pass","IG_score_abs_diff"]})
            ri=rawinfo[(rid,did)];cov_ok=ri["ok"] and not ri["run_resolution_inconsistent"]
            o["Coverage_evaluable"]=int(cov_ok);o["Coverage_reason"]="" if cov_ok else ("RAW_MAP_RESOLUTION_INCONSISTENT_WITHIN_RUN" if ri["run_resolution_inconsistent"] else ri["reason"])
            o["KnownArea_m2"]=ri["known_count"]*ri["resolution"]**2 if cov_ok else math.nan
            if prev is None:
                o["DeltaKnownArea_m2"]=math.nan;o["DeltaKnownArea_reason"]="FIRST_DECISION_NO_PREDECESSOR";o["decision_dt_s"]=math.nan;o["KnownAreaRate_m2_s"]=math.nan;o["KnownAreaRate_reason"]="FIRST_DECISION_NO_PREDECESSOR"
            else:
                if cov_ok and prev["Coverage_evaluable"]:
                    delta=o["KnownArea_m2"]-prev["KnownArea_m2"];o["DeltaKnownArea_m2"]=delta;o["DeltaKnownArea_reason"]=""
                else:o["DeltaKnownArea_m2"]=math.nan;o["DeltaKnownArea_reason"]="ADJACENT_KNOWN_AREA_INVALID"
                t=fin(d.get("time_s"));tp=fin(prev["_time_s"]);dt=t-tp if math.isfinite(t) and math.isfinite(tp) else math.nan;o["decision_dt_s"]=dt
                if not math.isfinite(fin(o["DeltaKnownArea_m2"])):o["KnownAreaRate_m2_s"]=math.nan;o["KnownAreaRate_reason"]="ADJACENT_KNOWN_AREA_INVALID"
                elif not math.isfinite(t) or not math.isfinite(tp):o["KnownAreaRate_m2_s"]=math.nan;o["KnownAreaRate_reason"]="INVALID_TIMESTAMP"
                elif dt<=0:o["KnownAreaRate_m2_s"]=math.nan;o["KnownAreaRate_reason"]="NONPOSITIVE_DECISION_DT"
                else:o["KnownAreaRate_m2_s"]=o["DeltaKnownArea_m2"]/dt;o["KnownAreaRate_reason"]=""
            o["_time_s"]=d.get("time_s","");per.append(o)
            source.append({"run_id":rid,"decision_id":did,"decisions_blob":EXPECTED_DECISION_BLOBS[rid],"policy_raw":d.get("mapex_policy_decision_id",""),"outcome":d.get("outcome",""),"candidate_total":d.get("candidate_total",""),"candidate_lookup_attempted":ig["candidate_lookup_attempted"],"candidate_table_path":ig["candidate_table_path"],"candidate_table_sha256":ig["candidate_table_sha256"],"policy_decision_id":ig["policy_decision_id"],"IG_reason":ig["IG_reason"],"raw_map":ri.get("path",""),"raw_map_sha256":ri.get("sha256",""),"raw_resolution":ri.get("resolution",""),"raw_shape":ri.get("shape",""),"coverage_source_reason":o["Coverage_reason"]})
            prev={**o,"_time_s":d.get("time_s","")}
    for r in per:r.pop("_time_s",None)
    if len(per)!=365 or len({(r["run_id"],r["decision_id"]) for r in per})!=365:raise RuntimeError("PER_DECISION_MEMBERSHIP_FAIL")
    no=Counter(r["run_id"] for r in per if r["IG_reason"]=="NO_RUNTIME_SELECTED_FRONTIER")
    if dict(no)!=EXPECTED_NO_SELECTION:raise RuntimeError("NO_SELECTION_COUNT_FAIL:"+repr(dict(no)))
    if any(r["IG_reason"]=="BLANK_POLICY_ID_INCONSISTENT_RUNTIME_STATE" for r in per):raise RuntimeError("UNEXPECTED_INCONSISTENT_BLANK_POLICY")
    if sum(int(r["IG_evaluable"]) for r in per)!=306:raise RuntimeError("IG_EVALUABLE_COUNT_FAIL")
    if sum(int(r["Coverage_evaluable"]) for r in per)!=365:raise RuntimeError("COVERAGE_EVALUABLE_COUNT_FAIL")
    if sum(int(r["candidate_lookup_attempted"]) for r in source)!=306:raise RuntimeError("CANDIDATE_LOOKUP_COUNT_FAIL")
    if any(r["candidate_lookup_attempted"]=="1" and r["IG_reason"] not in ("","IG_POLICY_SCORE_PARITY_FAIL") for r in source):raise RuntimeError("CANDIDATE_SOURCE_FAILURE_PRESENT")
    if any(str(r["IG_score_parity_pass"])=="0" for r in per if int(r["IG_evaluable"])):raise RuntimeError("SCORE_PARITY_FAILURE_PRESENT")
    write_csv(OUT/"MX027_IG_COVERAGE_PER_DECISION.csv",per);write_csv(OUT/"MX027_IG_COVERAGE_SOURCE_PARITY.csv",source)
    byrun={rid:[r for r in per if r["run_id"]==rid] for rid in RUNS}
    igr=[];cvr=[]
    for rid,rows in byrun.items():
        a={"run_id":rid,"decision_count":len(rows),"IG_evaluable_n":sum(int(r["IG_evaluable"]) for r in rows),"IG_no_selection_n":sum(r["IG_reason"]=="NO_RUNTIME_SELECTED_FRONTIER" for r in rows)}
        for m in METRICS[:4]:
            v=[fin(r[m]) for r in rows if math.isfinite(fin(r[m]))];a[m+"_first_valid"]=v[0] if v else math.nan;a[m+"_final_valid"]=v[-1] if v else math.nan;a[m+"_median"]=med(v);a[m+"_valid_n"]=len(v)
        igr.append(a)
        a={"run_id":rid,"decision_count":len(rows),"Coverage_evaluable_n":sum(int(r["Coverage_evaluable"]) for r in rows)}
        for m in METRICS[4:]:
            v=[fin(r[m]) for r in rows if math.isfinite(fin(r[m]))];a[m+"_first_valid"]=v[0] if v else math.nan;a[m+"_final_valid"]=v[-1] if v else math.nan;a[m+"_median"]=med(v);a[m+"_valid_n"]=len(v);a[m+"_negative_n"]=sum(x<0 for x in v)
        cvr.append(a)
    write_csv(OUT/"MX027_IG_PER_RUN.csv",igr);write_csv(OUT/"MX027_COVERAGE_PER_RUN.csv",cvr)
    bins=[];cache={}
    for rid,rows in byrun.items():
        for m in METRICS:
            for bn,_,_,_ in BINS:
                v=[fin(r[m]) for r in rows if r["progress_bin"]==bn and math.isfinite(fin(r[m]))]
                rec={"row_type":"RUN_BIN","run_id":rid,"metric":m,"progress_bin":bn,"valid_decision_n":len(v),"run_bin_median":med(v)};bins.append(rec);cache[(rid,m,bn)]=rec
    for m in METRICS:
        for bn,_,_,_ in BINS:
            v=[fin(cache[(rid,m,bn)]["run_bin_median"]) for rid in RUNS if math.isfinite(fin(cache[(rid,m,bn)]["run_bin_median"]))]
            bins.append({"row_type":"COHORT_BIN","run_id":"","metric":m,"progress_bin":bn,"run_contributor_n":len(v),"run_macro_median":med(v),"run_macro_q25":quant(v,.25),"run_macro_q75":quant(v,.75)})
    write_csv(OUT/"MX027_IG_COVERAGE_PROGRESS_BINS.csv",bins)
    dirs=[]
    for rid,rows in byrun.items():
        for m in METRICS:
            v=[fin(r[m]) for r in rows if math.isfinite(fin(r[m]))];sp=spearman(m,rows)
            dirs.append({"run_id":rid,"metric":m,"first_valid_value":v[0] if v else math.nan,"final_valid_value":v[-1] if v else math.nan,"final_minus_first":(v[-1]-v[0]) if v else math.nan,"valid_n":len(v),"total_n":len(rows),**sp})
    write_csv(OUT/"MX027_IG_COVERAGE_WHOLE_RUN_DIRECTION.csv",dirs)
    # exact read-only five-family join
    pi={(r["run_id"],int(r["decision_id"])):r for r in per};joined=[]
    for m in mx:
        k=(m["run_id"],int(m["decision_id"]));n=pi[k]
        if str(m["normalized_progress"])!=str(n["normalized_progress"]) or str(m["progress_bin"])!=str(n["progress_bin"]) or str(m["Oracle4_marker"])!=str(n["Oracle4_marker"]):raise RuntimeError("FRAME_JOIN_MISMATCH:"+repr(k))
        row=dict(m)
        for key,val in n.items():
            if key not in ("run_id","decision_id","decision_index","decision_count","normalized_progress","progress_bin","Oracle4_marker"):row["MX027_"+key]=val
        joined.append(row)
    if len(joined)!=365:raise RuntimeError("JOIN_COUNT_FAIL")
    write_csv(OUT/"MX027_FIVE_FAMILY_PER_DECISION_VIEW.csv",joined)
    plots=[plot_run(rid,byrun[rid]) for rid in RUNS]
    dictionary={"task":"MX027","method_revision":METHOD_SHA,"technical_base":BASE_SHA,"accepted_mx026_result":MX026_SHA,"accepted_mx026_blob":MX026_BLOB,
      "metrics":{"IG_selected":{"role":"MAPEX_NATIVE_PRIMARY_IG"},"IG_visible_unknown_cells":{"role":"MAPEX_NATIVE_DOMAIN_SIZE_CONTEXT"},"IG_density":{"role":"PROJECT_DERIVED_DIAGNOSTIC"},"IG_policy_score":{"role":"MAPEX_NATIVE_POLICY_CONTEXT"},"KnownArea_m2":{"role":"DECISION_TIME_ONLINE_RECOVERABLE"},"DeltaKnownArea_m2":{"role":"PRIMARY_STAGNATION_SIGNAL_SIGNED"},"KnownAreaRate_m2_s":{"role":"PRIMARY_STAGNATION_RATE_SIGNED"}},
      "no_selection":{"count":59,"reason":"NO_RUNTIME_SELECTED_FRONTIER","IG_zero_fill":False},"quantile":"numpy linear, finite binary64 only, within-run bin median then equal-run macro","spearman":"finite complete cases, average-rank ties, Pearson ranks, n>=5, constant vectors NA, no p-value","forbidden":["model rerun","simulation","fallback","IG reconstruction","IG zero fill","interpolation","composite","winner","retuning","STOP"]}
    (OUT/"MX027_IG_COVERAGE_METRIC_DICTIONARY.json").write_text(json.dumps(dictionary,indent=2,sort_keys=True)+"\n")
    dindex=defaultdict(list)
    for r in dirs:dindex[r["metric"]].append(r)
    lines=["# MX027 Analyst Report — full-trajectory IG + Coverage/Stagnation","",f"Accepted methodology: {METHOD_SHA}",f"Frozen technical base: {BASE_SHA}","","## Inventory",
      f"- 365/365 decision rows; IG evaluable 306/365; legitimate runtime no-selection IG NA 59/365; Coverage evaluable 365/365.",
      "- Candidate lookup attempted only for the 306 nonblank-policy decisions; all 59 no-selection rows skipped candidate lookup.",
      "- Five-family joined view contains 365 exact accepted MX026 keys and preserves MX026 P/U/R values read-only.","","## Whole-run descriptive direction"]
    for m in METRICS:
        rs=dindex[m];dv=[fin(r["final_minus_first"]) for r in rs if math.isfinite(fin(r["final_minus_first"]))];rh=[fin(r["rho"]) for r in rs if math.isfinite(fin(r["rho"]))]
        lines.append(f"- {m}: final-minus-first negative/positive/zero = {sum(x<0 for x in dv)}/{sum(x>0 for x in dv)}/{sum(x==0 for x in dv)}; run-macro median delta={med(dv):.9g}; median rho(progress)={med(rh):.6g} across {len(rh)} finite run rhos.")
    neg_delta=sum(fin(r["DeltaKnownArea_m2"])<0 for r in per if math.isfinite(fin(r["DeltaKnownArea_m2"])))
    neg_rate=sum(fin(r["KnownAreaRate_m2_s"])<0 for r in per if math.isfinite(fin(r["KnownAreaRate_m2_s"])))
    lines+=["","## Signed coverage/stagnation context",f"- Negative DeltaKnownArea decisions: {neg_delta}; negative KnownAreaRate decisions: {neg_rate}. Negative values are retained, not clamped.","","## Boundaries","- No-selection IG is semantic NA, never IG=0.","- Coverage/Stagnation is independently computed from exact raw_map/resolution/time_s and does not depend on IG evaluability.","- No binary stagnation threshold, STOP rule, cross-family correlation matrix, composite, winner, causal claim, retuning, interpolation, model rerun or new simulation is produced."]
    (OUT/"MX027_ANALYST_REPORT.md").write_text("\n".join(lines)+"\n")
    arts=[OUT/"MX027_IG_COVERAGE_PER_DECISION.csv",OUT/"MX027_IG_PER_RUN.csv",OUT/"MX027_COVERAGE_PER_RUN.csv",OUT/"MX027_IG_COVERAGE_PROGRESS_BINS.csv",OUT/"MX027_IG_COVERAGE_WHOLE_RUN_DIRECTION.csv",OUT/"MX027_IG_COVERAGE_METRIC_DICTIONARY.json",OUT/"MX027_IG_COVERAGE_SOURCE_PARITY.csv",OUT/"MX027_FIVE_FAMILY_PER_DECISION_VIEW.csv",OUT/"MX027_ANALYST_REPORT.md",*plots]
    manifest={"schema":"mx027_ig_coverage_execution_v1","status":"COMPLETE_PENDING_INDEPENDENT_RESULT_QA","method_revision":METHOD_SHA,"technical_base":BASE_SHA,"accepted_mx026_result":MX026_SHA,
      "inventory":{"decision_rows":365,"IG_evaluable":sum(int(r["IG_evaluable"]) for r in per),"IG_no_selection":sum(r["IG_reason"]=="NO_RUNTIME_SELECTED_FRONTIER" for r in per),"Coverage_evaluable":sum(int(r["Coverage_evaluable"]) for r in per),"candidate_lookup_attempted":sum(int(r["candidate_lookup_attempted"]) for r in source),"progress_bin_rows":len(bins),"whole_run_direction_rows":len(dirs),"joined_rows":len(joined),"plot_count":len(plots)},
      "no_selection_by_run":dict(no),"guards":{"IG_zero_fill":False,"fallback":False,"interpolation":False,"composite":False,"winner":False,"retuning":False,"online_stop":False,"model_rerun":False,"simulation_rerun":False},
      "artifacts":[{"path":str(p.relative_to(OUT)),"sha256":sha256(p),"size":p.stat().st_size} for p in arts]}
    (OUT/"MX027_IG_COVERAGE_ARTIFACT_MANIFEST.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n")
    print(json.dumps({"inventory":manifest["inventory"],"no_selection_by_run":manifest["no_selection_by_run"],"negative_delta":neg_delta,"negative_rate":neg_rate},indent=2,sort_keys=True))
if __name__=="__main__":main()
# execution trigger: MX027 exact V3
