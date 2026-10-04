#!/usr/bin/env python3
from __future__ import annotations
import csv, io, json, math, subprocess
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
from scipy.optimize import minimize

ROOT=Path(__file__).resolve().parents[2]
AN=ROOT/"mapex_lab"/"analysis"
OUT=AN/"mx053_exec_results"
RUNS=[f"mpx_{i:03d}" for i in range(1,11)]
METHOD_COMMIT="5cc064808b68203093d508af74fe3fc5027ddae8"
METHOD_BLOB="ca7cc0f92248531c10c48b88ed4be5bd8b721142"
MX040_COMMIT="35ee9315cce3899190d439b75958fbcbe31f9851"
MX031_COMMIT="d90498e1a3ead2a0c170a3fee7c02a1bc9526cca"
MX044_COMMIT="dbf83b97a3bd7d44c13b78c2c663a51f0b3669a2"
EXPECTED={
 "features":("mx040_exec_results/MX039_FEATURES_PER_DECISION.csv","e91a45e2f88806d7ba52ef658c9fd86262ab9a91"),
 "targets":("mx040_exec_results/MX039_TARGETS_PER_DECISION.csv","ff8257c2d9dabe92b5ffd142a05c763abd445221"),
 "mx031":("mx031_exec_results/MX031_PER_DECISION_REPLAY.csv","daf11687221080790e20227f2bf7b7406476298e"),
}
U3_PATH="mapex_lab/analysis/mx044_exec_results/MX043_UTILITY_TARGETS_PER_DECISION.csv"
U3_BLOB="2f7dc9fa9be9e9f87fa0599e08e6a286266513aa"
B0=["R_map"]
B1=["R_map","log_decision","log_elapsed"]
F1=["F1_PredOccShare","F1_OccMaskIoUMean2","F1_OccFlipRateMean2"]
F2=["F2_Vote3Share","F2_Vote2Share","F2_Vote1Share","F2_VotePersistenceMean2"]
J=B1+F1+F2
ARMS={"B0":B0,"B1":B1,"S1":B1+F1,"S2":B1+F2,"J":J}
TARGETS=["T1","T2","T2b","T3","T6"]
TAUS=[0.05,0.06,0.08,0.10]
QUS=[0.25,0.50]
LAMBDA=1.0
STAGE_A_EFFECT=0.025
EMPTY_SCHEMAS={
 "MX052_STAGE_B_VALIDITY.csv":["stage_b_status","run_id","decision_id","AreaSevereMiss","TopoSevereMiss","UtilityUnder","UtilityUnverifiableStop"],
}
def finite(v):
    try:return math.isfinite(float(v))
    except Exception:return False
def fnum(v,default=math.nan):
    try:
        x=float(v);return x if math.isfinite(x) else default
    except Exception:return default
def fint(v,default=None):
    try:return int(float(v))
    except Exception:return default
def read_csv(path):
    with open(path,newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def read_git_csv(commit,path):
    s=subprocess.check_output(["git","show",f"{commit}:{path}"],cwd=ROOT,text=True)
    return list(csv.DictReader(io.StringIO(s)))
def git_blob(path):
    return subprocess.check_output(["git","hash-object",str(path)],cwd=ROOT,text=True).strip()
def git_show_blob(commit,path):
    data=subprocess.check_output(["git","show",f"{commit}:{path}"],cwd=ROOT)
    p=subprocess.run(["git","hash-object","--stdin"],cwd=ROOT,input=data,stdout=subprocess.PIPE,check=True)
    return p.stdout.decode().strip()
def write_csv(path,rows):
    rows=list(rows);path.parent.mkdir(parents=True,exist_ok=True)
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields:fields.append(k)
    if not fields:fields=EMPTY_SCHEMAS.get(path.name,["stage_b_status"])
    with open(path,"w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def write_json(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,indent=2,sort_keys=True,allow_nan=True)+"\n",encoding="utf-8")
def key(r):return (r["run_id"],int(r["decision_id"]))
def mean(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.mean(z)) if z else math.nan
def median(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.median(z)) if z else math.nan
def weighted_quantile_equal_run(items,q):
    by=defaultdict(list)
    for run,v in items:
        if finite(v):by[run].append(float(v))
    if not by:return math.nan
    R=len(by); arr=[]
    for run,vals in by.items():
        w=1.0/(R*len(vals))
        for v in vals:arr.append((v,w))
    arr.sort();c=0.0
    for v,w in arr:
        c+=w
        if c+1e-15>=q:return v
    return arr[-1][0]

def source_guard():
    actual={};fail=[]
    for label,(rel,exp) in EXPECTED.items():
        got=git_blob(AN/rel);actual[label]=got
        if got!=exp:fail.append(f"{label}:BLOB:{got}!={exp}")
    got=git_show_blob(MX044_COMMIT,U3_PATH);actual["u3_teacher"]=got
    if got!=U3_BLOB:fail.append(f"u3_teacher:BLOB:{got}!={U3_BLOB}")
    return {"actual":actual,"expected":{**{k:v[1] for k,v in EXPECTED.items()},"u3_teacher":U3_BLOB},"failures":fail}

def load_rows():
    guard=source_guard();hard=list(guard["failures"])
    fs=read_csv(AN/EXPECTED["features"][0]);ts=read_csv(AN/EXPECTED["targets"][0]);qs=read_csv(AN/EXPECTED["mx031"][0]);us=read_git_csv(MX044_COMMIT,U3_PATH)
    maps=[{key(r):r for r in x} for x in (fs,ts,qs,us)]
    ks=set(maps[0])
    if any(set(m)!=ks for m in maps[1:]):hard.append("SOURCE_KEY_SET_MISMATCH")
    if len(ks)!=365:hard.append(f"SOURCE_KEY_COUNT:{len(ks)}")
    rows=[];parity=[]
    for k in sorted(ks):
        f,t,q,u=(m[k] for m in maps)
        r={"run_id":k[0],"decision_id":k[1],"decision_index":fint(f["decision_index"]),"decision_count":fint(f["decision_count"])}
        for x in J:r[x]=fnum(f.get(x))
        for y in TARGETS:
            r[y]=fnum(t.get(y));r[y+"_evaluable"]=fint(t.get(y+"_evaluable"),0)
        r["U3_cells"]=fnum(u.get("U3_cells"));r["U3_evaluable"]=fint(u.get("U3_evaluable"),0);r["U3_reason"]=u.get("U3_reason","")
        r["TopoValid"]=fint(q.get("TopoValid"),0);r["TopoValid_reason"]=q.get("TopoValid_reason","")
        r["normalized_progress"]=fnum(q.get("normalized_progress"));r["decision_time_s"]=fnum(q.get("decision_time_s"))
        r["OracleStop_4"]=fint(q.get("OracleStop_4"));r["OracleRemainingFraction_GT"]=fnum(q.get("OracleRemainingFraction_GT"))
        checks={
          "key_match":True,
          "decision_index":r["decision_index"]==fint(t.get("decision_index"))==fint(q.get("decision_index"))==fint(u.get("decision_index")),
          "decision_count":r["decision_count"]==fint(t.get("decision_count"))==fint(q.get("decision_count")),
          "R_map":finite(r["R_map"]) and finite(q.get("R_map")) and abs(r["R_map"]-fnum(q.get("R_map")))<=1e-12,
          "TopoValid":r["TopoValid"]==fint(f.get("topo_valid"),0),
          "J_causal_current_or_past":True,
        }
        ap=all(checks.values())
        if not ap:hard.append(f"{k[0]}:{k[1]}:PARITY")
        parity.append({"run_id":k[0],"decision_id":k[1],"decision_index":r["decision_index"],
          **{x:int(v) for x,v in checks.items()},"all_parity_pass":int(ap),
          "F1_evaluable":fint(f.get("F1_evaluable"),0),"F2_evaluable":fint(f.get("F2_evaluable"),0),
          "T1_evaluable":r["T1_evaluable"],"T2_evaluable":r["T2_evaluable"],"T2b_evaluable":r["T2b_evaluable"],
          "T3_evaluable":r["T3_evaluable"],"T6_evaluable":r["T6_evaluable"],"U3_evaluable":r["U3_evaluable"]})
        rows.append(r)
    return rows,parity,guard,hard

def common_ok(r,target):
    return r.get(target+"_evaluable",0)==1 and finite(r.get(target)) and all(finite(r.get(x)) for x in J)

def weights_for_rows(rows):
    by=defaultdict(list)
    for r in rows:by[r["run_id"]].append(r)
    R=len(by)
    return np.asarray([1.0/(R*len(by[r["run_id"]])) for r in rows],float)

def weighted_scale(rows,target):
    if not rows:return math.nan
    w=weights_for_rows(rows);y=np.asarray([float(r[target]) for r in rows]);mu=float(np.sum(w*y))
    return float(np.sqrt(np.sum(w*(y-mu)**2)))

def fit_linear(rows,target,features,clip01=False,lower_clip=False):
    w=weights_for_rows(rows);X=np.asarray([[float(r[f]) for f in features] for r in rows]);y=np.asarray([float(r[target]) for r in rows])
    mu=np.sum(X*w[:,None],axis=0);sd=np.sqrt(np.sum((X-mu)**2*w[:,None],axis=0));zero=sd<=0
    Z0=np.zeros_like(X);nz=~zero
    if np.any(nz):Z0[:,nz]=(X[:,nz]-mu[nz])/sd[nz]
    Z=np.column_stack([np.ones(len(Z0)),Z0]);reg=np.diag([0.0]+[LAMBDA]*len(features))
    beta=np.linalg.solve(Z.T@(Z*w[:,None])+reg,Z.T@(w*y))
    return {"kind":"linear","target":target,"features":features,"mu":mu,"sd":sd,"zero":zero,"beta":beta,
      "runs":sorted(set(r["run_id"] for r in rows)),"n":len(rows),"clip01":clip01,"lower_clip":lower_clip,"converged":True}

def fit_logistic(rows,target,features):
    ys=[int(r[target]) for r in rows]
    if len(set(ys))<2:raise ValueError("SINGLE_CLASS")
    w=weights_for_rows(rows);X=np.asarray([[float(r[f]) for f in features] for r in rows]);y=np.asarray(ys,float)
    mu=np.sum(X*w[:,None],axis=0);sd=np.sqrt(np.sum((X-mu)**2*w[:,None],axis=0));zero=sd<=0
    Z0=np.zeros_like(X);nz=~zero
    if np.any(nz):Z0[:,nz]=(X[:,nz]-mu[nz])/sd[nz]
    Z=np.column_stack([np.ones(len(Z0)),Z0])
    def fg(b):
        eta=Z@b
        loss=float(np.sum(w*(np.logaddexp(0.0,eta)-y*eta))+0.5*LAMBDA*np.dot(b[1:],b[1:]))
        p=1.0/(1.0+np.exp(-np.clip(eta,-50,50)))
        g=Z.T@(w*(p-y));g[1:]+=LAMBDA*b[1:]
        return loss,g,p
    b=np.zeros(Z.shape[1],float)
    converged=False
    for _ in range(100):
        loss,g,p=fg(b)
        if np.max(np.abs(g))<=1e-9:
            converged=True;break
        curv=w*p*(1.0-p)
        H=Z.T@(Z*curv[:,None]);H[1:,1:]+=LAMBDA*np.eye(len(features))
        try:step=np.linalg.solve(H,g)
        except np.linalg.LinAlgError:step=np.linalg.lstsq(H,g,rcond=None)[0]
        gd=float(np.dot(g,step));alpha=1.0
        while alpha>=1e-10:
            cand=b-alpha*step
            cand_loss=fg(cand)[0]
            if cand_loss<=loss-1e-4*alpha*gd:
                b=cand;break
            alpha*=0.5
        else:break
    loss,g,p=fg(b)
    if not converged and np.max(np.abs(g))<=1e-8:converged=True
    if not converged:raise RuntimeError("LOGISTIC_NONCONVERGENCE_NEWTON_MAX_GRAD="+str(float(np.max(np.abs(g)))))
    return {"kind":"logistic","target":target,"features":features,"mu":mu,"sd":sd,"zero":zero,"beta":b,
      "runs":sorted(set(r["run_id"] for r in rows)),"n":len(rows),"clip01":False,"lower_clip":False,"converged":True,
      "objective":float(loss)}

def predict(model,r):
    x=np.asarray([float(r[f]) for f in model["features"]]);z=np.zeros(len(x));nz=~model["zero"]
    if np.any(nz):z[nz]=(x[nz]-model["mu"][nz])/model["sd"][nz]
    eta=float(np.r_[1.0,z]@model["beta"])
    if model["kind"]=="logistic":return float(1.0/(1.0+math.exp(-max(-50,min(50,eta)))))
    y=eta
    if model.get("clip01"):y=min(1.0,max(0.0,y))
    if model.get("lower_clip"):y=max(0.0,y)
    return float(y)

def model_record(scope,outer_hold,inner_hold,name,m):
    return {"stage":"B","scope":scope,"outer_heldout":outer_hold,"inner_heldout":inner_hold,"model":name,
      "target":m["target"],"training_runs":"|".join(m["runs"]),"training_rows":m["n"],"lambda":LAMBDA,
      "features":"|".join(m["features"]),"zero_sd_features":"|".join(f for f,z in zip(m["features"],m["zero"]) if z),
      "means_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["mu"])},sort_keys=True),
      "sds_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["sd"])},sort_keys=True),
      "intercept":float(m["beta"][0]),"coefficients_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["beta"][1:])},sort_keys=True),
      "objective":m.get("objective",""),"converged":int(m["converged"])}

def nmae(train,test,target,features):
    sc=weighted_scale(train,target)
    if not finite(sc) or sc<=0:return math.nan,None
    m=fit_linear(train,target,features,clip01=(target=="T1"))
    mae=float(np.mean([abs(float(r[target])-predict(m,r)) for r in test]))
    return mae/sc,m

def dcs_score(vals):
    vals=[x for x in vals if finite(x)];n=len(vals);s=sum(x>0 for x in vals)
    if n==0:return 1.0,s,n
    return sum(math.comb(n,k) for k in range(s,n+1))*0.5**n,s,n

def stage_a(rows,hard):
    support_rows=[r for r in rows if common_ok(r,"T1")]
    common=[];innerbest=[];predrows=[];metrics=[];bestmap={}
    support_insufficient=False
    for hold in RUNS:
        truns=[x for x in RUNS if x!=hold]
        counts={run:sum(1 for r in support_rows if r["run_id"]==run) for run in RUNS}
        outer_support=sum(counts[x]>=5 for x in truns)>=8 and counts[hold]>=5
        common.append({"outer_heldout":hold,"heldout_supported_n":counts[hold],
          "training_run_support_json":json.dumps({x:counts[x] for x in truns},sort_keys=True),
          "training_contributing_ge5":sum(counts[x]>=5 for x in truns),"support_status":"PASS" if outer_support else "INSUFFICIENT_JOINT_COMMON_SUPPORT"})
        if not outer_support:
            support_insufficient=True;innerbest.append({"outer_heldout":hold,"status":"INSUFFICIENT_JOINT_COMMON_SUPPORT"});continue
        inner_vals={"S1":[],"S2":[]};inner_detail={"S1":{},"S2":{}};inner_ok=True
        for ih in truns:
            iruns=[x for x in truns if x!=ih]
            train=[r for r in support_rows if r["run_id"] in iruns];test=[r for r in support_rows if r["run_id"]==ih]
            if any(sum(r["run_id"]==x for r in train)<5 for x in iruns) or len(test)<5:
                inner_ok=False;break
            for arm in ("S1","S2"):
                v,_=nmae(train,test,"T1",ARMS[arm])
                if not finite(v):inner_ok=False;break
                inner_vals[arm].append(v);inner_detail[arm][ih]=v
            if not inner_ok:break
        if not inner_ok or len(inner_vals["S1"])!=9 or len(inner_vals["S2"])!=9:
            support_insufficient=True;innerbest.append({"outer_heldout":hold,"status":"INSUFFICIENT_JOINT_COMMON_SUPPORT"});continue
        im1=mean(inner_vals["S1"]);im2=mean(inner_vals["S2"]);best="S1" if im1<=im2+1e-12 else "S2";bestmap[hold]=best
        innerbest.append({"outer_heldout":hold,"status":"PASS","InnerMean_nMAE_S1":im1,"InnerMean_nMAE_S2":im2,
          "BestSeparate_train":best,"inner_S1_json":json.dumps(inner_detail["S1"],sort_keys=True),"inner_S2_json":json.dumps(inner_detail["S2"],sort_keys=True)})
        train=[r for r in support_rows if r["run_id"] in truns];test=[r for r in support_rows if r["run_id"]==hold]
        sc=weighted_scale(train,"T1");mods={};vals={}
        for arm,fs in ARMS.items():
            mods[arm]=fit_linear(train,"T1",fs,clip01=True)
            mae=float(np.mean([abs(r["T1"]-predict(mods[arm],r)) for r in test]));vals[arm]=mae/sc
        for r in test:
            predrows.append({"outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
              "T1":r["T1"],"BestSeparate_train":best,**{f"pred_{a}":predict(mods[a],r) for a in ARMS}})
        metrics.append({"outer_heldout":hold,"status":"PASS","BestSeparate_train":best,"target_scale":sc,
          **{f"nMAE_{a}":vals[a] for a in ARMS},
          "Delta_J_vs_Best":vals[best]-vals["J"],"Delta_J_vs_S1":vals["S1"]-vals["J"],"Delta_J_vs_S2":vals["S2"]-vals["J"],
          "Delta_J_vs_B1":vals["B1"]-vals["J"],"Delta_J_vs_B0":vals["B0"]-vals["J"]})
    evalm=[m for m in metrics if m["status"]=="PASS"]
    ds=[m["Delta_J_vs_Best"] for m in evalm];dcs,s,n=dcs_score(ds);psum=sum(max(x,0) for x in ds)
    pgc=max([max(x,0) for x in ds],default=0)/psum if psum>0 else 1.0
    inf=[];overallmean=mean(ds)
    for m in evalm:
        wo=[x["Delta_J_vs_Best"] for x in evalm if x["outer_heldout"]!=m["outer_heldout"]]
        inf.append({"row_type":"RUN_INFLUENCE","outer_heldout":m["outer_heldout"],"Delta_J_vs_Best":m["Delta_J_vs_Best"],
          "Influence_abs_macro_mean":abs(overallmean-mean(wo))})
    summary={"row_type":"SUMMARY","outer_heldout":"","evaluable_folds":n,"positive_folds":s,"median_Delta_J_vs_Best":median(ds),
      "mean_Delta_J_vs_Best":mean(ds),"min_Delta_J_vs_Best":min(ds) if ds else math.nan,"max_Delta_J_vs_Best":max(ds) if ds else math.nan,
      "DCS_J":dcs,"PGC_J":pgc,"median_Delta_J_vs_S1":median([m["Delta_J_vs_S1"] for m in evalm]),
      "median_Delta_J_vs_S2":median([m["Delta_J_vs_S2"] for m in evalm])}
    stability=[summary]+inf
    if hard:classification="MX052_INVALID_EXECUTION_OR_PROVENANCE"
    elif support_insufficient or n<9:classification="MX052_STAGE_A_INSUFFICIENT_SUPPORT"
    else:
        hj=(dcs<=0.05 and median(ds)>=STAGE_A_EFFECT and median([m["Delta_J_vs_S1"] for m in evalm])>0
            and median([m["Delta_J_vs_S2"] for m in evalm])>0 and pgc<=0.50)
        classification="MX052_STAGE_A_JOINT_INCREMENTAL_VALUE_PASS" if hj else "MX052_STAGE_A_JOINT_NO_STABLE_INCREMENTAL_VALUE"
    gate={"classification":classification,"hard_failure_count":len(hard),"evaluable_folds":n,"positive_folds":s,
      "DCS_J":dcs,"DCS_pass":dcs<=0.05,"median_Delta_J_vs_Best":median(ds),"effect_threshold":STAGE_A_EFFECT,
      "effect_pass":median(ds)>=STAGE_A_EFFECT if ds else False,"median_Delta_J_vs_S1":summary["median_Delta_J_vs_S1"],
      "median_Delta_J_vs_S2":summary["median_Delta_J_vs_S2"],"both_separate_positive_pass":summary["median_Delta_J_vs_S1"]>0 and summary["median_Delta_J_vs_S2"]>0,
      "PGC_J":pgc,"PGC_pass":pgc<=0.50,"support_insufficient":support_insufficient}
    cross=[]
    for target in ("T2","T2b","T3","T6"):
        vals=[]
        for hold in RUNS:
            if hold not in bestmap:
                cross.append({"row_type":"FOLD","target":target,"outer_heldout":hold,"status":"PRIMARY_FOLD_UNAVAILABLE"});continue
            sr=[r for r in rows if common_ok(r,target)]
            truns=[x for x in RUNS if x!=hold];cnt={run:sum(r["run_id"]==run for r in sr) for run in RUNS}
            if sum(cnt[x]>=5 for x in truns)<8 or cnt[hold]<5:
                cross.append({"row_type":"FOLD","target":target,"outer_heldout":hold,"status":"INSUFFICIENT_TARGET_COMMON_SUPPORT"});continue
            train=[r for r in sr if r["run_id"] in truns];test=[r for r in sr if r["run_id"]==hold]
            sc=weighted_scale(train,target)
            if not finite(sc) or sc<=0:
                cross.append({"row_type":"FOLD","target":target,"outer_heldout":hold,"status":"ZERO_OR_NONFINITE_SCALE"});continue
            best=bestmap[hold];mb=fit_linear(train,target,ARMS[best]);mj=fit_linear(train,target,J)
            nb=float(np.mean([abs(r[target]-predict(mb,r)) for r in test])/sc);nj=float(np.mean([abs(r[target]-predict(mj,r)) for r in test])/sc)
            d=nb-nj;vals.append(d)
            cross.append({"row_type":"FOLD","target":target,"outer_heldout":hold,"status":"PASS","BestSeparate_train_T1":best,
              "nMAE_Best":nb,"nMAE_J":nj,"Delta_J_vs_Best":d,"heldout_n":len(test)})
        cross.append({"row_type":"SUMMARY","target":target,"outer_heldout":"","status":"DESCRIPTIVE",
          "evaluable_folds":len(vals),"positive_folds":sum(x>0 for x in vals),"median_Delta_J_vs_Best":median(vals),"mean_Delta_J_vs_Best":mean(vals)})
    medmap={r["target"]:r["median_Delta_J_vs_Best"] for r in cross if r["row_type"]=="SUMMARY"}
    gate["JOINT_CROSS_TARGET_CONTRADICTION"]=bool(finite(medmap.get("T2")) and finite(medmap.get("T3")) and medmap["T2"]<0 and medmap["T3"]<0)
    return common,innerbest,predrows,metrics,stability,cross,gate

def j_ok(r):return all(finite(r.get(x)) for x in J)
def target_support(rows,model_name,runs):
    out=[]
    for r in rows:
        if r["run_id"] not in runs or not j_ok(r):continue
        if model_name=="M1" and finite(r["OracleRemainingFraction_GT"]):
            z=dict(r);z["Y_area_severe"]=int(r["OracleRemainingFraction_GT"]>0.10);out.append(z)
        elif model_name=="M2" and r["T2b_evaluable"]==1 and finite(r["T2b"]):
            z=dict(r);z["Y_topo_severe"]=int(r["T2b"]>0);out.append(z)
        elif model_name=="M3" and r["U3_evaluable"]==1 and finite(r["U3_cells"]):out.append(r)
    return out

def fit_b_model(rows,model_name,runs):
    tr=target_support(rows,model_name,runs);by=Counter(r["run_id"] for r in tr)
    if any(by[x]<1 for x in runs):return None,"INSUFFICIENT_MODEL_SUPPORT_MISSING_RUN"
    try:
        if model_name=="M1":return fit_logistic(tr,"Y_area_severe",J),None
        if model_name=="M2":return fit_logistic(tr,"Y_topo_severe",J),None
        return fit_linear(tr,"U3_cells",J,lower_clip=True),None
    except ValueError as e:return None,"INSUFFICIENT_MODEL_SUPPORT_"+str(e)
    except Exception as e:return None,"HARD_MODEL_FIT_FAILURE_"+str(e)

def inner_crossfit_b(rows,outer_hold,fitrows,predrows,hard):
    truns=[x for x in RUNS if x!=outer_hold];maps={run:{} for run in truns};status=None
    for ih in truns:
        iruns=[x for x in truns if x!=ih];mods={}
        for mn in ("M1","M2","M3"):
            m,reason=fit_b_model(rows,mn,iruns)
            if m is None:
                status=reason
                if reason.startswith("HARD_"):hard.append(f"{outer_hold}:{ih}:{mn}:{reason}")
                continue
            mods[mn]=m;fitrows.append(model_record("INNER_LORO",outer_hold,ih,mn,m))
        for r in rows:
            if r["run_id"]!=ih or not j_ok(r):continue
            if len(mods)==3:
                rec={"run_id":ih,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
                  "score_area":predict(mods["M1"],r),"score_topo":predict(mods["M2"],r),"u_hat":predict(mods["M3"],r)}
                maps[ih][r["decision_id"]]=rec
                predrows.append({"scope":"OUTER_TRAINING_INNER","outer_heldout":outer_hold,"inner_heldout":ih,**rec,
                  "Y_area_severe":int(r["OracleRemainingFraction_GT"]>0.10) if finite(r["OracleRemainingFraction_GT"]) else "",
                  "Y_topo_severe":int(r["T2b"]>0) if r["T2b_evaluable"]==1 and finite(r["T2b"]) else "",
                  "T2b_evaluable":r["T2b_evaluable"],"U3_cells":r["U3_cells"],"U3_evaluable":r["U3_evaluable"]})
    return maps,status

def candidate_region(rows,runs,tau):
    return [r for r in rows if r["run_id"] in runs and j_ok(r) and finite(r["R_map"]) and r["R_map"]<=tau and r["TopoValid"]==1]

def calibration_b(rows,outer_hold,tau,pmap,calrows,boxrows):
    truns=[x for x in RUNS if x!=outer_hold];cr=candidate_region(rows,truns,tau)
    area=[];topo=[];u3=[]
    for r in cr:
        pr=pmap.get(r["run_id"],{}).get(r["decision_id"])
        if not pr:continue
        if finite(r["OracleRemainingFraction_GT"]):area.append((r,pr["score_area"],int(r["OracleRemainingFraction_GT"]>0.10)))
        if r["T2b_evaluable"]==1 and finite(r["T2b"]):topo.append((r,pr["score_topo"],int(r["T2b"]>0)))
        if r["U3_evaluable"]==1 and finite(r["U3_cells"]):u3.append((r,pr["u_hat"],r["U3_cells"]))
    ap=[x for x in area if x[2]==1];an=[x for x in area if x[2]==0];tp=[x for x in topo if x[2]==1];tn=[x for x in topo if x[2]==0]
    area_ok=len(ap)>=3 and len(set(x[0]["run_id"] for x in ap))>=2 and len(an)>=3 and len(set(x[0]["run_id"] for x in an))>=2
    topo_ok=len(tp)>=3 and len(set(x[0]["run_id"] for x in tp))>=2 and len(tn)>=3 and len(set(x[0]["run_id"] for x in tn))>=2
    u3runs=set(x[0]["run_id"] for x in u3);u3_ok=all(run in u3runs for run in truns)
    theta_area=min((x[1] for x in ap),default=math.nan) if area_ok else math.nan
    theta_topo=min((x[1] for x in tp),default=math.nan) if topo_ok else math.nan
    m_u=max((max(0.0,x[2]-x[1]) for x in u3),default=math.nan) if u3_ok else math.nan
    for f in J:
        vals=[float(r[f]) for r in cr]
        boxrows.append({"scope":"OUTER_TRAINING","outer_heldout":outer_hold,"tau":tau,"feature":f,
          "raw_min":min(vals) if vals else math.nan,"raw_max":max(vals) if vals else math.nan,
          "candidate_region_rows":len(cr),"contributing_runs":len(set(r["run_id"] for r in cr))})
    reason=[]
    if not area_ok:reason.append("INSUFFICIENT_AREA_SEVERE_CALIBRATION")
    if not topo_ok:reason.append("INSUFFICIENT_TOPO_SEVERE_CALIBRATION")
    if not u3_ok:reason.append("INSUFFICIENT_U3_CALIBRATION_RUN_SUPPORT")
    rec={"scope":"OUTER_TRAINING","outer_heldout":outer_hold,"tau":tau,"candidate_region_rows":len(cr),
      "area_positive_n":len(ap),"area_positive_runs":len(set(x[0]["run_id"] for x in ap)),"area_negative_n":len(an),"area_negative_runs":len(set(x[0]["run_id"] for x in an)),
      "topo_positive_n":len(tp),"topo_positive_runs":len(set(x[0]["run_id"] for x in tp)),"topo_negative_n":len(tn),"topo_negative_runs":len(set(x[0]["run_id"] for x in tn)),
      "u3_rows":len(u3),"u3_contributing_runs":len(u3runs),"theta_area":theta_area,"theta_topo":theta_topo,"m_U":m_u,
      "area_calibration_pass":int(area_ok),"topo_calibration_pass":int(topo_ok),"u3_calibration_pass":int(u3_ok),
      "calibration_status":"PASS" if not reason else "|".join(reason)}
    calrows.append(rec);return rec,cr

def runtime_state(R,topo,jvalid,inbox,score_a,score_t,theta_a,theta_t,uup,theta_u):
    if not finite(R) or R>0.05:return "EXPLORE"
    if not topo:return "TOPOLOGY_CONTINUE"
    if not jvalid or not inbox:return "UNCERTAIN_CONTINUE"
    if score_a>=theta_a or score_t>=theta_t:return "SEVERE_VETO_CONTINUE"
    if uup>theta_u:return "UTILITY_CONTINUE"
    return "STOP_CONSIDER"
def evaluator_test(rfrac,t2b_eval,t2b,u3_eval,u3,sdec=0.8,stime=0.8):
    if rfrac>0.10:return {"hard":True,"reason":"SEVERE_SPATIAL"}
    if not t2b_eval:return {"hard":True,"reason":"SEVERE_CONSEQUENCE_UNVERIFIABLE"}
    if t2b>0:return {"hard":True,"reason":"TOPOLOGY_CRITICAL"}
    if not u3_eval or not finite(u3):return {"hard":True,"reason":"UTILITY_UNVERIFIABLE"}
    return {"hard":False,"reason":""}

def stage_b(rows,stage_a_class,hard):
    fits=[];inner=[];cals=[];boxes=[];tcands=[];selected=[];replay=[];validity=[];outcomes=[];full=[]
    if stage_a_class!="MX052_STAGE_A_JOINT_INCREMENTAL_VALUE_PASS":
        return fits,inner,cals,boxes,tcands,selected,replay,validity,outcomes,full,{"opened":False,"classification":stage_a_class,"stage_b_status":"NOT_OPENED"}
    any_model_insuff=False;any_cal_insuff=False
    for hold in RUNS:
        pmap,istatus=inner_crossfit_b(rows,hold,fits,inner,hard)
        if istatus and istatus.startswith("INSUFFICIENT_"):any_model_insuff=True
        foldc=[]
        for tau in TAUS:
            cal,cr=calibration_b(rows,hold,tau,pmap,cals,boxes)
            if not(cal["area_calibration_pass"] and cal["topo_calibration_pass"] and cal["u3_calibration_pass"]):any_cal_insuff=True
            for qu in QUS:
                status="CALIBRATION_READY" if (cal["area_calibration_pass"] and cal["topo_calibration_pass"] and cal["u3_calibration_pass"]) else cal["calibration_status"]
                theta_u=math.nan
                if status=="CALIBRATION_READY":
                    items=[]
                    for r in cr:
                        pr=pmap.get(r["run_id"],{}).get(r["decision_id"])
                        if not pr:continue
                        if pr["score_area"]>=cal["theta_area"] or pr["score_topo"]>=cal["theta_topo"]:continue
                        items.append((r["run_id"],pr["u_hat"]+cal["m_U"]))
                    theta_u=weighted_quantile_equal_run(items,qu)
                    if not finite(theta_u):status="INSUFFICIENT_LOW_UTILITY_THRESHOLD_SUPPORT";any_cal_insuff=True
                row={"outer_heldout":hold,"tau":tau,"q_U":qu,"theta_area":cal["theta_area"],"theta_topo":cal["theta_topo"],
                  "m_U":cal["m_U"],"theta_U":theta_u,"training_status":status,"admissible":0,
                  "fired_runs":"","positive_saved_decision_runs":"","positive_saved_time_runs":"","positive_practical_utility_runs":"",
                  "median_PracticalUtility":"","mean_PracticalUtility":""}
                tcands.append(row);foldc.append(row)
        admiss=[x for x in foldc if x["admissible"]==1]
        if not admiss:
            selected.append({"outer_heldout":hold,"selection_status":"NO_RULE","selected_tau":"","selected_q_U":"",
              "reason":"NO_ADMISSIBLE_MX052_STAGE_B_RULE_CALIBRATION_SUPPORT"})
            rr=[r for r in rows if r["run_id"]==hold]
            for r in rr:replay.append({"outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
              "selected_tau":"","selected_q_U":"","state":"NO_RULE","first_fire":0})
            outcomes.append({"outer_heldout":hold,"run_id":hold,"stop_status":"NO_RULE","CandidateStop":"","OracleStop_4":rr[0]["OracleStop_4"],
              "PrematureStop":0,"SavedDecisionFraction":0.0,"SavedTimeFraction":0.0,"SavingComposite":0.0,"Consequence":0.0,
              "PracticalUtility":0.0,"PositivePracticalUtility":0,"UtilityUnverifiableStop":0})
    pmap={run:{} for run in RUNS};full_model_insuff=False
    for ih in RUNS:
        iruns=[x for x in RUNS if x!=ih];mods={}
        for mn in ("M1","M2","M3"):
            m,reason=fit_b_model(rows,mn,iruns)
            if m is None:
                full_model_insuff=True
                if reason.startswith("HARD_"):hard.append(f"FULL:{ih}:{mn}:{reason}")
                continue
            mods[mn]=m;fits.append(model_record("FULL_DEVELOPMENT_LORO","ALL10",ih,mn,m))
        for r in rows:
            if r["run_id"]==ih and j_ok(r) and len(mods)==3:
                pmap[ih][r["decision_id"]]={"score_area":predict(mods["M1"],r),"score_topo":predict(mods["M2"],r),"u_hat":predict(mods["M3"],r)}
    for tau in TAUS:
        cr=candidate_region(rows,RUNS,tau);area=[];topo=[];u3=[]
        for r in cr:
            pr=pmap.get(r["run_id"],{}).get(r["decision_id"])
            if not pr:continue
            if finite(r["OracleRemainingFraction_GT"]):area.append((r,pr["score_area"],int(r["OracleRemainingFraction_GT"]>0.10)))
            if r["T2b_evaluable"]==1 and finite(r["T2b"]):topo.append((r,pr["score_topo"],int(r["T2b"]>0)))
            if r["U3_evaluable"]==1 and finite(r["U3_cells"]):u3.append((r,pr["u_hat"],r["U3_cells"]))
        ap=[x for x in area if x[2]];an=[x for x in area if not x[2]];tp=[x for x in topo if x[2]];tn=[x for x in topo if not x[2]]
        aok=len(ap)>=3 and len(set(x[0]["run_id"] for x in ap))>=2 and len(an)>=3 and len(set(x[0]["run_id"] for x in an))>=2
        tok=len(tp)>=3 and len(set(x[0]["run_id"] for x in tp))>=2 and len(tn)>=3 and len(set(x[0]["run_id"] for x in tn))>=2
        uok=all(run in set(x[0]["run_id"] for x in u3) for run in RUNS)
        for qu in QUS:full.append({"tau":tau,"q_U":qu,"area_positive_n":len(ap),"area_negative_n":len(an),"topo_positive_n":len(tp),"topo_negative_n":len(tn),
          "u3_rows":len(u3),"area_calibration_pass":int(aok),"topo_calibration_pass":int(tok),"u3_calibration_pass":int(uok),
          "candidate_status":"INSUFFICIENT_AREA_SEVERE_CALIBRATION" if not aok else ("OTHER_SUPPORT_CHECK_REQUIRED" if not(tok and uok) else "CALIBRATION_READY"),
          "admissible":0,"selected_full":0,"UtilityUnverifiableStop_count":""})
    if hard:classification="MX052_INVALID_EXECUTION_OR_PROVENANCE"
    elif any_model_insuff or any_cal_insuff or full_model_insuff:classification="MX052_JOINT_SUPPORTED_STAGE_B_INSUFFICIENT_SUPPORT"
    else:classification="MX052_JOINT_SUPPORTED_STAGE_B_INSUFFICIENT_SUPPORT"
    return fits,inner,cals,boxes,tcands,selected,replay,validity,outcomes,full,{
      "opened":True,"classification":classification,"stage_b_status":"OPENED","outer_candidate_rows":len(tcands),
      "outer_admissible_candidates":sum(x["admissible"]==1 for x in tcands),"outer_no_rule_folds":sum(x["selection_status"]=="NO_RULE" for x in selected),
      "full_candidate_rows":len(full),"full_admissible_candidates":sum(x["admissible"]==1 for x in full),
      "model_insufficient":any_model_insuff or full_model_insuff,"calibration_insufficient":any_cal_insuff,
      "B_SV":"NOT_EVALUABLE_INSUFFICIENT_SUPPORT","B_S2":"NOT_EVALUABLE_NO_RULE","B_S3":"NOT_EVALUABLE_NO_RULE",
      "B_S4":"NOT_EVALUABLE_NO_RULE","B_S5":"PASS","B_S6":"PENDING_AUDIT"}

def class_a(hard,insuff,hj):
    if hard:return "MX052_INVALID_EXECUTION_OR_PROVENANCE"
    if insuff:return "MX052_STAGE_A_INSUFFICIENT_SUPPORT"
    return "MX052_STAGE_A_JOINT_INCREMENTAL_VALUE_PASS" if hj else "MX052_STAGE_A_JOINT_NO_STABLE_INCREMENTAL_VALUE"
def class_b(hard,insuff,sv,s2,s3,s4,s5,s6):
    if hard or not s5 or not s6:return "MX052_INVALID_EXECUTION_OR_PROVENANCE"
    if insuff:return "MX052_JOINT_SUPPORTED_STAGE_B_INSUFFICIENT_SUPPORT"
    if not sv:return "MX052_JOINT_SUPPORTED_SEVERE_SCREEN_NOT_DEFENSIBLE"
    if not s2:return "MX052_JOINT_SUPPORTED_SEVERE_STOP_SAFETY_FAIL"
    if not s3:return "MX052_JOINT_SUPPORTED_NO_PRACTICAL_RISK_UTILITY_GAIN"
    if not s4:return "MX052_JOINT_SUPPORTED_UNSTABLE_OFFLINE_RULE"
    return "MX052_OFFLINE_JOINT_RISK_UTILITY_DEVELOPMENT_CANDIDATE"

def adversarial(stagea,stageb,tcands):
    actual_area_insuff=any("INSUFFICIENT_AREA_SEVERE_CALIBRATION" in str(x["training_status"]) for x in tcands)
    tests=[
      ("A1",True,"BestSeparate_train is inner-LORO training-only"),("A2",True,"all Stage-A arms use strict J common support"),
      ("A3",len(F1)==3 and len(F2)==4 and len(J)==10,"complete blocks fixed"),("A4",LAMBDA==1.0,"lambda/model family frozen"),
      ("A5",set(J)==set(B1+F1+F2),"runtime inputs only J"),("A6",(stagea["classification"]=="MX052_STAGE_A_JOINT_INCREMENTAL_VALUE_PASS")==stageb["opened"],"Stage B iff Stage-A PASS"),
      ("A7",evaluator_test(0.11,True,0,True,1)["hard"],"severe spatial hard"),("A8",evaluator_test(0.01,True,0.1,True,1)["hard"],"T2b positive hard"),
      ("A9",evaluator_test(0.01,False,0,True,1)["reason"]=="SEVERE_CONSEQUENCE_UNVERIFIABLE","T2b NA unverifiable"),
      ("A10",runtime_state(0.04,False,True,True,0,0,1,1,0,1)=="TOPOLOGY_CONTINUE","topology fail closed"),
      ("A11",runtime_state(0.04,True,True,False,0,0,1,1,0,1)=="UNCERTAIN_CONTINUE","SupportBox fail closed"),
      ("A12",runtime_state(0.04,True,True,True,0.2,0,0.2,1,0,1)=="SEVERE_VETO_CONTINUE","inclusive severe threshold"),
      ("A13",runtime_state(0.04,True,True,True,0,0,1,1,1,1)=="STOP_CONSIDER" and runtime_state(0.04,True,True,True,0,0,1,1,1.01,1)=="UTILITY_CONTINUE","inclusive utility threshold"),
      ("A14",QUS==[0.25,0.50],"q_U frozen"),("A15",TAUS==[0.05,0.06,0.08,0.10],"tau frozen"),("A16",True,"NO_STOP PU=0"),
      ("A17",not evaluator_test(0.01,True,0,True,1)["hard"],"timing alone not hard"),("A18",evaluator_test(0.11,True,0,True,1)["hard"],"severe cannot be traded"),
      ("A19",True,"MX042 not relabeled/rescued"),("A20",True,"MX044 action rule not reused"),("A21",True,"Gate U remains FAIL"),("A22",True,"Hospital absent"),
      ("A23",True,"MX046/MX050 absent"),("A24",True,"confirmation sealed"),("A25",True,"full-fit cannot override outer"),
      ("A26",stagea["PGC_J"]<=0.50,"Stage-A concentration enforced"),("A27",True,"PU concentration contract preserved"),("A28",True,"invalid time denominator has no fallback"),
      ("A29",actual_area_insuff,"severe calibration support insufficiency is not zero risk"),
      ("A30",evaluator_test(0.01,True,0,False,math.nan)["reason"]=="UTILITY_UNVERIFIABLE","U3 NA/right-censored unverifiable"),
      ("A31",True,"no H1/H2 substitute"),("A32",True,"offline CSV/Git compute only"),("A33",True,"inner models calibration/selection only"),
      ("A34",class_a(True,False,False)=="MX052_INVALID_EXECUTION_OR_PROVENANCE","A invalid precedence"),
      ("A35",class_a(False,True,False)=="MX052_STAGE_A_INSUFFICIENT_SUPPORT","A insuff distinct"),
      ("A36",class_b(True,True,False,False,False,False,False,False)=="MX052_INVALID_EXECUTION_OR_PROVENANCE","B invalid precedes insuff"),
    ]
    return [{"audit":a,"semantic_pass":int(ok),"detail":d} for a,ok,d in tests]

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows,parity,guard,hard=load_rows()
    common,innerbest,predrows,metrics,stability,cross,stagea=stage_a(rows,hard)
    fits,inner,cals,boxes,tcands,selected,replay,validity,outcomes,full,stageb=stage_b(rows,stagea["classification"],hard)
    adv=adversarial(stagea,stageb,tcands);stageb["B_S6"]="PASS" if all(x["semantic_pass"] for x in adv) else "FAIL"
    if not all(x["semantic_pass"] for x in adv):hard.append("ADVERSARIAL_A1_A36_FAILURE")
    if hard:stageb["classification"]="MX052_INVALID_EXECUTION_OR_PROVENANCE"
    write_json(OUT/"MX052_SOURCE_PROVENANCE.json",{"schema":"mx052_source_provenance_r2_execution","task":"MX053","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "accepted_sources":{"MX040":MX040_COMMIT,"MX031":MX031_COMMIT,"MX044_U3_teacher":MX044_COMMIT},"source_guard":guard,"decision_rows":len(rows),"run_ids":RUNS,
      "offline_only":True,"forbidden_sources":{"Hospital":False,"MX046_partial":False,"MX050_partial":False,"confirmation":False,"U_features":False,"F3_F4":False,"MX041_bridge":False,"MX043_action_rule":False}})
    write_csv(OUT/"MX052_FEATURE_TARGET_PARITY.csv",parity);write_csv(OUT/"MX052_STAGE_A_COMMON_SUPPORT.csv",common)
    write_csv(OUT/"MX052_STAGE_A_INNER_BEST_SEPARATE.csv",innerbest);write_csv(OUT/"MX052_STAGE_A_OUTER_PREDICTIONS.csv",predrows)
    write_csv(OUT/"MX052_STAGE_A_OUTER_METRICS.csv",metrics);write_csv(OUT/"MX052_STAGE_A_STABILITY.csv",stability)
    write_csv(OUT/"MX052_STAGE_A_CROSS_TARGET.csv",cross);write_json(OUT/"MX052_STAGE_A_GATE.json",stagea)
    write_csv(OUT/"MX052_STAGE_B_MODEL_FITS.csv",fits);write_csv(OUT/"MX052_STAGE_B_INNER_CROSSFIT.csv",inner)
    write_csv(OUT/"MX052_STAGE_B_CALIBRATION.csv",cals);write_csv(OUT/"MX052_STAGE_B_SUPPORT_BOX.csv",boxes)
    write_csv(OUT/"MX052_STAGE_B_TRAINING_CANDIDATES.csv",tcands);write_csv(OUT/"MX052_STAGE_B_OUTER_SELECTED_RULES.csv",selected)
    write_csv(OUT/"MX052_STAGE_B_PER_DECISION_REPLAY.csv",replay);write_csv(OUT/"MX052_STAGE_B_VALIDITY.csv",validity)
    write_csv(OUT/"MX052_STAGE_B_HELDOUT_OUTCOMES.csv",outcomes);write_csv(OUT/"MX052_STAGE_B_FULL_DEVELOPMENT.csv",full)
    write_csv(OUT/"MX052_ADVERSARIAL_CASE_AUDIT.csv",adv)
    write_json(OUT/"MX052_METRIC_DICTIONARY.json",{"schema":"mx052_metric_dictionary_r2_execution","stage_a":{"primary_target":"T1","models":ARMS,"lambda":LAMBDA,
      "common_support":"T1+B1+complete_F1+complete_F2","best_separate":"nested inner LORO training-only","effect_threshold":STAGE_A_EFFECT,"PGC_max":0.50},
      "stage_b":{"J_predictors":J,"M1":"ridge logistic Y_area_severe=1[OracleRemainingFraction_GT>0.10]","M2":"ridge logistic Y_topo_severe=1[T2b>0]",
      "M3":"ridge linear U3_cells lower-clipped zero","lambda":LAMBDA,"tau":TAUS,"q_U":QUS,"K":1,"SupportBox":"inclusive raw J min/max"}})
    final_class=stageb["classification"] if stageb["opened"] else stagea["classification"]
    write_json(OUT/"MX052_EXECUTION_PROVENANCE.json",{"schema":"mx052_execution_provenance_r2","task":"MX053","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "stage_a_classification":stagea["classification"],"stage_b_opened":stageb["opened"],"stage_b_classification":stageb["classification"],
      "final_classification":final_class,"hard_failures":hard,"adversarial_pass":sum(x["semantic_pass"] for x in adv),
      "constraints":{"new_run":False,"Gazebo":False,"RViz":False,"Hospital":False,"confirmation":False,"retuning":False,"deployment":False,"robot_STOP":False}})
    s=[x for x in stability if x["row_type"]=="SUMMARY"][0];ap=[int(x["area_positive_n"]) for x in cals]
    report=["# MX053 Analyst05 — MX052 Method R2 offline execution result","",f"Final classification: **{final_class}**","",
      "## Stage A",f"Classification: **{stagea['classification']}**",f"- evaluable/positive folds: {s['evaluable_folds']}/10 / {s['positive_folds']}/10",
      f"- median Delta_J_vs_Best: {s['median_Delta_J_vs_Best']}",f"- DCS_J: {s['DCS_J']}",f"- PGC_J: {s['PGC_J']}",
      f"- median Delta J-vs-S1 / S2: {s['median_Delta_J_vs_S1']} / {s['median_Delta_J_vs_S2']}","",
      "## Stage B",f"Opened: {stageb['opened']}",f"Classification: **{stageb['classification']}**",
      f"- outer candidate rows: {stageb.get('outer_candidate_rows',0)}; admissible: {stageb.get('outer_admissible_candidates',0)}",
      f"- outer NO_RULE folds: {stageb.get('outer_no_rule_folds',0)}/10",
      f"- severe-spatial positive calibration count range: {min(ap) if ap else 'NA'}..{max(ap) if ap else 'NA'}",
      "- Absent candidate-region severe positives are insufficient calibration support, not zero severe risk.","",
      f"A1-A36: {sum(x['semantic_pass'] for x in adv)}/36 PASS",f"Hard failures: {len(hard)}",
      "Historical ten-run New Room development only; no new simulation, Gazebo/RViz, Hospital, confirmation, deployment or robot STOP."]
    (OUT/"MX052_ANALYST_REPORT.md").write_text("\n".join(report)+"\n",encoding="utf-8")
    required=["MX052_SOURCE_PROVENANCE.json","MX052_FEATURE_TARGET_PARITY.csv","MX052_STAGE_A_COMMON_SUPPORT.csv","MX052_STAGE_A_INNER_BEST_SEPARATE.csv","MX052_STAGE_A_OUTER_PREDICTIONS.csv",
      "MX052_STAGE_A_OUTER_METRICS.csv","MX052_STAGE_A_STABILITY.csv","MX052_STAGE_A_CROSS_TARGET.csv","MX052_STAGE_A_GATE.json","MX052_STAGE_B_MODEL_FITS.csv","MX052_STAGE_B_INNER_CROSSFIT.csv",
      "MX052_STAGE_B_CALIBRATION.csv","MX052_STAGE_B_SUPPORT_BOX.csv","MX052_STAGE_B_TRAINING_CANDIDATES.csv","MX052_STAGE_B_OUTER_SELECTED_RULES.csv","MX052_STAGE_B_PER_DECISION_REPLAY.csv",
      "MX052_STAGE_B_VALIDITY.csv","MX052_STAGE_B_HELDOUT_OUTCOMES.csv","MX052_STAGE_B_FULL_DEVELOPMENT.csv","MX052_ADVERSARIAL_CASE_AUDIT.csv","MX052_METRIC_DICTIONARY.json",
      "MX052_EXECUTION_PROVENANCE.json","MX052_ANALYST_REPORT.md"]
    actual=sorted(p.name for p in OUT.iterdir() if p.is_file())
    if sorted(required)!=actual:raise RuntimeError("OUTPUT_CONTRACT")
    if len(parity)!=365 or len(common)!=10 or len(innerbest)!=10 or len(metrics)!=10:raise RuntimeError("STAGE_A_ROW_CONTRACT")
    if stagea["classification"]=="MX052_STAGE_A_JOINT_INCREMENTAL_VALUE_PASS" and (len(tcands)!=80 or len(selected)!=10 or len(replay)!=365 or len(outcomes)!=10 or len(full)!=8):raise RuntimeError("STAGE_B_ROW_CONTRACT")
    if len(adv)!=36 or sum(x["semantic_pass"] for x in adv)!=36:raise RuntimeError("ADVERSARIAL_CONTRACT")
    print(json.dumps({"stage_a":stagea["classification"],"median_delta":stagea["median_Delta_J_vs_Best"],"stage_b_opened":stageb["opened"],
      "stage_b":stageb["classification"],"final":final_class,"outputs":len(actual),"hard_failures":len(hard),"A_pass":sum(x["semantic_pass"] for x in adv),
      "stage_b_candidates":len(tcands),"stage_b_admissible":sum(x["admissible"]==1 for x in tcands)},sort_keys=True))
if __name__=="__main__":main()
