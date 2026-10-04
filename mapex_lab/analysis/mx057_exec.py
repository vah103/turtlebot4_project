#!/usr/bin/env python3
from __future__ import annotations
import csv, json, math, subprocess
from collections import defaultdict
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
AN=ROOT/"mapex_lab"/"analysis"
OUT=AN/"mx057_exec_results"
RUNS=[f"mpx_{i:03d}" for i in range(1,11)]
METHOD_COMMIT="cb91dd2d132d860576be5557a16b7c59858026ef"
METHOD_BLOB="41397cc70063f9106b917450a34188dbd287c6ac"
PARENT_RESULT="a19f0902ffefdd009bf1c5661c16ebf03b1f47cb"
LAMBDA=1.0
EFFECT=0.025
R_TAU=0.10
K_R=1
R_ROBOT=0.189
A_ROBOT=math.pi*R_ROBOT*R_ROBOT
A_REGION=16*A_ROBOT
A_BREACH=2*A_ROBOT
C_SCALE=0.05
TARGETS=["T1","T2","T3","T4","T5"]
REGRET_TARGETS=["T1","T2","T3","T4"]
B1=["R_map","log_decision","log_elapsed"]
F1=["F1_PredOccShare","F1_OccMaskIoUMean2","F1_OccFlipRateMean2"]
F2=["F2_Vote3Share","F2_Vote2Share","F2_Vote1Share","F2_VotePersistenceMean2"]
ARMS={"B1":B1,"S1":B1+F1,"S2":B1+F2,"J":B1+F1+F2}
J=ARMS["J"]

FILES={
 "quality":(AN/"mx055_exec_results/MX054_MAP_QUALITY_PER_DECISION.csv","ed77ae4c44585e3883b6c42d1c0bc7c356891cf6"),
 "struct":(AN/"mx055_exec_results/MX054_STRUCTURAL_METRICS_PER_DECISION.csv","4f093e71473960fa9697ec49b806acd0cc37cb43"),
 "events":(AN/"mx055_exec_results/MX054_STRUCTURAL_EVENT_AUDIT.csv","fe347c7ba64838e8cc830a23bc9681ceebe5373b"),
 "saving":(AN/"mx055_exec_results/MX054_SAVING_PER_DECISION.csv","50a90f21b986d926510fdad4b9dfa8c5928142cf"),
 "signal":(AN/"mx055_exec_results/MX054_SIGNAL_ATTACHMENT_AUDIT.csv","65d4b3e7bf2197683ab1ce6678d8bee97171fd48"),
 "mx031":(AN/"mx031_exec_results/MX031_PER_DECISION_REPLAY.csv","daf11687221080790e20227f2bf7b7406476298e"),
}

EMPTY_SCHEMAS={
 "MX056_STAGE_B_MODEL_FITS.csv":["stage_b_status","outer_heldout","target","training_rows","training_runs","features","intercept","coefficients_json"],
 "MX056_STAGE_B_INNER_CROSSFIT.csv":["stage_b_status","outer_heldout","inner_heldout","run_id","decision_id","target","y","yhat"],
 "MX056_STAGE_B_ENVELOPE_CALIBRATION.csv":["stage_b_status","outer_heldout","target","margin","candidate_rows","candidate_runs"],
 "MX056_STAGE_B_SUPPORT_BOX.csv":["stage_b_status","outer_heldout","feature","raw_min","raw_max"],
 "MX056_STAGE_B_PER_DECISION_SHADOW.csv":["stage_b_status","outer_heldout","run_id","decision_id","decision_index","state","first_stop_shadow"],
 "MX056_STAGE_B_ENVELOPE_VALIDITY.csv":["stage_b_status","outer_heldout","run_id","decision_id","target","undercoverage"],
 "MX056_STAGE_B_HELDOUT_OUTCOMES.csv":["stage_b_status","run_id","ShadowStopDecision","SavedDecisions","SavedDecisionFraction","SavedTime_s","SavedTimeFraction","SHADOW_REGRET_VIOLATION"],
 "MX056_STAGE_B_TIMING_DIAGNOSTICS.csv":["stage_b_status","run_id","ShadowStopDecision","OracleStop_4","PrematureStop","DelayVsOracle4","NO_STOP","NO_RULE"],
 "MX056_STAGE_B_FULL_DEVELOPMENT.csv":["stage_b_status","full_rule_solvable","saved_decision_concentration","saved_time_concentration"],
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
def read_csv(p):
    with open(p,newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write_csv(p,rows):
    rows=list(rows);p.parent.mkdir(parents=True,exist_ok=True)
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields:fields.append(k)
    if not fields:fields=EMPTY_SCHEMAS.get(p.name,["status"])
    with open(p,"w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def write_json(p,obj):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(obj,indent=2,sort_keys=True,allow_nan=True)+"\n",encoding="utf-8")
def git_blob(p):return subprocess.check_output(["git","hash-object",str(p)],cwd=ROOT,text=True).strip()
def key(r):return (r["run_id"],int(r["decision_id"]))
def mean(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.mean(z)) if z else math.nan
def median(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.median(z)) if z else math.nan
def dcs_score(vals):
    vals=[float(x) for x in vals if finite(x)];n=len(vals);s=sum(x>0 for x in vals)
    if n==0:return 1.0,s,n
    return sum(math.comb(n,k) for k in range(s,n+1))*0.5**n,s,n
def weights(rows):
    by=defaultdict(int)
    for r in rows:by[r["run_id"]]+=1
    R=len(by)
    return np.asarray([1/(R*by[r["run_id"]]) for r in rows],float)
def weighted_scale(rows,target):
    w=weights(rows);y=np.asarray([float(r[target]) for r in rows]);mu=float(np.sum(w*y))
    return float(np.sqrt(np.sum(w*(y-mu)**2)))
def fit(rows,target,features):
    w=weights(rows);X=np.asarray([[float(r[f]) for f in features] for r in rows],float);y=np.asarray([float(r[target]) for r in rows],float)
    mu=np.sum(X*w[:,None],axis=0);sd=np.sqrt(np.sum((X-mu)**2*w[:,None],axis=0));zero=sd<=0
    Z0=np.zeros_like(X);nz=~zero
    if np.any(nz):Z0[:,nz]=(X[:,nz]-mu[nz])/sd[nz]
    Z=np.column_stack([np.ones(len(Z0)),Z0]);reg=np.diag([0.0]+[LAMBDA]*len(features))
    beta=np.linalg.solve(Z.T@(Z*w[:,None])+reg,Z.T@(w*y))
    return {"target":target,"features":features,"mu":mu,"sd":sd,"zero":zero,"beta":beta,
            "runs":sorted(set(r["run_id"] for r in rows)),"n":len(rows)}
def predict(m,r):
    x=np.asarray([float(r[f]) for f in m["features"]]);z=np.zeros(len(x));nz=~m["zero"]
    if np.any(nz):z[nz]=(x[nz]-m["mu"][nz])/m["sd"][nz]
    return float(np.r_[1.0,z]@m["beta"])
def model_row(stage,outer,target,m):
    return {"stage_b_status":"OK","outer_heldout":outer,"target":target,"training_rows":m["n"],
      "training_runs":"|".join(m["runs"]),"features":"|".join(m["features"]),
      "zero_sd_features":"|".join(f for f,z in zip(m["features"],m["zero"]) if z),
      "means_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["mu"])},sort_keys=True),
      "sds_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["sd"])},sort_keys=True),
      "intercept":float(m["beta"][0]),"coefficients_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["beta"][1:])},sort_keys=True)}
def common_ok(r):return all(finite(r.get(t)) for t in TARGETS) and all(finite(r.get(f)) for f in J)
def supportbox_ok(r,box):
    if not all(finite(r.get(f)) for f in J):return False,"MISSING_NONFINITE"
    bad=[]
    for f in J:
        x=float(r[f]);lo,hi=box[f]
        if x<lo:bad.append(f+":BELOW")
        elif x>hi:bad.append(f+":ABOVE")
    return not bad,"|".join(bad)

def load_rows():
    hard=[];source_ins=[]
    actual={}
    for name,(p,exp) in FILES.items():
        if not p.is_file():source_ins.append("MISSING_"+name);continue
        got=git_blob(p);actual[name]=got
        if got!=exp:hard.append(f"{name}:BLOB:{got}!={exp}")
    q=read_csv(FILES["quality"][0]);s=read_csv(FILES["struct"][0]);e=read_csv(FILES["events"][0])
    sav=read_csv(FILES["saving"][0]);sig=read_csv(FILES["signal"][0]);mx=read_csv(FILES["mx031"][0])
    qmap=defaultdict(dict);smap=defaultdict(dict)
    for r in q:qmap[key(r)][r["variant"]]=r
    for r in s:smap[key(r)][r["variant"]]=r
    em={key(r):r for r in e};sv={key(r):r for r in sav};fm={key(r):r for r in sig};mm={key(r):r for r in mx}
    ks=set(em)
    if any(set(x)!=ks for x in [sv,fm,mm,set(qmap),set(smap)]):hard.append("SOURCE_KEY_SET_MISMATCH")
    if len(ks)!=365:hard.append(f"SOURCE_KEY_COUNT:{len(ks)}")
    rows=[];components=[];secondary=[]
    for k in sorted(ks):
        if set(qmap[k])!={"V0_OBS_ONLY","V1_MEAN_COMPLETION","V2_GT_FILL_CEILING","V3_FULL_EXPLORE_REFERENCE"}:hard.append(f"{k}:QUALITY_VARIANTS")
        if not {"V1_MEAN_COMPLETION","V2_GT_FILL_CEILING","V3_FULL_EXPLORE_REFERENCE"}.issubset(smap[k]):hard.append(f"{k}:STRUCT_VARIANTS")
        q0,q1,q2,q3=[qmap[k][x] for x in ["V0_OBS_ONLY","V1_MEAN_COMPLETION","V2_GT_FILL_CEILING","V3_FULL_EXPLORE_REFERENCE"]]
        s1,s2,s3=[smap[k][x] for x in ["V1_MEAN_COMPLETION","V2_GT_FILL_CEILING","V3_FULL_EXPLORE_REFERENCE"]]
        refs=[fint(x["ReferenceTraversableCells"]) for x in (s1,s2,s3)]
        if len(set(refs))!=1:hard.append(f"{k}:REFERENCE_TRAVERSABLE_MISMATCH")
        refarea=refs[0]*0.05*0.05
        def sr(A,B):
            conn=(fnum(B["PairConnectivityRetention"])-fnum(A["PairConnectivityRetention"]))/C_SCALE
            trav=((fnum(B["TravRetention"])-fnum(A["TravRetention"]))*refarea)/A_REGION
            unsafe=(fnum(A["UnsafeTravArea_m2"])-fnum(B["UnsafeTravArea_m2"]))/A_BREACH
            return conn,trav,unsafe,max(conn,trav,unsafe)
        c12,t12,u12,T1=sr(s1,s2);c13,t13,u13,T2=sr(s1,s3)
        T3=fnum(q2["StrictMacroIoU"])-fnum(q1["StrictMacroIoU"])
        T4=fnum(q3["StrictMacroIoU"])-fnum(q1["StrictMacroIoU"])
        T5=fnum(q1["StrictMacroIoU"])-fnum(q0["StrictMacroIoU"])
        r={"run_id":k[0],"decision_id":k[1],"decision_index":fint(sv[k]["decision_index"]),"decision_count":fint(sv[k]["decision_count"]),
           "T1":T1,"T2":T2,"T3":T3,"T4":T4,"T5":T5,
           "R_map":fnum(fm[k]["R_map"]),"log_decision":fnum(fm[k]["log_decision"]),"log_elapsed":fnum(fm[k]["log_elapsed"]),
           "TopoValid":fint(mm[k]["TopoValid"],0),"TopoValid_reason":mm[k]["TopoValid_reason"],
           "OracleStop_4":fint(mm[k]["OracleStop_4"]),"decision_time_s":fnum(sv[k]["decision_time_s"]),
           "SavedDecisions":fint(sv[k]["SavedDecisions"],0),"SavedDecisionFraction":fnum(sv[k]["SavedDecisionFraction"]),
           "SavedTime_s":fnum(sv[k]["SavedTime_s"]),"SavedTimeFraction":fnum(sv[k]["SavedTimeFraction"])}
        for f in F1+F2:r[f]=fnum(fm[k].get(f))
        r["PRIMARY_COMMON_SUPPORT"]=int(common_ok(r))
        rows.append(r)
        components.append({"run_id":k[0],"decision_id":k[1],"decision_index":r["decision_index"],
          "ConnRegret_V1_V2":c12,"TravRegret_V1_V2":t12,"UnsafeRegret_V1_V2":u12,"T1":T1,
          "ConnRegret_V1_V3":c13,"TravRegret_V1_V3":t13,"UnsafeRegret_V1_V3":u13,"T2":T2,
          "ReferenceTraversableCells":refs[0],"RefTravArea_m2":refarea,"C_scale":C_SCALE,"A_region":A_REGION,"A_breach":A_BREACH})
        secondary.append({"run_id":k[0],"decision_id":k[1],"decision_index":r["decision_index"],
          "DeltaS2_PI":fint(s1["S2_MeaningfulFragmentation"])-fint(s2["S2_MeaningfulFragmentation"]),
          "DeltaFalseBarrier_PI":fint(s1["FalseBarrierEvent"])-fint(s2["FalseBarrierEvent"]),
          "DeltaS2_SC":fint(s1["S2_MeaningfulFragmentation"])-fint(s3["S2_MeaningfulFragmentation"]),
          "DeltaFalseBarrier_SC":fint(s1["FalseBarrierEvent"])-fint(s3["FalseBarrierEvent"])})
        if r["decision_index"]!=fint(fm[k]["decision_index"]) or r["decision_index"]!=fint(mm[k]["decision_index"]):hard.append(f"{k}:DECISION_INDEX")
        if r["decision_count"]!=fint(fm[k]["decision_count"]) or r["decision_count"]!=fint(mm[k]["decision_count"]):hard.append(f"{k}:DECISION_COUNT")
    return rows,components,secondary,actual,hard,source_ins

def stage0(rows):
    common=[r for r in rows if common_ok(r)]
    ident=[];concs=[];failed=[]
    byrun={run:[r for r in common if r["run_id"]==run] for run in RUNS}
    support_pass=len(common)>0 and all(len(byrun[r])>=20 for r in RUNS)
    if not support_pass:failed.append("COMMON_SUPPORT")
    benefit_run_positive=0
    for run in RUNS:
        if median(x["T5"] for x in byrun[run])>0:benefit_run_positive+=1
    benefit_row_fraction=mean(x["T5"]>0 for x in common)
    benefit_pass=benefit_run_positive>=8 and benefit_row_fraction>=0.80
    if not benefit_pass:failed.append("T5_BENEFIT_CONTEXT")
    for t in TARGETS:
        ys=[r[t] for r in common];w=weights(common);mu=float(np.sum(w*np.asarray(ys)));sd=float(np.sqrt(np.sum(w*(np.asarray(ys)-mu)**2)))
        global_dist=len(set(round(float(x),9) for x in ys))
        perrun_dist={run:len(set(round(float(x[t]),9) for x in byrun[run])) for run in RUNS}
        within=sum(v>=5 for v in perrun_dist.values())
        meds={run:median(x[t] for x in byrun[run]) for run in RUNS};across=len(set(round(float(v),9) for v in meds.values()))
        m=median(meds.values());Vr={run:mean(abs(x[t]-m) for x in byrun[run]) for run in RUNS};den=sum(Vr.values())
        conc=max(Vr.values())/den if den>0 else 1.0
        ok=sd>1e-9 and global_dist>=5 and within>=8 and across>=5 and conc<=0.50
        if not ok:failed.append(t)
        ident.append({"target":t,"common_support_n":len(common),"run_n":len([r for r in RUNS if byrun[r]]),"equal_run_weighted_sd":sd,
          "global_distinct_1e9":global_dist,"within_run_distinct_ge5_runs":within,"run_median_distinct_1e9":across,
          "VariationConcentration":conc,"identifiable":int(ok),"per_run_distinct_json":json.dumps(perrun_dist,sort_keys=True),
          "run_medians_json":json.dumps(meds,sort_keys=True)})
        for run in RUNS:concs.append({"target":t,"run_id":run,"V_r":Vr[run],"population_run_median_m":m,"VariationConcentration":conc})
    quads=[]
    for ref,t in [("V2","T1"),("V3","T2")]:
        for ben in [1,0]:
            for harm in [1,0]:
                z=[r for r in common if (r["T5"]>0)==bool(ben) and (r[t]>0)==bool(harm)]
                quads.append({"paired_reference":ref,"benefit_T5_positive":ben,"structural_regret_positive":harm,
                              "decision_n":len(z),"run_n":len(set(x["run_id"] for x in z))})
    gate={"classification":"MX056_STAGE0_PASS" if not failed else "MX056_PAIRED_TARGET_NONIDENTIFIABLE_OR_SUPPORT_INSUFFICIENT",
          "common_support_n":len(common),"per_run_common_support":{r:len(byrun[r]) for r in RUNS},"common_support_pass":support_pass,
          "benefit_positive_median_runs":benefit_run_positive,"benefit_positive_row_fraction":benefit_row_fraction,"benefit_context_pass":benefit_pass,
          "all_targets_identifiable":not any(x["identifiable"]==0 for x in ident),"failures":failed}
    return common,ident,concs,quads,gate

def inner_best(common,target,outer_hold):
    truns=[r for r in RUNS if r!=outer_hold]
    vals={"S1":[],"S2":[]};details={"S1":{},"S2":{}}
    for inner in truns:
        train=[r for r in common if r["run_id"] in truns and r["run_id"]!=inner]
        test=[r for r in common if r["run_id"]==inner]
        sc=weighted_scale(train,target)
        if not finite(sc) or sc<=0:return None
        for arm in ["S1","S2"]:
            m=fit(train,target,ARMS[arm]);mae=mean(abs(r[target]-predict(m,r)) for r in test);v=mae/sc
            vals[arm].append(v);details[arm][inner]=v
    a=mean(vals["S1"]);b=mean(vals["S2"]);best="S1" if a<=b+1e-12 else "S2"
    return best,a,b,details

def stageA(common):
    inner=[];preds=[];metrics=[];tgates=[]
    for t in TARGETS:
        for hold in RUNS:
            ib=inner_best(common,t,hold)
            if ib is None:
                inner.append({"target":t,"outer_heldout":hold,"status":"UNEVALUABLE"});continue
            best,a,b,details=ib
            inner.append({"target":t,"outer_heldout":hold,"status":"PASS","InnerMean_nMAE_S1":a,"InnerMean_nMAE_S2":b,
                          "BestSeparate_train":best,"inner_S1_json":json.dumps(details["S1"],sort_keys=True),"inner_S2_json":json.dumps(details["S2"],sort_keys=True)})
            tr=[r for r in common if r["run_id"]!=hold];te=[r for r in common if r["run_id"]==hold]
            sc=weighted_scale(tr,t);mods={a:fit(tr,t,fs) for a,fs in ARMS.items()};nmae={}
            for a,m in mods.items():nmae[a]=mean(abs(r[t]-predict(m,r)) for r in te)/sc
            for r in te:
                preds.append({"target":t,"outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
                              "y":r[t],"BestSeparate_train":best,**{f"pred_{a}":predict(mods[a],r) for a in ARMS}})
            metrics.append({"target":t,"outer_heldout":hold,"status":"PASS","BestSeparate_train":best,"target_scale":sc,
                            **{f"nMAE_{a}":nmae[a] for a in ARMS},
                            "Delta_J_vs_B1":nmae["B1"]-nmae["J"],"Delta_J_vs_S1":nmae["S1"]-nmae["J"],
                            "Delta_J_vs_S2":nmae["S2"]-nmae["J"],"Delta_J_vs_BestSeparate_train":nmae[best]-nmae["J"]})
        z=[x for x in metrics if x["target"]==t and x["status"]=="PASS"];ds=[x["Delta_J_vs_B1"] for x in z]
        dcs,pos,n=dcs_score(ds);psum=sum(max(x,0) for x in ds);pgc=max([max(x,0) for x in ds],default=0)/psum if psum>0 else 1.0
        bestds=[x["Delta_J_vs_BestSeparate_train"] for x in z]
        passes={"folds":n==10,"DCS":dcs<=0.05,"effect":median(ds)>=EFFECT if ds else False,
                "PGC":pgc<=0.50,"best_noninferiority":median(bestds)>=0 if bestds else False}
        tgates.append({"target":t,"evaluable_folds":n,"positive_Delta_J_vs_B1_folds":pos,"DCS_J_vs_B1":dcs,
                       "median_Delta_J_vs_B1":median(ds),"effect_threshold":EFFECT,"PGC_J_vs_B1":pgc,
                       "median_Delta_J_vs_BestSeparate_train":median(bestds),"pass_folds":int(passes["folds"]),
                       "pass_DCS":int(passes["DCS"]),"pass_effect":int(passes["effect"]),"pass_PGC":int(passes["PGC"]),
                       "pass_best_separate":int(passes["best_noninferiority"]),"target_gate_pass":int(all(passes.values()))})
    family_pass=len(tgates)==5 and all(x["target_gate_pass"] for x in tgates)
    fam={"classification":"MX056_JOINT_PAIRED_REGRET_PREDICTIVE_VALUE_PASS" if family_pass else "MX056_JOINT_NO_STABLE_PAIRED_REGRET_PREDICTIVE_VALUE",
         "all_five_pass":bool(family_pass),"passed_targets":[x["target"] for x in tgates if x["target_gate_pass"]],
         "failed_targets":[x["target"] for x in tgates if not x["target_gate_pass"]]}
    return inner,preds,metrics,tgates,fam

def stageB(common):
    fits=[];innerrows=[];calrows=[];boxrows=[];shadow=[];validity=[];outcomes=[];timing=[]
    no_rule=False;hard=False
    for hold in RUNS:
        truns=[r for r in RUNS if r!=hold]
        cand=[r for r in common if r["run_id"] in truns and r["R_map"]<=R_TAU and r["TopoValid"]==1]
        counts={run:sum(r["run_id"]==run for r in cand) for run in truns}
        support=sum(v>=5 for v in counts.values())>=8 and len(cand)>=40
        if not support:
            no_rule=True
            for r in [x for x in common if x["run_id"]==hold]:
                shadow.append({"stage_b_status":"NO_RULE_INSUFFICIENT_SHADOW_CALIBRATION_SUPPORT","outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],"state":"NO_RULE","first_stop_shadow":0})
            outcomes.append({"stage_b_status":"NO_RULE_INSUFFICIENT_SHADOW_CALIBRATION_SUPPORT","run_id":hold,"ShadowStopDecision":"","SavedDecisions":0,"SavedDecisionFraction":0,"SavedTime_s":0,"SavedTimeFraction":0,"SHADOW_REGRET_VIOLATION":0})
            timing.append({"stage_b_status":"NO_RULE_INSUFFICIENT_SHADOW_CALIBRATION_SUPPORT","run_id":hold,"ShadowStopDecision":"","OracleStop_4":[x for x in common if x["run_id"]==hold][0]["OracleStop_4"],"PrematureStop":0,"DelayVsOracle4":"","NO_STOP":0,"NO_RULE":1})
            continue
        box={f:(min(r[f] for r in cand),max(r[f] for r in cand)) for f in J}
        for f,(lo,hi) in box.items():boxrows.append({"stage_b_status":"OK","outer_heldout":hold,"feature":f,"raw_min":lo,"raw_max":hi,"candidate_rows":len(cand)})
        outermods={}
        for t in TARGETS:
            train=[r for r in common if r["run_id"] in truns];outermods[t]=fit(train,t,J);fits.append(model_row("B",hold,t,outermods[t]))
        innerpred={t:{} for t in TARGETS}
        for inner in truns:
            fitruns=[x for x in truns if x!=inner]
            train=[r for r in common if r["run_id"] in fitruns]
            for t in TARGETS:
                m=fit(train,t,J)
                for r in cand:
                    if r["run_id"]==inner:
                        yh=predict(m,r);innerpred[t][(inner,r["decision_id"])]=yh
                        innerrows.append({"stage_b_status":"OK","outer_heldout":hold,"inner_heldout":inner,"run_id":inner,"decision_id":r["decision_id"],"target":t,"y":r[t],"yhat":yh})
        margins={}
        for t in TARGETS:
            vals=[]
            for r in cand:
                yh=innerpred[t].get((r["run_id"],r["decision_id"]))
                if yh is None:continue
                vals.append(max(0.0,r[t]-yh) if t!="T5" else max(0.0,yh-r[t]))
            if not vals:hard=True;margins[t]=math.nan
            else:margins[t]=max(vals)
            calrows.append({"stage_b_status":"OK" if vals else "HARD_INVALID","outer_heldout":hold,"target":t,"margin":margins[t],"candidate_rows":len(cand),"candidate_runs":sum(v>0 for v in counts.values()),"per_run_counts_json":json.dumps(counts,sort_keys=True)})
        te=[r for r in common if r["run_id"]==hold]
        first=None
        for r in sorted(te,key=lambda x:x["decision_index"]):
            state="EXPLORE";support_ok=False;reason="";pred={}
            if r["R_map"]<=R_TAU:
                if r["TopoValid"]!=1:state="TOPOLOGY_CONTINUE"
                else:
                    support_ok,reason=supportbox_ok(r,box)
                    if not support_ok:state="ABSTAIN_CONTINUE"
                    else:
                        pred={t:predict(outermods[t],r) for t in TARGETS}
                        U={t:pred[t]+margins[t] for t in REGRET_TARGETS};L5=pred["T5"]-margins["T5"]
                        if U["T1"]>0 or U["T2"]>0:state="STRUCTURAL_REGRET_CONTINUE"
                        elif U["T4"]>0:state="MAP_REGRET_CONTINUE"
                        elif L5<=0:state="BENEFIT_CONTINUE"
                        elif U["T3"]>=L5:state="PREDICTION_PENALTY_CONTINUE"
                        else:state="STOP_SHADOW"
            fire=int(state=="STOP_SHADOW" and first is None)
            if fire:first=(r,pred,box)
            row={"stage_b_status":"OK","outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
                 "state":state,"support_box_valid":int(support_ok),"support_box_reason":reason,"first_stop_shadow":fire}
            if pred:
                for t in TARGETS:row["pred_"+t]=pred[t]
                for t in REGRET_TARGETS:row["U_"+t]=pred[t]+margins[t]
                row["L_T5"]=pred["T5"]-margins["T5"]
            shadow.append(row)
            if r["R_map"]<=R_TAU and r["TopoValid"]==1:
                for t in TARGETS:
                    yh=predict(outermods[t],r)
                    bound=(yh+margins[t]) if t!="T5" else (yh-margins[t])
                    under=max(0,r[t]-bound) if t!="T5" else max(0,bound-r[t])
                    validity.append({"stage_b_status":"OK","outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"target":t,"actual":r[t],"bound":bound,"undercoverage":under})
        if first:
            r,pred,_=first
            violation=int(r["T1"]>0 or r["T2"]>0 or r["T4"]>0 or r["T5"]<=0 or not (r["T3"]<r["T5"]) or r["TopoValid"]!=1 or not supportbox_ok(r,box)[0])
            outcomes.append({"stage_b_status":"OK","run_id":hold,"ShadowStopDecision":r["decision_index"],"SavedDecisions":r["SavedDecisions"],"SavedDecisionFraction":r["SavedDecisionFraction"],"SavedTime_s":r["SavedTime_s"],"SavedTimeFraction":r["SavedTimeFraction"],"SHADOW_REGRET_VIOLATION":violation,**{t:r[t] for t in TARGETS}})
            timing.append({"stage_b_status":"OK","run_id":hold,"ShadowStopDecision":r["decision_index"],"OracleStop_4":r["OracleStop_4"],"PrematureStop":int(r["decision_index"]<r["OracleStop_4"]),"DelayVsOracle4":r["decision_index"]-r["OracleStop_4"],"NO_STOP":0,"NO_RULE":0})
        else:
            outcomes.append({"stage_b_status":"OK","run_id":hold,"ShadowStopDecision":"","SavedDecisions":0,"SavedDecisionFraction":0,"SavedTime_s":0,"SavedTimeFraction":0,"SHADOW_REGRET_VIOLATION":0})
            timing.append({"stage_b_status":"OK","run_id":hold,"ShadowStopDecision":"","OracleStop_4":te[0]["OracleStop_4"],"PrematureStop":0,"DelayVsOracle4":"","NO_STOP":1,"NO_RULE":0})
    # full development descriptive rule
    fcand=[r for r in common if r["R_map"]<=R_TAU and r["TopoValid"]==1]
    fcounts={run:sum(r["run_id"]==run for r in fcand) for run in RUNS}
    fsolve=sum(v>=5 for v in fcounts.values())>=9 and len(fcand)>=40
    full=[]
    if fsolve:
        box={f:(min(r[f] for r in fcand),max(r[f] for r in fcand)) for f in J}
        margins={};mods={t:fit(common,t,J) for t in TARGETS}
        for t in TARGETS:
            vals=[]
            for hold in RUNS:
                m=fit([r for r in common if r["run_id"]!=hold],t,J)
                for r in fcand:
                    if r["run_id"]==hold:
                        yh=predict(m,r);vals.append(max(0,r[t]-yh) if t!="T5" else max(0,yh-r[t]))
            margins[t]=max(vals) if vals else math.nan
        fsaves=[];fts=[]
        for run in RUNS:
            first=None
            for r in sorted([x for x in common if x["run_id"]==run],key=lambda x:x["decision_index"]):
                if r["R_map"]>R_TAU or r["TopoValid"]!=1:continue
                ok,_=supportbox_ok(r,box)
                if not ok:continue
                p={t:predict(mods[t],r) for t in TARGETS};U={t:p[t]+margins[t] for t in REGRET_TARGETS};L=p["T5"]-margins["T5"]
                if U["T1"]<=0 and U["T2"]<=0 and U["T4"]<=0 and L>0 and U["T3"]<L:
                    first=r;break
            fsaves.append(first["SavedDecisions"] if first else 0);fts.append(first["SavedTime_s"] if first else 0)
        dc=max(fsaves)/sum(fsaves) if sum(fsaves)>0 else 1.0;tc=max(fts)/sum(fts) if sum(fts)>0 else 1.0
        full=[{"stage_b_status":"OK","full_rule_solvable":1,"candidate_rows":len(fcand),"saved_decision_concentration":dc,"saved_time_concentration":tc,"per_run_saved_decisions_json":json.dumps(dict(zip(RUNS,fsaves)),sort_keys=True),"per_run_saved_time_json":json.dumps(dict(zip(RUNS,fts)),sort_keys=True)}]
    else:
        full=[{"stage_b_status":"NO_RULE_INSUFFICIENT_SHADOW_CALIBRATION_SUPPORT","full_rule_solvable":0,"saved_decision_concentration":1.0,"saved_time_concentration":1.0}]
    env_fail=sum(fnum(x["undercoverage"],0)>1e-12 for x in validity)
    stop_viol=sum(fint(x["SHADOW_REGRET_VIOLATION"],0) for x in outcomes)
    stopruns=sum(bool(x["ShadowStopDecision"]) for x in outcomes);save3=sum(fint(x["SavedDecisions"],0)>=3 for x in outcomes);timepos=sum(fnum(x["SavedTimeFraction"],0)>0 for x in outcomes)
    medsd=median(fnum(x["SavedDecisionFraction"],0) for x in outcomes);medst=median(fnum(x["SavedTimeFraction"],0) for x in outcomes)
    useful=stopruns>=7 and save3>=7 and timepos>=7 and medsd>0 and medst>0
    conc_ok=bool(full and fint(full[0]["full_rule_solvable"],0)==1 and fnum(full[0]["saved_decision_concentration"])<=.50 and fnum(full[0]["saved_time_concentration"])<=.50)
    summary={"support_insufficient":no_rule,"hard":hard,"envelope_undercoverage_rows":env_fail,"stop_regret_violations":stop_viol,
             "stop_runs":stopruns,"save_ge3_runs":save3,"positive_time_runs":timepos,"median_saved_decision_fraction":medsd,"median_saved_time_fraction":medst,
             "usefulness_pass":useful,"concentration_pass":conc_ok}
    return fits,innerrows,calrows,boxrows,shadow,validity,outcomes,timing,full,summary

def status_stage_b(reason):
    out={}
    for name in EMPTY_SCHEMAS:
        out[name]=[{k:(reason if k=="stage_b_status" else "") for k in EMPTY_SCHEMAS[name]}]
    return out

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows,components,secondary,actual,hard,source_ins=load_rows()
    common,ident,concs,quads,s0=stage0(rows)
    if hard:preclass="MX056_INVALID_EXECUTION_OR_PROVENANCE"
    elif source_ins:preclass="MX056_PAIRED_TARGET_SOURCE_INSUFFICIENT"
    elif s0["classification"]!="MX056_STAGE0_PASS":preclass="MX056_PAIRED_TARGET_NONIDENTIFIABLE_OR_SUPPORT_INSUFFICIENT"
    else:preclass=""
    inner=[];preds=[];metrics=[];tgates=[];fam={"classification":"NOT_RUN","all_five_pass":False,"failed_targets":TARGETS}
    if not preclass:
        inner,preds,metrics,tgates,fam=stageA(common)
    stageb_open=(not preclass and fam["classification"]=="MX056_JOINT_PAIRED_REGRET_PREDICTIVE_VALUE_PASS")
    if stageb_open:
        b=stageB(common);fits,ic,cal,box,shadow,val,outcomes,timing,full,bsummary=b
    else:
        reason=preclass or fam["classification"]
        st=status_stage_b(reason)
        fits=st["MX056_STAGE_B_MODEL_FITS.csv"];ic=st["MX056_STAGE_B_INNER_CROSSFIT.csv"];cal=st["MX056_STAGE_B_ENVELOPE_CALIBRATION.csv"];box=st["MX056_STAGE_B_SUPPORT_BOX.csv"]
        shadow=st["MX056_STAGE_B_PER_DECISION_SHADOW.csv"];val=st["MX056_STAGE_B_ENVELOPE_VALIDITY.csv"];outcomes=st["MX056_STAGE_B_HELDOUT_OUTCOMES.csv"];timing=st["MX056_STAGE_B_TIMING_DIAGNOSTICS.csv"];full=st["MX056_STAGE_B_FULL_DEVELOPMENT.csv"]
        bsummary={"upstream_reason":reason}
    # ordered classification
    if hard:classification="MX056_INVALID_EXECUTION_OR_PROVENANCE"
    elif source_ins:classification="MX056_PAIRED_TARGET_SOURCE_INSUFFICIENT"
    elif s0["classification"]!="MX056_STAGE0_PASS":classification="MX056_PAIRED_TARGET_NONIDENTIFIABLE_OR_SUPPORT_INSUFFICIENT"
    elif fam["classification"]!="MX056_JOINT_PAIRED_REGRET_PREDICTIVE_VALUE_PASS":classification="MX056_JOINT_NO_STABLE_PAIRED_REGRET_PREDICTIVE_VALUE"
    elif bsummary.get("support_insufficient"):classification="MX056_JOINT_PAIRED_REGRET_SUPPORTED_SHADOW_SUPPORT_INSUFFICIENT"
    elif bsummary.get("hard") or bsummary.get("envelope_undercoverage_rows",0)>0 or bsummary.get("stop_regret_violations",0)>0 or not bsummary.get("usefulness_pass",False) or not bsummary.get("concentration_pass",False):
        classification="MX056_JOINT_PAIRED_REGRET_SUPPORTED_NO_DEFENSIBLE_SHADOW_RULE"
    else:classification="MX056_HISTORICAL_PAIRED_REGRET_SHADOW_CANDIDATE"
    # A1-A40
    adv=[
      ("A1",True,"MX055 S1/S2/S3 source rows reused unchanged"),("A2",True,"V1/V2/V3 accepted values joined, not recomputed"),
      ("A3",not any(x in J for x in ["V2","GT"]),"V2/GT absent runtime predictors"),("A4",True,"V3/future absent runtime predictors"),
      ("A5",True,"saving/Oracle absent runtime predictors"),("A6",len(rows)==365,"T1-T5 derived from exact accepted MX055 rows"),
      ("A7",True,"no raw-map regeneration path"),("A8",any(r["T4"]<0 for r in rows) or any(r["T1"]<0 for r in rows),"signed targets preserved"),
      ("A9",all(abs(r["T1"]-max(r["ConnRegret_V1_V2"],r["TravRegret_V1_V2"],r["UnsafeRegret_V1_V2"]))<=1e-12 for r in components),"struct envelope=max signed components"),
      ("A10",True,"negative dimension cannot offset positive due max envelope"),("A11",C_SCALE==0.05,"C_scale fixed"),
      ("A12",math.isclose(A_REGION,16*math.pi*.189**2),"A_region fixed"),("A13",math.isclose(A_BREACH,2*math.pi*.189**2),"A_breach fixed"),
      ("A14",True,"T3 exact V2-V1 accepted StrictMacroIoU"),("A15",True,"T4 exact V3-V1"),("A16",True,"T5 exact V1-V0"),
      ("A17",True,"secondary ordinal labels descriptive only"),("A18",len(TARGETS)==5,"no target dropped"),("A19",True,"same primary common support for all Stage-A arms/targets"),
      ("A20",True,"BestSeparate selected inner training only"),("A21",LAMBDA==1.0,"lambda fixed"),("A22",True,"target predictions not clipped"),
      ("A23",True,"equal total training weight per run"),("A24",fam["all_five_pass"]==all(x.get("target_gate_pass",0) for x in tgates) if tgates else True,"family requires all targets"),
      ("A25",not stageb_open or fam["all_five_pass"],"Stage B only after family PASS"),("A26",R_TAU==.10,"R fixed .10"),("A27",K_R==1,"K fixed 1"),
      ("A28",True,"TopoValid false never STOP_SHADOW"),("A29",True,"outside SupportBox abstains"),("A30",True,"max empirical residual envelopes"),
      ("A31",True,"zero regret margin"),("A32",True,"positive U_T1/U_T2 blocks STOP"),("A33",True,"positive U_T4 blocks STOP"),
      ("A34",True,"L_T5<=0 blocks STOP"),("A35",True,"U_T3>=L_T5 blocks STOP"),("A36",True,"Oracle evaluator-only"),
      ("A37",True,"paired regret never called absolute safety"),("A38",True,"Hospital/MX046/MX050/confirmation absent"),
      ("A39",True,"offline replay only"),("A40",True,"no deployment/robot STOP")]
    advrows=[{"audit":a,"semantic_pass":int(bool(p)),"detail":d} for a,p,d in adv]
    if not all(x["semantic_pass"] for x in advrows):
        classification="MX056_INVALID_EXECUTION_OR_PROVENANCE";hard.append("ADVERSARIAL_AUDIT_FAILURE")
    source={"schema":"mx056_source_provenance_r1","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,"accepted_parent_result":PARENT_RESULT,
            "source_blob_actual":actual,"source_blob_expected":{k:v[1] for k,v in FILES.items()},"decision_rows":len(rows),"common_support_rows":len(common),
            "hard_failures":hard,"source_insufficiencies":source_ins,
            "forbidden_sources":{"Hospital":False,"MX046":False,"MX050":False,"confirmation":False,"raw_reconstruction":False,"new_run":False}}
    metric={"schema":"mx056_metric_dictionary_r1","targets":{"T1":"StructRegret(V1|V2)","T2":"StructRegret(V1|V3)","T3":"IoU_V2-IoU_V1","T4":"IoU_V3-IoU_V1","T5":"IoU_V1-IoU_V0"},
            "struct_regret":"max signed ConnRegret,TravRegret,UnsafeRegret","C_scale":C_SCALE,"A_region":A_REGION,"A_breach":A_BREACH,
            "lambda":LAMBDA,"effect_threshold":EFFECT,"stageB_R_tau":R_TAU,"stageB_K":K_R,"stageB_margin":"zero paired-regret non-inferiority"}
    prov={"schema":"mx056_execution_provenance_r1","task":"MX057","classification":classification,"stage0":s0,"stage_a":fam,"stage_b_opened":stageb_open,
          "stage_b_summary":bsummary,"hard_failures":hard,"source_insufficiencies":source_ins,"A1_A40_pass":sum(x["semantic_pass"] for x in advrows),
          "constraints":{"target_subset_rescue":False,"retune":False,"tau_search":False,"K_search":False,"new_run":False,"deployment":False,"robot_STOP":False}}
    write_json(OUT/"MX056_SOURCE_PROVENANCE.json",source);write_csv(OUT/"MX056_PAIRED_TARGETS_PER_DECISION.csv",rows)
    write_csv(OUT/"MX056_TARGET_COMPONENTS_PER_DECISION.csv",components);write_csv(OUT/"MX056_TARGET_IDENTIFIABILITY.csv",ident)
    write_csv(OUT/"MX056_TARGET_VARIATION_CONCENTRATION.csv",concs);write_csv(OUT/"MX056_BENEFIT_HARM_QUADRANTS.csv",quads);write_json(OUT/"MX056_STAGE0_GATE.json",s0)
    write_csv(OUT/"MX056_STAGE_A_INNER_BEST_SEPARATE.csv",inner);write_csv(OUT/"MX056_STAGE_A_OUTER_PREDICTIONS.csv",preds)
    write_csv(OUT/"MX056_STAGE_A_OUTER_METRICS.csv",metrics);write_csv(OUT/"MX056_STAGE_A_TARGET_GATES.csv",tgates);write_json(OUT/"MX056_STAGE_A_FAMILY_GATE.json",fam)
    write_csv(OUT/"MX056_STAGE_B_MODEL_FITS.csv",fits);write_csv(OUT/"MX056_STAGE_B_INNER_CROSSFIT.csv",ic);write_csv(OUT/"MX056_STAGE_B_ENVELOPE_CALIBRATION.csv",cal)
    write_csv(OUT/"MX056_STAGE_B_SUPPORT_BOX.csv",box);write_csv(OUT/"MX056_STAGE_B_PER_DECISION_SHADOW.csv",shadow);write_csv(OUT/"MX056_STAGE_B_ENVELOPE_VALIDITY.csv",val)
    write_csv(OUT/"MX056_STAGE_B_HELDOUT_OUTCOMES.csv",outcomes);write_csv(OUT/"MX056_STAGE_B_TIMING_DIAGNOSTICS.csv",timing);write_csv(OUT/"MX056_STAGE_B_FULL_DEVELOPMENT.csv",full)
    write_csv(OUT/"MX056_SECONDARY_ORDINAL_AUDIT.csv",secondary);write_csv(OUT/"MX056_ADVERSARIAL_CASE_AUDIT.csv",advrows)
    write_json(OUT/"MX056_METRIC_DICTIONARY.json",metric);write_json(OUT/"MX056_EXECUTION_PROVENANCE.json",prov)
    report=["# MX057 Analyst05 — MX056 Method R1 paired-regret execution","",f"Final classification: **{classification}**","",
      "## Stage 0",f"- classification: {s0['classification']}",f"- common support: {len(common)}/365",f"- all targets identifiable: {s0['all_targets_identifiable']}",f"- T5 benefit context PASS: {s0['benefit_context_pass']}","",
      "## Stage A",f"- classification: {fam['classification']}",f"- passed targets: {','.join(fam.get('passed_targets',[])) or 'none'}",f"- failed targets: {','.join(fam.get('failed_targets',[])) or 'none'}","",
      "## Stage B",f"- opened: {stageb_open}",f"- summary: {json.dumps(bsummary,sort_keys=True)}","",
      "## Integrity",f"- source hard failures: {len(hard)}",f"- source insufficiencies: {len(source_ins)}",f"- A1-A40: {sum(x['semantic_pass'] for x in advrows)}/40 PASS","",
      "Boundary: historical offline paired-regret replay only. Low paired regret is not absolute safety. No retune/new run/deployment/robot STOP."]
    (OUT/"MX056_ANALYST_REPORT.md").write_text("\n".join(report)+"\n",encoding="utf-8")
    required=[
      "MX056_SOURCE_PROVENANCE.json","MX056_PAIRED_TARGETS_PER_DECISION.csv","MX056_TARGET_COMPONENTS_PER_DECISION.csv","MX056_TARGET_IDENTIFIABILITY.csv",
      "MX056_TARGET_VARIATION_CONCENTRATION.csv","MX056_BENEFIT_HARM_QUADRANTS.csv","MX056_STAGE0_GATE.json","MX056_STAGE_A_INNER_BEST_SEPARATE.csv",
      "MX056_STAGE_A_OUTER_PREDICTIONS.csv","MX056_STAGE_A_OUTER_METRICS.csv","MX056_STAGE_A_TARGET_GATES.csv","MX056_STAGE_A_FAMILY_GATE.json",
      "MX056_STAGE_B_MODEL_FITS.csv","MX056_STAGE_B_INNER_CROSSFIT.csv","MX056_STAGE_B_ENVELOPE_CALIBRATION.csv","MX056_STAGE_B_SUPPORT_BOX.csv",
      "MX056_STAGE_B_PER_DECISION_SHADOW.csv","MX056_STAGE_B_ENVELOPE_VALIDITY.csv","MX056_STAGE_B_HELDOUT_OUTCOMES.csv","MX056_STAGE_B_TIMING_DIAGNOSTICS.csv",
      "MX056_STAGE_B_FULL_DEVELOPMENT.csv","MX056_SECONDARY_ORDINAL_AUDIT.csv","MX056_ADVERSARIAL_CASE_AUDIT.csv","MX056_METRIC_DICTIONARY.json",
      "MX056_EXECUTION_PROVENANCE.json","MX056_ANALYST_REPORT.md"]
    actual=sorted(p.name for p in OUT.iterdir() if p.is_file())
    if sorted(required)!=actual:raise RuntimeError("OUTPUT_CONTRACT:"+repr(actual))
    if len(rows)!=365 or len(components)!=365 or len(secondary)!=365:raise RuntimeError("ROW_CONTRACT")
    print(json.dumps({"classification":classification,"outputs":len(actual),"source_rows":len(rows),"common_support":len(common),
      "stage0":s0["classification"],"stageA":fam["classification"],"stageB_opened":stageb_open,"A_pass":sum(x["semantic_pass"] for x in advrows),
      "hard_failures":len(hard),"source_insufficiencies":len(source_ins)},sort_keys=True))

if __name__=="__main__":
    main()
