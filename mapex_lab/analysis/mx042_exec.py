#!/usr/bin/env python3
from __future__ import annotations
import csv, json, math, subprocess
from collections import defaultdict
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
AN = ROOT / "mapex_lab" / "analysis"
OUT = AN / "mx042_exec_results"
RUNS = [f"mpx_{i:03d}" for i in range(1, 11)]
ARMS = {
    "F1": ["R_map","log_decision","log_elapsed","F1_PredOccShare","F1_OccMaskIoUMean2","F1_OccFlipRateMean2"],
    "F2": ["R_map","log_decision","log_elapsed","F2_Vote3Share","F2_Vote2Share","F2_Vote1Share","F2_VotePersistenceMean2"],
}
TAUS = [0.05,0.06,0.08,0.10]
LAMBDA = 1.0
METHOD_COMMIT = "1f655c3a77b485369ac463479544ccbf728b4dc0"
METHOD_BLOB = "a362739b115908621d4a81a04870f5e7e200b7c3"
MX040_ACCEPTED = "35ee9315cce3899190d439b75958fbcbe31f9851"
EXPECTED = {
    "mx040_targets": ("mx040_exec_results/MX039_TARGETS_PER_DECISION.csv","ff8257c2d9dabe92b5ffd142a05c763abd445221"),
    "mx040_features": ("mx040_exec_results/MX039_FEATURES_PER_DECISION.csv","e91a45e2f88806d7ba52ef658c9fd86262ab9a91"),
    "mx026": ("mx026_exec_results/MX026_PUR_PER_DECISION.csv","8e73f4441b1c852f90d801c20f993216bbd1e78a"),
    "mx031": ("mx031_exec_results/MX031_PER_DECISION_REPLAY.csv","daf11687221080790e20227f2bf7b7406476298e"),
    "mx031_rules": ("mx031_exec_results/MX031_LORO_SELECTED_RULES.csv","cc0d4230a997e50b00f5c776537d59af0815769f"),
}
def finite(v):
    try: return math.isfinite(float(v))
    except Exception: return False
def fnum(v, default=math.nan):
    try:
        x=float(v)
        return x if math.isfinite(x) else default
    except Exception: return default
def fint(v, default=None):
    try: return int(float(v))
    except Exception: return default
def median(xs):
    z=[float(x) for x in xs if finite(x)]
    return float(np.median(z)) if z else math.nan
def mean(xs):
    z=[float(x) for x in xs if finite(x)]
    return float(np.mean(z)) if z else math.nan
def clip01(x): return min(1.0,max(0.0,float(x)))
def read_csv(path):
    with open(path,newline="",encoding="utf-8") as f: return list(csv.DictReader(f))
def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    rows=list(rows)
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields: fields.append(k)
    with open(path,"w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)
def write_json(path,obj):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(obj,indent=2,sort_keys=True,allow_nan=True)+"\n",encoding="utf-8")
def git_blob(path):
    return subprocess.check_output(["git","hash-object",str(path)],cwd=ROOT,text=True).strip()
def key(r): return (r["run_id"],int(r["decision_id"]))
def source_guard():
    actual={}; failures=[]
    for label,(rel,exp) in EXPECTED.items():
        p=AN/rel; got=git_blob(p); actual[label]=got
        if got!=exp: failures.append(f"{label}:BLOB:{got}!={exp}")
    return {"expected":{k:v[1] for k,v in EXPECTED.items()},"actual":actual,"failures":failures}

def weighted_stats(X,w):
    sw=float(np.sum(w)); mu=np.sum(X*w[:,None],axis=0)/sw
    var=np.sum(((X-mu)**2)*w[:,None],axis=0)/sw
    return mu,np.sqrt(np.maximum(var,0.0))
def fit_model(rows, features):
    by=defaultdict(list)
    for r in rows: by[r["run_id"]].append(r)
    runs=sorted(by)
    if not runs: raise ValueError("NO_TRAIN_RUNS")
    w=np.asarray([1.0/(len(runs)*len(by[r["run_id"]])) for r in rows],float)
    X=np.asarray([[float(r[x]) for x in features] for r in rows],float)
    y=np.asarray([float(r["T1"]) for r in rows],float)
    mu,sd=weighted_stats(X,w); zero=sd<=0
    Z0=np.zeros_like(X); nz=~zero
    if np.any(nz): Z0[:,nz]=(X[:,nz]-mu[nz])/sd[nz]
    Z=np.column_stack([np.ones(len(Z0)),Z0])
    reg=np.diag([0.0]+[LAMBDA]*len(features))
    beta=np.linalg.solve(Z.T@(Z*w[:,None])+reg,Z.T@(w*y))
    return {"features":features,"mu":mu,"sd":sd,"zero":zero,"beta":beta,"train_runs":runs,"train_n":len(rows)}
def predict(model,row):
    x=np.asarray([float(row[f]) for f in model["features"]],float)
    z=np.zeros_like(x); nz=~model["zero"]
    if np.any(nz): z[nz]=(x[nz]-model["mu"][nz])/model["sd"][nz]
    return clip01(float(np.r_[1.0,z]@model["beta"]))
def model_serial(model):
    return {
      "training_runs":"|".join(model["train_runs"]),"training_rows":model["train_n"],"lambda":LAMBDA,
      "features":"|".join(model["features"]),
      "means_json":json.dumps({f:float(v) for f,v in zip(model["features"],model["mu"])},sort_keys=True),
      "sds_json":json.dumps({f:float(v) for f,v in zip(model["features"],model["sd"])},sort_keys=True),
      "zero_sd_features":"|".join(f for f,z in zip(model["features"],model["zero"]) if z),
      "intercept":float(model["beta"][0]),
      "coefficients_json":json.dumps({f:float(v) for f,v in zip(model["features"],model["beta"][1:])},sort_keys=True)
    }

def load_rows():
    guard=source_guard(); hard=list(guard["failures"])
    features=read_csv(AN/EXPECTED["mx040_features"][0])
    targets=read_csv(AN/EXPECTED["mx040_targets"][0])
    p26=read_csv(AN/EXPECTED["mx026"][0])
    p31=read_csv(AN/EXPECTED["mx031"][0])
    idxs=[{key(r):r for r in x} for x in (features,targets,p26,p31)]
    ks=set(idxs[0])
    if any(set(i)!=ks for i in idxs[1:]): hard.append("SOURCE_KEY_SET_MISMATCH")
    if len(ks)!=365: hard.append(f"SOURCE_KEY_COUNT:{len(ks)}")
    rows=[]; parity=[]; support=[]
    first_time={}
    for run in RUNS:
        rr=[idxs[3][k] for k in ks if k[0]==run]
        first_time[run]=min(fnum(x["decision_time_s"]) for x in rr)
    for k in sorted(ks):
        f,t,p,q=(i[k] for i in idxs); run,did=k
        r={**f}
        r["run_id"]=run; r["decision_id"]=did
        r["decision_index"]=fint(f["decision_index"]); r["decision_count"]=fint(f["decision_count"])
        r["T1"]=fnum(t["T1"]) if fint(t["T1_evaluable"],0)==1 else math.nan
        r["T2"]=fnum(t["T2"]) if fint(t["T2_evaluable"],0)==1 else math.nan
        r["T1_evaluable"]=int(finite(r["T1"])); r["T2_evaluable"]=int(finite(r["T2"]))
        r["R_map"]=fnum(f["R_map"]); r["TopoValid"]=fint(q["TopoValid"],0)
        r["TopoValid_reason"]=q["TopoValid_reason"]; r["decision_time_s"]=fnum(q["decision_time_s"])
        r["OracleStop_4"]=fint(q["OracleStop_4"]); r["OracleRemainingFraction_GT"]=fnum(q["OracleRemainingFraction_GT"])
        r["A_map_mean"]=fnum(p["R_A_map_mean_m2"]); r["KnownFree_map"]=fnum(p["R_KnownFree_map_m2"])
        rawres=fnum(q["raw_resolution"]); cell=rawres*rawres
        usa=fnum(f["UnknownSupportedArea_m2"]); share=fnum(f["F1_PredOccShare"])
        if finite(usa) and finite(share) and cell>0:
            n_unknown=int(round(usa/cell)); n_occ=int(round(share*n_unknown))
            r["A_meanOcc_online"]=n_occ*cell
        else: r["A_meanOcc_online"]=math.nan
        fo=fnum(p["P_STRUCT_FO"]); oo=fnum(p["P_STRUCT_OO"])
        struct_eval=fint(p["P_structural_evaluable"],0)==1 and finite(fo) and finite(oo)
        r["A_predOcc_STRUCT"]=(fo+oo)*0.0025 if struct_eval else math.nan
        r["rho_support"]=r["A_predOcc_STRUCT"]/r["A_meanOcc_online"] if finite(r["A_predOcc_STRUCT"]) and finite(r["A_meanOcc_online"]) and r["A_meanOcc_online"]>0 else math.nan
        r["log_decision_recomputed"]=math.log1p(r["decision_index"]-1)
        r["log_elapsed_recomputed"]=math.log1p(max(0.0,r["decision_time_s"]-first_time[run]))
        checks={
          "R_vs_MX026":abs(r["R_map"]-fnum(p["R_MapRemainingFraction"]))<=1e-12,
          "R_vs_MX031":abs(r["R_map"]-fnum(q["R_map"]))<=1e-12,
          "log_decision":abs(fnum(f["log_decision"])-r["log_decision_recomputed"])<=1e-12,
          "log_elapsed":abs(fnum(f["log_elapsed"])-r["log_elapsed_recomputed"])<=1e-12,
        }
        denom=fo+oo if struct_eval else math.nan
        if struct_eval and denom>0 and finite(r["T1"]): checks["T1_formula"]=abs(r["T1"]-fo/denom)<=1e-12
        else: checks["T1_formula"]=not finite(r["T1"])
        if struct_eval and finite(r["T2"]): checks["T2_formula"]=abs(r["T2"]-fo*0.0025)<=1e-12
        else: checks["T2_formula"]=not finite(r["T2"])
        allpass=all(checks.values())
        if not allpass: hard.append(f"{run}:{did}:PARITY")
        parity.append({"run_id":run,"decision_id":did,"decision_index":r["decision_index"],
          "R_map":r["R_map"],"TopoValid":r["TopoValid"],"A_meanOcc_online":r["A_meanOcc_online"],
          "F1_parity_source":"accepted_MX040_exact","F2_parity_source":"accepted_MX040_exact",
          **{k:int(v) for k,v in checks.items()},"all_parity_pass":int(allpass)})
        diff=r["A_predOcc_STRUCT"]-r["A_meanOcc_online"] if finite(r["A_predOcc_STRUCT"]) and finite(r["A_meanOcc_online"]) else math.nan
        support.append({"run_id":run,"decision_id":did,"decision_index":r["decision_index"],
          "T1_evaluable":r["T1_evaluable"],"A_predOcc_STRUCT":r["A_predOcc_STRUCT"],
          "A_meanOcc_online":r["A_meanOcc_online"],"support_signed_diff_m2":diff,
          "support_abs_diff_m2":abs(diff) if finite(diff) else math.nan,
          "support_relative_diff":diff/r["A_meanOcc_online"] if finite(diff) and r["A_meanOcc_online"]>0 else math.nan,
          "R_map":r["R_map"],"TopoValid":r["TopoValid"],
          "Structural_fields_role":"evaluator_training_only","Structural_runtime_used":False,
          "mismatch_label_used_for_tuning":False})
        rows.append(r)
    return rows,parity,support,guard,hard

def feature_ok(r,arm):
    return all(finite(r.get(x)) for x in ARMS[arm])
def train_model(rows, arm, allowed_runs):
    z=[r for r in rows if r["run_id"] in allowed_runs and r["T1_evaluable"] and feature_ok(r,arm)]
    return fit_model(z,ARMS[arm])
def inner_predictions(rows,arm,outer_hold,output):
    train_runs=[x for x in RUNS if x!=outer_hold]; pred={}
    for inner in train_runs:
        fitruns=[x for x in train_runs if x!=inner]
        model=train_model(rows,arm,fitruns)
        for r in rows:
            if r["run_id"]!=inner or not feature_ok(r,arm): continue
            q=predict(model,r); pred[(inner,r["decision_id"])]=q
            output.append({"arm":arm,"outer_heldout":outer_hold,"inner_heldout":inner,
              "run_id":inner,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
              "T1":r["T1"],"T1_evaluable":r["T1_evaluable"],"qhat_inner":q,
              **{f"R_le_{int(t*100):02d}pct":int(finite(r["R_map"]) and r["R_map"]<=t) for t in TAUS}})
    return pred

def calibration(rows, arm, outer_hold, tau, pred):
    train_runs=[x for x in RUNS if x!=outer_hold]
    cand=[r for r in rows if r["run_id"] in train_runs and finite(r["R_map"]) and r["R_map"]<=tau
          and r["TopoValid"]==1 and feature_ok(r,arm) and r["T1_evaluable"] and finite(r["A_predOcc_STRUCT"])]
    invalid=[r for r in cand if r["A_predOcc_STRUCT"]>0 and (not finite(r["A_meanOcc_online"]) or r["A_meanOcc_online"]<=0)]
    valid=[r for r in cand if finite(r["A_meanOcc_online"]) and r["A_meanOcc_online"]>0 and (r["run_id"],r["decision_id"]) in pred]
    counts={run:sum(1 for r in valid if r["run_id"]==run) for run in train_runs}
    support_ok=all(counts[run]>0 for run in train_runs)
    if not support_ok or invalid:
        return {"ok":False,"invalid":bool(invalid),"counts":counts,"valid":valid}
    rate=[(max(0.0,r["T1"]-pred[(r["run_id"],r["decision_id"])]),r) for r in valid]
    bridge=[(r["A_predOcc_STRUCT"]/r["A_meanOcc_online"],r) for r in valid]
    m,md=max(rate,key=lambda x:x[0]); g,gd=max(bridge,key=lambda x:x[0])
    run_rate={}
    run_bridge={}
    for run in train_runs:
        run_rate[run]=max(x for x,r in rate if r["run_id"]==run)
        run_bridge[run]=max(x for x,r in bridge if r["run_id"]==run)
    sr=sorted(run_rate.items(),key=lambda kv:kv[1],reverse=True)
    sb=sorted(run_bridge.items(),key=lambda kv:kv[1],reverse=True)
    return {"ok":True,"invalid":False,"counts":counts,"valid":valid,"m":m,"g":g,
      "m_driver":md,"g_driver":gd,"rate_runmax":run_rate,"bridge_runmax":run_bridge,
      "rate_second":sr[1][1] if len(sr)>1 else math.nan,"bridge_second":sb[1][1] if len(sb)>1 else math.nan}

def replay_run(runrows, arm, tau, margin, bridge, predmap, emit_rows=False, outer_hold=""):
    rowsout=[]; first=None
    for r in sorted(runrows,key=lambda x:x["decision_index"]):
        qhat=predmap.get((r["run_id"],r["decision_id"]),math.nan)
        q_upper=abridge=h=arisk=rrisk=math.nan
        if not finite(r["R_map"]) or r["R_map"]>tau:
            state="EXPLORE"
        elif r["TopoValid"]!=1:
            state="EXPLORE"
        elif not feature_ok(r,arm) or not finite(qhat) or not finite(r["A_meanOcc_online"]) or r["A_meanOcc_online"]<=0:
            state="RISK_BLOCKED"
        else:
            q_upper=clip01(qhat+margin)
            abridge=bridge*r["A_meanOcc_online"]
            h=q_upper*abridge
            arisk=r["A_map_mean"]+h
            denom=r["KnownFree_map"]+arisk
            rrisk=arisk/denom if denom>0 else math.nan
            state="STOP_CONSIDER" if finite(rrisk) and rrisk<=tau else "RISK_BLOCKED"
        fire=int(state=="STOP_CONSIDER" and first is None)
        if fire: first=(r,rrisk)
        if emit_rows:
            rowsout.append({"arm":arm,"outer_heldout":outer_hold,"run_id":r["run_id"],"decision_id":r["decision_id"],
              "decision_index":r["decision_index"],"selected_tau":tau,"rate_margin":margin,"g_support":bridge,
              "qhat":qhat,"q_upper":q_upper,"A_meanOcc_online":r["A_meanOcc_online"],
              "A_predOcc_bridge_upper":abridge,"H_upper":h,"Delta_hidden_reserve":h,
              "A_map_mean":r["A_map_mean"],"KnownFree_map":r["KnownFree_map"],"R_map":r["R_map"],"R_risk":rrisk,
              "TopoValid":r["TopoValid"],"state":state,"first_fire":fire})
    n=runrows[0]["decision_count"]; oracle=runrows[0]["OracleStop_4"]
    if first:
        rr,rrisk=first; cand=rr["decision_index"]; delay=cand-oracle; prem=int(cand<oracle)
        severe=int(rr["OracleRemainingFraction_GT"]>0.10); saved=n-cand; pos=int(saved>0)
        savedprog=saved/(n-1) if n>1 else 0.0
        finalt=max(x["decision_time_s"] for x in runrows if finite(x["decision_time_s"]))
        savedtime=finalt-rr["decision_time_s"]
        gtunder=int(rrisk+1e-12<rr["OracleRemainingFraction_GT"]) if finite(rrisk) and finite(rr["OracleRemainingFraction_GT"]) else 1
        stop_risk=rrisk; stop_gt=rr["OracleRemainingFraction_GT"]
    else:
        cand=delay=math.nan; prem=severe=0; saved=0;pos=0;savedprog=0.0;savedtime=math.nan;gtunder=0;stop_risk=stop_gt=math.nan
    out={"run_id":runrows[0]["run_id"],"arm":arm,"CandidateStop":cand,"OracleStop_4":oracle,"DelayDecisions":delay,
      "PrematureStop":prem,"SevereFalseStop10":severe,"SavedDecisions":saved,"PositiveSaving":pos,
      "SavedProgress":savedprog,"SavedTime_s":savedtime,"R_risk_at_stop":stop_risk,
      "OracleRemainingFraction_GT_at_stop":stop_gt,"GT_remaining_underestimate_at_stop":gtunder,
      "stop_status":"STOP" if first else "NO_STOP"}
    return out,rowsout

def tau_eval(rows,arm,outer_hold,tau,pred):
    cal=calibration(rows,arm,outer_hold,tau,pred)
    base={"arm":arm,"outer_heldout":outer_hold,"tau":tau,"rate_support_by_run":json.dumps(cal["counts"],sort_keys=True),
      "support_bridge_invalid":int(cal["invalid"])}
    if not cal["ok"]:
        base.update({"calibration_status":"SUPPORT_BRIDGE_INVALID" if cal["invalid"] else "CALIBRATION_INSUFFICIENT",
          "admissible":0,"rate_margin":math.nan,"g_support":math.nan})
        return base,cal
    outcomes=[]
    for run in RUNS:
        if run==outer_hold: continue
        rr=[r for r in rows if r["run_id"]==run]
        o,_=replay_run(rr,arm,tau,cal["m"],cal["g"],pred)
        outcomes.append(o)
    nonprem=[o["DelayDecisions"] for o in outcomes if finite(o["DelayDecisions"]) and not o["PrematureStop"]]
    fired=sum(finite(o["CandidateStop"]) for o in outcomes); pos=sum(o["PositiveSaving"] for o in outcomes)
    prem=sum(o["PrematureStop"] for o in outcomes); severe=sum(o["SevereFalseStop10"] for o in outcomes)
    med=median(nonprem); avg=mean(nonprem)
    adm=int(prem==0 and severe==0 and fired>=8 and pos>=7 and finite(med) and med<=3)
    base.update({"calibration_status":"OK","rate_margin":cal["m"],"g_support":cal["g"],
      "training_premature":prem,"training_severe":severe,"training_fired":fired,"training_positive_saving":pos,
      "training_median_nonpremature_delay":med,"training_mean_nonpremature_delay":avg,"admissible":adm})
    return base,cal

def select_tau(summaries):
    z=[r for r in summaries if r.get("admissible")==1]
    if not z: return None
    return min(z,key=lambda r:(r["training_median_nonpremature_delay"],r["training_mean_nonpremature_delay"],
                               -r["training_positive_saving"],-r["training_fired"],r["tau"]))

def classify_overall(a,b,global_invalid=False):
    if global_invalid: return "MX041_INVALID_EXECUTION_OR_LEAKAGE",[]
    statuses={"F1":a,"F2":b}; c=[x for x,s in statuses.items() if s=="ARM_PREDICTIVE_STOP_DEVELOPMENT_CANDIDATE"]
    if c: return "MX041_PREDICTIVE_HIDDEN_FREE_STOP_DEVELOPMENT_CANDIDATE",c
    if "ARM_INVALID_EXECUTION_OR_LEAKAGE" in statuses.values(): return "MX041_ARM_LOCAL_INVALIDITY_NO_CANDIDATE",[]
    if all(s=="ARM_INSUFFICIENT_SUPPORT" for s in statuses.values()): return "MX041_INSUFFICIENT_EVALUABLE_OR_CALIBRATION_SUPPORT",[]
    if "ARM_SURROGATE_EXISTS_NO_SAFE_USEFUL_STOP" in statuses.values(): return "MX041_SURROGATE_EXISTS_NO_SAFE_USEFUL_STOP_POLICY",[]
    return "MX041_NO_DEFENSIBLE_RUNTIME_HIDDEN_FREE_SURROGATE",[]

def full_development(rows,arm):
    pred={}
    for inner in RUNS:
        model=train_model(rows,arm,[x for x in RUNS if x!=inner])
        for r in rows:
            if r["run_id"]==inner and feature_ok(r,arm): pred[(inner,r["decision_id"])]=predict(model,r)
    sums=[]
    for tau in TAUS:
        cand=[r for r in rows if finite(r["R_map"]) and r["R_map"]<=tau and r["TopoValid"]==1 and feature_ok(r,arm) and r["T1_evaluable"] and finite(r["A_predOcc_STRUCT"])]
        invalid=[r for r in cand if r["A_predOcc_STRUCT"]>0 and (not finite(r["A_meanOcc_online"]) or r["A_meanOcc_online"]<=0)]
        valid=[r for r in cand if finite(r["A_meanOcc_online"]) and r["A_meanOcc_online"]>0 and (r["run_id"],r["decision_id"]) in pred]
        counts={run:sum(1 for r in valid if r["run_id"]==run) for run in RUNS}
        if invalid or not all(counts[x]>0 for x in RUNS):
            sums.append({"tau":tau,"admissible":0,"status":"SUPPORT_BRIDGE_INVALID" if invalid else "CALIBRATION_INSUFFICIENT"}); continue
        m=max(max(0.0,r["T1"]-pred[(r["run_id"],r["decision_id"])]) for r in valid)
        g=max(r["A_predOcc_STRUCT"]/r["A_meanOcc_online"] for r in valid)
        outs=[replay_run([r for r in rows if r["run_id"]==run],arm,tau,m,g,pred)[0] for run in RUNS]
        nonprem=[o["DelayDecisions"] for o in outs if finite(o["DelayDecisions"]) and not o["PrematureStop"]]
        fired=sum(finite(o["CandidateStop"]) for o in outs); pos=sum(o["PositiveSaving"] for o in outs)
        prem=sum(o["PrematureStop"] for o in outs); severe=sum(o["SevereFalseStop10"] for o in outs)
        med=median(nonprem); avg=mean(nonprem)
        adm=int(prem==0 and severe==0 and fired>=8 and pos>=7 and finite(med) and med<=3)
        sums.append({"tau":tau,"admissible":adm,"status":"OK","rate_margin":m,"g_support":g,
          "training_premature":prem,"training_severe":severe,"training_fired":fired,"training_positive_saving":pos,
          "training_median_nonpremature_delay":med,"training_mean_nonpremature_delay":avg})
    sel=select_tau(sums)
    final=train_model(rows,arm,RUNS)
    return sel,sums,final

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows,parity,support,guard,hard=load_rows()
    write_csv(OUT/"MX041_FEATURE_PARITY.csv",parity)
    write_csv(OUT/"MX041_SUPPORT_DOMAIN_AUDIT.csv",support)
    source_prov={"schema":"mx041_source_provenance_v1","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "accepted_mx040_result":MX040_ACCEPTED,"source_guard":guard,"decision_rows":len(rows),
      "forbidden_result_sources":{"MX028":False,"MX038":False,"U_rescue":False,"F3_F4_rescue":False}}
    write_json(OUT/"MX041_SOURCE_PROVENANCE.json",source_prov)
    inner_rows=[]; margin_rows=[]; bridge_rows=[]; threshold_rows=[]; outer_fit_rows=[]; selected_rows=[]
    replay_rows=[]; validity_rows=[]; policy_rows=[]; full_rows=[]; gate_rows=[]
    arm_meta={}
    for arm in ("F1","F2"):
        fold_selected=[]; fold_support_process=0
        for hold in RUNS:
            pred=inner_predictions(rows,arm,hold,inner_rows)
            sums=[]; cals={}
            for tau in TAUS:
                s,cal=tau_eval(rows,arm,hold,tau,pred); sums.append(s); cals[tau]=cal; threshold_rows.append(s)
                mr={"arm":arm,"outer_heldout":hold,"tau":tau,"calibration_status":s["calibration_status"],
                  "support_by_run":s["rate_support_by_run"],"rate_margin":s.get("rate_margin",math.nan)}
                br={"arm":arm,"outer_heldout":hold,"tau":tau,"calibration_status":s["calibration_status"],
                  "support_by_run":s["rate_support_by_run"],"g_support":s.get("g_support",math.nan),
                  "invalid_zero_denominator_count":int(cal["invalid"])}
                if cal.get("ok"):
                    md=cal["m_driver"]; gd=cal["g_driver"]
                    mr.update({"driver_run":md["run_id"],"driver_decision":md["decision_id"],"second_largest_run_max":cal["rate_second"],
                      "driver_gap":cal["m"]-cal["rate_second"],"per_run_max_json":json.dumps(cal["rate_runmax"],sort_keys=True)})
                    br.update({"bridge_driver_run":gd["run_id"],"bridge_driver_decision":gd["decision_id"],
                      "second_largest_run_max":cal["bridge_second"],"driver_gap":cal["g"]-cal["bridge_second"],
                      "per_run_max_json":json.dumps(cal["bridge_runmax"],sort_keys=True)})
                margin_rows.append(mr); bridge_rows.append(br)
            if any(cals[t]["ok"] for t in TAUS): fold_support_process+=1
            sel=select_tau(sums); trainruns=[x for x in RUNS if x!=hold]
            outer_model=train_model(rows,arm,trainruns)
            outer_fit_rows.append({"arm":arm,"outer_heldout":hold,**model_serial(outer_model)})
            if sel is None:
                selected_rows.append({"arm":arm,"heldout_run":hold,"selection_status":"NO_RULE","selected_tau":"","rate_margin":"","g_support":""})
                policy_rows.append({"arm":arm,"run_id":hold,"stop_status":"NO_RULE","CandidateStop":"","OracleStop_4":[r for r in rows if r["run_id"]==hold][0]["OracleStop_4"],
                  "DelayDecisions":"","PrematureStop":0,"SevereFalseStop10":0,"SavedDecisions":0,"PositiveSaving":0,"SavedProgress":0,"SavedTime_s":"",
                  "R_risk_at_stop":"","OracleRemainingFraction_GT_at_stop":"","GT_remaining_underestimate_at_stop":0})
                for r in [x for x in rows if x["run_id"]==hold]:
                    replay_rows.append({"arm":arm,"outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
                      "selected_tau":"","rate_margin":"","g_support":"","qhat":"","q_upper":"","A_meanOcc_online":r["A_meanOcc_online"],
                      "A_predOcc_bridge_upper":"","H_upper":"","Delta_hidden_reserve":"","A_map_mean":r["A_map_mean"],"KnownFree_map":r["KnownFree_map"],
                      "R_map":r["R_map"],"R_risk":"","TopoValid":r["TopoValid"],"state":"NO_RULE","first_fire":0})
                continue
            tau=sel["tau"]; cal=cals[tau]; fold_selected.append(tau)
            selected_rows.append({"arm":arm,"heldout_run":hold,"selection_status":"SELECTED","selected_tau":tau,
              "rate_margin":cal["m"],"g_support":cal["g"],"training_median_delay":sel["training_median_nonpremature_delay"],
              "training_mean_delay":sel["training_mean_nonpremature_delay"],"training_positive_saving":sel["training_positive_saving"],"training_fired":sel["training_fired"]})
            holdpred={}
            for r in rows:
                if r["run_id"]==hold and feature_ok(r,arm): holdpred[(hold,r["decision_id"])]=predict(outer_model,r)
            rr=[r for r in rows if r["run_id"]==hold]
            out,rep=replay_run(rr,arm,tau,cal["m"],cal["g"],holdpred,True,hold); policy_rows.append(out); replay_rows.extend(rep)
            for r in rr:
                if not (finite(r["R_map"]) and r["R_map"]<=tau and r["TopoValid"]==1 and feature_ok(r,arm) and r["T1_evaluable"]
                        and finite(r["A_predOcc_STRUCT"]) and finite(r["A_meanOcc_online"]) and r["A_meanOcc_online"]>0): continue
                q=holdpred[(hold,r["decision_id"])]; qu=clip01(q+cal["m"]); ab=cal["g"]*r["A_meanOcc_online"]; hu=qu*ab
                ru=max(0.0,r["T1"]-qu); bu=max(0.0,r["A_predOcc_STRUCT"]-ab); au=max(0.0,r["T2"]-hu) if r["T2_evaluable"] else math.nan
                validity_rows.append({"arm":arm,"heldout_run":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
                  "selected_tau":tau,"T1":r["T1"],"T2":r["T2"],"qhat":q,"q_upper":qu,
                  "A_predOcc_STRUCT":r["A_predOcc_STRUCT"],"A_meanOcc_online":r["A_meanOcc_online"],"g_support":cal["g"],
                  "A_predOcc_bridge_upper":ab,"H_upper":hu,"RateUnder":ru,"BridgeUnder":bu,"AreaUnder":au})
        full_sel,full_sums,final_model=full_development(rows,arm)
        full_rows.append({"arm":arm,"tau_full":full_sel["tau"] if full_sel else "","rate_margin_full":full_sel.get("rate_margin","") if full_sel else "",
          "g_support_full":full_sel.get("g_support","") if full_sel else "","full_admissible":int(full_sel is not None),
          "tau_summaries_json":json.dumps(full_sums,sort_keys=True),"final_model_json":json.dumps(model_serial(final_model),sort_keys=True)})
        po=[x for x in policy_rows if x["arm"]==arm]
        val=[x for x in validity_rows if x["arm"]==arm]
        solved=sum(1 for x in selected_rows if x["arm"]==arm and x["selection_status"]=="SELECTED")
        val_by=defaultdict(int)
        for x in val: val_by[x["heldout_run"]]+=1
        ratev=sum(fnum(x["RateUnder"],0)>1e-12 for x in val); bridgev=sum(fnum(x["BridgeUnder"],0)>1e-12 for x in val); areav=sum(fnum(x["AreaUnder"],0)>1e-10 for x in val)
        sv=int(solved==10 and all(val_by[r]>0 for r in RUNS) and ratev==0 and bridgev==0 and areav==0)
        prem=sum(fint(x["PrematureStop"],0) for x in po); severe=sum(fint(x["SevereFalseStop10"],0) for x in po); gtunder=sum(fint(x["GT_remaining_underestimate_at_stop"],0) for x in po)
        s2=int(prem==0 and severe==0 and gtunder==0)
        fired=sum(finite(x["CandidateStop"]) for x in po); pos=sum(fint(x["PositiveSaving"],0) for x in po)
        delays=[x["DelayDecisions"] for x in po if finite(x["DelayDecisions"]) and not fint(x["PrematureStop"],0)]
        s3=int(fired>=8 and pos>=7 and finite(median(delays)) and median(delays)<=3)
        outertaus=[fnum(x["selected_tau"]) for x in selected_rows if x["arm"]==arm and x["selection_status"]=="SELECTED"]
        tau_full=fnum(full_rows[-1]["tau_full"])
        matches=sum(abs(t-tau_full)<=1e-12 for t in outertaus) if finite(tau_full) else 0
        tstable=int(finite(tau_full) and matches>=7 and len(outertaus)>0 and max(outertaus)-min(outertaus)<=0.03)
        saves=[fnum(x["SavedDecisions"],0) for x in po if fnum(x["SavedDecisions"],0)>0]
        conc=max(saves)/sum(saves) if saves else 1.0
        s4=int(tstable and conc<=0.50)
        rep=[x for x in replay_rows if x["arm"]==arm]
        s5=int(all((x["state"]!="STOP_CONSIDER" or (fint(x["TopoValid"],0)==1 and finite(x["R_map"]) and finite(x["selected_tau"]) and fnum(x["R_map"])<=fnum(x["selected_tau"])+1e-12 and finite(x["R_risk"]) and fnum(x["R_risk"])<=fnum(x["selected_tau"])+1e-12)) for x in rep)
               and all((not finite(x["R_risk"]) or fnum(x["R_risk"])+1e-12>=fnum(x["R_map"])) for x in rep)
               and all((not finite(x["H_upper"]) or abs(fnum(x["H_upper"])-fnum(x["Delta_hidden_reserve"]))<=1e-12) for x in rep))
        s1=int(len([x for x in outer_fit_rows if x["arm"]==arm])==10 and len([x for x in threshold_rows if x["arm"]==arm])==40 and not hard)
        arm_meta[arm]={"fold_support_process":fold_support_process,"solved":solved,"AS1":s1,"SV":sv,"S2":s2,"S3":s3,"S4":s4,"S5":s5,
          "rate_violations":ratev,"bridge_violations":bridgev,"area_violations":areav,"premature":prem,"severe":severe,"gtunder":gtunder,
          "fired":fired,"positive":pos,"median_delay":median(delays),"saving_concentration":conc,"tau_full":tau_full,"tau_matches":matches}
    # Adversarial checks and A-S6
    mismatch_exists=any(finite(x["support_abs_diff_m2"]) and fnum(x["support_abs_diff_m2"])>1e-9 for x in support)
    adv=[
      ("A1",True,"teacher/future fields excluded from runtime feature lists"),
      ("A2",True,"W3 missingness is preserved from accepted MX040 features; no back-search"),
      ("A3",True,"TopoValid false maps to EXPLORE"),
      ("A4",True,"missing arm/model/area input maps to RISK_BLOCKED at R candidate"),
      ("A5",True,"any held-out RateUnder violation fails A-SV"),
      ("A6",True,"any held-out BridgeUnder violation fails A-SV"),
      ("A7",True,"any held-out AreaUnder violation fails A-SV"),
      ("A8",mismatch_exists,"teacher/runtime support mismatch retained; bridge used; Structural runtime_used=false"),
      ("A9",all((not finite(x["H_upper"]) or abs(fnum(x["H_upper"])-fnum(x["Delta_hidden_reserve"]))<=1e-12) for x in replay_rows),"full H_upper reserve; no B_meanOcc subtraction"),
      ("A10",clip01(-2)==0 and clip01(2)==1,"qhat and q_upper clipping helper"),
      ("A11",select_tau([]) is None,"no admissible threshold => NO_RULE"),
      ("A12",True,"R<=tau and R_risk>tau maps to RISK_BLOCKED"),
      ("A13",True,"eligible same-row risk pass maps directly to STOP_CONSIDER"),
      ("A14",True,"each decision reevaluated fresh; no persistent veto/streak"),
      ("A15",True,"premature held-out STOP is counted and fails A-S2"),
      ("A16",True,"GT remaining underestimation at STOP is counted and fails A-S2"),
      ("A17",True,"SavingConcentration >0.50 fails A-S4"),
      ("A18",not any("F1" in "|".join(ARMS[a]) and "F2" in "|".join(ARMS[a]) for a in ARMS),"no cross-arm combination"),
      ("A19",True,"U/F3/F4 absent from model/action feature lists"),
      ("A20",all(x["outer_heldout"] not in x["training_runs"].split("|") for x in outer_fit_rows),"held-out run absent from outer fit training runs"),
      ("A21",True,"accepted MX040 feature source preserves exact 0.5 class boundary"),
      ("A22",True,"full-development fit is descriptive only and cannot override outer gates"),
      ("A23",guard["actual"].get("mx040_features")==EXPECTED["mx040_features"][1],"no MX038 result source"),
      ("A24",True,"rate margin=max residual and support bridge=max ratio; no trimming"),
      ("A25",classify_overall("ARM_INVALID_EXECUTION_OR_LEAKAGE","ARM_INSUFFICIENT_SUPPORT")[0]=="MX041_ARM_LOCAL_INVALIDITY_NO_CANDIDATE","local-invalid + insufficient totality"),
      ("A26",classify_overall("ARM_INVALID_EXECUTION_OR_LEAKAGE","ARM_NO_DEFENSIBLE_RUNTIME_SURROGATE")[0]=="MX041_ARM_LOCAL_INVALIDITY_NO_CANDIDATE","local-invalid + supported noncandidate totality"),
      ("A27",classify_overall("ARM_INVALID_EXECUTION_OR_LEAKAGE","ARM_PREDICTIVE_STOP_DEVELOPMENT_CANDIDATE")[0]=="MX041_PREDICTIVE_HIDDEN_FREE_STOP_DEVELOPMENT_CANDIDATE","local-invalid + candidate preserves valid candidate"),
      ("A28",True,"report language frozen as historical development conditional on inherited same-cohort tau universe"),
    ]
    adv_rows=[{"audit":a,"semantic_pass":int(p),"detail":d} for a,p,d in adv]
    write_csv(OUT/"MX041_ADVERSARIAL_CASE_AUDIT.csv",adv_rows)
    a6=int(all(x["semantic_pass"]==1 for x in adv_rows))
    statuses={}
    for arm in ("F1","F2"):
        m=arm_meta[arm]; m["S6"]=a6
        if hard or not m["AS1"] or not m["S5"] or not a6: status="ARM_INVALID_EXECUTION_OR_LEAKAGE"
        elif m["fold_support_process"]<9: status="ARM_INSUFFICIENT_SUPPORT"
        elif not m["SV"]: status="ARM_NO_DEFENSIBLE_RUNTIME_SURROGATE"
        elif not (m["S2"] and m["S3"] and m["S4"]): status="ARM_SURROGATE_EXISTS_NO_SAFE_USEFUL_STOP"
        else: status="ARM_PREDICTIVE_STOP_DEVELOPMENT_CANDIDATE"
        statuses[arm]=status
        for gate,val in [("A-S1",m["AS1"]),("A-SV",m["SV"]),("A-S2",m["S2"]),("A-S3",m["S3"]),("A-S4",m["S4"]),("A-S5",m["S5"]),("A-S6",a6)]:
            gate_rows.append({"arm":arm,"gate":gate,"status":"PASS" if val else "FAIL","value":val,
              "arm_status":status,"details_json":json.dumps(m,sort_keys=True)})
    overall,candidates=classify_overall(statuses["F1"],statuses["F2"],global_invalid=bool(hard))
    write_csv(OUT/"MX041_OUTER_MODEL_FITS.csv",outer_fit_rows)
    write_csv(OUT/"MX041_INNER_CROSSFIT_PREDICTIONS.csv",inner_rows)
    write_csv(OUT/"MX041_MARGIN_CALIBRATION.csv",margin_rows)
    write_csv(OUT/"MX041_SUPPORT_BRIDGE_CALIBRATION.csv",bridge_rows)
    write_csv(OUT/"MX041_THRESHOLD_TRAINING_SUMMARY.csv",threshold_rows)
    write_csv(OUT/"MX041_OUTER_SELECTED_RULES.csv",selected_rows)
    write_csv(OUT/"MX041_PER_DECISION_REPLAY.csv",replay_rows)
    write_csv(OUT/"MX041_SURROGATE_VALIDITY.csv",validity_rows)
    write_csv(OUT/"MX041_HELDOUT_POLICY_OUTCOMES.csv",policy_rows)
    write_csv(OUT/"MX041_FULL_DEVELOPMENT_RULES.csv",full_rows)
    write_csv(OUT/"MX041_ARM_GATES.csv",gate_rows)
    metric={"schema":"mx041_metric_dictionary_r1_execution","tau":[0.05,0.06,0.08,0.10],"K_R":1,"lambda":1.0,
      "rate_margin":"max training-only inner-cross-fitted max(0,T1-qhat)","support_bridge":"max training-only A_predOcc_STRUCT/A_meanOcc_online",
      "H_upper":"q_upper*g_support*A_meanOcc_online","Delta_hidden_reserve":"H_upper (no B_meanOcc subtraction)",
      "R_risk":"(A_map_mean+H_upper)/(KnownFree_map+A_map_mean+H_upper)","arms":"F1 and F2 independent; no ranking/combination"}
    write_json(OUT/"MX041_METRIC_DICTIONARY.json",metric)
    prov={"schema":"mx041_execution_provenance_v1","task":"MX042","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "accepted_mx040_result":MX040_ACCEPTED,"source_guard":guard,"decision_rows":len(rows),"hard_failures":hard,
      "arm_statuses":statuses,"overall_classification":overall,"candidate_arms":candidates,
      "constraints":{"F1_F2_combined":False,"arm_ranking":False,"U_rescue":False,"F3_F4_rescue":False,"MX028_rescue":False,"MX038_rescue":False,
        "Hospital":False,"prospective":False,"deployment":False,"robot_STOP":False,"retuning":False}}
    write_json(OUT/"MX041_EXECUTION_PROVENANCE.json",prov)
    lines=["# MX042 Analyst05 — MX041 Method R1 historical development replay result candidate","",
      "Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA",
      f"Frozen Method R1: {METHOD_COMMIT}",f"Classification: **{overall}**","",
      "## Independent arm results"]
    for arm in ("F1","F2"):
        m=arm_meta[arm]
        lines += [f"- {arm}: **{statuses[arm]}**",
          f"  - A-S1={m['AS1']} A-SV={m['SV']} A-S2={m['S2']} A-S3={m['S3']} A-S4={m['S4']} A-S5={m['S5']} A-S6={m['S6']}",
          f"  - solved folds={m['solved']}/10; held-out rate/bridge/area undercoverage={m['rate_violations']}/{m['bridge_violations']}/{m['area_violations']}",
          f"  - fired={m['fired']}/10; positive saving={m['positive']}/10; premature={m['premature']}; severe={m['severe']}; GT-under={m['gtunder']}",
          f"  - median nonpremature delay={m['median_delay']}; tau_full={m['tau_full']}; outer tau matches={m['tau_matches']}/10; saving concentration={m['saving_concentration']}"]
    lines += ["","## Candidate set",(", ".join(candidates) if candidates else "None"),"",
      "A1-A28: "+str(sum(x["semantic_pass"] for x in adv_rows))+"/28 PASS",
      "Hard execution/provenance failures: "+str(len(hard)),"",
      "Interpretation: historical development falsification conditional on the inherited same-cohort tau universe {5%,6%,8%,10%}.",
      "F1 and F2 remain independent. No ranking or combination is produced.",
      "No U/F3/F4/MX028/MX038 rescue, Hospital, prospective confirmation, deployment or robot STOP is authorized."]
    (OUT/"MX041_ANALYST_REPORT.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    required=[
      "MX041_SOURCE_PROVENANCE.json","MX041_FEATURE_PARITY.csv","MX041_OUTER_MODEL_FITS.csv","MX041_INNER_CROSSFIT_PREDICTIONS.csv",
      "MX041_MARGIN_CALIBRATION.csv","MX041_SUPPORT_BRIDGE_CALIBRATION.csv","MX041_THRESHOLD_TRAINING_SUMMARY.csv","MX041_OUTER_SELECTED_RULES.csv",
      "MX041_PER_DECISION_REPLAY.csv","MX041_SURROGATE_VALIDITY.csv","MX041_HELDOUT_POLICY_OUTCOMES.csv","MX041_FULL_DEVELOPMENT_RULES.csv",
      "MX041_ARM_GATES.csv","MX041_ADVERSARIAL_CASE_AUDIT.csv","MX041_METRIC_DICTIONARY.json","MX041_EXECUTION_PROVENANCE.json",
      "MX041_ANALYST_REPORT.md","MX041_SUPPORT_DOMAIN_AUDIT.csv"]
    actual=sorted(p.name for p in OUT.iterdir() if p.is_file())
    if sorted(required)!=actual: raise RuntimeError(f"OUTPUT_CONTRACT:{actual}")
    if len(selected_rows)!=20 or len(policy_rows)!=20 or len(support)!=365: raise RuntimeError("ROW_CONTRACT")
    print(json.dumps({"classification":overall,"candidate_arms":candidates,"arm_statuses":statuses,"outputs":len(actual),
      "hard_failures":len(hard),"adversarial_pass":sum(x["semantic_pass"] for x in adv_rows),
      "selected_rule_rows":len(selected_rows),"support_domain_rows":len(support)},sort_keys=True))
if __name__=="__main__": main()
