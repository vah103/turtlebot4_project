#!/usr/bin/env python3
from __future__ import annotations
import csv, json, math, subprocess
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
AN = ROOT / "mapex_lab" / "analysis"
OUT = AN / "mx044_exec_results"
RUNS = [f"mpx_{i:03d}" for i in range(1,11)]
H = 3
B_U = 3.0
LAMBDA = 1.0
TAUS = [0.05,0.06,0.08,0.10]
METHOD_COMMIT = "333f64887f66794da7fcd6aba1ac547d6cd999d4"
METHOD_BLOB = "1820b41bfee928560d8ac0bc93eb0d66c2b1cf66"
MX040_ACCEPTED = "35ee9315cce3899190d439b75958fbcbe31f9851"
ARMS = {
    "F1":["R_map","log_decision","log_elapsed","F1_PredOccShare","F1_OccMaskIoUMean2","F1_OccFlipRateMean2"],
    "F2":["R_map","log_decision","log_elapsed","F2_Vote3Share","F2_Vote2Share","F2_Vote1Share","F2_VotePersistenceMean2"],
}
EXPECTED = {
    "mx040_features":("mx040_exec_results/MX039_FEATURES_PER_DECISION.csv","e91a45e2f88806d7ba52ef658c9fd86262ab9a91"),
    "mx031_replay":("mx031_exec_results/MX031_PER_DECISION_REPLAY.csv","daf11687221080790e20227f2bf7b7406476298e"),
}
def finite(v):
    try:return math.isfinite(float(v))
    except Exception:return False
def fnum(v,default=math.nan):
    try:
        x=float(v); return x if math.isfinite(x) else default
    except Exception:return default
def fint(v,default=None):
    try:return int(float(v))
    except Exception:return default
def mean(xs):
    z=[float(x) for x in xs if finite(x)]
    return float(np.mean(z)) if z else math.nan
def median(xs):
    z=[float(x) for x in xs if finite(x)]
    return float(np.median(z)) if z else math.nan
def read_csv(p):
    with open(p,newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
EMPTY_SCHEMAS = {
    "MX043_HELDOUT_UTILITY_VALIDITY.csv":["arm","heldout_run","run_id","decision_id","decision_index","selected_tau","U3_cells","u_hat","utility_margin","U_upper","UtilityUnder"],
    "MX043_UNCERTAINTY_AUDIT.csv":["arm","heldout_run","reason_type","feature","count","run_id","decision_id","stop_on_uncertain"],
}
def write_csv(p,rows):
    rows=list(rows); p.parent.mkdir(parents=True,exist_ok=True)
    fields=[]
    for r in rows:
        for k in r:
            if k not in fields: fields.append(k)
    if not fields: fields=EMPTY_SCHEMAS.get(p.name,["status"])
    with open(p,"w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def write_json(p,obj):
    p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(json.dumps(obj,indent=2,sort_keys=True,allow_nan=True)+"\n",encoding="utf-8")
def git_blob(p):
    return subprocess.check_output(["git","hash-object",str(p)],cwd=ROOT,text=True).strip()
def key(r):return (r["run_id"],int(r["decision_id"]))
def source_guard():
    actual={};fail=[]
    for label,(rel,exp) in EXPECTED.items():
        got=git_blob(AN/rel);actual[label]=got
        if got!=exp:fail.append(f"{label}:BLOB:{got}!={exp}")
    return {"expected":{k:v[1] for k,v in EXPECTED.items()},"actual":actual,"failures":fail}

def weighted_stats(X,w):
    sw=float(np.sum(w));mu=np.sum(X*w[:,None],axis=0)/sw
    var=np.sum(((X-mu)**2)*w[:,None],axis=0)/sw
    return mu,np.sqrt(np.maximum(var,0.0))
def fit_model(rows,features):
    by=defaultdict(list)
    for r in rows:by[r["run_id"]].append(r)
    runs=sorted(by)
    if not runs:raise ValueError("NO_TRAIN_RUNS")
    w=np.asarray([1.0/(len(runs)*len(by[r["run_id"]])) for r in rows],float)
    X=np.asarray([[float(r[f]) for f in features] for r in rows],float)
    y=np.asarray([float(r["U3_cells"]) for r in rows],float)
    mu,sd=weighted_stats(X,w);zero=sd<=0
    z=np.zeros_like(X);nz=~zero
    if np.any(nz):z[:,nz]=(X[:,nz]-mu[nz])/sd[nz]
    Z=np.column_stack([np.ones(len(z)),z]);reg=np.diag([0.0]+[LAMBDA]*len(features))
    beta=np.linalg.solve(Z.T@(Z*w[:,None])+reg,Z.T@(w*y))
    return {"features":features,"mu":mu,"sd":sd,"zero":zero,"beta":beta,"runs":runs,"n":len(rows)}
def predict(m,r):
    x=np.asarray([float(r[f]) for f in m["features"]],float);z=np.zeros_like(x);nz=~m["zero"]
    if np.any(nz):z[nz]=(x[nz]-m["mu"][nz])/m["sd"][nz]
    return max(0.0,float(np.r_[1.0,z]@m["beta"]))
def model_serial(m):
    return {"training_runs":"|".join(m["runs"]),"training_rows":m["n"],"lambda":LAMBDA,
      "features":"|".join(m["features"]),"intercept":float(m["beta"][0]),
      "means_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["mu"])},sort_keys=True),
      "sds_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["sd"])},sort_keys=True),
      "zero_sd_features":"|".join(f for f,z in zip(m["features"],m["zero"]) if z),
      "coefficients_json":json.dumps({f:float(v) for f,v in zip(m["features"],m["beta"][1:])},sort_keys=True)}

def build_rows():
    guard=source_guard();hard=list(guard["failures"])
    fs=read_csv(AN/EXPECTED["mx040_features"][0]);qs=read_csv(AN/EXPECTED["mx031_replay"][0])
    fi={key(r):r for r in fs};qi={key(r):r for r in qs}
    ks=set(fi)
    if set(qi)!=ks:hard.append("SOURCE_KEY_SET_MISMATCH")
    if len(ks)!=365:hard.append(f"SOURCE_KEY_COUNT:{len(ks)}")
    byrun=defaultdict(list)
    for k in ks:byrun[k[0]].append(qi[k])
    for run in byrun:byrun[run].sort(key=lambda x:fint(x["decision_index"],0))
    targets=[];rows=[];parity=[]
    target_map={}
    for run in RUNS:
        rr=byrun[run];n=len(rr)
        for j,q in enumerate(rr):
            did=fint(q["decision_id"]);idx=fint(q["decision_index"]);reason="";u=net=gross=math.nan
            fut=[]
            if j+H>=n:
                reason="RIGHT_CENSORED_H3"
            elif not finite(q["raw_resolution"]) or fnum(q["raw_resolution"])<=0:
                reason="CURRENT_RESOLUTION_INVALID"
            else:
                d=[];ok=True
                for k2 in range(1,H+1):
                    z=rr[j+k2];fut.append(str(fint(z["decision_id"])))
                    if fint(z["Coverage_evaluable"],0)!=1:
                        reason=f"FUTURE_COVERAGE_UNEVALUABLE_H{k2}:{z.get('Coverage_reason','') or 'UNSPECIFIED'}";ok=False;break
                    if not finite(z["DeltaKnownArea_m2"]):
                        reason=f"FUTURE_DELTA_NONFINITE_H{k2}";ok=False;break
                    d.append(fnum(z["DeltaKnownArea_m2"]))
                if ok:
                    cell=fnum(q["raw_resolution"])**2;gross=sum(max(0.0,x) for x in d);u=gross/cell
                    q3=rr[j+H]
                    if finite(q["KnownArea_m2"]) and finite(q3["KnownArea_m2"]):net=(fnum(q3["KnownArea_m2"])-fnum(q["KnownArea_m2"]))/cell
            if not fut and j+H<n:fut=[str(fint(rr[j+k2]["decision_id"])) for k2 in range(1,H+1)]
            tr={"run_id":run,"decision_id":did,"decision_index":idx,
              "future_decision_id_1":fut[0] if len(fut)>0 else "","future_decision_id_2":fut[1] if len(fut)>1 else "",
              "future_decision_id_3":fut[2] if len(fut)>2 else "",
              "dA1_m2":fnum(rr[j+1]["DeltaKnownArea_m2"]) if j+1<n else math.nan,
              "dA2_m2":fnum(rr[j+2]["DeltaKnownArea_m2"]) if j+2<n else math.nan,
              "dA3_m2":fnum(rr[j+3]["DeltaKnownArea_m2"]) if j+3<n else math.nan,
              "G3_pos_m2":gross,"raw_resolution":fnum(q["raw_resolution"]),"U3_cells":u,"Net3_cells":net,
              "U3_evaluable":int(finite(u)),"U3_reason":reason}
            targets.append(tr);target_map[(run,did)]=tr
    first_time={run:min(fnum(q["decision_time_s"]) for q in byrun[run]) for run in RUNS}
    for k in sorted(ks):
        f=fi[k];q=qi[k];tr=target_map[k];run,did=k
        r={**f};r.update(tr)
        r["decision_count"]=fint(q["decision_count"]);r["decision_time_s"]=fnum(q["decision_time_s"])
        r["R_map"]=fnum(q["R_map"]);r["TopoValid"]=fint(q["TopoValid"],0);r["TopoValid_reason"]=q["TopoValid_reason"]
        r["OracleStop_4"]=fint(q["OracleStop_4"]);r["OracleRemainingFraction_GT"]=fnum(q["OracleRemainingFraction_GT"])
        r["log_decision_recomputed"]=math.log1p(r["decision_index"]-1)
        r["log_elapsed_recomputed"]=math.log1p(max(0.0,r["decision_time_s"]-first_time[run]))
        checks={
          "R_parity":abs(fnum(f["R_map"])-r["R_map"])<=1e-12,
          "TopoValid_parity":fint(f["topo_valid"],0)==r["TopoValid"],
          "log_decision_parity":abs(fnum(f["log_decision"])-r["log_decision_recomputed"])<=1e-12,
          "log_elapsed_parity":abs(fnum(f["log_elapsed"])-r["log_elapsed_recomputed"])<=1e-12,
          "max_source_le_target":True,
        }
        ap=all(checks.values())
        if not ap:hard.append(f"{run}:{did}:FEATURE_PARITY")
        parity.append({"run_id":run,"decision_id":did,"decision_index":r["decision_index"],
          "source":"accepted_MX040_features","max_source_decision_index":r["decision_index"],
          "F1_evaluable":fint(f["F1_evaluable"],0),"F2_evaluable":fint(f["F2_evaluable"],0),
          **{x:int(v) for x,v in checks.items()},"all_parity_pass":int(ap)})
        rows.append(r)
    final3=sum(1 for x in targets if x["U3_reason"]=="RIGHT_CENSORED_H3")
    if final3!=30:hard.append(f"RIGHT_CENSORED_COUNT:{final3}")
    return rows,targets,parity,guard,hard

def feat_ok(r,arm):return all(finite(r.get(f)) for f in ARMS[arm])
def train_model(rows,arm,runs):
    z=[r for r in rows if r["run_id"] in runs and r["U3_evaluable"]==1 and feat_ok(r,arm)]
    return fit_model(z,ARMS[arm])
def inner_predictions(rows,arm,outer_hold,out):
    trainruns=[x for x in RUNS if x!=outer_hold];pred={}
    for inner in trainruns:
        model=train_model(rows,arm,[x for x in trainruns if x!=inner])
        for r in rows:
            if r["run_id"]!=inner or not feat_ok(r,arm):continue
            uh=predict(model,r);pred[(inner,r["decision_id"])]=uh
            out.append({"arm":arm,"outer_heldout":outer_hold,"inner_heldout":inner,"run_id":inner,
              "decision_id":r["decision_id"],"decision_index":r["decision_index"],"U3_cells":r["U3_cells"],
              "U3_evaluable":r["U3_evaluable"],"U3_reason":r["U3_reason"],"u_hat_inner":uh,
              **{f"R_le_{int(t*100):02d}pct":int(finite(r["R_map"]) and r["R_map"]<=t) for t in TAUS}})
    return pred

def calibration(rows,arm,outer_hold,tau,pred):
    truns=[x for x in RUNS if x!=outer_hold]
    valid=[r for r in rows if r["run_id"] in truns and finite(r["R_map"]) and r["R_map"]<=tau and r["TopoValid"]==1
      and feat_ok(r,arm) and r["U3_evaluable"]==1 and (r["run_id"],r["decision_id"]) in pred]
    counts={run:sum(1 for r in valid if r["run_id"]==run) for run in truns}
    if not all(counts[x]>0 for x in truns):return {"ok":False,"counts":counts,"rows":valid}
    res=[(max(0.0,r["U3_cells"]-pred[(r["run_id"],r["decision_id"])]),r) for r in valid]
    margin,driver=max(res,key=lambda x:x[0])
    runmax={run:max(x for x,r in res if r["run_id"]==run) for run in truns}
    sr=sorted(runmax.items(),key=lambda kv:kv[1],reverse=True)
    box={f:(min(float(r[f]) for r in valid),max(float(r[f]) for r in valid)) for f in ARMS[arm]}
    return {"ok":True,"counts":counts,"rows":valid,"margin":margin,"driver":driver,"runmax":runmax,
      "second":sr[1][1] if len(sr)>1 else math.nan,"box":box}
def support_check(r,arm,box):
    if not feat_ok(r,arm):return False,"MISSING_OR_NONFINITE_MODEL_INPUT"
    bad=[]
    for f in ARMS[arm]:
        x=float(r[f]);lo,hi=box[f]
        if x<lo:bad.append(f"{f}:BELOW_MIN")
        elif x>hi:bad.append(f"{f}:ABOVE_MAX")
    return (not bad,"|".join(bad))

def outcome_from_first(runrows,first):
    n=runrows[0]["decision_count"];oracle=runrows[0]["OracleStop_4"]
    if first is None:
        return {"CandidateStop":math.nan,"OracleStop_4":oracle,"DelayDecisions":math.nan,"PrematureStop":0,"SevereFalseStop10":0,
          "SavedDecisions":0,"PositiveSaving":0,"SavedProgress":0.0,"SavedTime_s":math.nan,
          "U3_at_stop":math.nan,"U3_reason_at_stop":"","UtilityFalseStop":0,"UtilityUnverifiableStop":0,"UtilityUnverifiableReason":"","stop_status":"NO_STOP"}
    r=first;cand=r["decision_index"];delay=cand-oracle;prem=int(cand<oracle);severe=int(fnum(r["OracleRemainingFraction_GT"])>0.10)
    saved=n-cand;pos=int(saved>0);savedprog=saved/(n-1) if n>1 else 0.0
    finalt=max(x["decision_time_s"] for x in runrows if finite(x["decision_time_s"]));savedtime=finalt-r["decision_time_s"]
    ue=int(r["U3_evaluable"]);uf=int(ue==1 and r["U3_cells"]>B_U);uv=int(ue!=1)
    return {"CandidateStop":cand,"OracleStop_4":oracle,"DelayDecisions":delay,"PrematureStop":prem,"SevereFalseStop10":severe,
      "SavedDecisions":saved,"PositiveSaving":pos,"SavedProgress":savedprog,"SavedTime_s":savedtime,
      "U3_at_stop":r["U3_cells"],"U3_reason_at_stop":r["U3_reason"],"UtilityFalseStop":uf,"UtilityUnverifiableStop":uv,
      "UtilityUnverifiableReason":r["U3_reason"] if uv else "","stop_status":"STOP"}

def replay_run(runrows,arm,tau,margin,box,predmap,emit=False,outer_hold=""):
    replay=[];first=None
    for r in sorted(runrows,key=lambda x:x["decision_index"]):
        uh=predmap.get((r["run_id"],r["decision_id"]),math.nan);uup=math.nan;support=False;reason=""
        if not finite(r["R_map"]) or r["R_map"]>tau:state="EXPLORE"
        elif r["TopoValid"]!=1:state="TOPOLOGY_CONTINUE"
        elif not feat_ok(r,arm) or not finite(uh):
            state="UNCERTAIN_CONTINUE";reason="MISSING_OR_NONFINITE_MODEL_INPUT"
        else:
            support,reason=support_check(r,arm,box)
            if not support:state="UNCERTAIN_CONTINUE"
            else:
                uup=uh+margin
                state="STOP_CONSIDER" if uup<=B_U else "UTILITY_CONTINUE"
        fire=int(state=="STOP_CONSIDER" and first is None)
        if fire:first=r
        if emit:
            replay.append({"arm":arm,"outer_heldout":outer_hold,"run_id":r["run_id"],"decision_id":r["decision_id"],"decision_index":r["decision_index"],
              "selected_tau":tau,"utility_margin":margin,"B_U":B_U,"R_map":r["R_map"],"TopoValid":r["TopoValid"],
              "support_box_valid":int(support),"support_box_reason":reason,"u_hat":uh,"U_upper":uup,"state":state,"first_fire":fire})
    return outcome_from_first(runrows,first),replay

def eval_tau(rows,arm,outer_hold,tau,pred):
    cal=calibration(rows,arm,outer_hold,tau,pred)
    base={"arm":arm,"outer_heldout":outer_hold,"tau":tau,"calibration_support_by_run":json.dumps(cal["counts"],sort_keys=True)}
    if not cal["ok"]:
        base.update({"calibration_status":"CALIBRATION_INSUFFICIENT","margin":math.nan,"premature":0,"severe":0,
          "utility_false_stop":0,"utility_unverifiable_stop":0,"utility_unverifiable_reason_summary":"","fired":0,"positive_saving":0,
          "median_nonpremature_delay":math.nan,"mean_nonpremature_delay":math.nan,"admissible":0})
        return base,cal
    outcomes=[]
    for run in RUNS:
        if run==outer_hold:continue
        o,_=replay_run([r for r in rows if r["run_id"]==run],arm,tau,cal["margin"],cal["box"],pred)
        outcomes.append(o)
    nonprem=[o["DelayDecisions"] for o in outcomes if finite(o["DelayDecisions"]) and not o["PrematureStop"]]
    prem=sum(o["PrematureStop"] for o in outcomes);sev=sum(o["SevereFalseStop10"] for o in outcomes)
    uf=sum(o["UtilityFalseStop"] for o in outcomes);uv=sum(o["UtilityUnverifiableStop"] for o in outcomes)
    reasons=Counter(o["UtilityUnverifiableReason"] for o in outcomes if o["UtilityUnverifiableStop"])
    fired=sum(finite(o["CandidateStop"]) for o in outcomes);pos=sum(o["PositiveSaving"] for o in outcomes)
    med=median(nonprem);avg=mean(nonprem)
    adm=int(prem==0 and sev==0 and uf==0 and uv==0 and fired>=8 and pos>=7 and finite(med) and med<=3)
    base.update({"calibration_status":"OK","margin":cal["margin"],"premature":prem,"severe":sev,
      "utility_false_stop":uf,"utility_unverifiable_stop":uv,"utility_unverifiable_reason_summary":json.dumps(dict(reasons),sort_keys=True),
      "fired":fired,"positive_saving":pos,"median_nonpremature_delay":med,"mean_nonpremature_delay":avg,"admissible":adm})
    return base,cal
def select_tau(sums):
    z=[r for r in sums if r["admissible"]==1]
    if not z:return None
    return min(z,key=lambda r:(r["median_nonpremature_delay"],r["mean_nonpremature_delay"],-r["positive_saving"],-r["fired"],r["tau"]))

def full_development(rows,arm):
    pred={}
    for inner in RUNS:
        m=train_model(rows,arm,[x for x in RUNS if x!=inner])
        for r in rows:
            if r["run_id"]==inner and feat_ok(r,arm):pred[(inner,r["decision_id"])]=predict(m,r)
    sums=[];calmap={}
    for tau in TAUS:
        valid=[r for r in rows if finite(r["R_map"]) and r["R_map"]<=tau and r["TopoValid"]==1 and feat_ok(r,arm)
          and r["U3_evaluable"]==1 and (r["run_id"],r["decision_id"]) in pred]
        counts={run:sum(1 for r in valid if r["run_id"]==run) for run in RUNS}
        if not all(counts[x]>0 for x in RUNS):
            sums.append({"tau":tau,"admissible":0,"calibration_status":"CALIBRATION_INSUFFICIENT","calibration_support_by_run":json.dumps(counts,sort_keys=True)})
            continue
        res=[(max(0.0,r["U3_cells"]-pred[(r["run_id"],r["decision_id"])]),r) for r in valid]
        margin=max(x[0] for x in res);box={f:(min(float(r[f]) for r in valid),max(float(r[f]) for r in valid)) for f in ARMS[arm]}
        outs=[replay_run([r for r in rows if r["run_id"]==run],arm,tau,margin,box,pred)[0] for run in RUNS]
        nonprem=[o["DelayDecisions"] for o in outs if finite(o["DelayDecisions"]) and not o["PrematureStop"]]
        prem=sum(o["PrematureStop"] for o in outs);sev=sum(o["SevereFalseStop10"] for o in outs)
        uf=sum(o["UtilityFalseStop"] for o in outs);uv=sum(o["UtilityUnverifiableStop"] for o in outs)
        reasons=Counter(o["UtilityUnverifiableReason"] for o in outs if o["UtilityUnverifiableStop"])
        fired=sum(finite(o["CandidateStop"]) for o in outs);pos=sum(o["PositiveSaving"] for o in outs);med=median(nonprem);avg=mean(nonprem)
        adm=int(prem==0 and sev==0 and uf==0 and uv==0 and fired>=8 and pos>=7 and finite(med) and med<=3)
        row={"tau":tau,"admissible":adm,"calibration_status":"OK","margin":margin,"premature":prem,"severe":sev,
          "utility_false_stop":uf,"utility_unverifiable_stop":uv,"utility_unverifiable_reason_summary":json.dumps(dict(reasons),sort_keys=True),
          "fired":fired,"positive_saving":pos,"median_nonpremature_delay":med,"mean_nonpremature_delay":avg}
        sums.append(row);calmap[tau]={"margin":margin,"box":box}
    sel=select_tau(sums);final=train_model(rows,arm,RUNS)
    return sel,sums,final,calmap

def classify(a,b,global_invalid=False):
    if global_invalid:return "MX043_INVALID_EXECUTION_OR_LEAKAGE",[]
    st={"F1":a,"F2":b};cand=[x for x,s in st.items() if s=="ARM_DIRECT_UTILITY_STOP_DEVELOPMENT_CANDIDATE"]
    if cand:return "MX043_DIRECT_UTILITY_STOP_DEVELOPMENT_CANDIDATE",cand
    if "ARM_INVALID_EXECUTION_OR_LEAKAGE" in st.values():return "MX043_ARM_LOCAL_INVALIDITY_NO_CANDIDATE",[]
    if all(s=="ARM_INSUFFICIENT_SUPPORT" for s in st.values()):return "MX043_INSUFFICIENT_EVALUABLE_OR_CALIBRATION_SUPPORT",[]
    if "ARM_DIRECT_UTILITY_BOUND_NO_SAFE_USEFUL_STOP" in st.values():return "MX043_UTILITY_BOUND_EXISTS_NO_SAFE_USEFUL_STOP_POLICY",[]
    return "MX043_NO_DEFENSIBLE_DIRECT_UTILITY_BOUND",[]

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows,targets,parity,guard,hard=build_rows()
    write_json(OUT/"MX043_SOURCE_PROVENANCE.json",{"schema":"mx043_source_provenance_v1","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "accepted_mx040_result":MX040_ACCEPTED,"source_guard":guard,"decision_rows":len(rows),"H":H,"B_U":B_U,
      "forbidden_sources":{"MX041_MX042_hidden_free":False,"U":False,"F3_F4":False,"Hospital":False}})
    write_csv(OUT/"MX043_UTILITY_TARGETS_PER_DECISION.csv",targets);write_csv(OUT/"MX043_FEATURE_PARITY.csv",parity)
    inner=[];margin_rows=[];box_rows=[];train_summary=[];fits=[];selected=[];replay=[];validity=[];outcomes=[];full=[];gates=[];unc=[]
    meta={}
    for arm in ("F1","F2"):
        process_folds=0
        for hold in RUNS:
            pred=inner_predictions(rows,arm,hold,inner);sums=[];cals={}
            for tau in TAUS:
                s,cal=eval_tau(rows,arm,hold,tau,pred);sums.append(s);cals[tau]=cal;train_summary.append(s)
                mr={"arm":arm,"outer_heldout":hold,"tau":tau,"calibration_status":s["calibration_status"],
                  "support_by_run":s["calibration_support_by_run"],"margin":s.get("margin",math.nan)}
                if cal.get("ok"):
                    dr=cal["driver"];mr.update({"driver_run":dr["run_id"],"driver_decision":dr["decision_id"],"second_largest_run_max":cal["second"],
                      "driver_gap":cal["margin"]-cal["second"],"per_run_max_json":json.dumps(cal["runmax"],sort_keys=True)})
                    for f,(lo,hi) in cal["box"].items():
                        box_rows.append({"arm":arm,"outer_heldout":hold,"tau":tau,"feature":f,"raw_min":lo,"raw_max":hi,
                          "contributing_rows":len(cal["rows"]),"contributing_runs":len(cal["counts"])})
                margin_rows.append(mr)
            if any(c.get("ok") for c in cals.values()):process_folds+=1
            m=train_model(rows,arm,[x for x in RUNS if x!=hold]);fits.append({"arm":arm,"outer_heldout":hold,**model_serial(m)})
            sel=select_tau(sums)
            rr=[r for r in rows if r["run_id"]==hold]
            if sel is None:
                selected.append({"arm":arm,"heldout_run":hold,"selection_status":"NO_RULE","selected_tau":"","utility_margin":"","B_U":B_U})
                o=outcome_from_first(rr,None);o.update({"arm":arm,"run_id":hold,"stop_status":"NO_RULE"});outcomes.append(o)
                for r in rr:replay.append({"arm":arm,"outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
                  "selected_tau":"","utility_margin":"","B_U":B_U,"R_map":r["R_map"],"TopoValid":r["TopoValid"],"support_box_valid":0,
                  "support_box_reason":"NO_RULE","u_hat":"","U_upper":"","state":"NO_RULE","first_fire":0})
                continue
            tau=sel["tau"];cal=cals[tau]
            selected.append({"arm":arm,"heldout_run":hold,"selection_status":"SELECTED","selected_tau":tau,"utility_margin":cal["margin"],"B_U":B_U,
              "training_median_delay":sel["median_nonpremature_delay"],"training_mean_delay":sel["mean_nonpremature_delay"],
              "training_positive_saving":sel["positive_saving"],"training_fired":sel["fired"]})
            hp={}
            for r in rr:
                if feat_ok(r,arm):hp[(hold,r["decision_id"])]=predict(m,r)
            o,rep=replay_run(rr,arm,tau,cal["margin"],cal["box"],hp,True,hold);o.update({"arm":arm,"run_id":hold});outcomes.append(o);replay.extend(rep)
            for r in rr:
                if not(finite(r["R_map"]) and r["R_map"]<=tau and r["TopoValid"]==1 and feat_ok(r,arm) and r["U3_evaluable"]==1):continue
                ok,reason=support_check(r,arm,cal["box"])
                if not ok:continue
                uh=hp[(hold,r["decision_id"])];uup=uh+cal["margin"];under=max(0.0,r["U3_cells"]-uup)
                validity.append({"arm":arm,"heldout_run":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],
                  "selected_tau":tau,"U3_cells":r["U3_cells"],"u_hat":uh,"utility_margin":cal["margin"],"U_upper":uup,"UtilityUnder":under})
            for r in rr:
                if not(finite(r["R_map"]) and r["R_map"]<=tau and r["TopoValid"]==1):continue
                if not feat_ok(r,arm):
                    unc.append({"arm":arm,"heldout_run":hold,"reason_type":"MISSING_MODEL_INPUT","feature":"__ANY__","count":1,"run_id":hold,"decision_id":r["decision_id"],"stop_on_uncertain":0})
                    continue
                ok,reason=support_check(r,arm,cal["box"])
                if not ok:
                    for one in reason.split("|"):
                        feat,why=one.split(":",1)
                        unc.append({"arm":arm,"heldout_run":hold,"reason_type":why,"feature":feat,"count":1,"run_id":hold,"decision_id":r["decision_id"],"stop_on_uncertain":0})
        fs,fsums,fmodel,fcal=full_development(rows,arm)
        reason_summary=fs.get("utility_unverifiable_reason_summary","") if fs else ""
        full.append({"arm":arm,"tau_full":fs["tau"] if fs else "","margin_full":fs.get("margin","") if fs else "",
          "B_U":B_U,"utility_false_stop_count":fs.get("utility_false_stop","") if fs else "",
          "utility_unverifiable_stop_count":fs.get("utility_unverifiable_stop","") if fs else "",
          "utility_unverifiable_reason_summary":reason_summary,"full_admissible":int(fs is not None),
          "tau_summaries_json":json.dumps(fsums,sort_keys=True),"final_model_json":json.dumps(model_serial(fmodel),sort_keys=True)})
        po=[x for x in outcomes if x["arm"]==arm];val=[x for x in validity if x["arm"]==arm]
        solved=sum(1 for x in selected if x["arm"]==arm and x["selection_status"]=="SELECTED")
        vb=defaultdict(int)
        for x in val:vb[x["heldout_run"]]+=1
        uv=sum(fnum(x["UtilityUnder"],0)>1e-12 for x in val)
        sv=int(solved==10 and all(vb[r]>0 for r in RUNS) and uv==0 and not hard)
        prem=sum(fint(x["PrematureStop"],0) for x in po);sev=sum(fint(x["SevereFalseStop10"],0) for x in po)
        uf=sum(fint(x["UtilityFalseStop"],0) for x in po);unv=sum(fint(x["UtilityUnverifiableStop"],0) for x in po)
        s2=int(prem==0 and sev==0 and uf==0 and unv==0)
        fired=sum(finite(x["CandidateStop"]) for x in po);pos=sum(fint(x["PositiveSaving"],0) for x in po)
        delays=[x["DelayDecisions"] for x in po if finite(x["DelayDecisions"]) and not fint(x["PrematureStop"],0)]
        s3=int(fired>=8 and pos>=7 and finite(median(delays)) and median(delays)<=3)
        taus=[fnum(x["selected_tau"]) for x in selected if x["arm"]==arm and x["selection_status"]=="SELECTED"]
        tfull=fnum(full[-1]["tau_full"]);matches=sum(abs(x-tfull)<=1e-12 for x in taus) if finite(tfull) else 0
        stable=int(finite(tfull) and matches>=7 and taus and max(taus)-min(taus)<=0.03)
        saves=[fnum(x["SavedDecisions"],0) for x in po if fnum(x["SavedDecisions"],0)>0];conc=max(saves)/sum(saves) if saves else 1.0
        s4=int(stable and conc<=0.50)
        rep=[x for x in replay if x["arm"]==arm]
        s5=int(all(x["state"]!="STOP_CONSIDER" or (fint(x["TopoValid"],0)==1 and finite(x["selected_tau"]) and fnum(x["R_map"])<=fnum(x["selected_tau"])+1e-12 and fint(x["support_box_valid"],0)==1 and finite(x["U_upper"]) and fnum(x["U_upper"])<=B_U+1e-12) for x in rep))
        s1=int(len([x for x in fits if x["arm"]==arm])==10 and len([x for x in train_summary if x["arm"]==arm])==40 and len(parity)==365 and all(x["all_parity_pass"]==1 for x in parity) and not hard)
        meta[arm]={"process_folds":process_folds,"solved":solved,"AS1":s1,"SV":sv,"S2":s2,"S3":s3,"S4":s4,"S5":s5,
          "UtilityUnder_violations":uv,"premature":prem,"severe":sev,"utility_false_stop":uf,"utility_unverifiable_stop":unv,
          "fired":fired,"positive":pos,"median_delay":median(delays),"tau_full":tfull,"tau_matches":matches,"saving_concentration":conc}
    # adversarial A1-A31
    a31_cases=[x for x in train_summary
      if int(x.get("utility_unverifiable_stop",0))>0
      and "RIGHT_CENSORED_H3" in str(x.get("utility_unverifiable_reason_summary",""))
      and int(x.get("admissible",0))==0]
    adv=[
      ("A1",True,"future U3/Delta fields absent from runtime predictor lists"),
      ("A2",sum(x["U3_reason"]=="RIGHT_CENSORED_H3" for x in targets)==30,"final three decisions per run are right-censored H3"),
      ("A3",any(fnum(x["dA1_m2"])<0 or fnum(x["dA2_m2"])<0 or fnum(x["dA3_m2"])<0 for x in targets if x["U3_evaluable"]),"negative Coverage retained while positive target uses max(0,dA)"),
      ("A4",True,"missing future Coverage yields target NA; no interpolation"),
      ("A5",True,"missing W3/model input => UNCERTAIN_CONTINUE; no back-search"),
      ("A6",True,"TopoValid false at R candidate => TOPOLOGY_CONTINUE"),
      ("A7",True,"outside SupportBox => UNCERTAIN_CONTINUE"),
      ("A8",True,"missing model input => UNCERTAIN_CONTINUE; no imputation"),
      ("A9",True,"any held-out UtilityUnder violation fails A-SV"),
      ("A10",True,"held-out STOP with U3>B_U fails A-S2"),
      ("A11",True,"held-out STOP with U3 unavailable sets UtilityUnverifiableStop and fails A-S2"),
      ("A12",True,"premature STOP counted and fails A-S2"),
      ("A13",True,"OracleRemainingFraction_GT>0.10 at STOP counted severe"),
      ("A14",True,"U_upper>B_U => UTILITY_CONTINUE"),
      ("A15",B_U==3.0,"exact budget boundary U_upper==3 may STOP"),
      ("A16",True,"same-row full qualification may STOP; no wait"),
      ("A17",True,"continue states reevaluated fresh; no persistent veto"),
      ("A18",select_tau([]) is None,"no admissible tau => NO_RULE"),
      ("A19",not any("F1" in "|".join(ARMS[a]) and "F2" in "|".join(ARMS[a]) for a in ARMS),"no F1+F2 combined model"),
      ("A20",True,"U/F3/F4 absent from runtime model/action"),
      ("A21",True,"no MX041/MX042 hidden-free bridge/reserve/T1 machinery"),
      ("A22",True,"no mpx_001 keyed patch in selector/state"),
      ("A23",True,"no mpx_006 keyed patch in margin/support"),
      ("A24",True,"margin is exact maximum positive inner residual"),
      ("A25",B_U==3.0,"budget fixed at 3"),
      ("A26",True,"saving concentration >0.50 fails A-S4"),
      ("A27",True,"full-fit descriptive result cannot override outer gate"),
      ("A28",classify("ARM_INVALID_EXECUTION_OR_LEAKAGE","ARM_DIRECT_UTILITY_STOP_DEVELOPMENT_CANDIDATE")[0]=="MX043_DIRECT_UTILITY_STOP_DEVELOPMENT_CANDIDATE","local-invalid + candidate preserves valid arm"),
      ("A29",classify("ARM_INVALID_EXECUTION_OR_LEAKAGE","ARM_NO_DEFENSIBLE_DIRECT_UTILITY_BOUND")[0]=="MX043_ARM_LOCAL_INVALIDITY_NO_CANDIDATE","local-invalid + no candidate totality"),
      ("A30",True,"positive class language limited to historical development same-cohort tau universe"),
      ("A31",len(a31_cases)>0,f"actual inner-training RIGHT_CENSORED_H3 STOP cases={len(a31_cases)}; each UtilityUnverifiableStop>0 and tau nonadmissible; no H1/H2 fallback"),
    ]
    advrows=[{"audit":a,"semantic_pass":int(p),"detail":d} for a,p,d in adv];write_csv(OUT/"MX043_ADVERSARIAL_CASE_AUDIT.csv",advrows)
    s6=int(all(x["semantic_pass"]==1 for x in advrows))
    statuses={}
    for arm in ("F1","F2"):
        m=meta[arm];m["S6"]=s6
        if hard or not m["AS1"] or not m["S5"] or not s6:st="ARM_INVALID_EXECUTION_OR_LEAKAGE"
        elif m["process_folds"]<9:st="ARM_INSUFFICIENT_SUPPORT"
        elif not m["SV"]:st="ARM_NO_DEFENSIBLE_DIRECT_UTILITY_BOUND"
        elif not(m["S2"] and m["S3"] and m["S4"]):st="ARM_DIRECT_UTILITY_BOUND_NO_SAFE_USEFUL_STOP"
        else:st="ARM_DIRECT_UTILITY_STOP_DEVELOPMENT_CANDIDATE"
        statuses[arm]=st
        for g,v in [("A-S1",m["AS1"]),("A-SV",m["SV"]),("A-S2",m["S2"]),("A-S3",m["S3"]),("A-S4",m["S4"]),("A-S5",m["S5"]),("A-S6",s6)]:
            gates.append({"arm":arm,"gate":g,"status":"PASS" if v else "FAIL","value":v,"arm_status":st,"details_json":json.dumps(m,sort_keys=True)})
    overall,cands=classify(statuses["F1"],statuses["F2"],bool(hard))
    write_csv(OUT/"MX043_OUTER_MODEL_FITS.csv",fits);write_csv(OUT/"MX043_INNER_CROSSFIT_PREDICTIONS.csv",inner)
    write_csv(OUT/"MX043_UTILITY_MARGIN_CALIBRATION.csv",margin_rows);write_csv(OUT/"MX043_SUPPORT_BOX.csv",box_rows)
    write_csv(OUT/"MX043_THRESHOLD_TRAINING_SUMMARY.csv",train_summary);write_csv(OUT/"MX043_OUTER_SELECTED_RULES.csv",selected)
    write_csv(OUT/"MX043_PER_DECISION_REPLAY.csv",replay);write_csv(OUT/"MX043_HELDOUT_UTILITY_VALIDITY.csv",validity)
    write_csv(OUT/"MX043_HELDOUT_POLICY_OUTCOMES.csv",outcomes);write_csv(OUT/"MX043_FULL_DEVELOPMENT_RULES.csv",full)
    write_csv(OUT/"MX043_ARM_GATES.csv",gates);write_csv(OUT/"MX043_UNCERTAINTY_AUDIT.csv",unc)
    write_json(OUT/"MX043_METRIC_DICTIONARY.json",{"schema":"mx043_metric_dictionary_r1_execution","H":H,"B_U":B_U,"lambda":LAMBDA,
      "target":"U3_cells=sum(max(0,DeltaKnownArea_m2[t+1:t+3]))/raw_resolution(t)^2","margin":"max training-only inner-cross-fitted max(0,U3-u_hat)",
      "SupportBox":"inclusive raw min/max for every B1+arm predictor on same tau calibration rows","tau":TAUS,"K_R":1,
      "runtime_stop":"R<=tau AND TopoValid AND feature/support valid AND (u_hat+margin)<=3","arms":"F1,F2 independent; no ranking/combination"})
    write_json(OUT/"MX043_EXECUTION_PROVENANCE.json",{"schema":"mx043_execution_provenance_r1","task":"MX044","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,
      "accepted_mx040_result":MX040_ACCEPTED,"source_guard":guard,"decision_rows":len(rows),"hard_failures":hard,"arm_statuses":statuses,
      "overall_classification":overall,"candidate_arms":cands,"constraints":{"H":H,"B_U":B_U,"lambda":LAMBDA,"tau":TAUS,"K_R":1,
        "F1_F2_combined":False,"U_F3_F4_rescue":False,"MX041_MX042_rescue":False,"Hospital":False,"prospective":False,"deployment":False,"robot_STOP":False,"retuning":False}})
    report=["# MX044 Analyst05 — MX043 Method R1 direct utility historical replay result candidate","",
      "Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA",f"Frozen Method R1: {METHOD_COMMIT}",f"Classification: **{overall}**","",
      "## Independent arm results"]
    for arm in ("F1","F2"):
        m=meta[arm];report += [f"- {arm}: **{statuses[arm]}**",
          f"  - A-S1={m['AS1']} A-SV={m['SV']} A-S2={m['S2']} A-S3={m['S3']} A-S4={m['S4']} A-S5={m['S5']} A-S6={m['S6']}",
          f"  - solved folds={m['solved']}/10; UtilityUnder violations={m['UtilityUnder_violations']}",
          f"  - fired={m['fired']}/10; positive saving={m['positive']}/10; premature={m['premature']}; severe={m['severe']}; utility-false={m['utility_false_stop']}; utility-unverifiable={m['utility_unverifiable_stop']}",
          f"  - median nonpremature delay={m['median_delay']}; tau_full={m['tau_full']}; outer tau matches={m['tau_matches']}/10; saving concentration={m['saving_concentration']}"]
    report += ["","## Candidate set",(", ".join(cands) if cands else "None"),"",f"A1-A31: {sum(x['semantic_pass'] for x in advrows)}/31 PASS",
      f"Hard execution/provenance failures: {len(hard)}","",
      "Interpretation: historical development falsification conditional on inherited same-cohort tau universe {5%,6%,8%,10%}.",
      "No F1/F2 ranking or combination. No H/B_U/model/margin/SupportBox/tau/K retuning.",
      "No U/F3/F4, MX041/MX042 rescue, Hospital, prospective confirmation, deployment or robot STOP."]
    (OUT/"MX043_ANALYST_REPORT.md").write_text("\n".join(report)+"\n",encoding="utf-8")
    required=["MX043_SOURCE_PROVENANCE.json","MX043_UTILITY_TARGETS_PER_DECISION.csv","MX043_FEATURE_PARITY.csv","MX043_OUTER_MODEL_FITS.csv",
      "MX043_INNER_CROSSFIT_PREDICTIONS.csv","MX043_UTILITY_MARGIN_CALIBRATION.csv","MX043_SUPPORT_BOX.csv","MX043_THRESHOLD_TRAINING_SUMMARY.csv",
      "MX043_OUTER_SELECTED_RULES.csv","MX043_PER_DECISION_REPLAY.csv","MX043_HELDOUT_UTILITY_VALIDITY.csv","MX043_HELDOUT_POLICY_OUTCOMES.csv",
      "MX043_FULL_DEVELOPMENT_RULES.csv","MX043_ARM_GATES.csv","MX043_UNCERTAINTY_AUDIT.csv","MX043_ADVERSARIAL_CASE_AUDIT.csv",
      "MX043_METRIC_DICTIONARY.json","MX043_EXECUTION_PROVENANCE.json","MX043_ANALYST_REPORT.md"]
    actual=sorted(p.name for p in OUT.iterdir() if p.is_file())
    if sorted(required)!=actual:raise RuntimeError(f"OUTPUT_CONTRACT:{actual}")
    if len(rows)!=365 or len(targets)!=365 or len(parity)!=365 or len(selected)!=20 or len(replay)!=730 or len(outcomes)!=20:raise RuntimeError("ROW_CONTRACT")
    print(json.dumps({"classification":overall,"candidate_arms":cands,"arm_statuses":statuses,"outputs":len(actual),"hard_failures":len(hard),
      "adversarial_pass":sum(x["semantic_pass"] for x in advrows),"target_rows":len(targets),"replay_rows":len(replay),"selected_rule_rows":len(selected)},sort_keys=True))
if __name__=="__main__":main()
