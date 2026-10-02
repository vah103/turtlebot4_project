#!/usr/bin/env python3
"""MX036 exact retrospective execution of accepted MX035 Method V2.

Frozen 365-decision New Room replay only. No retuning, Hospital, MX028,
prospective collection, engineering, deployment, or robot STOP.
"""
from __future__ import annotations

import csv, hashlib, json, math, os, statistics, subprocess, sys
from collections import defaultdict, Counter
from pathlib import Path

SCRIPT=Path(__file__).resolve()
ANALYSIS=SCRIPT.parent
REPO=ANALYSIS.parent.parent
RESULTS=ANALYSIS/"mx035_exec_results"
INPUTS=ANALYSIS/"mx035_inputs"

METHOD="5842dd9a6fb34bb6266904bdd943dfdc514db38b"
METHOD_BLOB="c9b5ad841f505c4abab4edf3131fa80291dde277"
MX031_REV="d90498e1a3ead2a0c170a3fee7c02a1bc9526cca"
MX032_PACK="9aefb2bbe0be8685c8e779f3def677b0b5054e82"
MX034_ACCEPTED_RESULT="62c1cb772edcdaa6b515d7f39860b9a79a36cf7d"

PRIMARY=ANALYSIS/"mx031_exec_results"/"MX031_PER_DECISION_REPLAY.csv"
LORO=ANALYSIS/"mx031_exec_results"/"MX031_LORO_SELECTED_RULES.csv"
MX026=ANALYSIS/"mx026_exec_results"/"MX026_PUR_PER_DECISION.csv"
MX027=ANALYSIS/"mx027_exec_results"/"MX027_IG_COVERAGE_PER_DECISION.csv"
MX027SRC=ANALYSIS/"mx027_exec_results"/"MX027_IG_COVERAGE_SOURCE_PARITY.csv"
MX025SRC=ANALYSIS/"mx025_exec_results"/"MX025_R_MAP_SOURCE_PARITY.csv"
ORACLE=ANALYSIS/"d1"/"results"/"oracle_stop_retro_v1"/"oracle_decisions.csv"
CASEBOOK=INPUTS/"MX032_RUN_EXCEPTION_CASEBOOK.csv"

EXPECTED={
 PRIMARY:"daf11687221080790e20227f2bf7b7406476298e",
 LORO:"cc0d4230a997e50b00f5c776537d59af0815769f",
 MX026:"8e73f4441b1c852f90d801c20f993216bbd1e78a",
 MX027:"dab247d44578d728c44ffc2245ee306d7375d9cb",
 MX027SRC:"629560dba455a91c95f2f636d4d532016fd5a4c1",
 MX025SRC:"0a3af2267f37805e7fc7e1964d89f68d3d49013a",
 ORACLE:"9ae8f0479cf37a2b6aa9426a158f120efdc655cf",
 CASEBOOK:"f465036fbb08de7f790af8943d56d8ad59f1dcb0",
}
RUNS=tuple(f"mpx_{i:03d}" for i in range(1,11))
FIRST_FIRE={"mpx_001":23,"mpx_002":18,"mpx_003":22,"mpx_004":16,"mpx_005":19,"mpx_006":23,"mpx_007":19,"mpx_008":20,"mpx_009":19,"mpx_010":23}
Q=[("Q1",0.50,1),("Q2",0.50,2),("Q3",0.75,1),("Q4",0.75,2)]
HMAX=3
R_TAU=0.05
BREF=3

def git_blob(p): return subprocess.check_output(["git","hash-object",str(p)],cwd=REPO,text=True).strip()
def sha256(p):
    h=hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda:f.read(1<<20),b""): h.update(b)
    return h.hexdigest()
def read_csv(p):
    with p.open("r",encoding="utf-8",newline="") as f: return list(csv.DictReader(f))
def idx(rows):
    out={}
    for r in rows:
        k=(r["run_id"],int(r["decision_id"]))
        if k in out: raise RuntimeError(f"duplicate key {k}")
        out[k]=r
    return out
def blank(v): return v is None or str(v).strip()==""
def finite(v):
    try:return math.isfinite(float(v))
    except:return False
def fv(v,d=math.nan): return float(v) if finite(v) else d
def iv(v,d=None):
    try:
        if blank(v): return d
        x=float(v)
        return int(x) if math.isfinite(x) and int(x)==x else d
    except:return d
def bv(v): return str(v).strip().lower() in {"1","true","yes"}
def same(a,z,tol=1e-12):
    if not finite(a) and not finite(z): return True
    return finite(a) and finite(z) and math.isclose(float(a),float(z),rel_tol=0.0,abs_tol=tol)
def med(xs):
    a=[float(x) for x in xs if finite(x)]
    return statistics.median(a) if a else math.nan
def avg(xs):
    a=[float(x) for x in xs if finite(x)]
    return sum(a)/len(a) if a else math.nan
def clean(v):
    if v is None:return ""
    if isinstance(v,float): return v if math.isfinite(v) else ""
    if isinstance(v,bool):return int(v)
    return v
def write_csv(p,rows):
    keys=[];seen=set()
    for r in rows:
        for k in r:
            if k not in seen: seen.add(k);keys.append(k)
    p.parent.mkdir(parents=True,exist_ok=True)
    with p.open("w",encoding="utf-8",newline="") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in rows:w.writerow({k:clean(r.get(k,"")) for k in keys})
def jclean(v):
    if isinstance(v,dict):return {str(k):jclean(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [jclean(x) for x in v]
    if isinstance(v,float):return v if math.isfinite(v) else None
    return v
def write_json(p,obj):
    p.write_text(json.dumps(jclean(obj),indent=2,sort_keys=True)+"\n",encoding="utf-8")

def source_parity(primary,mx026,mx027,mx027src,oracle):
    p,i26,i27,i27s,io=idx(primary),idx(mx026),idx(mx027),idx(mx027src),idx(oracle)
    fail=[];keys=set(p)
    if len(keys)!=365:fail.append(f"PRIMARY_KEYS={len(keys)}")
    for name,z in [("MX026",i26),("MX027",i27),("MX027SRC",i27s),("ORACLE",io)]:
        if set(z)!=keys:fail.append(f"{name}_KEY_MISMATCH")
    for k in sorted(keys):
        a,b,c,s,o=p[k],i26[k],i27[k],i27s[k],io[k]
        for nm,x,y in [
            ("R_map",a["R_map"],b["R_MapRemainingFraction"]),
            ("R_available",a["R_available"],b["R_map_evaluable"]),
            ("IG_selected",a["IG_selected"],c["IG_selected"]),
            ("IG_evaluable",a["IG_evaluable"],c["IG_evaluable"]),
            ("DeltaKnownArea_m2",a["DeltaKnownArea_m2"],c["DeltaKnownArea_m2"]),
            ("KnownAreaRate_m2_s",a["KnownAreaRate_m2_s"],c["KnownAreaRate_m2_s"]),
            ("raw_resolution",a["raw_resolution"],s["raw_resolution"]),
            ("OracleRemainingFraction_GT",a["OracleRemainingFraction_GT"],o["OracleRemainingFraction_GT"]),
        ]:
            if not same(x,y):fail.append(f"{k}:{nm}")
        for nm,x,y in [
            ("R_reason",a["R_reason"],b["R_map_reason"]),
            ("IG_reason",a["IG_reason"],c["IG_reason"]),
            ("Coverage_reason",a["Coverage_reason"],c["Coverage_reason"]),
            ("Delta_reason",a["DeltaKnownArea_reason"],c["DeltaKnownArea_reason"]),
            ("Rate_reason",a["KnownAreaRate_reason"],c["KnownAreaRate_reason"]),
        ]:
            if str(x).strip()!=str(y).strip():fail.append(f"{k}:{nm}")
        if iv(a["hard_replay_integrity_failure"],1)!=0:fail.append(f"{k}:hard_integrity")
    return fail

def prepare(primary):
    out=defaultdict(list)
    for z in primary:
        r=dict(z)
        r["_did"]=int(r["decision_id"]);r["_i"]=int(r["decision_index"]);r["_N"]=int(r["decision_count"])
        r["_p"]=float(r["normalized_progress"]);r["_time"]=float(r["decision_time_s"])
        r["_Rok"]=bv(r["R_available"]) and finite(r["R_map"]);r["_R"]=fv(r["R_map"])
        r["_topo"]=bv(r["TopoValid"]);r["_state"]=r["FrontierState"]
        st=r["_state"]
        if st=="B_SELECTED_FRONTIER_AVAILABLE":
            if not bv(r["IG_evaluable"]) or not finite(r["IG_selected"]) or float(r["IG_selected"])<0:
                raise RuntimeError(f"{r['run_id']} d{r['_did']}: bad State-B IG")
            r["_IG"]=float(r["IG_selected"])
        elif st in {"A_NO_RUNTIME_SELECTED_FRONTIER","C_FRONTIER_INDETERMINATE"}:
            if not (blank(r["IG_selected"]) or str(r["IG_selected"]).strip().lower()=="nan"):
                raise RuntimeError(f"{r['run_id']} d{r['_did']}: A/C IG not NA")
            r["_IG"]=math.nan
        else: raise RuntimeError(f"{r['run_id']} d{r['_did']}: State D/unknown")

        ce=bv(r["Coverage_evaluable"])
        reasons=[str(r["Coverage_reason"]).strip(),str(r["DeltaKnownArea_reason"]).strip(),str(r["KnownAreaRate_reason"]).strip()]
        r["_gain_valid"]=False;r["_gain"]=math.nan;r["_coverage_kind"]="UNAVAILABLE"
        if ce and not any(reasons):
            if not finite(r["DeltaKnownArea_m2"]) or not finite(r["KnownAreaRate_m2_s"]) or not finite(r["raw_resolution"]) or float(r["raw_resolution"])<=0:
                raise RuntimeError(f"{r['run_id']} d{r['_did']}: malformed coverage")
            d=float(r["DeltaKnownArea_m2"]);rate=float(r["KnownAreaRate_m2_s"]);res=float(r["raw_resolution"])
            if d<0 or rate<0:
                r["_coverage_kind"]="SIGNED_REGRESSION"
            else:
                r["_gain_valid"]=True;r["_gain"]=d/(res*res);r["_coverage_kind"]="VALID_NONNEGATIVE"
        elif "FIRST_DECISION_NO_PREDECESSOR" in reasons:
            r["_coverage_kind"]="FIRST_DECISION_NO_PREDECESSOR"
        out[r["run_id"]].append(r)
    if set(out)!=set(RUNS):raise RuntimeError("run set mismatch")
    for run in RUNS:
        rr=sorted(out[run],key=lambda x:x["_i"])
        if [x["_i"] for x in rr]!=list(range(1,len(rr)+1)):raise RuntimeError(f"{run}: noncontiguous decision_index")
        out[run]=rr
    return out

def refs(rr,pos):
    w=rr[max(0,pos-2):pos+1]
    ig=[r["_IG"] for r in w if r["_state"]=="B_SELECTED_FRONTIER_AVAILABLE" and finite(r["_IG"]) and r["_IG"]>=0]
    gain=[r["_gain"] for r in w if r["_gain_valid"]]
    return med(ig),med(gain),[r["_did"] for r in w],len(ig),len(gain)

def frontier_weak(r,ig_ref,theta):
    if r["_state"]=="A_NO_RUNTIME_SELECTED_FRONTIER":return True,math.nan,"STATE_A_SEMANTIC_WEAK"
    if r["_state"]=="B_SELECTED_FRONTIER_AVAILABLE":
        if not finite(ig_ref):return False,math.nan,"NO_CAUSAL_IG_REFERENCE"
        ig=r["_IG"]
        if ig_ref>0:ratio=ig/ig_ref
        elif ig==0:ratio=0.0
        else:ratio=math.inf
        return ratio<=theta,ratio,("IG_RATIO_WEAK" if ratio<=theta else "IG_RATIO_STRONG")
    if r["_state"]=="C_FRONTIER_INDETERMINATE":return False,math.nan,"STATE_C_RESET"
    raise RuntimeError("State D")

def coverage_weak(r,gain_ref,theta):
    if r["_coverage_kind"]=="SIGNED_REGRESSION":return False,math.nan,"SIGNED_COVERAGE_REGRESSION"
    if not r["_gain_valid"]:return False,math.nan,r["_coverage_kind"]
    if gain_ref>0:ratio=r["_gain"]/gain_ref
    elif r["_gain"]==0:ratio=0.0
    else:ratio=math.inf
    return ratio<=theta,ratio,("GAIN_RATIO_WEAK" if ratio<=theta else "GAIN_RATIO_STRONG")

def replay(rr,qid,theta,J):
    state="RUN";episode=None;ep_seq=0;episodes=[];trace=[];stop=None
    def close_episode(reason,decision_id,state_after):
        nonlocal episode
        if episode is None:return
        e=dict(episode)
        e.update({"terminal_reason":reason,"terminal_decision":decision_id,"terminal_state":state_after})
        episodes.append(e);episode=None

    for pos,r in enumerate(rr):
        state_prior=state
        if state=="RESET_ABSTAIN": state="RUN"
        state_before=state
        rec={
            "run_id":r["run_id"],"decision_id":r["_did"],"decision_index":r["_i"],
            "qid":qid,"theta":theta,"J":J,"state_prior":state_prior,"state_before_processing":state_before,
            "RSmall":int(r["_Rok"] and r["_R"]<=R_TAU),"TopoValid":int(r["_topo"]),"FrontierState":r["_state"],
            "GainCells":r["_gain"],"Coverage_kind":r["_coverage_kind"],
            "candidate_entry":0,"episode_id":"","t0":"","W_ref_decisions":"","IG_ref":"","IG_ref_support":"",
            "Gain_ref":"","Gain_ref_support":"","IGRatio":"","GainRatio":"","FrontierWeak":"","CoverageWeak":"",
            "JointWeak":"","weak_streak":"","episode_age":"","reset_abstain_reason":"","first_STOP_fire":0
        }
        if state in {"STOP_CONSIDER","ABSTAIN_LOCKED"}:
            rec["state_after"]=state;trace.append(rec);continue
        if state=="HARD_INVALID": raise RuntimeError("carried HARD_INVALID")

        rsmall=r["_Rok"] and r["_R"]<=R_TAU
        if state=="RUN":
            if not rsmall:
                state="RUN"
            elif not r["_topo"]:
                state="RESET_ABSTAIN";rec["reset_abstain_reason"]="TOPO_INVALID_AT_ENTRY"
            elif r["_state"]=="C_FRONTIER_INDETERMINATE":
                state="RESET_ABSTAIN";rec["reset_abstain_reason"]="STATE_C_AT_ENTRY"
            elif r["_state"] not in {"A_NO_RUNTIME_SELECTED_FRONTIER","B_SELECTED_FRONTIER_AVAILABLE"}:
                raise RuntimeError("State D entry")
            else:
                igref,gref,w,ign,gn=refs(rr,pos)
                rec.update({"W_ref_decisions":";".join(map(str,w)),"IG_ref":igref,"IG_ref_support":ign,"Gain_ref":gref,"Gain_ref_support":gn})
                if not finite(gref):
                    state="RESET_ABSTAIN";rec["reset_abstain_reason"]="NO_CAUSAL_GAIN_REFERENCE"
                else:
                    ep_seq+=1
                    episode={"run_id":r["run_id"],"qid":qid,"theta":theta,"J":J,"episode_id":ep_seq,"t0":r["_did"],
                             "IG_ref":igref,"IG_ref_support":ign,"Gain_ref":gref,"Gain_ref_support":gn,
                             "W_ref_decisions":";".join(map(str,w)),"final_age":0,"final_weak_streak":0}
                    state="TEMPORAL_RECHECK"
                    rec["candidate_entry"]=1
                    rec["episode_id"]=ep_seq;rec["t0"]=r["_did"];rec["weak_streak"]=0;rec["episode_age"]=0
        elif state=="TEMPORAL_RECHECK":
            assert episode is not None
            rec.update({"episode_id":episode["episode_id"],"t0":episode["t0"],"W_ref_decisions":episode["W_ref_decisions"],
                        "IG_ref":episode["IG_ref"],"IG_ref_support":episode["IG_ref_support"],
                        "Gain_ref":episode["Gain_ref"],"Gain_ref_support":episode["Gain_ref_support"]})
            if not rsmall:
                state="RESET_ABSTAIN";rec["reset_abstain_reason"]="R_NOT_SMALL_OR_UNAVAILABLE"
                close_episode(rec["reset_abstain_reason"],r["_did"],state)
            elif not r["_topo"]:
                state="RESET_ABSTAIN";rec["reset_abstain_reason"]="TOPO_INVALID_DURING_RECHECK"
                close_episode(rec["reset_abstain_reason"],r["_did"],state)
            elif r["_state"]=="C_FRONTIER_INDETERMINATE":
                state="RESET_ABSTAIN";rec["reset_abstain_reason"]="STATE_C_DURING_RECHECK"
                close_episode(rec["reset_abstain_reason"],r["_did"],state)
            elif r["_state"] not in {"A_NO_RUNTIME_SELECTED_FRONTIER","B_SELECTED_FRONTIER_AVAILABLE"}:
                raise RuntimeError("State D recheck")
            else:
                episode["final_age"]+=1
                fw,igr,fr=frontier_weak(r,episode["IG_ref"],theta)
                cw,gr,cr=coverage_weak(r,episode["Gain_ref"],theta)
                joint=fw and cw
                episode["final_weak_streak"]=episode["final_weak_streak"]+1 if joint else 0
                rec.update({"IGRatio":igr,"GainRatio":gr,"FrontierWeak":int(fw),"CoverageWeak":int(cw),"JointWeak":int(joint),
                            "weak_streak":episode["final_weak_streak"],"episode_age":episode["final_age"],
                            "frontier_reason":fr,"coverage_weak_reason":cr})
                if episode["final_weak_streak"]>=J:
                    state="STOP_CONSIDER";rec["first_STOP_fire"]=1;stop=r
                    close_episode("STOP_CONSIDER",r["_did"],state)
                elif episode["final_age"]>=HMAX:
                    state="ABSTAIN_LOCKED"
                    close_episode("HMAX_TIMEOUT",r["_did"],state)
        else: raise RuntimeError(f"unexpected state {state}")
        rec["state_after"]=state;trace.append(rec)

    terminal_reason=""
    final_age="";final_weak=""
    if stop is None and state=="TEMPORAL_RECHECK":
        assert episode is not None
        terminal_reason="END_OF_RUN_BEFORE_HMAX";final_age=episode["final_age"];final_weak=episode["final_weak_streak"]
        close_episode(terminal_reason,rr[-1]["_did"],"TEMPORAL_RECHECK")
        run_status="NO_STOP_END_OF_RUN_RECHECK"
    elif stop is None and state=="ABSTAIN_LOCKED":
        run_status="ABSTAIN_LOCKED";terminal_reason="HMAX_TIMEOUT"
        if episodes: final_age=episodes[-1]["final_age"];final_weak=episodes[-1]["final_weak_streak"]
    elif stop is None:
        run_status="NO_STOP"
    else:
        run_status="FIRED"
    return {"stop":stop,"trace":trace,"episodes":episodes,"terminal_state":state,"run_status":run_status,
            "terminal_reason":terminal_reason,"final_episode_age":final_age,"final_weak_streak":final_weak}

def score(replay_obj,rr):
    stop=replay_obj["stop"];oracle=int(rr[0]["OracleStop_4"]);N=len(rr)
    if stop is None:
        return {"RunOutcome":replay_obj["run_status"],"CandidateStop":"","OracleStop_4":oracle,"DelayDecisions":math.nan,
                "PrematureStop":False,"PrematureByDecisions":0,"SevereFalseStop10":False,"SavedDecisions":0,
                "PositiveSaving":False,"SavedProgress":0.0,"SavedTime_s":math.nan,
                "terminal_reason":replay_obj["terminal_reason"],"final_episode_age":replay_obj["final_episode_age"],
                "final_weak_streak":replay_obj["final_weak_streak"]}
    ic=stop["_i"];delay=ic-oracle
    status="PREMATURE" if delay<0 else ("ON_TARGET" if delay==0 else "LATE")
    saved=N-ic
    return {"RunOutcome":status,"CandidateStop":stop["_did"],"OracleStop_4":oracle,"DelayDecisions":delay,
            "PrematureStop":delay<0,"PrematureByDecisions":max(0,-delay),
            "SevereFalseStop10":float(stop["OracleRemainingFraction_GT"])>0.10,
            "SavedDecisions":saved,"PositiveSaving":saved>0,"SavedProgress":1-stop["_p"],
            "SavedTime_s":rr[-1]["_time"]-stop["_time"],"terminal_reason":"","final_episode_age":"","final_weak_streak":""}

def aggregate(outcomes,n):
    fired=sum(not blank(x["CandidateStop"]) for x in outcomes)
    pos=sum(bool(x["PositiveSaving"]) for x in outcomes)
    prem=sum(bool(x["PrematureStop"]) for x in outcomes)
    sev=sum(bool(x["SevereFalseStop10"]) for x in outcomes)
    d=[x["DelayDecisions"] for x in outcomes if x["RunOutcome"] in {"ON_TARGET","LATE"} and finite(x["DelayDecisions"])]
    return {"truth_valid_run_count":n,"required_fired_run_count":math.ceil(.8*n),"required_positive_saving_run_count":math.ceil(.7*n),
            "fired_run_count":fired,"positive_saving_run_count":pos,"StopCoverage":fired/n,"PositiveSavingCoverage":pos/n,
            "premature_stop_count":prem,"SevereFalseStop10_count":sev,"median_nonpremature_DelayDecisions":med(d),
            "mean_nonpremature_DelayDecisions":avg(d),"nonpremature_delay_support":len(d),
            "MeanSavedProgress":sum(float(x["SavedProgress"]) for x in outcomes)/n,
            "NO_STOP_count":sum(x["RunOutcome"]=="NO_STOP" for x in outcomes),
            "NO_STOP_END_OF_RUN_RECHECK_count":sum(x["RunOutcome"]=="NO_STOP_END_OF_RUN_RECHECK" for x in outcomes),
            "ABSTAIN_LOCKED_count":sum(x["RunOutcome"]=="ABSTAIN_LOCKED" for x in outcomes),"integrity_failure_count":0}
def admissible(a):
    return a["premature_stop_count"]==0 and a["SevereFalseStop10_count"]==0 and a["fired_run_count"]>=a["required_fired_run_count"] and a["positive_saving_run_count"]>=a["required_positive_saving_run_count"] and finite(a["median_nonpremature_DelayDecisions"]) and a["median_nonpremature_DelayDecisions"]<=3 and a["integrity_failure_count"]==0
def selkey(r):return (float(r["median_nonpremature_DelayDecisions"]),float(r["mean_nonpremature_DelayDecisions"]),-int(r["positive_saving_run_count"]),-int(r["fired_run_count"]),float(r["theta"]),-int(r["J"]))

def base_parity(primary,by_run):
    rows=[];fails=[];first={}
    for run in RUNS:
        rr=by_run[run];fire=None;streak=0
        for r in rr:
            rcond=r["_Rok"] and r["_R"]<=R_TAU
            qual=rcond and r["_topo"]
            streak=streak+1 if qual else 0
            first_here=fire is None and streak>=1
            if first_here:fire=r["_did"]
            expected_reset=""
            if qual: expected_reset=""
            elif not r["_Rok"]: expected_reset="R_UNAVAILABLE"
            elif not rcond: expected_reset="R_ABOVE_TAU"
            elif not r["_topo"]: expected_reset="TOPO_INVALID:"+str(r["TopoValid_reason"])
            else: expected_reset="QUALIFY_FALSE"
            checks={
                "family":r["FullFit_family"]=="F0","tau":same(r["FullFit_tau_R_pct"],5.0),"K":iv(r["FullFit_K"])==1,
                "c":blank(r["FullFit_c"]),"RCondition":bv(r["FullFit_RCondition"])==rcond,
                "Qualify":bv(r["FullFit_Qualify"])==qual,"streak":iv(r["FullFit_streak"])==streak,
                "first_fire":bv(r["FullFit_first_fire_here"])==first_here,"reset_reason":str(r["FullFit_reset_reason"])==expected_reset}
            ok=all(checks.values())
            if not ok:fails.append(f"{run}:d{r['_did']}:{[k for k,v in checks.items() if not v]}")
            rows.append({"row_type":"DECISION_PARITY","run_id":run,"decision_id":r["_did"],"parity_pass":int(ok),
                         "recomputed_RCondition":int(rcond),"accepted_RCondition":r["FullFit_RCondition"],
                         "recomputed_Qualify":int(qual),"accepted_Qualify":r["FullFit_Qualify"],
                         "recomputed_streak":streak,"accepted_streak":r["FullFit_streak"],
                         "recomputed_first_fire_here":int(first_here),"accepted_first_fire_here":r["FullFit_first_fire_here"],
                         "recomputed_reset_reason":expected_reset,"accepted_reset_reason":r["FullFit_reset_reason"]})
        first[run]=fire
        ffok=fire==FIRST_FIRE[run]
        if not ffok:fails.append(f"{run}:first_fire={fire}!={FIRST_FIRE[run]}")
        rows.append({"row_type":"FIRST_FIRE_CHECK","run_id":run,"decision_id":"","parity_pass":int(ffok),
                     "recomputed_first_fire":fire,"accepted_first_fire":FIRST_FIRE[run]})
    return rows,fails,first

def make_synth(run="SYNTH"):
    return []
def synthrow(d,R=.049,topo=True,state="B_SELECTED_FRONTIER_AVAILABLE",ig=10.0,gain=10.0):
    return {"run_id":"SYNTH","_did":d,"_i":d,"_N":d,"_p":0.0,"_time":float(d),"_Rok":True,"_R":R,"_topo":topo,"_state":state,
            "_IG":ig if state=="B_SELECTED_FRONTIER_AVAILABLE" else math.nan,
            "_gain_valid":gain is not None and gain>=0,"_gain":gain if gain is not None and gain>=0 else math.nan,
            "_coverage_kind":"VALID_NONNEGATIVE" if gain is not None and gain>=0 else ("SIGNED_REGRESSION" if gain is not None else "UNAVAILABLE"),
            "OracleStop_4":"1","OracleRemainingFraction_GT":"0","TopoValid_reason":"","IG_evaluable":"1",
            "IG_selected":str(ig) if state=="B_SELECTED_FRONTIER_AVAILABLE" else "",
            "Coverage_evaluable":"1","Coverage_reason":"","DeltaKnownArea_m2":str(gain*.01 if gain is not None else ""),
            "DeltaKnownArea_reason":"","KnownAreaRate_m2_s":"1","KnownAreaRate_reason":"","raw_resolution":"0.1"}

def adversarial(by_run,replays,casebook):
    cb={(r["run_id"],int(r["decision_id"])):r for r in casebook};rows=[]
    def tr(run,qid,d):
        return next(x for x in replays[(run,qid)]["trace"] if x["decision_id"]==d)
    # A1
    a1=all(tr("mpx_001",q,20)["state_after"]=="RUN" and tr("mpx_001",q,20)["candidate_entry"]==0 and tr("mpx_001",q,21)["state_after"]=="RESET_ABSTAIN" and tr("mpx_001",q,21)["first_STOP_fire"]==0 for q,_,_ in Q)
    rows.append({"audit":"A1","semantic_pass":int(a1),"detail":"mpx_001 d20 R>5 RUN/no candidate; d21 R<5 Topo invalid RESET/no STOP; base first d23"})
    # A2
    a2=all(tr("mpx_002",q,15)["state_after"]=="RUN" and tr("mpx_002",q,15)["candidate_entry"]==0 and tr("mpx_002",q,18)["candidate_entry"]==1 and tr("mpx_002",q,18)["state_after"]=="TEMPORAL_RECHECK" and tr("mpx_002",q,18)["first_STOP_fire"]==0 for q,_,_ in Q)
    rows.append({"audit":"A2","semantic_pass":int(a2),"detail":"mpx_002 d15 R>5 no candidate; d18 legitimate entry, no entry-row STOP"})
    # A3
    oracle_rows=[]
    for run in RUNS:
        r=next(x for x in by_run[run] if bv(x["Oracle4_marker"]))
        oracle_rows.append(r)
    a3=all(r["_state"]=="B_SELECTED_FRONTIER_AVAILABLE" and finite(r["DeltaKnownArea_m2"]) and float(r["DeltaKnownArea_m2"])>0 for r in oracle_rows)
    rows.append({"audit":"A3","semantic_pass":int(a3),"detail":"10/10 Oracle anchors State B + positive Delta; evaluator-only, not absolute-zero requirement"})
    # A4
    r30=next(x for x in by_run["mpx_001"] if x["_did"]==30)
    fw=all(frontier_weak(r30,1.0,t)[0] for t in (.5,.75))
    a4=r30["_state"]=="A_NO_RUNTIME_SELECTED_FRONTIER" and not finite(r30["_IG"]) and fw and not r30["_topo"] and all(tr("mpx_001",q,30)["first_STOP_fire"]==0 for q,_,_ in Q)
    rows.append({"audit":"A4","semantic_pass":int(a4),"detail":"State A IG NA remains semantically frontier-weak; Topo invalid prevents STOP"})
    # A5: accepted State C semantic + constructed active episode uses same state machine.
    acc=next(x for x in by_run["mpx_004"] if x["_did"]==30)
    syn=[synthrow(1),dict(acc,run_id="SYNTH",_did=2,_i=2,_N=2)]
    a5=True
    for q,t,j in Q:
        rp=replay(syn,q,t,j)
        x=rp["trace"][1]
        a5 &= acc["_state"]=="C_FRONTIER_INDETERMINATE" and not finite(acc["_IG"]) and x["state_after"]=="RESET_ABSTAIN" and x["first_STOP_fire"]==0
    rows.append({"audit":"A5","semantic_pass":int(a5),"detail":"accepted State-C row causes active-episode RESET; IG not imputed; no same-row rearm"})
    # A6: accepted Topo-invalid d21 inserted as next row after synthetic candidate.
    acc=next(x for x in by_run["mpx_001"] if x["_did"]==21)
    syn=[synthrow(1),dict(acc,run_id="SYNTH",_did=2,_i=2,_N=3),synthrow(3,R=.048)]
    a6=True
    for q,t,j in Q:
        rp=replay(syn,q,t,j);x1=rp["trace"][1];x2=rp["trace"][2]
        a6 &= x1["state_after"]=="RESET_ABSTAIN" and x2["state_prior"]=="RESET_ABSTAIN" and x2["state_before_processing"]=="RUN" and x2["candidate_entry"]==1
    rows.append({"audit":"A6","semantic_pass":int(a6),"detail":"Topo-invalid active row resets; immediate next accepted row clears RESET->RUN and can re-arm"})
    # A7
    neg=next(x for x in by_run["mpx_006"] if x["_did"]==24)
    a7=neg["_coverage_kind"]=="SIGNED_REGRESSION" and float(neg["DeltaKnownArea_m2"])<0 and float(neg["KnownAreaRate_m2_s"])<0
    for q,t,j in Q:
        cw,_,reason=coverage_weak(neg,10,t);a7 &= (not cw and reason=="SIGNED_COVERAGE_REGRESSION")
    rows.append({"audit":"A7","semantic_pass":int(a7),"detail":"signed negative Coverage preserved; CoverageWeak=false; no clamp"})
    # A8
    a8=True
    for q,t,j in [("Q2",.5,2),("Q4",.75,2)]:
        d20=tr("mpx_007",q,20);d21=tr("mpx_007",q,21)
        a8 &= d20["JointWeak"]==1 and d20["weak_streak"]==1 and d21["FrontierWeak"]==0 and d21["weak_streak"]==0 and d21["first_STOP_fire"]==0
    rows.append({"audit":"A8","semantic_pass":int(a8),"detail":"mpx_007 J=2 weak streak at d20 resets on real d21 IG rebound"})
    # A9 exact reset + next-decision rearm
    syn=[synthrow(1,R=.049),synthrow(2,R=.051),synthrow(3,R=.048)]
    a9=True
    for q,t,j in Q:
        rp=replay(syn,q,t,j);x0,x1,x2=rp["trace"]
        a9 &= x0["candidate_entry"]==1 and x1["state_after"]=="RESET_ABSTAIN" and x1["candidate_entry"]==0 and x2["state_prior"]=="RESET_ABSTAIN" and x2["state_before_processing"]=="RUN" and x2["candidate_entry"]==1
    rows.append({"audit":"A9","semantic_pass":int(a9),"detail":"synthetic R rebound: no same-row rearm; immediately next row processes under RUN and can re-arm"})
    # A10 timeout on third actual post-entry decision
    syn=[synthrow(1,ig=10,gain=10),synthrow(2,ig=10,gain=10),synthrow(3,ig=10,gain=10),synthrow(4,ig=10,gain=10),synthrow(5,ig=0,gain=0)]
    a10=True
    for q,t,j in Q:
        rp=replay(syn,q,t,j);x3=rp["trace"][3];x4=rp["trace"][4]
        a10 &= x3["episode_age"]==3 and x3["state_after"]=="ABSTAIN_LOCKED" and x3["first_STOP_fire"]==0 and x4["state_before_processing"]=="ABSTAIN_LOCKED" and rp["run_status"]=="ABSTAIN_LOCKED"
    rows.append({"audit":"A10","semantic_pass":int(a10),"detail":"third actual post-entry row reaches H_max=>ABSTAIN_LOCKED; no later rearm"})
    # A11 EOR before Hmax
    syn=[synthrow(1,ig=10,gain=10),synthrow(2,ig=10,gain=10),synthrow(3,ig=10,gain=10)]
    a11=True
    for q,t,j in Q:
        rp=replay(syn,q,t,j)
        a11 &= rp["run_status"]=="NO_STOP_END_OF_RUN_RECHECK" and rp["terminal_reason"]=="END_OF_RUN_BEFORE_HMAX" and rp["final_episode_age"]==2 and rp["stop"] is None and rp["terminal_state"]=="TEMPORAL_RECHECK"
    rows.append({"audit":"A11","semantic_pass":int(a11),"detail":"run ends age=2 unresolved: NO_STOP_END_OF_RUN_RECHECK; no synthetic timeout/decision"})
    return rows

def main():
    RESULTS.mkdir(parents=True,exist_ok=True)
    blob_fail=[]
    for p,e in EXPECTED.items():
        a=git_blob(p)
        if a!=e:blob_fail.append(f"{p.relative_to(REPO)}:{a}!={e}")

    primary=read_csv(PRIMARY);loro=read_csv(LORO);m26=read_csv(MX026);m27=read_csv(MX027);m27s=read_csv(MX027SRC);oracle=read_csv(ORACLE);casebook=read_csv(CASEBOOK)
    value_fail=source_parity(primary,m26,m27,m27s,oracle)
    by_run=prepare(primary)
    parity_rows,base_fail,first=base_parity(primary,by_run)

    # Precompute every frozen run/q replay once.
    replays={};outcomes={};episodes=[]
    monotonicity=0
    for run in RUNS:
        base_first=FIRST_FIRE[run]
        for qid,theta,J in Q:
            rp=replay(by_run[run],qid,theta,J);replays[(run,qid)]=rp
            sc=score(rp,by_run[run]);outcomes[(run,qid)]=sc
            if not blank(sc["CandidateStop"]) and int(sc["CandidateStop"])<base_first:monotonicity+=1
            for e in rp["episodes"]:
                episodes.append({**e,"RunOutcome":sc["RunOutcome"],"CandidateStop":sc["CandidateStop"]})

    # Outer LORO.
    grid=[];selected_rows=[];selected_q={}
    for held in RUNS:
        train=[r for r in RUNS if r!=held];cand=[]
        for qid,theta,J in Q:
            a=aggregate([outcomes[(r,qid)] for r in train],9)
            ok=admissible(a)
            reasons=[]
            if a["premature_stop_count"]:reasons.append("PREMATURE_STOP_GT0")
            if a["SevereFalseStop10_count"]:reasons.append("SEVERE_FALSE_STOP10_GT0")
            if a["fired_run_count"]<8:reasons.append("FIRED_RUN_COUNT_LT8")
            if a["positive_saving_run_count"]<7:reasons.append("POSITIVE_SAVING_RUN_COUNT_LT7")
            if not finite(a["median_nonpremature_DelayDecisions"]):reasons.append("NO_NONPREMATURE_DELAY_SUPPORT")
            elif a["median_nonpremature_DelayDecisions"]>3:reasons.append("MEDIAN_DELAY_GT3")
            row={"heldout_run":held,"training_runs":";".join(train),"qid":qid,"theta":theta,"J":J,**a,"admissible":int(ok),"selector_rank":"","reject_reason":"|".join(reasons)}
            cand.append(row)
        ranked=sorted([x for x in cand if x["admissible"]],key=selkey)
        for rank,x in enumerate(ranked,1):x["selector_rank"]=rank
        grid.extend(cand)
        if not ranked:
            selected_q[held]=None
            selected_rows.append({"heldout_run":held,"fold_status":"NO_ADMISSIBLE_MX035_TEMPORAL_RULE","selected_q":"","theta":"","J":"",
                                  "heldout_status":"NO_RULE","CandidateStop":"","OracleStop_4":int(by_run[held][0]["OracleStop_4"]),"DelayDecisions":"",
                                  "PrematureStop":0,"PrematureByDecisions":0,"SevereFalseStop10":0,"SavedDecisions":0,"PositiveSaving":0,
                                  "SavedProgress":0.0,"SavedTime_s":"","terminal_reason":"","final_episode_age":"","final_weak_streak":""})
        else:
            s=ranked[0];qid=s["qid"];selected_q[held]=qid;sc=outcomes[(held,qid)]
            selected_rows.append({"heldout_run":held,"fold_status":"ADMISSIBLE","selected_q":qid,"theta":s["theta"],"J":s["J"],
                                  "training_median_nonpremature_DelayDecisions":s["median_nonpremature_DelayDecisions"],
                                  "training_mean_nonpremature_DelayDecisions":s["mean_nonpremature_DelayDecisions"],
                                  "training_fired_run_count":s["fired_run_count"],"training_positive_saving_run_count":s["positive_saving_run_count"],
                                  "heldout_status":sc["RunOutcome"],"CandidateStop":sc["CandidateStop"],"OracleStop_4":sc["OracleStop_4"],
                                  "DelayDecisions":sc["DelayDecisions"],"PrematureStop":sc["PrematureStop"],"PrematureByDecisions":sc["PrematureByDecisions"],
                                  "SevereFalseStop10":sc["SevereFalseStop10"],"SavedDecisions":sc["SavedDecisions"],"PositiveSaving":sc["PositiveSaving"],
                                  "SavedProgress":sc["SavedProgress"],"SavedTime_s":sc["SavedTime_s"],"terminal_reason":sc["terminal_reason"],
                                  "final_episode_age":sc["final_episode_age"],"final_weak_streak":sc["final_weak_streak"]})

    # Full fit after outer frozen.
    full=[]
    for qid,theta,J in Q:
        a=aggregate([outcomes[(r,qid)] for r in RUNS],10);ok=admissible(a)
        full.append({"row_type":"CANDIDATE","qid":qid,"theta":theta,"J":J,**a,"admissible":int(ok),"selector_rank":"","selected_full_fit":0,"full_fit_status":""})
    ranked=sorted([x for x in full if x["admissible"]],key=selkey)
    for rank,x in enumerate(ranked,1):x["selector_rank"]=rank
    if ranked:
        ranked[0]["selected_full_fit"]=1;qfull=ranked[0]["qid"];full_status="ADMISSIBLE_FULL_FIT"
    else:qfull=None;full_status="NO_ADMISSIBLE_MX035_FULL_FIT"
    full.append({"row_type":"FULL_FIT_SUMMARY","qid":qfull or "","theta":next((t for q,t,j in Q if q==qfull),""),
                 "J":next((j for q,t,j in Q if q==qfull),""),"truth_valid_run_count":10,
                 "required_fired_run_count":8,"required_positive_saving_run_count":7,
                 "admissible":int(qfull is not None),"selected_full_fit":int(qfull is not None),"full_fit_status":full_status})

    # 365-row wide state replay: all Q plus aliases for held-out selected q.
    wide=[]
    for run in RUNS:
        for pos,r in enumerate(by_run[run]):
            row={"run_id":run,"decision_id":r["_did"],"decision_index":r["_i"],"decision_count":r["_N"],
                 "normalized_progress":r["_p"],"R_available":int(r["_Rok"]),"R_map":r["_R"],"RSmall":int(r["_Rok"] and r["_R"]<=R_TAU),
                 "TopoValid":int(r["_topo"]),"TopoValid_reason":r["TopoValid_reason"],"FrontierState":r["_state"],
                 "IG_selected":r["IG_selected"],"Coverage_evaluable":r["Coverage_evaluable"],"DeltaKnownArea_m2":r["DeltaKnownArea_m2"],
                 "KnownAreaRate_m2_s":r["KnownAreaRate_m2_s"],"raw_resolution":r["raw_resolution"],"GainCells":r["_gain"],
                 "OracleStop_4":r["OracleStop_4"],"OracleRemainingFraction_GT":r["OracleRemainingFraction_GT"],
                 "selected_outer_q":selected_q[run] or ""}
            for qid,theta,J in Q:
                tr=replays[(run,qid)]["trace"][pos]
                for field in ["state_prior","state_before_processing","state_after","candidate_entry","episode_id","t0","W_ref_decisions","IG_ref","IG_ref_support","Gain_ref","Gain_ref_support","IGRatio","GainRatio","FrontierWeak","CoverageWeak","JointWeak","weak_streak","episode_age","reset_abstain_reason","first_STOP_fire"]:
                    row[f"{field}_{qid}"]=tr.get(field,"")
            if selected_q[run]:
                qid=selected_q[run];tr=replays[(run,qid)]["trace"][pos]
                for field in ["state_prior","state_before_processing","state_after","candidate_entry","episode_id","t0","W_ref_decisions","IG_ref","IG_ref_support","Gain_ref","Gain_ref_support","IGRatio","GainRatio","FrontierWeak","CoverageWeak","JointWeak","weak_streak","episode_age","reset_abstain_reason","first_STOP_fire"]:
                    row[f"selected_{field}"]=tr.get(field,"")
            wide.append(row)

    # Gates
    solved=sum(r["fold_status"]=="ADMISSIBLE" for r in selected_rows)
    fired=sum(not blank(r["CandidateStop"]) for r in selected_rows)
    positive=sum(bv(r["PositiveSaving"]) for r in selected_rows)
    prem=sum(bv(r["PrematureStop"]) for r in selected_rows)
    severe=sum(bv(r["SevereFalseStop10"]) for r in selected_rows)
    delays=[r["DelayDecisions"] for r in selected_rows if r["heldout_status"] in {"ON_TARGET","LATE"} and finite(r["DelayDecisions"])]
    md=med(delays);mn=avg(delays)
    s1=solved==10;s2=prem==0 and severe==0;s3=fired>=8 and positive>=7 and finite(md) and md<=3
    solved_ids=[r["selected_q"] for r in selected_rows if r["fold_status"]=="ADMISSIBLE"]
    if qfull is None or not solved_ids:s4=False;exact=0;trange=math.nan;jrange=math.nan
    else:
        qmap={q:(t,j) for q,t,j in Q};exact=sum(q==qfull for q in solved_ids)
        th=[qmap[q][0] for q in solved_ids];js=[qmap[q][1] for q in solved_ids]
        trange=max(th)-min(th);jrange=max(js)-min(js);s4=exact>=7 and trange<=.25 and jrange<=1

    adv=adversarial(by_run,replays,casebook)
    s6=len(adv)==11 and all(r["semantic_pass"] for r in adv)
    hard=len(blob_fail)+len(value_fail)+len(base_fail)
    s5=hard==0 and monotonicity==0
    gates=[
      {"gate":"S1","status":"PASS" if s1 else "FAIL","value":f"{solved}/10","requirement":"10/10 outer folds admissible"},
      {"gate":"S2","status":"PASS" if s2 else "FAIL","value":f"premature={prem};severe={severe}","requirement":"held-out premature=0; SevereFalseStop10=0; retrospective non-regression only"},
      {"gate":"S3","status":"PASS" if s3 else "FAIL","value":f"fired={fired};positive={positive};median_delay={md if finite(md) else 'NA'}","requirement":"fired>=8; positive>=7; finite median delay<=3"},
      {"gate":"S4","status":"PASS" if s4 else "FAIL","value":f"full_status={full_status};q_full={qfull};exact_q={exact};theta_range={trange};J_range={jrange}","requirement":"full fit exists; >=7 exact q; theta range<=0.25; J range<=1"},
      {"gate":"S5","status":"PASS" if s5 else "FAIL","value":f"blob_fail={len(blob_fail)};value_fail={len(value_fail)};base_parity_fail={len(base_fail)};monotonicity={monotonicity}","requirement":"365-row base parity + source/key/value integrity + monotonicity"},
      {"gate":"S6","status":"PASS" if s6 else "FAIL","value":f"adversarial={sum(r['semantic_pass'] for r in adv)}/11","requirement":"A1-A11 exact semantics"},
    ]
    if not s5 or not s6:classification="MX035_INVALID_EXECUTION"
    elif not s2:classification="MX035_TEMPORAL_RECHECK_SAFETY_FAIL"
    elif not (s1 and s3 and s4):classification="MX035_TEMPORAL_RECHECK_NO_STABLE_CANDIDATE"
    else:classification="MX035_TEMPORAL_RECHECK_RETROSPECTIVE_CANDIDATE"

    write_csv(RESULTS/"MX035_PER_DECISION_STATE_REPLAY.csv",wide)
    write_csv(RESULTS/"MX035_TRAINING_TEMPORAL_GRID.csv",grid)
    write_csv(RESULTS/"MX035_LORO_SELECTED_TEMPORAL.csv",selected_rows)
    write_csv(RESULTS/"MX035_FULL_FIT_DESCRIPTIVE.csv",full)
    write_csv(RESULTS/"MX035_STABILITY_GATES.csv",gates)
    write_csv(RESULTS/"MX035_BASE_PARITY_AUDIT.csv",parity_rows)
    write_csv(RESULTS/"MX035_ADVERSARIAL_CASE_AUDIT.csv",adv)
    write_csv(RESULTS/"MX035_EPISODE_SUMMARY.csv",episodes)

    dictionary={
      "schema":"mx035_metric_dictionary_v2_execution","method_commit":METHOD,
      "fixed_backbone":{"R_threshold":0.05,"K":1,"TopoValid_required":True,"B_ref_accepted_decisions":3,"H_max_post_entry_decisions":3},
      "q_domain":{"Q1":{"theta":.5,"J":1},"Q2":{"theta":.5,"J":2},"Q3":{"theta":.75,"J":1},"Q4":{"theta":.75,"J":2}},
      "W_ref":"t0 plus at most two immediately preceding accepted same-run decisions; no back-search",
      "IG_ref":"median finite nonnegative State-B IG in W_ref; NA allowed",
      "GainCells":"DeltaKnownArea_m2 / raw_resolution^2 when Coverage valid and Delta/Rate nonnegative",
      "Gain_ref":"median eligible GainCells in W_ref; required for candidate entry",
      "FrontierWeak":"State A true; State B IGRatio<=theta when causal IG_ref finite; State C resets",
      "CoverageWeak":"GainRatio<=theta for valid nonnegative gain; regression/unavailable false and weak_streak reset",
      "JointWeak":"FrontierWeak AND CoverageWeak","candidate_entry_counts_toward_J":False,
      "RESET_ABSTAIN":"row causing reset cannot rearm; next accepted row clears RESET before evaluating same row once under RUN",
      "ABSTAIN_LOCKED":"only when third actual post-entry accepted decision is processed without STOP",
      "NO_STOP_END_OF_RUN_RECHECK":"end of accepted run while TEMPORAL_RECHECK age<3; ordinary non-fire denominator; no synthetic decision",
      "online_fields":["R_map","R_available","TopoValid","FrontierState","State-B IG_selected","accepted same-run history <=t","Coverage fields","raw_resolution","episode memory"],
      "evaluator_only":["Oracle","P correctness","truth-conditioned U","retrospective/future topology"],
      "classification_order":["MX035_INVALID_EXECUTION","MX035_TEMPORAL_RECHECK_SAFETY_FAIL","MX035_TEMPORAL_RECHECK_NO_STABLE_CANDIDATE","MX035_TEMPORAL_RECHECK_RETROSPECTIVE_CANDIDATE"]
    }
    write_json(RESULTS/"MX035_METRIC_DICTIONARY.json",dictionary)
    provenance={"schema":"mx035_execution_provenance_v2","authorization_task":"MX036","executor_role":"Data & Evidence Analyst successor 04",
                "method_commit":METHOD,"method_blob":METHOD_BLOB,"mx031_source_revision":MX031_REV,"mx032_pack":MX032_PACK,
                "mx034_accepted_result_revision":MX034_ACCEPTED_RESULT,
                "source_blobs":{str(p.relative_to(REPO)):git_blob(p) for p in EXPECTED},
                "expected_source_blobs":{str(p.relative_to(REPO)):e for p,e in EXPECTED.items()},
                "source_blob_failures":blob_fail,"source_value_parity_failures":value_fail,"fixed_base_parity_failures":base_fail,
                "recomputed_fixed_base_first_fires":first,"expected_fixed_base_first_fires":FIRST_FIRE,
                "temporal_monotonicity_violations":monotonicity,"hard_replay_integrity_failure_count":hard,
                "cohort":{"runs":list(RUNS),"decision_rows":len(primary)},"q_domain":[{"qid":q,"theta":t,"J":j} for q,t,j in Q],
                "guards":{"retuning":False,"Hospital":False,"MX028":False,"P_online":False,"U_online":False,"prospective":False,"Engineer":False,"deployment":False,"robot_STOP":False},
                "classification":classification,"runtime":{"python":sys.version.split()[0],"github_run_id":os.environ.get("GITHUB_RUN_ID",""),"github_sha_at_start":os.environ.get("GITHUB_SHA","")}}
    write_json(RESULTS/"MX035_EXECUTION_PROVENANCE.json",provenance)
    report=f"""# MX035 Analyst04 retrospective result candidate — executed under MX036

Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA

Frozen Method V2: {METHOD}

Result classification: **{classification}**

## Outer LORO
- solved folds: {solved}/10
- held-out fired: {fired}/10
- held-out positive-saving: {positive}/10
- held-out premature: {prem}
- held-out SevereFalseStop10: {severe}
- median non-premature delay: {md if finite(md) else 'NA'} decisions
- mean non-premature delay: {mn if finite(mn) else 'NA'} decisions

## Full-fit descriptive
- status: {full_status}
- q_full: {qfull if qfull else 'undefined'}

## Frozen gates
{chr(10).join(f"- {g['gate']}: **{g['status']}** — {g['value']}" for g in gates)}

## Integrity
- 365-row fixed-base parity failures: {len(base_fail)}
- source blob/value parity failures: {len(blob_fail)+len(value_fail)}
- temporal-before-base monotonicity violations: {monotonicity}
- adversarial semantic audits: {sum(r['semantic_pass'] for r in adv)}/11 PASS

Retrospective S2 is only a non-regression check because the frozen 5%/K1 fixed base
already has zero in-sample premature stops on these studied runs.

No retuning, Hospital, MX028, prospective data, engineering, deployment or robot STOP
is authorized or performed.
"""
    (RESULTS/"MX035_ANALYST_REPORT.md").write_text(report,encoding="utf-8")
    required=["MX035_PER_DECISION_STATE_REPLAY.csv","MX035_TRAINING_TEMPORAL_GRID.csv","MX035_LORO_SELECTED_TEMPORAL.csv","MX035_FULL_FIT_DESCRIPTIVE.csv","MX035_STABILITY_GATES.csv","MX035_BASE_PARITY_AUDIT.csv","MX035_ADVERSARIAL_CASE_AUDIT.csv","MX035_EPISODE_SUMMARY.csv","MX035_METRIC_DICTIONARY.json","MX035_EXECUTION_PROVENANCE.json","MX035_ANALYST_REPORT.md"]
    missing=[x for x in required if not (RESULTS/x).exists()]
    if missing:raise RuntimeError(f"missing outputs {missing}")
    print(json.dumps({"classification":classification,"decision_rows":len(wide),"training_grid_rows":len(grid),"loro_rows":len(selected_rows),
                      "full_rows":len(full),"base_parity_rows":len(parity_rows),"episode_rows":len(episodes),
                      "solved_folds":solved,"heldout_fired":fired,"heldout_positive":positive,"heldout_premature":prem,
                      "heldout_severe":severe,"median_delay":md if finite(md) else None,"full_status":full_status,"q_full":qfull,
                      "gates":{g["gate"]:g["status"] for g in gates},"adversarial_pass":sum(r["semantic_pass"] for r in adv),"required_outputs":len(required)},sort_keys=True))

if __name__=="__main__":main()
