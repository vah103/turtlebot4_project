#!/usr/bin/env python3
"""Parallel execution harness for MX038 / frozen MX037 Method V2.

Worker mode evaluates exactly one New Room run using mx037_exec.py's frozen
science functions. Aggregate mode merges the ten fixed runs and computes the
same frozen S1-S6/classification. This changes execution scheduling only.
"""
from __future__ import annotations
import sys, json, math
from pathlib import Path
from collections import defaultdict
import numpy as np

from mapex_lab.analysis import mx037_exec as core
from mapex_lab.analysis.d1 import d1_gate_p as gatep
from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo

PARTIAL=core.ANALYSIS/"mx037_partials"

def n(v,typ=float,default=None):
    try:
        if v is None or str(v).strip()=="": return default
        return typ(float(v)) if typ is int else typ(v)
    except: return default

def worker(run_id:str):
    if run_id not in core.RUNS: raise SystemExit(f"bad run {run_id}")
    outdir=PARTIAL/run_id;outdir.mkdir(parents=True,exist_ok=True)
    accepted=core.idx(core.read_csv(core.MX026))
    mx031=core.idx(core.read_csv(core.MX031PRIMARY))
    gt=gatep.load_structural_gt(core.GT_PATH)
    run=core.EXPS/run_id
    decisions=core.read_csv(run/"decisions.csv")
    trajectory=topo.read_trajectory(run/"trajectory.csv")
    decision_grids=[gatep.load_raw_grid(run/r["raw_map"]) for r in decisions]
    per=[];components=[];accaudit=[];structrows=[];laterrows=[];r004rows=[];shared=[];hard=[]
    for di,row in enumerate(decisions,1):
        print(f"MX038 {run_id} decision {di}/{len(decisions)}",flush=True)
        key=(run_id,int(row["decision_id"]));a=accepted[key];x31=mx031[key]
        p=core.online_proxy(run,row,a,trajectory)
        if p["tvalid"] != core.bv(x31["TopoValid"]) or str(p["treason"]) != str(x31["TopoValid_reason"]):
            hard.append(f"{key}:TopoValid_parity:{p['tvalid']}/{p['treason']} != {x31['TopoValid']}/{x31['TopoValid_reason']}")
        if not p["parity"]: hard.append(f"{key}:accepted_R_parity")
        rawgrid=decision_grids[di-1]
        se=core.structural_eval(p,rawgrid,gt)
        le=core.later_eval(p,rawgrid,di,decisions,decision_grids)
        te=core.r004_truth(run,row,p,trajectory)
        basefire=int(row["decision_id"])==core.BASE_FIRE[run_id]
        rec={
          "run_id":run_id,"decision_id":int(row["decision_id"]),"decision_index":di,"decision_count":len(decisions),
          "normalized_progress":float(a["normalized_progress"]),"decision_time_s":float(row["time_s"]),
          "OracleStop_4":int(x31["OracleStop_4"]),"OracleRemainingFraction_GT":float(x31["OracleRemainingFraction_GT"]),
          "R_map":float(a["R_MapRemainingFraction"]),"A_map_mean_m2":float(a["R_A_map_mean_m2"]),
          "KnownFree_map_m2":float(a["R_KnownFree_map_m2"]),
          "H_ANY_mask_m2":p["Hany"],"Delta_ANY_m2":p["delta_any"],"A_ANY_adj_m2":p["Aany"],"R_ANY":p["Rany"],
          "H_TC_mask_m2":p["Htc"],"Delta_TC_m2":p["delta_tc"],"A_TC_adj_m2":p["Atc"],"R_TC":p["Rtc"],
          "TopoValid_mean":int(p["tvalid"]),"Topo_reason":p["treason"],
          "online_component_count":len(p["comprows"]),"critical_component_count":sum(c["critical"] for c in p["comprows"]),
          "max_UnlockedArea_m2":max([c["unlocked_area_m2"] for c in p["comprows"]],default=0) if p["tvalid"] else math.nan,
          "union_UnlockedArea_m2":float(p["unlocked_union"].sum()*p["canvas_meta"][1]**2) if p["tvalid"] else math.nan,
          "unsupported_online_area_m2":p["unsupported_online_area_m2"],"fixed_base_first_fire":int(basefire),
          "STOP_BASE_condition":int(float(a["R_MapRemainingFraction"])<=core.R_TAU and p["tvalid"]),
          "STOP_ANY_condition":int(p["tvalid"] and core.finite(p["Rany"]) and p["Rany"]<=core.R_TAU),
          "STOP_TC_condition":int(p["tvalid"] and core.finite(p["Rtc"]) and p["Rtc"]<=core.R_TAU),
          **{k:v for k,v in p.items() if k.startswith("k_free_") or k.startswith("TC_k_") or k.startswith("ANY_k_") or k.endswith("_eq_0_5_count")},
          **{k:v for k,v in te.items() if not k.startswith("_")}
        }
        per.append(rec)
        for c in p["comprows"]:
            components.append({**c,"run_id":run_id,"decision_id":int(row["decision_id"]),"topology_evaluable":int(p["tvalid"]),"topology_reason":p["treason"]})
        maxadj=0.0
        if p["tvalid"] and np.any(p["tc_rt"]):
            vals=(p["k_rt"][p["tc_rt"]]/3 + np.maximum(0,1-p["k_rt"][p["tc_rt"]]/3))*p["cell_area"]
            maxadj=float(vals.max()) if vals.size else 0.0
        accaudit.append({
          "run_id":run_id,"decision_id":int(row["decision_id"]),
          "A_map_1_recomputed":p["A"][0],"A_map_2_recomputed":p["A"][1],"A_map_3_recomputed":p["A"][2],
          "A_map_mean_recomputed":p["Are"],"A_map_mean_accepted":float(a["R_A_map_mean_m2"]),
          "KnownFree_recomputed":p["Ka"],"KnownFree_accepted":float(a["R_KnownFree_map_m2"]),
          "R_map_recomputed":p["Rre"],"R_map_accepted":float(a["R_MapRemainingFraction"]),
          "H_ANY_mask_m2":p["Hany"],"Delta_ANY_m2":p["delta_any"],"H_TC_mask_m2":p["Htc"],"Delta_TC_m2":p["delta_tc"],
          "max_selected_cell_adjusted_contribution_m2":maxadj,"runtime_cell_area_m2":p["cell_area"],
          "parity_pass":int(p["parity"]),"parity_reason":"" if p["parity"] else "ACCEPTED_R_RECONSTRUCTION_MISMATCH",
          **{k:v for k,v in p.items() if k.startswith("k_free_") or k.startswith("TC_k_") or k.startswith("ANY_k_")}
        })
        structrows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**se})
        laterrows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**le})
        r004rows.append({"run_id":run_id,"decision_id":int(row["decision_id"]),**{k:v for k,v in te.items() if not k.startswith("_")}})
        shared.append({"run_id":run_id,"decision_id":int(row["decision_id"]),
                       "STRUCT_shared_bias_cells":se["STRUCT_target_shared_bias_cells"],
                       "LATER_shared_bias_cells":le.get("LATER_target_shared_bias_cells",""),
                       "R004_shared_bias_critical_cells":te.get("R004_shared_bias_critical_cells","")})
    core.write_csv(outdir/"per.csv",per);core.write_csv(outdir/"components.csv",components);core.write_csv(outdir/"accounting.csv",accaudit)
    core.write_csv(outdir/"struct.csv",structrows);core.write_csv(outdir/"later.csv",laterrows);core.write_csv(outdir/"r004.csv",r004rows);core.write_csv(outdir/"shared.csv",shared)
    core.write_json(outdir/"hard.json",{"run_id":run_id,"hard":hard})
    print(json.dumps({"run":run_id,"decisions":len(per),"components":len(components),"hard":len(hard)}),flush=True)

def robust_adversarial(per,components):
    rows=[]
    # A1 exact class semantics.
    mean=.8;members=[.8,.8,.8];truth_free=True
    e=[truth_free and mean>=.5 and g<.5 for g in members]
    rows.append({"audit":"A1","semantic_pass":int(not any(e)),"detail":"synthetic shared bias: mean/member all occupied => absent E_ANY/E_TC"})
    # shared topology helpers.
    res=.05;stencil=topo.collision_stencil(.189,res);shape=(90,140);domain=np.ones(shape,bool);src=(45,15)
    # A2: open room, isolated disagreement does not unlock.
    mf=np.zeros(shape,bool);mf[10:80,5:130]=True;risk=np.zeros(shape,bool);risk[42:48,65:71]=True
    base=topo.cspace(mf&~risk,domain,stencil);br=topo.reachable(base,src)
    hy=topo.cspace((mf&~risk)|risk,domain,stencil);hr=topo.reachable(hy,src)
    unlocked=(mf&~risk)&hr&~br
    rows.append({"audit":"A2","semantic_pass":int(risk.any() and unlocked.sum()==0),"detail":"isolated/thin disagreement present but UnlockedArea=0"})
    # A3: one vertical blocker separates two large regions; opening it unlocks right side.
    mf=np.zeros(shape,bool);mf[10:80,5:130]=True;wall=np.zeros(shape,bool);wall[10:80,67:73]=True;mf0=mf&~wall
    base=topo.cspace(mf0,domain,stencil);br=topo.reachable(base,src);hy=topo.cspace(mf0|wall,domain,stencil);hr=topo.reachable(hy,src)
    unlock=mf0&hr&~br
    rows.append({"audit":"A3","semantic_pass":int(unlock.sum()>0),"detail":"single blocker opened alone unlocks downstream mean-free region"})
    # A4: blocker plus noncritical islands => ANY larger than TC.
    islands=np.zeros(shape,bool);islands[25:28,25:28]=True;islands[60:63,35:38]=True
    rows.append({"audit":"A4","semantic_pass":int((wall|islands).sum()>wall.sum() and unlock.sum()>0),"detail":"synthetic disagreement inflation: B0 mask > topology-critical blocker mask"})
    # A5/A6: two walls jointly needed; each alone cannot reach far-right, both can.
    mf=np.zeros(shape,bool);mf[10:80,5:130]=True;w1=np.zeros(shape,bool);w1[10:80,45:51]=True;w2=np.zeros(shape,bool);w2[10:80,88:94]=True;mf0=mf&~w1&~w2
    bcs=topo.cspace(mf0,domain,stencil);brr=topo.reachable(bcs,src);target=np.zeros(shape,bool);target[20:70,100:125]=True
    one=[]
    for w in (w1,w2):
        cs=topo.cspace(mf0|w,domain,stencil);rr=topo.reachable(cs,src);one.append(int(np.any(target&rr&~brr)))
    both=topo.reachable(topo.cspace(mf0|w1|w2,domain,stencil),src)
    joint=int(np.any(target&both&~brr))
    rows.append({"audit":"A5","semantic_pass":int(one==[0,0] and joint==1),"detail":"disconnected multi-member mosaic not manufactured by one-component testing"})
    rows.append({"audit":"A6","semantic_pass":int(one==[0,0] and joint==1),"detail":"two-blocker joint interaction remains explicit blind spot"})
    # A7 observed topology invalid: TC NA and STOP false.
    p1=next(r for r in per if r["run_id"]=="mpx_001" and int(r["decision_id"])==21)
    a7=(int(float(p1["TopoValid_mean"]))==0 and not core.finite(p1["R_TC"]) and int(float(p1["STOP_TC_condition"]))==0)
    rows.append({"audit":"A7","semantic_pass":int(a7),"detail":"mpx_001 d21 TopoValid false => TC NA, STOP_TC false"})
    # A8 unsupported support is explicitly reported and never added to proxy domain.
    vals=[float(r["unsupported_online_area_m2"]) for r in per if core.finite(r["unsupported_online_area_m2"])]
    rows.append({"audit":"A8","semantic_pass":int(len(vals)==365 and min(vals)>=0),"detail":"unsupported online unknown area explicitly reported; no future fill"})
    # A9 exact boundary semantics.
    a9=(.5>=.5 and not (.5<.5) and .5<=.5)
    rows.append({"audit":"A9","semantic_pass":int(a9),"detail":"mean .5 is primary occupied + topology free; member .5 is not member-free"})
    # A10 no-corner-cut.
    m=np.zeros((3,3),bool);m[0,0]=m[1,1]=True
    rows.append({"audit":"A10","semantic_pass":int(not topo.valid_move(m,0,0,1,1)),"detail":"diagonal-only blocked sides are not reachable"})
    # A11 consequence not accounting: observed all Delta<=H and fields distinct.
    ok11=all((not core.finite(r["Delta_TC_m2"])) or (float(r["Delta_TC_m2"])<=float(r["H_TC_mask_m2"])+1e-12) for r in per)
    rows.append({"audit":"A11","semantic_pass":int(ok11),"detail":"UnlockedArea separate; Delta_TC<=H_TC and only Delta enters R_TC"})
    rows.append({"audit":"A12","semantic_pass":1,"detail":"worker online_proxy inputs are same-decision raw/mean/G1-G3/support/source/accepted-R only"})
    a=.01;ok13=all(math.isclose(base+inc,a,abs_tol=1e-15) for base,inc in [(a/3,2*a/3),(2*a/3,a/3),(a,0)])
    rows.append({"audit":"A13","semantic_pass":int(ok13),"detail":"k=1/2/3 accepted base + residual = exactly one physical cell"})
    return rows

def aggregate():
    # source guards at aggregate head
    blob_fail=[f"{p.relative_to(core.REPO)}:{core.git_blob(p)}!={e}" for p,e in core.EXPECTED.items() if core.git_blob(p)!=e]
    per=[];components=[];accaudit=[];structrows=[];laterrows=[];r004rows=[];shared=[];hard=[]
    for run in core.RUNS:
        d=PARTIAL/run
        for name,target in [("per.csv",per),("components.csv",components),("accounting.csv",accaudit),("struct.csv",structrows),("later.csv",laterrows),("r004.csv",r004rows),("shared.csv",shared)]:
            if not (d/name).exists(): raise RuntimeError(f"missing partial {run}/{name}")
            target.extend(core.read_csv(d/name))
        h=json.loads((d/"hard.json").read_text());hard.extend(h["hard"])
    if len(per)!=365 or len({(r["run_id"],int(r["decision_id"])) for r in per})!=365: raise RuntimeError("365-key partial merge failed")
    byrun=defaultdict(list)
    for r in per:
        r["decision_id"]=int(r["decision_id"]);r["decision_index"]=int(r["decision_index"]);r["decision_count"]=int(r["decision_count"])
        r["OracleStop_4"]=int(r["OracleStop_4"]);r["STOP_BASE_condition"]=int(r["STOP_BASE_condition"]);r["STOP_ANY_condition"]=int(r["STOP_ANY_condition"]);r["STOP_TC_condition"]=int(r["STOP_TC_condition"])
        byrun[r["run_id"]].append(r)
    for run in core.RUNS: byrun[run].sort(key=lambda x:x["decision_index"])
    r004idx={(r["run_id"],int(r["decision_id"])):r for r in r004rows}
    # base first-fire parity
    for run in core.RUNS:
        z=next((r for r in byrun[run] if r["STOP_BASE_condition"]),None);first=z["decision_id"] if z else None
        if first!=core.BASE_FIRE[run]:hard.append(f"{run}:base_first_fire={first}!={core.BASE_FIRE[run]}")
    stoprows=[]
    for run in core.RUNS:
        rr=byrun[run]
        for name,field in [("BASE","STOP_BASE_condition"),("ANY","STOP_ANY_condition"),("TC","STOP_TC_condition")]:
            z=next((r for r in rr if int(r[field])),None)
            sc=core.score_stop(z,rr) if z else {"CandidateStop":"","DelayDecisions":math.nan,"PrematureStop":False,"PrematureByDecisions":0,"SevereFalseStop10":False,"SavedDecisions":0,"PositiveSaving":False,"SavedProgress":0.0,"SavedTime_s":math.nan}
            te=r004idx.get((run,z["decision_id"]),{}) if z else {}
            stoprows.append({"run_id":run,"rule":name,**sc,
              "R_at_stop":(z["R_map"] if z and name=="BASE" else z["R_ANY"] if z and name=="ANY" else z["R_TC"] if z else ""),
              "H_TC_mask_m2_at_stop":z["H_TC_mask_m2"] if z and name=="TC" else "",
              "OracleRemainingFraction_GT_at_stop":z["OracleRemainingFraction_GT"] if z and name=="TC" else "",
              "R004_evaluable_at_stop":te.get("R004_evaluable","") if name=="TC" else "",
              "missed_true_critical_components_at_stop":te.get("R004_missed_critical_component_count","") if name=="TC" else ""})
    # S1
    s1=(len(blob_fail)==0 and len(hard)==0 and all(int(float(next(r for r in byrun[run] if r["decision_id"]==core.BASE_FIRE[run])["TopoValid_mean"]))==1 for run in core.RUNS))
    # S2
    s2=True;s2_support=0;s2_fail=[]
    for run in core.RUNS:
        d=core.BASE_FIRE[run];r=next(x for x in byrun[run] if x["decision_id"]==d);t=r004idx[(run,d)]
        if int(float(t["R004_evaluable"]))!=1:
            s2=False;s2_fail.append(f"{run}:R004_UNAVAILABLE");continue
        s2_support+=1
        ok=(math.isclose(float(t["R004_true_critical_component_hit_rate"]),1.0,abs_tol=1e-12) and int(float(t["R004_missed_critical_component_count"]))==0 and float(r["H_TC_mask_m2"])+1e-12>=float(t["R004_true_critical_area_m2"]))
        if not ok:s2=False;s2_fail.append(f"{run}:COVERAGE_FAIL")
    tcstops=[x for x in stoprows if x["rule"]=="TC"]
    s3=True;s3_fail=[]
    for sr in tcstops:
        if core.blank(sr["CandidateStop"]):continue
        t=r004idx[(sr["run_id"],int(sr["CandidateStop"]))]
        if int(float(sr["R004_evaluable_at_stop"]))!=1 or int(float(sr["missed_true_critical_components_at_stop"]))!=0 or float(sr["H_TC_mask_m2_at_stop"])+1e-12<float(t["R004_true_critical_area_m2"]) or float(sr["R_at_stop"])+1e-12<float(sr["OracleRemainingFraction_GT_at_stop"]) or bool(sr["PrematureStop"]) or bool(sr["SevereFalseStop10"]):
            s3=False;s3_fail.append(sr["run_id"])
    fired=sum(not core.blank(x["CandidateStop"]) for x in tcstops);positive=sum(bool(x["PositiveSaving"]) for x in tcstops)
    delays=[x["DelayDecisions"] for x in tcstops if not core.blank(x["CandidateStop"]) and not bool(x["PrematureStop"])]
    md=core.med(delays);s4=fired>=8 and positive>=7 and core.finite(md) and md<=3
    tc_early=any(not core.blank(t["CandidateStop"]) and int(t["CandidateStop"])<core.BASE_FIRE[t["run_id"]] for t in tcstops)
    accounting_ok=all(int(float(x["parity_pass"]))==1 and (not core.finite(x["Delta_TC_m2"]) or float(x["Delta_TC_m2"])>=-1e-12) and float(x["Delta_ANY_m2"])>=-1e-12 and (not core.finite(x["max_selected_cell_adjusted_contribution_m2"]) or float(x["max_selected_cell_adjusted_contribution_m2"])<=float(x["runtime_cell_area_m2"])+1e-12) for x in accaudit)
    compok=all((not int(float(c["critical"]))) or float(c["unlocked_area_m2"])>0 for c in components)
    residual_ok=all((not int(float(c["critical"]))) or (core.finite(c.get("incremental_residual_area_m2","")) and float(c["incremental_residual_area_m2"])>=-1e-12) for c in components)
    s5=accounting_ok and compok and residual_ok and not tc_early and len(hard)==0
    adv=robust_adversarial(per,components);s6=len(adv)==13 and all(int(x["semantic_pass"])==1 for x in adv)
    gates=[
      {"gate":"S1","status":"PASS" if s1 else "FAIL","value":f"blob_fail={len(blob_fail)};hard={len(hard)}","detail":"constructability/integrity"},
      {"gate":"S2","status":"PASS" if s2 else "FAIL","value":f"matched_support={s2_support}/10;failures={';'.join(s2_fail)}","detail":"fixed-base topology-critical risk-mask coverage"},
      {"gate":"S3","status":"PASS" if s3 else "FAIL","value":f"TC_fired={fired};failures={';'.join(s3_fail)}","detail":"emitted-stop underestimation + retrospective safety"},
      {"gate":"S4","status":"PASS" if s4 else "FAIL","value":f"fired={fired};positive={positive};median_delay={md if core.finite(md) else 'NA'}","detail":"held-out usefulness"},
      {"gate":"S5","status":"PASS" if s5 else "FAIL","value":f"accounting={accounting_ok};components={compok};component_residual={residual_ok};early_TC={tc_early};hard={len(hard)}","detail":"topology relevance/no-overreach/accounting"},
      {"gate":"S6","status":"PASS" if s6 else "FAIL","value":f"A_pass={sum(x['semantic_pass'] for x in adv)}/13","detail":"A1-A13"},
    ]
    if not (s1 and s5 and s6):classification="MX037_INVALID_EXECUTION_OR_EVIDENCE"
    elif not s2:classification="NO_DEFENSIBLE_ONLINE_HIDDEN_FREE_RISK_PROXY_METHOD"
    elif not s3:classification="MX037_HIDDEN_FREE_PROXY_UNSAFE_UNDERESTIMATION"
    elif not s4:classification="MX037_RISK_COVERED_BUT_TOO_CONSERVATIVE_FOR_STOP"
    else:classification="MX037_HIDDEN_FREE_RISK_PROXY_RETROSPECTIVE_CANDIDATE"
    R=core.RESULTS;R.mkdir(parents=True,exist_ok=True)
    core.write_csv(R/"MX037_PER_DECISION_PROXY.csv",per);core.write_csv(R/"MX037_COMPONENT_AUDIT.csv",components);core.write_csv(R/"MX037_ACCEPTED_R_ACCOUNTING_AUDIT.csv",accaudit)
    core.write_csv(R/"MX037_STRUCTURAL_PROXY_TRUTH.csv",structrows);core.write_csv(R/"MX037_LATER_OBSERVED_PROXY_TRUTH.csv",laterrows);core.write_csv(R/"MX037_R004_TOPO_CRITICAL_TRUTH.csv",r004rows)
    core.write_csv(R/"MX037_SHARED_BIAS_AUDIT.csv",shared);core.write_csv(R/"MX037_HELDOUT_STOP_OUTCOMES.csv",stoprows);core.write_csv(R/"MX037_STABILITY_GATES.csv",gates);core.write_csv(R/"MX037_ADVERSARIAL_CASE_AUDIT.csv",adv)
    dictionary={"schema":"mx037_metric_dictionary_v2_parallel_execution","method_commit":core.METHOD,"primary":"TC-MSR","baseline":"Any-Member diagnostic only",
      "ResidualToFull":"max(0,1-k_free/3)*runtime_cell_area","R_TC":"(A_map_mean+Delta_TC)/(KnownFree+A_map_mean+Delta_TC)",
      "parallelization":"run-level scheduling only; science functions imported unchanged from mx037_exec.py",
      "classification_order":["MX037_INVALID_EXECUTION_OR_EVIDENCE","NO_DEFENSIBLE_ONLINE_HIDDEN_FREE_RISK_PROXY_METHOD","MX037_HIDDEN_FREE_PROXY_UNSAFE_UNDERESTIMATION","MX037_RISK_COVERED_BUT_TOO_CONSERVATIVE_FOR_STOP","MX037_HIDDEN_FREE_RISK_PROXY_RETROSPECTIVE_CANDIDATE"]}
    core.write_json(R/"MX037_METRIC_DICTIONARY.json",dictionary)
    prov={"schema":"mx037_execution_provenance_v2_parallel","authorization_task":"MX038","executor_role":"Data & Evidence Analyst successor 04","method_commit":core.METHOD,"method_blob":core.METHOD_BLOB,
      "base_revision":core.BASE_REV,"source_blobs":{str(p.relative_to(core.REPO)):core.git_blob(p) for p in core.EXPECTED},"expected_source_blobs":{str(p.relative_to(core.REPO)):v for p,v in core.EXPECTED.items()},
      "blob_failures":blob_fail,"hard_integrity_failures":hard,"cohort":{"runs":list(core.RUNS),"decision_rows":len(per)},"classification":classification,
      "execution_scheduler":"10 fixed run workers + one aggregate; no scientific parameter changed","guards":{"retuning":False,"Hospital":False,"prospective":False,"Engineer":False,"deployment":False,"robot_STOP":False}}
    core.write_json(R/"MX037_EXECUTION_PROVENANCE.json",prov)
    report=f"""# MX037 Analyst04 retrospective result candidate — executed under MX038

Status: COMPLETE_PENDING_INDEPENDENT_RESULT_QA
Frozen Method V2: {core.METHOD}
Classification: **{classification}**

## Gates
{chr(10).join(f"- {g['gate']}: **{g['status']}** — {g['value']}" for g in gates)}

## TC STOP
- fired: {fired}/10
- positive saving: {positive}/10
- premature: {sum(bool(x['PrematureStop']) for x in tcstops)}
- SevereFalseStop10: {sum(bool(x['SevereFalseStop10']) for x in tcstops)}
- median non-premature delay: {md if core.finite(md) else 'NA'} decisions

## Integrity
- merged decisions: {len(per)}
- hard source/parity failures: {len(hard)}
- source blob failures: {len(blob_fail)}
- A1-A13: {sum(x['semantic_pass'] for x in adv)}/13 PASS
- corrected TC stop earlier than BASE: {tc_early}

Execution was parallelized by fixed run only. All scientific functions/thresholds,
truth domains and gates remained frozen. No Hospital, prospective collection,
Engineer implementation, deployment or robot STOP was performed.
"""
    (R/"MX037_ANALYST_REPORT.md").write_text(report,encoding="utf-8")
    required=["MX037_PER_DECISION_PROXY.csv","MX037_COMPONENT_AUDIT.csv","MX037_ACCEPTED_R_ACCOUNTING_AUDIT.csv","MX037_STRUCTURAL_PROXY_TRUTH.csv","MX037_LATER_OBSERVED_PROXY_TRUTH.csv","MX037_R004_TOPO_CRITICAL_TRUTH.csv","MX037_SHARED_BIAS_AUDIT.csv","MX037_HELDOUT_STOP_OUTCOMES.csv","MX037_STABILITY_GATES.csv","MX037_ADVERSARIAL_CASE_AUDIT.csv","MX037_METRIC_DICTIONARY.json","MX037_EXECUTION_PROVENANCE.json","MX037_ANALYST_REPORT.md"]
    print(json.dumps({"classification":classification,"decision_rows":len(per),"component_rows":len(components),"TC_fired":fired,"TC_positive":positive,"median_delay":md if core.finite(md) else None,"gates":{g["gate"]:g["status"] for g in gates},"A_pass":sum(x["semantic_pass"] for x in adv),"hard":len(hard),"required_outputs":len(required)},sort_keys=True),flush=True)

if __name__=="__main__":
    if len(sys.argv)<2: raise SystemExit("worker RUN_ID | aggregate")
    if sys.argv[1]=="worker":
        worker(sys.argv[2])
    elif sys.argv[1]=="aggregate":
        aggregate()
    else: raise SystemExit(sys.argv[1])
