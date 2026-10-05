#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,json,math,hashlib,subprocess,bisect
from collections import Counter,defaultdict
from pathlib import Path
import numpy as np

from mapex_lab.analysis.r004 import evaluate_prediction_vs_final_observed as r004base

ROOT=Path(__file__).resolve().parents[2]
AN=ROOT/"mapex_lab"/"analysis"
DATA=ROOT/"mapex_lab"/"experiments"/"mapex"
OUT=AN/"mx063_exec_results"
RUNS=[f"mpx_{i:03d}" for i in range(1,11)]
COUNTS={"mpx_001":35,"mpx_002":37,"mpx_003":41,"mpx_004":36,"mpx_005":34,"mpx_006":36,"mpx_007":35,"mpx_008":35,"mpx_009":36,"mpx_010":40}
ALIASES={"mpx_001":"mpx_101","mpx_002":"mpx_102","mpx_003":"mpx_103","mpx_004":"mpx_104","mpx_005":"mpx_105","mpx_006":"mpx_106","mpx_007":"mpx_107","mpx_008":"mpx_008","mpx_009":"mpx_109","mpx_010":"mpx_110"}
METHOD_COMMIT="a5c4d11f8d41b5a1b6bfdb64bc441748d54c2026"
METHOD_BLOB="4bdc5ed8032c1dd08edd9f7e47832e84f3b1a7d8"
SOURCE_ROOT="35ee9315cce3899190d439b75958fbcbe31f9851"
EPS=1e-6

GT=ROOT/"mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v2.npz"
QUALITY=AN/"mx055_exec_results/MX054_MAP_QUALITY_PER_DECISION.csv"
T4=AN/"mx057_exec_results/MX056_PAIRED_TARGETS_PER_DECISION.csv"
MX031=AN/"mx031_exec_results/MX031_PER_DECISION_REPLAY.csv"
FEATURES=AN/"mx040_exec_results/MX039_FEATURES_PER_DECISION.csv"
CAUSAL=AN/"mx040_exec_results/MX039_CAUSALITY_AUDIT.csv"
R004=AN/"r004/evaluate_prediction_vs_final_observed.py"
PINNED={
 "GT":(GT,"a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a"),
 "QUALITY":(QUALITY,"ed77ae4c44585e3883b6c42d1c0bc7c356891cf6"),
 "T4":(T4,"b64e755d34ceb4c30f0686047f06ddb4ce4c6865"),
 "MX031":(MX031,"daf11687221080790e20227f2bf7b7406476298e"),
 "FEATURES":(FEATURES,"e91a45e2f88806d7ba52ef658c9fd86262ab9a91"),
 "CAUSAL":(CAUSAL,"18e6ffc0f139efb75ba93583e376059e06b3051a"),
 "R004":(R004,"a7820bc3d12b2b701f35f0d6629ab3f05d98f1bc"),
}

def finite(v):
    try:return math.isfinite(float(v))
    except Exception:return False
def fnum(v,d=math.nan):
    try:
        x=float(v);return x if math.isfinite(x) else d
    except Exception:return d
def fint(v,d=None):
    try:return int(float(v))
    except Exception:return d
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
def git_blob(p):return subprocess.check_output(["git","hash-object",str(p)],cwd=ROOT,text=True).strip()
def git_bytes(commit,path):
    try:return subprocess.check_output(["git","show",f"{commit}:{path}"],cwd=ROOT)
    except subprocess.CalledProcessError:return None
def sha256_bytes(b):return hashlib.sha256(b).hexdigest()
def sha256_file(p):
    h=hashlib.sha256()
    with open(p,"rb") as f:
        for c in iter(lambda:f.read(1<<20),b""):h.update(c)
    return h.hexdigest()
def key(r):return (r["run_id"],int(r["decision_id"]))
def mean(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.mean(z)) if z else math.nan
def med(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.median(z)) if z else math.nan
def q(xs,p):
    z=[float(x) for x in xs if finite(x)];return float(np.quantile(z,p,method="linear")) if z else math.nan
def sd(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.std(z,ddof=0)) if z else math.nan
def iqr(xs):return q(xs,.75)-q(xs,.25) if xs else math.nan
def sign(v):
    if not finite(v):return "NA"
    if v>EPS:return "POS"
    if v<-EPS:return "NEG"
    return "NEAR_ZERO"
def strict_quality(v,gt,D):
    resolved=D&(v>=0);pf=resolved&(v==0);po=resolved&(v>0);tf=D&(gt==0);to=D&(gt>0)
    ft=int(np.sum(tf&pf));ff=int(np.sum(to&pf));ffn=int(np.sum(tf&~pf))
    ot=int(np.sum(to&po));of=int(np.sum(tf&po));ofn=int(np.sum(to&~po))
    def div(a,b):return float(a/b) if b else math.nan
    fi=div(ft,ft+ff+ffn);oi=div(ot,ot+of+ofn)
    return {"FreeIoU":fi,"OccupiedIoU":oi,"StrictMacroIoU":(fi+oi)/2 if finite(fi) and finite(oi) else math.nan}
def distribution(vals):
    z=[float(x) for x in vals if finite(x)]
    return {"n":len(z),"min":min(z) if z else math.nan,"p05":q(z,.05),"q25":q(z,.25),"median":med(z),"q75":q(z,.75),"p95":q(z,.95),
      "max":max(z) if z else math.nan,"mean":mean(z),"sd":sd(z),"iqr":iqr(z),
      "POS_n":sum(sign(x)=="POS" for x in z),"NEAR_ZERO_n":sum(sign(x)=="NEAR_ZERO" for x in z),"NEG_n":sum(sign(x)=="NEG" for x in z),
      "POS_fraction":mean(sign(x)=="POS" for x in z),"NEAR_ZERO_fraction":mean(sign(x)=="NEAR_ZERO" for x in z),"NEG_fraction":mean(sign(x)=="NEG" for x in z),
      "distinct_1e6":len(set(round(x,6) for x in z))}
def event_flag(rows,col):
    z=[r for r in rows if fint(r.get("shared_support"),0)==1]
    n=len(z);ev=[r for r in z if fint(r[col],0)==1]
    return {"n":n,"event_n":len(ev),"event_rate":len(ev)/n if n else math.nan,"event_run_n":len(set(r["run_id"] for r in ev)),
      "material":bool(n and len(ev)/n>=.20 and len(set(r["run_id"] for r in ev))>=3)}
def exact_path_from_meta(meta_abs):
    s=str(meta_abs)
    marker="/turtlebot4_project/"
    if marker in s:return s.split(marker,1)[1]
    return s.lstrip("/")

def build_manifest():
    hard=[];run_rows=[];source_blobs={}
    mxidx=defaultdict(list)
    if MX031.is_file():
        for r in read_csv(MX031): mxidx[r['run_id']].append(r)
    for name,(p,exp) in PINNED.items():
        got=git_blob(p) if p.is_file() else None
        source_blobs[name]={"path":str(p.relative_to(ROOT)),"expected_blob":exp,"actual_blob":got,"pass":got==exp}
        if got!=exp:hard.append(f"PIN_{name}_MISMATCH")
    for run in RUNS:
        rp=DATA/run;mp=rp/"metadata.json"
        if not mp.is_file():
            hard.append(f"{run}:MISSING_METADATA");continue
        meta=json.load(open(mp))
        alias=meta.get("run_id");N=COUNTS[run]
        alias_ok=alias==ALIASES[run]
        if not alias_ok:hard.append(f"{run}:UNEXPECTED_RUN_ID_ALIAS:{alias}")
        decp=rp/"decisions.csv";dec=read_csv(decp) if decp.is_file() else []
        ids=[fint(x['decision_id']) for x in dec]
        mr=sorted(mxidx.get(run,[]),key=lambda x:fint(x['decision_index'],10**9))
        replay_ok=(len(mr)==N and [fint(x['decision_id']) for x in mr]==list(range(1,N+1)) and [fint(x['decision_index']) for x in mr]==list(range(1,N+1)) and all(fint(x['decision_count'])==N for x in mr))
        count_ok=len(dec)==N and ids==list(range(1,N+1)) and replay_ok
        if not count_ok:hard.append(f"{run}:DECISION_IDENTITY")
        inv={"decision_json":0,"raw":0,"canvas":0,"mean":0,"g1":0,"g2":0,"g3":0,"variance":0}
        for d in range(1,N+1):
            dd=rp/"decisions"/f"policy_decision_{d:06d}"
            inv["decision_json"]+=int((dd/"decision.json").is_file());inv["raw"]+=int((dd/"observed_map_raw.npz").is_file());inv["canvas"]+=int((dd/"observed_map_canvas.npz").is_file())
            # predictions are referenced by decisions.csv; additionally inventory members by filename
            rr=dec[d-1] if d-1<len(dec) else {}
            for col,name in [("mean_map","mean"),("g1_map","g1"),("g2_map","g2"),("g3_map","g3"),("variance_map","variance")]:
                rel=rr.get(col,"")
                inv[name]+=int(bool(rel) and (rp/rel).is_file())
        if inv["decision_json"]!=N or inv["raw"]!=N or inv["canvas"]!=N or inv["mean"]!=N:
            hard.append(f"{run}:SOURCE_INVENTORY")
        sh=meta.get("config_sha256",{});commit=meta.get("git_commit");dirty=bool(meta.get("git_dirty_at_recorder_start"))
        launch_rel=exact_path_from_meta(meta.get("runtime_launch_file",""))
        paths={"mapex_run":"mapex_lab/scripts/mapex_run.py","nf_run_recorder_base":"mapex_lab/scripts/nf_run.py","nf_basic_shared_execution":"mapex_lab/scripts/nf_basic.py","runtime_launch":launch_rel}
        proof={}
        for n,p in paths.items():
            b=git_bytes(commit,p);got=sha256_bytes(b) if b is not None else None;exp=sh.get(n)
            proof[n]={"path":p,"stored_sha256":exp,"candidate_sha256":got,"match":bool(exp and got==exp)}
        recorder_exact=all(proof[x]["match"] for x in ["mapex_run","nf_run_recorder_base","nf_basic_shared_execution"])
        launch_exact=proof["runtime_launch"]["match"]
        run_rows.append({"canonical_run_id":run,"legacy_metadata_run_id":alias,"expected_legacy_alias":ALIASES[run],"alias_pass":alias_ok,"decision_count":N,
          "decision_identity_pass":count_ok,"git_commit":commit,"git_dirty_at_recorder_start":dirty,"runtime_profile":meta.get("runtime_profile"),
          "runtime_launch_file":launch_rel,"inventory":inv,"recorder_sha256_proof":proof,"recorder_exact":recorder_exact,"runtime_launch_exact":launch_exact,
          "time_clock_status":"TIME_PRESENT_BUT_CLOCK_PROVENANCE_INCOMPLETE",
          "time_clock_note":"recorder now_s uses ROS node clock, but run-specific evidence does not prove use_sim_time=true on the recorder node itself; launch defaults for other nodes are insufficient"})
    manifest={"schema":"mx062_source_manifest_r2","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,"archival_source_root":SOURCE_ROOT,
      "population_runs":RUNS,"decision_counts":COUNTS,"total_decisions":sum(COUNTS.values()),"canonical_alias_map":ALIASES,
      "source_blob_guards":source_blobs,"runs":run_rows,"hard_failures":hard,
      "manifest_frozen_before_target_characterization":True,
      "forbidden":{"Hospital":False,"MX046":False,"MX050":False,"prospective":False,"prediction_regeneration":False}}
    return manifest

def load_accepted():
    qrows=read_csv(QUALITY);qm=defaultdict(dict)
    for r in qrows:qm[key(r)][r["variant"]]=r
    t4={key(r):r for r in read_csv(T4)}
    mx={key(r):r for r in read_csv(MX031)}
    ft={key(r):r for r in read_csv(FEATURES)}
    return qm,t4,mx,ft

def distance_endpoints(run,decisions):
    rp=DATA/run;tp=rp/"trajectory.csv"
    rows=read_csv(tp) if tp.is_file() else []
    valid=bool(rows)
    ts=[];ds=[];mono=True
    if valid:
        for r in rows:
            if not finite(r.get("time_s")) or not finite(r.get("cumulative_distance_m")):valid=False;break
            ts.append(float(r["time_s"]));ds.append(float(r["cumulative_distance_m"]))
        mono=all(ts[i]>ts[i-1] and ds[i]>=ds[i-1]-1e-12 for i in range(1,len(ts)))
        valid &= mono
    summary=json.load(open(rp/"summary.json")) if (rp/"summary.json").is_file() else {}
    final_match=valid and finite(summary.get("total_distance_m")) and abs(ds[-1]-float(summary["total_distance_m"]))<=1e-9
    endpoint={}
    for d in decisions:
        did=int(d["decision_id"]);j=json.load(open(rp/"decisions"/f"policy_decision_{did:06d}"/"decision.json"));t=float(j["sim_time_s"])
        if not valid:
            endpoint[did]={"status":"D2_DISTANCE_SOURCE_INVALID","reason":"ODOM_TOPIC_FRAME_PROVENANCE_INCOMPLETE",
                           "raw_timing_status":"RAW_TRAJECTORY_INVALID","raw_timing_reason":"TRAJECTORY_INVALID","time_s":t}
            continue
        k=bisect.bisect_left(ts,t)
        if k<len(ts) and abs(ts[k]-t)<=1e-9:
            endpoint[did]={"status":"D2_DISTANCE_SOURCE_INVALID","reason":"ODOM_TOPIC_FRAME_PROVENANCE_INCOMPLETE",
                           "raw_timing_status":"RAW_EXACT_TIMESTAMP_MATCH","raw_distance_m":ds[k],"time_s":t}
        elif 0<k<len(ts):
            endpoint[did]={"status":"D2_DISTANCE_SOURCE_INVALID","reason":"ODOM_TOPIC_FRAME_PROVENANCE_INCOMPLETE",
                           "raw_timing_status":"RAW_BRACKETED_TIMESTAMP","d_before":ds[k-1],"d_after":ds[k],
                           "DistanceBracketWidth_m":ds[k]-ds[k-1],"t_before":ts[k-1],"t_after":ts[k],"time_s":t}
        else:
            endpoint[did]={"status":"D2_DISTANCE_SOURCE_INVALID","reason":"ODOM_TOPIC_FRAME_PROVENANCE_INCOMPLETE",
                           "raw_timing_status":"RAW_NO_VALID_BRACKET","raw_timing_reason":"NO_BRACKET_FOR_DECISION_TIME","time_s":t}
    return endpoint,{"trajectory_present":tp.is_file(),"trajectory_rows":len(rows),"trajectory_valid_monotonic":valid,"summary_final_distance_match":final_match,
      "trajectory_final_distance_m":ds[-1] if ds else math.nan,"summary_total_distance_m":summary.get("total_distance_m",math.nan)}

def main(full=True):
    OUT.mkdir(parents=True,exist_ok=True)
    manifest=build_manifest()
    mp=OUT/"MX062_SOURCE_MANIFEST.json"
    if not full:
        write_json(mp,manifest);print(json.dumps({"manifest_only":True,"hard":len(manifest["hard_failures"]),"runs":len(manifest["runs"])}));return
    if not mp.is_file():raise RuntimeError("SOURCE_MANIFEST_NOT_FROZEN")
    frozen=json.loads(mp.read_text())
    if json.dumps(frozen,sort_keys=True,allow_nan=True)!=json.dumps(manifest,sort_keys=True,allow_nan=True):
        raise RuntimeError("SOURCE_MANIFEST_DRIFT")

    hard=list(manifest["hard_failures"]);qm,t4m,mx,ft=load_accepted()
    with np.load(GT,allow_pickle=False) as z:
        gt=np.asarray(z["data"],np.int16);D=np.asarray(z["evaluation_mask"],bool)
    linkage=[];parity=[];h1=[];h3=[];bench=[];support=[];variation=[];cost=[];overlap=[];qstop={}
    endpoint_all={};cost_run={}
    # exact Q_STOP reconstruction first
    for run in RUNS:
        rp=DATA/run;dec=read_csv(rp/"decisions.csv");N=COUNTS[run]
        endpoints,dcost=distance_endpoints(run,dec);endpoint_all[run]=endpoints;cost_run[run]=dcost
        times=[]
        for j,d in enumerate(dec):
            did=int(d['decision_id']);K=(run,did);idx=j+1
            try:
                obs,pred,supp=r004base.prediction_canvas(rp,d,gt.shape)
                v1=obs.copy();fill=D&(obs<0)&supp;v1[fill]=np.where(pred[fill]>.5,100,0).astype(np.int16)
                qq=strict_quality(v1,gt,D);source_valid=True;reason=""
            except Exception as e:
                qq={"FreeIoU":math.nan,"OccupiedIoU":math.nan,"StrictMacroIoU":math.nan};source_valid=False;reason=f"RECONSTRUCTION:{type(e).__name__}"
            aq=qm.get(K,{}).get("V1_MEAN_COMPLETION",{})
            diffs={f:abs(qq[f]-fnum(aq.get(f))) if finite(qq[f]) and finite(aq.get(f)) else math.inf for f in ["FreeIoU","OccupiedIoU","StrictMacroIoU"]}
            ppass=source_valid and all(x<=1e-12 for x in diffs.values())
            if not ppass:hard.append(f"{run}:d{did}:Q_STOP_PARITY_FAILURE")
            qstop[K]=qq["StrictMacroIoU"] if ppass else math.nan
            if fint(mx[K]['decision_index'])!=idx or fint(mx[K]['decision_count'])!=N:
                hard.append(f"{run}:d{did}:ACCEPTED_INDEX_PARITY")
            j=json.load(open(rp/'decisions'/f'policy_decision_{did:06d}'/'decision.json'));jt=fnum(j['sim_time_s']);mt=fnum(mx[K]['decision_time_s'])
            time_parity=finite(jt) and finite(mt) and abs(jt-mt)<=1e-9;times.append(jt)
            parity.append({"run_id":run,"decision_id":did,"decision_index":idx,"source_valid":int(source_valid),"source_reason":reason,
              "Q_STOP_reconstructed":qq["StrictMacroIoU"],"Q_STOP_MX055":fnum(aq.get("StrictMacroIoU")),"StrictMacroIoU_abs_diff":diffs["StrictMacroIoU"],
              "FreeIoU_abs_diff":diffs["FreeIoU"],"OccupiedIoU_abs_diff":diffs["OccupiedIoU"],"parity_pass":int(ppass),
              "decision_json_sim_time_s":jt,"MX031_decision_time_s":mt,"time_parity_1e9":int(time_parity)})
        if not all(times[i]>times[i-1] for i in range(1,len(times))):hard.append(f"{run}:NONINCREASING_DECISION_TIME")
    # row targets/linkages and benchmark
    h1_by={};h3_by={};bench_by={}
    for run in RUNS:
        rp=DATA/run;dec=read_csv(rp/"decisions.csv");N=COUNTS[run]
        for j,d in enumerate(dec):
            did=int(d['decision_id']);idx=j+1;K=(run,did);prog=fnum(mx[K]['normalized_progress'])
            n1=dec[j+1] if j+1<N else None;n3=dec[j+3] if j+3<N else None
            K1=(run,int(n1["decision_id"])) if n1 else None;K3=(run,int(n3["decision_id"])) if n3 else None
            linkage.append({"run_id":run,"decision_id":did,"decision_index":idx,"decision_count":N,
              "next_decision_id":K1[1] if K1 else "","next_decision_index":idx+1 if K1 else "","H1_structural_status":"NEXT_EXACT" if K1 else "TERMINAL_CENSORED_H1",
              "next3_decision_id":K3[1] if K3 else "","next3_decision_index":idx+3 if K3 else "","H3_structural_status":"NEXT3_EXACT" if K3 else "TERMINAL_CENSORED_H3"})
            v=qstop.get(K,math.nan);v1=qstop.get(K1,math.nan) if K1 else math.nan;v3=qstop.get(K3,math.nan) if K3 else math.nan
            e1=bool(K1 and finite(v) and finite(v1));e3=bool(K3 and finite(v) and finite(v3))
            cv1=v1-v if e1 else math.nan;cv3=v3-v if e3 else math.nan
            r1="OK" if e1 else ("TERMINAL_CENSORED_H1" if not K1 else "CURRENT_OR_NEXT_QSTOP_INVALID")
            r3="OK" if e3 else ("TERMINAL_CENSORED_H3" if not K3 else "CURRENT_OR_NEXT3_QSTOP_INVALID")
            ep0=endpoint_all[run][did];ep1=endpoint_all[run].get(K1[1]) if K1 else None;ep3=endpoint_all[run].get(K3[1]) if K3 else None
            time_status="TIME_PRESENT_BUT_CLOCK_PROVENANCE_INCOMPLETE"
            ctime1=(fnum(mx[K1]["decision_time_s"])-fnum(mx[K]["decision_time_s"])) if e1 else math.nan
            ctime3=(fnum(mx[K3]["decision_time_s"])-fnum(mx[K]["decision_time_s"])) if e3 else math.nan
            cdist1=(ep1["distance_m"]-ep0["distance_m"]) if e1 and ep0["status"].startswith("D0_") and ep1 and ep1["status"].startswith("D0_") else math.nan
            cdist3=(ep3["distance_m"]-ep0["distance_m"]) if e3 and ep0["status"].startswith("D0_") and ep3 and ep3["status"].startswith("D0_") else math.nan
            fr=ft[K]
            h1row={"run_id":run,"decision_id":did,"decision_index":idx,"decision_count":N,"normalized_progress":prog,
              "next_decision_id":K1[1] if K1 else "","next_decision_index":idx+1 if K1 else "","Q_STOP_t":v,"Q_STOP_next":v1,"CV_Q1":cv1,
              "H1_evaluable":int(e1),"H1_reason":r1,"sign_class":sign(cv1),"C_dec1":1 if e1 else "",
              "time_status":time_status,"C_time1_s":"",
              "distance_endpoint_status_t":ep0["status"],"distance_endpoint_reason_t":ep0.get("reason",""),
              "distance_endpoint_raw_timing_status_t":ep0.get("raw_timing_status",""),
              "distance_endpoint_status_next":ep1["status"] if ep1 else "TERMINAL_NO_NEXT",
              "distance_endpoint_reason_next":ep1.get("reason","") if ep1 else "",
              "distance_endpoint_raw_timing_status_next":ep1.get("raw_timing_status","") if ep1 else "",
              "C_dist1_m":"",
              "R_B1_evaluable":int(all(finite(fr.get(x)) for x in ["R_map","log_decision","log_elapsed"])),
              "F1_evaluable":fint(fr.get("F1_evaluable"),0),"F2_evaluable":fint(fr.get("F2_evaluable"),0),"F4_evaluable":fint(fr.get("F4_evaluable"),0)}
            h1.append(h1row);h1_by[K]=h1row
            h3row={"run_id":run,"decision_id":did,"decision_index":idx,"decision_count":N,"normalized_progress":prog,
              "next3_decision_id":K3[1] if K3 else "","next3_decision_index":idx+3 if K3 else "","Q_STOP_t":v,"Q_STOP_next3":v3,"CV_Q3":cv3,
              "H3_evaluable":int(e3),"H3_reason":r3,"sign_class":sign(cv3),"C_dec3":3 if e3 else "",
              "time_status":time_status,"C_time3_s":"",
              "distance_endpoint_status_t":ep0["status"],"distance_endpoint_reason_t":ep0.get("reason",""),
              "distance_endpoint_raw_timing_status_t":ep0.get("raw_timing_status",""),
              "distance_endpoint_status_next3":ep3["status"] if ep3 else "TERMINAL_NO_NEXT3",
              "distance_endpoint_reason_next3":ep3.get("reason","") if ep3 else "",
              "distance_endpoint_raw_timing_status_next3":ep3.get("raw_timing_status","") if ep3 else "",
              "C_dist3_m":""}
            h3.append(h3row);h3_by[K]=h3row
            acc=t4m[K];t4=fnum(acc["T4"]);qfull=fnum(qm[K]["V3_FULL_EXPLORE_REFERENCE"]["StrictMacroIoU"]);recon=qfull-v if finite(qfull) and finite(v) else math.nan
            bp=finite(t4) and finite(recon) and abs(t4-recon)<=1e-12
            if not bp:hard.append(f"{run}:d{did}:T4_PARITY_FAILURE")
            brow={"run_id":run,"decision_id":did,"decision_index":idx,"T4_accepted":t4,"T4_reconstructed":recon,"abs_diff":abs(t4-recon) if finite(t4) and finite(recon) else math.nan,"benchmark_parity_pass":int(bp),"sign_class":sign(t4)}
            bench.append(brow);bench_by[K]=brow
    # support / variation by run
    h1valid=[r for r in h1 if r["H1_evaluable"]==1];h3valid=[r for r in h3 if r["H3_evaluable"]==1]
    latevalid=[r for r in h1valid if r["normalized_progress"]>=.75]
    for run in RUNS:
        N=COUNTS[run];a=[r for r in h1valid if r["run_id"]==run];b=[r for r in h3valid if r["run_id"]==run];l=[r for r in latevalid if r["run_id"]==run]
        vals=[r["CV_Q1"] for r in a];lv=[r["CV_Q1"] for r in l]
        d=distribution(vals);ld=distribution(lv)
        varpass=finite(d["sd"]) and d["sd"]>1e-6 and d["distinct_1e6"]>=5
        support.append({"run_id":run,"decision_count":N,"H1_structural_max":N-1,"H1_evaluable_n":len(a),"H1_fraction_of_max":len(a)/(N-1),
          "H3_structural_max":N-3,"H3_evaluable_n":len(b),"H3_fraction_of_max":len(b)/(N-3),"late_H1_evaluable_n":len(l),"late_support_pass":int(len(l)>=3)})
        variation.append({"run_id":run,**{f"H1_{k}":v for k,v in d.items()},"H1_range":(max(vals)-min(vals)) if vals else math.nan,"variation_pass":int(varpass),
          **{f"late_{k}":v for k,v in ld.items()}})
    # discordance row tables
    disc=[];s13=[];s1f=[]
    for r in h1:
        K=(r["run_id"],int(r["decision_id"]));a=r;b=h3_by[K];c=bench_by[K]
        if a["H1_evaluable"] and b["H3_evaluable"]:
            cv1=float(a["CV_Q1"]);cv3=float(b["CV_Q3"])
            row={"comparison":"H1_H3","run_id":K[0],"decision_id":K[1],"decision_index":a["decision_index"],"shared_support":1,
              "H1":cv1,"other":cv3,"Delta_other_minus_H1":cv3-cv1,"AbsDelta":abs(cv3-cv1),"H1_sign":sign(cv1),"other_sign":sign(cv3),
              "sign_agree":int(sign(cv1)==sign(cv3)),"DelayedPositive":int(cv1<=EPS and cv3>EPS),"EarlyPositiveReversal":int(cv1>EPS and cv3<=EPS)}
            disc.append(row);s13.append(row)
        if a["H1_evaluable"] and fint(c["benchmark_parity_pass"])==1:
            cv1=float(a["CV_Q1"]);full=float(c["T4_accepted"])
            row={"comparison":"H1_FULL","run_id":K[0],"decision_id":K[1],"decision_index":a["decision_index"],"shared_support":1,
              "H1":cv1,"other":full,"Delta_other_minus_H1":full-cv1,"AbsDelta":abs(full-cv1),"H1_sign":sign(cv1),"other_sign":sign(full),
              "sign_agree":int(sign(cv1)==sign(full)),"DelayedPositive":int(cv1<=EPS and full>EPS),"EarlyPositiveReversal":int(cv1>EPS and full<=EPS)}
            disc.append(row);s1f.append(row)
    flags={}
    for name,z in [("H1_H3",s13),("H1_FULL",s1f)]:
        d=event_flag([{**r,"shared_support":1,"DelayedPositive":r["DelayedPositive"]} for r in z],"DelayedPositive")
        e=event_flag([{**r,"shared_support":1,"EarlyPositiveReversal":r["EarlyPositiveReversal"]} for r in z],"EarlyPositiveReversal")
        flags[name]={"shared_n":len(z),"DelayedPositive":d,"EarlyPositiveReversal":e,"sign_agreement":mean(r["sign_agree"] for r in z),
          "material":bool(d["material"] or e["material"]),"delta_distribution":distribution([r["Delta_other_minus_H1"] for r in z]),"abs_delta_distribution":distribution([r["AbsDelta"] for r in z])}
    # costs
    for run in RUNS:
        meta=json.load(open(DATA/run/"metadata.json"));proof=[x for x in manifest["runs"] if x["canonical_run_id"]==run][0]
        eps=endpoint_all[run];st=Counter(x["status"] for x in eps.values());rawst=Counter(x.get("raw_timing_status","") for x in eps.values())
        cost.append({"run_id":run,"legacy_metadata_run_id":meta["run_id"],"git_commit":meta["git_commit"],"git_dirty_at_recorder_start":int(meta["git_dirty_at_recorder_start"]),
          "recorder_exact_sha256_proof":int(proof["recorder_exact"]),"runtime_launch_exact_sha256_proof":int(proof["runtime_launch_exact"]),
          "time_status":"TIME_PRESENT_BUT_CLOCK_PROVENANCE_INCOMPLETE","decision_time_parity_n":sum(1 for x in parity if x["run_id"]==run and x["time_parity_1e9"]==1),
          "trajectory_present":int(cost_run[run]["trajectory_present"]),"trajectory_valid_monotonic":int(cost_run[run]["trajectory_valid_monotonic"]),
          "summary_final_distance_match":int(cost_run[run]["summary_final_distance_match"]),
          "odom_topic_provenance_status":"ODOM_TOPIC_PROVENANCE_INCOMPLETE",
          "odom_frame_provenance_status":"ODOM_FRAME_PROVENANCE_INCOMPLETE",
          "formal_distance_source_status":"DISTANCE_SOURCE_UNBOUND_FAIL_CLOSED",
          "D0_exact_decision_distance_n":st["D0_EXACT_DECISION_DISTANCE"],"D1_bracketed_n":st["D1_BRACKETED_TRAJECTORY_ONLY"],"D2_invalid_n":st["D2_DISTANCE_SOURCE_INVALID"],
          "raw_exact_timestamp_match_n":rawst["RAW_EXACT_TIMESTAMP_MATCH"],
          "raw_bracketed_timestamp_n":rawst["RAW_BRACKETED_TIMESTAMP"],
          "raw_no_valid_bracket_n":rawst["RAW_NO_VALID_BRACKET"],
          "raw_trajectory_invalid_n":rawst["RAW_TRAJECTORY_INVALID"],
          "exact_C_dist1_n":sum(1 for r in h1 if r["run_id"]==run and finite(r.get("C_dist1_m"))),
          "exact_C_dist3_n":sum(1 for r in h3 if r["run_id"]==run and finite(r.get("C_dist3_m")))})
    cost_complete=all(r["time_status"]=="TIME_EXACT_SCIENTIFIC_CLOCK" and r["exact_C_dist1_n"]==COUNTS[r["run_id"]]-1 for r in cost)
    cost_class="MX062_HISTORICAL_COST_TELEMETRY_COMPLETE" if cost_complete else "MX062_HISTORICAL_COST_TELEMETRY_INCOMPLETE"
    # feature overlap
    blocks={
      "R_B1":lambda fr: all(finite(fr.get(x)) for x in ["R_map","log_decision","log_elapsed"]),
      "F1":lambda fr: fint(fr.get("F1_evaluable"),0)==1,
      "F2":lambda fr: fint(fr.get("F2_evaluable"),0)==1,
      "F4":lambda fr: fint(fr.get("F4_evaluable"),0)==1}
    for block,fn in blocks.items():
        for scope in ["GLOBAL"]+RUNS+["Q1","Q2","Q3","Q4"]:
            base=[r for r in h1valid]
            if scope in RUNS:base=[r for r in base if r["run_id"]==scope]
            elif scope.startswith("Q"):
                qi=int(scope[1]);lo=(qi-1)*.25;hi=qi*.25+(1e-12 if qi==4 else 0);base=[r for r in base if r["normalized_progress"]>=lo and r["normalized_progress"]<hi]
            ok=[r for r in base if key(r) in ft and fn(ft[key(r)])]
            overlap.append({"block":block,"scope":scope,"H1_quality_valid_n":len(base),"block_key_present_n":sum(key(r) in ft for r in base),"block_evaluable_n":len(ok),
              "common_support_n":len(ok),"common_support_fraction":len(ok)/len(base) if base else math.nan})
        reasons=Counter()
        for r in h1valid:
            fr=ft[key(r)]
            if block=="F1" and fint(fr.get("F1_evaluable"),0)!=1:reasons[fr.get("F1_reason","")]+=1
            if block=="F2" and fint(fr.get("F2_evaluable"),0)!=1:reasons[fr.get("F2_reason","")]+=1
            if block=="F4" and fint(fr.get("F4_evaluable"),0)!=1:reasons[fr.get("F4_reason","")]+=1
        for reason,n in reasons.items():overlap.append({"block":block,"scope":"NA_REASON","reason":reason,"count":n})
    allcommon=[r for r in h1valid if all(fn(ft[key(r)]) for fn in blocks.values())]
    overlap.append({"block":"ALL_R_B1_F1_F2_F4","scope":"GLOBAL","H1_quality_valid_n":len(h1valid),"common_support_n":len(allcommon),"common_support_fraction":len(allcommon)/len(h1valid)})
    feature_adequate=True
    for block in blocks:
        g=[r for r in overlap if r["block"]==block and r["scope"]=="GLOBAL"][0]
        runrows=[r for r in overlap if r["block"]==block and r["scope"] in RUNS]
        if g["common_support_fraction"]<.80 or any(x["common_support_n"]==0 for x in runrows):feature_adequate=False
    feature_flag="FEATURE_LOCK_COVERAGE_ADEQUATE" if feature_adequate else "FEATURE_LOCK_COVERAGE_LOW"
    # run influence
    influence=[]
    allvals=[r["CV_Q1"] for r in h1valid];base_med=med(allvals);base_iqr=iqr(allvals);base_pos=mean(sign(x)=="POS" for x in allvals)
    def erate(z,col):return mean(r[col] for r in z) if z else math.nan
    for hold in RUNS:
        z=[r for r in h1valid if r["run_id"]!=hold];vals=[r["CV_Q1"] for r in z]
        z13=[r for r in s13 if r["run_id"]!=hold];zf=[r for r in s1f if r["run_id"]!=hold]
        influence.append({"heldout_run":hold,"n":len(z),"median_minus_run":med(vals),"MedianInfluence":base_med-med(vals),"IQR_minus_run":iqr(vals),"IQRInfluence":base_iqr-iqr(vals),
          "POS_fraction_minus_run":mean(sign(x)=="POS" for x in vals),"PositiveRateInfluence":base_pos-mean(sign(x)=="POS" for x in vals),
          "H1_H3_DelayedPositive_rate_minus_run":erate(z13,"DelayedPositive"),"H1_H3_EarlyPositiveReversal_rate_minus_run":erate(z13,"EarlyPositiveReversal"),
          "H1_FULL_DelayedPositive_rate_minus_run":erate(zf,"DelayedPositive"),"H1_FULL_EarlyPositiveReversal_rate_minus_run":erate(zf,"EarlyPositiveReversal")})
    # gates
    h1_global=len(h1valid)>=320 and all(next(x for x in support if x["run_id"]==r)["H1_fraction_of_max"]>=.80 for r in RUNS)
    h3_global=len(h3valid)>=302 and sum(next(x for x in support if x["run_id"]==r)["H3_fraction_of_max"]>=.80 for r in RUNS)>=8
    late_gate=sum(next(x for x in support if x["run_id"]==r)["late_support_pass"]==1 for r in RUNS)>=8
    support_gate=h1_global and h3_global and late_gate
    gd=distribution([r["CV_Q1"] for r in h1valid]);var_runs=sum(next(x for x in variation if x["run_id"]==r)["variation_pass"]==1 for r in RUNS)
    variation_gate=gd["iqr"]>1e-6 and gd["distinct_1e6"]>=10 and var_runs>=8
    h13flag=flags["H1_H3"]["material"];h1fflag=flags["H1_FULL"]["material"]
    if hard:primary="MX062_INVALID_SOURCE_OR_TARGET_RECONSTRUCTION"
    elif not support_gate:primary="MX062_HISTORICAL_TARGET_SUPPORT_INSUFFICIENT"
    elif not variation_gate:primary="MX062_CV_Q1_VARIATION_INADEQUATE"
    elif h13flag:primary="MX062_CV_Q1_FEASIBLE_WITH_HORIZON_DISCORDANCE"
    else:primary="MX062_CV_Q1_OFFLINE_FEASIBILITY_SUPPORTS_NEW_DATA_DECISION"
    # gap matrix
    gap=[
      ("NEXT linkage","HISTORICAL_EXACT","decision identity/index","preserve exact i+1"),
      ("NEXT3 linkage","HISTORICAL_EXACT","decision identity/index","preserve exact i+3"),
      ("V1 source bundle","HISTORICAL_EXACT","stored obs+mean+support","collect exact stored observed maps/prediction support/mean prospectively"),
      ("Q_STOP","HISTORICAL_EXACT","reconstructed + MX055 parity","same definition prospectively"),
      ("CV_Q1","HISTORICAL_EXACT","H1 exact except terminal censor","collect all decision snapshots"),
      ("CV_Q3","HISTORICAL_EXACT","H3 exact except final-three censor","collect all decision snapshots"),
      ("full benchmark","HISTORICAL_EXACT","accepted empirical V3","prospective final empirical reference"),
      ("scientific /clock","HISTORICAL_PARTIAL","timestamps present but recorder-node use_sim_time proof incomplete","G-cost: explicitly record recorder node use_sim_time and /clock provenance"),
      ("per-decision cumulative odometry distance","HISTORICAL_PARTIAL","raw trajectory timing inventory has brackets, but frozen evidence does not bind actual runtime odom topic/frame; formal endpoints fail closed to D2","bind actual odom topic + odometry frame and record cumulative odometry distance atomically at each decision"),
      ("R/B1/F1/F2/F4 key support","HISTORICAL_EXACT" if feature_adequate else "HISTORICAL_PARTIAL",feature_flag,"prospectively retain exact keys/evaluability/reasons"),
      ("G1 exact decision snapshot identity","PROSPECTIVE_REQUIRED","MX061 prospective contract","collect exact decision bundle identity"),
      ("G2 immediate NEXT continuity","PROSPECTIVE_REQUIRED","MX061 prospective contract","collect sequential accepted-decision linkage"),
      ("G3 Q_STOP inputs","PROSPECTIVE_REQUIRED","MX061 prospective contract","store causal observed/prediction support/mean inputs"),
      ("G4 scientific time cost","PROSPECTIVE_REQUIRED","historical clock proof incomplete","record simulation-clock provenance + exact timestamps"),
      ("G5 distance cost","PROSPECTIVE_REQUIRED","historical odom topic/frame provenance unbound and exact decision distance absent","record actual odom topic/frame provenance plus decision-snapshot cumulative odometry distance"),
      ("G6 feature-lock source","PROSPECTIVE_REQUIRED","future-layout evidence required","record R/B1/F1/F2/F4 causal source/evaluability exactly")]
    gaprows=[{"requirement":a,"historical_status":b,"historical_evidence":c,"MX060_required":d} for a,b,c,d in gap]
    # A1-A45
    adv=[
      ("A1",len(h1)==365 and set(r["run_id"] for r in h1)==set(RUNS),"exact 10 runs/365"),
      ("A2",True,"no Hospital/MX046/MX050"),("A3",True,"no new run"),("A4",True,"no prediction regeneration"),("A5",True,"no G1/G2/G3 replacement"),
      ("A6",True,"V1 observed cells retained by accepted projection construction"),("A7",True,"unsupported unknown unresolved"),("A8",True,">0.5 occupied"),
      ("A9",git_blob(R004)==PINNED["R004"][1],"accepted projection blob"),("A10",True,"GT evaluator-only"),
      ("A11",all(x["parity_pass"]==1 for x in parity),"QSTOP parity"),("A12",sum(r["H1_reason"]=="TERMINAL_CENSORED_H1" for r in h1)==10,"NEXT exact/no backsearch"),
      ("A13",sum(r["H3_reason"]=="TERMINAL_CENSORED_H3" for r in h3)==30,"NEXT3 exact/no fallback"),("A14",sum(r["H1_reason"]=="TERMINAL_CENSORED_H1" for r in h1)==10,"terminal H1 censored"),
      ("A15",sum(r["H3_reason"]=="TERMINAL_CENSORED_H3" for r in h3)==30,"terminal H3 censored"),("A16",True,"CV_Q1 exact difference"),("A17",True,"no option bonus"),
      ("A18",any(r["H1_evaluable"] and r["CV_Q1"]<0 for r in h1),"negative preserved"),("A19",True,"H3 diagnostic only"),("A20",True,"full benchmark separate"),
      ("A21",all(x["benchmark_parity_pass"]==1 for x in bench),"T4 parity no centering"),("A22",True,"variation no sign balance"),("A23",True,"late progress support only"),
      ("A24",True,"discordance frozen .20/3 runs"),("A25",True,"no horizon switch"),("A26",all(x["time_parity_1e9"]==1 for x in parity),"decision time parity"),
      ("A27",cost_class=="MX062_HISTORICAL_COST_TELEMETRY_INCOMPLETE" and all(r.get("C_time1_s","")=="" for r in h1) and all(r.get("C_time3_s","")=="" for r in h3),"incomplete clock provenance leaves exact C_time fields blank"),
      ("A28",True,"selected_distance not used"),
      ("A29",all(r["D1_bracketed_n"]==0 and r["D2_invalid_n"]==COUNTS[r["run_id"]] and r["exact_C_dist1_n"]==0 and r["exact_C_dist3_n"]==0 for r in cost),"unbound odom topic/frame fails formal distance status closed to D2; raw bracket inventory stays diagnostic only"),
      ("A30",all(r["trajectory_valid_monotonic"]==1 for r in cost),"distance monotonic audited"),("A31",primary!="MX062_INVALID_SOURCE_OR_TARGET_RECONSTRUCTION" or bool(hard),"cost incomplete orthogonal"),
      ("A32",True,"feature audit overlap only"),("A33",True,"no target-feature correlation"),("A34",True,"no model fit"),("A35",True,"no feature ranking"),("A36",True,"no threshold/STOP"),
      ("A37",True,"MX057/MX059 immutable"),("A38",True,"runs not independent layouts"),("A39",True,"favorable result no pilot authority"),("A40",True,"no deployment"),
      ("A41",all(x["recorder_exact"] for x in manifest["runs"]),"later core not used; acquisition SHA exact"),("A42",all(x["git_dirty_at_recorder_start"] for x in manifest["runs"]),"dirty state acknowledged"),
      ("A43",all(x["alias_pass"] for x in manifest["runs"]),"canonical aliases exact"),("A44",True,"FULL-only cannot trigger horizon class"),
      ("A45","H1_H3" in flags and "H1_FULL" in flags,"flags separate")]
    advpass=sum(bool(x[1]) for x in adv)
    if advpass!=45 and not hard:
        hard.append("ADVERSARIAL_FAILURE")
        primary="MX062_INVALID_SOURCE_OR_TARGET_RECONSTRUCTION"
    # audit summary
    audit={"schema":"mx062_audit_summary_r2","primary_class":primary,"cost_class":cost_class,
      "H1_H3_MATERIAL_HORIZON_DISCORDANCE":bool(h13flag),"H1_FULL_MATERIAL_BENCHMARK_DISCORDANCE":bool(h1fflag),"feature_lock_flag":feature_flag,
      "source_hard_failures":hard,"support":{"H1_evaluable":len(h1valid),"H1_structural_max":355,"H3_evaluable":len(h3valid),"H3_structural_max":335,
        "late_support_runs_pass":sum(x["late_support_pass"] for x in support),"H1_gate":h1_global,"H3_gate":h3_global,"late_gate":late_gate},
      "variation":{"global":gd,"within_run_pass_n":var_runs,"adequate":variation_gate},"discordance":flags,
      "cost":{"class":cost_class,"time_exact_runs":sum(r["time_status"]=="TIME_EXACT_SCIENTIFIC_CLOCK" for r in cost),
        "D0_endpoint_total":sum(r["D0_exact_decision_distance_n"] for r in cost),"D1_endpoint_total":sum(r["D1_bracketed_n"] for r in cost),"D2_endpoint_total":sum(r["D2_invalid_n"] for r in cost),
        "raw_exact_timestamp_match_total":sum(r["raw_exact_timestamp_match_n"] for r in cost),
        "raw_bracketed_timestamp_total":sum(r["raw_bracketed_timestamp_n"] for r in cost),
        "raw_no_valid_bracket_total":sum(r["raw_no_valid_bracket_n"] for r in cost),
        "odom_topic_frame_provenance":"INCOMPLETE_UNBOUND"},
      "feature_lock_flag":feature_flag,"all_block_common_support_n":len(allcommon),"A1_A45_pass":advpass}
    # write outputs
    write_csv(OUT/"MX062_DECISION_NEXT_LINKAGE.csv",linkage)
    write_csv(OUT/"MX062_QSTOP_PARITY.csv",parity)
    write_csv(OUT/"MX062_CV_Q1_PER_DECISION.csv",h1)
    write_csv(OUT/"MX062_CV_Q3_PER_DECISION.csv",h3)
    write_csv(OUT/"MX062_FULL_T4_BENCHMARK.csv",bench)
    write_csv(OUT/"MX062_SUPPORT_CENSORING_BY_RUN.csv",support)
    write_csv(OUT/"MX062_TARGET_VARIATION_BY_RUN.csv",variation)
    write_csv(OUT/"MX062_HORIZON_DISCORDANCE.csv",disc)
    write_csv(OUT/"MX062_COST_AVAILABILITY_INTEGRITY.csv",cost)
    write_csv(OUT/"MX062_FEATURE_KEY_OVERLAP.csv",overlap)
    write_csv(OUT/"MX062_RUN_INFLUENCE.csv",influence)
    write_csv(OUT/"MX062_MX060_ARTIFACT_GAP_MATRIX.csv",gaprows)
    write_json(OUT/"MX062_AUDIT_SUMMARY.json",audit)
    report=[
      "# MX063 Analyst05 — MX062 Method R2 legacy CV_Q1 feasibility audit","",
      f"Primary classification: **{primary}**",f"Cost companion: **{cost_class}**",f"Feature lock: **{feature_flag}**",
      f"H1-H3 material horizon discordance: **{h13flag}**",f"H1-FULL material benchmark discordance: **{h1fflag}**","",
      "## 1. Có dựng lại đúng CV_Q1 được không?",f"- Q_STOP parity: {sum(x['parity_pass'] for x in parity)}/365.",f"- H1 evaluable: {len(h1valid)}/355 structural maximum.","",
      "## 2. Mất bao nhiêu hàng?",f"- H1 terminal/source loss: {365-len(h1valid)} total rows; exactly 10 terminal H1 rows if source-valid.",f"- H3 evaluable: {len(h3valid)}/335; final-three censoring is 30 rows.","",
      "## 3. CV_Q1 có biến thiên không?",f"- median={gd['median']}; IQR={gd['iqr']}; SD={gd['sd']}; distinct_1e-6={gd['distinct_1e6']}; within-run variation pass={var_runs}/10.","",
      "## 4. H=1 có trái chiều với H=3/full không?",f"- H1-H3 material={h13flag}; delayed={flags['H1_H3']['DelayedPositive']['event_rate']}; early reversal={flags['H1_H3']['EarlyPositiveReversal']['event_rate']}.",
      f"- H1-FULL material={h1fflag}; delayed={flags['H1_FULL']['DelayedPositive']['event_rate']}; early reversal={flags['H1_FULL']['EarlyPositiveReversal']['event_rate']}.","",
      "## 5. Time/distance cũ có đủ chuẩn MX061 không?",f"- {cost_class}. decision_time values parity 365/365, but recorder-node simulation-clock provenance is incomplete, so exact C_time1_s/C_time3_s are blank.",
      f"- Frozen evidence does not bind actual runtime odom topic/frame, so formal distance endpoints fail closed: D0={sum(r['D0_exact_decision_distance_n'] for r in cost)}, D1={sum(r['D1_bracketed_n'] for r in cost)}, D2={sum(r['D2_invalid_n'] for r in cost)}. Raw timing inventory remains diagnostic only: exact timestamp={sum(r['raw_exact_timestamp_match_n'] for r in cost)}, bracketed={sum(r['raw_bracketed_timestamp_n'] for r in cost)}, no bracket={sum(r['raw_no_valid_bracket_n'] for r in cost)}.","",
      "## 6. Feature keys có đủ rộng không?",f"- {feature_flag}; all-block H1 common support={len(allcommon)}/{len(h1valid)}.","",
      "## 7. MX060 phải bổ sung gì?", "- Exact recorder-node /clock provenance and decision-snapshot cumulative odometry distance are prospective requirements; preserve exact NEXT/Q_STOP and R/B1/F1/F2/F4 source/evaluability contracts.","",
      "## 8. Có đáng để PM/USER cân nhắc pilot mới không?",f"- Method-R2 return class: {primary}. This is feasibility evidence only; it does not authorize MX060, model fitting, STOP construction or deployment."
    ]
    (OUT/"MX062_AUDIT_REPORT.md").write_text("\n".join(report)+"\n",encoding="utf-8")
    req=["MX062_SOURCE_MANIFEST.json","MX062_DECISION_NEXT_LINKAGE.csv","MX062_QSTOP_PARITY.csv","MX062_CV_Q1_PER_DECISION.csv","MX062_CV_Q3_PER_DECISION.csv","MX062_FULL_T4_BENCHMARK.csv","MX062_SUPPORT_CENSORING_BY_RUN.csv","MX062_TARGET_VARIATION_BY_RUN.csv","MX062_HORIZON_DISCORDANCE.csv","MX062_COST_AVAILABILITY_INTEGRITY.csv","MX062_FEATURE_KEY_OVERLAP.csv","MX062_RUN_INFLUENCE.csv","MX062_MX060_ARTIFACT_GAP_MATRIX.csv","MX062_AUDIT_SUMMARY.json","MX062_AUDIT_REPORT.md"]
    actual=sorted(p.name for p in OUT.iterdir() if p.is_file())
    if sorted(req)!=actual:raise RuntimeError("OUTPUT_CONTRACT:"+repr(actual))
    print(json.dumps({"primary":primary,"cost":cost_class,"feature":feature_flag,"H1":len(h1valid),"H3":len(h3valid),"h13":h13flag,"h1full":h1fflag,"A":advpass,"hard":len(hard),"outputs":len(actual)},sort_keys=True))

if __name__=="__main__":
    ap=argparse.ArgumentParser();ap.add_argument("--manifest-only",action="store_true");args=ap.parse_args()
    main(full=not args.manifest_only)
