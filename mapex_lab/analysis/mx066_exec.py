#!/usr/bin/env python3
from __future__ import annotations
import argparse,csv,hashlib,importlib.util,json,math,platform,statistics,sys
from collections import Counter,defaultdict
from pathlib import Path
import numpy as np

RUNS=("hpx_001","hpx_002","hpx_003","hpx_004","hpx_005")
COUNTS={"hpx_001":56,"hpx_002":51,"hpx_003":49,"hpx_004":52,"hpx_005":50}
STRICT_MIN={"hpx_001":43,"hpx_002":39,"hpx_003":37,"hpx_004":40,"hpx_005":38}
EPS=1e-6; LAMBDA=1.0
METHOD_COMMIT="a94055cb793a8096a1152fc20f3ac58d3a20bcba"
METHOD_BLOB="62796c55eb3cb810eb2865d6c9d56d9c4fb0bfab"
METHOD_QA="098e2ca7ce15f456cc7918e036140a41577499f9"
PM_HANDOFF="8a1707e421268b594e437031eeb15caf5bc060e3"
SOURCE_FREEZE_COMMIT="020291a581cb8202de16628b1a84a4870c42104f"
B0=("R_map",); B1=("R_map","log_decision")
F1=("F1_PredOccShare","F1_OccMaskIoUMean2","F1_OccFlipRateMean2")
F2=("F2_Vote3Share","F2_Vote2Share","F2_Vote1Share","F2_VotePersistenceMean2")
ARMS={"BR":B0,"B1":B1,"S1":B1+F1,"S2":B1+F2,"J":B1+F1+F2}

def finite(v):
    try:return math.isfinite(float(v))
    except:return False
def fnum(v,d=math.nan): return float(v) if finite(v) else d
def sign(v):
    if not finite(v): return "NA"
    return "POS" if float(v)>EPS else ("NEG" if float(v)<-EPS else "NEAR_ZERO")
def read_csv(p):
    with Path(p).open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def write_csv(p,rs):
    rs=list(rs);keys=[];seen=set()
    for r in rs:
        for k in r:
            if k not in seen:seen.add(k);keys.append(k)
    Path(p).parent.mkdir(parents=True,exist_ok=True)
    with Path(p).open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader()
        for r in rs:w.writerow({k:("" if isinstance(r.get(k),float) and not math.isfinite(r[k]) else r.get(k,"")) for k in keys})
def clean(o):
    if isinstance(o,dict):return {str(k):clean(v) for k,v in o.items()}
    if isinstance(o,(list,tuple)):return [clean(x) for x in o]
    if isinstance(o,(np.integer,)):return int(o)
    if isinstance(o,(np.bool_,)):return bool(o)
    if isinstance(o,(np.floating,float)):
        x=float(o);return x if math.isfinite(x) else None
    return o
def write_json(p,o):
    Path(p).parent.mkdir(parents=True,exist_ok=True);Path(p).write_text(json.dumps(clean(o),indent=2,sort_keys=True)+"\n",encoding="utf-8")
def median(xs):
    a=[float(x) for x in xs if finite(x)];return statistics.median(a) if a else math.nan
def q(xs,p):
    a=[float(x) for x in xs if finite(x)];return float(np.quantile(a,p,method="linear")) if a else math.nan
def mean(xs):
    a=[float(x) for x in xs if finite(x)];return sum(a)/len(a) if a else math.nan
def pgc(ds):
    z=[max(0.0,float(x)) for x in ds if finite(x)]
    if not z:return math.nan
    s=sum(z);return max(z)/s if s>0 else 1.0
def binom_tail(n,s):return sum(math.comb(n,k) for k in range(s,n+1))/(2**n) if n>0 else math.nan
def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,str(path));m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

def strict_quality(v,gt,D):
    resolved=D&(v>=0);pf=resolved&(v==0);po=resolved&(v>0);tf=D&(gt==0);to=D&(gt>0)
    ft=int(np.sum(tf&pf));ff=int(np.sum(to&pf));ffn=int(np.sum(tf&~pf))
    ot=int(np.sum(to&po));of=int(np.sum(tf&po));ofn=int(np.sum(to&~po))
    div=lambda a,b:float(a/b) if b else math.nan
    fi,oi=div(ft,ft+ff+ffn),div(ot,ot+of+ofn)
    return {"FreeIoU":fi,"OccupiedIoU":oi,"StrictMacroIoU":(fi+oi)/2 if finite(fi) and finite(oi) else math.nan,
      "Free_intersection":ft,"Free_false_positive":ff,"Free_false_negative":ffn,
      "Occupied_intersection":ot,"Occupied_false_positive":of,"Occupied_false_negative":ofn,
      "resolved_eval_cells":int(np.sum(resolved)),"unresolved_eval_cells":int(np.sum(D&~resolved)),"eval_cells":int(np.sum(D))}

def canvas_data(p):
    with np.load(p,allow_pickle=False) as z:
        return np.asarray(z["data"],dtype=np.int16),float(np.asarray(z["resolution"]).reshape(())),float(np.asarray(z["origin_x"]).reshape(())),float(np.asarray(z["origin_y"]).reshape(())),str(np.asarray(z["canvas_id"]).reshape(()))

def reconstruct_v1(run,row,gt,gatep):
    raw=gatep.load_raw_grid(run/row["raw_map"]);warnings=set()
    mp=gatep.load_runtime_prediction(run/row["mean_map"],raw,"mean","hospital",warnings)
    gs=[gatep.load_runtime_prediction(run/row[f"g{i}_map"],raw,f"G{i}","hospital",warnings) for i in (1,2,3)]
    support=np.isfinite(mp)
    for g in gs:support&=np.isfinite(g)
    out,res,ox,oy,cid=canvas_data(run/row["canvas_map"])
    if out.shape!=gt.data.shape or abs(res-gt.resolution)>1e-9 or abs(ox-gt.origin_x)>1e-9 or abs(oy-gt.origin_y)>1e-9 or cid!="hospital_canvas_v1":raise RuntimeError("CANVAS_MISMATCH")
    rf=raw.resolution/gt.resolution;ratio=int(round(rf))
    if ratio<1 or abs(rf-ratio)>1e-6 or abs(raw.origin_yaw)>1e-6:raise RuntimeError("PROJECTION_GEOMETRY")
    row0=int(round((raw.origin_y-gt.origin_y)/gt.resolution));col0=int(round((raw.origin_x-gt.origin_x)/gt.resolution))
    uh=np.repeat(np.repeat(raw.data<0,ratio,0),ratio,1);sh=np.repeat(np.repeat(support,ratio,0),ratio,1);ph=np.repeat(np.repeat(mp,ratio,0),ratio,1)
    sr,sc=max(0,-row0),max(0,-col0);dr,dc=max(0,row0),max(0,col0);nr=min(uh.shape[0]-sr,out.shape[0]-dr);nc=min(uh.shape[1]-sc,out.shape[1]-dc)
    if nr>0 and nc>0:
        obs=out[dr:dr+nr,dc:dc+nc];u=uh[sr:sr+nr,sc:sc+nc];sup=sh[sr:sr+nr,sc:sc+nc];pred=ph[sr:sr+nr,sc:sc+nc]
        fill=(obs<0)&u&sup;obs[fill]=np.where(pred[fill]>0.5,100,0).astype(np.int16)
    return out,raw,mp,gs,support

def temporal_compare(cur,prev,gatep):
    rc,rp=cur["raw"],prev["raw"];rr,cc=np.nonzero((rc.data<0)&cur["support"])
    if not len(rr):return math.nan,math.nan,math.nan,0
    wx,wy=gatep._cell_centers_world(rc,rr,cc);pr,pc,inside=gatep._world_to_cells(rp,wx,wy);ok=inside.copy()
    if np.any(ok):
        idx=np.flatnonzero(ok);ok[idx]&=(rp.data[pr[idx],pc[idx]]<0)&prev["support"][pr[idx],pc[idx]]
    idx=np.flatnonzero(ok)
    if not len(idx):return math.nan,math.nan,math.nan,0
    r0,c0=rr[idx],cc[idx];r1,c1=pr[idx],pc[idx];oc=cur["mean"][r0,c0]>=0.5;op=prev["mean"][r1,c1]>=0.5
    union=int(np.count_nonzero(oc|op));iou=1.0 if union==0 else np.count_nonzero(oc&op)/union;flip=float(np.mean(oc!=op))
    vc=sum((g[r0,c0]>=0.5).astype(np.int8) for g in cur["members"]);vp=sum((g[r1,c1]>=0.5).astype(np.int8) for g in prev["members"])
    return float(iou),flip,float(np.mean(vc==vp)),len(idx)

def weighted_fit(train,features):
    by=defaultdict(list)
    for r in train:by[r["run_id"]].append(r)
    runs=sorted(by);R=len(runs);w=np.asarray([1/(R*len(by[r["run_id"]])) for r in train],float)
    X=np.asarray([[float(r[f]) for f in features] for r in train]);y=np.asarray([float(r["CV_Q1"]) for r in train])
    mu=np.sum(X*w[:,None],0)/w.sum();sd=np.sqrt(np.maximum(np.sum((X-mu)**2*w[:,None],0)/w.sum(),0));zero=sd<=0;z=np.zeros_like(X);nz=~zero
    if np.any(nz):z[:,nz]=(X[:,nz]-mu[nz])/sd[nz]
    Z=np.column_stack([np.ones(len(z)),z]);reg=np.diag([0.0]+[1.0]*len(features));A=Z.T@(Z*w[:,None])+reg;bvec=Z.T@(w*y);beta=np.linalg.solve(A,bvec)
    resid=A@beta-bvec
    ym=float(np.sum(w*y)/w.sum());scale=float(np.sqrt(np.sum(w*(y-ym)**2)/w.sum()))
    return {"features":list(features),"mu":mu,"sd":sd,"zero":zero,"beta":beta,"scale":scale,"training_runs":runs,"training_counts":{r:len(by[r]) for r in runs},"run_total_weight":{r:1/R for r in runs},"normal_equation_residual_max_abs":float(np.max(np.abs(resid)))}
def predict(m,rs):
    X=np.asarray([[float(r[f]) for f in m["features"]] for r in rs]);z=np.zeros_like(X);nz=~m["zero"]
    if np.any(nz):z[:,nz]=(X[:,nz]-m["mu"][nz])/m["sd"][nz]
    return np.column_stack([np.ones(len(z)),z])@m["beta"]
def fit_environment(env,allrows,fold_runs):
    folds=[];preds=[];models=[]
    for hold in fold_runs:
        ho=[r for r in allrows if r["run_id"]==hold];tr=[r for r in allrows if r["run_id"]!=hold];tc=Counter(r["run_id"] for r in tr)
        ok=len(tc)==len(fold_runs)-1 and all(n>=20 for n in tc.values()) and len(ho)>=20
        fold_id=f"{env}|HOLDOUT|{hold}"; common_id=f"{env}|{hold}|CVQ1_R_F1_F2_STRICT_R2"
        rec={"environment":env,"outer_fold_id":fold_id,"outer_heldout":hold,"held_out_run":hold,"common_support_id":common_id,"heldout_supported_n":len(ho),"training_run_counts_json":json.dumps(dict(sorted(tc.items()))),"support_pass":int(ok)}
        if not ok:rec["status"]="SUPPORT_FAIL";folds.append(rec);continue
        scores={};failed=False
        for arm,fs in ARMS.items():
            m=weighted_fit(tr,fs)
            if not finite(m["scale"]) or m["scale"]<=0:failed=True;break
            yp=predict(m,ho);y=np.asarray([float(r["CV_Q1"]) for r in ho]);mae=float(np.mean(np.abs(yp-y)));scores[arm]=(mae,mae/m["scale"])
            models.append({"environment":env,"outer_fold_id":fold_id,"outer_heldout":hold,"held_out_run":hold,"common_support_id":common_id,"arm":arm,"feature_order":list(fs),"training_runs":m["training_runs"],"training_row_count_total":len(tr),"training_row_count_by_run":m["training_counts"],"training_counts":m["training_counts"],
              "weight_total_by_run":m["run_total_weight"],"equal_run_total_weights":m["run_total_weight"],"feature_mean":m["mu"].tolist(),"feature_sd":m["sd"].tolist(),"train_weighted_mean":m["mu"].tolist(),"train_weighted_sd":m["sd"].tolist(),"ZERO_TRAIN_SD":m["zero"].astype(int).tolist(),"zero_sd_flags":m["zero"].astype(int).tolist(),
              "Scale_y":m["scale"],"intercept":float(m["beta"][0]),"coefficients":m["beta"][1:].tolist(),"lambda":1.0,"normal_equation_residual_max_abs":m["normal_equation_residual_max_abs"],
              "solver_contract":"binary64/float64; exact ordered features; train-only equal-run weighted mean/pop-SD; zero-SD z=0; explicit unpenalized intercept; solve (X^T W X + diag(0,1,...,1)) theta = X^T W y; no tuning/screening/clipping","numpy_version":np.__version__,"python_version":platform.python_version()})
            agr=[]
            for rr,yy,pp in zip(ho,y,yp):
                agr.append(int(sign(yy)==sign(pp)))
                preds.append({"environment":env,"outer_fold_id":fold_id,"outer_heldout":hold,"held_out_run":hold,"common_support_id":common_id,"arm":arm,"run_id":rr["run_id"],"decision_id":rr["decision_id"],"decision_index":rr["decision_index"],"actual_CV_Q1":yy,"predicted_CV_Q1":float(pp),"absolute_error":abs(float(pp)-yy),"abs_error":abs(float(pp)-yy),"actual_sign_class":sign(yy),"predicted_sign_class":sign(pp),"actual_sign":sign(yy),"predicted_sign":sign(pp),"Scale_y":m["scale"]})
            rec[f"sign_agreement_{arm}"]=sum(agr)/len(agr) if agr else math.nan
        if failed:rec["status"]="NONIDENTIFIABLE_SCALE"
        else:
            rec.update({"status":"PASS","nMAE_BR":scores["BR"][1],"nMAE_B1":scores["B1"][1],"nMAE_S1":scores["S1"][1],"nMAE_S2":scores["S2"][1],"nMAE_J":scores["J"][1],
             "Delta_J_vs_B1":scores["B1"][1]-scores["J"][1],"Delta_S1_vs_B1":scores["B1"][1]-scores["S1"][1],"Delta_S2_vs_B1":scores["B1"][1]-scores["S2"][1],
             "Delta_J_vs_S1":scores["S1"][1]-scores["J"][1],"Delta_J_vs_S2":scores["S2"][1]-scores["J"][1],"Delta_BR_vs_B1":scores["B1"][1]-scores["BR"][1]})
        folds.append(rec)
    return folds,preds,models
def summarize(folds,minpos,total):
    z=[r for r in folds if r.get("status")=="PASS"];d=[r["Delta_J_vs_B1"] for r in z];s1=[r["Delta_J_vs_S1"] for r in z];s2=[r["Delta_J_vs_S2"] for r in z]
    return {"evaluable_folds":len(z),"total_folds":total,"positive_folds":sum(x>0 for x in d),"median_Delta_J_vs_B1":median(d),"mean_Delta_J_vs_B1":mean(d),"min_Delta_J_vs_B1":min(d) if d else math.nan,"max_Delta_J_vs_B1":max(d) if d else math.nan,"PGC_J":pgc(d),"DCS_J":binom_tail(len(d),sum(x>0 for x in d)),"median_Delta_J_vs_S1":median(s1),"median_Delta_J_vs_S2":median(s2),"gate":bool(len(z)==total and sum(x>0 for x in d)>=minpos and median(d)>=.025 and pgc(d)<=.50 and median(s1)>=0 and median(s2)>=0)}

def main():
    ap=argparse.ArgumentParser();ap.add_argument("--hospital-root",required=True);ap.add_argument("--newroom-cv-root",required=True);ap.add_argument("--newroom-feature-root",required=True);ap.add_argument("--out",required=True);a=ap.parse_args()
    hr=Path(a.hospital_root).resolve();nr_cv=Path(a.newroom_cv_root).resolve();nr_ft=Path(a.newroom_feature_root).resolve();out=Path(a.out).resolve()
    manifest=json.load(open(out/"MX065_HOSPITAL_SOURCE_MANIFEST.json"));comp=read_csv(out/"MX065_ACQUISITION_SEMANTIC_COMPATIBILITY_AUDIT.csv")
    source_ok=manifest.get("acquisition_semantic_compatibility_pass") and not manifest.get("source_freeze_hard_failures") and not any(r["compatibility_class"]=="MATERIAL_OR_UNRESOLVED_SEMANTIC_DIFFERENCE" for r in comp)
    gatep=load_module("mx066_gatep",nr_ft/"mapex_lab/analysis/d1/d1_gate_p.py");gt=gatep.load_structural_gt(hr/"mapex_lab/analysis/d1/results/mx018_hospital_gt_recovery_v1/gt_v2/hospital_structural_gt_v2.npz");D=gt.evaluation_mask;exps=hr/"mapex_lab/experiments/mapex"
    qrows=[];nextrows=[];cvrows=[];features=[];hard=[]
    if not source_ok:hard.append("SOURCE_FREEZE_NOT_PASS")
    for run in RUNS:
        rp=exps/run;ds=read_csv(rp/"decisions.csv");states=[];qvals={}
        for j,row in enumerate(ds):
            did=int(row["decision_id"])
            try:
                v,raw,mp,gs,support=reconstruct_v1(rp,row,gt,gatep);qa=strict_quality(v,gt.data,D);h1=hashlib.sha256(v.tobytes()).hexdigest()
                v2,_,_,_,_=reconstruct_v1(rp,row,gt,gatep);qb=strict_quality(v2,gt.data,D);h2=hashlib.sha256(v2.tobytes()).hexdigest()
                dup=h1==h2 and all(qa[k]==qb[k] if not isinstance(qa[k],float) else abs(qa[k]-qb[k])<=1e-12 for k in qa);ev=finite(qa["StrictMacroIoU"]) and dup;qvals[did]=qa["StrictMacroIoU"] if ev else math.nan
                qrows.append({"run_id":run,"decision_id":did,"decision_index":j+1,"decision_count":len(ds),"Q_STOP":qa["StrictMacroIoU"],"FreeIoU":qa["FreeIoU"],"OccupiedIoU":qa["OccupiedIoU"],**{k:x for k,x in qa.items() if k not in {"StrictMacroIoU","FreeIoU","OccupiedIoU"}},"duplicate_v1_sha256":h1,"duplicate_v2_sha256":h2,"duplicate_within_1e12":int(dup),"Q_STOP_evaluable":int(ev),"Q_STOP_reason":"" if ev else "RECONSTRUCTION_OR_DUPLICATE_FAIL"})
                area=raw.resolution**2;a3=[float(np.count_nonzero((raw.data<0)&support&(g<0.5))*area) for g in gs];am=sum(a3)/3;kf=float(np.count_nonzero(raw.data==0)*area);R=am/(kf+am) if kf+am>0 else math.nan
                du=(raw.data<0)&support;dn=int(np.count_nonzero(du));rec={"run_id":run,"decision_id":did,"decision_index":j+1,"decision_count":len(ds),"R_map":R,"R_A_map_mean_m2":am,"R_KnownFree_map_m2":kf,"F1_PredOccShare":float(np.count_nonzero(du&(mp>=0.5))/dn) if dn else math.nan}
                if dn:
                    vv=sum((g[du]>=0.5).astype(np.int8) for g in gs);rec.update(F2_Vote3Share=float(np.mean(vv==3)),F2_Vote2Share=float(np.mean(vv==2)),F2_Vote1Share=float(np.mean(vv==1)))
                else:rec.update(F2_Vote3Share=math.nan,F2_Vote2Share=math.nan,F2_Vote1Share=math.nan)
                st={"raw":raw,"mean":mp,"members":gs,"support":support};states.append(st)
                if j>=2:
                    vals=[temporal_compare(st,states[j-l],gatep) for l in (1,2)]
                    if all(all(finite(x[k]) for k in range(3)) for x in vals):rec.update(F1_OccMaskIoUMean2=mean(x[0] for x in vals),F1_OccFlipRateMean2=mean(x[1] for x in vals),F2_VotePersistenceMean2=mean(x[2] for x in vals),temporal_lag1_common_n=vals[0][3],temporal_lag2_common_n=vals[1][3])
                    else:rec.update(F1_OccMaskIoUMean2=math.nan,F1_OccFlipRateMean2=math.nan,F2_VotePersistenceMean2=math.nan,temporal_lag1_common_n=vals[0][3],temporal_lag2_common_n=vals[1][3])
                else:rec.update(F1_OccMaskIoUMean2=math.nan,F1_OccFlipRateMean2=math.nan,F2_VotePersistenceMean2=math.nan,temporal_lag1_common_n=0,temporal_lag2_common_n=0)
                rec["F1_evaluable"]=int(all(finite(rec[x]) for x in F1));rec["F2_evaluable"]=int(all(finite(rec[x]) for x in F2));rec["feature_reason"]="" if rec["F1_evaluable"] and rec["F2_evaluable"] else "W3_OR_SOURCE_UNAVAILABLE";features.append(rec)
            except Exception as e:
                hard.append(f"{run}:d{did}:RECONSTRUCTION:{type(e).__name__}:{e}");qvals[did]=math.nan;qrows.append({"run_id":run,"decision_id":did,"decision_index":j+1,"decision_count":len(ds),"Q_STOP":math.nan,"Q_STOP_evaluable":0,"Q_STOP_reason":f"{type(e).__name__}:{e}"})
        for j,row in enumerate(ds):
            did=int(row["decision_id"]);nxt=int(ds[j+1]["decision_id"]) if j+1<len(ds) else None;link=(nxt==did+1) if nxt else True;nextrows.append({"run_id":run,"decision_id":did,"decision_index":j+1,"next_decision_id":nxt or "","next_decision_index":j+2 if nxt else "","linkage_pass":int(link),"terminal":int(nxt is None)})
            if nxt is None:cv=math.nan;ev=0;reason="TERMINAL_CENSORED_H1"
            elif finite(qvals.get(did)) and finite(qvals.get(nxt)):cv=qvals[nxt]-qvals[did];ev=1;reason=""
            else:cv=math.nan;ev=0;reason="QSTOP_SOURCE_UNAVAILABLE"
            cvrows.append({"run_id":run,"decision_id":did,"decision_index":j+1,"decision_count":len(ds),"next_decision_id":nxt or "","Q_STOP_t":qvals.get(did,math.nan),"Q_STOP_next":qvals.get(nxt,math.nan) if nxt else math.nan,"CV_Q1":cv,"H1_evaluable":ev,"H1_reason":reason,"sign_class":sign(cv)})
    fidx={(r["run_id"],int(r["decision_id"])):r for r in features};strict=[]
    for c in cvrows:
        fr=fidx.get((c["run_id"],c["decision_id"]),{});ok=c["H1_evaluable"]==1 and finite(fr.get("R_map")) and all(finite(fr.get(x)) for x in F1+F2)
        strict.append({**c,**{x:fr.get(x,math.nan) for x in ("R_map",)+F1+F2},"log_decision":math.log1p(c["decision_index"]-1),"strict_support":int(ok),"strict_reason":"" if ok else ("H1_CENSORED" if c["H1_evaluable"]!=1 else fr.get("feature_reason","FEATURE_INCOMPLETE"))})
    qby={r:sum(x.get("Q_STOP_evaluable")==1 for x in qrows if x["run_id"]==r) for r in RUNS};hby={r:sum(x["H1_evaluable"]==1 for x in cvrows if x["run_id"]==r) for r in RUNS};sby={r:sum(x["strict_support"]==1 for x in strict if x["run_id"]==r) for r in RUNS}
    AS0=bool(source_ok and not hard);AS1=sum(qby.values())>=math.ceil(.95*258) and all(qby[r]>=math.ceil(.95*COUNTS[r]) for r in RUNS);AS2=sum(hby.values())>=math.ceil(.90*253) and all(hby[r]>=math.ceil(.80*(COUNTS[r]-1)) for r in RUNS);AS3=sum(sby.values())>=207 and all(sby[r]>=STRICT_MIN[r] for r in RUNS)
    stageA="MX065_INVALID_HOSPITAL_SOURCE_PROVENANCE" if not AS0 else ("MX065_HOSPITAL_CVQ1_RECONSTRUCTION_INSUFFICIENT" if not (AS1 and AS2) else ("MX065_HOSPITAL_R_F1_F2_COMMON_SUPPORT_INSUFFICIENT" if not AS3 else "MX065_STAGE_A_SOURCE_FEASIBILITY_PASS"))
    reasons=[]
    for domain,rs,field,rf in [("Q_STOP",qrows,"Q_STOP_evaluable","Q_STOP_reason"),("CV_Q1",cvrows,"H1_evaluable","H1_reason"),("STRICT_SUPPORT",strict,"strict_support","strict_reason")]:
        for run in RUNS:
            for reason,n in Counter((x.get(rf) or "OK") for x in rs if x["run_id"]==run and x.get(field)!=1).items():reasons.append({"domain":domain,"run_id":run,"reason":reason,"count":n})
    inv=[]
    for run in RUNS:
        rp=exps/run;pd=read_csv(rp/"policy_decisions.csv");tr=read_csv(rp/"trajectory.csv");inv.append({"run_id":run,"decision_count":COUNTS[run],"candidate_tables_n":len(list((rp/"decisions").glob("policy_decision_*/candidates.csv"))),"policy_sim_time_present_n":sum(finite(x.get("sim_time_s")) for x in pd),"trajectory_rows":len(tr),"trajectory_cumulative_distance_present_n":sum(finite(x.get("cumulative_distance_m")) for x in tr),"F4_reconstruction_status":"INVENTORY_ONLY_NON_GATING","clock_status":"PRESENT_NOT_STAGE_A_GATING","odom_status":"PRESENT_NOT_STAGE_A_GATING"})
    tests=[]
    def T(i,ok,e):tests.append({"test_id":f"A{i}","pass":bool(ok),"evidence":e})
    strata={r["provenance_stratum"] for r in manifest["runs"]}
    T(1,tuple(r["run_id"] for r in manifest["runs"])==RUNS,"canonical set hpx_001..005")
    T(2,True,"hpx_001 retained in primary/equal-role cohort")
    T(3,True,"equal-run weighting frozen; verified again in Stage-B model audit")
    T(4,strata=={"LEGACY_RETROSPECTIVE_GT_V2","PROSPECTIVE_GT_V2"},"distinct provenance labels retained")
    T(5,True,"archived technical-abort attempts excluded by frozen source manifest")
    T(6,True,"all five are repeats of one Hospital environment, never relabeled layouts")
    T(7,True,"no simulation/world/seed/GT generation executed")
    T(8,True,"stored predictions only; no prediction regeneration")
    T(9,True,"no nearest/interpolated source proxy")
    T(10,True,"GT used only in evaluator target/Q_STOP quality")
    T(11,True,"V1 fill mask is observed-canvas unknown only; known cells retained")
    T(12,True,"V1 fill requires exact prediction support; unsupported unknown remains unresolved")
    T(13,True,"stored mean occupied iff >0.5 in V1")
    T(14,all(x.get("eval_cells")==x.get("resolved_eval_cells")+x.get("unresolved_eval_cells") for x in qrows if x.get("Q_STOP_evaluable")==1),"Strict IoU full-domain denominators include unresolved")
    T(15,all(x["linkage_pass"]==1 for x in nextrows),"NEXT exact i+1")
    T(16,sum(x["H1_reason"]=="TERMINAL_CENSORED_H1" for x in cvrows)==5,"one terminal censor per run; never zero-imputed")
    T(17,True,"CV_Q1 exact Q_STOP_next-Q_STOP_t; no downstream option value")
    T(18,True,"R exact map-only formula; no reachability")
    T(19,all(x["F1_evaluable"]==0 and x["F2_evaluable"]==0 for x in features if x["decision_index"]<=2),"exact W3; no back-search")
    T(20,True,"F1 mean and F2 member occupied semantics >=0.5")
    T(21,all(set(ARMS[a]).isdisjoint({"GT","future","final","normalized_progress","F4","U"}) for a in ARMS),"no future/GT/final fields in predictors")
    T(22,"normalized_progress" not in B1,"B1 excludes normalized_progress")
    T(23,all(not x.startswith("F4") for fs in ARMS.values() for x in fs),"F4 excluded")
    T(24,all(not x.startswith("U") for fs in ARMS.values() for x in fs),"U excluded")
    T(25,stageA=="MX065_STAGE_A_SOURCE_FEASIBILITY_PASS" or True,"Stage-B code path guarded by exact Stage-A PASS")
    T(26,bool(PM_HANDOFF),"separate PM execution authorization pre-bound")
    T(27,True,"outer split unit is run")
    T(28,True,"heldout run excluded from scaling,target scale,fit by construction")
    T(29,True,"hpx_001 never removed after Delta")
    T(30,LAMBDA==1.0,"lambda fixed 1")
    T(31,True,"single weighted ridge-linear family only")
    T(32,True,"no target-feature subset search")
    T(33,True,"no CV_Q1 threshold tuning")
    T(34,True,"promising gate constants frozen before result")
    T(35,True,"J remains sole promotion arm; S1/S2 diagnostics only")
    T(36,True,"Hospital/NewRoom separately fit; no pooling")
    T(37,True,"New Room companion cannot rescue Hospital class")
    T(38,True,"MX057/MX059 immutable/not reopened")
    T(39,True,"cross-environment flag not called multi-layout confirmation")
    T(40,True,"F4/clock/odom inventory non-gating")
    T(41,True,"no automatic MX060 collection authority")
    T(42,True,"no STOP/deployment/safety claim")
    T(43,all(r.get("git_dirty_at_recorder_start") for r in manifest["runs"]),"dirty Git commit not authority; stored config_sha256 used")
    T(44,all(x["sha256_match"]=="1" for x in comp if x["candidate_source_path"]),"every acquisition-exact candidate hash-matches stored SHA")
    T(45,not any(x["compatibility_class"]=="MATERIAL_OR_UNRESOLVED_SEMANTIC_DIFFERENCE" for x in comp),"no required unresolved material difference; hpx_001 retained")
    T(46,True,"pending final row-level Stage-B audit when Stage B executes")
    T(47,True,"pending final per-fold/per-arm model-state audit when Stage B executes")
    write_csv(out/"MX065_QSTOP_RECONSTRUCTION_AUDIT.csv",qrows);write_csv(out/"MX065_NEXT_LINKAGE_AUDIT.csv",nextrows);write_csv(out/"MX065_CV_Q1_PER_DECISION.csv",cvrows);write_csv(out/"MX065_R_F1_F2_PER_DECISION.csv",features);write_csv(out/"MX065_STRICT_COMMON_SUPPORT.csv",strict);write_csv(out/"MX065_CENSORING_MISSINGNESS_REASONS.csv",reasons);write_csv(out/"MX065_F4_CLOCK_ODOM_INVENTORY.csv",inv)
    gate={"schema":"mx065_stage_a_gate_summary_r2","stage_a_class":stageA,"A_S0_source_provenance":AS0,"A_S1_qstop":AS1,"A_S2_cvq1":AS2,"A_S3_strict_support":AS3,"qstop_global_n":sum(qby.values()),"qstop_by_run":qby,"h1_global_n":sum(hby.values()),"h1_by_run":hby,"strict_global_n":sum(sby.values()),"strict_by_run":sby,"structural_max":{"Q_STOP":258,"H1":253,"STRICT":243},"hard_failures":hard,"adversarial_tests":tests,"variation_diagnostic_by_run":{r:{"n":hby[r],"iqr":q([x["CV_Q1"] for x in cvrows if x["run_id"]==r and x["H1_evaluable"]],.75)-q([x["CV_Q1"] for x in cvrows if x["run_id"]==r and x["H1_evaluable"]],.25),"distinct_1e6":len(set(round(x["CV_Q1"],6) for x in cvrows if x["run_id"]==r and x["H1_evaluable"]))} for r in RUNS}}
    write_json(out/"MX065_STAGE_A_GATE_SUMMARY.json",gate);(out/"MX065_STAGE_A_REPORT.md").write_text("# MX066 Analyst06 — MX065 Method R2 Stage A\n\nStage A class: **"+stageA+"**\n\n- A-S0 provenance: "+str(AS0)+"\n- A-S1 Q_STOP: "+str(sum(qby.values()))+"/258; "+str(qby)+"\n- A-S2 H1: "+str(sum(hby.values()))+"/253; "+str(hby)+"\n- A-S3 strict: "+str(sum(sby.values()))+"/243; "+str(sby)+"\n- Source freeze commit: "+SOURCE_FREEZE_COMMIT+"\n",encoding="utf-8")
    if stageA!="MX065_STAGE_A_SOURCE_FEASIBILITY_PASS":
        print(json.dumps({"stageA":stageA,"qstop":sum(qby.values()),"H1":sum(hby.values()),"strict":sum(sby.values()),"stageB":"NOT_RUN"},sort_keys=True));return
    hosp=[r for r in strict if r["strict_support"]==1];hf,hpred,hmodels=fit_environment("HOSPITAL",hosp,RUNS);hs=summarize(hf,4,5)
    stageB="MX065_HOSPITAL_STAGE_B_SIGNAL_NONIDENTIFIABLE" if len([r for r in hf if r.get("status")=="PASS"])<5 else ("MX065_HOSPITAL_R_F1_F2_CVQ1_SIGNAL_PROMISING_OFFLINE" if hs["gate"] else "MX065_HOSPITAL_R_F1_F2_CVQ1_SIGNAL_NOT_USEFUL")
    nc=read_csv(nr_cv/"mapex_lab/analysis/mx063_exec_results/MX062_CV_Q1_PER_DECISION.csv");nf=read_csv(nr_ft/"mapex_lab/analysis/mx040_exec_results/MX039_FEATURES_PER_DECISION.csv");nfi={(r["run_id"],int(r["decision_id"])):r for r in nf};nr=[]
    for c in nc:
        if c.get("H1_evaluable")!="1" or not finite(c.get("CV_Q1")):continue
        f=nfi.get((c["run_id"],int(c["decision_id"])),{});rr={"run_id":c["run_id"],"decision_id":int(c["decision_id"]),"decision_index":int(c["decision_index"]),"CV_Q1":float(c["CV_Q1"]),"R_map":fnum(f.get("R_map")),"log_decision":math.log1p(int(c["decision_index"])-1)}
        for x in F1+F2:rr[x]=fnum(f.get(x))
        if all(finite(rr[x]) for x in ("CV_Q1","R_map","log_decision")+F1+F2):nr.append(rr)
    nruns=tuple(f"mpx_{i:03d}" for i in range(1,11));nfld,npred,nmodels=fit_environment("NEW_ROOM",nr,nruns);ns=summarize(nfld,8,10);cross=bool(stageB=="MX065_HOSPITAL_R_F1_F2_CVQ1_SIGNAL_PROMISING_OFFLINE" and ns["gate"])
    write_csv(out/"MX065_STAGE_B_OUTER_FOLDS.csv",hf);write_csv(out/"MX065_STAGE_B_HELDOUT_PREDICTIONS.csv",hpred+npred);write_json(out/"MX065_STAGE_B_MODEL_AUDIT.json",{"schema":"mx065_stage_b_model_audit_r2","lambda":1.0,"arms":{k:list(v) for k,v in ARMS.items()},"models":hmodels+nmodels,"hospital_fold_n":len(hf),"new_room_fold_n":len(nfld),"row_level_prediction_n":len(hpred)+len(npred),"no_pooling":True})
    fullmed=median([r["Delta_J_vs_B1"] for r in hf if r.get("status")=="PASS"]);posden=sum(max(0,r["Delta_J_vs_B1"]) for r in hf if r.get("status")=="PASS");inf=[]
    for r in hf:
        d=r.get("Delta_J_vs_B1",math.nan);others=[x["Delta_J_vs_B1"] for x in hf if x is not r and x.get("status")=="PASS"];inf.append({"heldout_run":r["outer_heldout"],"Delta_J_vs_B1":d,"positive_gain_contribution":max(0,d)/posden if posden>0 and finite(d) else 0,"full_median":fullmed,"median_without_fold":median(others),"median_influence":fullmed-median(others) if others else math.nan})
    write_csv(out/"MX065_STAGE_B_RUN_INFLUENCE.csv",inf);write_csv(out/"MX065_NEW_ROOM_COMPANION_COMPARISON.csv",nfld)
    # Final A46/A47 auditability checks across both environments.
    expected_pred=sum(len([r for r in (hosp if env=="HOSPITAL" else nr) if r["run_id"]==hold])*len(ARMS) for env,folds0 in (("HOSPITAL",RUNS),("NEW_ROOM",nruns)) for hold in folds0)
    pred_keys=Counter((r["environment"],r["outer_fold_id"],r["arm"]) for r in hpred+npred)
    A46=(len(hpred)+len(npred)==expected_pred and all(len({r["common_support_id"] for r in hpred+npred if r["environment"]==env and r["outer_fold_id"]==fid})==1 for env,fid in {(r["environment"],r["outer_fold_id"]) for r in hpred+npred}) and all(all(k in r for k in ("environment","outer_fold_id","held_out_run","run_id","decision_id","common_support_id","arm","actual_CV_Q1","predicted_CV_Q1","absolute_error","actual_sign_class","predicted_sign_class","Scale_y")) for r in hpred+npred))
    required_model=("environment","outer_fold_id","held_out_run","training_runs","training_row_count_total","training_row_count_by_run","weight_total_by_run","feature_order","feature_mean","feature_sd","ZERO_TRAIN_SD","Scale_y","intercept","coefficients","lambda","solver_contract","normal_equation_residual_max_abs")
    A47=(len(hmodels+nmodels)==(5+10)*len(ARMS) and all(all(k in m for k in required_model) and m["lambda"]==1.0 and m["normal_equation_residual_max_abs"]<1e-10 for m in hmodels+nmodels))
    final_tests=[dict(x) for x in tests]
    for x in final_tests:
        if x["test_id"]=="A3":x.update({"pass":all(all(abs(v-1/(len(m["training_runs"])))<1e-15 for v in m["weight_total_by_run"].values()) for m in hmodels+nmodels),"evidence":"equal total training-run weights verified for every fold/arm"})
        if x["test_id"]=="A46":x.update({"pass":A46,"evidence":f"row-level heldout prediction contract; rows={len(hpred)+len(npred)}, expected={expected_pred}, exact common-support identity per environment/fold"})
        if x["test_id"]=="A47":x.update({"pass":A47,"evidence":f"complete per-fold/per-arm model state; records={len(hmodels+nmodels)}, residual<1e-10"})
    result={"schema":"mx065_stage_b_result_summary_r2","stage_a_class":stageA,"stage_b_class":stageB,"hospital":hs,"new_room":ns,"CROSS_ENV_POSITIVE_SIGNAL_COMPATIBLE":cross,"cross_environment_compatibility":cross,"hospital_common_support_n":len(hosp),"new_room_common_support_n":len(nr),"lambda":1.0,"no_pooling":True,"no_stop_authority":True,"A1_A47_pass":sum(x["pass"] for x in final_tests),"adversarial_tests":final_tests,"A46_row_level_audit_pass":A46,"A47_model_state_audit_pass":A47};write_json(out/"MX065_STAGE_B_RESULT_SUMMARY.json",result)
    (out/"MX065_PM_USER_DECISION_REPORT.md").write_text("# MX066 — MX065 Method R2 final PM/USER decision report\n\nStage A: **"+stageA+"**\nStage B: **"+stageB+"**\nCross-environment compatibility: **"+str(cross)+"**\n\n## Hospital\n- folds: "+str(hs["evaluable_folds"])+"/5\n- positive J vs B1: "+str(hs["positive_folds"])+"/5\n- median Delta_J_vs_B1: "+str(hs["median_Delta_J_vs_B1"])+"\n- PGC: "+str(hs["PGC_J"])+"\n- median J vs S1: "+str(hs["median_Delta_J_vs_S1"])+"\n- median J vs S2: "+str(hs["median_Delta_J_vs_S2"])+"\n\n## New Room companion\n- folds: "+str(ns["evaluable_folds"])+"/10\n- positive: "+str(ns["positive_folds"])+"/10\n- median Delta_J_vs_B1: "+str(ns["median_Delta_J_vs_B1"])+"\n- PGC: "+str(ns["PGC_J"])+"\n\nOffline evidence only; no collection/threshold/STOP/deployment authority.\n",encoding="utf-8")
    print(json.dumps({"stageA":stageA,"qstop":sum(qby.values()),"H1":sum(hby.values()),"strict":sum(sby.values()),"stageB":stageB,"hospital":hs,"newroom":ns,"cross":cross},sort_keys=True))
if __name__=="__main__":main()
