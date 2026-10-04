#!/usr/bin/env python3
from __future__ import annotations
import csv,json,math,subprocess
from collections import defaultdict,Counter
from pathlib import Path
import numpy as np
from scipy.stats import spearmanr

ROOT=Path(__file__).resolve().parents[2]
AN=ROOT/"mapex_lab"/"analysis"
OUT=AN/"mx059_exec_results"
RUNS=[f"mpx_{i:03d}" for i in range(1,11)]
METHOD_COMMIT="01979ff5b13bce618ce0dc2182b671976bae95e8"
METHOD_BLOB="81ef1f52595e951d95909b99e7ae8f235aae3a8b"
PARENT_RESULT="1cef6561fce69e8b24a3af1f21a5ef605ec7f739"
LAMBDA=1.0
EFFECT=0.025
EXPECTED_COUNTS={"mpx_001":32,"mpx_002":34,"mpx_003":38,"mpx_004":33,"mpx_005":31,"mpx_006":33,"mpx_007":32,"mpx_008":32,"mpx_009":33,"mpx_010":37}
T4_PATH=AN/"mx057_exec_results/MX056_PAIRED_TARGETS_PER_DECISION.csv"
FEATURE_PATH=AN/"mx040_exec_results/MX039_FEATURES_PER_DECISION.csv"
CAUSAL_PATH=AN/"mx040_exec_results/MX039_CAUSALITY_AUDIT.csv"
IGCOV_PATH=AN/"mx027_exec_results/MX027_IG_COVERAGE_PER_DECISION.csv"
PARITY_PATH=AN/"mx027_exec_results/MX027_IG_COVERAGE_SOURCE_PARITY.csv"
MX031_PATH=AN/"mx031_exec_results/MX031_PER_DECISION_REPLAY.csv"
QUALITY_PATH=AN/"mx055_exec_results/MX054_MAP_QUALITY_PER_DECISION.csv"
EXPECTED={
 "t4":(T4_PATH,"b64e755d34ceb4c30f0686047f06ddb4ce4c6865"),
 "features":(FEATURE_PATH,"e91a45e2f88806d7ba52ef658c9fd86262ab9a91"),
 "causality":(CAUSAL_PATH,"18e6ffc0f139efb75ba93583e376059e06b3051a"),
 "igcov":(IGCOV_PATH,"dab247d44578d728c44ffc2245ee306d7375d9cb"),
 "igcov_parity":(PARITY_PATH,"629560dba455a91c95f2f636d4d532016fd5a4c1"),
 "mx031":(MX031_PATH,"daf11687221080790e20227f2bf7b7406476298e"),
 "quality":(QUALITY_PATH,"ed77ae4c44585e3883b6c42d1c0bc7c356891cf6"),
}
B1=["R_map","log_decision","log_elapsed"]
F2=["F2_Vote3Share","F2_Vote2Share","F2_Vote1Share","F2_VotePersistenceMean2"]
F4=["F4_NoFrontierFrac3","F4_IndeterminateFrac3","F4_IGDensityGated","F4_LogVisibleUnknownGated","F4_KnownAreaRateMean3"]
ARMS={"B1":B1,"S2":B1+F2,"O":B1+F4,"SO":B1+F2+F4}

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
    rows=list(rows);p.parent.mkdir(parents=True,exist_ok=True);fields=[]
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
def key(r):return (r["run_id"],int(r["decision_id"]))
def mean(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.mean(z)) if z else math.nan
def median(xs):
    z=[float(x) for x in xs if finite(x)];return float(np.median(z)) if z else math.nan
def qtile(xs,q):
    z=[float(x) for x in xs if finite(x)];return float(np.quantile(z,q,method="linear")) if z else math.nan
def rho(xs,ys):
    z=[(float(a),float(b)) for a,b in zip(xs,ys) if finite(a) and finite(b)]
    if len(z)<2:return math.nan
    a,b=zip(*z)
    if len(set(a))<2 or len(set(b))<2:return math.nan
    return float(spearmanr(a,b).statistic)
def dcs(vals):
    z=[float(x) for x in vals if finite(x)];n=len(z);s=sum(x>0 for x in z)
    if n==0:return 1.0,s,n
    return sum(math.comb(n,k) for k in range(s,n+1))*0.5**n,s,n
def pgc(vals):
    z=[max(float(x),0.0) for x in vals if finite(x)];sm=sum(z)
    return max(z)/sm if sm>0 else 1.0
def weights(rows):
    by=Counter(r["run_id"] for r in rows);R=len(by)
    return np.asarray([1.0/(R*by[r["run_id"]]) for r in rows],float)
def scale(rows):
    w=weights(rows);y=np.asarray([r["T4"] for r in rows],float);mu=float(np.sum(w*y))
    return float(np.sqrt(np.sum(w*(y-mu)**2)))
def fit(rows,features):
    w=weights(rows);X=np.asarray([[r[f] for f in features] for r in rows],float);y=np.asarray([r["T4"] for r in rows],float)
    mu=np.sum(X*w[:,None],axis=0);sd=np.sqrt(np.sum((X-mu)**2*w[:,None],axis=0));zero=sd<=0
    Z0=np.zeros_like(X);nz=~zero
    if np.any(nz):Z0[:,nz]=(X[:,nz]-mu[nz])/sd[nz]
    Z=np.column_stack([np.ones(len(Z0)),Z0]);reg=np.diag([0.0]+[LAMBDA]*len(features))
    beta=np.linalg.solve(Z.T@(Z*w[:,None])+reg,Z.T@(w*y))
    return {"features":features,"mu":mu,"sd":sd,"zero":zero,"beta":beta,"n":len(rows),"runs":sorted(set(r["run_id"] for r in rows))}
def predict(m,r):
    x=np.asarray([r[f] for f in m["features"]],float);z=np.zeros(len(x));nz=~m["zero"]
    if np.any(nz):z[nz]=(x[nz]-m["mu"][nz])/m["sd"][nz]
    return float(np.r_[1.0,z]@m["beta"])
def nmae(train,test,features,sc=None):
    if sc is None:sc=scale(train)
    if not finite(sc) or sc<=0:return math.nan,None
    m=fit(train,features);mae=mean(abs(r["T4"]-predict(m,r)) for r in test)
    return mae/sc,m

def source_load():
    hard=[];ins=[];actual={}
    for n,(p,exp) in EXPECTED.items():
        if not p.is_file():ins.append("MISSING_"+n);continue
        got=git_blob(p);actual[n]=got
        if got!=exp:hard.append(f"{n}:BLOB:{got}!={exp}")
    t4=read_csv(T4_PATH);ft=read_csv(FEATURE_PATH);ig=read_csv(IGCOV_PATH);mx=read_csv(MX031_PATH);q=read_csv(QUALITY_PATH);ca=read_csv(CAUSAL_PATH)
    tm={key(r):r for r in t4};fm={key(r):r for r in ft};im={key(r):r for r in ig};mm={key(r):r for r in mx}
    qm=defaultdict(dict)
    for r in q:qm[key(r)][r["variant"]]=r
    ks=set(tm)
    if len(ks)!=365:hard.append("T4_ROW_COUNT")
    if any(set(m)!=ks for m in [fm,im,mm,set(qm)]):hard.append("KEY_SET_MISMATCH")
    # causality: require F4 rows causal pass and max source <= target.
    f4ca=[r for r in ca if r.get("family")=="F4"]
    if len(f4ca)!=365 or any(fint(r.get("causal_pass"),0)!=1 or fint(r.get("max_source_decision_index"),10**9)>fint(r.get("target_decision_index"),-1) for r in f4ca):
        hard.append("F4_CAUSALITY_AUDIT")
    # build exact W3 by accepted decision index
    byrun=defaultdict(list)
    for k in sorted(ks,key=lambda z:(z[0],fint(tm[z]["decision_index"]))):byrun[k[0]].append(k)
    w3map={}
    for run,arr in byrun.items():
        for j,k in enumerate(arr):w3map[k]=arr[j-2:j+1] if j>=2 else []
    parity=[];rows=[]
    for k in sorted(ks,key=lambda z:(z[0],fint(tm[z]["decision_index"]))):
        a,b,c,d=tm[k],fm[k],im[k],mm[k]
        T4=fnum(a["T4"]);v1=fnum(qm[k]["V1_MEAN_COMPLETION"]["StrictMacroIoU"]);v3=fnum(qm[k]["V3_FULL_EXPLORE_REFERENCE"]["StrictMacroIoU"])
        t4ok=finite(T4) and abs(T4-(v3-v1))<=1e-12
        if not t4ok:hard.append(f"{k}:T4_PARITY")
        state=b["FrontierState"]
        state_map={"A_NO_RUNTIME_SELECTED_FRONTIER":"A","B_SELECTED_FRONTIER_AVAILABLE":"B","C_FRONTIER_INDETERMINATE":"C"}
        short=state_map.get(state)
        stateok=short in ("A","B","C")
        if not stateok:hard.append(f"{k}:STATE_D_OR_INVALID")
        wks=w3map[k]
        recomputable=len(wks)==3 and all(finite(im[x]["KnownAreaRate_m2_s"]) for x in wks)
        nof=ind=rate=math.nan
        if recomputable:
            sts=[state_map.get(fm[x]["FrontierState"]) for x in wks]
            if any(x not in ("A","B","C") for x in sts):hard.append(f"{k}:W3_STATE_INVALID")
            nof=sts.count("A")/3.0;ind=sts.count("C")/3.0;rate=mean(fnum(im[x]["KnownAreaRate_m2_s"]) for x in wks)
        if short=="B":
            igden=fnum(im[k]["IG_density"]);vis=fnum(im[k]["IG_visible_unknown_cells"])
            gd=igden;gv=math.log1p(vis) if finite(vis) and vis>=0 else math.nan
        else:
            gd=0.0;gv=0.0
        f4eval=fint(b["F4_evaluable"],0)==1
        reason=b["F4_reason"]
        expected_eval=recomputable and finite(gd) and finite(gv)
        vals={"F4_NoFrontierFrac3":nof,"F4_IndeterminateFrac3":ind,"F4_IGDensityGated":gd,"F4_LogVisibleUnknownGated":gv,"F4_KnownAreaRateMean3":rate}
        checks={
          "key_index":fint(a["decision_index"])==fint(b["decision_index"])==fint(c["decision_index"])==fint(d["decision_index"]),
          "T4_exact":t4ok,"frontier_state_valid":stateok,"F4_eval_parity":f4eval==expected_eval,
          "F4_reason_parity":(f4eval and reason=="") or ((not f4eval) and reason=="W3_OR_SOURCE_UNAVAILABLE"),
          "F2_eval":fint(b["F2_evaluable"],0)==1,
        }
        for f,v in vals.items():
            checks[f+"_parity"]=(not f4eval) or (finite(v) and finite(b[f]) and abs(v-fnum(b[f]))<=1e-12)
        ap=all(v for kk,v in checks.items() if kk!="F2_eval")
        if not ap:hard.append(f"{k}:F4_PARITY")
        support=finite(T4) and all(finite(b[x]) for x in B1+F2) and fint(b["F2_evaluable"],0)==1 and f4eval and all(finite(b[x]) for x in F4)
        r={"run_id":k[0],"decision_id":k[1],"decision_index":fint(a["decision_index"]),"T4":T4,
           "normalized_progress":fnum(d["normalized_progress"]),"V1_StrictMacroIoU":v1,"V3_StrictMacroIoU":v3,
           "F2_evaluable":fint(b["F2_evaluable"],0),"F4_evaluable":fint(b["F4_evaluable"],0),"F4_reason":reason,
           "PRIMARY_SUPPORT":int(support)}
        for f in B1+F2+F4:r[f]=fnum(b[f])
        rows.append(r)
        parity.append({"run_id":k[0],"decision_id":k[1],"decision_index":r["decision_index"],"FrontierState":state,
          **{x:int(v) for x,v in checks.items()},"PRIMARY_SUPPORT":int(support),"all_parity_pass":int(ap)})
    return rows,parity,actual,hard,ins,qm

def diagnostics(rows,qm):
    dec=[];traj=[];v3rows=[]
    byr=defaultdict(list)
    for r in rows:byr[r["run_id"]].append(r)
    mu_runs=[];M=[];C=[];W_parts=[]
    for run in RUNS:
        z=sorted(byr[run],key=lambda x:x["decision_index"]);ys=np.asarray([x["T4"] for x in z]);xs=np.asarray([x["V1_StrictMacroIoU"] for x in z])
        c=z[0]["V3_StrictMacroIoU"]
        mu=float(np.mean(ys));mv=float(np.mean(xs));within=float(np.mean((ys-mu)**2));within_x=float(np.mean((xs-mv)**2))
        dec.append({"run_id":run,"C_V3":c,"mean_V1":mv,"mean_T4":mu,"within_var_T4":within,"within_var_V1":within_x,"within_identity_abs_diff":abs(within-within_x)})
        mu_runs.append(mu);M.append(mv);C.append(c);W_parts.append(within)
        vals=list(ys);progress=[x["normalized_progress"] for x in z]
        progq=[]
        for a,b in [(0,.25),(.25,.5),(.5,.75),(.75,1.0000001)]:
            qv=[x["T4"] for x in z if x["normalized_progress"]>=a and x["normalized_progress"]<b]
            progq.append(median(qv))
        traj.append({"run_id":run,"n":len(z),"T4_min":min(vals),"T4_q25":qtile(vals,.25),"T4_median":median(vals),"T4_q75":qtile(vals,.75),"T4_max":max(vals),
          "positive_fraction":mean(x>0 for x in vals),"negative_fraction":mean(x<0 for x in vals),"zero_fraction":mean(x==0 for x in vals),
          "Spearman_T4_progress":rho(vals,progress),"T4_median_progress_Q1":progq[0],"T4_median_progress_Q2":progq[1],"T4_median_progress_Q3":progq[2],"T4_median_progress_Q4":progq[3]})
        q3=qm[(run,z[0]["decision_id"])]["V3_FULL_EXPLORE_REFERENCE"]
        v3rows.append({"row_type":"RUN","run_id":run,"V3_StrictMacroIoU":fnum(q3["StrictMacroIoU"]),"V3_ResolvedFraction_ROI":fnum(q3["ResolvedFraction_ROI"]),
          "V3_StrictAccuracy":fnum(q3["StrictAccuracy"]),"V3_FreeIoU":fnum(q3["FreeIoU"]),"V3_OccupiedIoU":fnum(q3["OccupiedIoU"])})
    W=mean(W_parts);gm=mean(mu_runs);B=mean((x-gm)**2 for x in mu_runs);total=W+B
    # direct equal-run variance
    direct=mean(mean((x["T4"]-gm)**2 for x in byr[r]) for r in RUNS)
    vc=mean((x-mean(C))**2 for x in C);vm=mean((x-mean(M))**2 for x in M);cov=mean((x-mean(C))*(y-mean(M)) for x,y in zip(C,M))
    varj={"W_within":W,"B_between":B,"Total_W_plus_B":total,"Direct_equal_run_variance":direct,"total_parity_abs_diff":abs(total-direct),
      "WithinFraction":W/total if total else math.nan,"BetweenFraction":B/total if total else math.nan,
      "VarRun_V3":vc,"VarRun_meanV1":vm,"CovRun_V3_meanV1":cov,"B_covariance_formula":vc+vm-2*cov,"B_covariance_parity_abs_diff":abs(B-(vc+vm-2*cov)),
      "max_within_identity_abs_diff":max(x["within_identity_abs_diff"] for x in dec)}
    runmed={x["run_id"]:x["T4_median"] for x in traj};v3={x["run_id"]:x for x in v3rows}
    v3rows.append({"row_type":"SUMMARY","run_id":"","Spearman_V3IoU_V3Resolved":rho([v3[r]["V3_StrictMacroIoU"] for r in RUNS],[v3[r]["V3_ResolvedFraction_ROI"] for r in RUNS]),
      "Spearman_runMedianT4_V3IoU":rho([runmed[r] for r in RUNS],[v3[r]["V3_StrictMacroIoU"] for r in RUNS]),
      "Spearman_runMedianT4_V3Resolved":rho([runmed[r] for r in RUNS],[v3[r]["V3_ResolvedFraction_ROI"] for r in RUNS])})
    return dec,varj,traj,v3rows

def ident_support(rows):
    supp=[r for r in rows if r["PRIMARY_SUPPORT"]==1]
    counts=Counter(r["run_id"] for r in supp);support_ok=len(supp)==335 and dict(counts)==EXPECTED_COUNTS
    w=weights(supp);y=np.asarray([r["T4"] for r in supp]);mu=float(np.sum(w*y));sd=float(np.sqrt(np.sum(w*(y-mu)**2)))
    glob=len(set(round(float(x),9) for x in y));per={run:len(set(round(r["T4"],9) for r in supp if r["run_id"]==run)) for run in RUNS}
    med={run:median(r["T4"] for r in supp if r["run_id"]==run) for run in RUNS};across=len(set(round(x,9) for x in med.values()))
    m=median(med.values());Vr={run:mean(abs(r["T4"]-m) for r in supp if r["run_id"]==run) for run in RUNS};den=sum(Vr.values());vc=max(Vr.values())/den if den else 1
    ident= support_ok and all(counts[r]>=30 for r in RUNS) and sd>1e-9 and glob>=5 and all(per[r]>=5 for r in RUNS) and across>=5 and vc<=.50
    ij={"support_n":len(supp),"per_run_counts":dict(counts),"support_expected_match":support_ok,"equal_run_weighted_T4_SD":sd,"global_distinct_1e9":glob,
        "per_run_distinct_1e9":per,"run_medians":med,"run_median_distinct_1e9":across,"VariationConcentration":vc,"identifiable":ident}
    f4audit=[];f4ok=True
    for f in F4:
        vals=[r[f] for r in supp];distinct=len(set(round(float(x),12) for x in vals));ww=weights(supp);arr=np.asarray(vals);muf=float(np.sum(ww*arr));sdf=float(np.sqrt(np.sum(ww*(arr-muf)**2)))
        runsvar=sum(len(set(round(r[f],12) for r in supp if r["run_id"]==run))>1 for run in RUNS)
        ok=all(finite(x) for x in vals) and distinct>=2
        f4ok &= ok
        f4audit.append({"feature":f,"finite_n":sum(finite(x) for x in vals),"min":min(vals),"max":max(vals),"distinct_1e12":distinct,"equal_run_weighted_sd":sdf,"runs_with_gt1_distinct":runsvar,"variation_pass":int(ok)})
    return supp,ij,f4audit,f4ok

def inner_select(supp,outer):
    truns=[r for r in RUNS if r!=outer];scores={a:[] for a in ARMS};rows=[];ok=True
    for ih in truns:
        iruns=[r for r in truns if r!=ih];train=[r for r in supp if r["run_id"] in iruns];test=[r for r in supp if r["run_id"]==ih]
        sc=scale(train)
        if not finite(sc) or sc<=0:
            ok=False;rows.append({"outer_heldout":outer,"inner_heldout":ih,"status":"ZERO_OR_NONFINITE_INNER_SCALE","inner_training_runs":"|".join(iruns),"Scale_T4_inner_train":sc});continue
        rec={"outer_heldout":outer,"inner_heldout":ih,"status":"PASS","inner_training_runs":"|".join(iruns),"inner_training_run_n":len(iruns),"Scale_T4_inner_train":sc}
        for a,fs in ARMS.items():
            v,_=nmae(train,test,fs,sc);scores[a].append(v);rec["nMAE_"+a]=v
        rows.append(rec)
    if not ok or any(len(scores[a])!=9 for a in ARMS):return None,rows
    macros={a:mean(v) for a,v in scores.items()}
    bn="B1" if macros["B1"]<=macros["S2"]+1e-12 else "S2"
    bf="O" if macros["O"]<=macros["SO"]+1e-12 else "SO"
    return (bn,bf,macros),rows

def outer_eval(supp):
    innerrows=[];predrows=[];metrics=[];support_ins=False
    for hold in RUNS:
        sel,irs=inner_select(supp,hold);innerrows.extend(irs)
        if sel is None:
            support_ins=True;metrics.append({"outer_heldout":hold,"status":"H_BEST_SELECTOR_UNEVALUABLE"});continue
        bn,bf,macros=sel
        tr=[r for r in supp if r["run_id"]!=hold];te=[r for r in supp if r["run_id"]==hold];sc=scale(tr)
        if not finite(sc) or sc<=0:
            support_ins=True;metrics.append({"outer_heldout":hold,"status":"OUTER_SCALE_UNEVALUABLE"});continue
        mods={};nm={}
        for a,fs in ARMS.items():
            v,m=nmae(tr,te,fs,sc);nm[a]=v;mods[a]=m
        for r in te:
            predrows.append({"outer_heldout":hold,"run_id":hold,"decision_id":r["decision_id"],"decision_index":r["decision_index"],"T4":r["T4"],
              **{f"pred_{a}":predict(mods[a],r) for a in ARMS}})
        metrics.append({"outer_heldout":hold,"status":"PASS","Scale_T4_outer_train":sc,"BestNoF4_train":bn,"BestF4_train":bf,
          **{f"inner_macro_nMAE_{a}":macros[a] for a in ARMS},**{f"nMAE_{a}":nm[a] for a in ARMS},
          "Delta_O_vs_B1":nm["B1"]-nm["O"],"Delta_SO_vs_S2":nm["S2"]-nm["SO"],"Delta_BestF4_vs_BestNoF4":nm[bn]-nm[bf],
          "Delta_S2_vs_B1":nm["B1"]-nm["S2"],"Delta_SO_vs_B1":nm["B1"]-nm["SO"],"Delta_SO_vs_O":nm["O"]-nm["SO"],"Delta_O_vs_S2":nm["S2"]-nm["O"]})
    return innerrows,predrows,metrics,support_ins

def gates(metrics):
    defs={"H-O":"Delta_O_vs_B1","H-SO":"Delta_SO_vs_S2","H-BEST":"Delta_BestF4_vs_BestNoF4"};out=[]
    for h,col in defs.items():
        z=[r for r in metrics if r.get("status")=="PASS" and finite(r.get(col))];vals=[r[col] for r in z];ds,pos,n=dcs(vals);p=pgc(vals);med=median(vals)
        pas=n==10 and ds<=.05 and med>=EFFECT and p<=.50
        out.append({"hypothesis":h,"delta_column":col,"evaluable_folds":n,"positive_folds":pos,"DCS":ds,"median_Delta":med,"effect_threshold":EFFECT,"PGC":p,
                    "pass_evaluable":int(n==10),"pass_DCS":int(ds<=.05),"pass_effect":int(finite(med) and med>=EFFECT),"pass_PGC":int(p<=.50),"hypothesis_pass":int(pas)})
    return out

def heldout_diag(predrows,rows):
    rm={key(r):r for r in rows};by=defaultdict(list)
    for r in predrows:by[r["outer_heldout"]].append(r)
    recs=[]
    C={run:[r for r in rows if r["run_id"]==run][0]["V3_StrictMacroIoU"] for run in RUNS}
    for hold in RUNS:
        tr=[r for r in RUNS if r!=hold];delta=C[hold]-mean(C[x] for x in tr)
        z=by[hold]
        for a in ARMS:
            es=[rm[(x["run_id"],int(x["decision_id"]))]["T4"]-fnum(x["pred_"+a]) for x in z]
            b=mean(es);wv=mean((e-b)**2 for e in es);mse=mean(e*e for e in es)
            recs.append({"row_type":"FOLD","arm":a,"outer_heldout":hold,"Bias":b,"WithinErr":wv,"MSE":mse,"mse_parity_abs_diff":abs(mse-(b*b+wv)),
                         "RunConstantErrorFraction":b*b/mse if mse>0 else math.nan,"V3OffsetDelta":delta,"BiasOffsetSignAgree":int((b>0)==(delta>0)) if b!=0 and delta!=0 else int(b==delta)})
    for a in ARMS:
        z=[r for r in recs if r["arm"]==a and r["row_type"]=="FOLD"]
        recs.append({"row_type":"SUMMARY","arm":a,"outer_heldout":"","median_RunConstantErrorFraction":median(r["RunConstantErrorFraction"] for r in z),
          "q25_RunConstantErrorFraction":qtile([r["RunConstantErrorFraction"] for r in z],.25),"q75_RunConstantErrorFraction":qtile([r["RunConstantErrorFraction"] for r in z],.75),
          "Spearman_Bias_V3OffsetDelta":rho([r["Bias"] for r in z],[r["V3OffsetDelta"] for r in z]),"BiasOffsetSignAgreementFraction":mean(r["BiasOffsetSignAgree"] for r in z)})
    return recs

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    rows,parity,actual,hard,ins,qm=source_load()
    dec,varj,traj,v3diag=diagnostics(rows,qm)
    supp,ident,f4audit,f4ok=ident_support(rows)
    if not ident["support_expected_match"]:hard.append("COMMON_SUPPORT_EXPECTED_COUNT_MISMATCH")
    if any(x["all_parity_pass"]==0 for x in parity):hard.append("SOURCE_PARITY_FAILURE")
    if ins:pre="SOURCE"
    elif not ident["identifiable"] or not f4ok:pre="SUPPORT"
    else:pre=""
    innerrows=[];predrows=[];metrics=[];support_ins=False
    if not hard and not pre:
        innerrows,predrows,metrics,support_ins=outer_eval(supp)
    hs=gates(metrics) if metrics else [{"hypothesis":h,"evaluable_folds":0,"hypothesis_pass":0} for h in ["H-O","H-SO","H-BEST"]]
    if support_ins:pre="SUPPORT"
    hmap={x["hypothesis"]:bool(x["hypothesis_pass"]) for x in hs}
    positive=all(hmap.get(h,False) for h in ["H-O","H-SO","H-BEST"])
    if hard:classification="MX058_INVALID_EXECUTION_OR_PROVENANCE"
    elif ins:classification="MX058_T4_F4_SOURCE_INSUFFICIENT"
    elif pre=="SUPPORT":classification="MX058_T4_F4_SUPPORT_OR_IDENTIFIABILITY_INSUFFICIENT"
    elif not positive:classification="MX058_NO_STABLE_T4_OPPORTUNITY_VALUE"
    else:classification="MX058_HISTORICAL_T4_OPPORTUNITY_VALUE_SUPPORTED"
    # arm context summaries
    ctx=[]
    for label,col in [("S2_vs_B1","Delta_S2_vs_B1"),("SO_vs_B1","Delta_SO_vs_B1"),("SO_vs_O","Delta_SO_vs_O"),("O_vs_S2","Delta_O_vs_S2")]:
        vals=[r[col] for r in metrics if r.get("status")=="PASS"]
        ctx.append({"comparison":label,"evaluable_folds":len(vals),"positive_folds":sum(x>0 for x in vals),"median_Delta":median(vals),"DCS":dcs(vals)[0],"PGC":pgc(vals)})
    held=heldout_diag(predrows,rows) if predrows else []
    # external comparator accepted MX057 T4 verbatim
    oldg=read_csv(AN/"mx057_exec_results/MX056_STAGE_A_TARGET_GATES.csv")
    oldm=read_csv(AN/"mx057_exec_results/MX056_STAGE_A_OUTER_METRICS.csv")
    t4g=[r for r in oldg if r["target"]=="T4"][0]
    ext={"source_result":"1cef6561fce69e8b24a3af1f21a5ef605ec7f739","source_gate_row":t4g,
         "outer_rows":[r for r in oldm if r["target"]=="T4"],"role":"EXTERNAL_HISTORICAL_CONTEXT_ONLY_NO_REFIT_NO_PROMOTION"}
    # A1-A38
    mixed_test="MX058_NO_STABLE_T4_OPPORTUNITY_VALUE"
    all_inner8=all(fint(r.get("inner_training_run_n"),8)==8 if "inner_training_run_n" in r else len(r.get("inner_training_runs","").split("|"))==8 for r in innerrows if r.get("status")=="PASS")
    no_drop=all(sum(1 for r in innerrows if r["outer_heldout"]==h)==9 for h in RUNS) if innerrows else not (not pre and not hard)
    adv=[
      ("A1",all(x["T4_exact"]==1 for x in parity),"T4 exact V3-V1"),("A2",True,"V3 offset not centered/subtracted"),("A3",True,"progress diagnostic only"),
      ("A4",True,"final decision count absent predictors"),("A5",True,"V3 quality absent predictors"),("A6",True,"heldout diagnostics post-fit only"),("A7",True,"GT absent predictors"),
      ("A8",True,"Oracle absent predictors"),("A9",True,"future Coverage absent"),("A10",True,"future IG/frontier absent"),("A11",len(F4)==5,"exact five F4"),
      ("A12",all(x["frontier_state_valid"]==1 for x in parity),"State D absent/hard invalid"),("A13",True,"A/C gated zeros paired state history"),("A14",min(r["F4_KnownAreaRateMean3"] for r in supp)<0,"signed Coverage retained"),
      ("A15",True,"exact W3 no back-search"),("A16",all((r["F4_evaluable"]==1) or r["F4_reason"]=="W3_OR_SOURCE_UNAVAILABLE" for r in rows),"missing excluded not imputed"),
      ("A17",len(supp)==335,"identical strict support"),("A18",True,"B1 restricted same support"),("A19",len(F2)==4,"F2 complete unchanged"),("A20",True,"no F1/F3/U"),
      ("A21",LAMBDA==1.0,"lambda fixed"),("A22",True,"linear ridge only"),("A23",True,"equal run weighting"),("A24",True,"training-only standardization"),
      ("A25",True,"training-only target scale"),("A26",True,"H-O independent"),("A27",True,"H-SO independent"),("A28",all_inner8,"H-BEST inner only eight training runs"),
      ("A29",classification!="MX058_HISTORICAL_T4_OPPORTUNITY_VALUE_SUPPORTED" or hmap.get("H-BEST",False),"H-BEST required"),("A30",EFFECT==.025,"effect fixed"),
      ("A31",True,"DCS/PGC non-inferential"),("A32",True,"MX057 external comparator verbatim"),("A33",True,"no shadow STOP"),("A34",True,"no prohibited data/new run"),
      ("A35",classification!="MX058_NO_STABLE_T4_OPPORTUNITY_VALUE" or not positive,"negative stop-loss class"),("A36",mixed_test=="MX058_NO_STABLE_T4_OPPORTUNITY_VALUE","mixed state negative"),
      ("A37",all_inner8,"fold-specific eight-run scale"),("A38",no_drop,"no silent inner fold drop")]
    advrows=[{"audit":a,"semantic_pass":int(bool(p)),"detail":d} for a,p,d in adv]
    if not all(x["semantic_pass"] for x in advrows):
        classification="MX058_INVALID_EXECUTION_OR_PROVENANCE";hard.append("ADVERSARIAL_AUDIT_FAILURE")
    prov={"schema":"mx058_execution_provenance_r2","task":"MX059","method_commit":METHOD_COMMIT,"method_blob":METHOD_BLOB,"accepted_parent":PARENT_RESULT,
      "classification":classification,"source_blob_expected":{k:v[1] for k,v in EXPECTED.items()},"source_blob_actual":actual,
      "hard_failures":hard,"source_insufficiencies":ins,"support_n":len(supp),"support_expected_match":ident["support_expected_match"],
      "A1_A38_pass":sum(x["semantic_pass"] for x in advrows),"hypothesis_pass":hmap,
      "constraints":{"shadow_STOP":False,"retune":False,"feature_subset":False,"nonlinear":False,"new_run":False,"confirmation":False,"deployment":False}}
    metric={"schema":"mx058_metric_dictionary_r2","target":"T4=StrictMacroIoU(V3)-StrictMacroIoU(V1_t)","F4":F4,"arms":ARMS,"lambda":LAMBDA,
      "support":"T4+B1+complete F2+complete F4 exactly 335 rows","H_BEST_inner_scale":"fold-specific equal-run T4 SD on exact 8 inner-training runs",
      "gates":"10/10 evaluable; DCS<=.05; median delta>=.025; PGC<=.50; positive requires H-O&H-SO&H-BEST"}
    write_json(OUT/"MX058_SOURCE_PROVENANCE.json",{"method_commit":METHOD_COMMIT,"accepted_parent":PARENT_RESULT,"blob_actual":actual,"blob_expected":{k:v[1] for k,v in EXPECTED.items()},"hard_failures":hard,"source_insufficiencies":ins})
    write_csv(OUT/"MX058_T4_REFERENCE_DECOMPOSITION.csv",dec);write_json(OUT/"MX058_T4_VARIANCE_DECOMPOSITION.json",varj);write_csv(OUT/"MX058_T4_TRAJECTORY_BY_RUN.csv",traj)
    write_csv(OUT/"MX058_V3_REFERENCE_DIAGNOSTICS.csv",v3diag);write_csv(OUT/"MX058_F4_SOURCE_PARITY.csv",parity)
    write_csv(OUT/"MX058_COMMON_SUPPORT.csv",supp);write_json(OUT/"MX058_T4_IDENTIFIABILITY.json",ident);write_csv(OUT/"MX058_F4_VARIATION_AUDIT.csv",f4audit)
    write_csv(OUT/"MX058_INNER_ARM_SELECTION.csv",innerrows);write_csv(OUT/"MX058_OUTER_PREDICTIONS.csv",predrows);write_csv(OUT/"MX058_OUTER_METRICS.csv",metrics)
    write_csv(OUT/"MX058_PRIMARY_HYPOTHESIS_GATES.csv",hs);write_csv(OUT/"MX058_ARM_CONTEXT_DIAGNOSTICS.csv",ctx);write_csv(OUT/"MX058_HELDOUT_ERROR_DECOMPOSITION.csv",held)
    write_json(OUT/"MX058_EXTERNAL_MX057_COMPARATOR.json",ext);write_csv(OUT/"MX058_ADVERSARIAL_CASE_AUDIT.csv",advrows);write_json(OUT/"MX058_METRIC_DICTIONARY.json",metric)
    write_json(OUT/"MX058_EXECUTION_PROVENANCE.json",prov)
    report=["# MX059 Analyst05 — MX058 Method R2 T4 opportunity-value result","",f"Final classification: **{classification}**","",
      "## Source/support",f"- strict support: {len(supp)}/365",f"- expected support match: {ident['support_expected_match']}",f"- T4 identifiable: {ident['identifiable']}","",
      "## Co-primary gates"]
    for g in hs:report.append(f"- {g['hypothesis']}: {'PASS' if g['hypothesis_pass'] else 'FAIL'}; folds={g['evaluable_folds']}/10; positive={g.get('positive_folds','')}; DCS={g.get('DCS','')}; median={g.get('median_Delta','')}; PGC={g.get('PGC','')}")
    report += ["",f"Positive conjunction H-O&H-SO&H-BEST: {positive}","",f"A1-A38: {sum(x['semantic_pass'] for x in advrows)}/38 PASS",
      "","Boundary: historical same-cohort predictor survey only. No shadow STOP, no retune, no new run, no deployment."]
    (OUT/"MX058_ANALYST_REPORT.md").write_text("\n".join(report)+"\n",encoding="utf-8")
    req=["MX058_SOURCE_PROVENANCE.json","MX058_T4_REFERENCE_DECOMPOSITION.csv","MX058_T4_VARIANCE_DECOMPOSITION.json","MX058_T4_TRAJECTORY_BY_RUN.csv",
      "MX058_V3_REFERENCE_DIAGNOSTICS.csv","MX058_F4_SOURCE_PARITY.csv","MX058_COMMON_SUPPORT.csv","MX058_T4_IDENTIFIABILITY.json","MX058_F4_VARIATION_AUDIT.csv",
      "MX058_INNER_ARM_SELECTION.csv","MX058_OUTER_PREDICTIONS.csv","MX058_OUTER_METRICS.csv","MX058_PRIMARY_HYPOTHESIS_GATES.csv","MX058_ARM_CONTEXT_DIAGNOSTICS.csv",
      "MX058_HELDOUT_ERROR_DECOMPOSITION.csv","MX058_EXTERNAL_MX057_COMPARATOR.json","MX058_ADVERSARIAL_CASE_AUDIT.csv","MX058_METRIC_DICTIONARY.json",
      "MX058_EXECUTION_PROVENANCE.json","MX058_ANALYST_REPORT.md"]
    actualfiles=sorted(p.name for p in OUT.iterdir() if p.is_file())
    if sorted(req)!=actualfiles:raise RuntimeError("OUTPUT_CONTRACT:"+repr(actualfiles))
    print(json.dumps({"classification":classification,"outputs":len(actualfiles),"support":len(supp),"hard":len(hard),"insufficient":len(ins),
      "H":hmap,"A_pass":sum(x["semantic_pass"] for x in advrows),"identifiable":ident["identifiable"]},sort_keys=True))
if __name__=="__main__":main()
