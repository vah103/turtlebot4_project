#!/usr/bin/env python3
"""Frozen D1 Gate-R V2 historical LORO scorer (H084)."""
from __future__ import annotations
import hashlib, json, math
from pathlib import Path
from typing import Any
import numpy as np
import pandas as pd
from scipy.stats import spearmanr

RUNS=tuple(f"mpx_{i:03d}" for i in range(1,11))
COUNTS=dict(zip(RUNS,(35,37,41,36,34,36,35,35,36,40)))
SEED=20260927
CANDIDATES={
 "RemainingFraction":("RemainingFraction","OracleRemainingFraction_GT","fraction"),
 "A_mean_m2":("A_mean_m2","A_true_remaining_m2","area"),
}
BINS=("[0.00,0.25)","[0.25,0.50)","[0.50,0.75)","[0.75,1.00]")
ALLOWED=("FAIL","INSUFFICIENT_EVIDENCE","HISTORICAL_NEW_ROOM_FEASIBILITY_SUPPORTED_PENDING_CONFIRMATION")

def sha256(path:Path)->str:
 h=hashlib.sha256()
 with path.open("rb") as f:
  for b in iter(lambda:f.read(1<<20),b""): h.update(b)
 return h.hexdigest()

def validate(frame:pd.DataFrame)->None:
 if len(frame)!=365 or frame[["run_id","decision_id"]].duplicated().any(): raise ValueError("not 365 unique keys")
 if frame.groupby("run_id").size().to_dict()!=COUNTS: raise ValueError("run universe mismatch")
 if set(frame.run_id)!=set(RUNS): raise ValueError("run IDs mismatch")

def ecdf_le(reference:np.ndarray, values:np.ndarray)->np.ndarray:
 ref=np.sort(np.asarray(reference,float)); vals=np.asarray(values,float)
 if not len(ref): return np.full(len(vals),np.nan)
 return np.searchsorted(ref,vals,side="right")/len(ref)

def pair(g:pd.DataFrame,name:str)->pd.DataFrame:
 c,y,_=CANDIDATES[name]
 mask=g.oracle_truth_evaluable.astype(bool)&g.d1_runtime_region_evaluable.astype(bool)
 mask &= np.isfinite(pd.to_numeric(g[c],errors="coerce"))&np.isfinite(pd.to_numeric(g[y],errors="coerce"))
 return g.loc[mask].copy()

def rho_value(p:pd.DataFrame,name:str)->float:
 c,y,_=CANDIDATES[name]
 if len(p)<5 or p[c].nunique()<2 or p[y].nunique()<2:return np.nan
 return float(spearmanr(p[c],p[y]).statistic)

def residual(p:pd.DataFrame,name:str)->np.ndarray:
 c,y,kind=CANDIDATES[name]
 if kind=="fraction": return (p[c]-p[y]).to_numpy(float)
 total=float(p.N_GT.iloc[0])*.05**2
 return ((p[c]-p[y])/total).to_numpy(float)

def within_metrics(g:pd.DataFrame,name:str)->dict[str,Any]:
 p=pair(g,name); c,y,_=CANDIDATES[name]
 if not len(p): return {"pairs":0,"rho":np.nan,"trr50":np.nan,"fc25":np.nan,"low25":0,"fc25_count":0,"cal_mae":np.nan}
 cp=ecdf_le(p[c].to_numpy(float),p[c].to_numpy(float)); yp=ecdf_le(p[y].to_numpy(float),p[y].to_numpy(float))
 low50=cp<=.5; low25=cp<=.25; base=float(p[y].mean())
 trr=float(p.loc[low50,y].mean()/base) if low50.any() and base>0 else np.nan
 fc=low25&(yp>=.75)
 e=residual(p,name)
 return {"pairs":len(p),"rho":rho_value(p,name),"trr50":trr,"fc25":float(fc.sum()/low25.sum()) if low25.any() else np.nan,
         "low25":int(low25.sum()),"fc25_count":int(fc.sum()),"cal_mae":float(np.mean(np.abs(e)))}

def development_candidate(frame:pd.DataFrame,dev:list[str],name:str)->dict[str,Any]:
 rows=[]
 for rid in dev:
  m=within_metrics(frame[frame.run_id==rid],name); rows.append({"run_id":rid,**m})
 r=pd.DataFrame(rows); valid=r.rho.notna(); fc=r.fc25.dropna()
 out={"candidate":name,"development_runs":";".join(dev),"valid_rho_count":int(valid.sum()),"positive_rho_count":int((r.rho>0).sum()),
      "rho_dev_macro":float(r.loc[valid,"rho"].mean()) if valid.any() else np.nan,
      "TRR50_dev":float(r.trr50.mean()) if r.trr50.notna().any() else np.nan,
      "FC25_dev":float(fc.mean()) if len(fc) else np.nan,"FC25_dev_contributing_runs":int(len(fc)),
      "CalMAE_dev":float(r.cal_mae.mean()) if r.cal_mae.notna().any() else np.nan,
      "development_detail_json":json.dumps(rows,sort_keys=True,allow_nan=True)}
 checks=[out["valid_rho_count"]>=6,out["positive_rho_count"]>=5,out["rho_dev_macro"]>0,
         out["TRR50_dev"]<1 if np.isfinite(out["TRR50_dev"]) else False,
         out["FC25_dev"]<.25 if np.isfinite(out["FC25_dev"]) else False]
 out["admissible"]=all(checks)
 labels=("valid_rho<6","positive_rho<5","rho_macro<=0","TRR50>=1_or_NA","FC25>=.25_or_NA")
 out["admissibility_reason"]="ADMISSIBLE" if all(checks) else ";".join(x for x,ok in zip(labels,checks) if not ok)
 return out

def choose(rows:list[dict[str,Any]])->str|None:
 ok=[x for x in rows if x["admissible"]]
 if not ok:return None
 order={"RemainingFraction":0,"A_mean_m2":1}
 ok.sort(key=lambda x:(-x["rho_dev_macro"],x["TRR50_dev"],x["FC25_dev"],x["CalMAE_dev"],order[x["candidate"]]))
 return ok[0]["candidate"]

def mapped_percentiles(frame:pd.DataFrame,dev:list[str],held:pd.DataFrame,name:str)->tuple[np.ndarray,np.ndarray]:
 c,y,_=CANDIDATES[name]; cps=[]; yps=[]
 for rid in dev:
  g=frame[frame.run_id==rid]
  cv=pd.to_numeric(g.loc[g.d1_runtime_region_evaluable.astype(bool),c],errors="coerce").dropna().to_numpy()
  yv=pd.to_numeric(g.loc[g.oracle_truth_evaluable.astype(bool),y],errors="coerce").dropna().to_numpy()
  cps.append(ecdf_le(cv,held[c].to_numpy(float))); yps.append(ecdf_le(yv,held[y].to_numpy(float)))
 return np.mean(cps,axis=0),np.mean(yps,axis=0)

def heldout_metrics(frame:pd.DataFrame,held_id:str,dev:list[str],name:str)->tuple[dict[str,Any],pd.DataFrame]:
 full=frame[frame.run_id==held_id].sort_values("decision_index").copy(); p=pair(full,name); c,y,_=CANDIDATES[name]
 cp,yp=mapped_percentiles(frame,dev,p,name); p["C_dev_percentile"]=cp;p["Y_dev_percentile"]=yp
 p["low_C50"]=cp<=.5;p["low_C25"]=cp<=.25;p["high_truth25"]=yp>=.75;p["FC25"]=p.low_C25&p.high_truth25
 p["OracleFC10"]=p.low_C25&(p.OracleRemainingFraction_GT>.10); p["calibration_residual"]=residual(p,name)
 base=float(p[y].mean()) if len(p) else np.nan
 def trr(q):
  low=p.C_dev_percentile<=q
  return float(p.loc[low,y].mean()/base) if low.any() and base>0 else np.nan
 e=p.calibration_residual.to_numpy(float) if len(p) else np.array([])
 result={"held_out_run":held_id,"development_runs":";".join(dev),"selected_candidate":name,"finite_pair_support":len(p),
   "rho":rho_value(p,name),"TRR25":trr(.25),"TRR50":trr(.5),"TRR75":trr(.75),
   "low_C25_count":int(p.low_C25.sum()),"FC25_count":int(p.FC25.sum()),"FC25":float(p.FC25.sum()/p.low_C25.sum()) if p.low_C25.any() else np.nan,
   "OracleFC10_count":int(p.OracleFC10.sum()),"calibration_bias_mean":float(e.mean()) if len(e) else np.nan,
   "calibration_bias_median":float(np.median(e)) if len(e) else np.nan,"calibration_MAE":float(np.mean(abs(e))) if len(e) else np.nan,
   "calibration_p90_abs":float(np.quantile(abs(e),.9)) if len(e)>=10 else np.nan}
 return result,p

def classify(runs:pd.DataFrame,bins:list[dict[str,Any]],oracle_cp:list[float],no_admissible:int)->tuple[str,dict[str,bool]]:
 if no_admissible>=5:return "FAIL",{"pre_sufficiency_no_admissible":True}
 sel=runs.selected_candidate.ne("NO_ADMISSIBLE_C")
 sufficient=sel.sum()>=8 and (runs.finite_pair_support>=5).sum()>=8 and runs.rho.notna().sum()>=8 and runs.TRR50.notna().sum()>=8 and (runs.low_C25_count>0).sum()>=6
 if not sufficient:return "INSUFFICIENT_EVIDENCE",{"general_sufficiency":False}
 rho_med=float(runs.rho.median()); pos=int((runs.rho>0).sum()); trr=float(runs.TRR50.mean()); fc=float(runs.FC25.mean()); cal=float(runs.calibration_bias_mean.median())
 fail={"TRR50_mean_ge_1":trr>=1,"FC25_mean_ge_025":fc>=.25,"adverse_rho":rho_med<=0 and pos<=4,"calibration_median_le_minus_020":cal<=-.20}
 if any(fail.values()):return "FAIL",{"general_sufficiency":True,**fail}
 stage_bad=any(x["contributing_runs"]>=5 and np.isfinite(x["rho_median"]) and x["rho_median"]<=-.1 and np.isfinite(x["TRR50_mean"]) and x["TRR50_mean"]>=1 for x in bins)
 oracle_ok=(len(oracle_cp)<6) or float(np.median(oracle_cp))<=.5
 support={"rho_median_ge_040":rho_med>=.4,"positive_rho_runs_ge_8":pos>=8,"TRR50_mean_le_075":trr<=.75,
          "FC25_mean_le_015":fc<=.15,"calibration_median_gt_minus_010":cal>-.10,"stage_consistent":not stage_bad,"oracle_local_visible_or_insufficient":oracle_ok}
 return ("HISTORICAL_NEW_ROOM_FEASIBILITY_SUPPORTED_PENDING_CONFIRMATION" if all(support.values()) else "INSUFFICIENT_EVIDENCE"),{"general_sufficiency":True,**fail,**support}

def bootstrap(runs:pd.DataFrame,n:int=10000)->dict[str,Any]:
 rng=np.random.default_rng(SEED); out={}
 for col,stat in (("rho","median"),("TRR50","mean"),("FC25","mean"),("calibration_bias_mean","median")):
  a=runs[col].dropna().to_numpy(float)
  vals=[]
  for _ in range(n):
   s=rng.choice(a,len(a),replace=True); vals.append(np.median(s) if stat=="median" else np.mean(s))
  out[col]={"statistic":stat,"support":len(a),"low95":float(np.quantile(vals,.025)),"high95":float(np.quantile(vals,.975))}
 return out
