#!/usr/bin/env python3
"""R004 V3+V4 offline prediction-vs-final-observed evaluator."""
from __future__ import annotations
import argparse, csv, hashlib, json, math
from pathlib import Path
import numpy as np

METRICS=("free_precision","free_recall","free_iou","occupied_precision","occupied_recall","occupied_iou","macro_iou")
RUNS=tuple(f"mpx_{i:03d}" for i in range(1,11))

def div(a,b): return float(a/b) if b else math.nan
def metrics(pred, truth):
    pred=np.asarray(pred)>0.5; truth=np.asarray(truth,dtype=bool)
    po,pf=pred,~pred; to,tf=truth,~truth
    tp=int(np.sum(po&to)); tn=int(np.sum(pf&tf)); fp=int(np.sum(po&tf)); fn=int(np.sum(pf&to))
    oi=div(tp,tp+fp+fn); fi=div(tn,tn+fp+fn)
    return dict(free_precision=div(tn,tn+fn),free_recall=div(tn,tn+fp),free_iou=fi,
      occupied_precision=div(tp,tp+fp),occupied_recall=div(tp,tp+fn),occupied_iou=oi,
      macro_iou=(fi+oi)/2 if math.isfinite(fi) and math.isfinite(oi) else math.nan)

def seed(kind,run,decision,j):
    return int.from_bytes(hashlib.sha256(f"R004-{kind}|{run}|{decision}|{j}".encode()).digest()[:8],"big")
def progress(k,n): return (k-1)/(n-1) if n>=2 else math.nan
def pbin(x):
    if not math.isfinite(x): return ""
    return ("[0.00,0.25)" if x<.25 else "[0.25,0.50)" if x<.5 else "[0.50,0.75)" if x<.75 else "[0.75,1.00]")

def read_csv(p):
    with p.open(newline="",encoding="utf-8") as f:return list(csv.DictReader(f))
def scalar(z,k): return np.asarray(z[k]).reshape(()).item()
def final_snapshot(run):
    rows=read_csv(run/"snapshots.csv"); finals=[(i,r) for i,r in enumerate(rows) if r.get("event","").lower()=="final"]
    if len(finals)!=1: raise ValueError(f"{run.name}: expected one final row, got {len(finals)}")
    i,r=finals[0]; p=run/r["canvas_map_file"]
    fallback=False
    if not p.is_file():
        valid=[]
        for j,x in enumerate(rows[:i]):
            q=run/x.get("canvas_map_file","")
            if q.is_file(): valid.append((j,x,q))
        if not valid: raise ValueError(f"{run.name}: no defensible final observed snapshot")
        j,r,p=valid[-1]; fallback=True
    with np.load(p) as z:return np.asarray(z["data"],dtype=np.int16),str(p.relative_to(run)),fallback

def prediction_canvas(run,row,shape):
    with np.load(run/row["raw_map"]) as z:
        raw=np.asarray(z["data"]); rr=float(scalar(z,"resolution")); rx=float(scalar(z,"origin_x")); ry=float(scalar(z,"origin_y"))
    with np.load(run/row["canvas_map"]) as z:
        obs=np.asarray(z["data"],dtype=np.int16); cr=float(scalar(z,"resolution")); cx=float(scalar(z,"origin_x")); cy=float(scalar(z,"origin_y"))
    with np.load(run/row["mean_map"]) as z:
        a=np.asarray(z["data"],dtype=np.float32); h=int(scalar(z,"source_height")); w=int(scalar(z,"source_width")); top=int(scalar(z,"pad_top")); left=int(scalar(z,"pad_left"))
        if (h,w)!=raw.shape: raise ValueError("prediction/raw shape mismatch")
        pred=a[top:top+h,left:left+w]
    ratio=round(rr/cr)
    if ratio<1 or not math.isclose(rr/cr,ratio,abs_tol=1e-6): raise ValueError("incompatible grids")
    expanded=np.repeat(np.repeat(pred,ratio,0),ratio,1); out=np.full(shape,np.nan,np.float32); support=np.zeros(shape,bool)
    r0=round((ry-cy)/cr); c0=round((rx-cx)/cr); sr=max(0,-r0); sc=max(0,-c0); dr=max(0,r0); dc=max(0,c0)
    nr=min(expanded.shape[0]-sr,shape[0]-dr); nc=min(expanded.shape[1]-sc,shape[1]-dc)
    if nr>0 and nc>0: out[dr:dr+nr,dc:dc+nc]=expanded[sr:sr+nr,sc:sc+nc]; support[dr:dr+nr,dc:dc+nc]=True
    return obs,out,support

def analyze_run(run):
    final,final_file,fallback=final_snapshot(run); table=read_csv(run/"decisions.csv"); rows=[]; samples=[]
    for k,d in enumerate(table,1):
        obs,pred,support=prediction_canvas(run,d,final.shape); F=(obs<0)&(final>=0); E=F&support; truth=final>0
        fc=int(F.sum()); ec=int(E.sum()); ff=int(np.sum(F&~truth)); fo=int(np.sum(F&truth)); ef=int(np.sum(E&~truth)); eo=int(np.sum(E&truth))
        rec=dict(run_id=run.name,decision_id=int(d["decision_id"]),decision_index=k,decision_count=len(table),decision_progress=progress(k,len(table)),progress_bin=pbin(progress(k,len(table))),
          F_count=fc,E_count=ec,unsupported_count=fc-ec,support_coverage=div(ec,fc),free_support_coverage=div(ef,ff),occupied_support_coverage=div(eo,fo),
          final_snapshot=final_file,fallback_final_snapshot=int(fallback),GT_free_count=ef,GT_occupied_count=eo,predicted_free_count=int(np.sum(E&(pred<=.5))),predicted_occupied_count=int(np.sum(E&(pred>.5))))
        rec.update(metrics(pred[E],truth[E]) if ec else {m:math.nan for m in METRICS}); rows.append(rec); samples.append((rec,pred[E],truth[E]))
    return rows,samples

def write_csv(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else []); w.writeheader(); w.writerows(rows)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("data_root",type=Path); ap.add_argument("--output",type=Path,required=True); ap.add_argument("--inventory-only",action="store_true"); a=ap.parse_args()
    inventory=[]; allrows=[]
    for rid in RUNS:
        run=a.data_root/"mapex_lab/experiments/mapex"/rid
        rows,samples=analyze_run(run); inventory.append(dict(run_id=rid,decisions=len(rows),partial_support=sum(r["unsupported_count"]>0 for r in rows)))
        allrows.extend(rows)
    if a.inventory_only: print(json.dumps(inventory,indent=2)); return
    out=a.output; write_csv(out/"prediction_vs_final_observed_decisions.csv",allrows)
    runrows=[]
    for rid in RUNS:
        rr=[x for x in allrows if x["run_id"]==rid]; q=dict(run_id=rid,decisions=len(rr),partial_support_decisions=sum(x["unsupported_count"]>0 for x in rr))
        for m in ("support_coverage","free_support_coverage","occupied_support_coverage")+METRICS:
            v=np.array([x[m] for x in rr],float); v=v[np.isfinite(v)]; q[m+"_mean"]=float(v.mean()) if v.size else math.nan; q[m+"_n"]=int(v.size)
        runrows.append(q)
    write_csv(out/"prediction_vs_final_observed_runs.csv",runrows)
    summary={"schema":"r004_prediction_vs_final_observed_v4","runs":len(runrows),"decisions":len(allrows),"partial_support_decisions":sum(x["unsupported_count"]>0 for x in allrows),"F_count":sum(x["F_count"] for x in allrows),"E_count":sum(x["E_count"] for x in allrows),"unsupported_count":sum(x["unsupported_count"] for x in allrows),"method":"V3+V4","threshold":"occupied iff p > 0.5"}
    (out/"prediction_vs_final_observed_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    write_csv(out/"partial_zero_support_inventory.csv",[x for x in allrows if x["unsupported_count"]>0 or x["E_count"]==0])
    print(json.dumps(summary,indent=2))
if __name__=="__main__": main()
