#!/usr/bin/env python3
"""R004 V3+V4 offline prediction-vs-final-observed evaluator."""
from __future__ import annotations
import argparse, csv, hashlib, json, math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.stats import spearmanr

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
    final_known=int(np.sum(final>=0))
    for k,d in enumerate(table,1):
        obs,pred,support=prediction_canvas(run,d,final.shape); F=(obs<0)&(final>=0); E=F&support; truth=final>0
        fc=int(F.sum()); ec=int(E.sum()); ff=int(np.sum(F&~truth)); fo=int(np.sum(F&truth)); ef=int(np.sum(E&~truth)); eo=int(np.sum(E&truth))
        rec=dict(run_id=run.name,decision_id=int(d["decision_id"]),decision_index=k,decision_count=len(table),decision_progress=progress(k,len(table)),progress_bin=pbin(progress(k,len(table))),
          F_count=fc,E_count=ec,unsupported_count=fc-ec,support_coverage=div(ec,fc),free_support_coverage=div(ef,ff),occupied_support_coverage=div(eo,fo),
          F_free_count=ff,F_occupied_count=fo,E_free_count=ef,E_occupied_count=eo,F_occupied_fraction=div(fo,fc),E_occupied_fraction=div(eo,ec),
          known_fraction_final=1-div(fc,final_known),final_snapshot=final_file,fallback_final_snapshot=int(fallback),GT_free_count=ef,GT_occupied_count=eo,predicted_free_count=int(np.sum(E&(pred<=.5))),predicted_occupied_count=int(np.sum(E&(pred>.5))))
        rec.update(metrics(pred[E],truth[E]) if ec else {m:math.nan for m in METRICS}); rows.append(rec); samples.append((rec,pred[E],truth[E]))
    return rows,samples

def write_csv(path,rows):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else []); w.writeheader(); w.writerows(rows)

def finite(values):
    a=np.asarray(values,dtype=float); return a[np.isfinite(a)]

def summarize(values):
    a=finite(values)
    return dict(mean=float(np.mean(a)) if a.size else math.nan,std=float(np.std(a,ddof=1)) if a.size>1 else (0.0 if a.size else math.nan),median=float(np.median(a)) if a.size else math.nan,n=int(a.size))

def corr(x,y):
    a=np.asarray(x,float); b=np.asarray(y,float); ok=np.isfinite(a)&np.isfinite(b)
    if np.sum(ok)<2 or len(np.unique(a[ok]))<2 or len(np.unique(b[ok]))<2:return math.nan
    return float(spearmanr(a[ok],b[ok]).statistic)

def sensitivity_rows(run_id,samples,kind):
    eligible=[s for s in samples if len(s[1])>0]
    if kind=="total":
        target=max(1,int(math.floor(np.percentile([len(s[1]) for s in eligible],25)))) if eligible else 0
        ft=ot=0
    else:
        both=[s for s in eligible if np.any(~s[2]) and np.any(s[2])]
        ft=max(1,int(math.floor(np.percentile([np.sum(~s[2]) for s in both],25)))) if both else 0
        ot=max(1,int(math.floor(np.percentile([np.sum(s[2]) for s in both],25)))) if both else 0
        target=ft+ot
    out=[]
    for rec,pred,truth in samples:
        if kind=="total": ok=target>0 and len(pred)>=target
        else: ok=ft>0 and ot>0 and np.sum(~truth)>=ft and np.sum(truth)>=ot
        if not ok: continue
        reps={m:[] for m in METRICS}
        for j in range(100):
            rng=np.random.default_rng(seed(kind,run_id,rec["decision_id"],j))
            if kind=="total": idx=rng.choice(len(pred),target,replace=False)
            else:
                fi=rng.choice(np.flatnonzero(~truth),ft,replace=False); oi=rng.choice(np.flatnonzero(truth),ot,replace=False); idx=np.concatenate([fi,oi])
            mm=metrics(pred[idx],truth[idx])
            for m in METRICS:
                if math.isfinite(mm[m]): reps[m].append(mm[m])
        row={k:rec[k] for k in ("run_id","decision_id","decision_progress","progress_bin","F_count","E_count","support_coverage")}
        row.update(sensitivity=kind,sample_size=target,sample_free=ft,sample_occupied=ot,replicates=100)
        for m,v in reps.items():
            a=np.asarray(v,float); row[m+"_mean"]=float(np.mean(a)) if a.size else math.nan; row[m+"_p2_5"]=float(np.percentile(a,2.5)) if a.size else math.nan; row[m+"_p97_5"]=float(np.percentile(a,97.5)) if a.size else math.nan; row[m+"_n"]=int(a.size)
        out.append(row)
    return out

def bin_summaries(rows):
    bins=("[0.00,0.25)","[0.25,0.50)","[0.50,0.75)","[0.75,1.00]")
    metrics_all=("support_coverage","free_support_coverage","occupied_support_coverage")+METRICS
    per=[]
    for rid in RUNS:
        rr=[x for x in rows if x["run_id"]==rid]
        for b in bins:
            br=[x for x in rr if x["progress_bin"]==b]
            q={"run_id":rid,"progress_bin":b,"decision_count":len(br)}
            for m in metrics_all:
                a=finite([x[m] for x in br]); q[m+"_median"]=float(np.median(a)) if a.size else math.nan; q[m+"_n"]=int(a.size)
            per.append(q)
    macro=[]
    for b in bins:
        rr=[x for x in per if x["progress_bin"]==b]; q={"progress_bin":b}
        for m in metrics_all:
            s=summarize([x[m+"_median"] for x in rr]); q.update({m+"_"+k:v for k,v in s.items()})
        macro.append(q)
    return per,macro

def sensitivity_bin_summary(rows):
    per=[]
    for kind in ("total","class"):
        kr=[x for x in rows if x["sensitivity"]==kind]
        for rid in RUNS:
            for b in ("[0.00,0.25)","[0.25,0.50)","[0.50,0.75)","[0.75,1.00]"):
                br=[x for x in kr if x["run_id"]==rid and x["progress_bin"]==b]; q={"sensitivity":kind,"run_id":rid,"progress_bin":b,"eligible_decisions":len(br)}
                for m in METRICS:
                    a=finite([x[m+"_mean"] for x in br]); q[m+"_median"]=float(np.median(a)) if a.size else math.nan
                per.append(q)
    out=[]
    for kind in ("total","class"):
        for b in ("[0.00,0.25)","[0.25,0.50)","[0.50,0.75)","[0.75,1.00]"):
            rr=[x for x in per if x["sensitivity"]==kind and x["progress_bin"]==b]; q={"sensitivity":kind,"progress_bin":b,"eligible_decisions":sum(x["eligible_decisions"] for x in rr),"contributing_runs":sum(x["eligible_decisions"]>0 for x in rr)}
            for m in METRICS:
                s=summarize([x[m+"_median"] for x in rr]); q.update({m+"_"+k:v for k,v in s.items()})
            out.append(q)
    return per,out

def plot_lines(rows,out,macrobin):
    plots=out/"figures"; plots.mkdir(parents=True,exist_ok=True)
    def one(name,ys,title,ylabel):
        fig,ax=plt.subplots(figsize=(8,5))
        for rid in RUNS:
            rr=[x for x in rows if x["run_id"]==rid]; x=[z["decision_progress"] for z in rr]
            for y,label,style in ys: ax.plot(x,[z[y] for z in rr],style,alpha=.5,label=(label if rid==RUNS[0] else None))
        ax.set(xlabel="Normalized decision progress",ylabel=ylabel,title=title); ax.grid(alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(plots/name,dpi=160); plt.close(fig)
    for rid in RUNS:
        rr=[x for x in rows if x["run_id"]==rid]; fig,ax=plt.subplots(figsize=(7,4)); x=[z["decision_progress"] for z in rr]
        ax.plot(x,[z["free_iou"] for z in rr],label="Free IoU"); ax.plot(x,[z["occupied_iou"] for z in rr],label="Occupied IoU"); ax.plot(x,[z["macro_iou"] for z in rr],label="Macro IoU")
        ax.set(xlabel="Normalized decision progress",ylabel="IoU",title=f"{rid} conditional quality"); ax.grid(alpha=.25); ax.legend(); fig.tight_layout(); fig.savefig(plots/f"{rid}_quality_vs_progress.png",dpi=160); plt.close(fig)
    fig,ax=plt.subplots(figsize=(7,4)); x=np.arange(len(macrobin)); y=[z["macro_iou_mean"] for z in macrobin]; e=[z["macro_iou_std"] for z in macrobin]
    ax.errorbar(x,y,yerr=e,marker="o",capsize=4); ax.set_xticks(x,[z["progress_bin"] for z in macrobin],rotation=15); ax.set(xlabel="Normalized decision-progress bin",ylabel="Run-macro IoU mean ± std",title="10-run macro conditional quality"); ax.grid(alpha=.25); fig.tight_layout(); fig.savefig(plots/"run_macro_quality_vs_progress.png",dpi=160); plt.close(fig)
    one("support_coverage_vs_progress.png",[("support_coverage","Overall","-"),("free_support_coverage","Free","--"),("occupied_support_coverage","Occupied",":")],"Prediction support coverage","Coverage")
    one("unsupported_fraction_vs_progress.png",[("support_coverage","Support coverage","-")],"Support coverage (unsupported = 1 - curve)","Coverage")
    one("support_size_vs_progress.png",[("E_count","Scoreable E","-"),("F_count","Target F","--")],"Support size","Cells")
    one("class_balance_vs_progress.png",[("E_occupied_fraction","Occupied fraction E","-"),("F_occupied_fraction","Occupied fraction F","--")],"Class balance","Occupied fraction")
    one("free_iou_vs_progress.png",[("free_iou","Free IoU","-")],"Free IoU","IoU")
    one("occupied_iou_vs_progress.png",[("occupied_iou","Occupied IoU","-")],"Occupied IoU","IoU")

def overlays(data_root,out):
    od=out/"overlays"; od.mkdir(parents=True,exist_ok=True); records=[]
    for rid in RUNS:
        run=data_root/"mapex_lab/experiments/mapex"/rid; final,_,_=final_snapshot(run); table=read_csv(run/"decisions.csv"); n=len(table)
        for label,k in (("early",1),("middle",1+round((n-1)*.5)),("late",n)):
            d=table[k-1]; obs,pred,support=prediction_canvas(run,d,final.shape); F=(obs<0)&(final>=0); E=F&support; truth=final>0; cls=pred>.5
            img=np.ones((*final.shape,3),np.float32); img[obs>=0]=(.75,.75,.75); img[F&~support]=(.2,.45,.95); img[E&(cls==truth)]=(.2,.75,.25); img[E&(cls!=truth)]=(.9,.2,.2)
            visible=F|(obs>=0); ys,xs=np.nonzero(visible)
            if ys.size:
                margin=20; r0=max(0,int(ys.min())-margin); r1=min(img.shape[0],int(ys.max())+margin+1); c0=max(0,int(xs.min())-margin); c1=min(img.shape[1],int(xs.max())+margin+1); img=img[r0:r1,c0:c1]
            p=od/f"{rid}_{label}_decision_{int(d['decision_id']):03d}.png"; plt.imsave(p,img)
            records.append(dict(run_id=rid,position=label,decision_id=int(d["decision_id"]),decision_progress=progress(k,n),file=str(p.relative_to(out))))
    write_csv(out/"overlay_manifest.csv",records)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("data_root",type=Path); ap.add_argument("--output",type=Path,required=True); ap.add_argument("--inventory-only",action="store_true"); a=ap.parse_args()
    inventory=[]; allrows=[]; sample_map={}
    for rid in RUNS:
        run=a.data_root/"mapex_lab/experiments/mapex"/rid
        rows,samples=analyze_run(run); inventory.append(dict(run_id=rid,decisions=len(rows),partial_support=sum(r["unsupported_count"]>0 for r in rows)))
        allrows.extend(rows); sample_map[rid]=samples
    if a.inventory_only: print(json.dumps(inventory,indent=2)); return
    out=a.output; write_csv(out/"prediction_vs_final_observed_decisions.csv",allrows)
    runrows=[]
    for rid in RUNS:
        rr=[x for x in allrows if x["run_id"]==rid]; q=dict(run_id=rid,decisions=len(rr),partial_support_decisions=sum(x["unsupported_count"]>0 for x in rr))
        for m in ("support_coverage","free_support_coverage","occupied_support_coverage")+METRICS:
            v=np.array([x[m] for x in rr],float); v=v[np.isfinite(v)]; q[m+"_mean"]=float(v.mean()) if v.size else math.nan; q[m+"_n"]=int(v.size)
        q.update(spearman_macro_iou_vs_progress=corr([x["decision_progress"] for x in rr],[x["macro_iou"] for x in rr]),spearman_macro_iou_vs_log_support=corr([math.log(max(1,x["E_count"])) for x in rr],[x["macro_iou"] for x in rr]),spearman_macro_iou_vs_occupied_fraction=corr([x["E_occupied_fraction"] for x in rr],[x["macro_iou"] for x in rr]))
        runrows.append(q)
    write_csv(out/"prediction_vs_final_observed_runs.csv",runrows)
    perbin,macrobin=bin_summaries(allrows); write_csv(out/"run_progress_bin_medians.csv",perbin); write_csv(out/"run_macro_progress_bins.csv",macrobin)
    total=[]; composition=[]
    for rid in RUNS:
        total.extend(sensitivity_rows(rid,sample_map[rid],"total")); composition.extend(sensitivity_rows(rid,sample_map[rid],"class"))
    write_csv(out/"total_support_sensitivity.csv",total); write_csv(out/"class_composition_sensitivity.csv",composition)
    sper,smacro=sensitivity_bin_summary(total+composition); write_csv(out/"sensitivity_run_bin_medians.csv",sper); write_csv(out/"sensitivity_progress_bins.csv",smacro)
    summary={"schema":"r004_prediction_vs_final_observed_v4","runs":len(runrows),"included_runs":list(RUNS),"excluded_runs":[],"decisions":len(allrows),"partial_support_decisions":sum(x["unsupported_count"]>0 for x in allrows),"zero_support_decisions":sum(x["E_count"]==0 and x["F_count"]>0 for x in allrows),"F_count":sum(x["F_count"] for x in allrows),"E_count":sum(x["E_count"] for x in allrows),"unsupported_count":sum(x["unsupported_count"] for x in allrows),"support_coverage_pooled":div(sum(x["E_count"] for x in allrows),sum(x["F_count"] for x in allrows)),"method":"V3+V4","threshold":"occupied iff p > 0.5","total_sensitivity_rows":len(total),"class_sensitivity_rows":len(composition)}
    (out/"prediction_vs_final_observed_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    write_csv(out/"partial_zero_support_inventory.csv",[x for x in allrows if x["unsupported_count"]>0 or x["E_count"]==0])
    provenance={"schema":"r004_provenance_v1","data_root":str(a.data_root.resolve()),"runs":list(RUNS),"final_snapshot_fallbacks":[x["run_id"] for x in runrows if any(z["fallback_final_snapshot"] for z in allrows if z["run_id"]==x["run_id"])],"exclusions":[],"method":"R004 V3 + accepted V4 support amendment","prediction_support":"artifact geometry only","seed":"SHA256 deterministic","resamples":100,"overlay_legend":{"gray":"already observed","blue":"future-observed target without prediction support","green":"supported correct class","red":"supported incorrect class","white":"outside displayed evidence"}}
    (out/"provenance_exclusions_fallbacks.json").write_text(json.dumps(provenance,indent=2)+"\n")
    plot_lines(allrows,out,macrobin); overlays(a.data_root,out)
    print(json.dumps(summary,indent=2))
if __name__=="__main__": main()
