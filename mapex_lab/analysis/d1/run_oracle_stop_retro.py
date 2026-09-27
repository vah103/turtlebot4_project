#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,re,subprocess
from pathlib import Path
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import pandas as pd
from oracle_stop_retro import *

def git_blob(repo:Path,commit:str,path:str)->str:
    return subprocess.check_output(["git","rev-parse",f"{commit}:{path}"],cwd=repo,text=True).strip()

def main():
    ap=argparse.ArgumentParser(); ap.add_argument("--data-root",type=Path,required=True); ap.add_argument("--r004-reference",type=Path,required=True); ap.add_argument("--output",type=Path,required=True); ap.add_argument("--runs",default=",".join(RUNS)); ap.add_argument("--skip-tests",action="store_true"); a=ap.parse_args()
    root=Path(__file__).resolve().parents[3]; out=a.output; out.mkdir(parents=True,exist_ok=True)
    if not a.skip_tests:
        p=subprocess.run(["python3","-m","unittest","-v","test_oracle_stop_retro.py"],cwd=Path(__file__).parent,
                         stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        stable_log=re.sub(r"Ran (\d+) tests? in [0-9.]+s",r"Ran \1 tests",p.stdout)
        (out/"tests.log").write_text(stable_log,encoding="utf-8")
        if p.returncode: return p.returncode
    identities={
      "technical_base":TECHNICAL_BASE,
      "executed_implementation":EXECUTED_IMPLEMENTATION,
      "delivery_revision":DELIVERY_REVISION,
      "gt_blob":git_blob(root,"HEAD","mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v2.npz"),
      "gt_summary_blob":git_blob(root,"HEAD","mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v2_summary.json"),
      "gate_p_blob":git_blob(root,"HEAD","mapex_lab/analysis/d1/d1_gate_p.py"),
      "shared_blob":git_blob(root,"HEAD","mapex_lab/analysis/d1/results/shared_phase0_evidence_v1/shared_phase0_evidence.csv"),
      "r004_reference":subprocess.check_output(["git","rev-parse","HEAD"],cwd=a.r004_reference,text=True).strip(),
    }
    expected={"gt_blob":GT_BLOB,"gt_summary_blob":GT_SUMMARY_BLOB,"gate_p_blob":GATE_P_BLOB,"shared_blob":SHARED_BLOB,"r004_reference":R004_REFERENCE}
    for k,v in expected.items():
        if identities[k]!=v: raise RuntimeError(f"identity mismatch {k}: {identities[k]} != {v}")
    shared_path=root/"mapex_lab/analysis/d1/results/shared_phase0_evidence_v1/shared_phase0_evidence.csv"; shared=pd.read_csv(shared_path,low_memory=False); validate_universe(shared)
    selected=tuple(x for x in a.runs.split(",") if x); shared=shared[shared.run_id.isin(selected)].copy()
    raw_fingerprint_before=raw_input_fingerprint(shared,a.data_root)
    gt=load_structural_gt(root/"mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v2.npz"); topo,helper=load_r004_helper(a.r004_reference); universe=structural_universe(gt,topo)
    decisions=add_qualification_fields(score_decisions(shared,a.data_root,gt,universe)); runs,summary=summarize_runs(decisions); around=signal_around(decisions,runs)
    decisions.to_csv(out/"oracle_decisions.csv",index=False); runs.to_csv(out/"oracle_runs.csv",index=False); around.to_csv(out/"signal_around_oracle.csv",index=False)
    raw_fingerprint_after=raw_input_fingerprint(shared,a.data_root)
    if raw_fingerprint_after!=raw_fingerprint_before: raise RuntimeError("raw inputs changed during execution")
    identities["raw_input_fingerprint"]=raw_fingerprint_before
    summary.update({"identities":identities,"data_root":str(a.data_root),"structural_universe_cells":int(universe.sum()),"structural_universe_m2":float(universe.sum()*GT_RESOLUTION**2),"selected_runs":list(selected),"no_input_mutation":True})
    json_dump(summary,out/"oracle_summary.json")
    fig,ax=plt.subplots(figsize=(10,6))
    marker_by_tolerance={1:"o",5:"s",10:"^"}
    for rid,g in decisions.groupby("run_id"):
        line,=ax.plot(g.decision_progress,g.OracleRemainingFraction_GT,label=rid,alpha=.8)
        rr=runs[runs.run_id==rid].iloc[0]
        for p in TOLERANCES:
            decision=rr[f"oracle_{p}_decision"]
            if pd.notna(decision):
                point=g[g.decision_id==int(decision)].iloc[0]
                ax.scatter(point.decision_progress,point.OracleRemainingFraction_GT,marker=marker_by_tolerance[p],s=34,
                           color=line.get_color(),edgecolor="black",linewidth=.35,zorder=4)
    for p in TOLERANCES: ax.axhline(p/100,ls="--",lw=.8)
    handles,labels=ax.get_legend_handles_labels()
    handles.extend(Line2D([],[],color="black",marker=marker_by_tolerance[p],linestyle="None",label=f"OracleStop_{p}") for p in TOLERANCES)
    labels.extend(f"OracleStop_{p}" for p in TOLERANCES)
    ax.set(xlabel="Decision progress",ylabel="True remaining reachable fraction"); ax.legend(handles,labels,ncol=2,fontsize=7); fig.tight_layout(); fig.savefig(out/"remaining_fraction_traces.png",dpi=160); plt.close(fig)
    fig,ax=plt.subplots(figsize=(10,5)); x=range(len(runs)); w=.24
    for j,p in enumerate(TOLERANCES): ax.bar([i+(j-1)*w for i in x],runs[f"oracle_{p}_fraction_saved"],w,label=f"{p}%")
    ax.set_xticks(list(x),runs.run_id,rotation=45); ax.set_ylabel("Fraction saved"); ax.legend(); fig.tight_layout(); fig.savefig(out/"oracle_savings.png",dpi=160); plt.close(fig)
    signals=("U_p95","U_mean","U_disagreement","A_mean_m2","RemainingFraction"); fig,axes=plt.subplots(len(signals),1,figsize=(9,12),sharex=True)
    for ax,s in zip(axes,signals):
        for rid,g in around.groupby("run_id"): ax.plot(g.offset,pd.to_numeric(g[s],errors="coerce"),marker="o",alpha=.6); ax.set_ylabel(s)
    axes[-1].set_xlabel("Decision offset from OracleStop_5"); fig.tight_layout(); fig.savefig(out/"signals_around_oracle5.png",dpi=160); plt.close(fig)
    artifacts={p.name:sha256_file(p) for p in sorted(out.iterdir()) if p.is_file() and p.name!="artifact_manifest.json"}; json_dump({"identities":identities,"artifacts":artifacts},out/"artifact_manifest.json")
    return 0
if __name__=="__main__": raise SystemExit(main())
