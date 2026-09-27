#!/usr/bin/env python3
from __future__ import annotations
import argparse,json,re,subprocess
from pathlib import Path
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np,pandas as pd
from gate_r_v2 import *

ORACLE_BLOB='9ae8f0479cf37a2b6aa9426a158f120efdc655cf'
SHARED_BLOB='3e80124d545ea37b2791594dd775a7d953368bea'
START_HEAD='4f674a81add0c14eb15cc0ab95754130c5923ead'
METHOD='c883b0ed1e335cad08036d658d305f721ac56429'
def blob(root,p):return subprocess.check_output(['git','rev-parse',f'{START_HEAD}:{p}'],cwd=root,text=True).strip()
def dump(x,p):p.write_text(json.dumps(x,indent=2,sort_keys=True,allow_nan=False)+'\n')
def clean(x):
 if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
 if isinstance(x,list):return [clean(v) for v in x]
 if isinstance(x,(float,np.floating)) and not np.isfinite(x):return None
 if isinstance(x,(np.integer,np.bool_)):return x.item()
 return x

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--held-out');ap.add_argument('--skip-tests',action='store_true');a=ap.parse_args()
 root=Path(__file__).resolve().parents[3];out=a.output;out.mkdir(parents=True,exist_ok=True)
 if not a.skip_tests:
  q=subprocess.run(['python3','-m','unittest','-v','test_gate_r_v2.py'],cwd=Path(__file__).parent,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
  (out/'tests.log').write_text(re.sub(r'Ran (\d+) tests? in [0-9.]+s',r'Ran \1 tests',q.stdout));
  if q.returncode:return q.returncode
 op='mapex_lab/analysis/d1/results/oracle_stop_retro_v1/oracle_decisions.csv';sp='mapex_lab/analysis/d1/results/shared_phase0_evidence_v1/shared_phase0_evidence.csv'
 identities={'accepted_oracle_head':START_HEAD,'method_commit':METHOD,'oracle_decisions_blob':blob(root,op),'shared_evidence_blob':blob(root,sp)}
 if identities['oracle_decisions_blob']!=ORACLE_BLOB or identities['shared_evidence_blob']!=SHARED_BLOB:raise RuntimeError('accepted input identity mismatch')
 oracle=pd.read_csv(root/op,low_memory=False);shared=pd.read_csv(root/sp,low_memory=False);validate(oracle);validate(shared)
 truth=['run_id','decision_id','decision_index','decision_count','decision_progress','progress_bin','N_GT','A_true_remaining_m2','OracleRemainingFraction_GT','oracle_truth_evaluable','oracle_truth_reason']
 cand=['run_id','decision_id','A_mean_m2','RemainingFraction','d1_runtime_region_evaluable','d1_reason','known_fraction_final','F_count','runtime_resolution','macro_iou','non_evaluable_reasons']
 frame=oracle[truth].merge(shared[cand],on=['run_id','decision_id'],how='outer',validate='one_to_one',indicator=True)
 if not frame._merge.eq('both').all():raise RuntimeError('join mismatch')
 frame=frame.drop(columns='_merge');validate(frame)
 input_hash={'oracle_sha256':sha256(root/op),'shared_sha256':sha256(root/sp)}
 selected_runs=list(RUNS if not a.held_out else (a.held_out,)); fold_rows=[];run_rows=[];decision_rows=[];risk=[];oracle_cp=[]
 for held_id in selected_runs:
  dev=[x for x in RUNS if x!=held_id];details=[development_candidate(frame,dev,n) for n in CANDIDATES];chosen=choose(details)
  for d in details:d.update({'held_out_run':held_id,'selected':d['candidate']==chosen});fold_rows.append(d)
  if chosen is None:
   run_rows.append({'held_out_run':held_id,'development_runs':';'.join(dev),'selected_candidate':'NO_ADMISSIBLE_C'})
   continue
  rr,p=heldout_metrics(frame,held_id,dev,chosen);run_rows.append(rr);c,y,_=CANDIDATES[chosen]
  base=frame[frame.run_id==held_id].copy();base['selected_candidate']=chosen;base['candidate_value']=base[c];base['paired_truth']=base[y]
  base['accepted_oracle_head']=START_HEAD;base['method_commit']=METHOD;base['oracle_decisions_blob']=ORACLE_BLOB;base['shared_evidence_blob']=SHARED_BLOB
  for col in ['C_dev_percentile','Y_dev_percentile','calibration_residual']:base[col]=np.nan
  for col in ['low_C50','low_C25','high_truth25','FC25','OracleFC10']:base[col]=pd.Series(pd.NA,index=base.index,dtype='boolean')
  for col in ['C_dev_percentile','Y_dev_percentile','low_C50','low_C25','high_truth25','FC25','OracleFC10','calibration_residual']:base.loc[p.index,col]=p[col]
  decision_rows.append(base)
  events=p[p.FC25|p.OracleFC10].copy();events['selected_candidate']=chosen;risk.append(events)
 runs=pd.DataFrame(run_rows);folds=pd.DataFrame(fold_rows);decisions=pd.concat(decision_rows,ignore_index=True) if decision_rows else pd.DataFrame()
 if a.held_out:
  folds.to_csv(out/'gate_r_folds.csv',index=False);runs.to_csv(out/'gate_r_runs.csv',index=False);decisions.to_csv(out/'gate_r_decisions.csv',index=False);return 0
 # Exact oracle row, offsets and progress-bin summaries.
 oracle_runs=pd.read_csv(root/'mapex_lab/analysis/d1/results/oracle_stop_retro_v1/oracle_runs.csv'); local=[]; bin_rows=[]
 runs['progress_bin_summaries_json']='';runs['oracle_5_decision']=np.nan
 runs['oracle_exact_candidate_evaluable']=pd.Series(pd.NA,index=runs.index,dtype='boolean')
 runs['oracle_exact_candidate_value']=np.nan;runs['oracle_exact_C_dev_percentile']=np.nan;runs['broad_future_gain_json']=''
 for _,rr in runs.iterrows():
  if rr.selected_candidate=='NO_ADMISSIBLE_C':continue
  g=decisions[decisions.run_id==rr.held_out_run].sort_values('decision_index');oid=int(oracle_runs.loc[oracle_runs.run_id==rr.held_out_run,'oracle_5_decision'].iloc[0]);pos=int(np.where(g.decision_id.to_numpy()==oid)[0][0])
  for off in range(-3,4):
   if 0<=pos+off<len(g):
    row=g.iloc[pos+off];local.append({'run_id':rr.held_out_run,'offset':off,'decision_id':int(row.decision_id),'candidate':rr.selected_candidate,'candidate_value':row.candidate_value,'C_dev_percentile':row.C_dev_percentile,'truth_m2':row.A_true_remaining_m2,'truth_fraction':row.OracleRemainingFraction_GT})
    if off==0 and pd.notna(row.C_dev_percentile):oracle_cp.append(float(row.C_dev_percentile))
  run_bin_rows=[]
  for b in BINS:
   q=g[g.progress_bin==b];valid=q.C_dev_percentile.notna();z=q[valid]
   rho=float(spearmanr(z.candidate_value,z.paired_truth).statistic) if len(z)>=5 and z.candidate_value.nunique()>1 and z.paired_truth.nunique()>1 else np.nan
   base=float(z.paired_truth.mean()) if len(z) else np.nan;low=z.C_dev_percentile<=.5;trr=float(z.loc[low,'paired_truth'].mean()/base) if low.any() and base>0 else np.nan
   br={'run_id':rr.held_out_run,'progress_bin':b,'support':len(z),'rho':rho,'TRR50':trr,'calibration_bias':float(z.calibration_residual.mean()) if len(z) else np.nan};bin_rows.append(br);run_bin_rows.append(br)
  exact=[x for x in local if x['run_id']==rr.held_out_run and x['offset']==0]
  idx=runs.index[runs.held_out_run==rr.held_out_run][0]
  runs.loc[idx,'progress_bin_summaries_json']=json.dumps(run_bin_rows,sort_keys=True,allow_nan=True)
  runs.loc[idx,'oracle_5_decision']=oid
  runs.loc[idx,'oracle_exact_candidate_evaluable']=bool(exact and pd.notna(exact[0]['candidate_value']))
  runs.loc[idx,'oracle_exact_candidate_value']=exact[0]['candidate_value'] if exact else np.nan
  runs.loc[idx,'oracle_exact_C_dev_percentile']=exact[0]['C_dev_percentile'] if exact else np.nan
 bins_df=pd.DataFrame(bin_rows);bin_summary=[]
 for b,g in bins_df.groupby('progress_bin',sort=False):
  qualifying=g[g.support>=3]
  bin_summary.append({'progress_bin':b,'contributing_runs':len(qualifying),'rho_valid_runs':int(qualifying.rho.notna().sum()),'TRR50_valid_runs':int(qualifying.TRR50.notna().sum()),'rho_median':float(qualifying.rho.median()),'TRR50_mean':float(qualifying.TRR50.mean())})
 classification,criteria=classify(runs,bin_summary,oracle_cp,int(runs.selected_candidate.eq('NO_ADMISSIBLE_C').sum()))
 # Secondary future gains (trajectory-conditioned).
 broad=[]
 future_semantics={
  'FutureCoverageGain':'1 - known_fraction_final; accepted R004 source defines known_fraction_final = 1 - F_count/final_known_count',
  'FutureObservedAreaGain':'F_count * runtime_resolution^2; F is current-unknown cells inside the final-known support',
  'FutureIoUGain':'UNAVAILABLE: accepted package macro_iou is prediction-vs-final-observed IoU, not observed-map IoU_t; V2 permits this metric only when semantically valid',
 }
 for rid,g in decisions.groupby('run_id'):
  g=g.sort_values('decision_index').copy()
  g['FutureCoverageGain']=1-g.known_fraction_final
  g['FutureObservedAreaGain']=g.F_count*g.runtime_resolution**2
  g['FutureIoUGain']=np.nan
  for label in ('FutureCoverageGain','FutureObservedAreaGain','FutureIoUGain'):decisions.loc[g.index,label]=g[label]
  for target in ('FutureCoverageGain','FutureObservedAreaGain','FutureIoUGain'):
   z=g[['candidate_value','C_dev_percentile','decision_progress',target]].dropna();low=z.C_dev_percentile<=.5;baseline=float(z[target].mean()) if len(z) else np.nan
   late=z[z.decision_progress>=.75];late_low=late.C_dev_percentile<=.5;late_base=float(late[target].mean()) if len(late) else np.nan
   broad.append({'run_id':rid,'target':target,'support':len(z),'rho':float(spearmanr(z.candidate_value,z[target]).statistic) if len(z)>=5 and z[['candidate_value',target]].nunique().min()>1 else np.nan,
    'low_C50_count':int(low.sum()),'low_C50_selective_ratio':float(z.loc[low,target].mean()/baseline) if low.any() and baseline>0 else np.nan,
    'strict_late_support':len(late),'strict_late_rho':float(spearmanr(late.candidate_value,late[target]).statistic) if len(late)>=5 and late[['candidate_value',target]].nunique().min()>1 else np.nan,
    'strict_late_low_C50_count':int(late_low.sum()),'strict_late_low_C50_ratio':float(late.loc[late_low,target].mean()/late_base) if late_low.any() and late_base>0 else np.nan})
  idx=runs.index[runs.held_out_run==rid][0];runs.loc[idx,'broad_future_gain_json']=json.dumps([x for x in broad if x['run_id']==rid],sort_keys=True,allow_nan=True)
 # Stable explicit tie-break rank and all-ten development candidate when allowed.
 folds['selection_rank']=folds.groupby('held_out_run').apply(lambda g:g.sort_values(['admissible','rho_dev_macro','TRR50_dev','FC25_dev','CalMAE_dev','candidate'],ascending=[False,False,True,True,True,False]).assign(_rank=range(1,len(g)+1))['_rank']).reset_index(level=0,drop=True)
 full_details=[development_candidate(frame,list(RUNS),n) for n in CANDIDATES]
 c_primary=choose(full_details) if classification=='HISTORICAL_NEW_ROOM_FEASIBILITY_SUPPORTED_PENDING_CONFIRMATION' else None
 # Persist tables.
 decisions.to_csv(out/'gate_r_decisions.csv',index=False);runs.to_csv(out/'gate_r_runs.csv',index=False);folds.to_csv(out/'gate_r_folds.csv',index=False)
 pd.concat(risk,ignore_index=True).to_csv(out/'gate_r_false_completeness.csv',index=False) if risk else pd.DataFrame().to_csv(out/'gate_r_false_completeness.csv',index=False)
 pd.DataFrame(local).to_csv(out/'gate_r_oracle_local.csv',index=False);bins_df.to_csv(out/'gate_r_progress_bins.csv',index=False);pd.DataFrame(broad).to_csv(out/'gate_r_future_gain.csv',index=False)
 summary={'identities':identities,**input_hash,'runs':10,'decision_rows':365,'selection_frequency':runs.selected_candidate.value_counts().to_dict(),'NO_ADMISSIBLE_C_count':int(runs.selected_candidate.eq('NO_ADMISSIBLE_C').sum()),
  'metrics':{c:clean({'support':int(runs[c].notna().sum()),'mean':runs[c].mean(),'median':runs[c].median()}) for c in ('rho','TRR50','FC25','calibration_bias_mean')},
  'oracle_local_support':len(oracle_cp),'oracle_local_median_C_dev_percentile':float(np.median(oracle_cp)) if oracle_cp else None,'progress_bins':clean(bin_summary),'future_gain_semantics':future_semantics,'broad_future_gain':clean(broad),'bootstrap_10000':bootstrap(runs),'classification_criteria':criteria,'historical_classification':classification,
  'C_primary_development':c_primary,'all_ten_candidate_details':clean(full_details),
  'limitations':['Gate U remains FAIL','No full Gate-R PASS under V2','Historical development evidence only']}
 dump(clean(summary),out/'gate_r_summary.json')
 # Seven mandatory descriptive figures.
 fig,ax=plt.subplots(figsize=(9,5));
 for rid,g in decisions.groupby('run_id'):ax.plot(g.decision_progress,g.candidate_value,label=rid);ax.plot(g.decision_progress,g.paired_truth,ls='--',alpha=.5)
 ax.legend(ncol=2,fontsize=6);ax.set_title('Selected candidate (solid) vs paired structural truth (dashed)');fig.tight_layout();fig.savefig(out/'selected_traces.png',dpi=150);plt.close(fig)
 specs=[('scatter.png','candidate_value','paired_truth','Held-out predicted vs truth')]
 for fn,x,y,title in specs:
  fig,ax=plt.subplots(figsize=(7,5));
  for rid,g in decisions.groupby('run_id'):ax.scatter(g[x],g[y],s=10,alpha=.55,label=rid)
  ax.set(xlabel=x,ylabel=y,title=title);fig.tight_layout();fig.savefig(out/fn,dpi=150);plt.close(fig)
 # True selective 25/50/75 curve (equal-run means).
 curve=[]
 for q in (.25,.5,.75):
  vals=[]
  for _,g in decisions.groupby('run_id'):
   z=g[g.C_dev_percentile.notna()];base=z.paired_truth.mean();low=z.C_dev_percentile<=q
   if low.any() and base>0:vals.append(z.loc[low,'paired_truth'].mean()/base)
  curve.append(np.mean(vals) if vals else np.nan)
 fig,ax=plt.subplots(figsize=(6,4));ax.plot([25,50,75],curve,marker='o');ax.set(xlabel='Low-C percentile threshold',ylabel='Equal-run selective true-remaining ratio',title='TRR25 / TRR50 / TRR75');fig.tight_layout();fig.savefig(out/'selective_curve.png',dpi=150);plt.close(fig)
 # Explicit FC25 / OracleFC10 inventory.
 inv=decisions.groupby('run_id')[['FC25','OracleFC10']].sum().reindex(RUNS).fillna(0)
 fig,ax=plt.subplots(figsize=(8,4));x=np.arange(10);ax.bar(x-.18,inv.FC25,.36,label='FC25');ax.bar(x+.18,inv.OracleFC10,.36,label='OracleFC10');ax.set_xticks(x,RUNS,rotation=45);ax.set_ylabel('Event count');ax.legend();fig.tight_layout();fig.savefig(out/'false_completeness.png',dpi=150);plt.close(fig)
 # Aggregated progress-bin rho/TRR using the exact >=3-pair qualifying set.
 bs=pd.DataFrame(bin_summary);fig,ax=plt.subplots(figsize=(8,4));x=np.arange(len(bs));ax.bar(x-.18,bs.rho_median,.36,label='median rho');ax.bar(x+.18,bs.TRR50_mean,.36,label='mean TRR50');ax.set_xticks(x,bs.progress_bin,rotation=25);ax.legend();fig.tight_layout();fig.savefig(out/'progress_bins.png',dpi=150);plt.close(fig)
 # Exact OracleStop_5 +/-3 diagnostics.
 loc=pd.DataFrame(local);fig,ax=plt.subplots(figsize=(7,4));
 for rid,g in loc.groupby('run_id'):ax.plot(g.offset,g.C_dev_percentile,marker='o',label=rid)
 ax.axvline(0,color='black',ls='--');ax.set(xlabel='Offset from exact OracleStop_5',ylabel='C_dev_percentile',title='OracleStop_5 local diagnostic');ax.legend(ncol=2,fontsize=6);fig.tight_layout();fig.savefig(out/'oracle_local.png',dpi=150);plt.close(fig)
 # Full secondary future-gain diagnostics: coverage and observed area; IoU explicitly unavailable.
 fig,axes=plt.subplots(1,3,figsize=(14,4));
 for ax,target in zip(axes,('FutureCoverageGain','FutureObservedAreaGain','FutureIoUGain')):
  for rid,g in decisions.groupby('run_id'):ax.scatter(g.candidate_value,g[target],s=9,alpha=.5)
  ax.set(xlabel='selected candidate',ylabel=target,title=target if target!='FutureIoUGain' else 'FutureIoUGain unavailable (invalid source semantics)')
 fig.tight_layout();fig.savefig(out/'future_gain.png',dpi=150);plt.close(fig)
 input_hash_after={'oracle_sha256':sha256(root/op),'shared_sha256':sha256(root/sp)}
 if input_hash_after!=input_hash:raise RuntimeError('input mutation detected')
 artifacts={p.name:sha256(p) for p in sorted(out.iterdir()) if p.is_file() and p.name!='artifact_manifest.json'};dump(clean({'identities':identities,'inputs':input_hash,'no_input_mutation':True,'artifacts':artifacts}),out/'artifact_manifest.json')
 return 0
if __name__=='__main__':raise SystemExit(main())
