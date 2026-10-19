#!/home/dell/miniforge3/envs/lama/bin/python
"""Paper raycast original MapEx vs patched native New Room runs, terminal metrics.
Observed unknown/support from native persisted final source postscan/PNG (no GT).
Run-path differences and alternate coverage explicitly reported.
"""
from pathlib import Path
import json,numpy as np
from PIL import Image
B=Path('/home/dell/DELL_NATIVE2D/runs')
PAIR=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED')
SPECS=[
 ('MapEx code-reset original',PAIR/'native_plain_lama3small_run05/MAPEX_PLAIN_LAMA3SMALL_RUN05_visvarprob','plain',None),
 ('MapEx paper cumulative',B/'NR-RAYMX-02','plain','NR-RAYMX-02'),
 ('MapEx+G1 code-reset original',B/'NR-RN-04','room', 'NR-RN-04'),
 ('MapEx+G1 paper cumulative',B/'NR-RAYRN-02','room','NR-RAYRN-02'),
 ('SWEEP code-reset original',B/'NR-G2SWEEP-02','room','NR-G2SWEEP-02'),
 ('SWEEP paper cumulative',B/'NR-RAYSW-02','room','NR-RAYSW-02')
]
def load(name,p,mode,run_id):
 if p.name.endswith('_visvarprob'):
  root=p
 else:
  if not (p/'RUN_RECEIPT.json').exists():
   raise RuntimeError('NOT_COMPLETE '+str(p))
  receipt=json.loads((p/'RUN_RECEIPT.json').read_text())
  if receipt['outcome']!='SOURCE_TERMINAL' or receipt['returncode']!=0:
   raise RuntimeError('NON_TERMINAL '+str(p)+' '+receipt['outcome'])
  root=next(p.glob('native_*/*'))
 od=np.load(root/'odom.npy').astype(float)
 diffs=np.linalg.norm(np.diff(od,axis=0),axis=1)*.1
 if mode=='room':
  source=root/'room_r41'
  decisions=sorted((json.loads(x.read_text()) for x in (source/'decisions').glob('decision_*.json') if 'fallback' not in x.name),
                   key=lambda d:d['native_tick'])
  with np.load(source/'steps'/('tick_%06d_postscan.npz'%(len(od)-1))) as z:
   obs=z['observed']
  assert obs.shape==(1203,1263)
  known=obs[500:703,500:763]!=.5
 else:
  source=root/'plain_mapex_evidence'
  decisions=sorted((json.loads(x.read_text()) for x in (source/'decisions').glob('decision_*.json')),
                   key=lambda d:d['native_tick'])
  file=root/'global_obs'/('%08d.png'%(len(od)-1))
  final=np.array(Image.open(file))
  assert final.shape[:2]==(1203,1263)
  known=final[500:703,500:763,0]!=128
 assert known.shape==(203,263)
 by_mode={'GLOBAL':0.,'LOCAL':0.}
 phase_switch=0;last_mode=None
 decision_rows=[]
 for i,d in enumerate(decisions):
  tick=d['native_tick'];end=decisions[i+1]['native_tick'] if i+1<len(decisions) else len(od)-1
  assert end>=tick
  m=d.get('mode','GLOBAL'); m='GLOBAL' if m=='GLOBAL' else 'LOCAL'
  by_mode[m]+=float(diffs[tick:end].sum())
  if last_mode!=m:phase_switch+=1
  last_mode=m
  if mode=='room':
   plan=json.loads((source/'steps'/('tick_%06d_plan.json'%tick)).read_text())
   goal=plan['goal_rc']
  else:
   goal=d['first_selected_rc']
  decision_rows.append(dict(decision=d['decision'],tick=tick,mode=m,
    robot=[int(z-500) for z in d['robot_rc']],
    goal=[int(z-500) for z in goal]))
 with open(root/'mission_status.json') as f:mission=json.load(f)
 result=dict(name=name,id=run_id or 'COM1_FROZEN_RUN05',root=str(root),
   native_ticks=len(od),total_m=float(diffs.sum()),
   global_m=by_mode['GLOBAL'],local_m=by_mode['LOCAL'],
   decision_count=len(decisions),global_decisions=sum(x['mode']=='GLOBAL' for x in decision_rows),
   local_decisions=sum(x['mode']=='LOCAL' for x in decision_rows),
   phase_changes=phase_switch,final_known_cells=int(known.sum()),
   mission_complete=mission['mission_complete'],
   stop_reason=mission['fail_reason'],
   decision_trace=decision_rows,final_known=known)
 if run_id and (p/'RUN_RECEIPT.json').exists():
  rec=json.loads((p/'RUN_RECEIPT.json').read_text())
  result['runtime_wall_s']=rec['duration_s']
  if run_id.startswith('NR-RAY'):
   gate=json.loads((p/'DELL_PARITY_AUDIT.json').read_text())
   assert gate['result']=='D01_D02_VARIANT_INTEGRITY_PASS'
   result['patch_D01D02_variant_gate']=gate['result']
 return result
results=[load(*x) for x in SPECS]
for i in range(0,6,2):
 a,b=results[i:i+2]
 b['vs_original_code']=dict(delta_total_m=b['total_m']-a['total_m'],
  delta_global_m=b['global_m']-a['global_m'],delta_local_m=b['local_m']-a['local_m'],
  terminal_mask_missing=b['final_known_cells']-a['final_known_cells'],
  only_original_cells=int(np.count_nonzero(a['final_known']&~b['final_known'])),
  only_paper_cells=int(np.count_nonzero(b['final_known']&~a['final_known'])))
# Compare patched variants to each other, including terminal state differences
for i in [3,5]:
 old=results[1];new=results[i]
 new['vs_paper_MapEx_plain']=dict(delta_total_m=new['total_m']-old['total_m'],
  only_plain_cells=int(np.count_nonzero(old['final_known']&~new['final_known'])),
  only_new_cells=int(np.count_nonzero(new['final_known']&~old['final_known'])))
for item in results:
 del item['final_known']
 print('BENCHMARK',item['name'],'ticks',item['native_ticks'],'nDec',item['decision_count'],
  'distance',round(item['total_m'],4),
  'GLOBAL',round(item['global_m'],4),'LOCAL',round(item['local_m'],4),
  'P1 known',item['final_known_cells'],'terminal',item['mission_complete'],flush=True)
out=Path('/home/dell/DELL_NATIVE2D/analysis/raycast_parity_20261019/PAPER_RAYCAST_NATIVE_3WAY_COMPARISON.json')
out.write_text(json.dumps(dict(science='Single delta original probabilistic raycast accumulator scope; 3 New Room runs',
 methodology='Native terminal observed grids+odometry, no GT coverage claims; COMPARE end masks not just counts',
 runs=results),indent=2,ensure_ascii=False)+'\n')
print('OUTPUT',out,flush=True)
