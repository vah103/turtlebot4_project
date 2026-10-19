#!/home/dell/miniforge3/envs/lama/bin/python
"""Independent completed native G2 room sweep comparison.
No GT, compares observed P1 only and native odometry from frozen runs.
"""
from pathlib import Path
import numpy as np,json
B=Path('/home/dell/DELL_NATIVE2D/runs')
cases={'Run04_MapEx_G1':'NR-RN-04','PVU_R2':'NR-G2PVU-03','Sweep_R1':'NR-G2SWEEP-02'}
out={}
for label,run_id in cases.items():
 root=B/run_id
 child=next(root.glob('native_*/*'))
 if label=='Sweep_R1':
  receipt=root/'RUN_RECEIPT.json'
  if not receipt.exists():raise RuntimeError('SWEEP_HAS_NO_RECEIPT')
  rec=json.loads(receipt.read_text())
  print('RECEIPT',rec['outcome'],'returncode',rec['returncode'],flush=True)
  if rec['outcome']!='SOURCE_TERMINAL':raise RuntimeError('SWEEP_TERMINAL_NOT_SUCCESSFUL')
 ds=sorted([json.loads(f.read_text()) for f in (child/'room_r41/decisions').glob('decision_*.json') if 'fallback' not in f.name],key=lambda x:x['native_tick'])
 od=np.load(child/'odom.npy')
 seg=np.linalg.norm(np.diff(od.astype(float),axis=0),axis=1)*.1
 mode_dist={'GLOBAL':0.,'LOCAL':0.}
 first_room=None
 sweeps=[]
 for i,d in enumerate(ds):
  t=int(d['native_tick']);end=int(ds[i+1]['native_tick']) if i<len(ds)-1 else len(od)-1
  mode_dist[d['mode']]+=float(seg[t:end].sum())
  s=d.get('g2_room_sweep')
  if s and s.get('enabled'):sweeps.append(dict(decision=d['decision'],tick=t,room=d['room_id'],selected=s['first_index'],original=s['baseline_index'],reason=s['reason'],phase=s['phase'],sign=s.get('sign'),changed=s.get('changed_goal',False)))
  if first_room is None and d['mode']=='GLOBAL' and d['decision']>2:
   first_room=dict(end_decision=d['decision'],exit_tick=t,
    exit_robot_P1=(np.array(d['robot_rc'])-500).tolist(),
    local_m=round(float(seg[int(ds[1]['native_tick']):t].sum()),6),
    local_decisions=sum(q['mode']=='LOCAL' for q in ds[1:i]))
   with np.load(child/'room_r41/inference'/('decision_%06d_input.npz'%d['decision'])) as z:
    first_room['observed_P1_at_exit']=int((z['observed']!=1).sum())
 with np.load(child/'room_r41/steps'/('tick_%06d_postscan.npz'%(len(od)-1))) as z:
  final=z['observed'][500:703,500:763]
 known=(final!=.5)
 out[label]=dict(id=run_id,ticks=len(od),total_m=float(seg.sum()),
  global_m=mode_dist['GLOBAL'],local_m=mode_dist['LOCAL'],
  n_decisions=len(ds),n_global=sum(d['mode']=='GLOBAL' for d in ds),
  n_local=sum(d['mode']=='LOCAL' for d in ds),
  observed_P1=int(known.sum()),_mask=known,first_room=first_room,
  n_sweep_goal_changed=sum(q['changed'] for q in sweeps),
  n_sweep_reversal=sum(q['reason']=='ONE_CLEANUP_REVERSAL' for q in sweeps),
  n_sweep_residual_fallback=sum(q['reason']=='MAPEX_RESIDUAL_FALLBACK' for q in sweeps),
  g2_trace=sweeps)
for label,d in out.items():
 if label=='Sweep_R1':continue
 mask=d['_mask'];sw=out['Sweep_R1']['_mask']
 d['sweep_missed_vs_'+label]=int(np.count_nonzero(mask&~sw))
 d['sweep_extra_vs_'+label]=int(np.count_nonzero(sw&~mask))
 del d['_mask']
out['Sweep_R1']['differential']={key:{
 'distance_delta_m':round(out['Sweep_R1']['total_m']-out[key]['total_m'],6),
 'global_delta_m':round(out['Sweep_R1']['global_m']-out[key]['global_m'],6),
 'local_delta_m':round(out['Sweep_R1']['local_m']-out[key]['local_m'],6),
 'observed_delta_cells':out['Sweep_R1']['observed_P1']-out[key]['observed_P1'],
 'missing_core_cells':out[key]['sweep_missed_vs_'+key],
 'extra_core_cells':out[key]['sweep_extra_vs_'+key]}
 for key in ('Run04_MapEx_G1','PVU_R2')}
del out['Sweep_R1']['_mask']
dest=Path('/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2SWEEP_R1_NATIVE_COMPARE.json')
dest.write_text(json.dumps(dict(method='Completed native runs, identical P1 support and run config; compare end observed masks and numeric odometry. No GT used. Different final maps imply non-parity; do not claim equivalent-coverage gain.',runs=out),ensure_ascii=False,indent=2)+'\n')
for key,v in out.items():
 print('RESULT',key,'meters',round(v['total_m'],6),'global',round(v['global_m'],6),'local',round(v['local_m'],6),'known',v['observed_P1'],'decisions',v['n_decisions'], 'firstroom',v['first_room'],flush=True)
print('DELTA',out['Sweep_R1']['differential'],flush=True)
print('SWEEP_CHANGES',out['Sweep_R1']['n_sweep_goal_changed'],'REVERSALS',out['Sweep_R1']['n_sweep_reversal'],'RESIDUAL',out['Sweep_R1']['n_sweep_residual_fallback'])
print('OUTPUT',dest)
