#!/home/dell/miniforge3/envs/lama/bin/python
"""Completed native 2D G2 EarlyDoor vs frozen Sweep vs Run04: exact odometry,
observed-cell-mask and GLOBAL target transition audit. No GT used.
"""
from pathlib import Path
import numpy as np,json
from collections import Counter
B=Path('/home/dell/DELL_NATIVE2D/runs')
names={'MapExG1':'NR-RN-04','G2SweepR1':'NR-G2SWEEP-02','EarlyDoorR1':'NR-G2EARLYD-02'}
data={}
for label,runid in names.items():
 run=B/runid
 if label=='EarlyDoorR1':
  receipt=run/'RUN_RECEIPT.json'
  if not receipt.exists():raise RuntimeError('LIVE_RUN_NOT_COMPLETE')
  state=json.loads(receipt.read_text())
  assert state['outcome']=='SOURCE_TERMINAL' and state['returncode']==0,state
 root=next(run.glob('native_*/*'));odom=np.load(root/'odom.npy').astype(float)
 ds=sorted((json.loads(f.read_text()) for f in (root/'room_r41/decisions').glob('decision_*.json') if 'fallback' not in f.name),key=lambda d:d['native_tick'])
 chunks=np.linalg.norm(np.diff(odom,axis=0),axis=1)*.1
 totals={'GLOBAL':0.,'LOCAL':0.}
 globalrows=[];localrows=[];door_updates=[];early_releases=[]
 first_room=None
 for i,d in enumerate(ds):
  tick=d['native_tick'];end=ds[i+1]['native_tick'] if i+1<len(ds) else len(odom)-1
  # Correct for last decision: end=last native index
  section=float(chunks[tick:end].sum())
  totals[d['mode']]+=section
  plan=json.loads((root/'room_r41/steps'/('tick_%06d_plan.json'%tick)).read_text())
  if d['mode']=='GLOBAL':
   globalrows.append(dict(decision=d['decision'],tick=tick,pose_core=(np.array(d['robot_rc'])-500).tolist(),
     goal_core=(np.array(plan['goal_rc'])-500).tolist(),distance_m=round(section,4),reason=d.get('g2_early_global_door')))
  else:
   localrows.append(dict(decision=d['decision'],tick=tick,room=d['room_id'],
      goal_core=(np.array(plan['goal_rc'])-500).tolist(),distance_m=round(section,4)))
  diag=d.get('g2_early_global_door')
  if diag and d['mode']=='LOCAL':door_updates.append(dict(decision=d['decision'],tick=tick,room=d['room_id'],
    reserved=None if diag.get('current_goal_rc') is None else (np.array(diag['current_goal_rc'])-500).tolist(),
    exit=None if diag.get('current_portal_rc') is None else (np.array(diag['current_portal_rc'])-500).tolist(),
    n_portals=diag['n_portals'],new_or_changed=diag['new_or_changed_observed_portal'],
    changed_global=diag['next_global_goal_changed']))
  if diag and d['mode']=='GLOBAL' and 'reserved_goal_rc' in diag:
   early_releases.append(dict(decision=d['decision'],tick=tick,
      reason=diag['reason'],original_index=diag['original_index'],
      selected_index=diag['selected_index'],
      swapped=diag['selected_index']!=diag['original_index'],
      reserved=None if diag['reserved_goal_rc'] is None else (np.array(diag['reserved_goal_rc'])-500).tolist()))
  if first_room is None and d['mode']=='GLOBAL' and d['decision']>2:
   start=ds[1]['native_tick']
   with np.load(root/'room_r41/inference'/('decision_%06d_input.npz'%d['decision'])) as z:observed=z['observed']
   first_room=dict(end_decision=d['decision'],exit_tick=tick,local_m=round(float(chunks[start:tick].sum()),6),
     exit_pose=(np.array(d['robot_rc'])-500).tolist(),known_p1=int((observed!=1).sum()))
 with np.load(root/'room_r41/steps'/('tick_%06d_postscan.npz'%(len(odom)-1))) as z:
  mask=z['observed'][500:703,500:763]!=.5
 data[label]=dict(id=runid,ticks=len(odom),decisions=len(ds),distance_m=float(chunks.sum()),
   global_m=totals['GLOBAL'],local_m=totals['LOCAL'],known_p1=int(mask.sum()),final_known=mask,
   globals=globalrows,locals=localrows,
   first_room=first_room,updates=door_updates,releases=early_releases)
base=data['G2SweepR1'];new=data['EarlyDoorR1']
equality=dict(initial_85_odometry_equal=bool(np.array_equal(np.load(next((B/'NR-G2SWEEP-02').glob('native_*/*'))/'odom.npy')[:85],np.load(next((B/'NR-G2EARLYD-02').glob('native_*/*'))/'odom.npy')[:85])))
for name,x in data.items():
 if name=='EarlyDoorR1':continue
 x['vs_early']=dict(early_missing_p1=int(np.sum(x['final_known']&~new['final_known'])),
                    early_extra_p1=int(np.sum(new['final_known']&~x['final_known'])),
                    early_delta_total_m=float(new['distance_m']-x['distance_m']),
                    early_delta_global_m=float(new['global_m']-x['global_m']),
                    early_delta_local_m=float(new['local_m']-x['local_m']))
 del x['final_known']
del new['final_known']
prev_rooms=set();actual_new=0
for u in new['updates']:
 if u['room'] in prev_rooms and u['new_or_changed']:actual_new+=1
 prev_rooms.add(u['room'])
door_log=dict(n_LOCAL_epochs=len(new['updates']),n_new_openings_after_room_entry=actual_new,
  n_goal_or_door_updates=sum(q['changed_global'] for q in new['updates']),
  n_release=len(new['releases']),
  n_honored_early_goal=sum(q['reason']=='VALID_EARLY_GLOBAL_GOAL' for q in new['releases']),
  n_reassigned_first_global=sum(q['swapped'] for q in new['releases']),
  reason_counts=dict(Counter(q['reason'] for q in new['releases'])))
out=dict(method='Frozen 2D source-terminal run, original observed P1. Bit-equal parent LOCAL before first release; no GT. Run-relative global/local distances are not fixed-task comparisons.',runs=data,
   door_log=door_log,initial_parent_parity=equality)
dst=Path('/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2EARLYD_NATIVE_COMPARE.json')
dst.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
for label,x in data.items():
 print('RESULT',label,'meters',round(x['distance_m'],6),'GLOBAL',round(x['global_m'],6),'LOCAL',round(x['local_m'],6),'P1',x['known_p1'],'nDec',x['decisions'],'nTicks',x['ticks'],'first_room',x['first_room'],flush=True)
print('DELTA',[(n,x.get('vs_early')) for n,x in data.items()],flush=True)
print('DOOR',door_log,'RELEASES',new['releases'],flush=True)
print('OUTPUT',dst,flush=True)
