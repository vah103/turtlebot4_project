#!/home/dell/miniforge3/envs/lama/bin/python
"""Compare all GLOBAL decision states: original Run04 vs G2-EarlyDoor-R1.
Official native snapshots and selected routes; no GT, no future input to score.
Sequential GLOBAL ordinal comparison is descriptive, not matched causal state.
"""
from pathlib import Path
import json
import numpy as np
B=Path('/home/dell/DELL_NATIVE2D/runs')
def load(runid):
 r=next((B/runid).glob('native_*/*'));p=r/'room_r41';odom=np.load(r/'odom.npy')
 ds=sorted([json.loads(x.read_text()) for x in (p/'decisions').glob('decision_*.json') if 'fallback' not in x.name],key=lambda v:v['native_tick'])
 result=[]
 for i,d in enumerate(ds):
  if d['mode']!='GLOBAL':continue
  t=d['native_tick'];end=ds[i+1]['native_tick'] if i+1<len(ds) else len(odom)-1
  with np.load(p/'inference'/('decision_%06d_input.npz'%d['decision'])) as z:
   enc=z['observed'].copy();mean=z['mean'].copy();var=z['variance'].copy()
  plan=json.loads((p/'steps'/('tick_%06d_plan.json'%t)).read_text())
  robot=np.asarray(d['robot_rc'],float)-500
  nextgoal=np.asarray(plan['goal_rc'],float)-500
  frontiers=[]
  for f in d['frontiers']:
   pt=np.array([f['row']-500,f['col']-500],float)
   eu=float(np.linalg.norm(pt-robot))
   frontiers.append(dict(index=f['index'],core=pt.astype(int).tolist(),saved_cost=float(f['cost']),
     distance_euclid_m=round(eu*.1,6),raw_ig=round(-float(f['cost'])*eu,5)))
  planned=float(plan['astar_length_m'])
  traveled=float(np.linalg.norm(np.diff(odom[t:end+1].astype(float),axis=0),axis=1).sum()*.1)
  early=d.get('g2_early_global_door',{})
  result.append(dict(global_ordinal=len(result)+1,decision=d['decision'],tick=t,end_tick=end,
   robot_core=robot.astype(int).tolist(),selected_goal_core=nextgoal.astype(int).tolist(),
   traveled_m=round(traveled,6),astar_m=round(planned,6),
   n_frontiers=len(frontiers),observed=int((enc!=1).sum()),
   early_reason=early.get('reason'),early_selected_index=early.get('selected_index'),
   early_original_index=early.get('original_index'),
   early_reserved_goal_p1=(np.asarray(early['reserved_goal_rc'])-500).tolist()
       if early.get('reserved_goal_rc') is not None else None,
   _enc=enc,_mean=mean,_var=var,_frontiers=frontiers))
 return result
base=load('NR-RN-04');early=load('NR-G2EARLYD-02');sweep=load('NR-G2SWEEP-02')
assert len(base)==len(early)==6
# Frozen sweep D23 and early D23 have exact current map/pose until t551.
# Retrieve unmodified MapEx candidate costs from control, since applied goal
# override changes only the SELECTED cost field in EARLY log.
ea=early[4];sw=sweep[4]
assert ea['tick']==sw['tick']==551 and np.array_equal(ea['_enc'],sw['_enc']) and ea['robot_core']==sw['robot_core']
control_map={tuple(f['core']):f for f in sw['_frontiers']}
for f in ea['_frontiers']:
 if tuple(f['core']) in control_map:
  saved=control_map[tuple(f['core'])]
  assert np.isclose(f['saved_cost'],saved['saved_cost'],atol=1e-7) or f['core']==ea['selected_goal_core']
  f['original_mapex_cost_before_override']=saved['saved_cost']
 else:
  f['original_mapex_cost_before_override']=None
groups=[]
for k,(a,b) in enumerate(zip(base,early),1):
 knowna=a['_enc']!=1;knownb=b['_enc']!=1
 ua=(a['_enc']==1)&(b['_enc']==1)
 fa={tuple(z['core']):z for z in a['_frontiers']}
 fb={tuple(z['core']):z for z in b['_frontiers']}
 common=sorted(set(fa)&set(fb))
 def compact(v):
  s={k:x for k,x in v.items() if not k.startswith('_')}
  s['top_original_5']=sorted(v['_frontiers'],key=lambda f:
      f.get('original_mapex_cost_before_override',f['saved_cost']) if f.get('original_mapex_cost_before_override') is not None else f['saved_cost'])[:5]
  tgt=tuple(v['selected_goal_core'])
  s['chosen_candidate']=next((z for z in v['_frontiers'] if tuple(z['core'])==tgt),None)
  return s
 g=dict(index=k,original=compact(a),early=compact(b),differences=dict(
  traveled_m=round(b['traveled_m']-a['traveled_m'],6),
  exit_pose_separation_m=round(float(np.linalg.norm(np.array(a['robot_core'])-np.array(b['robot_core']))*.1),4),
  observed_count_delta=b['observed']-a['observed'],
  observed_known_XOR=int(np.count_nonzero(knowna^knownb)),
  known_run04_only=int(np.count_nonzero(knowna&~knownb)),
  known_early_only=int(np.count_nonzero(knownb&~knowna)),
  common_WFD_exact=len(common),
  common_unknown=int(ua.sum()),
  mean_mae_common_unknown=round(float(np.abs(a['_mean'][ua]-b['_mean'][ua]).mean()),6) if np.any(ua) else None,
  variance_mae_common_unknown=round(float(np.abs(a['_var'][ua]-b['_var'][ua]).mean()),6) if np.any(ua) else None,
  common_WFD_top3=[dict(goal=list(pt),run04_cost=round(fa[pt]['saved_cost'],6),
    early_logged_cost=round(fb[pt]['saved_cost'],6),
    euclid_run04_m=fa[pt]['distance_euclid_m'],
    euclid_early_m=fb[pt]['distance_euclid_m'])
        for pt in common[:3]]))
 groups.append(g)
 for v in [a,b]:
  del v['_enc'];del v['_mean'];del v['_var']
  del v['_frontiers']
out=dict(procedure='Native observed-same-epoch GLOBAL state comparison, ordinal match, no GT, no fixed-task causal attribution.',run_ids=['NR-RN-04','NR-G2EARLYD-02'],
 notes=['MapEx GLOBAL mode visvarprob originally selects min cost=-predicted UNKNOWN variance / Euclid robot-goal pixels.',
 'EarlyDoor makes early updates during LOCAL; at the first GLOBAL after each room, goal is overridden only if still matches current WFD and is reachable. Else standard MapEx.',
 'D23 first changed target; logged selected cost is overwritten, so original native MapEx original cost restored from independent SWEEP-R1 D23 with bit-identical observed and pose.',
 'GLOBAL rows are matched by chronological order only; physical tasks/maps are not equivalent.'],
 summaries=dict(original_global_m=round(sum(x['traveled_m'] for x in base),6),
                early_global_m=round(sum(x['traveled_m'] for x in early),6),
                differential_global_m=round(sum(x['traveled_m'] for x in early)-sum(x['traveled_m'] for x in base),6)),
 global_decisions=groups)
output=Path('/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2EARLYD_VS_RUN04_GLOBAL_STATES.json')
output.write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
for g in groups:
 a=g['original'];b=g['early'];c=g['differences']
 print('GLOBAL',g['index'],'D',a['decision'],b['decision'],
  'start',a['robot_core'],b['robot_core'],
  'dest',a['selected_goal_core'],b['selected_goal_core'],
  'actual_m',a['traveled_m'],b['traveled_m'],
  'delta',c['traveled_m'],'pose_sep',c['exit_pose_separation_m'],
  'known_XOR',c['observed_known_XOR'],'common_WFD',c['common_WFD_exact'],
  'earlyReason',b['early_reason'],flush=True)
print('WROTE',output)
