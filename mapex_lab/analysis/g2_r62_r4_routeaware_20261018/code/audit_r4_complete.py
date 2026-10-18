"""Final non-GT R4 run audit. Uses actual odom + P1 observed images + receipts."""
from pathlib import Path
import json, numpy as np, cv2
ROOT=Path('/home/dell/DELL_NATIVE2D')
RUNS=ROOT/'runs'
OUTPUT=ROOT/'analysis/g2_r62_r4_routeaware_20261018/evidence/R4_ALL_RUNS_FINAL_AUDIT.json'
SPECS=[
('R62_R2','NR-G2R62-03','native_g2_r62_general_r2_run01/G2_R62_GENERAL_R2_RUN01_visvarprob','g2_r62'),
('R62_R3','NR-G2R62R3-02','native_g2_r62_r3_native01/G2_R62_R3_NATIVE01_visvarprob','g2_r62_r3'),
('R4A_ON_R2','NR-G2R62R4-02','native_g2_r62_r4_native01/G2_R62_R4_NATIVE01_visvarprob','g2_r62'),
('R4A_ON_R3','NR-G2R62R4A-02','native_g2_r62_r4a_replay01/G2_R62_R4A_REPLAY01_visvarprob','g2_r62_r3')]
aud={'coordinate_system':'native P1 map 203 x 263, 0.1m/cell; GT never used for online decisions',
     'runs':{}}
masks={}
for name,rid,sub,g2root in SPECS:
 run=RUNS/rid
 source=run/sub
 receipt=json.loads((run/'RUN_RECEIPT.json').read_text())
 parity=json.loads((run/'DELL_PARITY_AUDIT.json').read_text())
 assert receipt['outcome']=='SOURCE_TERMINAL'
 pos=np.load(source/'odom.npy').astype(float)
 legs=np.linalg.norm(np.diff(pos,axis=0),axis=1)*.1
 local=0.;global_m=0.
 for tick,step in enumerate(legs):
  summary=json.loads((source/'room_r41/steps'/f'tick_{tick+1:06d}_summary.json').read_text())
  scope=summary.get('current_scope') or {}
  if scope.get('mode')=='LOCAL':local+=float(step)
  else:global_m+=float(step)
 obs=cv2.imread(str(source/'global_obs'/f'{len(pos)-1:08d}.png'),0)[500:703,500:763]
 mask=(obs!=128)
 masks[name]=mask
 decisions=[]
 for f in sorted((source/'room_r41/decisions').glob('decision_*.json')):
  x=json.loads(f.read_text())
  decisions.append({'d':x['decision'],'tick':x['native_tick'],'mode':x['mode'],
                    'room_id':x['room_id'],
                    'g1_reason':x.get('reason'),
                    'r4_guard':(x.get('r4_exit_guard') or {}).get('guard'),
                    'cumulative_m':x['cumulative_distance_m_before_decision']})
 r3res=[]
 for f in sorted((source/g2root/'decisions').glob('decision_*.json')):
  z=json.loads(f.read_text())
  if z.get('r3_choice_source')=='FRONTIER_RESIDUAL':
   r3res.append({'tick':z.get('native_tick'),'goal_rc':z.get('goal_core_rc')})
 events=[json.loads(f.read_text()) for f in sorted((source/'room_r41').glob('r4_exit_recheck_*.json'))]
 aud['runs'][name]={
    'run_id':rid,'outcome':receipt['outcome'],
    'native_steps':len(pos),'real_path_m':round(float(legs.sum()),6),
    'local_m':round(local,6),'global_m':round(global_m,6),
    'observed_cells':int(mask.sum()),'total_cells':int(mask.size),
    'observed_pct':round(100*mask.mean(),6),
    'room_node_decisions':len(decisions),'parity_summary':parity.get('result'),
    'r3_frontier_residual_goal_choices':r3res,
    'r4_clearance_recheck_count':len(events),
    'r4_clearance_recheck_events':events,
    'decisions':decisions
 }
for n in masks:
 aud['runs'][n]['observed_mask_difference_cells_vs_R2']=int(np.count_nonzero(masks[n]!=masks['R62_R2']))
 aud['runs'][n]['observed_mask_overlap_cells_vs_R2']=int(np.count_nonzero(masks[n]&masks['R62_R2']))
aud['comparisons']={
 'R4A_R2_vs_R2_distance_m':round(aud['runs']['R4A_ON_R2']['real_path_m']-aud['runs']['R62_R2']['real_path_m'],6),
 'R4A_R3_vs_R3_distance_m':round(aud['runs']['R4A_ON_R3']['real_path_m']-aud['runs']['R62_R3']['real_path_m'],6),
 'r4_guard_fired_in_closed_loop':False,
 'causal_attribution_to_guard':'NOT_ESTABLISHED',
 'all_terminal_observed_counts_equal':len(set(int(x.sum()) for x in masks.values()))==1,
 'all_terminal_observed_masks_equal':all(np.array_equal(x,masks['R62_R2']) for x in masks.values())
}
OUTPUT.write_text(json.dumps(aud,ensure_ascii=False,indent=2)+'\n')
for name,x in aud['runs'].items():
 print(name,'distance',x['real_path_m'],'LOCAL',x['local_m'],'GLOBAL',x['global_m'],'observed',x['observed_cells'],'pixels differing',x['observed_mask_difference_cells_vs_R2'],'D',len(x['decisions']),'rechecks',x['r4_clearance_recheck_count'])
print('FINAL_AUDIT',OUTPUT)
