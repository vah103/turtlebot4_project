import json,sys,time
from pathlib import Path
import numpy as np
DIR=Path(__file__).resolve().parent
sys.path.insert(0,str(DIR))
sys.path.insert(1,'/home/dell/DELL_NATIVE2D/build/g2_r62_general_structural_r2_20261018')
from r63_exit_planner import RoomGlobalWatchman
from g2_r6_watchman import RoomWatchman
BASE=Path('/home/dell/DELL_NATIVE2D/runs/NR-G2R62-03/native_g2_r62_general_r2_run01/G2_R62_GENERAL_R2_RUN01_visvarprob')
results=[]
for i in (2,8,11,14,16):
 z='decision_%06d'%i
 with np.load(BASE/'room_r41/inference'/(z+'_input.npz')) as f:obs,mean,var=f['observed'],f['mean'],f['variance']
 with np.load(BASE/'room_r41/inference'/(z+'_semantic.npz')) as f:sem=f['semantic']
 with np.load(BASE/'g2_r62'/('native_scope_%06d.npz'%i)) as f:nav=f['reachable_knownfree']
 x=json.loads((BASE/'room_r41/decisions'/(z+'.json')).read_text())
 robot=tuple(int(q)-500 for q in x['robot_rc'])
 base=RoomWatchman(obs,mean,var,sem,robot,nav)
 start=time.perf_counter();b=base.plan()
 new=RoomGlobalWatchman(obs,mean,var,sem,robot,nav)
 result=new.plan()
 row={
  'decision':i,'native_tick':x['native_tick'],'room_id':x['room_id'],
  'room_free_area':int(new.legal.sum()),
  'room_unknown':int(new.mission.sum()),
  'baseline_goals':b['goal_sequence_core_rc'],
  'R63_goals':result['goal_sequence_core_rc'],
  'baseline_room_m':b['predicted_route_distance_m'],
  'R63_room_m':result['predicted_route_distance_m'],
  'baseline_gain':result.get('baseline_predicted_visibility_gain'),
  'R63_gain':result['projected_weighted_gain'],
  'baseline_exit_lowerbound_m':result.get('baseline_global_exit_lowerbound_m'),
  'R63_exit_lowerbound_m':result.get('chosen_global_exit_lowerbound_m'),
  'baseline_total_m':result.get('baseline_total_predicted_cost_m'),
  'R63_total_m':result.get('chosen_total_predicted_cost_m'),
  'policy':result['exit_policy'],
  'extra_planning_s':result.get('exit_computation_s'),
  'goals_safety_knownFREE':result['goals_only_native_known_free'],
  'no_GT_used':not result['used_gt'],
  'time_s':round(time.perf_counter()-start,4)
 }
 results.append(row)
 print('D%02d'%i,'baseline_m',row['baseline_total_m'],'R63_m',row['R63_total_m'],
       'room_m',row['baseline_room_m'],'->',row['R63_room_m'],'exit_m',
       row['baseline_exit_lowerbound_m'],'->',row['R63_exit_lowerbound_m'],
       'gain',row['baseline_gain'],'->',row['R63_gain'],
       'policy',row['policy'],'time',row['time_s'],flush=True)
(DIR/'R63_FROZEN_FIVE_DECISIONS.json').write_text(json.dumps({'source_run':'NR-G2R62-03',
   'snapshots':results,'no_future_ground_truth':True,'all_same_model_and_roomnode_inputs':True,
   'metric':'predicted view plus observed FREE uninflated 4connected frontier exit lower bound, NOT observed executed movement',
   'science_status':'OFFLINE_PROPOSAL_ONLY'},indent=2,ensure_ascii=False)+'\n')