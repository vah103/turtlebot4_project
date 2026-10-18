import json,collections,hashlib
from pathlib import Path
import numpy as np,cv2
BASE=Path('/home/dell/DELL_NATIVE2D/runs')
paths={
 'NR-RN-04':BASE/'NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob',
 'NR-G2R63-02':BASE/'NR-G2R63-02/native_g2_r63_room_global_r1_run01/G2_R63_ROOM_GLOBAL_R1_RUN01_visvarprob'}
with np.load(paths['NR-G2R63-02']/'room_r41/inference/decision_000002_input.npz') as z:initial=z['observed']
with np.load(paths['NR-G2R63-02']/'room_r41/inference/decision_000002_semantic.npz') as z:sem=z['semantic']
_,label=cv2.connectedComponents(((sem==1)&(initial!=2)).astype(np.uint8),connectivity=4)
room=label==label[147,111]
anchors={}
for key,p in paths.items():
 arr=np.load(p/'odom.npy').astype(float)
 dist=np.r_[0,np.cumsum(np.linalg.norm(np.diff(arr,axis=0),axis=1)*.1)]
 lookup={}
 for t in range(len(arr)):
  q=p/'global_obs'/('%08d.png'%t)
  a=cv2.imread(str(q),cv2.IMREAD_GRAYSCALE)
  if a is None:raise RuntimeError('MISSING P1 OBS '+str(q))
  a=a[500:703,500:763]
  obs=int((a!=128).sum())
  # Reference D02 room mask uses observed coding 0=free,1=unknown,2=wall.
  # PNG has 128 for UNKNOWN; no GT used.
  remain=int(np.count_nonzero(room & (a==128)))
  lookup[t]=(obs,remain,float(dist[t]))
 final_obs,final_room,final_dist=lookup[len(arr)-1]
 milestones={}
 for n,threshold in [('90pct',int(np.ceil(0.9*53389))),('95pct',int(np.ceil(0.95*53389))),('96pct',int(np.ceil(0.96*53389))),('reference_final_P1_observed',51304)]:
  match=next(((t,*row) for t,row in lookup.items() if row[0]>=threshold),None)
  milestones[n]={'threshold_cells':threshold,'native_tick':match[0],'observed_P1':match[1],
                 'initial_D02_room_unknown':match[2],'distance_m':round(match[3],6)} if match else None
 hit=next(((t,*row) for t,row in lookup.items() if t>=12 and row[1]<=270),None)
 milestones['D02_ROOM_270']={'native_tick':hit[0],'P1_observed':hit[1],
        'initial_D02_room_unknown':hit[2],
        'distance_after_D02_m':round(hit[3]-dist[12],6),
        'total_distance_m':round(hit[3],6)} if hit else None
 last=cv2.imread(str(p/'global_obs'/('%08d.png'%(len(arr)-1))),cv2.IMREAD_GRAYSCALE)[500:703,500:763]
 data={'status':json.loads((BASE/key/'RUN_RECEIPT.json').read_text())['outcome'],
       'n_steps':len(arr),'total_distance_m':round(float(dist[-1]),6),
       'observed_P1':final_obs,'unknown_P1':53389-final_obs,'coverage_fraction_P1':round(final_obs/53389,9),
       'last_D02_room_unknown':final_room,'milestones':milestones,'map':last}
 if key=='NR-G2R63-02':
  gg=p/'g2_r63'
  events=[]
  for f in sorted((gg/'updates').glob('result_*.json')):
   z=json.loads(f.read_text());events.append(z.get('structural',{}).get('event'))
  log=(BASE/key/'worker.stdout.log').read_text(errors='replace')
  data.update({
    'S0_S1_S2_S3_S4_count':dict(collections.Counter(events)),
    'heavy_LaMa_RoomNode_decisions':len(list((p/'room_r41/decisions').glob('decision_??????.json'))),
    'G2_R63_watchman_goal_decisions':len(list((gg/'decisions').glob('decision_??????.json'))),
    'native_local_room_WFD_fallback_count':log.count('G2_R63_RESIDUAL_ROOM_WFD_FALLBACK'),
    'zero_progress_viewpoint_triggers':log.count('G2_R62_ZERO_PROGRESS_DISABLE_ROOM_WATCHMAN')})
 anchors[key]=data
a=anchors['NR-G2R63-02'];b=anchors['NR-RN-04']
different=int(np.count_nonzero(a['map']!=b['map']))
reference_obs=51304
result={
  'date_local':'2026-10-18',
  'run_id':'NR-G2R63-02',
  'comparison':'NR-RN-04',
  'metric_is_observed_LiDAR_not_GT':True,
  'baseline_full_DELL_COM1_parity_report':'/home/dell/DELL_NATIVE2D/runs/NR-RN-04/DELL_VS_COM1_FULL_PARITY.json',
  'new_run_D01D02_pre_goal_parity':json.loads((BASE/'NR-G2R63-02/DELL_PARITY_AUDIT.json').read_text())['result'],
  'P1_native_map_cells':53389,
  'frozen_D02_room_cells':int(room.sum()),
  'frozen_D02_initial_unknown':int(np.count_nonzero(room&(initial==1))),
  'baseline':{k:v for k,v in b.items() if k!='map'},
  'r63':{k:v for k,v in a.items() if k!='map'},
  'delta_total_distance_m':round(a['total_distance_m']-b['total_distance_m'],6),
  'distance_ratio':round(a['total_distance_m']/b['total_distance_m'],6),
  'paired_final_observed_cells_difference':int(a['observed_P1']-b['observed_P1']),
  'paired_final_core_occupancy_state_mismatch_cells':different,
  'r63_outcome_is_gt_complete':False,
  'any_gt_as_planner_input':False,
  'individual_models_and_thresholds_cross_Hospital_validated':False
}
for name in ['baseline','r63']:
 p=BASE/('NR-RN-04' if name=='baseline' else 'NR-G2R63-02')
 rc=json.loads((p/'RUN_RECEIPT.json').read_text())
 result[name].update({'guard_wall_s':rc['duration_s'],
                        'peak_pss_MiB':rc['peak_pss_MiB'],
                        'min_available_MiB':rc['min_available_MiB'],
                        'terminal_mission_state':rc['mission_status']})
result['wall_delta_s']=round(result['r63']['guard_wall_s']-result['baseline']['guard_wall_s'],5)
out=BASE/'NR-G2R63-02'
(out/'R63_FULL_P1_OBSERVED_AUDIT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({
   'D02_ROOM_THRESHOLD':{k:v['milestones']['D02_ROOM_270'] for k,v in [('NR-RN-04',b),('NR-G2R63-02',a)]},
   'MILESTONES':{k:v['milestones'] for k,v in [('NR-RN-04',b),('NR-G2R63-02',a)]},
   'FULL_R62':{k:result[k] for k in ['delta_total_distance_m','distance_ratio','wall_delta_s','paired_final_observed_cells_difference']},
   'R62_EVENTS':a['S0_S1_S2_S3_S4_count'],'WFD_FALLBACK':a['native_local_room_WFD_fallback_count'],
},indent=2))