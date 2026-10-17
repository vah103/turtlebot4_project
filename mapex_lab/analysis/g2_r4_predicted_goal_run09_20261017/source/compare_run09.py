"""Run08 pre-registered postrun audit: source-bound comparisons to frozen Run07/Run04/Run05.

No planner GT access. Evaluation mask frozen before new run (D02 initial).
Match first achieving <=270 old D02 UNKNOWN and examine exact arrival traces.
"""
from pathlib import Path
import numpy as np, cv2, json, io, zipfile, hashlib
from math import hypot
R=Path('/home/dell/mx072_g2_r4_predicted_goal_run09_20261017')
R08=Path('/home/dell/mx072_g2_r4_exact_run08_com1_20261017')
O=Path('/home/dell/mx072_g2_r4_native_com1_run07_20261017')
B=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED')
arms={
'Run04_RoomNode_MapEx': B/'native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob',
'Run05_plain_MapEx': B/'native_plain_lama3small_run05/MAPEX_PLAIN_LAMA3SMALL_RUN05_visvarprob',
'Run07_G2_R4_1m': O/'native_g2_r4_full_run07/G2_R4_GOALCHAIN_RUN07_visvarprob',
'Run08_G2_R4_exact': R08/'native_g2_r4_goal_exact_run08/G2_R4_EXACT_GOAL_RUN08_visvarprob',
'Run09_G2_R4_predicted': R/'native_g2_r4_predicted_run09/G2_R4_PREDICTED_GOAL_RUN09_visvarprob'}
with zipfile.ZipFile(O/'evidence/reference/D02_G2_R31_CAUSAL_EVIDENCE.zip') as z:
 with np.load(io.BytesIO(z.read('initial/memory_D02.npz')),allow_pickle=False) as n:
  room=n['original_room'].copy()
 with np.load(io.BytesIO(z.read('source_D02/decision_000002_input.npz')),allow_pickle=False) as n:
  initial=n['observed'].copy()
assert room.shape==(203,263) and int(room.sum())==6564
unknown_initial=int(np.sum(room&(initial==1)))
assert unknown_initial==3889
THRESHOLDS=[2000,1000,900,600,400,342,320,270,200,100,50,20,0]
out={'schema':'R09_PREDICTED_GOAL_NATIVE_COMPARE_V1','eval_roi':'frozen D02 initial RoomNode mask','initial_unknown':unknown_initial,
     'room_cells':int(room.sum()),'tick0':12,'resolution_m':0.1,
     'source_obs_sha256':hashlib.sha256(initial.tobytes()).hexdigest(),
     'goal_exact_cell_tolerance':'0 cell, executable waypoint ONLY; speculative goal never commands native unknown motion','arms':{}}
for name,p in arms.items():
 files=sorted((p/'global_obs').glob('*.png'))
 if not files:raise ValueError((name,'no observations'))
 last=int(files[-1].stem)
 odom=np.load(p/'odom.npy')
 dist=np.zeros(len(odom),np.float64)
 dist[1:]=np.cumsum(np.linalg.norm(np.diff(odom.astype(float),axis=0),axis=1)*0.1)
 assert len(odom)>=last+1,(name,len(odom),last)
 nrows={}
 cached={}
 def missing(t):
  if t not in cached:
   img=cv2.imread(str(p/'global_obs'/('%08d.png'%t)),cv2.IMREAD_GRAYSCALE)
   if img is None:raise ValueError((name,t,'obs missing'))
   current=img[500:703,500:763]
   assert current.shape==initial.shape
   if t==12:
    translated=np.where(current==0,0,np.where(current==128,1,2)).astype(np.uint8)
    assert np.array_equal(initial,translated),(name,'D02 initial mismatch')
   cached[t]=int(np.sum((current==128)&room))
  return cached[t]
 first=[]
 for t in range(12,last+1):
  u=missing(t)
  for threshold in THRESHOLDS:
   if u<=threshold and str(threshold) not in nrows:
    nrows[str(threshold)]={'tick':t,'remaining_unknown':u,'ticks_from_D02':t-12,
         'meters_from_D02':round(float(dist[t]-dist[12]),4),
         'nominal_travel_seconds_at_0_3mps':round(float((dist[t]-dist[12])/0.3),3)}
  if t%50==0:
   first.append({'tick':t,'remaining_unknown':u,'meters_from_D02':round(float(dist[t]-dist[12]),3)})
 primary=[]
 for f in sorted((p/'room_r41/decisions').glob('decision_[0-9]*.json')) if (p/'room_r41/decisions').exists() else []:
  if '_global_fallback' in f.stem:continue
  d=json.loads(f.read_text())
  t=int(d['native_tick'])
  plan=json.loads((p/'room_r41/steps'/('tick_%06d_plan.json'%t)).read_text())
  primary.append({'decision':int(d['decision']),'native_tick':t,'mode_native':plan['scope']['mode'],
                  'actual_goal':plan['goal_rc'],
                  'native_room_mode':d['mode'],'room_id':d['room_id'],
                  'prediction_s':float(d.get('elapsed_lama_prediction_s',0)),
                  'room_score_s':float(d.get('elapsed_room_plus_score_s',0))})
 chosen=[]
 if (p/'g2_r4/decisions').exists():
  for f in sorted((p/'g2_r4/decisions').glob('decision_*.json')):
   d=json.loads(f.read_text());i=int(f.stem.rsplit('_',1)[1]);t=int(d['native_tick'])
   chosen.append({'decision':i,'tick':t,'room_id':d['room_id'],'g2_decision':d['decision'],
                  'original_room_unknown':d['original_room_unknown'],'goal_rc_global':d['native_goal_global_rc'],
                  'predicted_target_core_rc':d.get('predicted_target_core_rc'),
                  'predicted_mode':d.get('predicted_goal_decision') and d['predicted_goal_decision']['status'],
                  'compute_s':d['analysis_time_s'],'goal_receipt':d['prior_observation_event'],
                  'r4_reason':d['R4_reason']})
 for index,item in enumerate(chosen):
  if item['g2_decision']!='GOAL':continue
  t=item['tick']
  until=chosen[index+1]['tick'] if index+1<len(chosen) else last
  if item['goal_rc_global'] is None:continue
  goal=np.array(item['goal_rc_global'])
  near=np.linalg.norm(odom[t:until+1]-goal,axis=1)*0.1
  item['next_G2_tick']=until
  item['min_residual_m_before_next_G2']=round(float(near.min()),4)
  item['reached_exact_pixel_before_next_G2']=bool(np.any(near==0))
  item['distance_at_next_G2_m']=round(float(near[-1]),4)
  # Decision may be GLOBAL after LOCAL exit, but exact arrival still audited per previous selected goal.
 mark=nrows.get('270')
 timings=None
 if mark:
  t=mark['tick'];pr=[x for x in primary if 12<=x['native_tick']<t]
  gr=[x for x in chosen if 12<=x['tick']<t]
  timings={'prediction_s':round(sum(x['prediction_s'] for x in pr),3),
           'room_native_scoring_s':round(sum(x['room_score_s'] for x in pr),3),
           'G2_policy_s':round(sum(x['compute_s'] for x in gr),3)}
  timings['travel_plus_recorded_compute_s_proxy']=round(mark['nominal_travel_seconds_at_0_3mps']+sum(timings.values()),3)
 stats={'source_dir':str(p),'native_final_tick':last,'native_odom_length':len(odom),
        'total_distance_from_start_m':round(float(dist[last]),3),
        'total_distance_after_D02_m':round(float(dist[last]-dist[12]),3),
        'initial_D02_observed_map_mismatch_cells':0,
        'last_remaining_D02_unknown':missing(last),
        'milestones':nrows,'trace_every50':first,'goal_decisions':chosen,
        'primary_decisions':len(primary),
        'G2_exact_visited':sum(x.get('reached_exact_pixel_before_next_G2',False) for x in chosen),
        'G2_goal_count':sum(x['g2_decision']=='GOAL' for x in chosen),
        'G2_completion_receipts':sum(x['goal_receipt'].startswith('COMPLETED_GOAL') for x in chosen),
        'G2_invalidated_receipts':sum(x['goal_receipt'].startswith('INVALIDATED_EARLY') for x in chosen),
        'time_components_until_270':timings}
 out['arms'][name]=stats
 print('ARM',name,'final_step',last,'end_m',stats['total_distance_from_start_m'],
       'D02@270',nrows.get('270'),'G2_GOALS',stats['G2_goal_count'],
       'EXACT',stats['G2_exact_visited'],
       'COMPLETED',stats['G2_completion_receipts'],'INVALIDATED',stats['G2_invalidated_receipts'],flush=True)
r=R/'evidence/RUN09_PREDICTED_GOAL_COMPARE_RUN04_05_07_08.json'
r.write_text(json.dumps(out,ensure_ascii=False,indent=2))
print('WROTE',r,round(r.stat().st_size/1024),'KiB')
