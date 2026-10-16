#!/usr/bin/env python3
"""Retrospective GT-hidden LiDAR evaluation of already frozen G2-R3 VP1 sequence.

GT is read ONLY here as hidden environment sensor + after-the-fact evaluation,
not used to select/reorder/fix any of the three plan goals.
This is a fixed-sequence diagnostic, NOT event-triggered replanning or MapEx native.
"""
from pathlib import Path
import json,numpy as np,cv2
from PIL import Image
ROOT=Path('/home/dell/mx072_g2_r3_sequence_optimizer_20261016')
OUT=ROOT/'results_final'
RUN=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
p=json.load(open(OUT/'R3_SEQUENCE_VP1.json'))
assert p['goal_count']==3
original=np.load(RUN/'room_r41/inference/decision_000002_input.npz')['observed']
sem=np.load(RUN/'room_r41/inference/decision_000002_semantic.npz')['semantic']
_,labs=cv2.connectedComponents(((sem==1)&(original!=2)).astype('uint8'),connectivity=4)
room=labs==labs[147,111]
obs=np.load('/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_MANUAL_G2_FULL_CHAIN_20261016/stage_01_postscan.npz')['observed'].copy()
assert np.array_equal(obs,np.load('/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_MANUAL_G2_FULL_CHAIN_20261016/stage_01_inference.npz')['observed'])
gt=np.asarray(Image.open(RUN/'gt_map.png'))[500:703,500:763,0]==255
theta=np.linspace(0,2*np.pi,2500,endpoint=False)[:,None]
steps=np.arange(801)[None,:]*.25
ix=np.arange(801)[None,:]
def lidar(site):
 y,x=site
 yy=np.rint(y+np.cos(theta)*steps).astype(np.int32)
 xx=np.rint(x+np.sin(theta)*steps).astype(np.int32)
 valid=(yy>=0)&(yy<203)&(xx>=0)&(xx<263)
 cy=np.clip(yy,0,202);cx=np.clip(xx,0,262)
 stop=np.where((~valid)|gt[cy,cx],ix,801).min(axis=1)
 visible=(ix<stop[:,None])&valid
 free=np.zeros(obs.shape,bool);wall=np.zeros(obs.shape,bool)
 free[cy[visible],cx[visible]]=True
 hitids=np.flatnonzero(stop<801)
 if hitids.size:
  hy=cy[hitids,stop[hitids]];hx=cx[hitids,stop[hitids]]
  iswall=valid[hitids,stop[hitids]]&gt[hy,hx]
  wall[hy[iswall],hx[iswall]]=True
 free&=~wall
 return free,wall

at=(168,135);initial_unk=(room&(obs==1)).sum()
summary={'scope':'R3 final snapshot at VP1; goals frozen prior to retrospective GT sensor replay',
         'future_or_GT_in_goal_choice':False,'sensor_rays':2500,
         'sensor_max_range_m':20.,'scan_spacing_m':.5,
         'initial_unknown_original_room_after_VP1':int(initial_unk),
         'original_room_unknown_initial_D02':int((room&(original==1)).sum()),
         'outcome':'PENDING','stages':[]}
total_dist=0;total_scans=0
for i,stage in enumerate(p['selected']['stages'],1):
 route=[tuple(q) for q in stage['path_rc']]
 assert route[0]==at
 assert route[-1]==tuple(p['goal_sequence'][i-1])
 if any(obs[q]!=0 for q in route):
  summary['outcome']='BLOCKED_BY_NEW_KNOWN_OBSTACLE'
  summary['block_stage']=i
  break
 if any(gt[q] for q in route):
  summary['outcome']='GT_COLLISION_ON_PLANNED_PATH'
  summary['block_stage']=i
  break
 scan=[route[j] for j in range(5,len(route),5)]
 if not scan or scan[-1]!=route[-1]:scan.append(route[-1])
 prior=obs.copy()
 for site in scan:
  free,wall=lidar(site)
  unknown=obs==1
  obs[unknown&free]=0
  obs[unknown&wall]=2
 at=route[-1]
 length=(len(route)-1)*.1;total_dist+=length;total_scans+=len(scan)
 left=room&(obs==1)
 e={'stage':i,'goal_xy_m':stage['goal_xy_m'],
 'distance_leg_m':round(length,2),'cumulative_distance_m':round(total_dist,2),
 'scan_count':len(scan),'new_original_room_observed':int((room&(prior==1)&(obs!=1)).sum()),
 'new_original_room_free':int((room&(prior==1)&(obs==0)).sum()),
 'new_original_room_wall':int((room&(prior==1)&(obs==2)).sum()),
 'remaining_original_room_unknown':int(left.sum()),
 'remaining_GT_free_unknown_EVAL_ONLY':int((left&~gt).sum()),
 'remaining_GT_wall_unknown_EVAL_ONLY':int((left&gt).sum())}
 summary['stages'].append(e)
 np.savez_compressed(OUT/f'R3_VP1_STAGE_{i:02d}_POSTSCAN.npz',observed=obs,original_room=room,robot_rc=np.asarray(at))
 print('STAGE',json.dumps(e),flush=True)
else:summary['outcome']='SEQUENCE_COMPLETED'
summary['total_distance_m']=round(total_dist,2);summary['total_scans']=total_scans
summary['original_room_unknown_end']=int((room&(obs==1)).sum())
summary['original_room_GT_free_unknown_end_EVAL_ONLY']=int((room&(obs==1)&~gt).sum())
summary['not_native']=True
summary['not_event_triggered_replan']=True
(OUT/'R3_VP1_FROZEN_CHAIN_SENSOR_EVAL.json').write_text(json.dumps(summary,indent=2))
print('FINAL',json.dumps({k:v for k,v in summary.items() if k!='stages'}),flush=True)
