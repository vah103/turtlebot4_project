#!/usr/bin/env python3
"""Reproduce precommitted D02→VP1→G1→G2→G3, use GT only for raycast sensor returns.
Chosen goals/path were frozen before this run. No GT in selection."""
import os,json,hashlib,collections
from pathlib import Path
import numpy as np,cv2
from PIL import Image
BASE=Path('/tmp/MX072_RUN04_D02_G2A_20261016')
OUT=Path('/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_MANUAL_G2_FULL_CHAIN_20261016')
OUT.mkdir(parents=True,exist_ok=True)
RUN=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
initial=np.load(RUN/'room_r41/inference/decision_000002_input.npz')['observed'].copy()
semantic=np.load(RUN/'room_r41/inference/decision_000002_semantic.npz')['semantic']
_,label=cv2.connectedComponents(((semantic==1)&(initial!=2)).astype('uint8'),connectivity=4)
room=(label==label[147,111]);assert room.sum()==6564
truth=np.asarray(Image.open(RUN/'gt_map.png'))[500:703,500:763,0]==255
assert truth.shape==initial.shape
reference_aftervp1=np.load(BASE/'D02_VP1_RECONSTRUCTED_POSTSCAN_OBSERVED.npz')['updated_observed']
reference_g2=np.load(BASE/'VP1_DYNAMIC_G2_3GOAL_HIRES_MASKS.npz')
angles=np.linspace(0,2*np.pi,2500)[:,None]
length=np.arange(801)[None,:]*.25
seq=np.arange(801)[None,:]
def lidar(rc):
 y,x=rc
 yy=np.rint(y+np.cos(angles)*length).astype(np.int32)
 xx=np.rint(x+np.sin(angles)*length).astype(np.int32)
 valid=(yy>=0)&(yy<203)&(xx>=0)&(xx<263)
 yy=np.clip(yy,0,202);xx=np.clip(xx,0,262)
 hits=np.where(~valid|truth[yy,xx],seq,801).min(axis=1)
 prior=(seq<hits[:,None])&valid
 free=np.zeros(initial.shape,bool);wall=free.copy()
 free[yy[prior],xx[prior]]=True
 ix=np.where(hits<801)[0]
 if len(ix):
  ry=yy[ix,hits[ix]];cx=xx[ix,hits[ix]]
  real=valid[ix,hits[ix]]&truth[ry,cx]
  wall[ry[real],cx[real]]=True
 free&=~wall
 return free,wall
def find_init_path():
 observed=initial
 legal=(observed==0)&room&(cv2.dilate((observed!=0).astype('uint8'),np.ones((5,5),np.uint8))==0)
 src=(147,111);dst=(168,135)
 Q=collections.deque([src]);prev={src:None}
 while Q:
  y,x=Q.popleft()
  for next in [(y+1,x),(y-1,x),(y,x+1),(y,x-1)]:
   if next in prev or not(0<=next[0]<203 and 0<=next[1]<263) or not legal[next]:continue
   prev[next]=(y,x);Q.append(next)
   if next==dst:break
  if dst in prev:break
 assert dst in prev
 route=[dst]
 while route[-1]!=src:route.append(prev[route[-1]])
 return np.array(route[::-1],dtype=np.int16)
paths=[
('D02_TO_VP1',find_init_path()),
('VP1_TO_G1',reference_g2['leg1']),
('G1_TO_G2',reference_g2['leg2']),
('G2_TO_G3',reference_g2['leg3'])]
assert [tuple(k[1][-1]) for k in paths]==[(168,135),(180,102),(192,174),(147,207)]
old=initial.copy();total_distance=0;sum_scans=0;results=[]
for i,(name,path) in enumerate(paths,1):
 if i>1:assert tuple(path[0])==tuple(paths[i-2][1][-1])
 assert all(old[tuple(p)]==0 for p in path),(name,'route no longer knownfree')
 assert not truth[path[:,0],path[:,1]].any(),(name,'route hidden GT collides')
 poses=[tuple(path[k]) for k in range(5,len(path),5)]
 if not poses or poses[-1]!=tuple(path[-1]):poses.append(tuple(path[-1]))
 prev_obs=old.copy()
 new_free=0;new_occ=0
 for pose in poses:
  free,wall=lidar(pose)
  unk=old==1
  freshfree=unk&free;freshwall=unk&wall
  old[freshfree]=0;old[freshwall]=2
  new_free+=int(freshfree.sum());new_occ+=int(freshwall.sum())
 total_distance+=(len(path)-1)*.1
 sum_scans+=len(poses)
 nrm=room&(old==1)
 result={'stage':i,'leg':name,'start_rc':list(map(int,path[0])),'goal_rc':list(map(int,path[-1])),
 'goal_xy_m':[round(float(path[-1,1]*.1),2),round(float(path[-1,0]*.1),2)],
 'length_m':round((len(path)-1)*.1,2),'cumulative_length_m':round(total_distance,2),
 'scans':len(poses),'cumulative_scans':sum_scans,
 'new_total_free':new_free,'new_total_occ':new_occ,
 'new_in_initial_room':int(((prev_obs==1)&(old!=1)&room).sum()),
 'new_free_in_initial_room':int(((prev_obs==1)&(old==0)&room).sum()),
 'new_wall_in_initial_room':int(((prev_obs==1)&(old==2)&room).sum()),
 'unknown_in_initial_room_remaining':int(nrm.sum()),
 'remaining_true_free_eval_only':int((nrm&~truth).sum()),
 'remaining_true_wall_eval_only':int((nrm&truth).sum()),
 'realized_original_room_unknown_fraction':round(1-int(nrm.sum())/3889,6)}
 assert result['new_free_in_initial_room']+result['new_wall_in_initial_room']==result['new_in_initial_room']
 np.savez_compressed(OUT/f'stage_{i:02d}_postscan.npz',observed=old,original_room=room,
 robot_rc=path[-1],newly_observed=(prev_obs==1)&(old!=1),route_rc=path,scan_rc=np.asarray(poses,dtype=np.int16))
 results.append(result)
 print('SCAN_STAGE',json.dumps(result),flush=True)
 if i==1:
  mismatch=int(np.sum(old!=reference_aftervp1))
  print('FROZEN_VP1_OBSERVED_PARITY',mismatch,'cells',flush=True)
  assert mismatch==0,"D02→VP1 must reproduce reference before executing G1 G2 G3"
summary={'source':'D02 originally frozen VP1, subsequent 3 goals frozen at VP1 with causal LaMa & R41','ground_truth_only_sensor':True,
'GT_read_footprint_never_goal_choice':True,'scans_every_m':.5,'rays':2500,'range_m':20,
'scan_step_cell':.25,'original_room_cells':int(room.sum()),
'original_unknown':3889,'total_distance_m':round(total_distance,2),
'total_scans':sum_scans,'unknown_after_all':results[-1]['unknown_in_initial_room_remaining'],
'observed_fraction_original_unknown':results[-1]['realized_original_room_unknown_fraction'],
'stages':results}
(OUT/'fullchain_sensor_summary.json').write_text(json.dumps(summary,indent=2))
print('ALL_STAGES_DONE',flush=True)
