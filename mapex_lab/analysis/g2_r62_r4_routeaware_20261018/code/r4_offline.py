"""R4 independent causal analysis:
 A: recreate native WFD threshold/representatives/inflated path eligibility.
 B: estimate marginal observed-room UNKNOWN visible along candidate GLOBAL transit.
 C: compare R2 vs R3 viewpoint from same snapshot, with preservation of coverage.
Only observed lidar + same-epoch LaMa prediction + RoomNode. Never GT or future for planning.
"""
import json,sys,time,os
from pathlib import Path
import numpy as np
import cv2
from scipy.ndimage import binary_dilation,convolve,label,generate_binary_structure
HERE=Path(__file__).parent
R=Path('/home/dell/DELL_NATIVE2D/runs')
R2=R/'NR-G2R62-03/native_g2_r62_general_r2_run01/G2_R62_GENERAL_R2_RUN01_visvarprob'
R3=R/'NR-G2R62R3-02/native_g2_r62_r3_native01/G2_R62_R3_NATIVE01_visvarprob'
sys.path.insert(0,str(R/'NR-G2R62-03'))
from g2_r6_watchman import RoomWatchman,_KnownFreePaths,_PredictedSight,R6Config
from r3_frontier import _pts,_line_to_frontier,connected_frontier
# Use exact params of MapEx native WFD (sim_utils.py) without requiring GPU.
NATIVE_WFD_MIN_EXCLUSIVE=10
NATIVE_INFLATION_DIAM=3
NATIVE_BORDER_PAD=500
NATIVE_EDGE_KERNEL=np.ones((3,3),np.int8);NATIVE_EDGE_KERNEL[1,1]=0

def indata(b,epoch):
 x=np.load(b/f'room_r41/inference/decision_{epoch:06d}_input.npz')
 sem=np.load(b/f'room_r41/inference/decision_{epoch:06d}_semantic.npz')['semantic']
 d=json.loads((b/f'room_r41/decisions/decision_{epoch:06d}.json').read_text())
 pose=tuple((np.array(d['robot_rc'])-500).tolist())
 return x['observed'],x['mean'],x['variance'],sem,pose,d
def room_at(obs,sem,pos):
 from cv2 import connectedComponents
 n,lab=cv2.connectedComponents(((sem==1)&(obs!=2)).astype(np.uint8),connectivity=4)
 if lab[pos]==0:return np.zeros_like(obs,bool)
 return lab==lab[pos]

def native_inflated(obs):
 # Native Mapper inflates observed walls (1.0) by 3x3; FREE and UNKNOWN
 # remain traversable to native GLOBAL, but G2 LOCAL additionally requires
 # observed FREE and in-room.
 walls=binary_dilation(obs==2,structure=np.ones((NATIVE_INFLATION_DIAM,)*2,bool))
 return ~walls

def strict_free_nav(obs,room,pose):
 safe=native_inflated(obs)&(obs==0)&room
 # Native A* must traverse the same connected safe component.
 n,lab=cv2.connectedComponents(safe.astype(np.uint8),connectivity=4)
 return safe & (lab==lab[pose]) if lab[pose] else np.zeros_like(safe,bool)

def official_wfd(obs,room):
 # Simulate G1's scoped padded map (out-of-room pixels forced occupied).
 scoped=np.ones_like(obs,np.float32)
 scoped[room]=np.where(obs[room]==0,0,np.where(obs[room]==1,.5,1))
 edges=(convolve((scoped==.5).astype(np.int8),NATIVE_EDGE_KERNEL,mode='reflect')>0)&(scoped==0)
 # G1 WFD uses 8-neighbor connectivity + minimum size EXCLUSIVE > 10.
 comps,n=label(edges,structure=generate_binary_structure(2,2))
 groups=[]
 for c in range(1,n+1):
  pts=np.argwhere(comps==c)
  centroid=pts.mean(axis=0)
  p=pts[np.argmin(np.linalg.norm(pts-centroid,axis=1))]
  groups.append((len(pts),tuple(map(int,p)),tuple(map(int,pts.min(axis=0))),tuple(map(int,pts.max(axis=0)))))
 return sorted(groups,reverse=True),edges

def audit_exit(b,epoch):
 obs,mean,var,sem,pose,meta=indata(b,epoch)
 room=room_at(obs,sem,pose); nav=strict_free_nav(obs,room,pose)
 groups,rawedge=official_wfd(obs,room)
 candidate_rows=[]
 for size,repr_pt,low,high in groups:
  within=bool(room[repr_pt])
  observed_free=bool(obs[repr_pt]==0)
  inflated_free=bool(native_inflated(obs)[repr_pt])
  reachable=bool(nav[repr_pt])
  c={'size':size,'representative':list(repr_pt),'accepted_size':size>10,
     'inside_room':within,'observed_free':observed_free,
     'inflated_free':inflated_free,'reachable_in_native_room_scope':reachable,
     'passed_all':size>10 and within and observed_free and reachable}
  candidate_rows.append(c)
 unknown=(obs==1)&room
 # Sensor-aided alternative: native-safe viewpoints (observed FREE) which
 # can see *actual frontier unknown*, not all UNKNOWN behind predicted walls.
 frontier,components=connected_frontier(obs,room,nav)
 fp=_pts(frontier)
 # Use a cheap generic witness selection from safe poses ADJACENT to each
 # frontier component. Do not count each raw UNKNOWN as a goal.
 recovery=[]
 paths=_KnownFreePaths(nav)
 for comp in components:
  pts=np.array(comp)
  if len(pts)==0:continue
  mask=np.zeros_like(frontier)
  mask[pts[:,0],pts[:,1]]=True
  near=binary_dilation(mask,structure=np.array([[0,1,0],[1,1,1],[0,1,0]],bool)) & nav
  poss=_pts(near)
  feasible=[]
  for p in poss:
   path=paths.path(pose,p)
   if path is None:continue
   covers=sum(_line_to_frontier(obs,p,f,200) for f in comp)
   feasible.append((int(covers),len(path)-1,p))
  if feasible:
   # Prefer maximum actual frontier LOS, break ties by path length.
   best=sorted(feasible,key=lambda x:(-x[0],x[1],x[2]))[0]
   recovery.append({'cluster_unknown_count':len(comp),
                    'nearby_safe_viewpoint_count':len(feasible),
                    'candidate':list(best[2]),
                    'knownfree_path_m':round(best[1]*.1,3),
                    'frontier_cells_line_of_sight_at_goal':best[0],
                    'path_fully_native_safe':True})
  else:
   recovery.append({'cluster_unknown_count':len(comp),
                    'nearby_safe_viewpoint_count':0,'candidate':None})
 return {'name':'R3_D10_EXIT','decision':epoch,'tick':meta['native_tick'],
         'observed_room_unknown':int(unknown.sum()),'official_local_WFD_reported':meta['local_candidate_count'],
         'native_WFD_reconstructed':candidate_rows,
         'native_WFD_reconstructed_count':sum(x['passed_all'] for x in candidate_rows),
         'raw_frontier_unknown':len(fp),'reachable_native_FREE_count':int(nav.sum()),
         'recovery_frontier_components':recovery,
         'recoverable_components':sum(x['candidate'] is not None for x in recovery),
         'assumption':'same-epoch P1 observed map, 3x3 wall dilation, 8-neighbor WFD size>10; no GT',
         'policy':'SUSPEND_LOCAL_EXIT only if G1 official LOCAL WFD is empty and independently certified accessible observed FREE viewpoint can see remaining real frontier; else GLOBAL + unresolved marker'}
def trial_candidate(b,epoch,altgoal):
 obs,mean,var,sem,pos,meta=indata(b,epoch)
 nav=strict_free_nav(obs,room_at(obs,sem,pos),pos)
 r=RoomWatchman(obs,mean,var,sem,pos,nav)
 plan=r.plan()
 original=tuple(plan['goal_sequence_core_rc'][0])
 basepath=r.paths.path(pos,original)
 altpath=r.paths.path(pos,altgoal)
 if altpath is None:return {'status':'ALT_NOT_CERTIFIED_NATIVE_FREE'}
 bc=r.rays.route(basepath);ac=r.rays.route(altpath)
 # Is alternate no worse in *unique exact* visibility masks at both nominal
 # and stress, not just scalar coverage? LaMa is a prediction; no GT.
 nom_missing=int((bc[0]&(~ac[0])).sum())
 stress_missing=int((bc[1]&(~ac[1])).sum())
 nom_new=int((ac[0]&(~bc[0])).sum())
 stress_new=int((ac[1]&(~bc[1])).sum())
 strict=(nom_missing==0 and stress_missing==0 and len(altpath)<=len(basepath))
 return {'epoch':epoch,'native_tick':meta['native_tick'],
    'original':list(original),'alternative':list(altgoal),
    'original_path_m':round((len(basepath)-1)*.1,3),
    'alternative_path_m':round((len(altpath)-1)*.1,3),
    'original_nominal_cells':int(bc[0].sum()),'alt_nominal_cells':int(ac[0].sum()),
    'original_stress_cells':int(bc[1].sum()),'alt_stress_cells':int(ac[1].sum()),
    'lost_nominal_cells':nom_missing,'lost_stress_cells':stress_missing,
    'gained_nominal_cells':nom_new,'gained_stress_cells':stress_new,
    'R4_safe_nonregression_pass':strict,
    'decision':'KEEP_R62' if not strict else 'ALT_ALLOWED_BY_VISIBILITY_DOMINANCE'}

def global_transit(b,epoch):
 obs,mean,var,sem,pos,meta=indata(b,epoch)
 room=room_at(obs,sem,pos)
 cur=RoomWatchman(obs,mean,var,sem,pos,strict_free_nav(obs,room,pos))
 local=cur.plan()
 # candidate GLOBAL WFD found by actual MapEx at the time, not future
 # decision. A global candidate is only a HYPOTHETICAL route at this stage.
 native=native_inflated(obs)&(obs==0)
 n,labels=cv2.connectedComponents(native.astype(np.uint8),connectivity=4)
 nav=(labels==labels[pos]) if labels[pos] else np.zeros_like(native,bool)
 paths=_KnownFreePaths(nav)
 cfg=R6Config()
 # Raycaster for all observable POSES, but value accrued ONLY in the
 # predicted ROOM currently accepted. Paths may go outside the room.
 sensor=_PredictedSight(obs,mean,var,np.ones_like(room,bool),room&(obs==1),cfg)
 results=[]
 for full in meta.get('original_global_frontier_rc',[]):
  core=tuple(np.array(full)-500)
  if not (0<=core[0]<obs.shape[0] and 0<=core[1]<obs.shape[1]):continue
  if not nav[core]:continue
  path=paths.path(pos,core)
  if not path:continue
  v=sensor.route(path)
  results.append({'goal_core':list(core),'native_global_path_m':round((len(path)-1)*.1,2),
    'room_unknown_nominal_observable':int(v[0].sum()),
    'room_unknown_stress_observable':int(v[1].sum()),
    'new_viewing_value_per_m':round(int(v[0].sum())/max(.1,(len(path)-1)*.1),2)})
 results.sort(key=lambda v:v['native_global_path_m'])
 leg=tuple(local['goal_sequence_core_rc'][0])
 lpath=paths.path(pos,leg)
 lvis=sensor.route(lpath)
 return {'name':'R3_D16_GLOBAL_TRANSIT','decision':epoch,'tick':meta['native_tick'],
    'original_local_first_goal_core':list(leg),
    'local_first_leg_native_path_m':round((len(lpath)-1)*.1,3),
    'local_first_leg_pred_room_nominal':int(lvis[0].sum()),
    'local_first_leg_pred_room_stress':int(lvis[1].sum()),
    'certified_global_candidate_count':len(results),
    'global_candidate_trajectories':results[:25],
    'scope':'Only immediate single-leg candidate A* on currently observed FREE with wall inflation. Prediction is NOT realized visibility, and candidates are NOT necessarily selected by MapEx.',
    'policy':'DO NOT early-exit solely because a hypothetical GLOBAL transit can see predicted room UNKNOWN; require safe LOCAL frontier settlement + actual next GLOBAL goal and native path.'}
def run():
 started=time.perf_counter()
 result={}
 result['A']=audit_exit(R3,10)
 for ep,tgt in [(3,(180,133)),(5,(186,203))]:
  result[f'C_D{ep:02d}']=trial_candidate(R3,ep,tgt)
 result['B']=global_transit(R3,16)
 result['elapsed_s']=round(time.perf_counter()-started,3)
 out=HERE/'evidence'/'R4_OFFLINE_CAUSAL_A_B_C.json'
 out.write_text(json.dumps(result,indent=2,ensure_ascii=False,default=lambda x:int(x) if isinstance(x,np.integer) else float(x))+'\n')
 print('COMPLETE',out,'elapsed',result['elapsed_s'])
 for k in ['A','C_D03','C_D05']:
  print(k,json.dumps(result[k],ensure_ascii=False,default=lambda x:int(x) if isinstance(x,np.integer) else float(x))[:4600])
 print('B',json.dumps(result['B'],ensure_ascii=False,default=lambda x:int(x) if isinstance(x,np.integer) else float(x))[:5700])
if __name__=='__main__':run()
