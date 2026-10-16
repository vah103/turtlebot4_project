import sys,cv2,numpy as np,json,collections,time
from PIL import Image
sys.path.insert(0,'/home/dell/mx072_g2_v1_r2_standalone_20261016')
from mapex_g2_v1.planner import G2Snapshot,G2Config,_room,_navigable,_reach,_candidate_goals,ProbabilisticVisibility
from pathlib import Path
ROOT=Path('/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_R2_CAUSAL_ROLLOUT_20261016')
RUN=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
a=np.load(ROOT/'cycle_02_input.npz');o=a['observed'];m=a['mean'];v=a['variance']
s=np.load(ROOT/'cycle_04_semantic.npz')['semantic'];pose=(198,150)
snap=G2Snapshot(o,m,v,s,pose)
room=_room(snap);cfg=G2Config();leg=_navigable(snap,room,cfg)
parents,dists=_reach(pose,leg)
vis=ProbabilisticVisibility(snap,room,cfg)
vantages=_candidate_goals(snap,room,leg,dists,vis,cfg)
source=np.load(ROOT/'cycle_02_postscan.npz');init=source['initial_room'];target=init&(o==1)
gt=np.asarray(Image.open(RUN/'gt_map.png'))[500:703,500:763,0]==255
t0=time.perf_counter();angles=np.linspace(0,2*np.pi,2500,endpoint=False)[:,None]
T=np.arange(801)[None,:]*.25;seq=np.arange(801)[None,:]
def ray(pos):
 r,c=pos
 y=np.rint(r+np.cos(angles)*T).astype(np.int32)
 x=np.rint(c+np.sin(angles)*T).astype(np.int32)
 ok=(y>=0)&(y<203)&(x>=0)&(x<263)
 y=np.clip(y,0,202);x=np.clip(x,0,262)
 hit=np.where(~ok|gt[y,x],seq,801).min(axis=1)
 before=(seq<hit[:,None])&ok
 seen=np.zeros(o.shape,bool);seen[y[before],x[before]]=1
 ix=np.where(hit<801)[0]
 if len(ix):
  ey=y[ix,hit[ix]];ex=x[ix,hit[ix]]
  valid=ok[ix,hit[ix]]&gt[ey,ex]
  seen[ey[valid],ex[valid]]=1
 return seen&target
scored=[];allseen=np.zeros(o.shape,bool)
for i,p in enumerate(vantages):
 seen=ray(p);allseen|=seen
 k=int(seen.sum());free=int((seen&~gt).sum())
 scored.append({'rc':list(p),'xy_m':[p[1]*.1,p[0]*.1],'route_m':dists[p]*.1,'new_unknown':k,'new_actual_free':free})
 if i%40==0:print('progress',i,'of',len(vantages),'best',max(x['new_unknown'] for x in scored),'elapsed',round(time.perf_counter()-t0,2),flush=True)
scored.sort(key=lambda x:(-x['new_unknown'],x['route_m']))
result={'qa_type':'GT-only retrospective feasibility diagnostic; not a goal plan',
  'vantage_source':'currently knownfree reachable R2 sampled candidates', 'vantages_evaluated':len(vantages),
  'remaining_original_unknown':int(target.sum()),
  'remaining_actual_free':int((target&~gt).sum()),
  'remaining_actual_wall':int((target&gt).sum()),
  'max_sensed_union_all_vantage_positions':int(allseen.sum()),
  'remaining_after_union':int((target&~allseen).sum()),
  'remaining_actual_free_after_union':int((target&~allseen&~gt).sum()),
  'top_vantage_by_true_scan':scored[:15],'elapsed_s':round(time.perf_counter()-t0,2)}
(ROOT/'final_GT_retrospective_vantage_audit.json').write_text(json.dumps(result,indent=2))
print('FINAL',json.dumps(result),flush=True)
