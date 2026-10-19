#!/home/dell/miniforge3/envs/lama/bin/python
"""G2 global frontier order shadow: no simulator, GT, future states, or native policy changes.
Compare S->N->F vs S->F->N on SAME observation/LaMa/native A* at each epoch.
Asymmetric endpoints marked, plus common terminal F and a third CURRENT WFD E.
"""
import sys,json,math,time
from pathlib import Path
from functools import lru_cache
import numpy as np
from scipy.ndimage import binary_dilation,distance_transform_cdt
sys.path.insert(0,'/home/dell/DELL_NATIVE2D/vendor')
sys.path.insert(0,'/home/dell/DELL_NATIVE2D/build')
import pyastar2d
from g2_pathwise_union_r2 import _predicted_visible_unknown_at
BASE=Path('/home/dell/DELL_NATIVE2D/runs')
OUT=Path('/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018')
N_BEAM=180
SAMPLE_STRIDE=50 # one scan proxy each sensor_radius/4, for 20m lidar
RADIUS=200
CELL_M=.1
MAX_NEAR=6
def source(n):
 root=next((BASE/n).glob('native_*/*'))/'room_r41'
 ds=sorted((json.loads(f.read_text()) for f in (root/'decisions').glob('decision_*.json') if 'fallback' not in f.name),key=lambda d:d['native_tick'])
 return root,ds
def snapshot(root,t):
 with np.load(root/'steps'/('tick_%06d_postscan.npz'%t)) as z:return z['observed'][500:703,500:763]
def check_path(root,dec,path):
 q=json.loads((root/'steps'/('tick_%06d_plan.json'%dec['native_tick'])).read_text())
 native=np.asarray(q['real_native_astar_rc'],dtype=int)-500
 return bool(len(native)==len(path) and np.array_equal(native,path))
def line_samples(path):
 n=len(path);idx=list(range(0,n,SAMPLE_STRIDE))
 if idx[-1]!=n-1:idx.append(n-1)
 return [(int(path[k,0]),int(path[k,1])) for k in idx]
def analyze(root,dec):
 t=int(dec['native_tick'])
 if t==0:return dict(decision=dec['decision'],tick=t,reason='NO_PREVIOUS_MAP')
 obs=snapshot(root,t-1)
 x=obs.copy()
 x[binary_dilation(obs>.5,structure=np.ones((3,3)))]=1.
 native=np.clip(10-distance_transform_cdt(x==0),1,10).astype(np.float32)
 native[x==1]=np.inf
 native=np.ascontiguousarray(native)
 WFD=dec['frontiers']
 if len(WFD)<2:return dict(decision=dec['decision'],tick=t,reason='ONE_WFD',n_wfd=len(WFD))
 g=[np.array([u['row']-500,u['col']-500],dtype=np.int32) for u in WFD]
 scores=[float(u['cost']) for u in WFD]
 S=np.asarray(dec['robot_rc'],np.int32)-500
 F=int(np.argmin(scores))
 coords=[S]+g
 # Native A* cost weight used exactly: unknown remains passable as original MapEx.
 @lru_cache(maxsize=None)
 def leg(a,b):
  if a==b:return np.array([coords[a]],np.int32)
  ca,cb=coords[a],coords[b]
  if not (np.isfinite(native[tuple(ca)]) and np.isfinite(native[tuple(cb)])):return None
  p=pyastar2d.astar_path(native,ca,cb,allow_diagonal=False)
  return p
 def m(a,b):
  path=leg(a,b)
  return math.inf if path is None else (len(path)-1)*CELL_M
 def weightcost(a,b):
  p=leg(a,b)
  return math.inf if p is None else float(native[p[1:,0],p[1:,1]].sum())
 base=leg(0,F+1)
 if base is None:return dict(decision=dec['decision'],tick=t,reason='BASELINE_GOAL_UNREACHABLE')
 path_match=check_path(root,dec,base)
 if not path_match:return dict(decision=dec['decision'],tick=t,reason='NATIVE_PATH_MISMATCH',
    n_logged_wfd=len(WFD),baseline_index=F,goal_core=g[F].tolist())
 # Select N using CURRENT native path lengths, NOT Euclidean or future.
 candidates=sorted([(m(0,i+1),i) for i in range(len(g)) if i!=F and np.isfinite(m(0,i+1)) and m(0,i+1)<m(0,F+1)],key=lambda z:(z[0],z[1]))
 # All 6 nearest are compared; no distance or map-specific threshold.
 near=candidates[:MAX_NEAR]
 if not near:
  return dict(decision=dec['decision'],tick=t,
              reason='NO_CLOSER_NATIVE_ASTAR_FRONTIER',
              n_wfd=len(WFD),native_Astar_path_exact=path_match,
              original_goal_core=g[F].tolist())
 with np.load(root/'inference'/('decision_%06d_input.npz'%dec['decision'])) as z:
  saved_ob=z['observed'];mean=z['mean'];var=z['variance']
 np.testing.assert_array_equal(saved_ob==0,obs==0)
 np.testing.assert_array_equal(saved_ob==2,obs==1)
 cache={}
 def seen_path(path):
  key=(tuple(path[0]),tuple(path[-1]))
  if key in cache:return cache[key]
  observed=np.zeros(obs.shape,dtype=np.uint8)
  for py,px in line_samples(path):
   view=_predicted_visible_unknown_at(py,px,obs,mean,var,
           np.ones(obs.shape,np.bool_),N_BEAM,RADIUS)
   observed|=view
  cache[key]=observed.astype(np.bool_)
  return cache[key]
 unknown=(obs==.5)&(var>0)
 v=np.where(unknown,var,0).astype(np.float32)
 visibility_base=seen_path(base)
 expected_base=float(v[visibility_base].sum())
 rows=[]
 for st,i in near:
  j=i+1;Fj=F+1
  nf=leg(j,Fj);fn=leg(Fj,j);sn=leg(0,j)
  if nf is None or fn is None or sn is None:continue
  nfirst=m(0,j)+m(j,Fj)
  ffirst=m(0,Fj)+m(Fj,j)
  # Weighted A* may have a geometrically shorter two-leg route than direct.
  detour=nfirst-m(0,Fj)
  # Matched endpoint F: far-first must return N->F.
  farfirst_same_endpoint=ffirst+m(j,Fj)
  # path expected variance union: S->F vs S->N->F
  vus=seen_path(sn)|seen_path(nf)
  new_info=vus&~visibility_base
  extra_expected=float(v[new_info].sum())
  new_unknown=int(np.count_nonzero(new_info&unknown))
  # Information F->N and N->F could overlap, compare union across route orders.
  far_v=visibility_base|seen_path(fn)
  union_diff_count=int(np.count_nonzero((vus!=far_v)&unknown))
  # Same endpoint E (third CURRENT original WFD) tests that route ending matters.
  third=[k for k in range(len(g)) if k!=F and k!=i and np.isfinite(m(k+1,Fj))]
  endpoints=[]
  if third:
   # closest to far and highest original-score E. Report both; no lookahead selection.
   choices=[]
   by_ff=min(third,key=lambda k:(m(Fj,k+1),k))
   by_score=min(third,key=lambda k:(scores[k],k))
   for k in (by_ff,by_score):
    if k not in choices:choices.append(k)
   for k in choices:
    E=k+1
    near_with_E=nfirst+m(Fj,E)
    far_with_E=ffirst+m(j,E)
    if np.isfinite(near_with_E) and np.isfinite(far_with_E):
     endpoints.append(dict(E_index=k,E_rc=g[k].tolist(),
       E_rule='closest_to_far' if k==by_ff else 'next_best_original_score',
       near_first_with_E_m=round(near_with_E,2),far_first_with_E_m=round(far_with_E,2),
       near_first_saving_m=round(far_with_E-near_with_E,2)))
  rows.append(dict(N_index=i,N_core=g[i].tolist(),N_original_score=round(scores[i],6),
    s_near_native_m=round(st,2),s_far_native_m=round(m(0,Fj),2),
    near_first_end_far_m=round(nfirst,2),
    far_first_end_near_m=round(ffirst,2),
    far_first_return_near_then_far_m=round(farfirst_same_endpoint,2),
    near_first_saved_same_far_terminal_m=round(farfirst_same_endpoint-nfirst,2),
    incremental_detour_if_near_clears_without_return_m=round(detour,2),
    round_trip_later_to_near_from_far_m=round(2*m(Fj,j),2),
    return_required_break_even_probability=round(max(0.,detour/max(2*m(Fj,j),1e-9)),4),
    extra_new_unknown_visible_on_near_detour=new_unknown,
    extra_predicted_visible_variance=round(extra_expected,6),
    original_far_predicted_visible_variance=round(expected_base,6),
    union_mask_order_difference_cells=union_diff_count,
    any_native_path_through_unknown=bool(any(np.any(obs[z[:,0],z[:,1]]==.5) for z in (sn,nf,fn))),
    common_terminal_third_frontier=endpoints))
 # best by informative proportionality, but shadow only (no online action).
 info_rank=sorted(rows,key=lambda q:(-q['extra_predicted_visible_variance']/max(q['incremental_detour_if_near_clears_without_return_m'],.1),q['N_index']))
 return dict(decision=dec['decision'],tick=t,room_id=dec['room_id'],robot_core=S.tolist(),n_wfd=len(WFD),
    original_first_index=F,original_goal_core=g[F].tolist(),
    original_frontier_score=float(scores[F]),native_Astar_path_exact=True,
    sensor_shadow_ray_count=N_BEAM,shadow_scan_stride_cells=SAMPLE_STRIDE,
    native_mapex_unknown_passable=True,
    candidate_choices=len(rows),candidate_shortlist_rule='up to 6 closest native-A* reachable other current WFD, strictly closer than native-selected far frontier',
    shortlist=rows,recommendation_shadow=info_rank[0]['N_index'] if info_rank else None)
def main():
 cases={}
 for n in ['NR-RN-04','NR-G2PVU-02','NR-G2PVU-03']:
  root,ds=source(n)
  recs=[]
  for d in ds:
   if d['mode']!='GLOBAL':continue
   tic=time.time()
   r=analyze(root,d)
   r['elapsed_s']=round(time.time()-tic,3)
   recs.append(r)
   chosen=r.get('shortlist',[])
   if chosen:
    # print near N that offers strongest delayed-return regret in common-terminal F.
    best=max(chosen,key=lambda z:z['near_first_saved_same_far_terminal_m'])
    watch=max(chosen,key=lambda z:z['extra_predicted_visible_variance']/max(z['incremental_detour_if_near_clears_without_return_m'],.1))
    print(n,'D',d['decision'],'tick',d['native_tick'],
         'nWFD',len(d['frontiers']),'orig',r['original_goal_core'],
         'regret_N',best['N_core'],'same_terminal_saving_m',best['near_first_saved_same_far_terminal_m'],
         'best_info_N',watch['N_core'],'new_unknown',watch['extra_new_unknown_visible_on_near_detour'],
         'expected_var',watch['extra_predicted_visible_variance'],
         'dt',r['elapsed_s'],flush=True)
   else:print(n,'D',d['decision'],r.get('reason'),'dt',r['elapsed_s'],flush=True)
  cases[n]=recs
 f=OUT/'NR_GLOBAL_NEAR_FIRST_VS_FAR_FIRST_SHADOW.json'
 f.write_text(json.dumps(dict(
  tool_version='GLOBAL_PAIR_ORDER_SHADOW_R1',
  limits='REPLAY-only, no goals commanded or GT. Only currently observed map + same-epoch LaMa and original official WFD. Costmap exactly reproduces chosen native A* (3x3 inflation+DT cost, UNKNOWN treated passable as original). Sampled probability-ray approximation not exact sensor; no future scans, no route completion assumption. Compare S->N->F vs S->F->N (unequal endpoint), and S->N->F vs S->F->N->F (same endpoint F); also two possible third currently-known WFD E to expose endpoint dependence. A lower route cost does NOT imply the near frontier remains needed after F. Shadow recommendation diagnostic not an online policy.',
  ray_count=N_BEAM,stride_cells=SAMPLE_STRIDE,
  cases=cases),ensure_ascii=False,indent=2)+'\n')
 print('SAVED',f,flush=True)
if __name__=='__main__':main()
