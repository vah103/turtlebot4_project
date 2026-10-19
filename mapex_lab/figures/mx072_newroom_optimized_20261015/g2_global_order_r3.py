"""G2-R3 Global Conditional Frontier Sequence — immutable precommit.
GLOBAL only; LOCAL exactly native MapEx+G1 Run04.
No GT, past/future snapshots, manual room waypoints, or native A* modification.
Predicted residual ratio is a HEURISTIC, never calibrated probability.
"""
from __future__ import print_function
import time
import numpy as np
import pyastar2d
from numba import njit
import math

RAYS=180
SAMPLE_STRIDE=50
MAX_NEAR=6

@njit(cache=True)
def _vis_one(y0,x0,obs,mean,var,ray_count,reach):
    h,w=obs.shape
    mask=np.zeros((h,w),np.uint8)
    for k in range(ray_count):
        angle=2.*math.pi*k/ray_count
        sy=math.sin(angle);sx=math.cos(angle)
        survival=1.
        ly=-1;lx=-1
        for r in range(1,reach+1):
            y=int(round(y0+sy*r));x=int(round(x0+sx*r))
            if y<0 or y>=h or x<0 or x>=w:break
            if y==ly and x==lx:continue
            ly=y;lx=x
            if obs[y,x]>=.99:break
            if obs[y,x]==.5:
                p=mean[y,x]
                if p<0.:p=0.
                if p>1.:p=1.
                survival*=1.-p
                if survival<=.2:break
                if var[y,x]>0.:mask[y,x]=1
    return mask

def select_global(robot,goals,original_costs,native_cost,observed,mean,variance,
                  cell_m=.1,reach=200,arrival_radius_m=1.):
    start=time.time()
    obs=np.asarray(observed,dtype=np.float32)
    m=np.asarray(mean,dtype=np.float32)
    v=np.asarray(variance,dtype=np.float32)
    cost=np.ascontiguousarray(native_cost,dtype=np.float32)
    pts=np.asarray(goals,dtype=np.int32).reshape(-1,2)
    r=np.asarray(robot,dtype=np.int32)
    orig=np.asarray(original_costs,dtype=np.float64).reshape(-1)
    n=len(pts)
    if n<1 or len(orig)!=n or any(z.shape!=obs.shape for z in (m,v,cost)):
        raise ValueError('G2R3_BAD_ALIGNMENT')
    if not (np.isfinite(orig).all() and np.isfinite(m).all() and np.isfinite(v).all()):
        raise ValueError('G2R3_BAD_INPUT_VALUES')
    if not all(0<=int(r[k])<obs.shape[k] for k in (0,1)):
        raise ValueError('G2R3_BAD_ROBOT')
    F=int(np.argmin(orig))
    result=dict(policy='G2_R3_GLOBAL_CONDITIONAL_SEQUENCE',
       baseline_index=F,selected_index=F,enabled=True,
       decision='BASELINE_FALLBACK',reason='NO_CERTIFIED_GAIN',
       n_candidates=n,trial_count=0,near_candidate_rows=[],
       ray_count=RAYS,sample_stride=SAMPLE_STRIDE,
       no_future_or_gt_online=True,
       residual_ratio_is_calibrated_probability=False)
    if n<3:
        result['reason']='FEWER_THAN_THREE_WFD'
        return F,result
    if not np.isfinite(cost[tuple(r)]):
        result['reason']='ROBOT_BLOCKED_IN_NATIVE_COST'
        return F,result
    cache={}
    def astar(i,j):
        key=(i,j)
        if key not in cache:
            a=r if i==-1 else pts[i]
            b=r if j==-1 else pts[j]
            if not (np.isfinite(cost[tuple(a)]) and np.isfinite(cost[tuple(b)])):
                cache[key]=None
            else:
                cache[key]=pyastar2d.astar_path(cost,a,b,allow_diagonal=False)
        return cache[key]
    def dist(i,j):
        p=astar(i,j)
        if p is None:return np.inf
        return float((len(p)-1)*cell_m)
    base_path=astar(-1,F)
    if base_path is None:
        result['reason']='ORIGINAL_GOAL_PATH_UNAVAILABLE'
        return F,result
    d_sf=dist(-1,F)
    options=sorted([(dist(-1,i),i) for i in range(n) if i!=F and dist(-1,i)<d_sf
                    and dist(-1,i)>arrival_radius_m],key=lambda x:(x[0],x[1]))[:MAX_NEAR]
    if not options:
        result['reason']='NO_CLOSER_WFD'
        return F,result
    weights=np.where(obs==.5,np.maximum(v,0),0).astype(np.float32)
    vis_cache={}
    def path_vis(path):
        key=(int(path[0,0]),int(path[0,1]),int(path[-1,0]),int(path[-1,1]))
        if key not in vis_cache:
            out=np.zeros(obs.shape,dtype=np.bool_)
            idx=list(range(0,len(path),SAMPLE_STRIDE))
            if idx[-1]!=len(path)-1:idx.append(len(path)-1)
            for p in idx:
                y,x=map(int,path[p])
                out |= _vis_one(y,x,obs,m,v,RAYS,int(reach)).astype(np.bool_)
            vis_cache[key]=out
        return vis_cache[key]
    visible_F=path_vis(base_path)
    best=None
    for d_sn,N in options:
        p_sn=astar(-1,N);p_nf=astar(N,F);p_fn=astar(F,N)
        if p_sn is None or p_nf is None or p_fn is None:continue
        # Guard: do not introduce a new G2 leg through currently UNKNOWN.
        # Original F leg may still pass UNKNOWN as allowed by native planner.
        clean=bool(np.all(obs[p_sn[:,0],p_sn[:,1]]==0.) and
                   np.all(obs[p_nf[:,0],p_nf[:,1]]==0.))
        if not clean:
            result['near_candidate_rows'].append(dict(index=int(N),reason='ADDED_LEG_UNKNOWN'))
            continue
        d_nf=dist(N,F);d_fn=dist(F,N)
        added=path_vis(p_sn)|path_vis(p_nf)
        unique=added & ~visible_F
        incr=float(weights[unique].sum())
        gain_near=float(weights[added].sum())
        if gain_near<=0 or incr<=0:
            result['near_candidate_rows'].append(dict(index=int(N),reason='NO_INCREMENTAL_OBSERVATION'))
            continue
        residual_ratio=min(1.,max(0.,incr/gain_near))
        e_candidates=[k for k in range(n) if k not in (F,N) and np.isfinite(dist(F,k)) and np.isfinite(dist(N,k))]
        if len(e_candidates)<2:
            result['near_candidate_rows'].append(dict(index=int(N),reason='INSUFFICIENT_CONTINUATION_ANCHORS'))
            continue
        E_close=min(e_candidates,key=lambda k:(dist(F,k),k))
        E_score=min(e_candidates,key=lambda k:(orig[k],k))
        anchors=list(dict.fromkeys([E_close,E_score]))
        if len(anchors)<2:
            result['near_candidate_rows'].append(dict(index=int(N),reason='CONTINUATION_ANCHORS_COLLAPSE'))
            continue
        d_near_first=d_sn+d_nf
        detour=d_near_first-d_sf
        scenario=[]
        for E in anchors:
            d_fe=dist(F,E);d_ne=dist(N,E)
            near=d_near_first+d_fe
            far_if_n_stays=d_sf+d_fn+d_ne
            far_if_n_clears=d_sf+d_fe
            # Residual ratio is an uncalibrated proxy. Conservatively choose
            # N only when it beats F across BOTH continuation anchors.
            conditional_far=far_if_n_clears + residual_ratio*max(0.,far_if_n_stays-far_if_n_clears)
            margin=conditional_far-near
            scenario.append(dict(continuation_index=int(E),near_first_m=round(near,4),
              far_first_if_N_stays_m=round(far_if_n_stays,4),
              far_first_if_N_clears_m=round(far_if_n_clears,4),
              conditional_proxy_cost_m=round(conditional_far,4),
              signed_advantage_m=round(margin,4)))
        worst=min(x['signed_advantage_m'] for x in scenario)
        row=dict(index=int(N),reason='EVALUATED',s_to_near_m=round(d_sn,4),
            s_to_far_m=round(d_sf,4),detour_to_far_m=round(detour,4),
            incremental_predicted_variance=round(incr,6),
            residual_visibility_ratio=round(residual_ratio,6),
            worst_anchor_margin_m=round(worst,4),scenarios=scenario)
        result['near_candidate_rows'].append(row)
        result['trial_count']+=1
        # Positive margin for every E; don't force a visit just because it's near.
        if worst>0:
            key=(-worst,-incr,orig[N],int(N))
            if best is None or key<best[0]:
                best=(key,int(N))
    if best is not None:
        result['selected_index']=int(best[1])
        result['decision']='CHANGE_GLOBAL_GOAL'
        result['reason']='POSITIVE_CONDITIONAL_MARGIN_AT_BOTH_ENDPOINTS'
    result['n_visibility_positions']=len(vis_cache)
    result['elapsed_s']=round(time.time()-start,4)
    return result['selected_index'],result
