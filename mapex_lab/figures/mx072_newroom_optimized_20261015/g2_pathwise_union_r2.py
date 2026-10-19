"""G2-PVU-R2: online 2-viewpoint pathwise visible-variance union.

Inputs strictly same-epoch observed P1/LaMa mean/variance, G1 official WFD
candidates and original clearance-weighted native A* planner. No GT, no
look-ahead LiDAR scans or hindsight, no fixed New Room coordinates.

The route through two viewpoints is a *scoring hypothesis*: only the first
valid original frontier is dispatched. The second may vanish after a real scan.
"""
from __future__ import print_function
import math
import time
import numpy as np
from numba import njit
import pyastar2d


@njit(cache=True)
def _predicted_visible_unknown_at(oy, ox, obs, mean, var, roommask, n_ray, radius):
    """Rays through same-epoch prediction, geometric approximation of native
    probabilistic raycast: probability of at least one obstacle hit >0.8 stops.
    Count each UNKNOWN at most once even across rays. No GT input.
    """
    h,w=obs.shape
    mask=np.zeros((h,w),dtype=np.uint8)
    if oy<0 or ox<0 or oy>=h or ox>=w:
        return mask
    for beam in range(n_ray):
        angle=(2.*math.pi*beam)/n_ray
        uy=math.sin(angle);ux=math.cos(angle)
        survive=1.
        py=-1;px=-1
        for radius_step in range(1,radius+1):
            y=int(round(oy+uy*radius_step))
            x=int(round(ox+ux*radius_step))
            if y<0 or y>=h or x<0 or x>=w:break
            if y==py and x==px:continue
            py=y;px=x
            # G1 restricts any planned local observation to its room;
            # do not fabricate visible pixels across room boundary.
            if not roommask[y,x]:break
            observed=obs[y,x]
            if observed>=.99:break
            if observed==.5:
                pocc=mean[y,x]
                if pocc<0.:pocc=0.
                if pocc>1.:pocc=1.
                survive*=1.-pocc
                if survive<=.2:break
                if var[y,x]>0.:mask[y,x]=1
    return mask


def _native_astar(costmap, source, goal):
    s=np.asarray(source,dtype=np.int32)
    g=np.asarray(goal,dtype=np.int32)
    if not (np.isfinite(costmap[tuple(s)]) and np.isfinite(costmap[tuple(g)])):
        return None
    return pyastar2d.astar_path(np.ascontiguousarray(costmap,dtype=np.float32),
                                s,g,allow_diagonal=False)


def _path_scan_samples(path, radius):
    """Sample waypoint and en-route positions, not just goal endpoint.
    Resolution radius/5 follows sensor range, not any room-specific threshold.
    """
    stride=max(1,int(radius)//5)
    sample=[0]
    if len(path)>2:
        sample.extend(range(stride,len(path)-1,stride))
    if sample[-1]!=len(path)-1:sample.append(len(path)-1)
    return tuple((int(path[i,0]),int(path[i,1])) for i in sample)


def select_first_frontier(robot, goals, costs_original, native_costmap, observed, mean, variance,
                          scope_mask, cell_m, laser_radius_cells, n_lasers, arrival_radius_m):
    """Online receding-horizon PVU scorer for LOCAL room frontier group size <=2.

    All pair variants compare expected unique predicted-visible variance PER
    NATIVE-A* PATH METRE; co-visibility is set-union, never sum of endpoints.
    Both legs use exactly original costmap and A* (clearance weighting unchanged).
    Choose only FIRST native-valid goal; do NOT enact second planning hypothesis.
    """
    started=time.time()
    obs=np.asarray(observed,dtype=np.float32)
    m=np.asarray(mean,dtype=np.float32)
    v=np.asarray(variance,dtype=np.float32)
    mask=np.asarray(scope_mask,dtype=np.bool_)
    r=np.asarray(robot,dtype=np.int32)
    gs=np.asarray(goals,dtype=np.int32).reshape((-1,2))
    orig=np.asarray(costs_original,dtype=np.float64).reshape(-1)
    native=np.asarray(native_costmap,dtype=np.float32)
    n=len(gs)
    if any(a.shape!=obs.shape for a in (m,v,mask,native)):
        raise ValueError('G2_PVU_SHAPES_OR_ALIGNMENT')
    if len(orig)!=n or n==0 or not np.isfinite(orig).all():
        raise ValueError('G2_PVU_BAD_CANDIDATES')
    if not (np.isfinite(m).all() and np.isfinite(v).all()):
        raise ValueError('G2_PVU_BAD_LAMA')
    if not (0<=r[0]<obs.shape[0] and 0<=r[1]<obs.shape[1] and mask[tuple(r)]):
        raise ValueError('G2_PVU_BAD_ROBOT')
    if n_lasers<1 or laser_radius_cells<1:
        raise ValueError('G2_PVU_INVALID_SENSOR')
    bas=int(np.argmin(orig))
    # Preserve original native arrival handling: never deliberately dispatch a
    # target already within the configured 1-m native goal-arrival distance.
    valid=[]
    paths={}
    for i in range(n):
        g=gs[i]
        if not mask[tuple(g)] or obs[tuple(g)]!=0.:continue
        if np.linalg.norm((g-r).astype(float))*cell_m<=arrival_radius_m:
            continue
        p=_native_astar(native,r,g)
        if p is None or len(p)<2:continue
        valid.append(i);paths[i]=p
    if not valid:
        raise RuntimeError('G2_PVU_NO_NATIVE_VALID_FRONTIER')
    # Predict only current-map UNKNOWN and only currently G1-confirmed ROOM.
    unknown=(obs==.5)&mask&(v>0.)
    weights=np.zeros_like(v,dtype=np.float32)
    weights[unknown]=v[unknown]
    # cache visibility at every sampled path position, to make pair evaluation
    # affordable. Both endpoints and en-route sensing are included.
    cache={}
    def visible_at(pos):
        if pos not in cache:
            a=_predicted_visible_unknown_at(pos[0],pos[1],obs,m,v,mask,
                                            int(n_lasers),int(laser_radius_cells))
            cache[pos]=a.astype(np.bool_)
        return cache[pos]
    def union_for_path(p):
        seen=np.zeros(obs.shape,np.bool_)
        for pos in _path_scan_samples(p,laser_radius_cells):
            seen |= visible_at(pos)
        return seen
    prefix={i:union_for_path(paths[i]) for i in valid}
    firstgain={i:float(weights[prefix[i]].sum()) for i in valid}
    original_use=bas if bas in valid else min(valid,key=lambda i:(orig[i],i))
    best_key=None;best=None;alternatives=[]
    # Option 1: one goal, second not forced or required.
    if len(valid)==1:
        for i in valid:
            dist=(len(paths[i])-1)*cell_m
            info=firstgain[i]
            quality=info/dist if dist>0 else 0.
            key=(-quality, orig[i], i, -1)
            if best_key is None or key<best_key:
                best_key=key;best=dict(first=i,second=None,score=quality,distance_m=dist,gain=info)
    # Option 2: ordered distinct pair; native A* through both, second is
    # tentative. Only first is executed. No Hamiltonian/TSP obligation.
    for i in valid:
        for j in valid:
            if i==j:continue
            p=_native_astar(native,gs[i],gs[j])
            if p is None or len(p)<2:continue
            second=union_for_path(p)
            joint=prefix[i]|second
            gain=float(weights[joint].sum())
            dist=(len(paths[i])+len(p)-2)*cell_m
            quality=gain/dist if dist>0 else 0.
            item=dict(first=i,second=j,score=quality,distance_m=dist,
                      gain=gain,first_gain=firstgain[i],
                      new_second_gain=max(0.,gain-firstgain[i]))
            alternatives.append(item)
            key=(-quality,orig[i],i,j)
            if best_key is None or key<best_key:
                best_key=key;best=item
    if best is None:
        # Unreachable second leg: score the sole valid first-leg alternative
        # rather than forcing a pair. Treat as a guarded fallback, not a planned
        # two-viewpoint policy. Such fallback is audited.
        i=min(valid,key=lambda k:(-firstgain[k]/max(1e-9,(len(paths[k])-1)*cell_m),orig[k],k))
        best=dict(first=i,second=None,
                  score=firstgain[i]/max(1e-9,(len(paths[i])-1)*cell_m),
                  distance_m=(len(paths[i])-1)*cell_m,gain=firstgain[i])
    pair_forced_when_available=len(valid)>=2 and bool(alternatives)
    winner=int(best['first'])
    result=dict(
        policy='G2_PVU_R2_PAIR_ONLY_OPTIONAL_SECOND',scope='LOCAL_ONLY',
        first_index=winner,second_index=best['second'],
        baseline_index=bas,n_eligible=len(valid),n_candidates=n,
        projected_score=float(best['score']),
        projected_first_path_m=round((len(paths[winner])-1)*cell_m,4),
        projected_group_distance_m=round(best['distance_m'],4),
        projected_unique_visible_variance=float(best['gain']),
        projected_second_incremental_variance=float(best.get('new_second_gain',0.)),
        original_mapex_first_cost=float(orig[winner]),
        old_frontier_index_if_invalid=int(original_use),
        quality_by_first={str(i):round(firstgain[i]/((len(paths[i])-1)*cell_m),5) for i in valid},
        first_visible_variance={str(i):round(firstgain[i],5) for i in valid},
        candidate_path_m={str(i):round((len(paths[i])-1)*cell_m,4) for i in valid},
        pair_evaluations=len(alternatives),
        pair_selected=best.get("second") is not None,
        pair_forced_when_available=pair_forced_when_available,
        cache_visibility_poses=len(cache),
        runtime_s=round(time.time()-started,4),
        no_future_gt_input=True,
    )
    return winner,result
