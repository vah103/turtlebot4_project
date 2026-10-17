"""Run10 LOCAL backstop: G2-owned observed-free viewpoints; WFD is only an exit gate.

Inputs: same-epoch observed/LaMa/RoomNode and the native known-free A* mask.
No frontier coordinates, WFD scores or hidden ground truth are accepted here.
Only return an observed FREE location reachable on the native scoped grid.
"""
from __future__ import annotations
from collections import deque
import cv2
import numpy as np
from mapex_g2_r3.structural import accepted_component
from mapex_g2_r3.sequence import _Visibility,SequenceConfig

def generate_local_viewpoint(obs,mean,var,semantic,robot,mission,native_walkable,
                             recent_goals=(), max_candidates=36):
    robot=tuple(map(int,robot))
    assert obs[robot]==0
    room=accepted_component(obs,semantic,robot)
    nav=(obs==0)&room&np.asarray(native_walkable,bool)
    nav[robot]=True
    h,w=obs.shape
    # Only native-reachable current knownfree cells.
    parent={robot:None};distance={robot:0}
    queue=deque([robot])
    while queue:
        u=queue.popleft()
        for n in ((u[0]+1,u[1]),(u[0]-1,u[1]),(u[0],u[1]+1),(u[0],u[1]-1)):
            if 0<=n[0]<h and 0<=n[1]<w and nav[n] and n not in parent:
                parent[n]=u;distance[n]=distance[u]+1;queue.append(n)
    # WFD is not queried here. Score by LaMa-predicted visibility of unknown
    # room/near-room pixels from knownfree viewing poses.
    inspect=np.uint8(room)
    inspect=cv2.dilate(inspect,np.ones((25,25),np.uint8))>0
    unknown=(obs==1)&(inspect|mission)
    if not unknown.any():return None
    cfg=SequenceConfig(coarse_beams=120,coarse_ray_step_cells=1.,range_m=15.)
    ray=_Visibility(obs,mean,var,(inspect|mission),unknown,cfg,120,1.)
    # Use spatially diverse candidates; add nearest support for recently
    # discovered room subregions. Never sample an actual WFD candidate list.
    points=[p for p in parent if p!=robot and distance[p]>=3 and
            p[0]%4==robot[0]%4 and p[1]%4==robot[1]%4]
    if not points:return None
    nearby=cv2.boxFilter(np.uint8(unknown),cv2.CV_32F,(25,25),normalize=False)
    points.sort(key=lambda p:(-float(nearby[p])+0.15*distance[p],p))
    sectors=set();shortlist=[]
    for p in points:
        sector=(p[0]//10,p[1]//10)
        if sector in sectors:continue
        sectors.add(sector);shortlist.append(p)
        if len(shortlist)>=max_candidates:break
    if not shortlist:return None
    recent=[tuple(map(int,x)) for x in recent_goals[-10:]]
    best=None
    for p in shortlist:
        a,b=ray.sight(p)
        nominal=a.bit_count()
        pessimistic=b.bit_count()
        quality=.7*nominal+.3*pessimistic
        # Score target viewing potential, path cost and repetition.
        repeat_penalty=sum(max(0.,4.-np.hypot(p[0]-old[0],p[1]-old[1]))*25 for old in recent)
        score=quality-.23*distance[p]-repeat_penalty
        candidate=(score,quality,p,distance[p],nominal,pessimistic)
        if best is None or (score,-distance[p],p)>(best[0],-best[3],best[2]):
            best=candidate
    # If predictive rays offer no robust gain, keep room LOCAL and use a
    # G2-generated uncertainty-proximity viewpoint. This is NOT a WFD goal:
    # uncertainty geometry is computed independently and no frontier
    # coordinates, center costs or WFD masks enter this function.
    proximity_used=False
    if best is None or best[1]<1:
        proximity_used=True
        dist_to_unknown=cv2.distanceTransform(np.uint8(~unknown),cv2.DIST_L2,3)
        # Rank ALL native-connected FREE poses, not only a raycast shortlist.
        poses=(q for q in parent if distance[q]>=3)
        def rank(q):
            near=sum(max(0.,16.-np.hypot(q[0]-old[0],q[1]-old[1])) for old in recent)
            return (10*float(dist_to_unknown[q])+.035*distance[q]+4.5*near,
                    distance[q],q)
        p=min(poses,key=rank,default=None)
        if p is None:return None
        pathsteps=distance[p]
        nominal=pessimistic=0;quality=0.
        score=-rank(p)[0]
    else:
        score,quality,p,pathsteps,nominal,pessimistic=best
    assert obs[p]==0 and nav[p] and p in parent
    return {'goal_core_rc':list(p),'generator':('G2_UNCERTAINTY_REPOSITION_NO_FRONTIER_GOALS' if proximity_used else 'G2_VISIBILITY_BACKSTOP_NO_FRONTIER_GOALS'),
            'predicted_visible_unknown_weighted':round(quality,3),
            'predicted_visible_unknown_nominal':int(nominal),
            'predicted_visible_unknown_stress':int(pessimistic),
            'native_knownfree_route_steps':int(pathsteps),
            'score':round(score,3),'reachable_native_knownfree':True,
            'used_WFD_for_goal_generation':False,
            'no_raycast_gain_fallback':bool(proximity_used)}
