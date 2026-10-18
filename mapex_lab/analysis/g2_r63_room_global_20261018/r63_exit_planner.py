"""G2-R6.3 causal exit-aware room path ordering (not a global goal command).
Observation-only global frontier distance heuristic; native A* remains execution
authority. No GT, future poses, map-specific constants or manually named doors.
"""
from itertools import permutations
from collections import deque
import time
import cv2
import numpy as np
from g2_r6_watchman import RoomWatchman, _KnownFreePaths, R6Config

CROSS=cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))

def observed_external_frontiers(obs, room, robot):
    """4-connected observed-FREE component; FREE cells adjacent to UNKNOWN
    OUTSIDE accepted local room. Nothing here is a GT or future frontier."""
    obs=np.asarray(obs)
    free=(obs==0)
    _,labels=cv2.connectedComponents(free.astype(np.uint8),connectivity=4)
    comp=(labels==labels[tuple(robot)]) if labels[tuple(robot)]!=0 else np.zeros_like(free,bool)
    exterior_unknown=(obs==1)&(~room)
    frontier=comp&(~room)&(cv2.dilate(exterior_unknown.astype(np.uint8),CROSS)>0)
    return comp,frontier

def distance_to_frontier(free_connected, frontier):
    """Multi-source 4-BFS from global frontier cells. 0.1m/step length only.
    This is a lower-bound/heuristic until the actual native inflated A* checks it.
    """
    h,w=frontier.shape
    distances=np.full((h,w),-1,np.int32)
    q=deque()
    for y,x in np.argwhere(frontier):
        y=int(y);x=int(x)
        distances[y,x]=0
        q.append((y,x))
    while q:
        y,x=q.popleft()
        v=distances[y,x]+1
        for ny,nx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
            if 0<=ny<h and 0<=nx<w and free_connected[ny,nx] and distances[ny,nx]<0:
                distances[ny,nx]=v
                q.append((ny,nx))
    return distances

class RoomGlobalWatchman(RoomWatchman):
    """R6.2 Watchman candidate selection, then end-position optimization.
    Reorders the SAME selected goal set; cannot invent or walk new UNKNOWN goals.
    """
    def plan(self):
        start=time.perf_counter()
        r=super().plan()
        original=tuple(self.plan_order)
        r['policy_version']='G2_R63_ROOM_TO_GLOBAL_R1'
        r['end_position_objective']='COST_TO_OBSERVED_GLOBAL_FRONTIER'
        r['actual_global_goal_commanded']=False
        r['projected_visibility_is_not_observed']=True
        r['baseline_room_route_m']=r['predicted_route_distance_m']
        r['room_goals_before_exit_ordering']=[list(p) for p in original]
        if len(original)<2:
            r['exit_policy']='UNCHANGED_FEWER_THAN_TWO_GOALS'
            return r
        free,frontier=observed_external_frontiers(self.obs,self.room,self.robot)
        if not frontier.any():
            r['exit_policy']='NO_OBSERVED_REACHABLE_EXTERNAL_FRONTIER_KEEP_R62'
            return r
        distance=distance_to_frontier(free,frontier)
        # Compare EXACT same proposed viewpoint set. Price the likely
        # continuation outside the room, without claiming the chosen global
        # frontier is MapEx-certified (it is an estimator only).
        def cost_end(goal):
            q=int(distance[goal])
            return float(q)*self.cfg.resolution_m if q>=0 else float('inf')
        end_cost={goal:cost_end(goal) for goal in original}
        if not any(np.isfinite(x) for x in end_cost.values()):
            r['exit_policy']='FRONTIER_UNREACHABLE_FROM_GOALS_KEEP_R62'
            return r
        # Evaluate permutations using known-FREE path segment lengths first.
        # Unlike GT-assisted hindsight, the only future estimate is old LaMa
        # visibility from this same current epoch.
        nodes=(self.robot,)+original
        paths={}
        lengths={}
        for a in nodes:
            for b in original:
                if a==b:continue
                segment=self.paths.path(a,b)
                paths[(a,b)]=segment
                lengths[(a,b)]=float('inf') if segment is None else .1*(len(segment)-1)
        vals=[]
        for order in permutations(original):
            distance_room=0.;a=self.robot
            for b in order:
                c=lengths.get((a,b),float('inf'))
                distance_room+=c;a=b
            if not np.isfinite(distance_room+end_cost[order[-1]]):
                continue
            vals.append((distance_room+end_cost[order[-1]],distance_room,order))
        if not vals:
            r['exit_policy']='NO_REACHABLE_PERMUTATION_KEEP_R62'
            return r
        vals.sort(key=lambda x:(x[0],x[1],x[2]))
        initial_plan=self._route_order(original)
        if initial_plan is None: raise RuntimeError('baseline R62 route unexpectedly unreachable')
        old_visibility=self._weighted(initial_plan['cover'])
        # Check several strong cost candidates with complete pathwise raycast.
        # No arbitrary 98% coverage floor; gain/cost is a measurable scalar
        # within one uncertainty/model epoch. Equal route cover gives a clean
        # apples-to-apples comparison to the source policy.
        old_exit=end_cost[original[-1]]
        original_score=old_visibility/max(initial_plan['distance_m']+old_exit,1e-9)
        best=(original_score,old_visibility,-initial_plan['distance_m']-old_exit,original,initial_plan)
        for _,_,order in vals[:min(20,len(vals))]:
            if order==original:continue
            plan=self._route_order(order)
            if plan is None:continue
            value=self._weighted(plan['cover'])
            total=plan['distance_m']+end_cost[order[-1]]
            score=value/max(total,1e-9)
            trial=(score,value,-total,order,plan)
            if trial[:3]>best[:3]:
                best=trial
        _score,_visibility,_neg_total,chosen,plan=best
        self.plan_order=list(chosen)
        r['goal_sequence_core_rc']=[list(x) for x in chosen]
        r['legs']=plan['legs']
        r['planned_grid_route_core_rc']=[list(p) for p in plan['route']]
        r['predicted_route_distance_m']=round(plan['distance_m'],3)
        r['projected_unique_unknown_nominal']=int((plan['cover'][0]&self.mission).sum())
        r['projected_unique_unknown_stress']=int((plan['cover'][1]&self.mission).sum())
        r['projected_weighted_gain']=round(float(_visibility),3)
        r['exit_policy']='REORDERED_WITH_SAME_GOALS' if chosen!=original else 'BASELINE_ALREADY_BEST_OR_MORE_CAUSAL_COVERAGE'
        r['global_exit_frontier_cells_now_observed']=int(frontier.sum())
        r['baseline_global_exit_lowerbound_m']=round(old_exit,3)
        r['chosen_global_exit_lowerbound_m']=round(end_cost[chosen[-1]],3)
        r['baseline_total_predicted_cost_m']=round(initial_plan['distance_m']+old_exit,3)
        r['chosen_total_predicted_cost_m']=round(plan['distance_m']+end_cost[chosen[-1]],3)
        r['baseline_predicted_visibility_gain']=round(old_visibility,3)
        r['candidate_route_count_evaluated']=min(20,len(vals))
        r['score_per_total_cost_gain']=round(float(_score-original_score),5)
        r['exit_computation_s']=round(time.perf_counter()-start,4)
        r['goals_only_native_known_free']=all(self.legal[p] for p in chosen)
        self.last_decision=r
        return r