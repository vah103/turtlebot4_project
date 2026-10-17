"""G2-R4.1: native 2D room **motion-action** search (offline, advisory).

Not a goal-list re-ranker: states expand over primitive traversals on observed FREE.
No ground truth, no future scans, no robot commands, no learned probabilities.
LaMa only scores hypothetical visibility; NEVER certifies traversability.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from time import perf_counter
from typing import Optional
import numpy as np
from mapex_g2_r3.sequence import StructuralRoutePlanner, SequenceConfig
from mapex_g2_r3.structural import _bfs, _route, sha
from r4_adaptive import R4Config, EvidenceLedger, PathValue

_DIRECTIONS = ((1,0),(0,1),(-1,0),(0,-1))

@dataclass(frozen=True)
class MotionConfig:
    action_lengths_cells: tuple[int,...] = (5,10,15)
    beam_width: int = 16
    max_depth: int = 11
    max_expansions: int = 1700
    rejoin_weight: float = 1.0
    turn_cost_equiv: float = 0.8
    max_execution_m: float = 0.5
    tile_cells: int = 10
    coarse_beams: int = 250
    dense_beams: int = 900
    min_new_conservative_cells: int = 1
    resolution_m: float = 0.1
    speed_mps: float = 0.3
    cost_per_second: float = 0.6
    nominal_weight: float = 0.7

    def __post_init__(self):
        if not self.action_lengths_cells or any(v<1 for v in self.action_lengths_cells):
            raise ValueError('invalid action lengths')
        if min(self.beam_width,self.max_depth,self.max_expansions,self.tile_cells,self.coarse_beams,self.dense_beams)<1:
            raise ValueError('invalid budgets')
        if self.max_execution_m<=0 or self.resolution_m<=0 or self.speed_mps<=0:
            raise ValueError('invalid physical proxy')
        if not 0<=self.nominal_weight<=1 or self.cost_per_second<0 or self.rejoin_weight<0:
            raise ValueError('invalid scoring weights')

@dataclass(frozen=True)
class Move:
    heading: int
    path: tuple[tuple[int,int], ...]
    nominal_bits: int
    conservative_bits: int
    novel_nominal: int
    novel_conservative: int
    turn_quarters: int

@dataclass(frozen=True)
class Node:
    pose: tuple[int,int]
    heading: Optional[int]
    nominal: int
    conservative: int
    distance_cells: int
    quarters: int
    route: tuple[tuple[int,int],...]
    moves: tuple[Move,...]
    quality: float
    robust: float
    score: float
    return_cells: int

class LocalMotionPlanner:
    def __init__(self,obs,mean,variance,semantic,robot,memory,region_report, *,
                 anchor=None, motion_cfg=None, evidence=None):
        self.cfg=motion_cfg or MotionConfig()
        self.cfg_r4=R4Config(resolution_m=self.cfg.resolution_m,speed_mps=self.cfg.speed_mps,
                           nominal_weight=self.cfg.nominal_weight,cost_per_second=self.cfg.cost_per_second,
                           max_execution_m=self.cfg.max_execution_m)
        self.robot=tuple(int(v) for v in robot)
        self.anchor=tuple(int(v) for v in (anchor if anchor is not None else robot))
        self.adapter=StructuralRoutePlanner(obs,mean,variance,semantic,self.robot,memory,region_report,
             SequenceConfig(resolution_m=self.cfg.resolution_m,coarse_beams=self.cfg.coarse_beams,
                            dense_beams=self.cfg.dense_beams))
        self.obs=obs
        self.free=self.adapter.legal
        if not (0<=self.anchor[0]<obs.shape[0] and 0<=self.anchor[1]<obs.shape[1] and
                self.free[self.anchor]):
            raise ValueError('rejoin anchor is not in observed-knownfree room')
        self.parent_to_anchor,self.dist_to_anchor=_bfs(self.anchor,self.free)
        if self.robot not in self.dist_to_anchor:
            raise ValueError('anchor not reachable on known-free graph')
        self.scorer=PathValue(self.adapter.major,self.adapter.region_bits,evidence or EvidenceLedger(),self.cfg_r4)
        self.unknown_count=int(self.adapter.unknown.sum())
        self._move_masks={}
        self.source_hash=region_report['snapshot_sha256']
        if sha(obs)!=self.source_hash:
            raise ValueError('stale map source')

    def _local_edge(self,src,direction,length):
        dy,dx=_DIRECTIONS[direction]
        path=tuple((src[0]+k*dy,src[1]+k*dx) for k in range(length+1))
        h,w=self.obs.shape
        for r,c in path:
            if r<0 or c<0 or r>=h or c>=w or not self.free[r,c] or self.obs[r,c]!=0:
                return None
        return path

    def _visibility(self,path,ray):
        # Every action includes scan poses along the traversed path, not just its endpoint.
        key=(path[0],path[-1],ray.beams)
        if key not in self._move_masks:
            self._move_masks[key]=ray.along(path)
        return self._move_masks[key]

    def _evaluate(self, nom,cons,travel_cells,turns,return_cells):
        gn=self.scorer.gain(nom); gc=self.scorer.gain(cons)
        quality=self.cfg.nominal_weight*gn+(1-self.cfg.nominal_weight)*gc
        time=travel_cells*self.cfg.resolution_m/self.cfg.speed_mps
        return_time=return_cells*self.cfg.resolution_m/self.cfg.speed_mps
        cost=self.cfg.cost_per_second*(time+self.cfg.rejoin_weight*return_time) + self.cfg.turn_cost_equiv*turns
        return quality, gc - cost, quality - cost

    def _expand(self,st,ray):
        for direction in range(4):
            turns=0 if st.heading is None else min((direction-st.heading)%4,(st.heading-direction)%4)
            for length in self.cfg.action_lengths_cells:
                path=self._local_edge(st.pose,direction,length)
                if path is None:continue
                target=path[-1]
                if target not in self.dist_to_anchor:continue
                # Prevent pointless immediate reversal; allow reaching same tile by new useful route.
                if st.moves and target==st.moves[-1].path[0]:continue
                vnom,vcons=self._visibility(path,ray)
                nnew=vnom & ~st.nominal
                cnew=vcons & ~st.conservative
                total_nom=st.nominal|vnom
                total_cons=st.conservative|vcons
                d=st.distance_cells+length
                qturns=st.quarters+turns
                ret=self.dist_to_anchor[target]
                qual,rob,score=self._evaluate(total_nom,total_cons,d,qturns,ret)
                move=Move(direction,path,vnom,vcons,nnew,cnew,turns)
                yield Node(target,direction,total_nom,total_cons,d,qturns,
                           st.route+path[1:],st.moves+(move,),qual,rob,score,ret)

    def _beam_select(self,states):
        # Score-rank distinct coverage hypotheses first; reserve spatial diversity
        # so low-reward transit segments can lead to later useful observations.
        unique={}
        for st in states:
            key=(st.pose,st.heading,st.nominal,st.conservative)
            if key not in unique or st.score>unique[key].score:
                unique[key]=st
        ranked=sorted(unique.values(),key=lambda v:(-v.score,-v.robust,v.distance_cells,v.route))
        chosen=[];taken=set()
        # First: best distinct spatial tiles/directions. No hard-coded viewpoint.
        for st in ranked:
            cell=(st.pose[0]//self.cfg.tile_cells,st.pose[1]//self.cfg.tile_cells,st.heading)
            if cell not in taken:
                taken.add(cell);chosen.append(st)
                if len(chosen)>=max(1,self.cfg.beam_width//2):break
        ids={id(s) for s in chosen}
        for st in ranked:
            if len(chosen)>=self.cfg.beam_width:break
            if id(st) not in ids:
                ids.add(id(st));chosen.append(st)
        return chosen

    def _best(self,states):
        feasible=[s for s in states if s.moves and s.robust>0 and
                  s.conservative.bit_count()>=self.cfg.min_new_conservative_cells]
        if not feasible:return None
        return max(feasible,key=lambda s:(s.score,s.robust,-s.distance_cells))

    def _dense_recheck(self,candidates):
        # Independent denser raycast of the top diverse *action routes*.
        ray=self.adapter.fine
        if ray is None:
            from mapex_g2_r3.sequence import _Visibility
            self.adapter.fine=_Visibility(self.obs,self.adapter.mean,self.adapter.variance,
                   self.adapter.memory.mission,self.adapter.unknown,self.adapter.cfg,
                   self.cfg.dense_beams,self.adapter.cfg.dense_ray_step_cells)
            ray=self.adapter.fine
        checked=[]
        for candidate in candidates:
            nom=cons=0
            for move in candidate.moves:
                vn,vc=ray.along(move.path)
                nom|=vn;cons|=vc
            quality,robust,score=self._evaluate(nom,cons,candidate.distance_cells,
                                                candidate.quarters,candidate.return_cells)
            checked.append((candidate,nom,cons,quality,robust,score))
        return checked

    def solve(self):
        t=perf_counter()
        root=Node(self.robot,None,0,0,0,0,(self.robot,),(),0.,0.,0.,self.dist_to_anchor[self.robot])
        if not self.adapter.major:
            return {'status':'DEFER_NO_MAJOR_REGIONS','action_sequence':[],'robot_command':None}
        states=[root];best_by_depth=[];all_states=[];expansions=0;reason='MAX_SEARCH_DEPTH'
        for depth in range(1,self.cfg.max_depth+1):
            successors=[]
            for st in states:
                for candidate in self._expand(st,self.adapter.coarse):
                    successors.append(candidate);expansions+=1
                    if expansions>=self.cfg.max_expansions:break
                if expansions>=self.cfg.max_expansions:break
            if not successors:
                reason='NO_MORE_VALID_ACTIONS';break
            states=self._beam_select(successors)
            all_states.extend(states)
            top=self._best(states)
            best_by_depth.append({'depth':depth,'survivors':len(states),
                                   'best_score':round(top.score,3) if top else None,
                                   'best_conservative_score':round(top.robust,3) if top else None})
            if expansions>=self.cfg.max_expansions:
                reason='MAX_EXPANSIONS';break
        # Dense verification is independent of coarse visibility and includes
        # terminal return cost (not free-travel or free-map completion).
        candidates=sorted((s for s in all_states if s.moves),key=lambda z:z.score,reverse=True)
        shortlist=[];seen=set()
        for st in candidates:
            key=(st.pose,st.distance_cells//5,st.moves[0].heading)
            if key in seen:continue
            seen.add(key);shortlist.append(st)
            if len(shortlist)>=10:break
        checked=self._dense_recheck(shortlist)
        checked.sort(key=lambda row:(row[5],row[4],-row[0].distance_cells),reverse=True)
        winner=next((row for row in checked if row[4]>0 and row[2].bit_count()>=self.cfg.min_new_conservative_cells),None)
        if winner is None:
            return {'version':'G2_R41_MOTION_SEARCH_V1','status':'DEFER_NO_POSITIVE_ACTION_ROUTE',
                    'decision':'DEFER_ROOM','action_sequence':[],
                    'robot_command':None,'observed_snapshot_sha256':self.source_hash,
                    'search':{'depths':best_by_depth,'expansions':expansions,'end_reason':reason,
                              'elapsed_s':round(perf_counter()-t,3)}}
        st,nom,cons,quality,rob,score=winner
        step_cells=max(1,int(self.cfg.max_execution_m/self.cfg.resolution_m))
        first=st.route[:step_cells+1]
        return_route=tuple(_route(self.parent_to_anchor,self.anchor,st.pose)[::-1])
        result={
            'version':'G2_R41_MOTION_SEARCH_V1','status':'CONDITIONAL_MOTION_ROUTE',
            'decision':'EXECUTE_SHORT_PREFIX_ADVISORY','observed_snapshot_sha256':self.source_hash,
            'robot_rc':list(self.robot),'rejoin_anchor_rc':list(self.anchor),
            'action_sequence':[{'step':i+1,'direction':m.heading,
                    'from_rc':list(m.path[0]),'to_rc':list(m.path[-1]),
                    'distance_m':round((len(m.path)-1)*self.cfg.resolution_m,3),
                    'turn_quarters':m.turn_quarters,
                    'new_nominal_cells_at_time_t':m.novel_nominal.bit_count(),
                    'new_conservative_cells_at_time_t':m.novel_conservative.bit_count()}
                   for i,m in enumerate(st.moves)],
            'route_rc':[list(q) for q in st.route],
            'execution_prefix_rc':[list(q) for q in first],
            'return_route_rc':[list(q) for q in return_route],
            'distance_outbound_m':round(st.distance_cells*self.cfg.resolution_m,3),
            'distance_return_m':round(st.return_cells*self.cfg.resolution_m,3),
            'nominal_unknown_union_cells':nom.bit_count(),
            'conservative_unknown_union_cells':cons.bit_count(),
            'weighted_structural_value':round(quality,4),
            'objective':round(score,4),'conservative_objective':round(rob,4),
            'search':{'depths':best_by_depth,'expansions':expansions,
                'coarse_scan_poses':len(self.adapter.coarse.cache),
                'dense_scan_poses':len(self.adapter.fine.cache),
                'shortlisted_dense':len(checked),
                'end_reason':reason,'elapsed_s':round(perf_counter()-t,3)},
            'model_calls_increment':0,'ground_truth_used':False,
            'future_observation_used':False,'source_r3_goals_used':False,
            'fixed_goal_count':False,'robot_command':None,'policy_action':None,
            'physical_safety_certified':False,'native_mapex_rejoin_verified':False,
            'cautions':['Movement always stays in observed free (inflation proxy), not predicted-free',
                 'Remaining modeled route is conditional; only execution prefix is a proposal',
                 'Return path anchor must be externally supplied to establish real MapEx handoff',
                 'Scores and LaMa scenarios are uncalibrated proxies',
                 'No intermediate observations simulated by this R4.1 planner']}
        return result
