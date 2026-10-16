"""G2-V1 R2: causal, room-scoped, conditional multi-viewpoint route planner.

Planner input: an ALREADY computed current observed occupancy map, current LaMa
mean/variance, current G1 V4-R4.1 semantic map, current robot pose. Never reads
GT, simulated future sensor returns, original future decisions, or previous
experiment answers. Uses conservative known-free navigation and uncertain
predicted geometry for sensing only.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field, asdict
from functools import lru_cache
from math import ceil, log, sqrt, cos, sin, pi
from time import perf_counter
from typing import Any
import hashlib

import numpy as np
import cv2


@dataclass(frozen=True)
class G2Config:
    resolution_m: float = 0.10
    lidar_range_m: float = 20.0
    planning_beams: int = 250  # search-only coarse beams
    ray_step_cells: float = 0.5
    final_beams: int = 2500  # dense verification of shortlisted full sequences
    final_ray_step_cells: float = 0.25
    fine_shortlist_per_depth: int = 5
    scan_every_m: float = 0.5
    candidate_spacing_m: float = 0.3
    clearance_m: float = 0.2
    max_candidates: int = 240  # search budget, NOT a limit on # rooms or goals
    beam_width: int = 6
    branch_width: int = 12  # 4x pathwise-diverse pool, not endpoint-stationary pruning
    # Distance/time/cell weights are CONFIGURABLE heuristic defaults, NOT fitted.
    robot_speed_mps: float = 0.25
    goal_stop_time_s: float = 1.0
    unknown_penalty_s_per_cell: float = 0.14
    risk_penalty_s_per_cell: float = 0.03
    uncertainty_weight: float = 0.5
    pessimistic_sigma: float = 1.0
    scenario_pessimistic_fraction: float = 0.3
    # Small improvements do not justify a new independent goal.
    min_net_improvement_s: float = 0.5
    min_new_weighted_cells: float = 1.0
    # replan thresholds do not assert a scientifically calibrated rule
    replan_new_occupied_cells: int = 1
    replan_new_observed_cells: int = 10
    replan_mean_change: float = 0.15
    replan_room_change_cells: int = 1

    def __post_init__(self):
        if not (self.resolution_m > 0 and self.lidar_range_m > 0 and
                self.robot_speed_mps > 0 and self.ray_step_cells > 0 and
                self.scan_every_m > 0 and self.candidate_spacing_m > 0):
            raise ValueError("resolution/range/speed/spacing must be positive")
        if not (8 <= self.planning_beams <= 2500 and 8 <= self.final_beams <= 2500):
            raise ValueError("planning_beams out of range 8..2500")
        if not (self.beam_width >= 1 and self.branch_width >= 1 and self.max_candidates >= 1 and self.fine_shortlist_per_depth >= 1 and self.final_ray_step_cells > 0):
            raise ValueError("search budgets must be positive")
        if not (0 <= self.scenario_pessimistic_fraction <= 1):
            raise ValueError("scenario_pessimistic_fraction must be in [0,1]")
        if min(self.unknown_penalty_s_per_cell, self.risk_penalty_s_per_cell,
               self.goal_stop_time_s, self.min_net_improvement_s) < 0:
            raise ValueError("cost parameters must not be negative")


@dataclass
class G2Snapshot:
    observed: np.ndarray  # MapEx numeric codes: 0=free, 1=unknown, 2=wall
    mean: np.ndarray      # LaMa occupancy prediction in [0,1]
    variance: np.ndarray
    semantic: np.ndarray  # G1/R4.1: 1=room; room is component containing robot
    robot_rc: tuple[int, int]
    frontier_rc: list[tuple[int, int]] = field(default_factory=list)
    ensemble: np.ndarray | None = None # optional (3, H, W) member probabilities
    decision_id: str = "current"


def _hash(a: np.ndarray) -> str:
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def _validate(snapshot: G2Snapshot):
    arrays = [snapshot.observed, snapshot.mean, snapshot.variance, snapshot.semantic]
    if any(x.ndim != 2 for x in arrays) or len({x.shape for x in arrays}) != 1:
        raise ValueError("observed/mean/variance/semantic must have identical 2D shape")
    obs = snapshot.observed
    if not np.isin(obs, (0, 1, 2)).all():
        raise ValueError("observed must use codes 0=FREE, 1=UNKNOWN, 2=OCCUPIED")
    if not (np.isfinite(snapshot.mean).all() and np.isfinite(snapshot.variance).all()):
        raise ValueError("NaN/inf in LaMa predictions")
    if (snapshot.mean.min() < 0 or snapshot.mean.max() > 1 or
        snapshot.variance.min() < 0):
        raise ValueError("mean must be probability [0,1], variance nonnegative")
    r,c = snapshot.robot_rc
    if not (0 <= r < obs.shape[0] and 0 <= c < obs.shape[1] and obs[r,c] == 0):
        raise ValueError("robot must be on observed FREE inside raster")
    if snapshot.ensemble is not None:
        if snapshot.ensemble.ndim != 3 or snapshot.ensemble.shape[1:] != obs.shape:
            raise ValueError("ensemble must be (models, H, W)")
        if not np.isfinite(snapshot.ensemble).all() or snapshot.ensemble.min()<0 or snapshot.ensemble.max()>1:
            raise ValueError("ensemble occupancy must be finite in [0,1]")


def _room(snapshot: G2Snapshot) -> np.ndarray:
    # Same connected-component rule used in Run04 native room integration.
    mask = ((snapshot.semantic == 1) & (snapshot.observed != 2)).astype(np.uint8)
    _, label = cv2.connectedComponents(mask, connectivity=4)
    id_ = label[snapshot.robot_rc]
    if id_ == 0:
        raise ValueError("robot is not in an accepted current Room Node (semantic==1)")
    return label == id_


def _navigable(snapshot: G2Snapshot, room: np.ndarray, config: G2Config) -> np.ndarray:
    # UNKNOWN is not drivable, including if mean predicts it FREE.
    free = snapshot.observed == 0
    radius = ceil(config.clearance_m / config.resolution_m - 1e-7)
    if radius:
        nonfree = (~free).astype(np.uint8)
        kernel = np.ones((2*radius+1, 2*radius+1), np.uint8)
        clearance = cv2.dilate(nonfree, kernel) == 0
    else:
        clearance = free.copy()
    legal = room & free & clearance
    # Robot may stand by a boundary: allow its exact currently observed FREE
    # pose as a starting node, never allow any nonfree cells in a route.
    legal[snapshot.robot_rc] = True
    return legal


def _reach(start: tuple[int,int], legal: np.ndarray):
    queue = deque([start]); parent = {start:None}; dist = {start:0}
    h,w = legal.shape
    while queue:
        y,x = queue.popleft()
        for p in ((y+1,x),(y-1,x),(y,x+1),(y,x-1)):
            if 0 <= p[0] < h and 0 <= p[1] < w and legal[p] and p not in parent:
                parent[p] = (y,x); dist[p] = dist[(y,x)]+1; queue.append(p)
    return parent,dist


def _trace(src, dst, parent):
    if dst not in parent:
        raise ValueError("point not reachable on observed FREE")
    path = [dst]
    while path[-1] != src: path.append(parent[path[-1]])
    return path[::-1]


class ProbabilisticVisibility:
    """Binary first-hit visibility per *map scenario*, never repeated credit per scan.

    If actual LaMa member maps are present, evaluate each independently. If only
    mean+variance are present, use exactly two EXPLICIT HEURISTIC proxies:
    threshold(mean, .5) and threshold(mean + sigma*sqrt(variance), .5).
    These proxy cases are not calibrated ensemble forecasts or probabilities.

    For a wall hit the cell itself is observable, then the ray terminates.
    All target unknown cells receive at most one visibility flag per scenario
    per whole sequence, including scans performed en route.
    """
    def __init__(self, snapshot:G2Snapshot, room:np.ndarray, config:G2Config,
                 beams:int|None=None, ray_step_cells:float|None=None):
        self.snapshot,self.room,self.config=snapshot,room,config
        self.target=(snapshot.observed==1)&room
        self.target_idx=np.flatnonzero(self.target.ravel())
        prob=np.clip(snapshot.mean,0,1)
        entropy=np.zeros_like(prob,dtype=np.float32)
        mid=(prob>1e-7)&(prob<1-1e-7)
        entropy[mid]=-(prob[mid]*np.log2(prob[mid])+(1-prob[mid])*np.log2(1-prob[mid]))
        unc=np.minimum(1,np.sqrt(np.maximum(snapshot.variance,0))/.5)
        self.weights=(1+config.uncertainty_weight*np.maximum(entropy,unc))[self.target].astype(np.float32)
        self.weight_sum=float(self.weights.sum())
        # Only unknown predictions may be blocked speculatively; known walls always block.
        if snapshot.ensemble is not None:
            maps=[a for a in snapshot.ensemble]
            self.scenario_mode='ENSEMBLE_MEMBERS'
            self.scenario_prob=np.ones(len(maps),np.float32)/len(maps)
        else:
            maps=[snapshot.mean, np.clip(snapshot.mean+config.pessimistic_sigma*np.sqrt(
                np.maximum(snapshot.variance,0)),0,1)]
            self.scenario_mode='MEAN_VARIANCE_TWO_PROXY_MAPS_NOT_CALIBRATED'
            self.scenario_prob=np.asarray([1-config.scenario_pessimistic_fraction,
                                            config.scenario_pessimistic_fraction],dtype=np.float32)
        self.blocks=np.stack([((snapshot.observed==2)|((snapshot.observed==1)&(m>=.5))|~room)
                              for m in maps],axis=0)
        self.n_scenarios=len(maps)
        self.beams=beams if beams is not None else config.planning_beams
        step=ray_step_cells if ray_step_cells is not None else config.ray_step_cells
        rng=np.arange(0,config.lidar_range_m/config.resolution_m+1e-9,step,dtype=np.float32)
        angles=np.linspace(0,2*pi,self.beams,endpoint=False,dtype=np.float32)[:,None]
        self.oy=np.rint(np.cos(angles)*rng).astype(np.int16)
        self.ox=np.rint(np.sin(angles)*rng).astype(np.int16)
        self.cache={}
        self.zero=np.zeros((self.n_scenarios,len(self.weights)),bool)

    def view(self,pos:tuple[int,int]):
        if pos in self.cache:return self.cache[pos]
        y,x=pos;h,w=self.target.shape
        yy=y+self.oy.astype(np.int32);xx=x+self.ox.astype(np.int32)
        valid=(yy>=0)&(yy<h)&(xx>=0)&(xx<w)
        yy=np.clip(yy,0,h-1);xx=np.clip(xx,0,w-1)
        # End-on-wall can be seen; later cells on the same beam cannot.
        unique=np.ones(yy.shape,bool)
        unique[:,1:]=(yy[:,1:]!=yy[:,:-1])|(xx[:,1:]!=xx[:,:-1])
        out=np.zeros_like(self.zero)
        n_steps=yy.shape[1]
        step_idx=np.arange(n_steps)[None,:]
        for k,occ in enumerate(self.blocks):
            hit=~valid|occ[yy,xx]
            earliest=np.where(hit,step_idx,n_steps).min(axis=1)
            allowed=valid&unique&(step_idx<=earliest[:,None])
            detected=np.zeros(self.target.shape,bool)
            detected[yy[allowed],xx[allowed]]=True
            out[k]=detected.ravel()[self.target_idx]
        self.cache[pos]=out
        return out

    def project_leg(self,path:list[tuple[int,int]],include_start=False):
        if not path:raise ValueError('empty path')
        stride=max(1,round(self.config.scan_every_m/self.config.resolution_m))
        ix=list(range(0 if include_start else stride,len(path),stride))
        if not ix or ix[-1]!=len(path)-1:ix.append(len(path)-1)
        union=self.zero.copy()
        for j in ix:union|=self.view(path[j])
        return union,len(ix)

    def gain(self,coverage):
        weighted=coverage.astype(np.float32)@self.weights
        expected=float(self.scenario_prob@weighted)
        # Spread between map scenarios is a heuristic risk, not new-area credit.
        gap=float(weighted.max()-weighted.min()) if len(weighted)>1 else 0.
        return expected,gap


def _candidate_goals(snapshot,room,legal,dist,vis,config):
    spacing=max(1,round(config.candidate_spacing_m/config.resolution_m))
    yy,xx=np.indices(legal.shape)
    grid=(yy%spacing==snapshot.robot_rc[0]%spacing)&(xx%spacing==snapshot.robot_rc[1]%spacing)
    pts = [tuple(map(int,x)) for x in zip(*np.where(grid & legal)) if tuple(map(int,x)) in dist]
    # Candidate source = geometric grid + observed frontier hints. Frontier
    # UNKNOWN itself is never a safe goal, so only its adjacent knownfree cells.
    for frontier in snapshot.frontier_rc:
        fy,fx=frontier
        for y,x in ((fy+1,fx),(fy-1,fx),(fy,fx-1),(fy,fx+1)):
            q=(y,x)
            if 0<=y<legal.shape[0] and 0<=x<legal.shape[1] and legal[q] and q in dist:
                pts.append(q)
    pts=list(dict.fromkeys(p for p in pts if p!=snapshot.robot_rc))
    # Resource cap is a *search budget*, with spatially distributed downsampling.
    # It cannot be interpreted as a cap on the number of rooms or goals.
    if len(pts)>config.max_candidates:
        indices=np.linspace(0,len(pts)-1,config.max_candidates,dtype=int)
        pts=[pts[i] for i in indices]
    return pts


def _cost(cfg:G2Config,vis:ProbabilisticVisibility,coverage:np.ndarray,dist_m:float,n_goals:int):
    gain,gap=vis.gain(coverage)
    remaining=max(0.,vis.weight_sum-gain)
    travel=dist_m/cfg.robot_speed_mps
    stop=n_goals*cfg.goal_stop_time_s
    risk_s=cfg.risk_penalty_s_per_cell*gap
    return {'objective_s_equivalent':float(travel+stop+cfg.unknown_penalty_s_per_cell*remaining+risk_s),
            'travel_time_s':float(travel),'goal_stop_time_s':float(stop),
            'unobserved_weight':float(remaining),'expected_observed_weight':float(gain),
            'uncertainty_gap_weight':float(gap),'uncertainty_penalty_s':float(risk_s)}


@dataclass
class _State:
    pos:tuple[int,int]
    selected:tuple[tuple[int,int],...]
    coverage:np.ndarray
    length_m:float
    steps:list[dict]
    cost:dict


def _is_better(new:_State,old:_State):
    return (new.cost['objective_s_equivalent']+1e-7 < old.cost['objective_s_equivalent'])


def _diverse(states:list[_State],limit:int):
    """Keep low-cost states without filling the beam with one spatial cluster."""
    picked=[];used=set()
    for st in states:
        cell=(st.pos[0]//8,st.pos[1]//8)
        if cell not in used:
            picked.append(st);used.add(cell)
            if len(picked)==limit:return picked
    for st in states:
        if not any(st is old for old in picked):
            picked.append(st)
            if len(picked)==limit:break
    return picked


def _pathwise_pool(states:list[_State],limit:int)->list[tuple[int,int]]:
    # Select independently from near-optimal pathwise states then spatially diverse alternatives.
    pool=[];tiles=set()
    for st in states:
        tile=(st.pos[0]//8,st.pos[1]//8)
        if tile not in tiles:
            pool.append(st.pos);tiles.add(tile)
            if len(pool)>=limit:break
    for st in states:
        if st.pos not in pool:
            pool.append(st.pos)
            if len(pool)>=limit:break
    return pool


def plan(snapshot:G2Snapshot,config:G2Config|None=None)->dict[str,Any]:
    """R2: route-first, scenario-union sequence planning with dense final validation.

    Coarse search evaluates ALL reachable 1-goal routes *pathwise* before
    pruning. Subsequent route expansion keeps different spatial groups.
    Rescore a shortlist using dense raycasts and explicit goal-deletion checks.
    Every input is from the current snapshot; no ground truth or future scans.
    """
    cfg=config or G2Config();t0=perf_counter();_validate(snapshot)
    room=_room(snapshot);legal=_navigable(snapshot,room,cfg)
    initial_parent,initial_dist=_reach(snapshot.robot_rc,legal)
    coarse=ProbabilisticVisibility(snapshot,room,cfg)
    candidate=_candidate_goals(snapshot,room,legal,initial_dist,coarse,cfg)
    n_unknown=int(((snapshot.observed==1)&room).sum())
    base_cov=coarse.zero.copy()
    base_cost=_cost(cfg,coarse,base_cov,0.,0)
    base=_State(snapshot.robot_rc,(),base_cov,0.,[],base_cost)
    all_candidates=[];layer=[base];best=base;record=[];examined=0
    parent_cache={snapshot.robot_rc:(initial_parent,initial_dist)}
    pool=None
    # No preselected fixed number of goals. Stop when all branches fail to improve.
    for depth in range(len(candidate)):
        options=[]
        for state in layer:
            if state.pos not in parent_cache:
                parent_cache[state.pos]=_reach(state.pos,legal)
            parent,dist=parent_cache[state.pos]
            proposals=candidate if depth==0 else pool
            for end in proposals:
                if end==state.pos or end in state.selected or end not in dist:continue
                path=_trace(state.pos,end,parent)
                cov,n_scans=coarse.project_leg(path)
                combined=state.coverage|cov
                length=state.length_m+dist[end]*cfg.resolution_m
                cost=_cost(cfg,coarse,combined,length,len(state.selected)+1)
                marginal=cost['expected_observed_weight']-state.cost['expected_observed_weight']
                if marginal<cfg.min_new_weighted_cells:continue
                if state.cost['objective_s_equivalent']-cost['objective_s_equivalent']<cfg.min_net_improvement_s:continue
                leg={'index':len(state.selected)+1,'goal_rc':list(end),
                     'distance_m':round(dist[end]*cfg.resolution_m,4),'scan_samples':int(n_scans),
                     'path_rc':[list(q) for q in path],
                     'marginal_expected_new_weight':round(marginal,4),
                     'cumulative_expected_new_weight':round(cost['expected_observed_weight'],4),
                     'predicted_remaining_weight':round(cost['unobserved_weight'],4),
                     'step_cost_s_equivalent':round(cost['objective_s_equivalent'],4),
                     'goal_is_observed_free':True,'route_is_observed_free':True,
                     'conditional_replan_before_next_leg':True}
                options.append(_State(end,state.selected+(end,),combined,length,state.steps+[leg],cost))
                examined+=1
        if not options:break
        options.sort(key=lambda x:(x.cost['objective_s_equivalent'],x.length_m,len(x.selected),x.selected))
        # Keep complete alternative sequences of EACH length for high-fidelity checks.
        all_candidates.extend(options[:cfg.fine_shortlist_per_depth])
        layer=_diverse(options,cfg.beam_width)
        record.append({'goal_count':depth+1,'best_objective_s_equivalent':round(options[0].cost['objective_s_equivalent'],4),
                       'length_m':round(options[0].length_m,4),
                       'expected_observed_weight':round(options[0].cost['expected_observed_weight'],4)})
        if _is_better(options[0],best):best=options[0]
        if depth==0:
            pool=_pathwise_pool(options,max(cfg.branch_width*4,24))
        # Starting with the second leg, add actual successors as possible pool members.
        else:
            pool=list(dict.fromkeys(pool + _pathwise_pool(options,cfg.branch_width)))
            pool=pool[:max(cfg.branch_width*5,30)]

    # DENSE CHECK uses same input with stronger ray resolution (no GT).
    fine=ProbabilisticVisibility(snapshot,room,cfg,beams=cfg.final_beams,
                                 ray_step_cells=cfg.final_ray_step_cells)
    dense_cache={}
    def dense_sequence(sequence:tuple[tuple[int,int],...]):
        if sequence in dense_cache:return dense_cache[sequence]
        cumulative=fine.zero.copy();steps=[];start=snapshot.robot_rc
        length=0.;previous_score=0.
        for index,end in enumerate(sequence,1):
            if start not in parent_cache:parent_cache[start]=_reach(start,legal)
            parent,dist=parent_cache[start]
            if end not in dist:return None
            path=_trace(start,end,parent)
            cov,n_scans=fine.project_leg(path)
            cumulative|=cov
            segment=dist[end]*cfg.resolution_m
            length+=segment
            cost=_cost(cfg,fine,cumulative,length,index)
            gain=cost['expected_observed_weight']-previous_score
            steps.append({'index':index,'goal_rc':list(end),
                          'distance_m':round(segment,4),'scan_samples':int(n_scans),
                          'path_rc':[list(q) for q in path],
                          'marginal_expected_new_weight':round(gain,4),
                          'cumulative_expected_new_weight':round(cost['expected_observed_weight'],4),
                          'predicted_remaining_weight':round(cost['unobserved_weight'],4),
                          'step_cost_s_equivalent':round(cost['objective_s_equivalent'],4),
                          'goal_is_observed_free':True,'route_is_observed_free':True,
                          'conditional_replan_before_next_leg':True})
            previous_score=cost['expected_observed_weight'];start=end
        final=_cost(cfg,fine,cumulative,length,len(sequence))
        out=_State(start,sequence,cumulative,length,steps,final)
        dense_cache[sequence]=out
        return out

    # All prefixes plus top coarse alternatives. Goal-deletion is checked on the
    # dense score, not just on the sparse-ray score. This is approximate search;
    # it does not certify global optimality across arbitrary candidate paths.
    sequences={st.selected for st in all_candidates}
    sequences.update(best.selected[:i] for i in range(1,len(best.selected)+1))
    for seq in tuple(sequences):
        for j in range(len(seq)):
            shortened=seq[:j]+seq[j+1:]
            if shortened:sequences.add(shortened)
    ranked=[]
    dense_base=_State(snapshot.robot_rc,(),fine.zero.copy(),0.,[],_cost(cfg,fine,fine.zero,0.,0))
    for seq in sorted(sequences,key=lambda x:(len(x),x)):
        state=dense_sequence(seq)
        if state is not None:ranked.append(state)
    ranked.sort(key=lambda x:(x.cost['objective_s_equivalent'],x.length_m,len(x.selected),x.selected))
    choice=dense_base
    if ranked and _is_better(ranked[0],choice):choice=ranked[0]
    # Iteratively delete any goal whose removal reduces dense objective;
    # compare shorter plans even if beam-search did not contain them.
    changes=0
    while len(choice.selected)>=2:
        subsets=[dense_sequence(choice.selected[:j]+choice.selected[j+1:]) for j in range(len(choice.selected))]
        subsets=[x for x in subsets if x is not None]
        subsets.sort(key=lambda x:(x.cost['objective_s_equivalent'],x.length_m,len(x.selected)))
        if subsets and _is_better(subsets[0],choice):choice=subsets[0];changes+=1
        else:break
    if n_unknown==0:status='COMPLETE_OBSERVED'
    elif choice.selected:status='CONTINUE'
    else:status='DEFER_NO_POSITIVE_SAFE_CHAIN'
    evidence={'decision_id':snapshot.decision_id,
         'observed_sha256':_hash(snapshot.observed),'mean_sha256':_hash(snapshot.mean),
         'variance_sha256':_hash(snapshot.variance),'semantic_sha256':_hash(snapshot.semantic),
         'input_includes_ground_truth':False,'input_is_current_decision_only':True,
         'ensemble_members':None if snapshot.ensemble is None else int(snapshot.ensemble.shape[0]),
         'algorithm':'G2-V1-R2 coarse pathwise diverse beam then dense scenario-union verification and goal deletion',
         'scenario_method':fine.scenario_mode,'scenario_proxy_is_not_calibrated':snapshot.ensemble is None,
         'no_global_optimum_claim':True,'configuration_is_not_calibrated':True,
         'prediction_must_be_refreshed_after_replan_trigger':True}
    return {
         'status':status,'robot_rc':list(snapshot.robot_rc),'room_cells':int(room.sum()),
         'observed_room_unknown_cells':n_unknown,
         'expected_total_unknown_weight':round(fine.weight_sum,4),
         'goal_sequence':[list(x) for x in choice.selected],
         'goal_count':len(choice.selected),'route_segments':choice.steps,
         'route_distance_m':round(choice.length_m,4),
         'cost':{k:round(v,4) for k,v in choice.cost.items()},
         'baseline_cost':{k:round(v,4) for k,v in dense_base.cost.items()},
         'search':{'candidate_goals':len(candidate),'examined_route_expansions':examined,
            'beam_width':cfg.beam_width,'branch_width':cfg.branch_width,
            'goal_count_frontier':record,'dense_shortlist_evaluated':len(dense_cache),
            'dense_goal_deletions':changes,'coarse_beams':cfg.planning_beams,
            'final_beams':cfg.final_beams,'no_fixed_goal_count':True,
            'stopped_when_no_positive_new_goal':True},
         'planning_runtime_s':round(perf_counter()-t0,3),'config':asdict(cfg),
         'provenance':evidence,
         'cautions':['No GT or future data; all results from current snapshot.',
             'Multiple goals are a conditional plan; replan after meaningful scans.',
             'Ensemble scenarios are binary map hypotheses, not calibrated posterior.',
             'Without ensemble, mean/variance proxy scenarios are heuristic.',
             'Free-grid route and raycast are approximate, not native collision proof.',
             'Search is budgeted/approximate; weights and time costs uncalibrated.',
             'COMPLETE only from observed map; prediction cannot certify completion.']}


def assess_replan(before:G2Snapshot, after_observed:np.ndarray,
                  after_mean:np.ndarray|None=None, after_semantic:np.ndarray|None=None,
                  remaining_route_rc:list[tuple[int,int]]|None=None,
                  config:G2Config|None=None)->dict:
    """Trigger evaluation. External caller MUST rerun LaMa + G1 when requested.

    No guessed predictions are synthesized from old mean map. The on-scan
    controller can call this before new mean/semantic arrays are available.
    """
    cfg=config or G2Config();_validate(before)
    old=before.observed
    if after_observed.shape!=old.shape or not np.isin(after_observed,(0,1,2)).all():
        raise ValueError("after_observed must be valid occupancy raster")
    if np.any((old!=1)&(after_observed!=old)):
        raise ValueError("new observation must not erase or rewrite observed cells")
    room=_room(before)
    newly=(old==1)&(after_observed!=1)&room
    occ=int(np.count_nonzero(newly&(after_observed==2)))
    count=int(newly.sum())
    reasons=[]
    if occ>=cfg.replan_new_occupied_cells and occ>0:reasons.append('NEW_OBSERVED_OBSTACLE')
    if count>=cfg.replan_new_observed_cells:reasons.append('SIGNIFICANT_NEW_OBSERVATION')
    route=remaining_route_rc or []
    if any(after_observed[tuple(q)]!=0 for q in route):reasons.append('PLANNED_ROUTE_NO_LONGER_OBSERVED_FREE')
    if after_mean is not None:
        if after_mean.shape!=old.shape:raise ValueError('mean shape mismatch')
        diff=(old==1)&room&(after_observed==1)
        if np.any(diff) and np.max(np.abs(after_mean[diff]-before.mean[diff]))>=cfg.replan_mean_change:
            reasons.append('LAMA_PREDICTION_CHANGED')
    if after_semantic is not None:
        after=G2Snapshot(after_observed,before.mean,before.variance,after_semantic,
                         before.robot_rc,decision_id='replan-evaluation')
        # robot may have moved after last snapshot; explicit caller must supply
        # the new pose to a fresh snapshot before actual replan.
        if after.observed[before.robot_rc]==0:
            try:
                new_room=_room(after)
                if int(np.count_nonzero(room^new_room))>=cfg.replan_room_change_cells:
                    reasons.append('ROOM_NODE_BOUNDARY_CHANGED')
            except ValueError:reasons.append('ROOM_NODE_INVALIDATED')
    return {'needs_replan':bool(reasons),'reasons':list(dict.fromkeys(reasons)),
            'new_observed_cells_in_room':count,'new_occupied_cells_in_room':occ,
            'required_action':'RERUN_LAMA_AND_ROOM_NODE_THEN_G2' if reasons else 'KEEP_CONDITIONAL_CHAIN',
            'updated_prediction_was_provided':after_mean is not None and after_semantic is not None}
