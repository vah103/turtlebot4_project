"""G2-V1: causal, room-scoped, conditional multi-viewpoint route planner.

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
    planning_beams: int = 128  # faster than native 2500; a preview only
    ray_step_cells: float = 1.0
    scan_every_m: float = 0.5
    candidate_spacing_m: float = 0.3
    clearance_m: float = 0.2
    max_candidates: int = 240  # search budget, NOT a limit on # rooms or goals
    beam_width: int = 5
    branch_width: int = 12
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
        if not (8 <= self.planning_beams <= 2500):
            raise ValueError("planning_beams out of range 8..2500")
        if not (self.beam_width >= 1 and self.branch_width >= 1 and self.max_candidates >= 1):
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
    """Expected first-hit visibility in observed-free + uncertain LaMa map.

    At cell c, the visibility probability is the product of estimated
    traversabilities of PRECEDING unknown cells. Thus a cell believed occupied
    may still be observed as a wall. Across beams and scan poses we use max,
    not independent union, to avoid double-counting correlated rays.
    """
    def __init__(self, snapshot: G2Snapshot, room: np.ndarray, config: G2Config):
        self.snapshot,self.room,self.config = snapshot,room,config
        obs = snapshot.observed
        self.target = (obs==1)&room
        self.target_idx=np.flatnonzero(self.target.reshape(-1))
        unknown = obs==1
        # Uncertainty raises the value of *testing* a cell, not believing it free.
        prob = np.clip(snapshot.mean,0,1)
        entropy = -(np.where((prob>0)&(prob<1),prob*np.log(np.maximum(prob,1e-7)) +
                        (1-prob)*np.log(np.maximum(1-prob,1e-7)),0))/log(2)
        # Continuous uncertainty, not thresholded predicted-free counting.
        uncertainty=np.minimum(1, np.sqrt(np.maximum(0,snapshot.variance))/.5)
        self.weights = (1 + config.uncertainty_weight * np.maximum(entropy,uncertainty))[self.target].astype(np.float32)
        self.weight_sum = float(self.weights.sum())
        self.scenarios=[]
        sig=np.sqrt(np.maximum(snapshot.variance,0))
        mean_occ=np.clip(snapshot.mean,0,1)
        pess=np.clip(mean_occ+config.pessimistic_sigma*sig,0,1)
        models = snapshot.ensemble
        # Ensemble is used directly when present; else mean is the primary map.
        if models is None:
            self.scenarios=[mean_occ]
        else:
            self.scenarios=[np.clip(q,0,1) for q in models]
        self.scenarios=[np.where(obs==2,1.,np.where(obs==0,0.,s)) for s in self.scenarios]
        self.pess=np.where(obs==2,1.,np.where(obs==0,0.,pess))
        beams=config.planning_beams
        steps=np.arange(1, int(config.lidar_range_m/config.resolution_m/config.ray_step_cells)+1,dtype=float)*config.ray_step_cells
        angle=np.linspace(0,2*pi,beams,endpoint=False,dtype=float)[:,None]
        self.offs_y=np.rint(np.cos(angle)*steps[None,:]).astype(np.int32)
        self.offs_x=np.rint(np.sin(angle)*steps[None,:]).astype(np.int32)
        self.cache={}
        self.pess_frac=config.scenario_pessimistic_fraction

    def _ray(self, origin, occupancy):
        h,w=self.target.shape
        Y=origin[0]+self.offs_y; X=origin[1]+self.offs_x
        valid=(Y>=0)&(Y<h)&(X>=0)&(X<w)
        Y=np.clip(Y,0,h-1);X=np.clip(X,0,w-1)
        # Do not repeatedly attenuate within duplicated rasterized grid cells.
        unique=np.ones(Y.shape,bool)
        unique[:,1:] = (Y[:,1:]!=Y[:,:-1]) | (X[:,1:]!=X[:,:-1])
        inside = valid & self.room[Y,X]
        p_occ = occupancy[Y,X]
        op=np.where(inside, np.where(unique,p_occ,0.),1.)
        trans_before=np.ones(op.shape,dtype=np.float32)
        trans_before[:,1:] = np.cumprod(1-op[:,:-1],axis=1,dtype=np.float32)
        eligible=inside & unique & self.target[Y,X] & (trans_before>1e-6)
        out=np.zeros(self.target.shape,np.float32)
        np.maximum.at(out,(Y[eligible],X[eligible]),trans_before[eligible])
        return out[self.target]

    def view(self, pos:tuple[int,int]):
        if pos in self.cache:return self.cache[pos]
        scen=np.stack([self._ray(pos,s) for s in self.scenarios],axis=0)
        avg=scen.mean(axis=0)
        pessimistic=self._ray(pos,self.pess)
        self.cache[pos]=(avg,pessimistic)
        return avg,pessimistic

    def project_leg(self, path:list[tuple[int,int]], include_start=False):
        if not path:raise ValueError("empty path")
        stride=max(1,round(self.config.scan_every_m/self.config.resolution_m))
        ids=list(range(0 if include_start else stride,len(path),stride))
        if not ids or ids[-1]!=len(path)-1:ids.append(len(path)-1)
        nom=np.zeros(len(self.weights),np.float32)
        pess=np.zeros_like(nom)
        for i in ids:
            a,b=self.view(path[i]);np.maximum(nom,a,out=nom);np.maximum(pess,b,out=pess)
        return nom,pess,len(ids)

    def gain(self, nom,pess):
        blended=(1-self.pess_frac)*nom+self.pess_frac*pess
        expected=float(np.dot(self.weights,blended))
        # Model disagreement is an uncertainty risk measured in cell-equivalents.
        gap=float(np.dot(self.weights,np.maximum(0,nom-pess)))
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


def _cost(cfg, vis:ProbabilisticVisibility, nom, pess, dist_m:float, n_goals:int):
    gain,risk=vis.gain(nom,pess)
    remaining=max(0.,vis.weight_sum-gain)
    travel=dist_m/cfg.robot_speed_mps
    stop=n_goals*cfg.goal_stop_time_s
    risk_s=cfg.risk_penalty_s_per_cell*risk
    objective=travel+stop+cfg.unknown_penalty_s_per_cell*remaining+risk_s
    return {'objective_s_equivalent':float(objective), 'travel_time_s':float(travel),
            'goal_stop_time_s':float(stop), 'unobserved_weight':float(remaining),
            'expected_observed_weight':float(gain),'uncertainty_gap_weight':float(risk),
            'uncertainty_penalty_s':float(risk_s)}


@dataclass
class _State:
    pos:tuple[int,int]
    selected:tuple[tuple[int,int],...]
    nom:np.ndarray
    pess:np.ndarray
    length_m:float
    steps:list[dict]
    cost:dict


def _is_better(new:_State,old:_State):
    return (new.cost['objective_s_equivalent']+1e-8 < old.cost['objective_s_equivalent'])


def plan(snapshot:G2Snapshot,config:G2Config|None=None)->dict[str,Any]:
    """Return a complete *conditional* goal sequence, not merely the next goal.

    This is an approximate finite beam search with no hardcoded goal count.
    Whole route is planned from the current snapshot, then the controller must
    replan on material observations. Replan does not secretly mutate maps.
    """
    cfg=config or G2Config();t0=perf_counter();_validate(snapshot)
    room=_room(snapshot)
    legal=_navigable(snapshot,room,cfg)
    parent0,dist0=_reach(snapshot.robot_rc,legal)
    vis=ProbabilisticVisibility(snapshot,room,cfg)
    frontiers=_candidate_goals(snapshot,room,legal,dist0,vis,cfg)
    observed_unknown=int(((snapshot.observed==1)&room).sum())
    room_cells=int(room.sum())
    evidence={
        'decision_id':snapshot.decision_id,
        'observed_sha256':_hash(snapshot.observed),'mean_sha256':_hash(snapshot.mean),
        'variance_sha256':_hash(snapshot.variance),'semantic_sha256':_hash(snapshot.semantic),
        'input_includes_ground_truth':False,
        'input_is_current_decision_only':True,
        'ensemble_members':None if snapshot.ensemble is None else int(snapshot.ensemble.shape[0]),
        'prediction_must_be_refreshed_after_replan_trigger':True,
        'algorithm':'multi-goal finite beam-search over current observed-free paths, probabilistic pathwise LaMa visibility',
        'no_global_optimum_claim':True,'configuration_is_not_calibrated':True}
    base_nom=np.zeros(len(vis.weights),np.float32);base_pess=base_nom.copy()
    base_cost=_cost(cfg,vis,base_nom,base_pess,0.,0)
    base=_State(snapshot.robot_rc,tuple(),base_nom,base_pess,0.,[],base_cost)
    best=base;layer=[base];search_records=[];seen_solutions=0
    # Cache pathfinding from each end viewpoint to every other valid viewpoint.
    parent_cache={snapshot.robot_rc:(parent0,dist0)}
    for depth in range(len(frontiers)):
        options=[]
        for state in layer:
            if state.pos not in parent_cache:
                parent_cache[state.pos]=_reach(state.pos,legal)
            parents,dists=parent_cache[state.pos]
            preliminary=[]
            for point in frontiers:
                if point in state.selected or point not in dists or point==state.pos:continue
                maybe_nom,maybe_pess=vis.view(point)
                a=np.maximum(state.nom,maybe_nom)
                b=np.maximum(state.pess,maybe_pess)
                expected,_=vis.gain(a,b)
                marginal=expected-state.cost['expected_observed_weight']
                if marginal <= 0:continue
                travel=dists[point]*cfg.resolution_m
                preliminary.append((marginal/(travel/cfg.robot_speed_mps+cfg.goal_stop_time_s+1),marginal,-travel,point))
            preliminary.sort(reverse=True)
            # Rerank by *pathwise* new information; stationary prelim only prunes.
            for _,__,___,point in preliminary[:cfg.branch_width]:
                route=_trace(state.pos,point,parents)
                route_nom,route_pess,n_scans=vis.project_leg(route)
                next_nom=np.maximum(state.nom,route_nom)
                next_pess=np.maximum(state.pess,route_pess)
                distance=(len(route)-1)*cfg.resolution_m
                length=state.length_m+distance
                next_cost=_cost(cfg,vis,next_nom,next_pess,length,len(state.selected)+1)
                gain=next_cost['expected_observed_weight']-state.cost['expected_observed_weight']
                if gain < cfg.min_new_weighted_cells:continue
                if state.cost['objective_s_equivalent']-next_cost['objective_s_equivalent'] < cfg.min_net_improvement_s:
                    continue
                leg={'index':len(state.selected)+1,'goal_rc':list(point),
                     'distance_m':round(distance,4),'scan_samples':int(n_scans),
                     'path_rc':[list(x) for x in route],
                     'marginal_expected_new_weight':round(gain,4),
                     'cumulative_expected_new_weight':round(next_cost['expected_observed_weight'],4),
                     'predicted_remaining_weight':round(next_cost['unobserved_weight'],4),
                     'step_cost_s_equivalent':round(next_cost['objective_s_equivalent'],4),
                     'goal_is_observed_free':True,'route_is_observed_free':True,
                     'conditional_replan_before_next_leg':True}
                options.append(_State(point,state.selected+(point,),next_nom,next_pess,
                                      length,state.steps+[leg],next_cost))
                seen_solutions+=1
        if not options:break
        options.sort(key=lambda x:(x.cost['objective_s_equivalent'],x.length_m,len(x.selected),x.selected))
        layer=options[:cfg.beam_width]
        best_layer=layer[0]
        search_records.append({'goal_count':depth+1,
                               'best_objective_s_equivalent':round(best_layer.cost['objective_s_equivalent'],4),
                               'length_m':round(best_layer.length_m,4),
                               'expected_observed_weight':round(best_layer.cost['expected_observed_weight'],4)})
        if _is_better(best_layer,best):best=best_layer
    # Conservative room completion: a model-based forecast is NEVER "COMPLETE".
    # Stop/defer decision is a controller concern, not inferred from LaMa.
    if observed_unknown==0:status='COMPLETE_OBSERVED'
    elif best.selected:status='CONTINUE'
    else:status='DEFER_NO_POSITIVE_SAFE_CHAIN'
    results={
        'status':status,'robot_rc':list(snapshot.robot_rc),'room_cells':room_cells,
        'observed_room_unknown_cells':observed_unknown,
        'expected_total_unknown_weight':round(vis.weight_sum,4),
        'goal_sequence':[list(x) for x in best.selected],
        'goal_count':len(best.selected),
        'route_segments':best.steps,
        'route_distance_m':round(best.length_m,4),
        'cost':{key:round(val,4) for key,val in best.cost.items()},
        'baseline_cost':{key:round(val,4) for key,val in base_cost.items()},
        'search':{'candidate_goals':len(frontiers),'examined_route_expansions':seen_solutions,
                  'beam_width':cfg.beam_width,'branch_width':cfg.branch_width,
                  'goal_count_frontier':search_records, 'stopped_when_no_positive_new_goal':True},
        'planning_runtime_s':round(perf_counter()-t0,3),
        'config':asdict(cfg),'provenance':evidence,
        'cautions':[
            'No ground truth or later decisions used; predictions may be wrong.',
            'Only observed FREE supports route; conditional future unknown requires replan.',
            'Probabilistic rays use independence/max approximation; not native RangeLibc.',
            'No certified robot footprint safety, native integration or scientific gain.',
            'Goal number is selected by heuristic objective; no fixed 1/2/3-goal limit.',
            'COMPLETE only when observed unknown within room is zero; DEFER is not COMPLETE.'
        ]}
    return results


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
