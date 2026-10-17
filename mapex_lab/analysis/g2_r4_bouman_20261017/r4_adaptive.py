"""G2-R4: Bouman-informed, MapEx/Room-Node structural receding-horizon *advisory*.

No GT, robot commands, LaMa inference, or changes to G2-R3/R3.1.
Uses existing R3 structural/pathwise visibility estimates and current observed map.
All gains/scores are heuristic. A grid path is NOT a physical safety certificate.
"""
from __future__ import annotations
from dataclasses import dataclass, field, asdict
from typing import Sequence, Mapping, Optional
import math

@dataclass(frozen=True)
class R4Config:
    resolution_m: float = 0.1
    speed_mps: float = 0.3
    viewpoint_dwell_s: float = 0.0
    nominal_weight: float = 0.7
    cost_per_second: float = 0.6  # heuristic structural-cell equivalents / second
    max_execution_m: float = 0.5
    minimum_replan_scans: int = 3
    major_new_cells_event: int = 100
    mismatch_min_expected: int = 12
    mismatch_fraction: float = 0.08
    minimum_region_reliability: float = 0.15
    keep_hysteresis: float = 0.05
    conservative_gate: bool = True

    def __post_init__(self):
        if not (self.resolution_m>0 and self.speed_mps>0 and self.max_execution_m>0):
            raise ValueError('invalid geometry or speed')
        if not (0<=self.nominal_weight<=1 and 0<=self.minimum_region_reliability<=1):
            raise ValueError('invalid probability-shaped heuristics')
        if self.minimum_replan_scans<1 or self.major_new_cells_event<1:
            raise ValueError('invalid triggers')
        if any(v<0 for v in (self.cost_per_second,self.keep_hysteresis)):
            raise ValueError('negative cost')

@dataclass
class RegionEvidence:
    """Observation-only evidence; prediction is never promoted to occupancy evidence."""
    expected: int = 0
    realized: int = 0
    failed_views: list = field(default_factory=list)
    last_scan: int = -1

@dataclass
class EvidenceLedger:
    regions: dict = field(default_factory=dict)

    def reliability(self, region_id: int, cfg: R4Config) -> float:
        row=self.regions.get(str(region_id))
        if row is None or row.expected==0:
            return 1.0
        # Smoothed realization ratio: NOT a calibrated probability.
        fraction=min(1.,(row.realized+2.)/(row.expected+2.))
        return max(cfg.minimum_region_reliability, fraction)

    def observe_completed_leg(self, before, after, prior_labels, expected_by_region: Mapping,
                              pose_rc, scan_id: int, cfg: R4Config):
        import numpy as np
        if before.shape != after.shape or before.shape != prior_labels.shape:
            raise ValueError('evidence map shapes differ')
        if bool(np.any((before!=1)&(before!=after))):
            raise ValueError('nonmonotone observed map; reject evidence')
        if not bool(np.all(np.isin(after,(0,1,2)))):
            raise ValueError('bad occupancy class')
        newly=(before==1)&(after!=1)
        result={}
        for k, n in expected_by_region.items():
            rid=int(k); expected=int(n)
            if expected<0:raise ValueError('negative expected mass')
            realized=int((newly&(prior_labels==rid)).sum())
            row=self.regions.setdefault(str(rid),RegionEvidence())
            if scan_id<=row.last_scan:raise ValueError('stale/replayed evidence')
            row.expected+=expected;row.realized+=realized;row.last_scan=scan_id
            if expected>=cfg.mismatch_min_expected and realized<cfg.mismatch_fraction*expected:
                if all(math.dist(pose_rc,q)>=10 for q in row.failed_views):
                    row.failed_views.append(tuple(int(v) for v in pose_rc))
            result[str(rid)]={'expected':expected,'realized':realized,
                              'reliability_proxy':self.reliability(rid,cfg),
                              'independent_failed_poses':len(row.failed_views)}
        return {'global_realized_cells':int(newly.sum()),'by_region':result}

    def to_json(self):
        return {k:asdict(v) for k,v in sorted(self.regions.items())}

    @classmethod
    def from_json(cls, data):
        rows={}
        for k,v in data.items():
            rows[str(int(k))]=RegionEvidence(expected=int(v['expected']),
                realized=int(v['realized']), failed_views=[tuple(q) for q in v['failed_views']],
                last_scan=int(v['last_scan']))
        return cls(rows)

@dataclass(frozen=True)
class Leg:
    goal_rc: tuple
    path_rc: tuple  # complete knownfree route, starting at current pose
    nominal: int  # packed, indexed only by current unknown mission cells
    conservative: int
    kind: str = 'view'  # view or exit
    def __post_init__(self):
        if not self.path_rc or self.path_rc[-1]!=self.goal_rc:
            raise ValueError('leg endpoint must equal goal')
        if self.nominal<0 or self.conservative<0 or self.kind not in ('view','exit'):
            raise ValueError('invalid leg')

@dataclass(frozen=True)
class Chain:
    label: str
    legs: tuple
    source_hash: str
    def __post_init__(self):
        for a,b in zip(self.legs,self.legs[1:]):
            if a.goal_rc != b.path_rc[0]:raise ValueError('disconnected route')

class PathValue:
    def __init__(self, regions: Sequence[Mapping], region_bits: Mapping[int,int],
                 evidence: EvidenceLedger, cfg: R4Config):
        self.regions=list(regions)
        self.bits={int(k):int(v) for k,v in region_bits.items()}
        self.evidence=evidence;self.cfg=cfg
        for r in regions:
            if int(r['region_id']) not in self.bits:raise ValueError('missing region mask')

    def gain(self,bits:int):
        ans=0.
        for reg in self.regions:
            rid=int(reg['region_id']); n=(bits & self.bits[rid]).bit_count()
            if not n:continue
            # Per-cell structural value, avoiding large rooms drowning small ones.
            m=max(1,int(reg['cells'])); mass=max(0.,float(reg['expected_open_cell_mass_proxy']))
            priority=(1.+math.log1p(mass)/math.sqrt(m))
            if reg.get('kind')!='POSSIBLE_OPEN_SPACE':priority*=0.35
            ans+=n*priority*self.evidence.reliability(rid,self.cfg)
        return ans

    def evaluate(self, chain:Chain, baseline_mask:Optional[tuple]=None,
                 anchor_required:bool=True):
        cfg=self.cfg
        if anchor_required and (not chain.legs or chain.legs[-1].kind!='exit'):
            return {'label':chain.label,'feasible':False,'reason':'MISSING_KNOWNFREE_REJOIN'}
        cumulative_nom=cumulative_cons=0;time=0.;distance=0.;details=[]
        for i,leg in enumerate(chain.legs):
            if i and leg.path_rc[0]!=chain.legs[i-1].goal_rc:
                return {'label':chain.label,'feasible':False,'reason':'BROKEN_PATH'}
            nnew=leg.nominal & ~cumulative_nom
            cnew=leg.conservative & ~cumulative_cons
            cumulative_nom|=leg.nominal;cumulative_cons|=leg.conservative
            length=max(0,len(leg.path_rc)-1)*cfg.resolution_m
            dt=length/cfg.speed_mps+(cfg.viewpoint_dwell_s if leg.kind=='view' else 0)
            time+=dt;distance+=length
            a=self.gain(nnew);b=self.gain(cnew)
            proxy=cfg.nominal_weight*a+(1-cfg.nominal_weight)*b
            score=proxy-cfg.cost_per_second*dt
            details.append({'goal_rc':list(leg.goal_rc),'kind':leg.kind,'distance_m':round(length,3),
                            'delta_time_s':round(dt,3), 'unique_nominal_cells':nnew.bit_count(),
                            'unique_conservative_cells':cnew.bit_count(),
                            'structural_proxy_gain':round(proxy,4),'net_step_proxy':round(score,4)})
        nominal=self.gain(cumulative_nom);conservative=self.gain(cumulative_cons)
        objective=cfg.nominal_weight*nominal+(1-cfg.nominal_weight)*conservative-cfg.cost_per_second*time
        robust=conservative-cfg.cost_per_second*time
        result={'label':chain.label,'feasible':True,'status':'HEURISTIC_NOT_CALIBRATED',
                'objective':round(objective,4),'conservative_objective':round(robust,4),
                'distance_m':round(distance,3),'estimated_time_s':round(time,3),
                'nominal_unique_cells':cumulative_nom.bit_count(),
                'conservative_unique_cells':cumulative_cons.bit_count(),
                'structural_nominal':round(nominal,4),'structural_conservative':round(conservative,4),
                'legs':details,'source_hash':chain.source_hash}
        if baseline_mask is not None:
            bn,bc=baseline_mask
            result['baseline_comparison']={
                'nominal_novel_cells':(cumulative_nom&~bn).bit_count(),
                'nominal_lost_cells':(bn&~cumulative_nom).bit_count(),
                'conservative_novel_cells':(cumulative_cons&~bc).bit_count(),
                'conservative_lost_cells':(bc&~cumulative_cons).bit_count(),
                'only_valid_if_same_snapshot_and_horizon':True}
        return result

    def choose(self, current:Optional[Chain], alternatives:Sequence[Chain],
               baseline_mask=None):
        hashes={c.source_hash for c in ([current] if current else [])+list(alternatives)}
        if len(hashes)>1:raise ValueError('cannot compare different time-t snapshot hashes')
        rows=[]
        if current is not None: rows.append(self.evaluate(current,baseline_mask))
        for c in alternatives:rows.append(self.evaluate(c,baseline_mask))
        passing=[r for r in rows if r['feasible'] and r['objective']>0 and
                 (not self.cfg.conservative_gate or r['conservative_objective']>0)]
        if not passing:
            return {'decision':'DEFER_ROOM','policy_action':None,'reason':'NO_POSITIVE_ROBUST_ROOM_CHAIN',
                    'unresolved_unknown_kept':True,'comparisons':rows}
        ranked=sorted(passing,key=lambda r:(-r['objective'],r['estimated_time_s'],r['label']))
        best=ranked[0]
        original=next((r for r in passing if current and r['label']==current.label),None)
        if original and best['label']!=original['label'] and best['objective'] < original['objective']*(1+self.cfg.keep_hysteresis):
            best=original
        return {'decision':'KEEP' if current and best['label']==current.label else 'REPLAN',
                'selected_label':best['label'],'policy_action':None,
                'reason':'ROBUST_WHOLE_CHAIN_COMPARISON','comparisons':rows,
                'requires_native_safety_certificate':True}


def safe_event(old_obs, new_obs, pose_rc, remaining_path, *, at_goal=False,
               structural_changed=False, expected_cells=None, realized_cells=None,
               scans_since_heavy=0, cfg=None, clearance_cells=2, roi_mask=None):
    """Cheap post-scan gate. NEVER interprets a candidate as a movement command.

    Missing/pending path => HOLD; NEW wall/changed geometry can invalidate route
    regardless of whether expensive LaMa/beam search is ready.
    """
    import numpy as np
    import cv2
    cfg=cfg or R4Config()
    if old_obs.shape!=new_obs.shape or not np.isin(new_obs,[0,1,2]).all():
        raise ValueError('invalid map')
    if np.any((old_obs!=1)&(new_obs!=old_obs)):
        raise ValueError('nonmonotone observed map')
    y,x=pose_rc
    if not (0<=y<new_obs.shape[0] and 0<=x<new_obs.shape[1] and new_obs[y,x]==0):
        return {'event':'SAFE_HOLD','reason':'ROBOT_NOT_OBSERVED_FREE','heavy':False,'policy_action':None}
    if at_goal:
        return {'event':'REVALUE_NOW','reason':'GOAL_ARRIVAL','heavy':True,'policy_action':None}
    if remaining_path is None or not remaining_path:
        return {'event':'SAFE_HOLD','reason':'NO_VERIFIED_ROUTE','heavy':False,'policy_action':None}
    forbidden=new_obs!=0
    if clearance_cells:
        kernel=np.ones((2*clearance_cells+1,2*clearance_cells+1),dtype=np.uint8)
        forbidden=cv2.dilate(forbidden.astype(np.uint8),kernel)!=0
    if tuple(remaining_path[0])!=tuple(pose_rc):
        return {'event':'SAFE_HOLD','reason':'ROUTE_START_MISMATCH','heavy':False,'policy_action':None}
    for q in remaining_path:
        r,c=q
        if not (0<=r<new_obs.shape[0] and 0<=c<new_obs.shape[1]) or forbidden[r,c]:
            return {'event':'SAFE_HOLD','reason':'ROUTE_NO_LONGER_KNOWNFREE_CLEAR','heavy':False,'policy_action':None}
    delta=(old_obs==1)&(new_obs!=1)
    if roi_mask is not None:
        if roi_mask.shape!=new_obs.shape:raise ValueError('ROI shape mismatch')
        delta=delta&roi_mask.astype(bool)
    n_new=int(delta.sum());new_walls=int((delta&(new_obs==2)).sum())
    discrepancy=expected_cells is not None and realized_cells is not None and \
        expected_cells>=cfg.mismatch_min_expected and realized_cells<expected_cells*cfg.mismatch_fraction
    if at_goal or structural_changed:
        why='GOAL_ARRIVAL' if at_goal else 'STRUCTURAL_VERSION_CHANGE'
        return {'event':'REVALUE_NOW','reason':why,'heavy':True,'policy_action':None,
                'new_cells':n_new,'new_walls':new_walls}
    if discrepancy or n_new>=cfg.major_new_cells_event or new_walls>0:
        if scans_since_heavy>=cfg.minimum_replan_scans:
            return {'event':'REVALUE_NOW','reason':'DISCREPANCY' if discrepancy else 'MATERIAL_OBSERVATION',
                    'heavy':True,'policy_action':None,'new_cells':n_new,'new_walls':new_walls}
        return {'event':'FAST_RECHECK','reason':'DEBOUNCED_MATERIAL_UPDATE','heavy':False,
                'policy_action':None,'new_cells':n_new,'new_walls':new_walls}
    return {'event':'FAST_KEEP','reason':'NO_MATERIAL_EVENT','heavy':False,'policy_action':None,
            'new_cells':n_new,'new_walls':new_walls}


def bounded_execution(path_rc, cfg:R4Config):
    if not path_rc:return []
    cells=max(1,int(cfg.max_execution_m/cfg.resolution_m))
    return [list(q) for q in path_rc[:cells+1]]


def from_r3_snapshot(obs,mean,variance,semantic,robot,memory,report,chains:Mapping[str,Sequence],
                     exit_anchor,ledger=None,config=None,source_hash=None):
    """Bridge to frozen G2-R3: validates source-bound map, knownfree reachability,
    and computes pathwise visibility for each supplied candidate and safe rejoin.
    R3 search itself stays untouched; call only on event-triggered episodes.
    """
    from mapex_g2_r3.sequence import StructuralRoutePlanner,SequenceConfig
    from mapex_g2_r3.structural import _route
    cfg=config or R4Config()
    ledger=ledger or EvidenceLedger()
    seq_cfg=SequenceConfig(resolution_m=cfg.resolution_m)
    adapter=StructuralRoutePlanner(obs,mean,variance,semantic,robot,memory,report,seq_cfg)
    if source_hash is not None and source_hash!=report['snapshot_sha256']:
        raise ValueError('snapshot SHA mismatch')
    root=report['snapshot_sha256']
    outputs=[];invalid={}
    for name, goals in chains.items():
        poses=[];start=tuple(robot);valid=True
        if not goals:invalid[name]='NO_ROOM_VIEW';continue
        for goal in list(goals)+[tuple(exit_anchor)]:
            dest=tuple(int(i) for i in goal)
            edge=adapter._leg(start,dest,adapter.coarse)
            if edge is None:
                invalid[name]='UNKNOWN_OR_UNREACHABLE_GOAL_OR_REJOIN';valid=False;break
            dist,masks,path=edge
            poses.append(Leg(dest,tuple(path),int(masks[0]),int(masks[1]),
                             'exit' if len(poses)==len(goals) else 'view'))
            start=dest
        if valid:outputs.append(Chain(name,tuple(poses),root))
    scorer=PathValue(adapter.major,adapter.region_bits,ledger,cfg)
    return scorer,outputs,invalid
