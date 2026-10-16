"""G2-R3.1 Structure-Commitment Planning.

Preserve planned structural subgoals across scans while they remain
observable and reachable. Independently track realized progress at each
goal. Replan only on invalidity, task change, or exhaustion of the
committed chain. Failed observations suppress unproductive re-visits.

This is a causal 2D *offline controller*, not a robot safety authority.
"""
from __future__ import annotations
from dataclasses import dataclass,field,asdict,replace
from pathlib import Path
from collections import defaultdict
import hashlib,json,math
import numpy as np
from .structural import _validate,sha,accepted_component,StructuralConfig,_navigable,_bfs,_route
from .sequence import StructuralRoutePlanner,SequenceConfig,_Visibility

@dataclass(frozen=True)
class CommitmentConfig:
    # An uncalibrated current-map predicted visibility is a reason to
    # visit, not proof of actually observing these unknown cells.
    minimum_remaining_expected_cells:int=16
    # A failed scan indicates that the view geometry/hypothesis is
    # unreliable; one failure defers same/nearby viewpoints, two
    # independent failures defer an obligation (without clearing it).
    minimum_expected_before_failure:int=12
    minimum_realized_fraction_for_success:float=.08
    independent_failure_separation_m:float=1.0
    repeat_view_radius_m:float=.8
    max_region_failure_angles:int=2
    # Compare the currently observed geometry for remaining targets;
    # always re-evaluate reachability after a new scan.
    scan_stride_cells:int=5
    coarse_beams:int=250
    ray_step_cells:float=.5
    max_global_stagnant_legs:int=2
    # A room cannot be declared explored just because predicted walls
    # remain or structure has shrunk. DEFER is explicitly not COMPLETE.
    active_room_must_be_robot_connected:bool=True

@dataclass
class CommitmentState:
    schema:str='G2_R31_COMMITMENT_V1'
    pending_goals:list=field(default_factory=list)
    pending_source_decision:str=''
    visits:list=field(default_factory=list)
    region_failures:dict=field(default_factory=dict)
    deferred_regions:list=field(default_factory=list)
    stagnant_legs:int=0
    audit:list=field(default_factory=list)

    def save(self,path):
        Path(path).parent.mkdir(parents=True,exist_ok=True)
        Path(path).write_text(json.dumps(asdict(self),ensure_ascii=False,indent=2))
    @classmethod
    def load(cls,path):
        x=json.loads(Path(path).read_text())
        if x.get('schema')!='G2_R31_COMMITMENT_V1':raise ValueError('unknown commitment schema')
        return cls(**x)

class ProgressAwarePlanner(StructuralRoutePlanner):
    """Base R3 joint sequence scoring is preserved; only proven futile
    targets/viewpoint duplicates excluded from the current proposal.
    """
    def __init__(self,obs,mean,var,sem,robot,memory,report,state,cfg=None,ctrl=None):
        super().__init__(obs,mean,var,sem,robot,memory,report,cfg)
        self.commitment_state=state
        self.commitment_config=ctrl or CommitmentConfig()
        excluded=set(state.deferred_regions)
        if excluded:
            self.major=[r for r in self.major if r['region_id'] not in excluded]
            self.rids=[r['region_id'] for r in self.major]
            indices=np.flatnonzero(self.unknown.ravel())
            self.region_bits={}
            for r in self.major:
                mask=(memory.labels.ravel()[indices]==r['region_id'])
                self.region_bits[r['region_id']]=int.from_bytes(np.packbits(mask,bitorder='little').tobytes(),'little')
            self.region_scales=[math.log1p(float(r['expected_open_cell_mass_proxy'])) for r in self.major]
            self.region_targets=[max(1,int(math.ceil(self.cfg.region_satisfaction_fraction*r['cells']))) for r in self.major]
            self.open_major_index=[i for i,r in enumerate(self.major) if r['kind']=='POSSIBLE_OPEN_SPACE']
            self.open_region_weight_total=sum(self.region_scales[i] for i in self.open_major_index)

    def _not_unproductive_view(self,pos):
        rad=self.commitment_config.repeat_view_radius_m/self.cfg.resolution_m
        for visited in self.commitment_state.visits:
            # Suppress only failed viewpoints; a prior successful location
            # can be revisited if the current map genuinely changes.
            if visited.get('realized_global_cells',0)>=self.commitment_config.minimum_expected_before_failure:
                continue
            prev=visited['goal_rc']
            if math.hypot(pos[0]-prev[0],pos[1]-prev[1])<=rad:return False
        return True

    def prepare_candidates(self):
        pts=super().prepare_candidates()
        self.candidates=[q for q in pts if self._not_unproductive_view(q)]
        self.candidate_filtered_no_progress=len(pts)-len(self.candidates)
        return self.candidates

def _leg_visibility(obs,mean,var,mem,src,dst,cc):
    sem=None
    cfg=SequenceConfig(coarse_beams=cc.coarse_beams,scan_every_cells=cc.scan_stride_cells,
        coarse_ray_step_cells=cc.ray_step_cells)
    ray=_Visibility(obs,mean,var,mem.mission,mem.mission&(obs==1),cfg,
                    cc.coarse_beams,cc.ray_step_cells)
    return ray

def check_remaining_goal(obs,mean,var,semantic,robot,mem,goal,cfg):
    """Rebuild current known-free shortest route and projected information
    to the next *committed* goal.  No future map or old cached path.
    """
    _validate(obs,mean,var,semantic,robot)
    room=accepted_component(obs,semantic,robot)
    if not room.any():return None,'ROBOT_NOT_IN_ACCEPTED_ROOM'
    legal=_navigable(obs,room,robot,StructuralConfig(clearance_cells=2),)
    parents,dist=_bfs(robot,legal)
    point=tuple(goal)
    if point not in parents:return None,'NO_CURRENT_KNOWNFREE_ROUTE'
    path=_route(parents,robot,point)
    view=_leg_visibility(obs,mean,var,mem,robot,point,cfg)
    a,b=view.along(path)
    expected=a.bit_count()
    if expected<cfg.minimum_remaining_expected_cells:
        return {'goal_rc':list(goal),'expected_unknown_nominal':expected,'route_distance_m':len(path)*.1},'NO_LONGER_USEFUL'
    return {'goal_rc':list(goal),'path_rc':[list(p) for p in path],
        'expected_unknown_nominal':expected,
        'expected_unknown_stress':b.bit_count(),
        'route_distance_m':round((len(path)-1)*.1,3)},'KEEP'

class CommitmentController:
    def __init__(self,state=None,cfg=None):
        self.state=state or CommitmentState()
        self.config=cfg or CommitmentConfig()

    def initialize(self,initial_plan):
        if self.state.visits or self.state.pending_goals:raise ValueError('commitment already initialized')
        if initial_plan['status']!='CONDITIONAL_VIEWPOINT_SEQUENCE':
            raise ValueError('initial plan must have a sequence')
        self.state.pending_goals=[list(g) for g in initial_plan['goal_sequence']]
        self.state.pending_source_decision=initial_plan['decision_id']
        self.state.audit.append({'event':'INITIAL_COMMIT','goals':self.state.pending_goals.copy()})

    def record_arrival(self,before_obs,after_obs,previous_labels,prior_plan,executed_goal,src_pose):
        """Call only AFTER a LiDAR scan, BEFORE structural labels are recomputed.
        Realized observations determine success; GT never enters this function.
        """
        if before_obs.shape!=after_obs.shape or before_obs.shape!=previous_labels.shape:
            raise ValueError('snapshot dimensions differ')
        if np.any((before_obs!=1)&(after_obs!=before_obs)):
            raise ValueError('observed evidence overwritten')
        if not self.state.pending_goals or self.state.pending_goals[0]!=list(executed_goal):
            raise ValueError('executed waypoint not next committed goal')
        actual_new=(before_obs==1)&(after_obs!=1)
        total=int(actual_new.sum())
        prior_regions={int(r['region_id']):r for r in prior_plan.get('regions',[])}
        # Expected per-region gain for the *committed next leg*, given
        # prior-time geometry, rather than the whole frozen sequence.
        # The caller can provide masked per-region expectations computed
        # from the prior snapshot's current line-of-sight.
        expected=prior_plan.get('first_leg_expected_by_region',{})
        progressed={}
        penalized=[]
        for key,value in expected.items():
            rid=int(key)
            pred=int(value)
            actual=int((actual_new&(previous_labels==rid)).sum())
            progressed[str(rid)]={'expected':pred,'realized':actual}
            if pred>=self.config.minimum_expected_before_failure:
                if actual>=max(1,pred*self.config.minimum_realized_fraction_for_success):
                    self.state.region_failures.pop(str(rid),None)
                else:
                    old=self.state.region_failures.setdefault(str(rid),[])
                    pos=list(executed_goal)
                    independent=all(math.hypot(pos[0]-a['rc'][0],pos[1]-a['rc'][1])>=
                        self.config.independent_failure_separation_m/.1 for a in old)
                    if independent:
                        old.append({'rc':pos,'expected':pred,'realized':actual})
                    if len(old)>=self.config.max_region_failure_angles and rid not in self.state.deferred_regions:
                        self.state.deferred_regions.append(rid);penalized.append(rid)
        self.state.visits.append({'goal_rc':list(executed_goal),'start_rc':list(src_pose),
            'realized_global_cells':total,'by_region':progressed})
        self.state.pending_goals.pop(0)
        self.state.stagnant_legs=self.state.stagnant_legs+1 if total==0 else 0
        ev={'event':'OBSERVATION_UPDATE','executed_goal_rc':list(executed_goal),
            'realized_global_cells':total,'regions':progressed,
            'deferred_after_independent_failures':penalized,
            'remaining_goals_before_safety_validation':self.state.pending_goals.copy()}
        self.state.audit.append(ev)
        return ev

    def decide(self,obs,mean,var,semantic,robot,mem,regions,sequence_cfg=None):
        """Maintain an existing valid multi-goal commitment before falling
        back to the original structure-guided multi-goal R3 search.
        """
        if self.state.stagnant_legs>=self.config.max_global_stagnant_legs:
            return {'status':'DEFER_STALLED_WITH_UNRESOLVED_MISSION',
                'goal_sequence':[],'decision':'DEFER','replanning_called':False,
                'reason':'Two consecutive observation-free legs; not ROOM_COMPLETE',
                'mission_unknown':regions['mission_unknown']}
        while self.state.pending_goals:
            check,reason=check_remaining_goal(obs,mean,var,semantic,robot,mem,
                    self.state.pending_goals[0],self.config)
            if reason=='KEEP':
                return {'status':'COMMITMENT_CONTINUED','goal_sequence':self.state.pending_goals.copy(),
                        'selected_next_leg':check,'decision':'KEEP','replanning_called':False,
                        'reason':'Knownfree route still valid and current-map projected unknown sufficient',
                        'remaining_unresolved_regions':regions['mission_unknown']}
            # If a committed goal no longer useful or reachable, no attempt
            # to use stale path. Replan from observed evidence only.
            self.state.audit.append({'event':'COMMITMENT_INVALIDATED',
                 'goal':self.state.pending_goals[0],'reason':reason,
                 'projected':check})
            self.state.pending_goals.clear()
            break

        pcfg=sequence_cfg or SequenceConfig()
        pcfg=replace(pcfg,decision_id=regions['decision_id'])
        planner=ProgressAwarePlanner(obs,mean,var,semantic,robot,mem,regions,
                self.state,pcfg,self.config)
        result=planner.solve()
        if result['status']!='CONDITIONAL_VIEWPOINT_SEQUENCE':
            return {'status':result['status'],'goal_sequence':[],'decision':'DEFER',
                'replanning_called':True,
                'reason':'No usable structural chain in safe candidate/search budget; unresolved obligations retained',
                'mission_unknown':regions['mission_unknown'],'planner_report':result}
        self.state.pending_goals=[list(g) for g in result['goal_sequence']]
        self.state.pending_source_decision=regions['decision_id']
        self.state.audit.append({'event':'NEW_COMMITMENT','goals':self.state.pending_goals.copy(),
              'regions':result.get('regions',[])})
        first=result['selected']['stages'][0]
        return {'status':'NEW_COMMITMENT','goal_sequence':self.state.pending_goals.copy(),
            'decision':'REPLAN','replanning_called':True,
            'reason':'Previous chain completed or invalidated; new structural regions evaluated',
            'selected_next_leg':{'goal_rc':first['goal_rc'],'path_rc':first['path_rc'],
                 'route_distance_m':first['distance_leg_m'],
                 'expected_unknown_nominal':first['newly_visible_unknown_nominal'],
                 'expected_unknown_stress':first['newly_visible_unknown_stress']},
            'planner_report':result}

def expected_first_leg_per_region(obs,mean,var,mem,regions,robot,route,cc=None):
    """Compute expectations before executing the chosen segment; captured
    for honest realized-vs-predicted progress comparison AFTER scan.
    """
    cc=cc or CommitmentConfig()
    ray=_leg_visibility(obs,mean,var,mem,tuple(robot),tuple(route[-1]),cc)
    nominal,stress=ray.along([tuple(p) for p in route])
    indices=np.flatnonzero((mem.mission&(obs==1)).ravel())
    reglabs=mem.labels.ravel()[indices]
    result={}
    for rid in np.unique(reglabs):
        if rid<=0:continue
        bit=int.from_bytes(np.packbits(reglabs==rid,bitorder='little').tobytes(),'little')
        value=(nominal&bit).bit_count()
        if value:result[str(int(rid))]=value
    return result
