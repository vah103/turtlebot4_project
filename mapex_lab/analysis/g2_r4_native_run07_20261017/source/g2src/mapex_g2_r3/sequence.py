"""G2-R3 Stage B — route-aware structural set cover on causal room missions.

Inputs: current observed/LaMa mean+variance/R4.1, R3 region report and
persistent mission labels. NEVER reads GT, future snapshots or prior hand goals.
Planned goals only on reachable observed-free; future unknown traversal cannot
be executed by this planner. Searches full conditional 1..N sequences, including
pathwise LiDAR, followed by dense independent re-ranking.
"""
from __future__ import annotations
from dataclasses import dataclass,asdict,replace
from functools import lru_cache
from math import pi,ceil,log1p,sqrt
from time import perf_counter
from collections import defaultdict
from pathlib import Path
import numpy as np
from .structural import _validate,accepted_component,_navigable,_bfs,_route,sha,StructuralConfig

@dataclass(frozen=True)
class SequenceConfig:
    resolution_m:float=.1
    range_m:float=20.
    coarse_beams:int=250
    dense_beams:int=2500
    coarse_ray_step_cells:float=.5
    dense_ray_step_cells:float=.25
    scan_every_cells:int=5
    grid_stride_cells:int=4
    clearance_cells:int=2
    max_initial_viewpoint_proposals:int=175
    max_search_candidates:int=48
    min_major_expected_open_mass:float=12.
    min_major_region_cells:int=18
    region_satisfaction_fraction:float=.9
    max_search_depth:int=7  # computational budget, NOT fixed n-goal policy
    beam_width:int=6
    shortlisted_final_sequences:int=10
    coverage_value_scale:float=1.0
    route_cost_per_m:float=.18
    stop_cost_per_goal:float=.45
    pessimistic_sigma:float=1.0
    # Explicit conservative heuristic, not learned probability.
    nominal_scenario_fraction:float=.7
    structural_quality_bins:int=50  # nominal equivalent quality within 2% tolerance
    decision_id:str='current' 

    def __post_init__(self):
        if self.resolution_m<=0 or self.range_m<=0 or self.coarse_beams<8 or self.dense_beams<8:
            raise ValueError('invalid ray or map config')
        if not(0<self.region_satisfaction_fraction<=1 and 0<=self.nominal_scenario_fraction<=1):
            raise ValueError('fractions outside [0,1]')
        if min(self.max_search_depth,self.beam_width,self.max_search_candidates,self.scan_every_cells,
               self.max_initial_viewpoint_proposals,self.shortlisted_final_sequences,self.structural_quality_bins)<1:
            raise ValueError('search budgets must be >0')
        if min(self.route_cost_per_m,self.stop_cost_per_goal)<0:
            raise ValueError('negative cost')

@dataclass
class _State:
    points:tuple
    length_cells:int
    visible:tuple[int,int]
    quality:float=0.
    objective:float=0.
    shortfall:float=0.

class _Visibility:
    def __init__(self,obs,mean,variance,mission,unknown_flat,cfg,beams,step):
        self.obs=obs;self.mission=mission;self.config=cfg;self.beams=beams;self.step=step
        self.target_flat=np.flatnonzero(unknown_flat.ravel())
        self.bitcount=int(self.target_flat.size)
        self.predictions=[mean,np.clip(mean+cfg.pessimistic_sigma*np.sqrt(variance),0,1)]
        self.blockers=[(obs==2)|(~mission)|((obs==1)&(p>=.5)) for p in self.predictions]
        self.ang=np.linspace(0,2*pi,beams,endpoint=False,dtype=np.float32)[:,None]
        self.rr=np.arange(0,cfg.range_m/cfg.resolution_m+step/2,step,dtype=np.float32)[None,:]
        self.cache={}
        self.sight_calls=0

    def _project(self,p):
        r,c=p
        yy=np.rint(r+np.cos(self.ang)*self.rr).astype(np.int32)
        xx=np.rint(c+np.sin(self.ang)*self.rr).astype(np.int32)
        H,W=self.obs.shape
        valid=(yy>=0)&(yy<H)&(xx>=0)&(xx<W)
        y=np.clip(yy,0,H-1);x=np.clip(xx,0,W-1)
        projected=[]
        for blockers in self.blockers:
            stop=(~valid)|blockers[y,x]
            seen=(~np.maximum.accumulate(stop,axis=1))&valid
            hits=np.where(stop,np.arange(stop.shape[1])[None,:],stop.shape[1]).min(axis=1)
            include=seen|((np.arange(stop.shape[1])[None,:]==hits[:,None])&valid)
            mask=np.zeros(self.obs.shape,bool);mask[y[include],x[include]]=True
            bitarr=np.packbits(mask.ravel()[self.target_flat],bitorder='little')
            projected.append(int.from_bytes(bitarr.tobytes(),'little'))
        self.sight_calls+=1
        return tuple(projected)

    def sight(self,p):
        if p not in self.cache:self.cache[p]=self._project(p)
        return self.cache[p]

    def along(self,path):
        poses=path[self.config.scan_every_cells::self.config.scan_every_cells]
        if not poses or poses[-1]!=path[-1]:poses=list(poses)+[path[-1]]
        nominal=pessimistic=0
        for p in poses:
            x,y=self.sight(p)
            nominal|=x;pessimistic|=y
        return nominal,pessimistic

class StructuralRoutePlanner:
    def __init__(self,obs,mean,variance,semantic,robot,memory,report,cfg=None):
        self.cfg=cfg or SequenceConfig()
        _validate(obs,mean,variance,semantic,robot)
        self.obs=obs;self.mean=mean;self.variance=variance;self.semantic=semantic;self.robot=tuple(robot)
        self.memory=memory;self.report=report
        if report['snapshot_sha256']!=sha(obs) or report['mean_sha256']!=sha(mean) or report['semantic_sha256']!=sha(semantic):
            raise ValueError('structural report does not match input snapshot; reject stale or future metadata')
        if not np.array_equal(memory.previous_observed,obs):
            raise ValueError('memory state must already be analyzed at this exact snapshot')
        self.current=accepted_component(obs,semantic,self.robot)
        if not self.current.any():raise ValueError('no robot-connected accepted room')
        self.unknown=memory.mission&(obs==1)
        if self.unknown.sum()!=report['mission_unknown']:raise ValueError('memory/report mission mismatch')
        self.scfg=StructuralConfig(resolution_m=self.cfg.resolution_m,
            clearance_cells=self.cfg.clearance_cells,grid_step_cells=self.cfg.grid_stride_cells)
        self.legal=_navigable(obs,self.current,self.robot,self.scfg)
        self.parent0,self.dist0=_bfs(self.robot,self.legal)
        self.paths_cache={self.robot:(self.parent0,self.dist0)}
        self.legs_cache={}
        self.all_rows=report['regions']
        self.major=[r for r in self.all_rows if r['cells']>=self.cfg.min_major_region_cells
            and r['expected_open_cell_mass_proxy']>=self.cfg.min_major_expected_open_mass]
        self.major=sorted(self.major,key=lambda x:(-x['expected_open_cell_mass_proxy'],x['region_id']))
        self.rids=[r['region_id'] for r in self.major]
        self.region_bits={}
        indices=np.flatnonzero(self.unknown.ravel())
        self.target_rc=[tuple(int(q) for q in v) for v in np.argwhere(self.unknown)]
        for r in self.major:
            # One integer bitset for every region; constant-time union by bitwise OR.
            mask=(memory.labels.ravel()[indices]==r['region_id'])
            self.region_bits[r['region_id']]=int.from_bytes(np.packbits(mask,bitorder='little').tobytes(),'little')
        self.region_scales=[log1p(float(r['expected_open_cell_mass_proxy'])) for r in self.major]
        self.region_targets=[max(1,int(ceil(self.cfg.region_satisfaction_fraction*r['cells']))) for r in self.major]
        self.open_major_index=[i for i,r in enumerate(self.major) if r['kind']=='POSSIBLE_OPEN_SPACE']
        self.open_region_weight_total=sum(self.region_scales[i] for i in self.open_major_index)
        self.coarse=_Visibility(obs,mean,variance,memory.mission,self.unknown,self.cfg,
                               self.cfg.coarse_beams,self.cfg.coarse_ray_step_cells)
        self.fine=None

    def _bfs(self,src):
        if src not in self.paths_cache:self.paths_cache[src]=_bfs(src,self.legal)
        return self.paths_cache[src]

    def _leg(self,src,dst,ray):
        key=(src,dst,id(ray))
        if key in self.legs_cache:return self.legs_cache[key]
        parent,dist=self._bfs(src)
        if dst not in parent:return None
        path=_route(parent,src,dst)
        visibility=ray.along(path)
        result=(dist[dst],visibility,path)
        self.legs_cache[key]=result
        return result

    def quality(self,masks,length_cells,goals):
        if not self.major:return 0.,0.,0.
        a,b=masks
        q=0.;deficit=0.;tot=0.
        for rid,scale,target in zip(self.rids,self.region_scales,self.region_targets):
            rbit=self.region_bits[rid]
            n=min((a&rbit).bit_count()/target,1.)
            p=min((b&rbit).bit_count()/target,1.)
            frac=self.cfg.nominal_scenario_fraction*n+(1-self.cfg.nominal_scenario_fraction)*p
            q+=scale*frac
            deficit+=scale*(1-frac)
            tot+=scale
        # Two-layer soft constraint. First reward complementary region resolution;
        # then charge incremental travel and goal overhead, without hard thresholding
        # each immediate prefix (lookahead is allowed).
        obj=self.cfg.coverage_value_scale*q-self.cfg.route_cost_per_m*length_cells*self.cfg.resolution_m-self.cfg.stop_cost_per_goal*goals
        return q,obj,deficit

    def _structural_priority(self,masks,length_cells,goals):
        """Two-level mission utility: meaningful open-space obligations FIRST,
        then travel and goal overhead when region coverage is equivalent.

        Unknown predicted-wall regions remain unresolved as explicit secondary
        evidence, never forced to 100% based on an uncalibrated mean map.
        """
        nom=masks[0]
        count=0
        score=0.
        for i in self.open_major_index:
            reg=self.major[i]
            visible=(nom&self.region_bits[reg['region_id']]).bit_count()
            target=self.region_targets[i]
            if visible>=target:count+=1
            score+=self.region_scales[i]*min(visible/target,1.)
        frac=score/max(self.open_region_weight_total,1e-9)
        quality_bin=int(round(frac*self.cfg.structural_quality_bins))
        # A small difference in coverage is not worth arbitrary detours;
        # within the same 2%-quality band prefer route length and fewer goals.
        # These are planning priorities, NOT robot-side completion claims.
        return (count,quality_bin,-round(length_cells*self.cfg.resolution_m,3),
                -goals)

    def _state_priority(self,st):
        return self._structural_priority(st.visible,st.length_cells,len(st.points))

    def prepare_candidates(self):
        points=[p for p in self.parent0 if p!=self.robot and
            p[0]%self.cfg.grid_stride_cells==self.robot[0]%self.cfg.grid_stride_cells and
            p[1]%self.cfg.grid_stride_cells==self.robot[1]%self.cfg.grid_stride_cells]
        points=sorted(points)
        if len(points)>self.cfg.max_initial_viewpoint_proposals:
            idx=np.linspace(0,len(points)-1,self.cfg.max_initial_viewpoint_proposals,dtype=int)
            points=[points[i] for i in idx]
        # Structural witnesses are non-final candidates, never hardcoded coordinates.
        for row in self.major:
            for witness in row.get('top_conditional_viewpoints',[]):
                point=tuple(witness['core_rc'])
                if point in self.parent0 and point!=self.robot and point not in points:
                    points.append(point)
        self.candidate_initial_count=len(points)
        cached={}
        base=(0,0)
        for point in points:
            edge=self._leg(self.robot,point,self.coarse)
            if edge is None:continue
            length,masks,_=edge
            q,obj,deficit=self.quality(masks,length,1)
            cached[point]=(q,obj,deficit,masks,length)
        # Region-specific top K by pathwise coverage; not stationary visibility.
        # Reserve viewpoints for EACH unresolved region before applying the
        # global candidate budget. An endpoint with low total gain can be the
        # unique useful witness for a smaller occluded branch.
        per_region=[]
        for rid in self.rids:
            bit=self.region_bits[rid]
            rank=sorted(cached,key=lambda p:(
                -(.7*(cached[p][3][0]&bit).bit_count()+.3*(cached[p][3][1]&bit).bit_count()),
                cached[p][4],p))
            per_region.append(rank)
        reserved=[];taken=set()
        for k in range(4):
            for rank in per_region:
                if k>=len(rank):continue
                point=rank[k]
                if point not in taken and len(reserved)<self.cfg.max_search_candidates:
                    reserved.append(point);taken.add(point)
        picked=set(reserved)
        # Complete pathwise highest-score candidates; various distance-efficiency options.
        picked.update(sorted(cached,key=lambda p:(-cached[p][1],cached[p][4],p))[:18])
        picked.update(sorted(cached,key=lambda p:(cached[p][4],-cached[p][0],p))[:10])
        # Spatial diversity to preserve remote / transit viewpoints.
        cells={}
        for p in sorted(cached,key=lambda x:(-cached[x][0],cached[x][4],x)):
            sector=(p[0]//16,p[1]//16)
            cells.setdefault(sector,p)
        picked.update(cells.values())
        ordered=sorted((p for p in picked if p not in taken),
                       key=lambda p:(-cached[p][1],cached[p][4],p))
        # Invariant: do not evict a region witness solely because a large
        # open region has many higher-scoring viewpoints.
        self.candidates=sorted(reserved+ordered[:max(0,self.cfg.max_search_candidates-len(reserved))])
        self.region_reserved_count=len(reserved)
        self.first_scores=cached
        return self.candidates

    def _candidate_state(self,state,goal,ray):
        src=state.points[-1] if state.points else self.robot
        edge=self._leg(src,goal,ray)
        if edge is None:return None
        length,mask,_=edge
        coverage=(state.visible[0]|mask[0],state.visible[1]|mask[1])
        new=_State(state.points+(goal,),state.length_cells+length,coverage)
        new.quality,new.objective,new.shortfall=self.quality(coverage,new.length_cells,len(new.points))
        return new

    def _sequence_visibility(self,points,ray):
        pos=self.robot;masks=(0,0);length=0;stages=[]
        for point in points:
            edge=self._leg(pos,point,ray)
            if edge is None:raise ValueError('planned goal unreachable on observed FREE')
            distance,bits,path=edge
            new=(masks[0]|bits[0],masks[1]|bits[1])
            q,obj,deficit=self.quality(new,length+distance,len(stages)+1)
            stage={'goal_rc':list(point),'goal_xy_m':[round(point[1]*self.cfg.resolution_m,2),round(point[0]*self.cfg.resolution_m,2)],
                'distance_leg_m':round(distance*self.cfg.resolution_m,2),
                'path_rc':[list(x) for x in path],
                'newly_visible_unknown_nominal':(bits[0]&~masks[0]).bit_count(),
                'newly_visible_unknown_stress':(bits[1]&~masks[1]).bit_count(),
                'cumulative_quality_score':round(q,4),'objective':round(obj,4)}
            stages.append(stage)
            masks=new;length+=distance;pos=point
        quality,objective,shortfall=self.quality(masks,length,len(points))
        structural_priority=self._structural_priority(masks,length,len(points))
        return {'points_rc':[list(g) for g in points],'goal_count':len(points),'distance_m':round(length*self.cfg.resolution_m,2),
            'quality_score':round(quality,5),'shortfall_score':round(shortfall,5),'objective_score':round(objective,5),
            'structural_regions_nominal_met':structural_priority[0],
            'structural_quality_bucket':structural_priority[1],
            'projected_unseen_major_region_cells_nominal':int(sum((rbit&~masks[0]).bit_count() for rbit in self.region_bits.values())),
            'stages':stages,'scenario':ray.beams,'heuristic_satisfaction_not_real_coverage':True,
            'scenario_visible_bits':masks}

    def _selected_region_scores(self,masks):
        vals=[]
        for r,scale,target in zip(self.major,self.region_scales,self.region_targets):
            rid=r['region_id'];mb=self.region_bits[rid]
            nom=(mb&masks[0]).bit_count();pess=(mb&masks[1]).bit_count()
            vals.append({'region_id':rid,'kind':r['kind'],'cells':r['cells'],
                'expected_open_mass_proxy':r['expected_open_cell_mass_proxy'],
                'satisfaction_cap_cells':target,'predicted_visible_nominal':nom,'predicted_visible_stress':pess,
                'nominal_target_met':nom>=target,'stress_target_met':pess>=target,
                'historically_outside_current_room_cells':r['historically_retained_outside_current']})
        return vals

    def solve(self):
        begin=perf_counter()
        if not self.unknown.any():
            return {'status':'OBSERVED_MISSION_NO_UNKNOWN','goal_sequence':[],'decision_id':self.cfg.decision_id,
                    'caution':'Only observed mission pixels are counted for completion'}
        if not self.major:
            return {'status':'DEFER_NO_MAJOR_REGIONS','goal_sequence':[],'decision_id':self.cfg.decision_id,
                    'mission_unknown':int(self.unknown.sum()),'regions_total':len(self.all_rows)}
        choices=self.prepare_candidates()
        if not choices:
            return {'status':'DEFER_NO_REACHABLE_VIEWPOINT','goal_sequence':[],
                    'mission_unknown':int(self.unknown.sum()),'regions_total':len(self.all_rows)}
        root=_State((),0,(0,0))
        pool=[root];all_states=[root];end_reasons=[]
        for depth in range(1,min(self.cfg.max_search_depth,len(choices))+1):
            successors=[]
            for st in pool:
                for candidate in choices:
                    if candidate in st.points:continue
                    trial=self._candidate_state(st,candidate,self.coarse)
                    if trial is not None:successors.append(trial)
            if not successors:
                end_reasons.append('NO_MORE_SAFE_GOALS')
                break
            # Global beam keeps full-route quality/distance, plus some states
            # with strong structural coverage even when current net score is lower.
            unique={}
            for st in successors:
                signature=st.points
                if signature not in unique or st.objective>unique[signature].objective:
                    unique[signature]=st
            ranking=sorted(unique.values(),key=lambda z:(self._state_priority(z),z.objective),reverse=True)
            extras=sorted(unique.values(),key=lambda z:(-z.quality,z.length_cells,z.points))
            selected=ranking[:max(1,self.cfg.beam_width-2)]
            for st in extras:
                if len(selected)>=self.cfg.beam_width:break
                if st.points not in {x.points for x in selected}:selected.append(st)
            pool=selected
            all_states.extend(selected)
            # No fixed 1-/2-/3-goal rule. End when all major regions saturate
            # and additional full-route beam expansion has only negative net utility.
            if depth>=2 and selected and max(s.quality for s in selected)>=sum(self.region_scales)-.01:
                end_reasons.append('SATURATED_MAJOR_REGIONS')
                break
        # Validate diverse sequences, not just the coarse top 1 (mitigates
        # sparse-ray artifacts that previously produced redundant G3 at D02).
        uniq={st.points:st for st in all_states if st.points}
        ranked=sorted(uniq.values(),key=lambda s:(self._state_priority(s),s.objective),reverse=True)
        top=ranked[:self.cfg.shortlisted_final_sequences]
        for st in sorted(uniq.values(),key=lambda s:(-s.quality,s.length_cells,s.points)):
            if len(top)>=self.cfg.shortlisted_final_sequences+4:break
            if st.points not in {x.points for x in top}:top.append(st)
        # Explicit prefix and deletion comparisons in dense verification.
        augmented={st.points for st in top}
        for points in list(augmented):
            for j in range(len(points)):
                sub=points[:j]+points[j+1:]
                if sub:augmented.add(sub)
            for i in range(1,len(points)):
                augmented.add(points[:i])
        self.fine=_Visibility(self.obs,self.mean,self.variance,self.memory.mission,self.unknown,
                    self.cfg,self.cfg.dense_beams,self.cfg.dense_ray_step_cells)
        dense=[]
        for points in sorted(augmented,key=lambda p:(len(p),p)):
            result=self._sequence_visibility(points,self.fine)
            dense.append((points,result))
        dense.sort(key=lambda item:(
            item[1]['structural_regions_nominal_met'],item[1]['structural_quality_bucket'],
            -item[1]['distance_m'],-len(item[0]),item[1]['objective_score']),reverse=True)
        first=dense[0][1]
        no_goal_score=0.
        accepted=first['structural_quality_bucket']>0 and first['quality_score']>0
        if not accepted:return {'status':'DEFER_NO_POSITIVE_STRUCTURAL_CHAIN','goal_sequence':[],
                     'mission_unknown':int(self.unknown.sum()),
                     'candidate_count':len(choices),'major_regions':len(self.major),
                     'top_rejected_score':first['objective_score']}
        # Strip bit integers from user-facing JSON while preserving human auditable
        # region-level completeness details. Goals still conditional on observations.
        chosen={k:v for k,v in first.items() if k!='scenario_visible_bits'}
        by_region=self._selected_region_scores(first['scenario_visible_bits'])
        status='CONDITIONAL_VIEWPOINT_SEQUENCE'
        report={
            'version':'G2_R3_STRUCTURE_SEQUENCE_V1','status':status,
            'decision_id':self.cfg.decision_id,'snapshot_sha256':sha(self.obs),
            'input_used_gt':False,'future_observation_used':False,
            'goal_sequence':chosen['points_rc'],'goal_sequence_xy_m':[
                [round(p[1]*self.cfg.resolution_m,2),round(p[0]*self.cfg.resolution_m,2)]
                for p in (tuple(q) for q in chosen['points_rc'])],
            'goal_count':chosen['goal_count'],'total_route_m':chosen['distance_m'],
            'selected':chosen,'regions':by_region,
            'mission_unknown':int(self.unknown.sum()),'major_regions':len(self.major),
            'unresolved_minor_regions':[{'region_id':r['region_id'],'kind':r['kind'],
                'cells':r['cells'],'historically_outside_current_room':r['historically_retained_outside_current']} for r in self.all_rows if r not in self.major],
            'current_room_unknown':self.report['current_room_unknown'],
            'historical_unknown_outside_current':self.report['unknown_outside_current_room'],
            'search':{'initial_goal_proposals':self.candidate_initial_count,'chosen_goal_candidate_pool':len(choices),
                'beam_width':self.cfg.beam_width,'depth_bound':self.cfg.max_search_depth,
                'primary_objective':'lexicographic open-region satisfaction then nominal 2%-bucketed coverage, then travel/number of goals',
                'unverified_wall_regions_not_automatically_cleared':True,
                'coarse_full_sequence_candidates':len(uniq),'dense_rechecked_full_sequences':len(dense),
                'coarse_rays':self.cfg.coarse_beams,'dense_rays':self.cfg.dense_beams,
                'coarse_cached_scan_poses':len(self.coarse.cache),'fine_cached_scan_poses':len(self.fine.cache),
                'wall_time_s':round(perf_counter()-begin,3),'stopping_notes':end_reasons},
            'config':asdict(self.cfg),
            'cautions':[
                'All goals are conditional and lie in currently observed-free connected region, not model-predicted unknown',
                'Selection covers meaningful unresolved room regions, NOT a proof every free cell has been explored',
                'LaMa mean+variance generate two occupancy stress proxies, NOT calibrated ensemble probabilities',
                'An observed-free grid path is not a certified physical robot path',
                'Mission unknown outside current R4.1 room persists and must be adjudicated',
                'Search and candidate subsets are budgeted approximations; no global-optimality claim',
                'Run LaMa and R4.1 again after materially new observed evidence; do not follow frozen goals blindly']}
        return report
