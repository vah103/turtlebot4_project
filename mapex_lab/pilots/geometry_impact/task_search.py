"""Whole-task bounds under the sealed mean-only correction class.

Search order may favour feasible/high-coverage prefixes. Only certified
distance bounds prune branches; heuristics never supply an upper bound.
"""
from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
import time

import numpy as np
from geometry_solver import GeometrySolver


class TaskBudget(RuntimeError):
    pass


class BoundFrontier:
    """Keeps parents live until all children/terminal bounds are recorded."""
    def __init__(self, baseline_steps):
        self.baseline_steps=int(baseline_steps)
        self.lower=0
        self.relaxed_terminal=0
        self.open={}

    def add(self,key,distance_steps):
        self.open[key]=max(0,self.baseline_steps-int(distance_steps))

    def close(self,key):
        del self.open[key]

    def hit(self,key,distance_steps,attainable):
        gain=max(0,self.baseline_steps-int(distance_steps))
        if attainable:
            self.lower=max(self.lower,gain)
        else:
            self.relaxed_terminal=max(self.relaxed_terminal,gain)
        self.close(key)

    def interval(self):
        return self.lower,max(self.lower,self.relaxed_terminal,max(self.open.values(),default=0))


def meets(counts,target):
    c,i,u,t=counts
    tc,ti,tu,tt=target
    numerator,denominator=(1,1) if u==0 else (i,u)
    tn,td=(1,1) if tu==0 else (ti,tu)
    return c>=tc and numerator*td>=tn*denominator and t>=tt


class TaskEvaluator:
    def __init__(self,engine,asset,seed):
        self.engine,self.asset=engine,asset
        self.start=tuple(int(x) for x in asset['start'])
        self.radius=engine.cfg['robot_radius_m']/engine.cfg['resolution_m']
        self.true_safe=engine.core.clearance_free(~asset['occupied'],self.radius)
        component=engine.core.reachable_component(self.true_safe,self.start)&asset['domain']
        legal=np.flatnonzero(component)
        if not len(legal):
            raise RuntimeError('No GT-reachable TU goal')
        rng=np.random.default_rng(seed)
        chosen=rng.choice(legal,size=100,replace=len(legal)<100)
        self.goals=[tuple(int(x) for x in np.unravel_index(flat,component.shape)) for flat in chosen]
        self.free_roi=asset['domain']&~asset['occupied']
        self.denominator=int(self.free_roi.sum())
        if not self.denominator:
            raise RuntimeError('Empty coverage denominator')

    def coverage_count(self,world):
        return int(((world.observed==0)&self.free_roi).sum())

    def counts(self,world,mean):
        completed=np.where(world.observed!=.5,world.observed,mean)>=.5
        truth,domain=self.asset['occupied'],self.asset['domain']
        intersection=int((completed&truth&domain).sum())
        union=int(((completed|truth)&domain).sum())
        safe=self.engine.core.clearance_free(~completed,self.radius)
        distances,parent=self.engine.fast.shortest_paths(safe,self.start)
        successes=0
        for goal in self.goals:
            if distances[goal]<0:
                continue
            path=self.engine.core.recover_path(parent,self.start,goal)
            if path is not None and all(self.true_safe[p] for p in path):
                successes+=1
        return self.coverage_count(world),intersection,union,successes

    def metrics(self,counts):
        c,i,u,t=counts
        return {'coverage':c/self.denominator,'occupied_iou':i/u if u else 1.,'TU':t/100.,'counts':list(counts)}


def distance_steps(world,resolution):
    steps=int(round(world.distance_m/resolution))
    if abs(world.distance_m-steps*resolution)>1e-6:
        raise RuntimeError('Non-cardinal or inconsistent accumulated distance')
    return steps


def run_p2(engine,adapter,cfg,out,clone_world,asset_from,save,event):
    began=time.perf_counter()
    deadline=began+cfg['p2_seconds']
    layout=cfg['p2_layout']
    asset=asset_from(cfg,layout)
    evaluator=TaskEvaluator(engine,asset,cfg['tu_seed'])
    save(out/'tu_goals.json',{'seed':cfg['tu_seed'],'start':list(evaluator.start),'goals':[list(p) for p in evaluator.goals],
                             'goal_rule':'fixed GT footprint-safe reachable component; 100 draws',
                             'evaluator':'fixed-source zero-heuristic A* / 4-neighbour shortest path, exact parent ordering; GT footprint collision check'})
    def check_time():
        if time.perf_counter()>=deadline:
            raise TaskBudget('900-second whole-task preflight cap')
    def predict(world):
        check_time()
        remaining=max(1,int(math.ceil(deadline-time.perf_counter())))
        engine.predictor.cfg['single_inference_timeout_s']=min(180,remaining)
        before=engine.core.array_hash(world.observed)
        value=engine.predictor.predict(world.observed)
        if before!=engine.core.array_hash(world.observed):
            raise RuntimeError('Predictor mutated observed map')
        return value

    world=adapter.world_from(engine.core,asset,cfg)
    root=clone_world(world)
    trace=[[0.,adapter.coverage(world,asset)]]
    frames=[]
    terminal='BUDGET_REACHED'
    reference_actions=[]
    try:
        while True:
            if len(frames)>=cfg['p2_task_nodes']:
                raise TaskBudget('Baseline consumed whole-task node cap')
            predictions,mean,var=predict(world)
            counts=evaluator.counts(world,mean)
            steps=distance_steps(world,cfg['resolution_m'])
            frame={'distance_steps':steps,'distance_m':steps*cfg['resolution_m'],
                   'counts':list(counts),'pose':list(world.pose),
                   'observed_hash':engine.core.array_hash(world.observed),
                   'mean_hash':engine.core.array_hash(mean),'variance_hash':engine.core.array_hash(var)}
            frames.append(frame)
            np.savez_compressed(out/f'baseline_state_{len(frames)-1:04d}.npz',observed=world.observed,pose=world.pose)
            save(out/'baseline_partial.json',{'frames':frames,'actions':reference_actions})
            event(stage='P2_BASELINE_STATE',states=len(frames),distance_m=frame['distance_m'],metrics=evaluator.metrics(counts),wall_s=time.perf_counter()-began)
            if world.distance_m>=cfg['budget_m']-cfg['resolution_m']/2:
                break
            candidates=engine.candidates(world)
            if not candidates:
                terminal='NO_SELECTABLE_CANDIDATE'
                break
            scores,selected,_=engine.score(world,mean,var,candidates)
            index=selected[adapter.BASE]
            candidate=candidates[index]
            record={'input':frame,'goal':list(candidate['goal']),'action_id':candidate['action_id'],
                    'planned_path':[list(p) for p in candidate['path']],'geometry':{'kind':'none'}}
            before=steps
            outcome=adapter.execute(world,candidate['path'],asset,cfg,trace)
            record['termination']=outcome
            record['executed_steps']=distance_steps(world,cfg['resolution_m'])-before
            reference_actions.append(record)
            if outcome=='COLLISION':
                raise RuntimeError('Baseline physical collision: primary valid-handoff preflight failed')
            if len(reference_actions)>=cfg['maximum_decisions']:
                raise TaskBudget('Baseline controller decision cap')
        target=tuple(frames[-1]['counts'])
        baseline_steps=min(f['distance_steps'] for f in frames if meets(tuple(f['counts']),target))
        q_ref=evaluator.metrics(target)
        baseline_outcome=adapter.outcome(world,asset,cfg,engine.predictor,trace,terminal)
        old=json.loads((Path(cfg['repo'])/'mapex_lab/pilots/visibility_impact_r4/results/run_20261008_controller_v2/summary.json').read_text())
        # R4's cohort results are ordered by the sealed layout list.
        expected=old['results'][cfg['layouts'].index(layout)]['reference']
        parity={k:abs(baseline_outcome[k]-expected[k])<=1e-9 for k in ('C_bar','Q','final_coverage','distance_m')}
        if not all(parity.values()):
            raise RuntimeError('A1 baseline differs from original R4 reference')
        replay=clone_world(root)
        replay_trace=[[0.,adapter.coverage(replay,asset)]]
        for j,action in enumerate(reference_actions):
            if engine.core.array_hash(replay.observed)!=action['input']['observed_hash']:
                raise RuntimeError('Baseline physical replay diverged at action input')
            outcome=adapter.execute(replay,[tuple(p) for p in action['planned_path']],asset,cfg,replay_trace)
            if outcome!=action['termination']:
                raise RuntimeError('Baseline controller termination replay mismatch')
        if engine.core.array_hash(replay.observed)!=frames[-1]['observed_hash']:
            raise RuntimeError('Baseline final observation replay mismatch')
        save(out/'baseline.json',{'status':'VALID_REFERENCE','terminal':terminal,'frames':frames,
                                'actions':reference_actions,'reference':q_ref,'first_hit_steps':baseline_steps,
                                'R4_reference_parity':parity,'physical_replay':True,'outcome':baseline_outcome})
    except (TaskBudget,adapter.ResourceAbort) as exc:
        save(out/'baseline_incomplete.json',{'reason':str(exc),'frames':frames,'actions':reference_actions})
        return {'status':'P2_BASELINE_INCOMPLETE','reason':str(exc),'whole_task_bounds_certified':False,
                'wall_s':time.perf_counter()-began,'baseline_states':len(frames)}

    bounds=BoundFrontier(baseline_steps)
    root_node={'id':0,'world':root,'attainable':True,'prefix':[],'decisions':0}
    bounds.add(0,0)
    heap=[(0,-evaluator.coverage_count(root),0,0,root_node)]
    sequence=0
    processed=0
    logs=[]
    relaxed_hits=[]
    best_prefix=[]
    best_counts=target
    inflight=None
    stop_reason='SEARCH_COMPLETE'
    def serialize(node):
        file=out/f'queue_state_{node["id"]:05d}.npz'
        np.savez_compressed(file,observed=node['world'].observed,pose=node['world'].pose,
                            distance_m=node['world'].distance_m,collisions=node['world'].collisions)
        return {'id':node['id'],'state_file':file.name,'attainable':node['attainable'],
                'decisions':node['decisions'],'prefix':node['prefix']}
    def progress():
        low,upper=bounds.interval()
        save(out/'progress.json',{'stage':'TASK_SEARCH','processed_nodes':processed,'baseline_states':len(frames),
                                 'open_nodes':len(bounds.open),'lower_m':low*cfg['resolution_m'],
                                 'upper_m':upper*cfg['resolution_m'],'wall_s':time.perf_counter()-began,
                                 'model_calls':engine.predictor.inner.calls,'cache_hits':engine.predictor.inner.cache_hits})
    try:
        while heap:
            check_time()
            if processed+len(frames)>=cfg['p2_task_nodes']:
                raise TaskBudget('120 total baseline/search node cap')
            _,_,_,_,node=heapq.heappop(heap)
            inflight=node
            world=node['world']
            d=distance_steps(world,cfg['resolution_m'])
            low,upper=bounds.interval()
            if max(0,baseline_steps-d)<=low:
                bounds.close(node['id'])
                inflight=None
                continue
            predictions,mean,var=predict(world)
            counts=evaluator.counts(world,mean)
            processed+=1
            observation_hash=engine.core.array_hash(world.observed)
            record={'id':node['id'],'distance_m':d*cfg['resolution_m'],'counts':list(counts),
                    'attainable':node['attainable'],'observed_hash':observation_hash,
                    'decisions':node['decisions']}
            if meets(counts,target):
                previous=bounds.lower
                bounds.hit(node['id'],d,node['attainable'])
                if node['attainable'] and bounds.lower>previous:
                    best_prefix=node['prefix']
                    best_counts=counts
                    save(out/'best_witness.json',{'prefix':best_prefix,'counts':list(counts),
                                                'gain_m':bounds.lower*cfg['resolution_m'],'observed_hash':observation_hash})
                elif not node['attainable']:
                    relaxed_hits.append({'id':node['id'],'distance_steps':d,'upper_gain_steps':max(0,baseline_steps-d),
                                         'prefix':node['prefix'],'counts':list(counts)})
                record['outcome']='TARGET_REACHED_PROVED' if node['attainable'] else 'TARGET_REACHED_UNPROVED_PREFIX'
                inflight=None
            elif d>=baseline_steps:
                bounds.close(node['id'])
                record['outcome']='CANNOT_IMPROVE_FIRST_HIT'
                inflight=None
            elif node['decisions']>=cfg['maximum_decisions']:
                # The controller limit is a resource stop, not a task proof.
                raise TaskBudget('Open prefix reached controller decision resource cap')
            else:
                candidates=engine.candidates(world)
                if not candidates:
                    bounds.close(node['id'])
                    record['outcome']='COMMON_CONTROLLER_TERMINAL_WITHOUT_TARGET'
                    inflight=None
                else:
                    solver=GeometrySolver(engine,world,mean,var,asset['occupied'],candidates)
                    result=solver.solve(max_nodes=cfg['p2_geometry_nodes'],seconds=min(cfg['p2_geometry_seconds'],max(.001,deadline-time.perf_counter())))
                    # Validate every used lower action against the frozen native
                    # scorer before declaring its child prefix attainable.
                    validated={}
                    native_scores,raw_selected,_=engine.score(world,mean,var,candidates)
                    raw_index=raw_selected[adapter.BASE]
                    if raw_index not in result['reachable_upper']:
                        raise RuntimeError('Geometry upper set excludes native baseline')
                    validated[raw_index]={'kind':'none'}
                    for index,certificate in result['witnesses'].items():
                        check_time()
                        if certificate['kind']=='none':
                            if raw_index!=int(index):
                                raise RuntimeError('Raw geometry witness failed native replay')
                            continue
                        corrected=mean.copy()
                        if certificate['kind']=='all_unknown':
                            corrected[world.observed==.5]=asset['occupied'][world.observed==.5]
                        elif certificate['kind']=='sparse':
                            cells=np.asarray(certificate['corrected_flat'],dtype=np.int64)
                            if len(cells):
                                if not np.all(world.observed.ravel()[cells]==.5):
                                    raise RuntimeError('Witness changed observed cells')
                                corrected.ravel()[cells]=asset['occupied'].ravel()[cells]
                        native_scores,chosen,_=engine.score(world,corrected,var,candidates)
                        if chosen[adapter.BASE]!=int(index):
                            raise RuntimeError('Geometry witness failed native replay')
                        validated[int(index)]=certificate
                    grouped={}
                    for index in result['reachable_upper']:
                        candidate=candidates[index]
                        action_id=candidate['action_id']
                        if action_id not in grouped or index in validated:
                            grouped[action_id]=index
                    for index in grouped.values():
                        check_time()
                        candidate=candidates[index]
                        child_world=clone_world(world)
                        outcome=adapter.execute(child_world,candidate['path'],asset,cfg,[])
                        child_d=distance_steps(child_world,cfg['resolution_m'])
                        if child_d<d:
                            raise RuntimeError('Non-monotone physical distance')
                        if outcome=='COLLISION':
                            # A failed child cannot supply a valid later handoff.
                            logs.append({'parent':node['id'],'action_index':index,'outcome':'COLLISION','distance_m':child_d*cfg['resolution_m']})
                            continue
                        sequence+=1
                        action={'observed_hash':observation_hash,'mean_hash':engine.core.array_hash(mean),
                                'variance_hash':engine.core.array_hash(var),'goal':list(candidate['goal']),
                                'action_id':candidate['action_id'],'planned_path':[list(p) for p in candidate['path']],
                                'geometry':validated.get(index),'termination':outcome,'executed_steps':child_d-d}
                        attainable=node['attainable'] and index in validated
                        child={'id':sequence,'world':child_world,'attainable':attainable,
                               'prefix':node['prefix']+[action],'decisions':node['decisions']+1}
                        bounds.add(sequence,child_d)
                        # This is only a search order. Bounds still include all
                        # low-priority/unattainable-prefix branches in the heap.
                        priority=(0 if attainable else 1,-evaluator.coverage_count(child_world),child_d,sequence,child)
                        heapq.heappush(heap,priority)
                    record.update(outcome='BRANCHED',geometry={k:v for k,v in result.items() if k not in ('subcubes',)},
                                  validated_lower_actions=sorted(validated),possible_actions=result['reachable_upper'])
                    bounds.close(node['id'])
                    inflight=None
            logs.append(record)
            progress()
            low,upper=bounds.interval()
            event(stage='P2_SEARCH_NODE',processed=processed,distance_m=d*cfg['resolution_m'],
                  lower_m=low*cfg['resolution_m'],upper_m=upper*cfg['resolution_m'],open_nodes=len(bounds.open),
                  outcome=record['outcome'],wall_s=time.perf_counter()-began)
            if upper-low<=round(1./cfg['resolution_m']):
                stop_reason='ONE_METRE_RESOLUTION_REACHED'
                break
    except (TaskBudget,adapter.ResourceAbort) as exc:
        stop_reason=str(exc)
    low,upper=bounds.interval()
    queued=[serialize(item[-1]) for item in heap if item[-1]['id'] in bounds.open]
    if inflight is not None and inflight['id'] in bounds.open:
        queued.append(serialize(inflight))
    save(out/'queue.json',{'open_bounds_steps':bounds.open,'queued':queued,'relaxed_terminal_upper_steps':bounds.relaxed_terminal,
                         'relaxed_hits':relaxed_hits,'incumbent_lower_steps':low,'stop_reason':stop_reason})
    save(out/'search_nodes.json',logs)
    progress()
    answer={'status':'P2_VALID_BOUNDS_RESOLVED_TO_ONE_METRE' if upper-low<=round(1./cfg['resolution_m']) else 'P2_VALID_BOUNDS_BUT_UNRESOLVED',
            'layout':layout,'baseline_first_hit_m':baseline_steps*cfg['resolution_m'],'baseline_budget_m':cfg['budget_m'],
            'baseline_reference':q_ref,'loss_lower_m':low*cfg['resolution_m'],'loss_upper_m':upper*cfg['resolution_m'],
            'relative_lower':low/baseline_steps if baseline_steps else 0.,'relative_upper':upper/baseline_steps if baseline_steps else 0.,
            'best_witness_counts':list(best_counts),'best_witness_actions':len(best_prefix),'baseline_states':len(frames),
            'processed_search_nodes':processed,'open_nodes':len(bounds.open),'unproved_target_prefixes':len(relaxed_hits),
            'stop_reason':stop_reason,'whole_task_bounds_certified':True,'wall_s':time.perf_counter()-began,
            'frequency_scope':'not estimated: one development task only',
            'secondary_area':{'status':'NOT_SEARCHED_TRIVIAL_BOUNDS_ONLY','lower_m2':0.,
                              'upper_m2':(evaluator.denominator-target[0])*cfg['resolution_m']**2},
            'causal_scope':'GT geometry repair class only; no global prediction adequacy or online recoverability verdict'}
    return answer
