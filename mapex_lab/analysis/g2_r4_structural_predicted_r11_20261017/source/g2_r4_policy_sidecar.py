"""G2-R4 (goal-chain version) COM1 native2D policy sidecar, Python >=3.11.
Source: same-epoch Room Node semantic + LaMa + observed occupancy.
Never receives hidden GT, future observations, robot commands or MapEx output goals.
"""
from __future__ import annotations
import sys,os,json,time,traceback
from pathlib import Path
from dataclasses import dataclass
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'g2src'))
from mapex_g2_r3.structural import MissionMemory,analyze,sha,StructuralConfig
from mapex_g2_r3.sequence import SequenceConfig
from mapex_g2_r3.commitment import CommitmentController,CommitmentState,expected_first_leg_per_region
from r4_episode import plan_episode
from r4_adaptive import EvidenceLedger,R4Config
from g2_predicted_goals import find_predicted
from g2_local_backstop import generate_local_viewpoint
from g2_structural_arbitration import arbitrate, choose_structural_backstop
from dataclasses import asdict

class Service:
    def __init__(self):
        self.active=None
        self.episode=0
    def choose(self,req):
        started=time.perf_counter()
        t=int(req['native_tick']);room_id=int(req['room_id'])
        robot=tuple(map(int,req['robot_core_rc']))
        with np.load(req['room_node_input'],allow_pickle=False) as n:
            obs=n['observed'].copy();mean=n['mean'].copy();var=n['variance'].copy()
        with np.load(req['room_node_semantic'],allow_pickle=False) as n:
            semantic=n['semantic'].copy()
        if not all(a.shape==obs.shape for a in (mean,var,semantic)):raise ValueError('snapshot mismatched shapes')
        if obs.dtype!=np.uint8 or not np.isin(obs,[0,1,2]).all():raise ValueError('bad observed coding')
        if not(0<=robot[0]<obs.shape[0] and 0<=robot[1]<obs.shape[1] and obs[robot]==0):
            raise ValueError('robot not on observed free')
        if self.active is None or self.active['room_id']!=room_id:
            memory=MissionMemory.begin(obs,semantic,robot)
            self.active={'room_id':room_id,'memory':memory,'controller':CommitmentController(),
                         'ledger':EvidenceLedger(),'before_obs':None,'before_labels':None,
                         'before_report':None,'before_pose':None,'last_goal':None,
                         'last_expected':{},'last_epoch':None,'predicted_target':None,'backstop_recent':[]}
        a=self.active
        if a['last_epoch'] is not None and t<=a['last_epoch']:raise ValueError('nonincreasing native tick')
        a['last_epoch']=t
        # Native MapEx first-hit LiDAR can refine a previously observed FREE
        # pixel into OCCUPIED. Preserve the latest wall (never clear it),
        # and reconcile prior evidence for the strict G2 memory invariant.
        # This is an auditable sensor correction, NOT predicted or GT input.
        corrected=[]
        previous=a['memory'].previous_observed
        conflicts=(previous!=1)&(obs!=previous)
        newly_blocked=conflicts&(previous==0)&(obs==2)
        invalid=conflicts&~newly_blocked
        if np.any(invalid):
            raise ValueError('unexpected native observed-map non-monotone transition')
        if np.any(newly_blocked):
            corrected=np.argwhere(newly_blocked).astype(int).tolist()
            previous[newly_blocked]=2
            if a['before_obs'] is not None:
                a['before_obs'][newly_blocked]=2
        # Update commitment evidence only if a selected goal was really reached.
        goal=a['last_goal']
        observation_event='NO_PRIOR_GOAL'
        if goal is not None:
            distance_cells=np.linalg.norm(np.array(robot)-np.array(goal))
            if distance_cells<=5 and a['controller'].state.pending_goals:
                previous=a['before_obs']
                before_report=dict(a['before_report'])
                before_report['first_leg_expected_by_region']=a['last_expected']
                if a['controller'].state.pending_goals[0]==list(goal):
                    arrival=a['controller'].record_arrival(previous,obs,a['before_labels'],
                         before_report,goal,a['before_pose'])
                    observation_event='COMPLETED_GOAL_OBS_'+str(arrival['realized_global_cells'])
                    expected_by_region=a['last_expected']
                    a['ledger'].observe_completed_leg(previous,obs,a['before_labels'],
                        expected_by_region,robot,t,R4Config())
            else:
                # Planner was called because old goal became invalid before reaching.
                # It was never an executed observation leg.
                a['controller'].state.pending_goals=[]
                observation_event='INVALIDATED_EARLY_NO_FALSE_GOAL_RECEIPT'
        region_cfg=StructuralConfig(beams=80,max_vantages=24)
        report,_,_=analyze(obs,mean,var,semantic,robot,a['memory'],
                           cfg=region_cfg,decision_id='native_tick_%06d'%t,include_vantage=False)
        old_pending=[list(x) for x in a['controller'].state.pending_goals]
        r31=a['controller'].decide(obs,mean,var,semantic,robot,a['memory'],report,
                                   sequence_cfg=SequenceConfig(decision_id=report['decision_id']))
        if r31.get('goal_sequence'):
            rec=plan_episode(obs=obs,mean=mean,variance=var,semantic=semantic,robot=robot,
                memory=a['memory'],report=report,pending_goals=old_pending,
                exit_anchor=robot,evidence=a['ledger'],cfg=R4Config(),
                event={'heavy':True},fresh_goals=r31['goal_sequence'],do_r3_search=False)
        else:
            rec={'decision':'DEFER_ROOM','reason':r31.get('reason','R31_NO_GOAL'),
                 'comparisons':[],'selected_goals_rc':[]}
        selected=[tuple(map(int,x)) for x in rec.get('selected_goals_rc',[])]
        selected_goal=selected[0] if selected else None
        if selected_goal is not None:
            a['controller'].state.pending_goals=[list(x) for x in selected]
            from mapex_g2_r3.commitment import check_remaining_goal
            leg,why=check_remaining_goal(obs,mean,var,semantic,robot,
                                          a['memory'],selected_goal,a['controller'].config)
            if why!='KEEP':
                selected_goal=None
                observation_event+=';G2_SELECTED_GOAL_INVALID_'+why
            else:
                expected=expected_first_leg_per_region(obs,mean,var,a['memory'],report,robot,leg['path_rc'])
                a['last_expected']=expected
        # Run10: Room Node already certified a LOCAL reachable WFD frontier.
        # WFD is an EXIT CONDITION ONLY, never a source of executable goals.
        # If structural G2-R4 DEFERs, G2 itself produces a visibility viewpoint.
        with np.load(req['native_scope_npz'],allow_pickle=False) as native:
            native_walkable=native['reachable_knownfree'].astype(bool)
        if native_walkable.shape!=obs.shape or not native_walkable[robot]:
            raise ValueError('native scope does not contain the robot')
        R4_unmodified_goal=list(selected_goal) if selected_goal is not None else None
        original_R4_defer=selected_goal is None
        local_backstop=None
        if original_R4_defer:
            # R11: R3 structural witnesses are the first fallback, not a
            # distance-to-unknown heuristic and NEVER the WFD frontier center.
            local_backstop=choose_structural_backstop(
                obs,mean,var,semantic,robot,a['memory'],report,native_walkable,
                recent_goals=a['backstop_recent'])
            if local_backstop is None:
                # Last-resort evidence-gathering reposition only. This is
                # explicitly NOT represented as a valuable R4 viewpoint.
                local_backstop=generate_local_viewpoint(obs,mean,var,semantic,robot,
                    a['memory'].mission,native_walkable,
                    recent_goals=a['backstop_recent'])
                if local_backstop is not None:
                    local_backstop['reposition_only']=True
            if local_backstop is not None:
                selected_goal=tuple(map(int,local_backstop['goal_core_rc']))
                assert obs[selected_goal]==0 and native_walkable[selected_goal]
                a['backstop_recent'].append(list(selected_goal))
                a['backstop_recent']=a['backstop_recent'][-12:]
                selected=[selected_goal]
                a['controller'].state.pending_goals=[list(selected_goal)]
                a['last_expected']={}
        if selected_goal is None:
            a['controller'].state.pending_goals=[]
            a['last_expected']={}
        a['last_goal']=selected_goal
        a['before_obs']=obs.copy();a['before_labels']=a['memory'].labels.copy()
        a['before_report']=report;a['before_pose']=robot
        # R09: after old R4 selects a safe goal, allow a deeper speculative
        # viewpoint only if current LaMa expects additional local information.
        # The *actual* robot command remains a currently observed FREE waypoint.
        predicted=None
        predicted_proposal=None
        structural_arbitration=None
        knownfree_baseline_goal=list(selected_goal) if selected_goal is not None else None
        R11_candidate_diagnostics=[]
        if selected_goal is not None:
            # The old predicted-proposal proxy only SHORTLISTS candidates.
            # Actual selection is G2-R3 pathwise structural arbitration.
            predicted_candidates=find_predicted(
                obs,mean,var,semantic,robot,a['memory'].mission,
                baseline=selected_goal,previous=a.get('predicted_target'),
                native_walkable=native_walkable,
                return_candidates=True,candidate_limit=4)
            accepted=[]
            for candidate in predicted_candidates:
                verdict=arbitrate(
                    obs,mean,var,semantic,robot,a['memory'],report,
                    native_walkable,selected_goal,candidate)
                R11_candidate_diagnostics.append({
                    'predicted_target_rc':candidate['predicted_target_rc'],
                    'safe_waypoint_rc':candidate['safe_waypoint_rc'],
                    'arbitration':verdict})
                if verdict['decision']=='USE_PREDICTED_GOAL':
                    prefix=verdict['executed_subgoal_structural']
                    rank=(prefix['open_regions_satisfied'],
                          prefix['structural_quality_bin'],
                          prefix['quality'],
                          verdict.get('future_target_discounted_gain',0.0),
                          prefix['objective'])
                    accepted.append((rank,candidate,verdict))
            if accepted:
                accepted.sort(key=lambda x:x[0],reverse=True)
                _,predicted,structural_arbitration=accepted[0]
                subgoal=tuple(int(x) for x in predicted['safe_waypoint_rc'])
                target=tuple(int(x) for x in predicted['predicted_target_rc'])
                if obs[subgoal]!=0 or not native_walkable[subgoal]:
                    raise ValueError('R11 selected non-certified subgoal')
                selected_goal=subgoal
                a['predicted_target']=target
                a['controller'].state.pending_goals=[list(subgoal)]
                a['last_goal']=subgoal
                a['last_expected']={} # never credit unvisited target
            else:
                # R4 wins against all speculative alternatives.
                a['predicted_target']=None
                structural_arbitration={'decision':'KEEP_R4_GOAL',
                    'reason':'NO_PREDICTED_CANDIDATE_PASSED_STRUCTURAL_ARBITRATION',
                    'evaluated_candidate_count':len(predicted_candidates)}
                predicted_proposal=predicted_candidates[0] if predicted_candidates else None
        else:
            a['predicted_target']=None
        self.episode+=1
        result={'event':'RESULT','native_tick':t,'room_id':room_id,
                'observed_sha256':sha(obs),'semantic_sha256':sha(semantic),
                'robot_core_rc':list(robot),
                'decision':'GOAL' if selected_goal is not None else 'DEFER',
                'goal_core_rc':list(selected_goal) if selected_goal else None,
                'selected_sequence_rc':[list(selected_goal)] if predicted else [list(x) for x in selected],
                'knownfree_R4_original_sequence_rc':[list(x) for x in selected],
                'R31_decision':r31['decision'],
                'R4_decision':rec['decision'],'R4_reason':rec.get('reason'),
                'run10_local_frontier_present':True,
                'run10_R4_defer_overridden_by_G2_backstop':bool(original_R4_defer and local_backstop),
                'run10_local_backstop':local_backstop,
                'R11_original_R4_knownfree_goal_core_rc':R4_unmodified_goal,
                'R11_structural_arbitration':structural_arbitration,
                'R11_candidate_diagnostics':R11_candidate_diagnostics,
                'R11_predicted_candidate_unexecuted':predicted_proposal if predicted is None else None,
                'R11_reposition_only':bool(local_backstop and local_backstop.get('reposition_only')),
                'R11_WFD_only_exit_gate':True,
                'run10_frontier_used_for_goal':False,
                'R4_candidate_scores':[{'label':x.get('label'),'score':x.get('objective'),
                    'conservative_score':x.get('conservative_objective'),
                    'distance_m':x.get('distance_m')} for x in rec.get('comparisons',[])],
                'prior_observation_event':observation_event,
                'native_observation_free_to_wall_corrections':corrected,
                'mission_unknown':report['mission_unknown'],
                'original_room_unknown':report['original_room_unknown'],
                'analysis_time_s':round(time.perf_counter()-started,4),
                'GT_used_for_selection':False,'future_observations_used':False,
                'fallback_is_global_mapex':selected_goal is None,'physical_safety_certified':False,
                'predicted_target_core_rc':predicted['predicted_target_rc'] if predicted else None,
                'predicted_goal_decision':predicted,
                'R4_knownfree_baseline_goal_core_rc':knownfree_baseline_goal,
                'executed_goal_is_observed_free':bool(selected_goal is None or obs[selected_goal]==0),
                'GT_used_for_prediction':False}
        return result

def main():
    service=Service()
    print(json.dumps({'event':'READY','G2':'GOAL_BASED_R4','python':sys.version.split()[0]}),flush=True)
    for line in sys.stdin:
        try:
            obj=json.loads(line)
            if obj.get('action')=='CLOSE':
                print(json.dumps({'event':'CLOSED'}),flush=True);break
            if obj.get('action')!='CHOOSE':raise ValueError('unsupported action')
            out=service.choose(obj)
        except Exception as e:
            out={'event':'ERROR','error_type':type(e).__name__,'error':str(e),
                 'traceback':traceback.format_exc()}
        print(json.dumps(out,ensure_ascii=False,allow_nan=False),flush=True)
if __name__=='__main__':main()
