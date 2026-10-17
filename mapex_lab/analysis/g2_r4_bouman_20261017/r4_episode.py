"""G2-R4 episodic supervisor connecting safe events to R3 source-shaped planning.

Produces offline/advisory decisions only: NEVER sets robot_command or policy_action.
"""
from __future__ import annotations
from typing import Mapping,Sequence
from r4_adaptive import (R4Config,EvidenceLedger,PathValue,from_r3_snapshot,
                         safe_event,bounded_execution)


def prefixes(label, goals):
    """Evaluate every prefix; do not force a fixed number of viewpoints."""
    return {f'{label}:{k}':tuple(tuple(x) for x in goals[:k]) for k in range(1,len(goals)+1)}


def r3_proposal(obs,mean,variance,semantic,robot,memory,report,commitment_state=None):
    """Reuses R3/R3.1, not a replacement planner. Call ONLY after material event.

    R3.1 needs state containing observation feedback and long-lived region failures;
    it keeps the same original source and search budgets.
    """
    if commitment_state is None:
        from mapex_g2_r3.sequence import StructuralRoutePlanner
        return StructuralRoutePlanner(obs,mean,variance,semantic,robot,memory,report).solve()
    from mapex_g2_r3.commitment import ProgressAwarePlanner
    return ProgressAwarePlanner(obs,mean,variance,semantic,robot,memory,report,
                                commitment_state).solve()


def plan_episode(*, obs,mean,variance,semantic,robot,memory,report,
                 pending_goals:Sequence,exit_anchor, evidence=None,cfg=None,
                 event=None,fresh_goals=None,commitment_state=None,do_r3_search=True):
    """On heavy event, compare KEEP to every prefix of new/reused R3 chains.

    An exit/rejoin path is required for each prefix and its travel/scan benefit
    is included in the comparison. No prediction cell is made traversable.
    """
    cfg=cfg or R4Config();ledger=evidence or EvidenceLedger()
    if event is not None and not event.get('heavy',False):
        return {'decision':'HOLD_OR_FAST_KEEP','reason':'NO_HEAVY_EVENT',
                'robot_command':None,'policy_action':None,'unresolved_unknown_kept':True}
    proposal_source='SUPPLIED_FROZEN'
    if fresh_goals is None and do_r3_search:
        fresh=r3_proposal(obs,mean,variance,semantic,robot,memory,report,commitment_state)
        fresh_goals=fresh.get('goal_sequence',())
        proposal_source=fresh.get('status','UNKNOWN')
    if fresh_goals is None:fresh_goals=()
    options={}
    options.update(prefixes('PENDING',pending_goals))
    options.update(prefixes('R3_FRESH',fresh_goals))
    if not options:
        return {'decision':'DEFER_ROOM','reason':'NO_PROPOSAL','policy_action':None,
                'robot_command':None,'unresolved_unknown_kept':True}
    scorer,chains,invalid=from_r3_snapshot(obs,mean,variance,semantic,robot,memory,report,
                         options,exit_anchor,ledger,cfg,source_hash=report['snapshot_sha256'])
    old=next((c for c in chains if c.label==f'PENDING:{len(pending_goals)}'),None)
    new=[c for c in chains if c is not old]
    recommendation=scorer.choose(old,new)
    label=recommendation.get('selected_label')
    selection=next((c for c in chains if c.label==label),None)
    first_leg=selection.legs[0] if selection else None
    return {
       **recommendation,
       'selected_goals_rc':[list(leg.goal_rc) for leg in selection.legs if leg.kind=='view'] if selection else [],
       'execution_prefix_rc':bounded_execution(first_leg.path_rc,cfg) if first_leg else [],
       'nominal_source':'cached_LaMa_R3_visibility_scenarios',
       'fresh_proposal_source':proposal_source,'candidate_invalid':invalid,
       'snapshot_sha256':report['snapshot_sha256'],
       'robot_command':None,'policy_action':None,
       'is_physical_route_certified':False,
       'ground_truth_used':False,'future_observation_used':False,
       'no_global_stop':True,
       'unresolved_unknown_kept':True,
    }


def on_scan_fast(*, before,after,pose_rc,remaining_route,scan_id,last_heavy_scan,
                 at_goal=False,structural_changed=False,expected_cells=None,
                 realized_cells=None,cfg=None,clearance_cells=2,roi_mask=None):
    """Cheap event gate: never invokes LaMa, raycast, or R3 beam search."""
    cfg=cfg or R4Config()
    e=safe_event(before,after,pose_rc,remaining_route,
            at_goal=at_goal,structural_changed=structural_changed,
            expected_cells=expected_cells,realized_cells=realized_cells,
            scans_since_heavy=max(0,scan_id-last_heavy_scan),cfg=cfg,
            clearance_cells=clearance_cells,roi_mask=roi_mask)
    if e['event']=='SAFE_HOLD':
        return {**e,'decision':'SAFE_HOLD','execution_prefix_rc':[],
                'robot_command':None,'policy_action':None}
    if e['heavy']:
        return {**e,'decision':'HEAVY_EVALUATION_REQUIRED','execution_prefix_rc':[],
                'robot_command':None,'policy_action':None}
    return {**e,'decision':'KEEP_SHORT_PREFIX',
            'execution_prefix_rc':bounded_execution(remaining_route,cfg),
            'robot_command':None,'policy_action':None}
