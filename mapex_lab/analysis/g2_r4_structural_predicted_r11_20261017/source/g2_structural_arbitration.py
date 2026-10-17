"""R11: preserve G2-R4 structural viewpoint quality when extending to predicted UNKNOWN.

Inputs are only current observed/mean/variance/semantic, G2 memory and report.
No GT, future scans or WFD frontier candidate coordinates are inputs.
Actual executed target must stay knownfree and native-route certified.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from mapex_g2_r3.sequence import StructuralRoutePlanner, SequenceConfig


@dataclass(frozen=True)
class ArbitrationConfig:
    max_quality_fraction_loss: float = .08
    max_structural_bin_loss: int = 1
    max_objective_loss: float = .4
    min_future_structural_gain: float = .08
    speculative_step_penalty: float = .012
    variance_penalty: float = .03
    max_unknown_steps: int = 24


def _quality(planner, goal):
    """G2-R3 pathwise structure on current native observed-safe trajectory."""
    goal = tuple(map(int, goal))
    edge = planner._leg(planner.robot, goal, planner.coarse)
    if edge is None:
        return None
    distance, masks, _ = edge
    q, objective, deficit = planner.quality(masks, distance, 1)
    satisfied, structural_bin, *_ = planner._structural_priority(masks, distance, 1)
    return {
        'goal_core_rc': list(goal), 'quality': float(q),
        'objective': float(objective), 'shortfall': float(deficit),
        'open_regions_satisfied': int(satisfied),
        'structural_quality_bin': int(structural_bin),
        'knownfree_distance_m': round(distance * planner.cfg.resolution_m, 3),
        'ray_masks': masks, 'distance_cells': int(distance),
    }


def _public(row):
    if row is None:
        return None
    return {k: v for k, v in row.items() if k not in ('ray_masks', 'distance_cells')}


def arbitrate(obs, mean, variance, semantic, robot, memory, report,
              native_walkable, baseline_goal, predicted_proposal,
              config=None, planner=None):
    """Compare R4 baseline, speculative terminal viewpoint and real FREE subgoal."""
    cfg = config or ArbitrationConfig()
    decision = {'decision': 'KEEP_R4_GOAL', 'reason': None,
                'baseline_goal_rc': list(baseline_goal) if baseline_goal is not None else None,
                'predicted_target_rc': None, 'safe_subgoal_rc': None,
                'used_frontier_as_goal': False, 'structural_comparison': True,
                'score_scale': 'G2_R3_structural_pathwise_quality'}
    if baseline_goal is None or predicted_proposal is None:
        decision['reason'] = 'NO_BASELINE_OR_PREDICTED_CANDIDATE'
        return decision
    baseline = tuple(map(int, baseline_goal))
    safe = tuple(map(int, predicted_proposal['safe_waypoint_rc']))
    target = tuple(map(int, predicted_proposal['predicted_target_rc']))
    decision['safe_subgoal_rc'] = list(safe)
    decision['predicted_target_rc'] = list(target)
    if not (obs.shape == mean.shape == variance.shape == semantic.shape):
        raise ValueError('snapshot epoch/shape mismatch')
    if np.asarray(native_walkable).shape != obs.shape:
        raise ValueError('native walkability shape mismatch')
    for goal in (baseline, safe, target):
        if len(goal) != 2 or not (0 <= goal[0] < obs.shape[0] and
                                 0 <= goal[1] < obs.shape[1]):
            decision['reason'] = 'GOAL_OUTSIDE_CURRENT_MAP'
            return decision
    if not (obs[baseline] == 0 and obs[safe] == 0 and
            native_walkable[baseline] and native_walkable[safe]):
        decision['reason'] = 'GOAL_NOT_NATIVE_CERTIFIED_FREE'
        return decision
    if not (obs[target] in (0, 1) and semantic[target] == 1):
        decision['reason'] = 'PREDICTED_TARGET_INVALID_OR_OUTSIDE_ROOM'
        return decision
    if planner is None:
        planner = StructuralRoutePlanner(obs, mean, variance, semantic,
                                         tuple(map(int, robot)), memory, report,
                                         cfg=SequenceConfig(decision_id=report['decision_id']))
    base = _quality(planner, baseline)
    prefix = _quality(planner, safe)
    decision['R4_knownfree_structural'] = _public(base)
    decision['executed_subgoal_structural'] = _public(prefix)
    if base is None or prefix is None:
        decision['reason'] = 'BASELINE_OR_SUBGOAL_ROUTE_NOT_STRUCTURALLY_REACHABLE'
        return decision
    if prefix['open_regions_satisfied'] < base['open_regions_satisfied']:
        decision['reason'] = 'SAFE_PREFIX_LOSES_REQUIRED_OPEN_ROOM_REGION'
        return decision
    if prefix['structural_quality_bin'] < base['structural_quality_bin'] - cfg.max_structural_bin_loss:
        decision['reason'] = 'SAFE_PREFIX_LOSES_STRUCTURAL_QUALITY_BIN'
        return decision
    if prefix['quality'] + 1e-10 < base['quality'] * (1.0 - cfg.max_quality_fraction_loss):
        decision['reason'] = 'SAFE_PREFIX_VIEW_QUALITY_TOO_LOW'
        return decision
    if prefix['objective'] < base['objective'] - cfg.max_objective_loss:
        decision['reason'] = 'SAFE_PREFIX_PATH_COST_TOO_HIGH'
        return decision
    if obs[target] == 0 and target == safe:
        decision['decision'] = 'USE_PREDICTED_GOAL'
        decision['reason'] = 'TARGET_ALREADY_CERTIFIED_AND_PREFIX_NOT_WORSE'
        return decision
    unknown_steps = int(predicted_proposal.get('hypothetical_unknown_steps', 0))
    if unknown_steps > cfg.max_unknown_steps:
        decision['reason'] = 'PREDICTED_TARGET_TOO_DEEP_IN_UNKNOWN'
        return decision
    # Terminal target is an unexecuted hypothesis. Discount its incremental
    # G2 structural coverage and DO NOT count it as safe route visibility.
    t_nom, t_stress = planner.coarse.sight(target)
    projected = (prefix['ray_masks'][0] | t_nom,
                 prefix['ray_masks'][1] | t_stress)
    projected_quality, _, _ = planner.quality(
        projected, prefix['distance_cells'], 1)
    raw_gain = max(0.0, projected_quality - prefix['quality'])
    reliability_heuristic = max(.15, 1.0 - .035 * unknown_steps)
    conservative_gain = (raw_gain * reliability_heuristic
                         - cfg.speculative_step_penalty * unknown_steps
                         - cfg.variance_penalty * float(variance[target]))
    decision['future_target_structural_gain'] = round(float(raw_gain), 5)
    decision['future_target_discounted_gain'] = round(float(conservative_gain), 5)
    decision['unknown_steps_hypothetical'] = unknown_steps
    decision['target_not_natively_traversable_until_observed'] = bool(obs[target] == 1)
    if conservative_gain < cfg.min_future_structural_gain:
        decision['reason'] = 'NO_MEANINGFUL_DISCOUNTED_FUTURE_STRUCTURAL_GAIN'
        return decision
    decision['decision'] = 'USE_PREDICTED_GOAL'
    decision['reason'] = 'STRUCTURAL_SAFE_PREFIX_PROTECTED_AND_FUTURE_GAIN'
    return decision


def choose_structural_backstop(obs, mean, variance, semantic, robot,
                               memory, report, native_walkable,
                               recent_goals=(), planner=None):
    """Prefer G2-R3 structural witnesses during R4 DEFER, never WFD goals."""
    if planner is None:
        planner = StructuralRoutePlanner(obs, mean, variance, semantic,
                                         tuple(map(int, robot)), memory, report,
                                         cfg=SequenceConfig(decision_id=report['decision_id']))
    if not planner.major:
        return None
    recent = [tuple(map(int, x)) for x in recent_goals[-12:]]
    candidates = planner.prepare_candidates()
    best = None
    for point in candidates:
        point = tuple(map(int, point))
        if not (obs[point] == 0 and native_walkable[point]):
            continue
        row = _quality(planner, point)
        if not row or row['quality'] <= 0:
            continue
        near = min((np.hypot(point[0]-g[0], point[1]-g[1]) for g in recent),
                   default=float('inf'))
        if near < 3.0:
            continue
        rank = (row['open_regions_satisfied'], row['structural_quality_bin'],
                row['objective'], -row['knownfree_distance_m'])
        if best is None or rank > best[0]:
            best = (rank, row)
    if best is None:
        return None
    return {'goal_core_rc': best[1]['goal_core_rc'],
            'generator': 'G2_R3_STRUCTURAL_WITNESS_BACKSTOP',
            'structural_view': _public(best[1]),
            'used_WFD_for_goal_generation': False,
            'reposition_only': False}
