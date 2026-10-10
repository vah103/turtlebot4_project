"""Use the untouched source scorer; record every ray and candidate result."""
import contextlib
import math
import numpy as np
from .contract import PRED_VIS, array_sha
from .prediction import source_modules

def score(pool, pose, pose_list, prediction, observer=None, mean_override=None):
    import torch
    sim, smu, _ = source_modules()
    planner = sim.FrontierPlanner('visvarprob')
    original_vis, original_val = smu.get_vis_mask, planner.get_frontier_val
    rays, values = [], []
    def capture_vis(*args, **kwargs):
        result = original_vis(*args, **kwargs)
        if kwargs.get('raycast_mode') == 'probabilistic':
            rays.append({'visibility': result[4].copy(), 'lidar_mask': result[1].copy(),
                         'hits': result[3].copy(), 'vis_indices': result[0].copy()})
        return result
    def capture_val(frontier_i, distances, obs_map, flooded_grid, var_map=None):
        value, filtered = original_val(frontier_i, distances, obs_map, flooded_grid, var_map)
        indices = np.argwhere(filtered)
        mass = torch.sum(var_map[indices[:, 0], indices[:, 1]])
        distance = float(distances[frontier_i])
        values.append({'candidate_index': int(frontier_i), 'candidate_id': 'c{:06d}'.format(frontier_i),
                       'distance_cells': distance, 'distance_m': distance*0.10,
                       'variance_mass': float(mass.item()), 'score': float(value.item()),
                       'visible_unknown_cells': int(filtered.sum()),
                       'visible_unknown_m2': int(filtered.sum())*0.01,
                       'zero_mass': bool(mass.item() == 0),
                       'nonfinite_score': not np.isfinite(value.item()),
                       'score_unit': 'VARIANCE_MASS_PER_GRID_CELL_DISTANCE',
                       'visibility_sha256': array_sha(filtered),
                       'support_reason': 'EMPTY_VISIBILITY' if not filtered.any() else 'VALID',
                       'intended_raw_rc': (np.asarray(pool[frontier_i])-500).tolist(),
                       'effective_raw_ray_rc': (np.asarray(pool[frontier_i])-[506, 500]).tolist()})
        rays[frontier_i]['visible_unknown'] = filtered.copy()
        return value, filtered
    smu.get_vis_mask, planner.get_frontier_val = capture_vis, capture_val
    try:
        # Empty pools preserve source failure semantics; diagnostics record NA separately.
        result = planner.score_frontiers(pool, pose, pose_list, prediction['pred_maputils'], PRED_VIS,
                    obs_map=prediction['padded_obs'],
                    mean_map=prediction['mean'] if mean_override is None else mean_override,
                    var_map=torch.from_numpy(prediction['variance']))
    finally:
        smu.get_vis_mask, planner.get_frontier_val = original_vis, original_val
    sorted_values = sorted(enumerate(values), key=lambda x: x[1]['score'], reverse=True)
    for rank, (index, _) in enumerate(sorted_values, 1):
        values[index]['source_sorted_rank'] = rank
        values[index]['source_argmin_selected'] = (index == int(np.argmin(result[1])))
        values[index]['all_zero_pool'] = bool(np.all(result[1] == 0))
        values[index]['tied_score_count'] = sum(v['score'] == values[index]['score'] for v in values)
    if observer is not None:
        observer(values, rays, result)
    return result, values, rays

def oracle_means(prediction, geo):
    from .contract import SHAPE_MODEL
    support = np.zeros(SHAPE_MODEL, bool)
    support[506:709, 500:763] = geo['support']
    write = prediction['unknown'] & support
    sensor, p2 = prediction['mean'].copy(), prediction['mean'].copy()
    sensor_truth, p2_truth = np.zeros(SHAPE_MODEL, np.float32), np.zeros(SHAPE_MODEL, np.float32)
    sensor_truth[506:709, 500:763] = geo['sensor_raw']
    p2_truth[506:709, 500:763] = geo['p2_anyocc']
    sensor[write], p2[write] = sensor_truth[write], p2_truth[write]
    np.testing.assert_array_equal(sensor[~write], prediction['mean'][~write])
    np.testing.assert_array_equal(p2[~write], prediction['mean'][~write])
    return {'V_PRED_NATIVE250': prediction['mean'],
            'V_GT_SENSORGRID_PREDONLY250': sensor, 'V_GT_PREDONLY250': p2}, write

def all_arms(pool, pose, pose_list, prediction, geo):
    means, write = oracle_means(prediction, geo)
    output = {}
    for arm, mean in means.items():
        if len(pool) == 0:
            output[arm] = {'rows': [], 'rays': [], 'costs': np.zeros(0),
                           'status': 'EMPTY_POOL_SOURCE_SCORE_NOT_CALLABLE'}
        else:
            result, rows, rays = score(pool, pose, pose_list, prediction, mean_override=mean)
            output[arm] = {'rows': rows, 'rays': rays, 'costs': result[1],
                           'status': 'NO_INTERVENTION_SUPPORT' if not write.any() else 'VALID'}
    return output, write

def selection_diagnostics(pool, pose, obs):
    """Offline source eligibility and all-candidate A*; never feeds acquisition."""
    import pyastar2d
    from .contract import LIDAR
    from .native_adapter import valid
    sim, _, _ = source_modules()
    planner_mapper = sim.Mapper(np.zeros_like(obs), LIDAR, use_distance_transform_for_planning=True)
    planner_mapper.obs_map = obs.copy()
    planning = planner_mapper.get_inflated_planning_maps(unknown_as_occ=True)
    rows = []
    for i, candidate in enumerate(pool):
        eligible = valid(candidate, planning, pose)
        path = pyastar2d.astar_path(planning, pose, candidate, allow_diagonal=False) if eligible else None
        reason = ('BELOW_SOURCE_STRICT_1M' if np.linalg.norm(candidate-pose) < 10 else
                  'PLANNING_OCCUPIED' if planning[candidate[0], candidate[1]] == np.inf else
                  'SOURCE_NO_ASTAR_PATH' if path is None else 'ELIGIBLE_REACHABLE')
        rows.append({'candidate_index': i, 'distance_eligible': bool(np.linalg.norm(candidate-pose) >= 10),
                     'source_valid_locked_goal': eligible, 'reachable': path is not None,
                     'disabled_reason': reason, 'path': np.empty((0, 2), int) if path is None else path,
                     'path_length_m': float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())*.10 if path is not None else None,
                     'path_provenance': 'AUDIT_GENERATED_PATH_NOT_EXECUTED'})
    return rows

def selected_index(costs, eligibility):
    remaining = list(range(len(costs)))
    while remaining:
        index = remaining[int(np.argmin(np.asarray(costs)[remaining]))]
        if eligibility[index]['source_valid_locked_goal'] and eligibility[index]['reachable']:
            return index
        remaining.remove(index)
    return None
