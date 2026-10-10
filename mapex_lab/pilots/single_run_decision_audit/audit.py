"""Layer 0 plus six descriptive layers; truth remains offline."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from .contract import LIDAR, CONFIG, array_sha, atomic_json, json_bytes, stratum
from .geometry import coverage
from .prediction import source_modules
from .recorder import ObjectStore

def prediction_layers(obs, prediction, geo):
    # Actual physical P1 cell location in the model frame, not shifted ray origins.
    raw = obs[500:703, 500:763]
    mean = prediction['mean'][506:709, 500:763]
    variance = prediction['variance'][506:709, 500:763]
    known, unknown = raw != 0.5, raw == 0.5
    common = unknown & geo['support']
    result = {'layer0': {'known_cells_raw_arena': int(known.sum()),
               'known_sensor_grid_mismatch_cells': int((known & (raw != geo['sensor_raw'])).sum()),
               'sensor_geometry_sha256': array_sha(geo['sensor_raw']),
               'coverage_p1_projected_to_p2_roi': coverage(obs, geo),
               'coverage_denominator_m2': 469.225},
              'prediction': {}, 'uncertainty': {},
              'structural_reference_contract': 'P2_ANYOCC_2X2_FULL_SUPPORT',
              'primary_prediction_contract': 'SENSOR_GEOMETRY_P1_CONDITIONAL_ON_COMMON_W',
              'classification_threshold': 0.5, 'common_unknown_cells': int(common.sum())}
    for name, truth in [('sensor_grid', geo['sensor_raw']), ('p2_structural', geo['p2_anyocc'])]:
        predicted = mean >= 0.5
        wrong = predicted != (truth == 1)
        n = int(common.sum())
        result['prediction'][name] = {'count': n,
           'wrong_count': int((wrong & common).sum()),
           'wrong_rate': float(wrong[common].mean()) if n else None,
           'mae': float(np.mean(np.abs(mean[common]-truth[common]))) if n else None,
           'known_prediction_identity_mismatch_cells': int((known & (mean != raw)).sum())}
        bins = [0, .05, .10, .15, .20, .25, .30, .35, float('inf')]
        rows = []
        for lo, hi in zip(bins[:-1], bins[1:]):
            mask = common & (variance >= lo) & (variance < hi)
            rows.append({'lo': lo, 'hi': hi if np.isfinite(hi) else 'INF',
                         'count': int(mask.sum()), 'wrong_count': int((mask & wrong).sum())})
        result['uncertainty'][name] = {'diagnostic_only': True, 'bins': rows,
                 'variance_mass': float(variance[common].sum(dtype=np.float64)),
                 'variance_p95': float(np.percentile(variance[common], 95)) if n else None,
                 'unit': 'SAMPLE_VARIANCE_MASS_NOT_M2'}
    return result

def physical_endpoint(pool, obs, geo):
    sim, _, _ = source_modules()
    mapper = sim.Mapper(geo['mapper_gt'], LIDAR, use_distance_transform_for_planning=True)
    rows = []
    raw_unknown = obs[500:703, 500:763] == 0.5
    for i, p in enumerate(pool):
        scan = mapper.get_instant_obs_at_pose(p)
        seen = np.zeros(obs.shape, bool)
        seen[scan['vis_ind'][:, 0], scan['vis_ind'][:, 1]] = True
        seen[scan['actual_hit_points'][:, 0], scan['actual_hit_points'][:, 1]] = True
        new = seen[500:703, 500:763] & raw_unknown
        reason = 'CENTER_OCCUPIED_EMPTY_SOURCE_VISIBILITY' if geo['mapper_gt'][p[0], p[1]] == 1 else 'VALID'
        rows.append({'candidate_id': 'c{:06d}'.format(i),
             'visibility_sha256': array_sha(seen), 'visibility': seen,
             'hypothetical_new_known_cells_raw_arena': int(new.sum()),
             'hypothetical_new_known_area_m2': int(new.sum())*.01,
             'hypothetical_new_roi_area_m2': float(geo['roi_weights'][new].sum())*.0025,
             'support_reason': reason, 'reference': 'V_GT_SENSOR_ENDPOINT2500',
             'outcome': 'HYPOTHETICAL_NOT_REALIZED'})
    return rows

def native_replay(run_root):
    from .scoring import score
    root = Path(run_root)
    objects = ObjectStore(root/'objects')
    count = 0
    na = []
    path = root/'native_decisions.csv'
    if not path.exists():
        return {'status': 'NO_NATIVE_EPOCHS_RETAINED', 'count': 0}
    with path.open() as stream:
        decisions = list(csv.DictReader(stream))
    candidates = []
    if (root/'all_candidates_native.csv').exists():
        with (root/'all_candidates_native.csv').open() as stream:
            candidates = list(csv.DictReader(stream))
    for decision in decisions:
        epoch = decision['native_score_epoch_id']
        if decision.get('outcome') == 'NA_EXCEPTION_DURING_NATIVE_EPOCH':
            na.append({'native_score_epoch_id': epoch, 'reason': decision['na_reason']})
            continue
        rows = [row for row in candidates if row['native_score_epoch_id'] == epoch]
        assert len(rows) == int(decision['pool_count']), 'NATIVE_REPLAY_CANDIDATE_COUNT_GAP'
        refs = json.loads(decision['prediction_refs'])
        bundle = {key: objects.get(ref) for key, ref in refs.items()}
        pool = np.asarray([np.asarray(json.loads(r['intended_raw_rc']))+500 for r in rows])
        pose = np.asarray(json.loads(decision['pose']))
        result, replayed, rays = score(pool, pose, np.atleast_2d(pose), bundle)
        for original, replay, ray in zip(rows, replayed, rays):
            np.testing.assert_equal(float(original['score']), replay['score'])
            np.testing.assert_equal(float(original['variance_mass']), replay['variance_mass'])
            assert original['visibility_sha256'] == replay['visibility_sha256']
            for key, ref in json.loads(original['ray_objects']).items():
                np.testing.assert_array_equal(objects.get(ref), ray[key])
        assert result[0][np.argmin(result[1])].tolist() == json.loads(decision['winner'])
        count += 1
    return {'status': 'PASS', 'count': count, 'na_epochs_retained': na, 'provenance': 'NATIVE_REPLAY'}

def executed_actions(run_root, geo):
    sim, _, _ = source_modules()
    root = Path(run_root)
    objects = ObjectStore(root/'objects')
    mapper = sim.Mapper(geo['mapper_gt'], LIDAR, use_distance_transform_for_planning=True)
    scan_path = root/'raw_scans.csv'
    scans = {}
    chain = '0'*64
    with scan_path.open() as stream:
        for row in csv.DictReader(stream):
            item = json.loads(row['scan'])
            assert item['previous_chain'] == chain
            payload = {key: value for key, value in item.items() if key != 'chain_sha256'}
            assert hashlib.sha256(json_bytes(payload)).hexdigest() == item['chain_sha256'], 'RAW_SCAN_CHAIN_HASH_GAP'
            chain = item['chain_sha256']
            scans[int(row['control_step'])] = item
    with (root/'native_steps.csv').open() as stream:
        steps = list(csv.DictReader(stream))
    gains = {}
    per_step = []
    for k in sorted(scans):
        before = mapper.obs_map.copy()
        item = scans[k]
        scan = {key: objects.get(ref) for key, ref in item['objects'].items()}
        mapper.accumulate_obs_given_dict(scan)
        assert array_sha(mapper.obs_map) == item['post_observation_sha256']
        newly = (before[500:703, 500:763] == .5) & (mapper.obs_map[500:703, 500:763] != .5)
        gains[k] = {'known_cells': int(newly.sum()), 'known_area_m2': int(newly.sum())*.01,
                    'new_roi_area_m2': float(geo['roi_weights'][newly].sum())*.0025,
                    'coverage': coverage(mapper.obs_map, geo)}
    groups = {}
    origins = {}
    if (root/'native_decisions.csv').exists():
        with (root/'native_decisions.csv').open() as stream:
            origins = {row['native_score_epoch_id']: row['stratum'] for row in csv.DictReader(stream)}
    for step in steps:
        k, dispatch = int(step['control_step']), step['dispatch_id']
        key = (dispatch, stratum(k))
        group = groups.setdefault(key, {'dispatch_id': dispatch, 'stratum': stratum(k),
                 'native_score_epoch_id': step['native_score_epoch_id'], 'control_steps': 0,
                 'score_origin_stratum': origins.get(step['native_score_epoch_id'], 'NA_ORIGIN_UNKNOWN'),
                 'distance_m': 0., 'known_gain_m2': 0., 'new_roi_area_m2': 0., 'compute_and_io_s': 0.,
                 'native_compute_s': 0., 'recorder_s': 0.})
        gain = gains.get(k, {'known_area_m2': 0., 'new_roi_area_m2': 0., 'coverage': None})
        assert abs(float(step['known_gain_m2_raw_arena'])-gain['known_area_m2']) < 1e-12
        group['control_steps'] += 1
        group['distance_m'] += float(step['distance_m'])
        group['known_gain_m2'] += gain['known_area_m2']
        group['new_roi_area_m2'] += gain['new_roi_area_m2']
        group['compute_and_io_s'] += float(step['wall_compute_and_io_s'])
        group['native_compute_s'] += float(step.get('native_compute_wall_excluding_recorder_s', 0))
        group['recorder_s'] += float(step.get('recorder_io_s_step', 0))
        per_step.append(dict(gain, control_step=k, dispatch_id=dispatch, stratum=stratum(k)))
    # Dispatches with no executed step remain with explicit zero/no-realized-gain.
    if (root/'native_dispatch.csv').exists():
        with (root/'native_dispatch.csv').open() as stream:
            for dispatch in csv.DictReader(stream):
                key = (dispatch['dispatch_id'], dispatch['stratum'])
                groups.setdefault(key, {'dispatch_id': key[0], 'stratum': key[1],
                      'native_score_epoch_id': dispatch['native_score_epoch_id'], 'control_steps': 0,
                      'score_origin_stratum': origins.get(dispatch['native_score_epoch_id'], 'NA_ORIGIN_UNKNOWN'),
                      'distance_m': 0., 'known_gain_m2': 0., 'new_roi_area_m2': 0., 'compute_and_io_s': 0.,
                      'native_compute_s': 0., 'recorder_s': 0.})
    return list(groups.values()), per_step

def write_audit(run_root, geometry, output):
    from .geometry import load
    root, out = Path(run_root), Path(output)
    out.mkdir(parents=True, exist_ok=False)
    geo, _ = load(geometry)
    replay = native_replay(root)
    actions, steps = executed_actions(root, geo)
    for name, rows in [('executed_gain', actions), ('coverage_steps', steps)]:
        if rows:
            with (out/(name+'.csv')).open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    atomic_json(out/'audit_summary.json', {'native_replay': replay, 'action_stratum_rows': len(actions),
                'control_steps': len(steps), 'corner_opportunities': 'NOT_COMPUTED_EXPLORATORY',
                'limits': 'ONE_RUN_DESCRIPTIVE_NO_ALTERNATE_ACTION_REALIZED_GAIN'})
    return replay
