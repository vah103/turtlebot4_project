"""Offline fresh scoring. Native replay is a separate audit/provenance stratum."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import numpy as np
from .contract import atomic_json, array_sha, file_sha, json_bytes, code_hashes
from .geometry import load
from .prediction import Predictor, source_modules
from .recorder import restore_state, ObjectStore
from .scoring import all_arms, score, selection_diagnostics, selected_index
from .audit import prediction_layers, physical_endpoint
from .resources import Guard

def unit(run_root, sid, output, geo, guard, predictor, binding_sha):
    meta, arrays = restore_state(run_root, sid)
    root = Path(output)
    job_key = {'snapshot_id': sid, 'state_sha256': meta['state_sha256'],
               'binding_sha256': binding_sha, 'adapter_files': code_hashes()}
    digest = hashlib.sha256(json_bytes(job_key)).hexdigest()
    destination = root/sid
    done = destination/'commit.json'
    if done.exists():
        previous = json.loads(done.read_text())
        assert previous['job_key_sha256'] == digest, 'SHADOW_RESUME_BINDING_GAP'
        for path, expected in previous['artifact_sha256'].items():
            assert file_sha(destination/path) == expected, 'SHADOW_RESUME_FILE_GAP'
        return previous
    destination.mkdir(parents=True, exist_ok=False)
    objects = ObjectStore(destination/'objects')
    obs, pose, history = arrays['obs_map'], arrays['pose'], arrays['pose_list']
    prediction, timing = predictor.predict(obs.copy())
    refs = {key: objects.put(value) for key, value in prediction.items()}
    sim, _, _ = source_modules()
    with guard.phase('shadow_frontier_generation'):
        pool, _, _ = sim.FrontierPlanner('visvarprob').get_frontier_centers_given_obs_map(obs)
        pool = np.asarray(pool, dtype=np.int64).reshape((-1, 2))
    with guard.phase('shadow_oracle_full_renderers'):
        arms, write = all_arms(pool, pose, history, prediction, geo)
        eligibility = selection_diagnostics(pool, pose, obs)
        for item in eligibility:
            item['path_object'] = objects.put(item.pop('path'))
        support_ref = objects.put(write)
        records = []
        for arm, result in arms.items():
            for row, ray, eligible in zip(result['rows'], result['rays'], eligibility):
                row = dict(row, **eligible)
                row.update(arm=arm, provenance='SHADOW_FRESH', snapshot_id=sid,
                           native_score_epoch_id=meta['native_score_epoch_id'],
                           write_support_sha256=support_ref['sha256'],
                           interpretation=('PRIMARY_PREDICTOR_EFFECT_SENSORGRID' if arm == 'V_GT_SENSORGRID_PREDONLY250'
                             else 'STRUCTURAL_P2_SENSITIVITY' if arm == 'V_GT_PREDONLY250' else 'NATIVE_SOURCE_BASELINE'),
                           ray_objects={key: objects.put(value) for key, value in ray.items()})
                records.append(row)
    with guard.phase('shadow_endpoint_sensor_diagnostic'):
        endpoints = physical_endpoint(pool, obs, geo)
        for row in endpoints:
            row['visibility_object'] = objects.put(row.pop('visibility'))
    goal = meta['locked_goal']
    locked = {'provenance': 'LOCKED_GOAL_REFERENCE', 'status': 'NO_LOCKED_GOAL',
              'goal': goal, 'candidate_pool_modified': False}
    if goal is not None:
        with guard.phase('shadow_locked_goal_reference'):
            _, row, ray = score(np.atleast_2d(goal), pose, history, prediction)
            locked.update(status='VALID' if np.isfinite(row[0]['score']) else 'NONFINITE_ZERO_DISTANCE',
                          row=row[0], ray_objects={key: objects.put(value) for key, value in ray[0].items()},
                          goal_in_fresh_pool=bool(np.any(np.all(pool == goal, axis=1))))
    with guard.phase('shadow_audit_io'):
        layers = prediction_layers(obs, prediction, geo)
        winners = {arm: selected_index(value['costs'], eligibility)
                   for arm, value in arms.items()}
        paired = {'support_sha256': support_ref['sha256'], 'support_cells': int(write.sum()),
                  'primary_native_vs_sensor': [winners['V_PRED_NATIVE250'], winners['V_GT_SENSORGRID_PREDONLY250']],
                  'raster_sensor_vs_p2': [winners['V_GT_SENSORGRID_PREDONLY250'], winners['V_GT_PREDONLY250']],
                  'whole_renderer_recomputed': True, 'local_mismatch_mask_certificate': False,
                  'status': 'NO_INTERVENTION_SUPPORT' if not write.any() else 'VALID'}
        paired['candidate_deltas'] = []
        for i in range(len(pool)):
            native, sensor, structural = [arms[name]['rows'][i] for name in
                       ['V_PRED_NATIVE250', 'V_GT_SENSORGRID_PREDONLY250', 'V_GT_PREDONLY250']]
            paired['candidate_deltas'].append({'candidate_id': native['candidate_id'],
              'native_to_sensor_score_delta': sensor['score']-native['score'],
              'sensor_to_p2_score_delta': structural['score']-sensor['score'],
              'native_to_sensor_variance_mass_delta': sensor['variance_mass']-native['variance_mass'],
              'sensor_to_p2_variance_mass_delta': structural['variance_mass']-sensor['variance_mass'],
              'native_to_sensor_rank_delta': sensor['source_sorted_rank']-native['source_sorted_rank'],
              'sensor_to_p2_rank_delta': structural['source_sorted_rank']-sensor['source_sorted_rank'],
              'native_to_sensor_visibility_changed_cells': int(np.count_nonzero(
                    arms['V_PRED_NATIVE250']['rays'][i]['visible_unknown'] !=
                    arms['V_GT_SENSORGRID_PREDONLY250']['rays'][i]['visible_unknown'])),
              'sensor_to_p2_visibility_changed_cells': int(np.count_nonzero(
                    arms['V_GT_SENSORGRID_PREDONLY250']['rays'][i]['visible_unknown'] !=
                    arms['V_GT_PREDONLY250']['rays'][i]['visible_unknown']))})
        artifact = {'job_key': job_key, 'provenance': 'SHADOW_FRESH', 'pose': pose.tolist(),
                    'predictions': refs, 'pool': pool.tolist(), 'candidates': records,
                    'paired': paired, 'endpoint_sensor': endpoints,
                    'arm_status': {arm: value['status'] for arm, value in arms.items()},
                    'pool_status': 'EMPTY_POOL' if not len(pool) else 'SINGLE_POOL' if len(pool) == 1 else 'MULTI_POOL',
                    'locked_goal_reference': locked, 'audit_layers': layers,
                    'timing': timing, 'corner_diagnostic': 'NOT_COMPUTED_EXPLORATORY'}
        atomic_json(destination/'shadow.json', artifact)
        # Flat per-epoch all-candidate ledger, including disabled/zero/NA rows.
        if records:
            with (destination/'all_candidates.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(records[0]))
                writer.writeheader()
                for row in records:
                    writer.writerow({key: json.dumps(value, sort_keys=True) if isinstance(value, (list, dict)) else value
                                     for key, value in row.items()})
        hashes = {str(p.relative_to(destination)): file_sha(p) for p in destination.rglob('*') if p.is_file()}
        commit = {'job_key_sha256': digest, 'artifact_sha256': hashes,
                  'snapshot_id': sid, 'state_sha256': meta['state_sha256'],
                  'candidate_count': len(pool), 'paired': paired, 'prediction_input_sha256': array_sha(obs),
                  'model_s': sum(row['load_s']+row['inference_s'] for row in timing['models']),
                  'committed_bytes': sum(p.stat().st_size for p in destination.rglob('*') if p.is_file())}
        atomic_json(done, commit)
        return commit

def execute(run_root, geometry, output, guard, selected_ids=None, resume=False):
    root, output = Path(run_root), Path(output)
    output.mkdir(parents=True, exist_ok=resume)
    geo, _ = load(geometry)
    manifest = json.loads((root/'manifest.json').read_text())
    binding_sha = hashlib.sha256(json_bytes(manifest['bindings'])).hexdigest()
    if selected_ids is None:
        with (root/'snapshots.csv').open() as stream:
            rows = list(csv.DictReader(stream))
        selected_ids = list(dict.fromkeys(r['snapshot_id'] for r in rows if r['event'] in
                                         ['INITIAL', 'DENSE_DISTANCE', 'TERMINAL']))
    predictor = Predictor(guard)
    results = [unit(root, sid, output, geo, guard, predictor, binding_sha) for sid in selected_ids]
    # Changes are descriptive on common unknown support, not weight learning.
    changes = []
    for previous, current in zip(results[:-1], results[1:]):
        artifacts = [json.loads((output/r['snapshot_id']/'shadow.json').read_text()) for r in [previous, current]]
        stores = [ObjectStore(output/r['snapshot_id']/'objects') for r in [previous, current]]
        means = [s.get(d['predictions']['mean']) for s, d in zip(stores, artifacts)]
        unknowns = [s.get(d['predictions']['unknown']) for s, d in zip(stores, artifacts)]
        support = np.zeros(means[0].shape, bool); support[506:709, 500:763] = geo['support']
        common = unknowns[0] & unknowns[1] & support
        delta = means[1]-means[0]
        changes.append({'previous_snapshot_id': previous['snapshot_id'],
             'snapshot_id': current['snapshot_id'], 'common_unknown_cells': int(common.sum()),
             'mean_absolute_prediction_change': float(np.abs(delta[common]).mean()) if common.any() else None,
             'interpretation': 'OBSERVATION_UPDATE_ONLY_NO_WEIGHT_LEARNING_NO_REALIZED_ACTION_CLAIM'})
    atomic_json(output/'prediction_changes.json', changes)
    if results:
        with (output/'dense_shadow_0p5m.csv').open('w', newline='') as stream:
            fields = ['snapshot_id', 'state_sha256', 'candidate_count', 'prediction_input_sha256',
                      'model_s', 'committed_bytes', 'paired']
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for result in results:
                writer.writerow({key: json.dumps(result[key], sort_keys=True) if isinstance(result[key], dict)
                                 else result[key] for key in fields})
    atomic_json(output/'shadow_summary.json', {'provenance': 'SHADOW_FRESH', 'snapshot_count': len(results),
                   'control_steps_are_not_snapshot_count': True,
                   'completed': results, 'scientific_execution_authorized': False})
    return results

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--run-root', required=True)
    p.add_argument('--geometry', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--resume', action='store_true')
    a = p.parse_args()
    with Guard(str(a.output)+'.resources', quota_roots=[a.run_root, a.output, str(a.output)+'.resources']) as guard:
        execute(a.run_root, a.geometry, a.output, guard, resume=a.resume)
