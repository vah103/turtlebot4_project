"""Technical gates use the pinned source, immutable fixtures and a <=20-step smoke."""
import csv
import hashlib
import json
import math
import os
import subprocess
import sys
import zipfile
from pathlib import Path
import numpy as np
from .contract import CONFIG, LIDAR, START, SOURCE, array_sha, atomic_json, file_sha, json_bytes, stratum
from .geometry import to_raw_cell
from .prediction import Predictor, fixture_predict, source_modules
from .native_adapter import NativeRun
from .recorder import Recorder, restore_state, ObjectStore
from .scoring import score, oracle_means, all_arms
from .source_reference import run_reference
from .audit import native_replay, write_audit

def check_frontier_fixture():
    sim, _, _ = source_modules()
    # A center representative and >10 cells must match the exact source rule.
    obs = np.full((32, 32), .5, np.float32)
    obs[5:15, 5:15] = 0
    pool, filtered, n = sim.FrontierPlanner('visvarprob').get_frontier_centers_given_obs_map(obs)
    assert n == 1 and len(pool) == 1
    assert filtered.sum() == 36
    assert np.array_equal(pool[0], [5, 9])

def full_checks(repo, geometry, out, geo, guard, probe):
    import torch
    import pyastar2d
    from .preflight import bindings
    from .shadow import execute as shadow_execute, unit
    from .render_report import render
    sim, smu, _ = source_modules()
    results, evidence = {}, {}
    def mark(name, status='PASS'):
        results[name] = status
        atomic_json(out/'gates_checkpoint.json', results)
        atomic_json(out/'technical_checks_checkpoint.json', evidence)
    with guard.phase('P2_lattice_and_frame'):
        assert to_raw_cell('0', '0') == (131, 131)
        assert to_raw_cell('-13.15', '-13.15') == (0, 0)
        assert to_raw_cell('13.15', '7.15') == (203, 263)
        assert to_raw_cell('-13.150001', '-13.150001') == (-1, -1)
        evidence['frame'] = {'raw': [203, 263], 'mapper': [1203, 1263],
           'model': [1216, 1264], 'padding': [6, 7, 0, 1],
           'intended_pose_raw': [131, 131], 'effective_source_ray_raw': [125, 131],
           'domain': 52461, 'mismatch': 2994, 'primary_reference': 'SENSOR_GEOMETRY_P1'}
    with guard.phase('P3_ABI_fixture'):
        path = pyastar2d.astar_path(np.ones((8, 8), np.float32), (1, 1), (6, 6), allow_diagonal=False)
        assert len(path) == 11
        assert np.all(np.linalg.norm(np.diff(path, axis=0), axis=1) == 1)
    with guard.phase('P5_sensor_planning_collision'):
        reference_mapper = sim.Mapper(geo['mapper_gt'], LIDAR, use_distance_transform_for_planning=True)
        reference_mapper.observe_and_accumulate_given_pose(np.array(START))
        raw_hash = array_sha(reference_mapper.obs_map)
        inflated_true = reference_mapper.get_inflated_planning_maps(unknown_as_occ=True)
        inflated_false = reference_mapper.get_inflated_planning_maps(unknown_as_occ=False)
        np.testing.assert_array_equal(inflated_true, inflated_false)
        assert array_sha(reference_mapper.obs_map) == raw_hash
        from scipy.ndimage import binary_dilation
        native_dilation = binary_dilation(reference_mapper.obs_map > .5, structure=np.ones((3, 3)))
        unaffected_unknown = (reference_mapper.obs_map == .5) & ~native_dilation
        assert np.isfinite(inflated_true[unaffected_unknown]).all()
        direct = reference_mapper.inflate_map(reference_mapper.obs_map, unknown_as_occ=False)
        np.testing.assert_array_equal(inflated_true, direct)
        # Controller indexes the path endpoint without inspecting intermediate GT.
        synthetic_path = np.asarray([[0, 0], [1, 0], [2, 0], [3, 0], [4, 0]])
        np.testing.assert_array_equal(sim.psuedo_traj_controller(synthetic_path[:, 0], synthetic_path[:, 1], 3), [3, 0])
        evidence['sensor'] = {'initial_obs_sha256': raw_hash,
            'physical_geometry_sha256': array_sha(reference_mapper.gt_map),
            'unknown_as_occ_requested': True, 'effective': False,
            'inflation': 'ONE_SOURCE_3X3_DILATION_FROM_RAW_OBS',
            'collision': 'SOURCE_CENTER_CELL_AFTER_CUR_POSE_ASSIGN_BEFORE_APPEND_SCAN'}
        mark('P5_SENSOR_PLANNING_COLLISION')
    with np.load(str(out/'model_probe_outputs.npz'), allow_pickle=False) as f:
        bundle = {key: f[key] for key in f.files}
    with guard.phase('P6_source_score_parity'):
        check_frontier_fixture()
        pool, _, _ = sim.FrontierPlanner('visvarprob').get_frontier_centers_given_obs_map(reference_mapper.obs_map)
        source = sim.FrontierPlanner('visvarprob').score_frontiers(pool, np.array(START), np.atleast_2d(START),
                bundle['pred_maputils'], CONFIG['pred_visibility'], obs_map=bundle['padded_obs'],
                mean_map=bundle['mean'], var_map=torch.from_numpy(bundle['variance']))
        captured, rows, rays = score(pool, np.array(START), np.atleast_2d(START), bundle)
        for a, b in zip(source[:2], captured[:2]):
            np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(source[2], captured[2])
        assert source[4:] == captured[4:]
        assert len(rows) == len(pool) and len(rays) == len(pool)
        zero_bundle = dict(bundle, variance=np.zeros_like(bundle['variance']))
        zero_result, zero_rows, _ = score(pool, np.array(START), np.atleast_2d(START), zero_bundle)
        assert np.all(zero_result[1] == 0)
        assert all(row['all_zero_pool'] and row['tied_score_count'] == len(pool) for row in zero_rows)
        assert int(np.argmin(zero_result[1])) == 0
        empty, _ = all_arms(np.empty((0, 2), int), np.array(START), np.atleast_2d(START), bundle, geo)
        assert all(value['status'] == 'EMPTY_POOL_SOURCE_SCORE_NOT_CALLABLE' and not value['rows'] for value in empty.values())
        evidence['scoring'] = {'candidate_count': len(pool), 'costs_bit_identical': True,
              'source_selected_index': int(np.argmin(source[1])), 'recorded_all_candidates': True,
              'all_zero_ties_preserve_argmin_first_candidate': True,
              'empty_offline_pool_retains_NA_status_without_source_repair': True}
        mark('P6_SCORER_PARITY')
    # Twelve fixture control steps: source4 + port4 + recorder port4, then real2.
    # Fixture models are deterministic observation-only stand-ins, not scientific data.
    with guard.phase('P7_source_controller_reference'):
        source_trace = run_reference(geo['mapper_gt'], 4)
        unrecorded = NativeRun(geo['mapper_gt'], fixture_predict, guard)
        unrecorded.execute(4)
        _, bound = bindings(repo, geometry)
        fixture_recorder = Recorder(out/'fixture_recorded', bound, 'TECHNICAL_SMOKE_FIXTURE')
        recorded = NativeRun(geo['mapper_gt'], fixture_predict, guard, fixture_recorder)
        recorded.execute(4)
        for key in ['poses', 'observations', 'epochs', 'step_goals']:
            assert unrecorded.trace[key] == recorded.trace[key], 'RECORDER_MUTATED_'+key
            assert source_trace[key] == recorded.trace[key], 'SOURCE_PORT_GAP_'+key
        assert source_trace['source_terminal']['fail_reason'] == recorded.trace['terminal']['source_fail_reason']
        assert source_trace['final_pose_list'] == recorded.pose_list.tolist()
        evidence['source_loop_parity'] = {'reference_control_steps': 4,
               'recorder_off_control_steps': 4, 'recorder_on_control_steps': 4,
               'poses_maps_goals_pools_costs_terminals_equal': True,
               'model_kind': 'OBSERVATION_ONLY_FIXTURE_NOT_SCIENCE'}
        mark('P7_CONTROLLER_AND_RECORDER_INVARIANCE')
    with guard.phase('P8_immutable_sampling_fixture'):
        fixture = Recorder(out/'sampling_fixture', bound, 'SAMPLING_FIXTURE_NOT_TRAJECTORY')
        obs = np.full((5, 5), .5, np.float32)
        state = {'arrays': {'obs_map': obs}, 'meta': {'control_step': 1, 'distance_m': 1.7,
                   'native_score_epoch_id': 1, 'dispatch_id': 1}}
        sid = fixture.post_step(state)
        assert fixture.snapshot_count == 1 and fixture.next_target_index == 4
        with (fixture.root/'snapshots.csv').open() as stream:
            row = list(csv.DictReader(stream))[0]
        assert json.loads(row['target_distances_m']) == [0.5, 1.0, 1.5]
        obs[:] = 1
        _, restored = restore_state(fixture.root, sid)
        assert np.all(restored['obs_map'] == .5), 'MUTABLE_STATE_ALIAS'
        state['meta']['control_step'] = 2
        assert fixture.post_step(state) is None  # No movement = no distance checkpoint.
        fixture.snapshot('NO_MOTION_EVENT', state)
        # The boundary event must survive a simultaneous distance crossing.
        state['meta'].update(control_step=1000, distance_m=2.2)
        fixture.post_step(state)
        with (fixture.root/'snapshots.csv').open() as stream:
            boundary_rows = [row for row in csv.DictReader(stream) if row['control_step'] == '1000']
        assert [row['event'] for row in boundary_rows] == ['DENSE_DISTANCE', 'NATIVE_BUDGET_BOUNDARY']
        assert boundary_rows[0]['snapshot_id'] == boundary_rows[1]['snapshot_id']
        # Resolution bookkeeping from one trace, no interval selection by outcome.
        distances = [0, .3, .6, .9, 1.2]
        evidence['sampling'] = {'overshoot_multi_threshold_shared_state': True,
           'no_motion_event_retained': True,
           'same_trace_interval_counts': {str(interval): int(distances[-1]//interval) for interval in [.25, .5, 1.]},
           'interval_is_still_preregistered_m': .5}
        mark('P8_DENSE_SNAPSHOTS')
    with guard.phase('P9_native_replay_and_hashes'):
        replay = native_replay(fixture_recorder.root)
        assert replay['status'] == 'PASS'
        final_sid = json.loads((fixture_recorder.root/'terminal.json').read_text())['final_snapshot_id']
        meta, restored = restore_state(fixture_recorder.root, final_sid)
        np.testing.assert_array_equal(restored['obs_map'], recorded.mapper.obs_map)
        np.testing.assert_array_equal(restored['pose_list'], recorded.pose_list)
        # Scan chain reconstruction equals the exact accumulated-hit-point state.
        rebuilt = []
        with (fixture_recorder.root/'raw_scans.csv').open() as stream:
            for row in csv.DictReader(stream):
                item = json.loads(row['scan'])
                rebuilt.append(fixture_recorder.objects.get(item['objects']['actual_hit_points']))
        np.testing.assert_array_equal(np.concatenate(rebuilt), recorded.mapper.accum_hit_points)
        assert meta['accum_hit_points_count'] == len(recorded.mapper.accum_hit_points)
        evidence['replay'] = dict(replay, state_hashes_validated=True, scan_history_reconstructed=True)
        mark('P9_REPLAY_NO_GT_FEEDBACK')
    with guard.phase('P10_matched_oracles'):
        means, write = oracle_means(bundle, geo)
        np.testing.assert_array_equal(means['V_GT_SENSORGRID_PREDONLY250'][~write], bundle['mean'][~write])
        np.testing.assert_array_equal(means['V_GT_PREDONLY250'][~write], bundle['mean'][~write])
        np.testing.assert_array_equal(geo['mapper_gt'][500:703, 500:763], geo['sensor_raw'])
        arms, support = all_arms(pool, np.array(START), np.atleast_2d(START), bundle, geo)
        assert array_sha(support) == array_sha(write)
        for arm, value in arms.items():
            assert len(value['rows']) == len(pool)
            for native, row in zip(rows, value['rows']):
                assert native['candidate_id'] == row['candidate_id']
                assert native['distance_cells'] == row['distance_cells']
        # Nonlocal-occlusion fixture: one changed obstacle may affect many output cells.
        p = np.full((128, 128), .1, np.float32)
        q = p.copy(); q[80, 64] = 1
        a = smu.get_vis_mask(p, (64, 64), laser_range=30, num_laser=250,
                            raycast_mode='probabilistic', hit_prob_threshold=.8)[4]
        b = smu.get_vis_mask(q, (64, 64), laser_range=30, num_laser=250,
                            raycast_mode='probabilistic', hit_prob_threshold=.8)[4]
        changed = a != b
        assert int(changed.sum()) > 1
        changed[80, 64] = False
        assert changed.any(), 'LOCAL_MASK_CANNOT_CERTIFY_OCCLUSION'
        evidence['oracles'] = {'common_write_cells': int(write.sum()), 'support_sha256': array_sha(write),
            'native_sensor_p2_complete_candidate_counts': {arm: len(v['rows']) for arm,v in arms.items()},
            'known_outside_padding_unchanged': True, 'same_variance_distance_pool_renderer': True,
            'single_cell_occlusion_nonlocal_changed_cells': int(changed.sum()),
            'P2_role': 'STRUCTURAL_P2_SENSITIVITY', 'primary_role': 'PRIMARY_PREDICTOR_EFFECT_SENSORGRID'}
        mark('P10_SENSORGRID_AND_P2_ORACLES')
    with guard.phase('P11_budget_boundary_fixture'):
        action = [{'k': k, 'distance_m': .3, 'gain_m2': .02, 'compute_s': 1.}
                  for k in [999, 1000, 1001, 1002]]
        split = {}
        for row in action:
            part = split.setdefault(stratum(row['k']), {'steps': 0, 'distance_m': 0., 'gain_m2': 0., 'compute_s': 0.})
            part['steps'] += 1
            for name in ['distance_m', 'gain_m2', 'compute_s']:
                part[name] += row[name]
        assert split['NATIVE_BUDGET_PREFIX']['steps'] == split['EXTENDED_BUDGET_TAIL']['steps'] == 2
        assert sum(v['steps'] for v in split.values()) == 4
        assert math.isclose(sum(v['distance_m'] for v in split.values()), 1.2)
        evidence['boundary'] = {'action_crosses_boundary': True, 'score_origin_k': 999,
             'parts': split, 'no_double_count': True, 'dense_counts_are_not_control_budget': True}
        mark('P11_PREFIX_TAIL')
    # Real New Room model/control smoke is two steps, no continuation to science.
    smoke_recorder = Recorder(out/'real_model_smoke', bound, 'TECHNICAL_SMOKE')
    predictor = Predictor(guard)
    smoke = NativeRun(geo['mapper_gt'], predictor.predict, guard, smoke_recorder)
    terminal = smoke.execute(2)
    assert terminal['status'] == 'TECHNICAL_SMOKE_BUDGET', terminal
    with guard.phase('P13_terminal_audit'):
        replay = write_audit(smoke_recorder.root, geometry, out/'smoke_audit')
        assert replay['status'] == 'PASS'
        sid = terminal['last_scan_step']
        final = json.loads((smoke_recorder.root/'terminal.json').read_text())
        _, arrays = restore_state(smoke_recorder.root, final['final_snapshot_id'])
        assert terminal['last_pose'] == arrays['pose'].tolist() == arrays['pose_list'][-1].tolist()
        assert terminal['last_scan_step'] == terminal['control_steps'] == 2
        assert array_sha(arrays['obs_map']) == smoke.trace['observations'][-1]
        evidence['smoke_terminal'] = terminal
        evidence['technical_control_steps_total'] = 14
        mark('P13_FINAL_STATE_TERMINAL')
    with (smoke_recorder.root/'snapshots.csv').open() as stream:
        snapshots = list(csv.DictReader(stream))
    dense = [r['snapshot_id'] for r in snapshots if r['event'] == 'DENSE_DISTANCE']
    selected = dense[:1] or [final['final_snapshot_id']]
    shadow_results = shadow_execute(smoke_recorder.root, geometry, out/'smoke_shadow', guard, selected)
    binding_sha = hashlib.sha256(json_bytes(bound)).hexdigest()
    # Resume validates all immutable artifacts and does not invoke models again.
    class NoModelAllowed:
        def predict(self, *args):
            raise AssertionError('RESUME_RECOMPUTED_A_COMMITTED_SHADOW_UNIT')
    resumed = unit(smoke_recorder.root, selected[0], out/'smoke_shadow', geo, guard,
                   NoModelAllowed(), binding_sha)
    assert resumed['job_key_sha256'] == shadow_results[0]['job_key_sha256']
    render(smoke_recorder.root, out/'smoke_audit', out/'smoke_shadow', out/'smoke_report')
    with guard.phase('P12_resource_projection'):
        with zipfile.ZipFile(str(out/'model_probe_outputs.npz')) as archive:
            measured = {item.filename: item.compress_size for item in archive.infolist()}
        compressed_per_model_epoch = sum(measured.values())
        # Max 10001 distinct observed states. Predictions at shared states can be
        # reused between native and shadow; do not double count that overlap.
        states = CONFIG['extended_cap']+1
        projected_model_bytes = states*compressed_per_model_epoch
        minimum_mandatory_five_maps = states*sum(measured[key+'.npy'] for key in ['G1', 'G2', 'G3', 'mean', 'variance'])
        model_s = sum(row['load_s']+row['inference_s'] for row in probe['models'])
        project = {'projection_kind': 'CAP_TIMES_MEASURED_COMPRESSED_BYTES_NOT_ACTUAL_SCIENTIFIC_USAGE',
            'maximum_unique_prediction_inputs': states, 'measured_prediction_bytes': compressed_per_model_epoch,
            'projected_prediction_bytes': projected_model_bytes,
            'five_mandatory_maps_only_projection_bytes': minimum_mandatory_five_maps,
            'output_quota_bytes': CONFIG['output_quota_bytes'],
            'native_shadow_input_overlap_dedup_accounted': True,
            'candidate_pool_measured': len(pool),
            'measured_complete_shadow_unit_bytes': shadow_results[0]['committed_bytes'],
            'maximum_dense_states': 6001,
            'native_model_compute_upper_s': CONFIG['extended_cap']*model_s,
            'shadow_model_compute_upper_s': 6001*model_s,
            'acquisition_wall_guard_s': CONFIG['acquisition_wall_guard_s'],
            'phase_timeout_seal_s': max(300, 3*max(row['load_s']+row['inference_s'] for row in probe['models'])),
            'memory_model_probe_rss_peak_bytes': json.loads((out/'resource_summary_probe.json').read_text())['rss_peak_bytes']
               if (out/'resource_summary_probe.json').exists() else None,
            'required_artifact_omissions': False, 'scientific_execution_allowed': False,
            'limitation': 'Initial-state compression is a measured projection basis, not a proven worst-case bound.'}
        if minimum_mandatory_five_maps > CONFIG['output_quota_bytes']:
            project.update(status='FAIL_STORAGE_PROJECTION',
               block_reason='Even five mandatory float32 maps per maximum distinct state exceed the 64GiB contract; no hidden thinning/crop/quantization/omission.')
            mark('P12_RAM_DISK_TIME_QUOTA', 'FAIL')
        else:
            # This implementation does not certify a candidate ceiling from one state.
            project.update(status='BLOCKED_CANDIDATE_MASK_CEILING_NOT_CERTIFIED')
            mark('P12_RAM_DISK_TIME_QUOTA', 'BLOCKED')
        evidence['resource_projection'] = project
        atomic_json(out/'resource_projection.json', project)
    with guard.phase('P14_adaptation_manifest'):
        evidence['adaptations'] = ['PINNED_NEW_ROOM_DIRECT_SDF_LOADER', 'CPU_FLOAT32_ONE_MODEL_RESIDENT',
            'HEADLESS_NO_INCIDENTAL_SOURCE_PLOTS', 'IMMUTABLE_RECORDER_AND_OFFLINE_SHADOW',
            'SOURCE_SENSOR_PLANNING_RAY_BUG_GOAL_CONTROLLER_TERMINALS_RETAINED']
        evidence['unverified_universal_equivalence'] = True
        mark('P14_SOURCE_ADAPTATION_FIXTURE_PARITY')
    atomic_json(out/'technical_checks.json', evidence)
    return results
