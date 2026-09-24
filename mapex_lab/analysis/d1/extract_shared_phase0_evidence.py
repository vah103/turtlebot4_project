#!/usr/bin/env python3
"""Neutral H067/H071 shared evidence; H072 known-only/zero clarification."""
from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np

BASE = 'd2b6eab35d5b97b6378996fcc938e58cdfdcda6d'
REFERENCE = '61b91640ca1d4fd608f716e152a91573ec072b5a'
COUNTS = dict(zip((f'mpx_{i:03}' for i in range(1, 11)), (35,37,41,36,34,36,35,35,36,40)))
R004 = 'mapex_lab/analysis/r004/'
TABLES = {
    'prediction': R004+'results/prediction_vs_final_observed_v1/prediction_vs_final_observed_decisions.csv',
    'free_error': R004+'results/free_error_diagnostic_v1/free_error_decomposition_decisions.csv',
    'topology': R004+'results/topology_traversability_v1/topology_diagnostic_decisions.csv',
    'alignment': R004+'results/topology_traversability_v1/source_pose_alignment_inventory.csv',
}
HELPERS = [R004+'evaluate_prediction_vs_final_observed.py', R004+'evaluate_topology_traversability.py']
ARTIFACTS = ('raw_map','canvas_map','g1_map','g2_map','g3_map','mean_map','variance_map')
RUNTIME_FIELDS = ('KnownReachableFree_count','KnownReachableFree_m2','RemainingFraction',
                  'R1_count','R2_count','R3_count','A1_m2','A2_m2','A3_m2',
                  'A_mean_m2','A_min_m2','A_max_m2','A_range_m2','R_union_count',
                  'U_p95','U_mean','U_disagreement')


def git(root, *args):
    return subprocess.check_output(['git','-C',str(root),*args]).decode().strip()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def object_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def read_csv(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def expected_keys(runs=None):
    return {(r, i) for r in (COUNTS if runs is None else runs) for i in range(1, COUNTS[r]+1)}


def index_rows(rows, expected):
    result = {}
    for row in rows:
        key = (row['run_id'], int(row['decision_id']))
        if key in result:
            raise ValueError(f'duplicate key {key}')
        result[key] = row
    if set(result) != set(expected):
        raise ValueError('missing/unexpected row membership')
    return result


def verified_file(root, commit, relative):
    blob = git(root, 'rev-parse', f'{commit}:{relative}')
    path = root / relative
    if git(root, 'hash-object', str(path)) != blob:
        raise ValueError(f'accepted blob mismatch: {relative}')
    return {'commit': commit, 'blob': blob, 'sha256': digest(path)}


def load_reference(root):
    root = Path(root).resolve()
    if git(root, 'rev-parse', 'HEAD') != REFERENCE:
        raise ValueError('accepted reference HEAD mismatch')
    files = HELPERS + list(TABLES.values()) + [
        R004+f'results/topology_traversability_v1/runs/{rid}/completion_manifest.json'
        for rid in COUNTS]
    identities = {p: verified_file(root, REFERENCE, p) for p in files}
    # Import only after exact blob verification; prevent fallback to another checkout.
    modules = []
    for relative, name in zip(HELPERS, ('evaluate_prediction_vs_final_observed','evaluate_topology_traversability')):
        spec = importlib.util.spec_from_file_location(name, root/relative)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        modules.append(module)
    base, topo = modules
    if Path(topo.base.__file__).resolve() != (root/HELPERS[0]).resolve():
        raise ValueError('topology imported an unpinned base helper')
    tables = {name: index_rows(read_csv(root/p), expected_keys()) for name,p in TABLES.items()}
    return topo, tables, identities


def local_path(run, relative):
    path = (run/relative).resolve()
    if run.resolve() not in path.parents:
        raise ValueError('artifact escapes run directory')
    return path


def run_inputs(run, reference):
    rows = read_csv(run/'decisions.csv')
    index_rows([dict(r, run_id=run.name) for r in rows], expected_keys([run.name]))
    accepted = json.loads((reference/R004/f'results/topology_traversability_v1/runs/{run.name}/completion_manifest.json').read_text())
    names = set(accepted['input_fingerprints']) | {'decisions.csv','trajectory.csv','early_stopping_analysis.csv'}
    names.update(r[k] for r in rows for k in ARTIFACTS)
    hashes = {}
    for name in sorted(names):
        path = local_path(run, name)
        hashes[name] = digest(path) if path.is_file() else 'MISSING'
    for name, value in accepted['input_fingerprints'].items():
        if hashes[name] != value:
            raise ValueError(f'{run.name}: accepted R004 input mismatch: {name}')
    return rows, hashes


def scalar(z, key):
    return np.asarray(z[key]).reshape(()).item()


def load_runtime(run, row):
    did = int(row['decision_id'])
    with np.load(local_path(run,row['raw_map']), allow_pickle=False) as z:
        raw = np.asarray(z['data'])
        meta = {k: scalar(z,k) for k in ('resolution','width','height','origin_x','origin_y','origin_yaw','frame_id','source_stamp_s')}
    if raw.ndim != 2 or raw.shape != (meta['height'],meta['width']) or not np.all(np.isfinite(raw)):
        raise ValueError('invalid runtime shape/values')
    if not all(math.isfinite(meta[k]) for k in ('resolution','origin_x','origin_y','origin_yaw','source_stamp_s')) or meta['resolution'] <= 0 or meta['origin_yaw'] != 0 or meta['frame_id'] != 'map':
        raise ValueError('invalid runtime geometry/frame')
    expected_raw = f'decision_{did:06}_raw.npz'
    if Path(row['raw_map']).name != expected_raw or Path(row['canvas_map']).name != f'decision_{did:06}_canvas.npz':
        raise ValueError('wrong decision map identity')
    predictions, crops = [], {}
    for key, member in zip(ARTIFACTS[2:], ('G1','G2','G3','mean','variance')):
        suffix = member.lower()
        if Path(row[key]).name != f'decision_{did:06}_{suffix}.npz':
            raise ValueError('wrong decision prediction identity')
        with np.load(local_path(run,row[key]), allow_pickle=False) as z:
            h,w,t,l = (scalar(z,k) for k in ('source_height','source_width','pad_top','pad_left'))
            if any(int(v) != v for v in (h,w,t,l)):
                raise ValueError('nonintegral crop metadata')
            h,w,t,l = map(int,(h,w,t,l))
            a = np.asarray(z['data'])
            if a.ndim != 2 or (h,w) != raw.shape or min(t,l)<0 or t+h>a.shape[0] or l+w>a.shape[1]:
                raise ValueError('invalid prediction crop/runtime parity')
            if scalar(z,'member') != member or scalar(z,'source_map_stamp_s') != meta['source_stamp_s']:
                raise ValueError('stale/wrong member timestamp')
            for field in ('resolution','origin_x','origin_y'):
                if scalar(z,field) != meta[field]:
                    raise ValueError('prediction geometry mismatch')
            cropped = a[t:t+h,l:l+w].copy()
            if not np.all(np.isfinite(cropped)) or np.any(cropped < 0) or (member != 'variance' and np.any(cropped > 1)):
                raise ValueError('invalid prediction/variance values')
            predictions.append(cropped)
            crops[key] = {'shape': list(a.shape), 'source_height':h,'source_width':w,'pad_top':t,'pad_left':l}
    return raw, predictions[:3], predictions[4], meta, crops


def remaining_fraction(known_count, counts, resolution):
    if known_count is None or counts is None:
        return math.nan
    if known_count == 0 and all(n == 0 for n in counts):
        return 0.0
    area = float(np.mean(counts)) * resolution**2
    return area / (known_count*resolution**2 + area)


def uncertainty(regions, members, variance):
    union = np.logical_or.reduce(regions)
    if not union.any():
        return union, dict(U_p95=0.0, U_mean=0.0, U_disagreement=0.0, u_empty_valid=True)
    votes = np.sum(np.asarray(members) < 0.5, axis=0)
    return union, dict(U_p95=float(np.percentile(variance[union],95)),
                      U_mean=float(np.mean(variance[union])),
                      U_disagreement=float(np.mean(np.minimum(votes[union],3-votes[union])/3)),
                      u_empty_valid=False)


def runtime_fields(raw, members, variance, resolution, source, topo):
    result = {name: math.nan for name in RUNTIME_FIELDS}
    result.update(d1_source_available=False, d1_runtime_region_evaluable=False,
                  u_r_union_evaluable=False, u_empty_valid=False, d1_reason='')
    if source is None:
        result['d1_reason'] = 'source_unavailable'
        return result
    r,c = source
    if not (0 <= r < raw.shape[0] and 0 <= c < raw.shape[1]):
        result['d1_reason'] = 'source_outside_runtime_grid'
        return result
    stencil = topo.collision_stencil(radius=0.189, resolution=resolution)
    domain = np.ones(raw.shape, bool)
    known_safe = topo.cspace(raw == 0, domain, stencil)
    if not known_safe[source]:
        result['d1_reason'] = 'source_not_known_footprint_safe'
        return result
    known_count = int(topo.reachable(known_safe,source).sum())
    regions = []
    for pred in members:
        free = (raw == 0) | ((raw < 0) & (pred < 0.5))
        safe = topo.cspace(free, domain, stencil)
        regions.append(topo.reachable(safe,source) & (raw < 0) & (pred < 0.5))
    counts = [int(x.sum()) for x in regions]
    areas = np.asarray(counts)*resolution**2
    union, u = uncertainty(regions,members,variance)
    result.update(u)
    for j, (n,a) in enumerate(zip(counts,areas),1):
        result[f'R{j}_count'] = n
        result[f'A{j}_m2'] = float(a)
    result.update(KnownReachableFree_count=known_count, KnownReachableFree_m2=known_count*resolution**2,
                  A_mean_m2=float(areas.mean()), A_min_m2=float(areas.min()), A_max_m2=float(areas.max()),
                  A_range_m2=float(np.ptp(areas)), R_union_count=int(union.sum()),
                  RemainingFraction=remaining_fraction(known_count,counts,resolution),
                  d1_source_available=True,d1_runtime_region_evaluable=True,u_r_union_evaluable=True)
    return result


def number(value):
    return float(value) if value not in ('',None) else math.nan


def extract_run(run, decisions, hashes, topo, tables, identity):
    trajectory = topo.read_trajectory(run/'trajectory.csv')
    legacy = index_rows([dict(r,run_id=run.name) for r in read_csv(run/'early_stopping_analysis.csv')], expected_keys([run.name]))
    rows = []
    for index, decision in enumerate(decisions,1):
        key = (run.name,int(decision['decision_id']))
        pred, err, topology, alignment = (tables[k][key] for k in ('prediction','free_error','topology','alignment'))
        row = dict(pred)
        # Prefix full imported records to retain every accepted value and avoid collisions.
        for prefix, record in (('r004_free_error',err),('r004_topology',topology),('r004_alignment',alignment)):
            row.update({prefix+'_'+k:v for k,v in record.items() if k not in ('run_id','decision_id')})
        for field in ('false_free_fraction','supported_future_free_miss_fraction','supported_future_free_count','unsupported_future_free_count','unsupported_future_free_fraction'):
            row[field] = err[field]
        for field in ('source_available','source_reason','reachable_future_free_count','prediction_reachable_future_free_count','reachable_future_free_retention','lost_reachable_future_free_fraction','reachable_pair_connectivity_retention','largest_predicted_piece_fraction','prediction_component_count','fragmented','navigable_missed_free_fraction','missed_free_clearance_cells_mean','missed_free_clearance_meters_mean'):
            row[field] = topology[field]
        row.update(run_directory=str(run.resolve()), decision_csv_row=json.dumps(decision,sort_keys=True),
                   decision_time_s=decision['time_s'], decision_index=index,
                   row_integrity_ok=True, non_evaluable_reasons='',
                   prediction_support_coverage=pred['support_coverage'],
                   broad_free_error=1-number(pred['free_iou']),
                   broad_error_evaluable=math.isfinite(number(pred['free_iou'])),
                   primary_topology_risk_evaluable=math.isfinite(number(topology['lost_reachable_future_free_fraction'])),
                   implementation_sha=identity['implementation_sha'], contract_identity=identity['contract_identity'],
                   run_input_fingerprint=object_digest(hashes),
                   accepted_reference_identity=object_digest(identity['reference_files']))
        for name in ('unknown_variance_p95','unknown_mean_variance'):
            row[name] = legacy[key][name]
        for name in ARTIFACTS:
            row[name] = decision[name]
            row[name+'_sha256'] = hashes[decision[name]]
        row.update(runtime_resolution=math.nan,runtime_origin_x=math.nan,runtime_origin_y=math.nan,
                   runtime_shape='',crop_metadata='',d1_source_row=math.nan,d1_source_col=math.nan,
                   d1_source_x=math.nan,d1_source_y=math.nan,d1_source_mode='',d1_source_lower_time_s=math.nan,
                   d1_source_upper_time_s=math.nan)
        runtime = {name:math.nan for name in RUNTIME_FIELDS}
        runtime.update(d1_source_available=False,d1_runtime_region_evaluable=False,u_r_union_evaluable=False,
                       u_empty_valid=False,d1_reason='runtime_integrity_failure')
        try:
            raw,members,variance,meta,crops = load_runtime(run,decision)
            pose = topo.align_source(float(decision['time_s']),trajectory)
            row.update(runtime_resolution=meta['resolution'],runtime_origin_x=meta['origin_x'],
                       runtime_origin_y=meta['origin_y'],runtime_shape=json.dumps(list(raw.shape)),
                       crop_metadata=json.dumps(crops,sort_keys=True),d1_source_x=pose['x'],d1_source_y=pose['y'],
                       d1_source_mode=pose['mode'],d1_source_lower_time_s=pose['lower_time_s'],
                       d1_source_upper_time_s=pose['upper_time_s'])
            source = topo.world_to_cell(pose['x'],pose['y'],meta['origin_x'],meta['origin_y'],meta['resolution']) if pose['available'] else None
            if source is not None:
                row.update(d1_source_row=source[0],d1_source_col=source[1])
            runtime = runtime_fields(raw,members,variance,meta['resolution'],source,topo)
            if not pose['available']:
                runtime['d1_reason'] = pose['mode']
        except (ValueError,KeyError,OSError) as exc:
            row['row_integrity_ok'] = False
            runtime['d1_reason'] = str(exc)
        row.update(runtime)
        row['non_evaluable_reasons'] = runtime['d1_reason']
        rows.append(row)
    index_rows(rows,expected_keys([run.name]))
    return rows
