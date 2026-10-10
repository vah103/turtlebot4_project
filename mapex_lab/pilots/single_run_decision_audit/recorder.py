"""Synchronous immutable objects and append-only event ledgers."""
import csv
import hashlib
import io
import json
import os
import random
import time
from pathlib import Path
import numpy as np
from .contract import CONFIG, array_sha, atomic_json, file_sha, json_bytes, stratum

def rng_state():
    import torch
    state = np.random.get_state()
    return {'python': random.getstate(), 'numpy': [state[0], state[1].tolist(),
            int(state[2]), int(state[3]), float(state[4])],
            'torch_cpu': torch.get_rng_state().tolist()}

class ObjectStore:
    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.bytes_written = sum(p.stat().st_size for p in self.root.glob('*.npz'))

    def put(self, value):
        a = np.ascontiguousarray(value).copy()  # Freeze before the next native mutation.
        digest = array_sha(a)
        path = self.root/(digest+'.npz')
        if not path.exists():
            tmp = self.root/(digest+'.tmp.npz')
            np.savez_compressed(str(tmp), data=a)
            if self.bytes_written+tmp.stat().st_size > CONFIG['output_quota_bytes']:
                tmp.unlink()
                raise RuntimeError('OUTPUT_QUOTA_EXCEEDED')
            with tmp.open('rb') as stream:
                os.fsync(stream.fileno())
            size = tmp.stat().st_size
            os.replace(str(tmp), str(path))
            self.bytes_written += size
        return {'sha256': digest, 'file': 'objects/'+path.name,
                'file_sha256': file_sha(path), 'dtype': a.dtype.str,
                'shape': list(a.shape), 'raw_bytes': a.nbytes,
                'compressed_bytes': path.stat().st_size}

    def get(self, ref):
        path = self.root/Path(ref['file']).name
        assert file_sha(path) == ref['file_sha256'], 'OBJECT_FILE_HASH_GAP'
        with np.load(str(path), allow_pickle=False) as obj:
            a = obj['data']
        assert array_sha(a) == ref['sha256'], 'OBJECT_ARRAY_HASH_GAP'
        assert list(a.shape) == ref['shape'] and a.dtype.str == ref['dtype']
        return a

class Recorder:
    def __init__(self, root, bindings, purpose):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=False)
        self.objects = ObjectStore(self.root/'objects')
        self.bindings = bindings
        self.purpose = purpose
        self.started = time.monotonic()
        self.next_target_index = 1
        self.snapshot_count = 0
        self.last_state_digest = None
        self.last_snapshot_id = None
        self.chain = '0'*64
        self.headers = {}
        self.io_s = 0.0
        atomic_json(self.root/'manifest.json', {'purpose': purpose, 'bindings': bindings,
                    'snapshot_interval_m': 0.5, 'scientific_execution_authorized': purpose == 'SCIENTIFIC_TRAJECTORY',
                    'distance_contract': 'SUM_EUCLIDEAN_SOURCE_CUR_POSE_UPDATES_M_INCLUDING_COLLISION_ENDPOINT',
                    'source_pose_list_distance_contract': 'SUM_SUCCESSFULLY_APPENDED_POSE_DISTANCES_M',
                    'simulation_clock': 'NA_SIM_TIME_UNDEFINED', 'record_version': 1})

    def row(self, name, values):
        begin = time.monotonic()
        path = self.root/(name+'.csv')
        data = dict(values)
        data.setdefault('wall_elapsed_s', time.monotonic()-self.started)
        if name not in self.headers:
            self.headers[name] = list(data)
            assert not path.exists(), 'LEDGER_OVERWRITE'
            with path.open('w', newline='') as stream:
                csv.DictWriter(stream, fieldnames=self.headers[name]).writeheader()
        assert set(data) == set(self.headers[name]), 'LEDGER_SCHEMA_DRIFT_'+name
        data = {key: (json.dumps(v, sort_keys=True) if isinstance(v, (dict, list, tuple)) else v)
                for key, v in data.items()}
        with path.open('a', newline='') as stream:
            csv.DictWriter(stream, fieldnames=self.headers[name]).writerow(data)
            stream.flush()
            os.fsync(stream.fileno())
        self.io_s += time.monotonic()-begin

    def scan(self, k, pose, obs_dict, observation_hash):
        begin = time.monotonic()
        io_before = self.io_s
        refs = {key: self.objects.put(value) for key, value in obs_dict.items()}
        payload = {'k': k, 'pose': np.asarray(pose).tolist(), 'objects': refs,
                   'post_observation_sha256': observation_hash, 'previous_chain': self.chain}
        self.chain = hashlib.sha256(json_bytes(payload)).hexdigest()
        payload['chain_sha256'] = self.chain
        self.row('raw_scans', {'control_step': k, 'scan': payload})
        self.io_s += max(0, time.monotonic()-begin-(self.io_s-io_before))
        return payload

    def snapshot(self, event, state, targets=None):
        begin = time.monotonic()
        io_before = self.io_s
        # Arrays include observation, current path, pose history, candidate cache.
        refs = {key: self.objects.put(value) for key, value in state['arrays'].items()}
        meta = dict(state['meta'], arrays=refs, rng=rng_state(), scan_chain=self.chain,
                    bindings_sha256=hashlib.sha256(json_bytes(self.bindings)).hexdigest())
        digest = hashlib.sha256(json_bytes(meta)).hexdigest()
        # Repeated event aliases may share exactly one state; targets are separate rows.
        if digest == self.last_state_digest:
            sid = self.last_snapshot_id
        else:
            self.snapshot_count += 1
            sid = 'snapshot_{:06d}'.format(self.snapshot_count)
            atomic_json(self.root/'snapshots'/ (sid+'.json'), dict(meta, snapshot_id=sid,
                        state_sha256=digest, branch_state='COMPLETE_HASH_BOUND_STATE_REPLAY_REQUIRED'))
            self.last_state_digest, self.last_snapshot_id = digest, sid
        k = state['meta']['control_step']
        self.row('snapshots', {'snapshot_id': sid, 'event': event, 'control_step': k,
                 'stratum': stratum(k), 'distance_m': state['meta']['distance_m'],
                 'target_distances_m': targets or [],
                 'overshoots_m': [state['meta']['distance_m']-t for t in targets or []],
                 'state_sha256': digest, 'native_score_epoch_id': state['meta']['native_score_epoch_id'],
                 'dispatch_id': state['meta']['dispatch_id'], 'scan_chain_sha256': self.chain})
        self.io_s += max(0, time.monotonic()-begin-(self.io_s-io_before))
        return sid

    def post_step(self, state):
        distance = state['meta']['distance_m']
        targets = []
        while distance >= 0.5*self.next_target_index:
            targets.append(0.5*self.next_target_index)
            self.next_target_index += 1
        sid = self.snapshot('DENSE_DISTANCE', state, targets) if targets else None
        if state['meta']['control_step'] == 1000:
            boundary = self.snapshot('NATIVE_BUDGET_BOUNDARY', state)
            return sid or boundary
        return sid

    def finish(self, state, terminal):
        sid = self.snapshot('TERMINAL', state)
        atomic_json(self.root/'terminal.json', dict(terminal, final_snapshot_id=sid,
                    recorder_io_s=self.io_s, purpose=self.purpose,
                    last_scan_chain=self.chain, committed_objects_bytes=self.objects.bytes_written))

def restore_state(root, snapshot_id):
    root = Path(root)
    meta = json.loads((root/'snapshots'/(snapshot_id+'.json')).read_text())
    verify = {key: value for key, value in meta.items() if key not in
              ['state_sha256', 'snapshot_id', 'branch_state']}
    assert hashlib.sha256(json_bytes(verify)).hexdigest() == meta['state_sha256']
    objects = ObjectStore(root/'objects')
    arrays = {key: objects.get(ref) for key, ref in meta['arrays'].items()}
    return meta, arrays
