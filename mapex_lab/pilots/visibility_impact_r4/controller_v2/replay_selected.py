"""Post-outcome integrity checks of fixed alternate A1 suffixes, cache only.

These cases were selected from the first five complete development episodes
to check a large negative primary effect and a positive sensitivity contrast.
They are not additional research samples or independent replication evidence.
No prediction, sample, branch artifact, outcome, or estimator is changed.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path
import numpy as np
import runner


class CacheOnly:
    def __init__(self, root, identity, core):
        self.root, self.identity, self.core = root, identity, core
        self.hits = 0

    def predict(self, observed):
        key = hashlib.sha256((self.identity+self.core.array_hash(observed)).encode()).hexdigest()
        path = self.root/'cache'/(key+'.npz')
        if not path.exists():
            raise RuntimeError('Replay diverged from recorded predictions; no inference is allowed: '+key)
        with np.load(path, allow_pickle=False) as z:
            assert str(z['identity']) == self.identity
            values = tuple(z[k].copy() for k in ('predictions', 'mean', 'variance'))
        known = observed != .5
        assert np.array_equal(values[1][known], observed[known])
        assert (values[2][known] == 0).all()
        self.hits += 1
        return values


def replay(cfg):
    root = Path(cfg['output'])
    assert not (root/'invalidation.json').exists()
    seal = json.loads((root/'seal.json').read_text())
    for path, expected in seal['source_hashes'].items():
        assert runner.digest(path) == expected
    core, _, fast = runner.dependencies(cfg)
    predictor = CacheOnly(root, seal['predictor_identity'], core)
    engine = runner.Engine(core, fast, predictor, cfg)
    targets = [(2, 'native_GT_U'), (3, 'sensor_P_uniform'), (3, 'sensor_GT_uniform')]
    layout = 'kth_50037764_PLAN1'
    folder = root/layout
    records = json.loads((folder/'reference.json').read_text())['decisions']
    selected = json.loads((folder/'sampling.json').read_text())['selected']
    with np.load(Path(cfg['assets'])/(layout+'.npz')) as z:
        asset = {k: z[k].copy() for k in z.files}
    asset['layout'] = layout
    results = []
    for state, arm in targets:
        assert state in selected
        record = records[state]
        with np.load(folder/f'state_{state:04d}.npz') as z:
            snapshot = {k: z[k].copy() for k in z.files}
        action = record['candidates'][record['chosen'][arm]]
        assert action['action_id'] == record['actions'][arm]
        expected = json.loads((folder/f"branch_{state:04d}_{action['action_id'][:12]}.json").read_text())
        world = runner.world_from(core, asset, cfg, snapshot)
        started = time.perf_counter()
        measured = runner.continue_episode(engine, world, asset, [p[:] for p in record['trace']], action)
        for key in ('C_bar', 'Q', 'distance_m', 'collisions', 'terminal', 'observed_hash'):
            a, b = measured[key], expected[key]
            assert a == b or (isinstance(a, float) and abs(a-b) < 1e-9), (state, arm, key, a, b)
        results.append(dict(state=state, arm=arm, action_id=action['action_id'],
                            status='EXACT_REPLAY', outcome=measured,
                            elapsed_s=time.perf_counter()-started))
    result = dict(status='PASS', scope='Post-outcome deterministic integrity checks; not extra research samples',
                  layout=layout, results=results, new_model_inferences=0, cache_hits=predictor.hits,
                  source_sha256=runner.digest(__file__), run_seal_sha256=runner.digest(root/'seal.json'))
    runner.save(root.parent/'controller_v2_alternate_suffix_replay.json', result)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--protocol', required=True)
    replay(json.loads(Path(parser.parse_args().protocol).read_text()))
