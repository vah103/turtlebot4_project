"""Bounded technical preflight. A probe never produces a READY token."""
import argparse
import importlib
import json
import os
import platform
import shutil
import subprocess
import sys
import traceback
from pathlib import Path
import numpy as np
from .contract import (CONFIG, ENV, SOURCE, SOURCE_PIN, LAMA_PIN, MODELS, WEIGHTS,
                       LIDAR, START, DESIGN_COMMIT, DESIGN_SHA256, REVIEW_COMMIT,
                       file_sha, array_sha, atomic_json, code_hashes, binding_sha)
from .geometry import load
from .prediction import Predictor, source_modules, configure_torch
from .resources import Guard, resources

sys.dont_write_bytecode = True

def git(path, *args):
    return subprocess.check_output(['git', '-C', str(path)]+list(args)).decode().strip()

def bindings(repo, geometry):
    assert platform.node() == 'com1', 'WRONG_DEVICE'
    assert Path(sys.prefix).resolve() == Path(ENV).resolve(), 'WRONG_ENV'
    assert platform.python_version() == '3.6.13'
    assert git(SOURCE, 'rev-parse', 'HEAD') == SOURCE_PIN
    assert git(SOURCE+'/lama', 'rev-parse', 'HEAD') == LAMA_PIN
    assert git(SOURCE, 'status', '--porcelain') == '', 'DIRTY_MAPEX'
    assert git(SOURCE+'/lama', 'status', '--porcelain') == '', 'DIRTY_LAMA'
    modules = {}
    for name in ['torch', 'numpy', 'pyastar2d', 'range_libc', 'albumentations', 'saicinpainting']:
        m = importlib.import_module(name)
        modules[name] = {'origin': str(Path(m.__file__).resolve()),
                         'version': getattr(m, '__version__', 'UNDECLARED')}
    assert modules['torch']['version'].split('+')[0] == '1.10.2'
    assert modules['numpy']['version'] == '1.19.5'
    assert modules['saicinpainting']['origin'].startswith(SOURCE+'/lama/')
    models = {}
    for ident, relative, expected_ckpt, expected_config in MODELS:
        root = Path(WEIGHTS)/relative
        ckpt, cfg = root/'models/best.ckpt', root/'config.yaml'
        actual_ckpt, actual_cfg = file_sha(ckpt), file_sha(cfg)
        assert (actual_ckpt, actual_cfg) == (expected_ckpt, expected_config), ident
        models[ident] = {'checkpoint_sha256': actual_ckpt, 'config_sha256': actual_cfg,
                        'root': str(root)}
    arrays, geom = load(geometry)
    source_files = {}
    for rel in ['scripts/explore.py', 'scripts/sim_utils.py', 'scripts/simple_mask_utils.py',
                'scripts/lama_pred_utils.py', 'configs/base.yaml',
                'lama/saicinpainting/training/data/datasets.py']:
        source_files[rel] = file_sha(Path(SOURCE)/rel)
    return arrays, {'device': platform.node(), 'python': platform.python_version(),
                    'env': sys.prefix, 'modules': modules, 'models': models,
                    'source': SOURCE, 'source_pin': SOURCE_PIN, 'lama_pin': LAMA_PIN,
                    'source_files': source_files, 'adapter_files': code_hashes(),
                    'design_commit': DESIGN_COMMIT, 'design_sha256': DESIGN_SHA256,
                    'review_commit': REVIEW_COMMIT, 'geometry': geom,
                    'config': CONFIG, 'resources_at_start': resources()}

def model_probe(geo, guard, out):
    sim, _, _ = source_modules()
    with guard.phase('initial_sensor'):
        mapper = sim.Mapper(geo['mapper_gt'], LIDAR, use_distance_transform_for_planning=True)
        mapper.observe_and_accumulate_given_pose(np.array(START))
        np.savez_compressed(str(out/'initial_observation.npz'), obs=mapper.obs_map)
    predictor = Predictor(guard, repeat_reference=True)
    bundle, evidence = predictor.predict(mapper.obs_map.copy())
    with guard.phase('model_probe_io'):
        np.savez_compressed(str(out/'model_probe_outputs.npz'), **bundle)
        evidence['outputs'] = {key: {'shape': list(a.shape), 'dtype': str(a.dtype),
                                    'sha256': array_sha(a)} for key, a in bundle.items()}
        atomic_json(out/'model_probe.json', evidence)
    return evidence

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--repo', required=True)
    p.add_argument('--geometry', required=True)
    p.add_argument('--output', required=True)
    p.add_argument('--stage', choices=['models', 'full'], default='full')
    p.add_argument('--reuse-model-probe', help='Reuse validated full-size model evidence with matching prediction/source/config bindings')
    a = p.parse_args()
    out = Path(a.output)
    out.mkdir(parents=True, exist_ok=False)
    report = {'status': 'TECHNICAL_PREFLIGHT_RUNNING', 'stage': a.stage,
              'scientific_execution_authorized': False, 'ready': False, 'gates': {}}
    atomic_json(out/'preflight.json', report)
    try:
        configure_torch()
        with Guard(out) as guard:
            with guard.phase('bindings_and_hashes'):
                geo, bound = bindings(a.repo, a.geometry)
                atomic_json(out/'bindings.json', bound)
                report['bindings_sha256'] = binding_sha(bound)
                report['gates']['P1_P2_P3_BINDINGS'] = 'PASS'
            if a.reuse_model_probe:
                previous = Path(a.reuse_model_probe)
                prior = json.loads((previous/'bindings.json').read_text())
                previous_report = json.loads((previous/'preflight.json').read_text())
                assert previous_report['gates']['P4_FULL_SIZE_MODELS'] == 'PASS'
                for key in ['device', 'python', 'env', 'modules', 'models', 'source', 'source_pin',
                            'lama_pin', 'source_files', 'config', 'geometry']:
                    assert prior[key] == bound[key], 'MODEL_PROBE_BINDING_DRIFT_'+key
                assert prior['adapter_files']['prediction.py'] == bound['adapter_files']['prediction.py']
                probe = json.loads((previous/'model_probe.json').read_text())
                with np.load(str(previous/'model_probe_outputs.npz'), allow_pickle=False) as saved:
                    assert set(saved.files) == set(probe['outputs']), 'MODEL_PROBE_ARRAY_SET_DRIFT'
                    for key in saved.files:
                        assert array_sha(saved[key]) == probe['outputs'][key]['sha256'], 'MODEL_PROBE_ARRAY_HASH_GAP_'+key
                shutil.copy2(str(previous/'model_probe_outputs.npz'), str(out/'model_probe_outputs.npz'))
                shutil.copy2(str(previous/'resource_summary.json'), str(out/'resource_summary_probe.json'))
                atomic_json(out/'model_probe.json', dict(probe, reused_from=str(previous),
                            output_file_sha256=file_sha(out/'model_probe_outputs.npz')))
            else:
                probe = model_probe(geo, guard, out)
            report['gates']['P4_FULL_SIZE_MODELS'] = 'PASS'
            report['sealed_phase_timeout_s'] = max(300, 3*max(
                row['load_s']+row['inference_s'] for row in probe['models']))
            if a.stage == 'full':
                from .checks import full_checks
                report['gates'].update(full_checks(a.repo, a.geometry, out, geo, guard, probe))
            else:
                report['gates']['P5_TO_P14'] = 'NOT_RUN_PROBE_ONLY'
        report['status'] = ('MODEL_PROBE_PASS_FULL_PREFLIGHT_PENDING' if a.stage == 'models'
                            else 'TECHNICAL_PREFLIGHT_PASS_PENDING_INDEPENDENT_REVIEW')
        if a.stage == 'full':
            report['status'] = ('TECHNICAL_PREFLIGHT_PASS_PENDING_INDEPENDENT_REVIEW'
                               if all(v == 'PASS' for v in report['gates'].values())
                               else 'IMPLEMENTATION_BLOCKED')
        # Independent technical review and scientific authorization remain separate.
        atomic_json(out/'preflight.json', report)
        print(json.dumps(report, sort_keys=True), flush=True)
        if report['status'] == 'IMPLEMENTATION_BLOCKED':
            raise SystemExit(2)
    except Exception as exc:
        report.update(status='IMPLEMENTATION_BLOCKED', error=repr(exc), traceback=traceback.format_exc())
        atomic_json(out/'preflight.json', report)
        print(json.dumps(report, sort_keys=True), flush=True)
        raise

if __name__ == '__main__':
    main()
