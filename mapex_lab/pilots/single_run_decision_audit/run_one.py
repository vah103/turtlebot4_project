"""Scientific entry requires an exact reviewed technical PASS and USER grant."""
import argparse
import hashlib
import json
import time
from pathlib import Path
from .contract import CONFIG, atomic_json, json_bytes, binding_sha
from .preflight import bindings
from .prediction import Predictor, configure_torch
from .native_adapter import NativeRun
from .recorder import Recorder
from .resources import Guard

def validate_authority(authorization, bound, report):
    auth = json.loads(Path(authorization).read_text())
    assert report['status'] == 'TECHNICAL_PREFLIGHT_PASS_PENDING_INDEPENDENT_REVIEW'
    assert all(value == 'PASS' for value in report['gates'].values())
    assert auth['user_authorized_scientific_run'] is True
    assert auth['single_trajectory'] is True
    assert auth['technical_review_verdict'] == 'ACCEPT'
    assert len(auth['technical_review_commit']) == 40
    assert auth['bindings_sha256'] == report['bindings_sha256'] == binding_sha(bound)
    assert auth['sealed_phase_timeout_s'] == report['sealed_phase_timeout_s']
    return auth

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--repo', required=True)
    p.add_argument('--geometry', required=True)
    p.add_argument('--preflight', required=True)
    p.add_argument('--authorization', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    configure_torch()
    geo, bound = bindings(a.repo, a.geometry)
    report = json.loads((Path(a.preflight)/'preflight.json').read_text())
    auth = validate_authority(a.authorization, bound, report)
    # No existing run directory, retry, overwrite or acquisition-resume path.
    recorder = Recorder(a.output, bound, 'SCIENTIFIC_TRAJECTORY')
    atomic_json(recorder.root/'authorization_used.json', auth)
    with Guard(recorder.root/'runtime_resources',
               phase_limit=auth['sealed_phase_timeout_s'],
               wall_limit=CONFIG['acquisition_wall_guard_s'], quota_roots=[recorder.root]) as guard:
        predictor = Predictor(guard)
        native = NativeRun(geo['mapper_gt'], predictor.predict, guard, recorder)
        native.execute(CONFIG['extended_cap'], 'SCIENTIFIC_TRAJECTORY')

if __name__ == '__main__':
    main()
