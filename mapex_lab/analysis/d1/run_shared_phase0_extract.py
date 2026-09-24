#!/usr/bin/env python3
"""H071 per-run transactional extraction with revalidated resume."""
from __future__ import annotations

import argparse
from collections import Counter
import datetime
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import tempfile
import traceback

import extract_shared_phase0_evidence as e

HUB = '902285a'  # Accepted H072 resume snapshot; files below are verified against it.
DOCS = ('D1_SHARED_PHASE0_EVIDENCE_CONTRACT_V1.md','D1_GATE_U_METHOD_V2.md',
        'D1_SHARED_EXTRACTOR_FROZEN_DETAIL_CLARIFICATION_V1.md',
        'D1_SHARED_EXTRACTOR_FROZEN_DETAIL_CLARIFICATION_V1_REVIEW.md',
        'H071_D1_SHARED_EXTRACTOR_EXECUTION.md')
CODE = tuple('mapex_lab/analysis/d1/'+p for p in (
    'extract_shared_phase0_evidence.py','run_shared_phase0_extract.py','test_extract_shared_phase0_evidence.py'))


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=True)+'\n')
    os.replace(tmp,path)


def validate_unit(path, identity, inputs, rid):
    manifest = json.loads((path/'completion_manifest.json').read_text())
    if manifest['identity'] != identity or manifest['input_fingerprints'] != inputs:
        raise ValueError('changed identity/input: reviewed recovery required')
    if e.digest(path/'evidence.csv') != manifest['output_sha256']:
        raise ValueError('output hash mismatch')
    rows = e.read_csv(path/'evidence.csv')
    e.index_rows(rows,e.expected_keys([rid]))
    if manifest['row_count'] != len(rows):
        raise ValueError('manifest row count mismatch')
    return rows


def identity(args, repo, reference_files):
    hub_files = {name:e.verified_file(args.hub,HUB,'project_management/mapex/'+name) for name in DOCS}
    code_hashes = {p:e.digest(repo/p) for p in CODE}
    for p in CODE:
        e.verified_file(repo,'HEAD',p)
    # Artifact-only delivery commits do not change executable implementation identity.
    implementation = e.git(repo,'log','-1','--format=%H','--',*CODE)
    return dict(schema='d1_shared_phase0_v1_h072',data_root=str(args.data_root.resolve()),
                output_root=str(args.output.resolve()),canonical_base=e.BASE,
                implementation_sha=implementation,implementation_files=code_hashes,
                reference_sha=e.REFERENCE,reference_files=reference_files,
                authority_files=hub_files,contract_identity=e.object_digest(hub_files))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-root',type=Path,required=True)
    ap.add_argument('--reference',type=Path,required=True)
    ap.add_argument('--hub',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args = ap.parse_args()
    # An earlier completion is not proof about the current inputs. Preserve it
    # as history before revalidation, and publish a fresh completion only last.
    old_completion = args.output/'completion_summary.json'
    if old_completion.exists():
        os.replace(old_completion,args.output/'previous_completion.json')
    repo = Path(__file__).resolve().parents[3]
    topo,tables,refs = e.load_reference(args.reference)
    if e.git(repo,'merge-base',e.BASE,'HEAD') != e.BASE:
        raise ValueError('implementation not based on frozen canonical base')
    ident = identity(args,repo,refs)
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    state_path = output/'batch_state.json'
    if state_path.exists() and json.loads(state_path.read_text())['identity'] != ident:
        raise ValueError('batch identity conflict; reviewed recovery required')
    all_inputs, decisions = {}, {}
    for rid in e.COUNTS:
        run = args.data_root/'mapex_lab/experiments/mapex'/rid
        decisions[rid],all_inputs[rid] = e.run_inputs(run,args.reference)
    preflight = dict(identity=ident,hostname=platform.node(),python=sys.version,
                     data_checkout_sha=e.git(args.data_root,'rev-parse','HEAD'),
                     input_fingerprints=all_inputs,expected_decisions=365)
    previous = output/'preflight.json'
    if previous.exists() and json.loads(previous.read_text())['input_fingerprints'] != all_inputs:
        raise ValueError('changed batch inputs; reviewed recovery required')
    atomic_json(previous,preflight)
    env = dict(os.environ,D1_REFERENCE=str(args.reference.resolve()))
    test = subprocess.run([sys.executable,'-m','unittest','-v','test_extract_shared_phase0_evidence'],
                          cwd=Path(__file__).parent,env=env,text=True,capture_output=True)
    (output/'tests.log').write_text(test.stdout+test.stderr)
    atomic_json(output/'test_summary.json',dict(exit_code=test.returncode,implementation_sha=ident['implementation_sha']))
    if test.returncode:
        raise ValueError('focused tests failed; see tests.log')
    print('PREFLIGHT_AND_TESTS_PASS',flush=True)
    state = dict(identity=ident,completed=[],events=[],status='RUNNING')
    if state_path.exists():
        state['events'] = json.loads(state_path.read_text()).get('events',[])
    run_root = output/'runs'
    run_root.mkdir(exist_ok=True)
    all_rows = []
    for rid in e.COUNTS:
        state.update(active=rid,status='RUNNING')
        atomic_json(state_path,state)
        unit = run_root/rid
        rows = None
        if unit.exists():
            try:
                rows = validate_unit(unit,ident,all_inputs[rid],rid)
            except (ValueError,OSError,KeyError,json.JSONDecodeError) as exc:
                if 'changed identity/input' in str(exc):
                    raise
                quarantine = unit.with_name(rid+'.quarantine.'+datetime.datetime.now().strftime('%Y%m%d%H%M%S%f'))
                unit.rename(quarantine)
                state['events'].append(dict(run_id=rid,action='quarantine_recompute',reason=str(exc),path=str(quarantine)))
        if rows is None:
            run = args.data_root/'mapex_lab/experiments/mapex'/rid
            temporary = Path(tempfile.mkdtemp(prefix=rid+'.incomplete.',dir=run_root))
            rows = e.extract_run(run,decisions[rid],all_inputs[rid],topo,tables,ident)
            # Detect mutation during processing before promotion.
            if e.run_inputs(run,args.reference)[1] != all_inputs[rid]:
                raise ValueError('input changed during extraction')
            topo.write_csv(temporary/'evidence.csv',rows)
            manifest = dict(identity=ident,input_fingerprints=all_inputs[rid],row_count=len(rows),
                            output_sha256=e.digest(temporary/'evidence.csv'))
            atomic_json(temporary/'completion_manifest.json',manifest)
            rows = validate_unit(temporary,ident,all_inputs[rid],rid)
            temporary.rename(unit)
            action = 'EXTRACTED'
        else:
            action = 'VALIDATED_SKIP'
        all_rows.extend(rows)
        state['completed'].append(rid)
        state['events'].append(dict(run_id=rid,action=action))
        atomic_json(state_path,state)
        print(f'{rid} {action} rows={len(rows)}',flush=True)
    e.index_rows(all_rows,e.expected_keys())
    topo.write_csv(output/'shared_phase0_evidence.csv',all_rows)
    flags = ('row_integrity_ok','d1_source_available','d1_runtime_region_evaluable','u_r_union_evaluable',
             'broad_error_evaluable','primary_topology_risk_evaluable','u_empty_valid')
    summary = {'total_rows':len(all_rows),'per_run':{},'flags':{},'reasons':dict(Counter(r['d1_reason'] for r in all_rows))}
    for rid in e.COUNTS:
        selected = [r for r in all_rows if r['run_id']==rid]
        summary['per_run'][rid] = dict(total_rows=len(selected),**{f:sum(r[f]=='True' for r in selected) for f in flags})
    summary['flags'] = {f:sum(r[f]=='True' for r in all_rows) for f in flags}
    summary['support_distribution'] = {}
    for field in ('prediction_support_coverage','free_support_coverage'):
        values = e.np.array([float(r[field]) for r in all_rows])
        values = values[e.np.isfinite(values)]
        summary['support_distribution'][field] = dict(n=len(values),min=float(values.min()),max=float(values.max()),mean=float(values.mean()))
    atomic_json(output/'support_evaluability_summary.json',summary)
    atomic_json(output/'contract_execution_manifest.json',ident)
    state.update(status='COMPLETE_PENDING_REVIEW',active=None)
    atomic_json(state_path,state)
    paths = [p for p in output.rglob('*') if p.is_file() and
             not any('.incomplete.' in s or '.quarantine.' in s for s in p.parts) and
             p.name not in ('execution.log','artifact_manifest.json','completion_summary.json','previous_completion.json')]
    hashes = {str(p.relative_to(output)):e.digest(p) for p in sorted(paths)}
    atomic_json(output/'artifact_manifest.json',hashes)
    atomic_json(output/'completion_summary.json',dict(status='COMPLETE_PENDING_REVIEW',identity=ident,
        rows=len(all_rows),runs=list(e.COUNTS),artifact_manifest_sha256=e.digest(output/'artifact_manifest.json'),
        support_evaluability=summary,events=state['events']))
    print(json.dumps(summary,indent=2),flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
