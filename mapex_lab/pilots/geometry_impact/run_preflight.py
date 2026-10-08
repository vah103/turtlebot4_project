"""Run the sealed P1/P2 development preflight on real author maps/LaMa.

P1 is a numerical/action-reachability test. P2 is a one-episode task search.
Neither phase estimates a population frequency or trains an online method.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import resource
import shutil
import sys
import time

os.environ.setdefault('OMP_NUM_THREADS','2')
os.environ.setdefault('MKL_NUM_THREADS','2')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
sys.path.insert(0, '/home/dell/mapex_verification2d_deps')
import numpy as np
import scipy
import shapely
from geometry_solver import GeometrySolver


def save(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    tmp.replace(path)


def event(**fields):
    print(json.dumps(fields,ensure_ascii=False,allow_nan=False),flush=True)


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):
            h.update(block)
    return h.hexdigest()


def load_adapter(cfg):
    folder=Path(cfg['repo'])/'mapex_lab/pilots/visibility_impact_r4/controller_v2'
    sys.path.insert(0,str(folder))
    spec=importlib.util.spec_from_file_location('r4_geometry_reference',folder/'runner.py')
    adapter=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=adapter
    spec.loader.exec_module(adapter)
    return adapter


def verify_sources(cfg):
    old_folder=Path(cfg['repo'])/'mapex_lab/pilots/visibility_impact_r4/results/run_20261008_controller_v2'
    original=json.loads((old_folder/'seal.json').read_text())
    root=Path(cfg['repo'])
    checks={}
    for original_path,expected in original['source_hashes'].items():
        if '/mapex_lab/' in original_path and not original_path.endswith('protocol_com1_controller_v2.json') and '/assets/' not in original_path:
            rel='mapex_lab/'+original_path.split('/mapex_lab/',1)[1]
            actual=digest(root/rel)
            checks[rel]={'expected':expected,'actual':actual,'match':expected==actual}
        elif '/assets/' in original_path:
            path=Path(cfg['assets'])/Path(original_path).name
            actual=digest(path)
            checks['assets/'+path.name]={'expected':expected,'actual':actual,'match':expected==actual}
    for layout in cfg['layouts']:
        source=Path(cfg['mapex'])/'kth_test_maps'/layout[4:]
        for filename in ('occ_map.npy','valid_space.npy'):
            original_key=next(k for k in original['source_hashes'] if k.endswith('/'+layout[4:]+'/'+filename))
            actual=digest(source/filename)
            expected=original['source_hashes'][original_key]
            checks['raw/'+layout+'/'+filename]={'expected':expected,'actual':actual,'match':expected==actual}
        raw=np.load(source/'occ_map.npy')
        valid=np.load(source/'valid_space.npy')>0
        h,w=raw.shape
        occupied=np.pad(raw==0,((0,h%2),(0,w%2)),constant_values=True)
        domain=np.pad(valid,((0,h%2),(0,w%2)),constant_values=False)
        shape=(occupied.shape[0]//2,2,occupied.shape[1]//2,2)
        with np.load(Path(cfg['assets'])/(layout+'.npz'),allow_pickle=False) as z:
            if not np.array_equal(occupied.reshape(shape).any(axis=(1,3)),z['occupied']):
                raise RuntimeError('GT reconstruction mismatch')
            if not np.array_equal(domain.reshape(shape).any(axis=(1,3)),z['domain']):
                raise RuntimeError('ROI reconstruction mismatch')
    if not all(v['match'] for v in checks.values()):
        raise RuntimeError('R4 source/data byte parity failed')
    return checks,json.loads((old_folder/'predictor.json').read_text())


def asset_from(cfg,layout):
    with np.load(Path(cfg['assets'])/(layout+'.npz'),allow_pickle=False) as z:
        asset={k:z[k].copy() for k in z.files}
    asset['layout']=layout
    return asset


def model_parity(inner,expected):
    for key in ('worker_sha256','bridge_sha256','upstream_preprocessing_sha256'):
        if inner.provenance[key]!=expected[key]:
            raise RuntimeError('Model code mismatch: '+key)
    for key in ('model_checkpoints','model_configs'):
        if [x['sha256'] for x in inner.provenance[key]]!=[x['sha256'] for x in expected[key]]:
            raise RuntimeError('Model bytes mismatch: '+key)


def clone_world(world):
    # The immutable sensor/world tables are shared; all mutable state is copied.
    other=object.__new__(type(world))
    other.__dict__=world.__dict__.copy()
    other.observed=world.observed.copy()
    return other


def p1(engine,adapter,cfg,out):
    results=[]
    determinism_done=False
    for layout in cfg['p1_layouts']:
        began=time.perf_counter()
        asset=asset_from(cfg,layout)
        world=adapter.world_from(engine.core,asset,cfg)
        candidates=engine.candidates(world)
        if not candidates:
            results.append({'layout':layout,'status':'NO_VALID_INITIAL_ACTION','candidates':0})
            continue
        predictions,mean,variance=engine.predictor.predict(world.observed)
        if not np.array_equal(predictions[:,world.observed!=.5],np.broadcast_to(world.observed[world.observed!=.5],predictions[:,world.observed!=.5].shape)):
            raise RuntimeError('Member predictions changed known cells')
        state_start=time.perf_counter()
        solver=GeometrySolver(engine,world,mean,variance,asset['occupied'],candidates)
        parity=solver.native_parity()
        remaining=max(.001,cfg['p1_state_seconds']-(time.perf_counter()-state_start))
        bounds=solver.solve(max_nodes=cfg['p1_geometry_nodes'],seconds=remaining)
        for index,certificate in bounds['witnesses'].items():
            if solver.replay_witness(certificate)!=int(index):
                raise RuntimeError('P1 geometry witness failed replay')
        scores,selected,_=engine.score(world,mean,variance,candidates)
        if selected[adapter.BASE] not in bounds['reachable_lower']:
            # Time-out during seed calculation may leave no witness yet.
            bounds['reachable_lower'].append(selected[adapter.BASE])
            bounds['reachable_lower'].sort()
            bounds['witnesses'][str(selected[adapter.BASE])]={'kind':'none'}
        if selected[adapter.BASE] not in bounds['reachable_upper']:
            raise RuntimeError('Baseline not enclosed')
        folder=out/layout
        folder.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(folder/'state.npz',observed=world.observed,pose=world.pose,mean=mean,variance=variance)
        result=dict(layout=layout,status='VALID_ACTION_BOUNDS_COMPLETE' if bounds['complete'] else 'VALID_ACTION_BOUNDS_UNRESOLVED',
                    shape=list(mean.shape),start=list(world.pose),native_baseline_index=selected[adapter.BASE],
                    baseline_scores=scores[adapter.BASE],candidates=[{'goal':list(c['goal']),'cost':c['cost'],'action_id':c['action_id']} for c in candidates],
                    parity_checks=parity,bounds=bounds,solver_stage_s=time.perf_counter()-state_start,
                    wall_s=time.perf_counter()-began,observed_hash=engine.core.array_hash(world.observed),
                    mean_hash=engine.core.array_hash(mean),variance_hash=engine.core.array_hash(variance))
        save(folder/'result.json',result)
        results.append(result)
        event(stage='P1_STATE_COMPLETE',layout=layout,status=result['status'],candidates=len(candidates),
              relevant_cells=bounds['correctable_relevant_cells'],lower=bounds['reachable_lower'],upper=bounds['reachable_upper'],wall_s=result['wall_s'])
        save(out/'progress.json',{'results':results})
        if not determinism_done:
            inner=engine.predictor.inner
            previous_cache=inner.cache_dir
            repeat=out/'uncached_determinism'
            repeat.mkdir(parents=True,exist_ok=True)
            inner.cache_dir=repeat
            try:
                actual_repeat=inner.predict(world.observed)
            finally:
                inner.cache_dir=previous_cache
            if not all(np.array_equal(a,b) for a,b in zip((predictions,mean,variance),actual_repeat)):
                raise RuntimeError('Fresh inference is not deterministic on P1 anchor')
            save(out/'determinism.json',{'status':'FRESH_UNCACHED_REPEAT_BYTE_IDENTICAL','layout':layout,'identity':inner.identity})
            determinism_done=True
    return {'status':'P1_VALID_BOUNDS_PREFLIGHT','results':results,'determinism_anchor_checked':determinism_done,
            'scope':'four development initial states; not whole-task impact or population evidence'}


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--phase',choices=('p1','p2'),required=True)
    args=parser.parse_args()
    cfg=json.loads(Path(args.config).read_text())
    out=Path(cfg['output'])/args.phase
    out.mkdir(parents=True,exist_ok=True)
    if (out/'summary.json').exists():
        raise RuntimeError('Output already completed; do not overwrite outcomes')
    source_checks,expected=verify_sources(cfg)
    sources={str(Path(__file__).name):digest(__file__),
             'geometry_solver.py':digest(Path(__file__).with_name('geometry_solver.py')),
             'protocol.json':digest(args.config)}
    if args.phase=='p2':
        sources['task_search.py']=digest(Path(__file__).with_name('task_search.py'))
    save(out/'seal.json',{'status':'SEALED_BEFORE_PHASE_OUTCOMES','config':cfg,'phase':args.phase,
                         'source_hashes':sources,'r4_source_parity':source_checks,'runtime':{
                         'python':sys.version.split()[0],'numpy':np.__version__,'scipy':scipy.__version__,'shapely':shapely.__version__}})
    adapter=load_adapter(cfg)
    core,Predictor,fast=adapter.dependencies(cfg)
    os.environ['MAPEX_ENSEMBLE_DIR']=cfg['ensemble_dir']
    cache=Path(cfg['output'])/'cache'
    cache.mkdir(parents=True,exist_ok=True)
    for source in cfg.get('reuse_cache_dirs',[]):
        for file in Path(source).glob('*.npz'):
            target=cache/file.name
            if not target.exists():
                shutil.copy2(file,target)
    began=time.perf_counter()
    inner=Predictor(Path(cfg['repo']),Path(cfg['mapex']),cache,worker_python=cfg['worker_python'])
    model_parity(inner,expected)
    save(out/'predictor.json',{'identity':inner.identity,'provenance':inner.provenance,'sha_parity_with_R4':True})
    event(stage='MODEL_AND_SOURCE_READY',phase=args.phase,startup_s=time.perf_counter()-began,
          real_model=True,checkpoint_parity=True,adapter_revision='A1')
    predictor=adapter.GuardedPredictor(inner,cfg)
    engine=adapter.Engine(core,fast,predictor,cfg)
    try:
        if args.phase=='p1':
            answer=p1(engine,adapter,cfg,out)
        else:
            from task_search import run_p2
            answer=run_p2(engine,adapter,cfg,out,clone_world,asset_from,save,event)
    except Exception as exc:
        import traceback
        traceback.print_exc()
        answer={'status':'INVALID_OR_INCOMPLETE_PREFLIGHT','reason':type(exc).__name__+': '+str(exc)}
    finally:
        inner.close()
    answer.update(model_calls=inner.calls,cache_hits=inner.cache_hits,inference_s=inner.inference_s,
                  maxrss_kb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                  confirmation='NOT_OPENED_TRAIN_SPLIT_UNVERIFIED',online_method='NOT_TRAINED')
    save(out/'summary.json',answer)
    event(stage='PHASE_COMPLETE',phase=args.phase,status=answer['status'],model_calls=inner.calls,
          inference_s=inner.inference_s)


if __name__=='__main__':
    main()
