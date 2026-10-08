"""R4 stage-A implementation: real ensemble, decision census, full suffixes."""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import time

os.environ.setdefault('OMP_NUM_THREADS', '2')
os.environ.setdefault('MKL_NUM_THREADS', '2')
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np
from scipy import ndimage as ndi
from estimation import estimate, sample_flips, stable_seed

ARMS = tuple(f'{r}_{m}_{u}' for r in ('native', 'sensor')
             for m in ('P', 'GT') for u in ('U', 'uniform'))
BASE = 'native_P_U'
CONTRASTS = ((BASE, 'native_GT_U'), ('sensor_P_U', 'sensor_GT_U'),
             ('native_P_uniform', 'native_GT_uniform'),
             ('sensor_P_uniform', 'sensor_GT_uniform'),
             (BASE, 'sensor_P_U'))


def save(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    tmp.replace(path)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''): h.update(block)
    return h.hexdigest()


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec); sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def dependencies(cfg):
    sys.path.insert(0, str(Path(cfg['pilot']).parent))
    from active_verification_2d import core
    from active_verification_2d.predictor import RealLamaPredictor
    fast = load_module(cfg['fast_bfs'], 'r4_fast_bfs')
    return core, RealLamaPredictor, fast


class ResourceAbort(RuntimeError): pass


class GuardedPredictor:
    def __init__(self, predictor, cfg):
        self.inner, self.cfg = predictor, cfg
        self.started=time.perf_counter()
    def predict(self, observed):
        if time.perf_counter()-self.started > self.cfg['maximum_wall_seconds']:
            raise ResourceAbort('Declared wall-clock resource limit reached')
        key = hashlib.sha256((self.inner.identity+dependencies_hash(observed)).encode()).hexdigest()
        if self.inner.inference_s >= self.cfg['maximum_inference_seconds'] and not (self.inner.cache_dir/(key+'.npz')).exists():
            raise ResourceAbort('Declared inference resource limit reached')
        if shutil.disk_usage(self.inner.cache_dir).free < 750*1024*1024:
            raise ResourceAbort('Insufficient disk reserve; existing data retained')
        def timeout(signum, frame): raise ResourceAbort('Single inference timeout')
        old = signal.signal(signal.SIGALRM, timeout)
        signal.alarm(self.cfg['single_inference_timeout_s'])
        try: return self.inner.predict(observed)
        finally:
            signal.alarm(0); signal.signal(signal.SIGALRM, old)


class Sensor:
    """Same half-cell first-hit rays as the physical GridWorld, including return."""
    def __init__(self, cfg):
        angles = np.linspace(0, 2*np.pi, cfg['sensor_rays'], endpoint=False)
        steps = np.arange(0, cfg['sensor_range_m']/cfg['resolution_m']+.25, .5)
        self.dr = np.floor(steps[:, None]*np.sin(angles)[None, :]+.5).astype(np.int32)
        self.dc = np.floor(steps[:, None]*np.cos(angles)[None, :]+.5).astype(np.int32)
    def visible(self, occupied, pose):
        h, w = occupied.shape; active = np.ones(self.dr.shape[1], dtype=bool)
        visible = np.zeros((h, w), dtype=bool)
        for dr, dc in zip(self.dr, self.dc):
            r, c = pose[0]+dr, pose[1]+dc
            active &= (r >= 0) & (r < h) & (c >= 0) & (c < w)
            if not active.any(): break
            inds = np.flatnonzero(active); rr, cc = r[inds], c[inds]
            visible[rr, cc] = True
            active[inds[occupied[rr, cc]]] = False
        return visible


def dependencies_hash(observed):
    a=np.ascontiguousarray(observed)
    return hashlib.sha256(str(a.shape).encode()+str(a.dtype).encode()+a.tobytes()).hexdigest()


class Engine:
    def __init__(self, core, fast, predictor, cfg):
        self.core, self.fast, self.predictor, self.cfg = core, fast, predictor, cfg
        self.native = core.MapExNumerics(Path(cfg['repo'])/'mapex_lab/scripts/mapex.py',
                          cfg['resolution_m'], cfg['sensor_range_m'], cfg['native_rays'])
        self.sensor = Sensor(cfg)
    def candidates(self, world):
        observed, pose = world.observed, world.pose
        reps = self.core.frontier_representatives(observed, 10)
        distant = [p for p in reps if np.linalg.norm(np.array(p)-pose)*world.resolution >= 1-1e-9]
        reps = distant or reps
        safe = self.core.observed_traversable(observed, self.cfg['robot_radius_m']/world.resolution)
        dist, parent = self.fast.shortest_paths(safe, pose)
        left = max(0, int(math.floor((self.cfg['budget_m']-world.distance_m+1e-7)/world.resolution)))
        candidates = []
        for goal in sorted(reps):
            if goal == pose or dist[goal] <= 0: continue
            path = self.core.recover_path(parent, pose, goal)
            if path is None or len(path)<2: continue
            executed = path[:left+1]
            if len(executed)<2: continue
            action_id = hashlib.sha256(np.asarray(executed, dtype=np.int32).tobytes()).hexdigest()
            candidates.append(dict(goal=goal, path=executed, action_id=action_id,
                                   cost=float(np.linalg.norm(np.asarray(goal)-pose)*world.resolution)))
        return candidates
    def score(self, world, mean, variance, candidates, truth=None, all_arms=False):
        unknown = world.observed==.5; scores={arm:[] for arm in ARMS} if all_arms else {BASE:[]}
        mismatch=[]
        for candidate in candidates:
            goal=candidate['goal']
            np_mask=self.native.visibility(goal, mean, world.observed)
            masks={'native_P':np_mask}
            if all_arms:
                masks['native_GT']=self.native.visibility(goal, truth.astype(np.float32), world.observed)
                masks['sensor_P']=self.sensor.visible(mean>=.5, goal)&unknown
                masks['sensor_GT']=self.sensor.visible(truth, goal)&unknown
                mismatch.append(dict(goal=list(goal),
                    native_false_visible=int((np_mask & ~masks['sensor_GT']).sum()),
                    native_missed_visible=int((~np_mask & masks['sensor_GT']).sum()),
                    sensor_false_visible=int((masks['sensor_P'] & ~masks['sensor_GT']).sum()),
                    sensor_missed_visible=int((~masks['sensor_P'] & masks['sensor_GT']).sum())))
            for arm in scores:
                r,m,u=arm.split('_'); mask=masks[r+'_'+m]
                benefit=float(variance[mask].sum()) if u=='U' else float(mask.sum())
                scores[arm].append(benefit/max(candidate['cost'],1e-9))
        selected={}
        for arm, values in scores.items():
            idx=min(range(len(candidates)), key=lambda i:(-values[i], candidates[i]['cost'],candidates[i]['goal']))
            selected[arm]=idx
        return scores, selected, mismatch


def world_from(core, asset, cfg, snapshot=None):
    pose=tuple(asset['start']) if snapshot is None else tuple(snapshot['pose'])
    observed=None if snapshot is None else snapshot['observed']
    w=core.GridWorld(asset['occupied'],cfg['resolution_m'],pose,cfg['sensor_range_m'],
                     cfg['sensor_rays'],cfg['robot_radius_m'],observed)
    if snapshot is None: w.sense()
    else:
        w.distance_m=float(snapshot['distance_m']); w.collisions=int(snapshot['collisions'])
    return w


def coverage(world, asset):
    free=asset['domain'] & ~asset['occupied']
    return float(((world.observed==0)&free).sum()/free.sum())


def quality(world, mean, asset):
    predicted=np.where(world.observed!=.5,world.observed,mean)>=.5
    target=asset['occupied']; domain=asset['domain']
    tp=int((predicted&target&domain).sum()); union=int(((predicted|target)&domain).sum())
    return float(tp/union) if union else 1.


def known_safe_step(observed,pose,radius_cells):
    """Local equivalent of observed_traversable; no truth or unknown inflation."""
    r,c=pose;h,w=observed.shape
    if not (0<=r<h and 0<=c<w) or observed[r,c]!=0:return False
    n=int(math.ceil(radius_cells))
    for dr in range(-n,n+1):
        for dc in range(-n,n+1):
            if dr*dr+dc*dc > (radius_cells+1e-9)**2:continue
            rr,cc=r+dr,c+dc
            if not (0<=rr<h and 0<=cc<w) or observed[rr,cc]==1:return False
    return True


def execute(world, path, asset, cfg, trace):
    changed=False
    for pose in path[1:]:
        if not known_safe_step(world.observed,pose,cfg['robot_radius_m']/world.resolution):
            world.sense();trace.append([world.distance_m,coverage(world,asset)])
            return 'OBSERVED_PATH_BLOCKED'
        if not world.step(pose):
            # Physical collision is an episode failure; preserve, never drop it.
            trace.append([world.distance_m,coverage(world,asset)])
            return 'COLLISION'
        changed=True
        if round(world.distance_m/world.resolution)%cfg['scan_stride_steps']==0:
            world.sense(); trace.append([world.distance_m,coverage(world,asset)])
    # Same action-end scan for every arm; native endpoint query is thus sensed.
    if changed:
        world.sense(); trace.append([world.distance_m,coverage(world,asset)])
    return 'DONE'


def outcome(world, asset, cfg, predictor, trace, terminal):
    _,mean,_=predictor.predict(world.observed)
    pts=trace+[[cfg['budget_m'],coverage(world,asset)]]
    xy=np.asarray(pts,dtype=float)
    auc=float(np.sum(xy[:-1,1]*np.diff(xy[:,0]))/cfg['budget_m'])
    return dict(status='COMPLETE',terminal=terminal,C_bar=auc,Q=quality(world,mean,asset),
                final_coverage=coverage(world,asset),distance_m=world.distance_m,
                collisions=world.collisions,observed_hash=hashlib.sha256(world.observed.tobytes()).hexdigest())


def continue_episode(engine, world, asset, trace, first=None):
    cfg=engine.cfg; count=0
    if first is not None:
        if execute(world,first['path'],asset,cfg,trace)=='COLLISION':
            return outcome(world,asset,cfg,engine.predictor,trace,'COLLISION')
    while world.distance_m < cfg['budget_m']-cfg['resolution_m']/2:
        count+=1
        if count>cfg['maximum_decisions']: raise ResourceAbort('Decision resource cap; not mission terminal')
        candidates=engine.candidates(world)
        if not candidates: return outcome(world,asset,cfg,engine.predictor,trace,'NO_SELECTABLE_CANDIDATE')
        _,mean,var=engine.predictor.predict(world.observed)
        _,selected,_=engine.score(world,mean,var,candidates)
        if execute(world,candidates[selected[BASE]]['path'],asset,cfg,trace)=='COLLISION':
            return outcome(world,asset,cfg,engine.predictor,trace,'COLLISION')
    return outcome(world,asset,cfg,engine.predictor,trace,'BUDGET_REACHED')


def reference(engine, asset, folder):
    cfg=engine.cfg; folder.mkdir(parents=True,exist_ok=True)
    w=world_from(engine.core,asset,cfg); trace=[[0.,coverage(w,asset)]]; records=[]
    terminal='BUDGET_REACHED'
    while w.distance_m < cfg['budget_m']-cfg['resolution_m']/2:
        if len(records)>=cfg['maximum_decisions']: raise ResourceAbort('Reference decision cap')
        candidates=engine.candidates(w)
        if not candidates:
            terminal='NO_SELECTABLE_CANDIDATE'; break
        predictions,mean,var=engine.predictor.predict(w.observed)
        started=time.perf_counter()
        scores,chosen,mismatch=engine.score(w,mean,var,candidates,asset['occupied'],True)
        state_id=len(records)
        np.savez_compressed(folder/f'state_{state_id:04d}.npz',observed=w.observed,
                            pose=np.asarray(w.pose),distance_m=w.distance_m,collisions=w.collisions)
        actions={arm:candidates[idx]['action_id'] for arm,idx in chosen.items()}
        record=dict(id=state_id,distance_m=w.distance_m,pose=list(w.pose),
                    observed_hash=engine.core.array_hash(w.observed),
                    predictions_hash=engine.core.array_hash(predictions),
                    mean_hash=engine.core.array_hash(mean),variance_hash=engine.core.array_hash(var),
                    prediction_cache_key=hashlib.sha256((engine.predictor.inner.identity+engine.core.array_hash(w.observed)).encode()).hexdigest(),
                    trace=[p[:] for p in trace],candidates=candidates,scores=scores,
                    chosen=chosen,actions=actions,mismatch=mismatch,
                    scoring_s=time.perf_counter()-started)
        records.append(record); save(folder/'decisions_partial.json',records)
        print(json.dumps(dict(stage='REFERENCE_DECISION',layout=asset['layout'],decision=state_id,
                             distance_m=w.distance_m,candidates=len(candidates),
                             action_changed=len(set(actions.values()))>1,
                             model_inference_s=engine.predictor.inner.inference_s)),flush=True)
        record['execution_status']=execute(w,candidates[chosen[BASE]]['path'],asset,cfg,trace)
        save(folder/'decisions_partial.json',records)
        if record['execution_status']=='COLLISION':
            terminal='COLLISION'; break
    result=outcome(w,asset,cfg,engine.predictor,trace,terminal)
    save(folder/'reference.json',dict(outcome=result,decisions=records,trace=trace))
    return result,records


def branches(engine,asset,folder,ref,records):
    actions={r['id']:list(r['actions'].values()) for r in records}
    selection=sample_flips(actions,engine.cfg['sample_cap'],stable_seed(engine.cfg['seed'],asset['layout']))
    save(folder/'sampling.json',selection)
    rows=[]
    for state_id in selection['selected']:
        record=records[state_id]
        with np.load(folder/f'state_{state_id:04d}.npz',allow_pickle=False) as z:
            snapshot={k:z[k].copy() for k in z.files}
        snapshot['pose']=tuple(snapshot['pose'])
        outcomes={record['actions'][BASE]:ref}
        # One physical suffix per distinct full action, including reference replay.
        w=world_from(engine.core,asset,engine.cfg,snapshot)
        replay=continue_episode(engine,w,asset,[p[:] for p in record['trace']],record['candidates'][record['chosen'][BASE]])
        for key in ['C_bar','Q','distance_m','observed_hash','collisions','terminal']:
            if replay[key]!=ref[key] and not (isinstance(replay[key],float) and abs(replay[key]-ref[key])<1e-10):
                raise RuntimeError('Baseline suffix replay parity failed: '+key)
        for arm in ARMS:
            action_id=record['actions'][arm]
            artifact=folder/f'branch_{state_id:04d}_{action_id[:12]}.json'
            context=hashlib.sha256(json.dumps(dict(seal=digest(folder.parent/'seal.json'),
                              layout=asset['layout'],state=state_id,snapshot=record['observed_hash'],
                              pose=record['pose'],distance_m=record['distance_m'],action=action_id,
                              RNG='deterministic-no-random-controller'),sort_keys=True).encode()).hexdigest()
            if action_id not in outcomes:
                if artifact.exists():
                    cached=json.loads(artifact.read_text())
                    if cached.get('replay_context')!=context: raise RuntimeError('Branch cache context mismatch')
                    outcomes[action_id]=cached
                else:
                    w=world_from(engine.core,asset,engine.cfg,snapshot)
                    outcomes[action_id]=continue_episode(engine,w,asset,[p[:] for p in record['trace']],
                                               record['candidates'][record['chosen'][arm]])
            result=dict(outcomes[action_id],replay_context=context,action_id=action_id)
            save(artifact,result)
        for before,after in CONTRASTS:
            b=outcomes[record['actions'][before]]; a=outcomes[record['actions'][after]]
            rows.append(dict(state=state_id,before=before,after=after,
                             delta_C=a['C_bar']-b['C_bar'],delta_Q=a['Q']-b['Q'],
                             same_action=record['actions'][before]==record['actions'][after],
                             replay_parity=True))
        save(folder/'effects_partial.json',rows)
        print(json.dumps(dict(stage='BRANCH_COMPLETE',layout=asset['layout'],state=state_id,
                             distinct_actions=len(outcomes),model_inference_s=engine.predictor.inner.inference_s)),flush=True)
    estimates={}
    for before,after in CONTRASTS:
        values=[r for r in rows if r['before']==before and r['after']==after]
        estimates[before+'__'+after]=estimate(selection['T'],selection['M'],
                               [r['delta_C'] for r in values],[r['delta_Q'] for r in values],
                               alpha=.05/(len(engine.cfg['layouts'])*len(CONTRASTS)))
    flip_rates={before+'__'+after:dict(count=sum(r['actions'][before]!=r['actions'][after] for r in records),
                T=len(records),rate=(sum(r['actions'][before]!=r['actions'][after] for r in records)/len(records)
                                    if records else None)) for before,after in CONTRASTS}
    answer=dict(status='COMPLETE_DEVELOPMENT_ONLY',reference=ref,sampling=selection,
                estimates=estimates,action_flip_rates=flip_rates,effects=rows,independent_building_verified=False)
    save(folder/'summary.json',answer)
    return answer


def aggregate(complete):
    """Equal layout descriptions and pooled secondary; no building inference."""
    valid=[r for r in complete if r['sampling']['T']>0]
    out=dict(unit='layout; building independence unverified',valid_episodes=len(valid),
             zero_decision_episodes=len(complete)-len(valid),contrasts={})
    if not valid: return out
    for before,after in CONTRASTS:
        key=before+'__'+after; estimates=[r['estimates'][key] for r in valid]
        weights=np.asarray([r['sampling']['T'] for r in valid],dtype=float);weights/=weights.sum()
        out['contrasts'][key]=dict(
            equal_layout_rate=float(np.mean([e['rate'] for e in estimates])),
            equal_layout_mean_effect=float(np.mean([e['mean_effect'] for e in estimates])),
            cohort_rate_interval=np.mean([e['rate_interval'] for e in estimates],axis=0).tolist(),
            cohort_mean_interval=np.mean([e['mean_interval'] for e in estimates],axis=0).tolist(),
            pooled_decision_rate=float(weights@np.asarray([e['rate'] for e in estimates])),
            pooled_decision_mean_effect=float(weights@np.asarray([e['mean_effect'] for e in estimates])),
            interval_scope='finite completed reference cohort; 95% simultaneous across planned layout/contrast estimates; bounded mean intervals conservative',
            generalization_interval=None)
    return out


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--protocol',required=True)
    args=parser.parse_args(); cfg=json.loads(Path(args.protocol).read_text())
    out=Path(cfg['output']);out.mkdir(parents=True,exist_ok=True)
    core,Predictor,fast=dependencies(cfg)
    identities={p:digest(p) for p in [__file__,str(Path(__file__).with_name('estimation.py')),
               cfg['pilot']+'/core.py',cfg['pilot']+'/predictor.py',cfg['fast_bfs'],
               cfg['repo']+'/mapex_lab/scripts/mapex.py',args.protocol]}
    for layout in cfg['layouts']:
        p=Path(cfg['assets'])/(layout+'.npz'); identities[str(p)]=digest(p)
        source=Path(cfg['mapex'])/'kth_test_maps'/layout[4:]
        for name in ('occ_map.npy','valid_space.npy'):
            identities[str(source/name)]=digest(source/name)
        raw=np.load(source/'occ_map.npy'); valid=np.load(source/'valid_space.npy')
        if not np.array_equal(np.unique(raw),[0,254]): raise RuntimeError('Unexpected raw map labels')
        h,w=raw.shape
        occ=np.pad(raw==0,((0,h%2),(0,w%2)),constant_values=True)
        domain=np.pad(valid>0,((0,h%2),(0,w%2)),constant_values=False)
        shape=(occ.shape[0]//2,2,occ.shape[1]//2,2)
        with np.load(p,allow_pickle=False) as z:
            if not np.array_equal(occ.reshape(shape).any(axis=(1,3)),z['occupied']): raise RuntimeError('Asset occupancy mismatch')
            if not np.array_equal(domain.reshape(shape).any(axis=(1,3)),z['domain']): raise RuntimeError('Asset domain mismatch')
    seal_path=out/'seal.json'
    if seal_path.exists():
        if json.loads(seal_path.read_text())['source_hashes']!=identities:
            raise RuntimeError('Source changed after seal; use a new run output')
    else: save(seal_path,dict(source_hashes=identities,config=cfg,status='SEALED_BEFORE_NEW_OUTCOMES'))
    os.environ['MAPEX_ENSEMBLE_DIR']=cfg['ensemble_dir']
    inner=Predictor(Path(cfg['repo']),Path(cfg['mapex']),out/'cache',worker_python=cfg['worker_python'])
    seal=json.loads(seal_path.read_text())
    if seal.get('predictor_identity') not in (None,inner.identity):
        inner.close();raise RuntimeError('Model provenance changed after seal')
    if 'predictor_identity' not in seal:
        seal['predictor_identity']=inner.identity;seal['predictor_provenance']=inner.provenance
        save(seal_path,seal)
    predictor=GuardedPredictor(inner,cfg); engine=Engine(core,fast,predictor,cfg)
    save(out/'predictor.json',inner.provenance)
    complete=[]; failure=None
    try:
        for layout in cfg['layouts']:
            folder=out/layout
            if (folder/'summary.json').exists():
                complete.append(json.loads((folder/'summary.json').read_text()));continue
            with np.load(Path(cfg['assets'])/(layout+'.npz'),allow_pickle=False) as z:
                asset={k:z[k].copy() for k in z.files};asset['layout']=layout
            if (folder/'reference.json').exists():
                loaded=json.loads((folder/'reference.json').read_text());ref=loaded['outcome'];records=loaded['decisions']
            else: ref,records=reference(engine,asset,folder)
            complete.append(branches(engine,asset,folder,ref,records))
            save(out/'progress.json',dict(completed_layouts=len(complete),total_layouts=len(cfg['layouts'])))
    except ResourceAbort as exc:
        failure=dict(status='RESOURCE_LIMIT_INCONCLUSIVE',reason=str(exc))
    except Exception as exc:
        import traceback
        traceback.print_exc()
        failure=dict(status='INVALID_RUN_INCONCLUSIVE',reason=type(exc).__name__+': '+str(exc))
    finally: inner.close()
    final=dict(status='COMPLETE_DEVELOPMENT_PILOT' if len(complete)==len(cfg['layouts']) else 'INCOMPLETE_INCONCLUSIVE',
               complete_layouts=len(complete),planned_layouts=len(cfg['layouts']),failure=failure,
               model_inference_s=inner.inference_s,model_calls=inner.calls,cache_hits=inner.cache_hits,
               confirmation_status='BLOCKED_UNVERIFIED_BUILDING_AND_PREDICTOR_SPLIT',
               online_f_g_status='NOT_TRAINED_NOT_TESTED',results=complete)
    final['cohort']=aggregate(complete)
    save(out/'summary.json',final); print(json.dumps(final),flush=True)


if __name__=='__main__': main()
