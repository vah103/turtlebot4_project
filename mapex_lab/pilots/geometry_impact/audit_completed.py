"""Audit actual P2 evidence without new model inference or further search."""
import argparse
import hashlib
import json
from pathlib import Path

from run_preflight import asset_from, digest, load_adapter, save, verify_sources
import numpy as np
from task_search import TaskEvaluator


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    args=parser.parse_args()
    cfg=json.loads(Path(args.config).read_text())
    folder=Path(cfg['output'])/'p2'
    summary=json.loads((folder/'summary.json').read_text())
    if not summary.get('whole_task_bounds_certified'):
        raise RuntimeError('P2 did not produce a valid task interval')
    seal=json.loads((folder/'seal.json').read_text())
    assert seal['config']==cfg
    for name,expected in seal['source_hashes'].items():
        path=Path(args.config) if name=='protocol.json' else Path(__file__).parent/name
        assert digest(path)==expected, name
    source_checks,_=verify_sources(cfg)
    assert all(x['match'] for x in source_checks.values())
    model=json.loads((folder/'predictor.json').read_text())
    assert model['sha_parity_with_R4']
    adapter=load_adapter(cfg)
    core,_,fast=adapter.dependencies(cfg)

    class CacheOnly:
        def predict(self,observed):
            observed_hash=core.array_hash(observed)
            key=hashlib.sha256((model['identity']+observed_hash).encode()).hexdigest()
            with np.load(Path(cfg['output'])/'cache'/(key+'.npz'),allow_pickle=False) as z:
                assert str(z['identity'])==model['identity']
                assert str(z['observed_hash'])==observed_hash
                predictions,mean,var=(z[k].copy() for k in ('predictions','mean','variance'))
            known=observed!=.5
            assert np.array_equal(mean[known],observed[known])
            assert np.array_equal(predictions[:,known],np.broadcast_to(observed[known],predictions[:,known].shape))
            return predictions,mean,var

    engine=adapter.Engine(core,fast,CacheOnly(),cfg)
    asset=asset_from(cfg,cfg['p2_layout'])
    evaluator=TaskEvaluator(engine,asset,cfg['tu_seed'])
    goals=json.loads((folder/'tu_goals.json').read_text())
    assert [list(g) for g in evaluator.goals]==goals['goals']
    baseline=json.loads((folder/'baseline.json').read_text())
    target=tuple(baseline['reference']['counts'])
    def hits(counts):
        c,i,u,t=counts; tc,ti,tu,tt=target
        a,b=(i,u) if u else (1,1)
        x,y=(ti,tu) if tu else (1,1)
        return c>=tc and a*y>=x*b and t>=tt
    first=min(f['distance_steps'] for f in baseline['frames'] if hits(f['counts']))
    assert first==baseline['first_hit_steps']
    assert abs(first*cfg['resolution_m']-summary['baseline_first_hit_m'])<1e-9
    replay=adapter.world_from(core,asset,cfg)
    trace=[[0.,adapter.coverage(replay,asset)]]
    for j,frame in enumerate(baseline['frames']):
        assert core.array_hash(replay.observed)==frame['observed_hash']
        _,mean,var=engine.predictor.predict(replay.observed)
        assert core.array_hash(mean)==frame['mean_hash']
        assert core.array_hash(var)==frame['variance_hash']
        assert evaluator.counts(replay,mean)==tuple(frame['counts'])
        assert int(round(replay.distance_m/cfg['resolution_m']))==frame['distance_steps']
        if j<len(baseline['actions']):
            action=baseline['actions'][j]
            candidates=engine.candidates(replay)
            _,chosen,_=engine.score(replay,mean,var,candidates)
            selected=candidates[chosen[adapter.BASE]]
            assert selected['action_id']==action['action_id']
            assert [list(p) for p in selected['path']]==action['planned_path']
            termination=adapter.execute(replay,selected['path'],asset,cfg,trace)
            assert termination==action['termination']
    assert replay.collisions==0
    reference=adapter.outcome(replay,asset,cfg,engine.predictor,trace,baseline['terminal'])
    old=json.loads((Path(cfg['repo'])/'mapex_lab/pilots/visibility_impact_r4/results/run_20261008_controller_v2/summary.json').read_text())
    expected=old['results'][cfg['layouts'].index(cfg['p2_layout'])]['reference']
    for key in ('C_bar','Q','final_coverage','distance_m'):
        assert abs(reference[key]-expected[key])<=1e-9

    queue=json.loads((folder/'queue.json').read_text())
    open_bounds={int(k):v for k,v in queue['open_bounds_steps'].items()}
    assert len(queue['queued'])==len(open_bounds)==summary['open_nodes']
    assert {q['id'] for q in queue['queued']}==set(open_bounds)
    for item in queue['queued']:
        with np.load(folder/item['state_file'],allow_pickle=False) as z:
            observed=z['observed']; steps=int(round(float(z['distance_m'])/cfg['resolution_m']))
            assert int(z['collisions'])==0
        assert np.isin(observed,[0.,.5,1.]).all()
        known=observed!=.5
        assert np.array_equal(observed[known],asset['occupied'][known].astype(observed.dtype))
        assert sum(a['executed_steps'] for a in item['prefix'])==steps
        assert open_bounds[item['id']]==max(0,first-steps)
        if item['attainable']:
            assert all(a['geometry'] is not None for a in item['prefix'])
    lower=queue['incumbent_lower_steps']
    relaxed=max((x['upper_gain_steps'] for x in queue['relaxed_hits']),default=0)
    assert relaxed==queue['relaxed_terminal_upper_steps']
    upper=max(lower,relaxed,max(open_bounds.values(),default=0))
    assert 0<=lower<=upper<=first
    assert abs(summary['loss_lower_m']-lower*cfg['resolution_m'])<1e-9
    assert abs(summary['loss_upper_m']-upper*cfg['resolution_m'])<1e-9
    best=folder/'best_witness.json'
    if lower:
        witness=json.loads(best.read_text())
        world=adapter.world_from(core,asset,cfg)
        for action in witness['prefix']:
            assert core.array_hash(world.observed)==action['observed_hash']
            _,mean,var=engine.predictor.predict(world.observed)
            assert core.array_hash(mean)==action['mean_hash']
            assert core.array_hash(var)==action['variance_hash']
            patch=mean.copy(); certificate=action['geometry']
            if certificate['kind']=='all_unknown':
                patch[world.observed==.5]=asset['occupied'][world.observed==.5]
            elif certificate['kind']=='sparse':
                cells=np.asarray(certificate['corrected_flat'],dtype=np.int64)
                assert np.all(world.observed.ravel()[cells]==.5)
                patch.ravel()[cells]=asset['occupied'].ravel()[cells]
            else:
                assert certificate['kind']=='none'
            candidates=engine.candidates(world)
            _,chosen,_=engine.score(world,patch,var,candidates)
            selected=candidates[chosen[adapter.BASE]]
            assert selected['action_id']==action['action_id']
            assert adapter.execute(world,selected['path'],asset,cfg,[])==action['termination']
        _,mean,_=engine.predictor.predict(world.observed)
        assert hits(evaluator.counts(world,mean))
        assert core.array_hash(world.observed)==witness['observed_hash']
        assert first-int(round(world.distance_m/cfg['resolution_m']))==lower
    logs=json.loads((folder/'search_nodes.json').read_text())
    nodes=[x for x in logs if 'id' in x]
    unlogged_inflight=summary['processed_search_nodes']-len(nodes)
    assert unlogged_inflight in (0,1)
    if unlogged_inflight:
        assert open_bounds and summary['stop_reason'] not in ('SEARCH_COMPLETE','ONE_METRE_RESOLUTION_REACHED')
    branched=[x for x in nodes if x.get('outcome')=='BRANCHED']
    changed=[x for x in branched if len(x['validated_lower_actions'])>1]
    attained=[x for x in nodes if x.get('outcome')=='TARGET_REACHED_PROVED']
    result={'status':'PASS_REAL_P2_SOURCE_BASELINE_CACHE_FRONTIER_AND_WITNESS_AUDIT',
            'baseline_frames_recomputed':len(baseline['frames']),
            'baseline_native_selection_and_physical_replay':True,
            'open_queue_states_checked':len(open_bounds),
            'completed_node_records':len(nodes),'unlogged_inflight_node':unlogged_inflight,
            'lower_m':lower*cfg['resolution_m'],'upper_m':upper*cfg['resolution_m'],
            'positive_lower_witness_replayed':bool(lower),
            'nodes_with_multiple_native_validated_actions':len(changed),
            'proved_target_hits':len(attained),'new_inference_calls':0,
            'limitations':'Audits recorded evidence and resource-stop ledger; finite tests do not establish universal software correctness.'}
    save(folder/'audit.json',result)
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    main()
