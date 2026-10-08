"""Audit raw R4 evidence and replay physical reference observations.

No new actions, sampling changes, model training, or research outcome runs.
"""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import numpy as np
import runner

def close(a,b):
    if isinstance(a,(float,int)) and isinstance(b,(float,int)):return abs(a-b)<1e-9
    return a==b

def aggregate_buildings(rows,metadata):
    if set(r['layout'] for r in rows)!=set(x['layout'] for x in metadata['layouts']):
        return dict(status='WAIT_FOR_ALL_FIXED_REFERENCE_EPISODES',planned_buildings=metadata['verified_buildings'])
    valid={r['layout']:r for r in rows if r['T']>0};groups={}
    for building,layouts in metadata['groups'].items():
        episodes=[valid[x] for x in layouts if x in valid]
        if not episodes:continue
        values={}
        for before,after in runner.CONTRASTS:
            key=before+'__'+after;es=[r['all_contrasts'][key] for r in episodes]
            values[key]=dict(rate=float(np.mean([e['rate'] for e in es])),
                            mean_effect=float(np.mean([e['mean_effect'] for e in es])),
                            rate_interval=np.mean([e['rate_interval'] for e in es],axis=0).tolist(),
                            mean_interval=np.mean([e['mean_interval'] for e in es],axis=0).tolist())
        groups[building]=dict(valid_floor_start_episodes=len(episodes),contrasts=values)
    result=dict(status='FINITE_DEVELOPMENT_COHORT_ONLY',planned_buildings=metadata['verified_buildings'],
                valid_buildings=len(groups),groups=groups,contrasts={},generalization_interval=None)
    if not groups:return result
    for before,after in runner.CONTRASTS:
        key=before+'__'+after;es=[g['contrasts'][key] for g in groups.values()]
        result['contrasts'][key]=dict(rate=float(np.mean([e['rate'] for e in es])),
                            mean_effect=float(np.mean([e['mean_effect'] for e in es])),
                            rate_interval=np.mean([e['rate_interval'] for e in es],axis=0).tolist(),
                            mean_interval=np.mean([e['mean_interval'] for e in es],axis=0).tolist())
    result['weighting']='Equal fixed floor/start episodes within each building, then equal buildings; zero-decision episodes excluded only from decision frequency'
    result['interval_scope']='Known finite cohort; simultaneous episode sampling bounds propagated through fixed positive weights; not uncertainty over new buildings'
    return result

def audit(cfg):
    root=Path(cfg['output']);seal=json.loads((root/'seal.json').read_text())
    if (root/'invalidation.json').exists():raise RuntimeError('Run invalidated; debug data only')
    for path,expected in seal['source_hashes'].items():
        assert runner.digest(path)==expected,'Sealed source changed: '+path
    core,_,_=runner.dependencies(cfg);rows=[];missing=[];primary='native_P_U__native_GT_U'
    for layout in cfg['layouts']:
        folder=root/layout
        if not (folder/'summary.json').exists():missing.append(layout);continue
        summary=json.loads((folder/'summary.json').read_text())
        source=json.loads((folder/'reference.json').read_text());records=source['decisions']
        sampling=summary['sampling'];T=sampling['T'];assert T==len(records)
        flip_ids=[r['id'] for r in records if len(set(r['actions'].values()))>1]
        assert sampling['M']==len(flip_ids)
        selected=sampling['selected'];assert len(set(selected))==sampling['k']
        assert all(i in flip_ids for i in selected)
        expected=runner.sample_flips({r['id']:list(r['actions'].values()) for r in records},
                    cfg['sample_cap'],runner.stable_seed(cfg['seed'],layout))
        assert sampling==expected,'Sample changed after outcomes'
        with np.load(Path(cfg['assets'])/(layout+'.npz')) as z:a={k:z[k].copy() for k in z.files}
        w=runner.world_from(core,a,cfg);trace=[[0.,runner.coverage(w,a)]];collision_probe=None
        terminal='BUDGET_REACHED'
        for record in records:
            assert record['id']<T and core.array_hash(w.observed)==record['observed_hash']
            assert list(w.pose)==record['pose'] and close(w.distance_m,record['distance_m'])
            cache=root/'cache'/(record['prediction_cache_key']+'.npz')
            with np.load(cache) as z:
                assert str(z['identity'])==seal['predictor_identity']
                assert core.array_hash(z['predictions'])==record['predictions_hash']
                assert core.array_hash(z['mean'])==record['mean_hash']
                assert core.array_hash(z['variance'])==record['variance_hash']
                known=w.observed!=.5
                assert np.array_equal(z['mean'][known],w.observed[known])
                assert (z['variance'][known]==0).all()
            for candidate in record['candidates']:
                path=candidate['path'];assert tuple(path[0])==w.pose
                assert hashlib.sha256(np.asarray(path,dtype=np.int32).tobytes()).hexdigest()==candidate['action_id']
                assert all(abs(x[0]-y[0])+abs(x[1]-y[1])==1 for x,y in zip(path,path[1:]))
            path=record['candidates'][record['chosen'][runner.BASE]]['path']
            if cfg.get('controller_revision'):
                following=(records[record['id']+1]['distance_m'] if record['id']+1<T else summary['reference']['distance_m'])
                steps=int(round((following-record['distance_m'])/cfg['resolution_m']))
                assert 0<=steps<len(path)
                for pose in path[1:steps+1]:
                    assert w.step(tuple(pose)),'Replayed measured-safe prefix collided'
                    if round(w.distance_m/w.resolution)%cfg['scan_stride_steps']==0:
                        w.sense();trace.append([w.distance_m,runner.coverage(w,a)])
                status=record['execution_status']
                if status=='OBSERVED_PATH_BLOCKED':
                    assert steps+1<len(path)
                    safe=core.observed_traversable(w.observed,cfg['robot_radius_m']/cfg['resolution_m'])
                    assert not safe[tuple(path[steps+1])],'Action abort not supported by measurements'
                    w.sense();trace.append([w.distance_m,runner.coverage(w,a)])
                elif status=='COLLISION':
                    assert steps+1<len(path)
                    safe=core.observed_traversable(w.observed,cfg['robot_radius_m']/cfg['resolution_m'])
                    assert safe[tuple(path[steps+1])],'Known-blocked collision reproduced A0 defect'
                    assert not w.step(tuple(path[steps+1]))
                    trace.append([w.distance_m,runner.coverage(w,a)]);terminal='COLLISION'
                    collision_probe=dict(next_pose=path[steps+1],blocked_in_updated_observation=False,
                                         decision_distance_m=record['distance_m'],collision_distance_m=w.distance_m)
                    break
                else:
                    assert status=='DONE' and steps==len(path)-1
                    if steps:w.sense();trace.append([w.distance_m,runner.coverage(w,a)])
                continue
            moved=False
            for pose in path[1:]:
                pose=tuple(pose)
                if not w._physical_free[pose]:
                    current_safe=core.observed_traversable(w.observed,cfg['robot_radius_m']/cfg['resolution_m'])
                    collision_probe=dict(next_pose=list(pose),
                             blocked_in_updated_observation=bool(not current_safe[pose]),
                             decision_distance_m=record['distance_m'],collision_distance_m=w.distance_m)
                if not w.step(pose):
                    trace.append([w.distance_m,runner.coverage(w,a)]);terminal='COLLISION';break
                moved=True
                if round(w.distance_m/w.resolution)%cfg['scan_stride_steps']==0:
                    w.sense();trace.append([w.distance_m,runner.coverage(w,a)])
            if terminal=='COLLISION':break
            if moved:
                w.sense();trace.append([w.distance_m,runner.coverage(w,a)])
        ref=summary['reference'];assert hashlib.sha256(w.observed.tobytes()).hexdigest()==ref['observed_hash']
        assert close(w.distance_m,ref['distance_m']) and w.collisions==ref['collisions']
        assert len(trace)==len(source['trace']) and np.allclose(trace,source['trace'],rtol=0,atol=1e-10)
        points=trace+[[cfg['budget_m'],runner.coverage(w,a)]]
        integral=sum((b[0]-a0[0])*a0[1] for a0,b in zip(points,points[1:]))/cfg['budget_m']
        assert close(integral,ref['C_bar'])
        final_key=hashlib.sha256((seal['predictor_identity']+core.array_hash(w.observed)).encode()).hexdigest()
        with np.load(root/'cache'/(final_key+'.npz')) as z:
            completed=np.where(w.observed==.5,z['mean'],w.observed)>=.5
        occupied=a['occupied'];domain=a['domain']
        union=((completed|occupied)&domain).sum();intersection=((completed&occupied)&domain).sum()
        Q=float(intersection/union) if union else 1.
        assert close(Q,ref['Q'])
        distinct=0;context_seal=runner.digest(root/'seal.json')
        for state in selected:
            record=records[state];outcomes={}
            prefix=record['trace']+[[record['distance_m'],record['trace'][-1][1]]]
            past=sum((b[0]-a0[0])*a0[1] for a0,b in zip(prefix,prefix[1:]))/cfg['budget_m']
            remaining=(cfg['budget_m']-record['distance_m'])/cfg['budget_m']
            min_auc=past+remaining*record['trace'][-1][1]
            max_auc=past+remaining
            for action in set(record['actions'].values()):
                artifact=folder/f'branch_{state:04d}_{action[:12]}.json';x=json.loads(artifact.read_text())
                context=hashlib.sha256(json.dumps(dict(seal=context_seal,layout=layout,state=state,
                            snapshot=record['observed_hash'],pose=record['pose'],distance_m=record['distance_m'],
                            action=action,RNG='deterministic-no-random-controller'),sort_keys=True).encode()).hexdigest()
                assert x['status']=='COMPLETE' and x['replay_context']==context and x['action_id']==action
                assert x['distance_m']<=cfg['budget_m']+1e-7
                assert min_auc-1e-9<=x['C_bar']<=max_auc+1e-9,'Suffix metric inconsistent with fixed common past and full-budget AUC'
                assert 0<=x['Q']<=1
                outcomes[action]=x
            distinct+=len(outcomes)-1
            native=outcomes[record['actions'][runner.BASE]]
            assert all(close(native[k],ref[k]) for k in ['C_bar','Q','distance_m','collisions','terminal','observed_hash'])
            for before,after in runner.CONTRASTS:
                row=next(e for e in summary['effects'] if e['state']==state and e['before']==before and e['after']==after)
                b=outcomes[record['actions'][before]];a1=outcomes[record['actions'][after]]
                assert close(row['delta_C'],a1['C_bar']-b['C_bar']) and close(row['delta_Q'],a1['Q']-b['Q'])
                if row['same_action']:assert row['delta_C']==0 and row['delta_Q']==0
        for before,after in runner.CONTRASTS:
            key=before+'__'+after;values=[x for x in summary['effects'] if x['before']==before and x['after']==after]
            e=runner.estimate(T,sampling['M'],[x['delta_C'] for x in values],[x['delta_Q'] for x in values],alpha=.05/(len(cfg['layouts'])*len(runner.CONTRASTS)))
            assert e==summary['estimates'][key]
        prim=summary['estimates'][primary]
        mismatch=[x for r in records for x in r['mismatch']]
        rows.append(dict(layout=layout,T=T,M=sampling['M'],k=sampling['k'],selected_states=selected,
                         additional_action_suffixes=distinct,reference=ref,
                         observed_path_abort_count=sum(r.get('execution_status')=='OBSERVED_PATH_BLOCKED' for r in records),
                         native_GT_action_flip=summary['action_flip_rates'][primary],
                         native_GT_primary=prim,all_contrasts=summary['estimates'],
                         baseline_collision_probe=collision_probe,
                         candidates=sum(len(r['candidates']) for r in records),
                         mean_candidate_native_false_visible_cells=float(np.mean([x['native_false_visible'] for x in mismatch])) if mismatch else None,
                         mean_candidate_native_missed_visible_cells=float(np.mean([x['native_missed_visible'] for x in mismatch])) if mismatch else None,
                         physical_reference_replay='EXACT',branch_auc_common_prefix_bounds='PASS',predictor_training_overlap_verified=False))
    result=dict(status='AUDITED_COMPLETE_DEVELOPMENT_ONLY' if not missing else 'AUDITED_PARTIAL_INCONCLUSIVE',
                planned_layouts=len(cfg['layouts']),completed_layouts=len(rows),missing_layouts=missing,
                total_valid_decisions=sum(x['T'] for x in rows),total_action_flip_states=sum(x['M'] for x in rows),
                total_sampled_states=sum(x['k'] for x in rows),rows=rows,
                secondary_layout_aggregate=runner.aggregate([json.loads((root/x['layout']/'summary.json').read_text()) for x in rows]),
                generalization='NOT_ESTABLISHED',online_f_g='NOT_TRAINED_NOT_TESTED',
                scope='frozen 2D adapter, endpoint diagnosis, reference population only')
    result['secondary_layout_aggregate']['unit']='layout; secondary descriptive aggregate; use verified building hierarchy for primary'
    result['primary_contrast']=primary
    result['practical_gate']=dict(delta_C_at_least=.02,delta_Q_at_least=-.005)
    result['interval_families']='95% simultaneous across planned six episodes and five contrasts within each rate or mean-effect family separately; no joint 95% claim over both families'
    result['audit_timestamp_utc']=datetime.now(timezone.utc).isoformat()
    result['audit_source_sha256']=runner.digest(__file__)
    result['run_seal_sha256']=runner.digest(root/'seal.json')
    metadata_path=Path(__file__).with_name('building_metadata.json')
    if metadata_path.exists():
        metadata=json.loads(metadata_path.read_text())
        ids={x['layout']:x['building_id'] for x in metadata['layouts']}
        for r in rows:r['building_id']=ids[r['layout']];r['building_id_verified']=True
        result['building_metadata']=metadata
        result['primary_building_aggregate']=aggregate_buildings(rows,metadata)
    runner.save(root.parent/(cfg.get('preflight_prefix','')+'verified_development_summary.json'),result);print(json.dumps(result,indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True)
    audit(json.loads(Path(p.parse_args().protocol).read_text()))

if __name__=='__main__':main()
