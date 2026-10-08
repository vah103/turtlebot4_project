"""Audit raw R4 evidence and replay physical reference observations.

No new actions, sampling changes, model training, or research outcome runs.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import runner

def close(a,b):
    if isinstance(a,(float,int)) and isinstance(b,(float,int)):return abs(a-b)<1e-9
    return a==b

def audit(cfg):
    root=Path(cfg['output']);seal=json.loads((root/'seal.json').read_text())
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
            if moved:w.sense();trace.append([w.distance_m,runner.coverage(w,a)])
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
            for action in set(record['actions'].values()):
                artifact=folder/f'branch_{state:04d}_{action[:12]}.json';x=json.loads(artifact.read_text())
                context=hashlib.sha256(json.dumps(dict(seal=context_seal,layout=layout,state=state,
                            snapshot=record['observed_hash'],pose=record['pose'],distance_m=record['distance_m'],
                            action=action,RNG='deterministic-no-random-controller'),sort_keys=True).encode()).hexdigest()
                assert x['status']=='COMPLETE' and x['replay_context']==context and x['action_id']==action
                assert x['distance_m']<=cfg['budget_m']+1e-7
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
                         native_GT_action_flip=summary['action_flip_rates'][primary],
                         native_GT_primary=prim,all_contrasts=summary['estimates'],
                         baseline_collision_probe=collision_probe,
                         candidates=sum(len(r['candidates']) for r in records),
                         mean_candidate_native_false_visible_cells=float(np.mean([x['native_false_visible'] for x in mismatch])) if mismatch else None,
                         mean_candidate_native_missed_visible_cells=float(np.mean([x['native_missed_visible'] for x in mismatch])) if mismatch else None,
                         physical_reference_replay='EXACT',independent_building_verified=False))
    result=dict(status='AUDITED_COMPLETE_DEVELOPMENT_ONLY' if not missing else 'AUDITED_PARTIAL_INCONCLUSIVE',
                planned_layouts=len(cfg['layouts']),completed_layouts=len(rows),missing_layouts=missing,
                total_valid_decisions=sum(x['T'] for x in rows),total_action_flip_states=sum(x['M'] for x in rows),
                total_sampled_states=sum(x['k'] for x in rows),rows=rows,
                cohort=runner.aggregate([json.loads((root/x['layout']/'summary.json').read_text()) for x in rows]),
                generalization='NOT_ESTABLISHED',online_f_g='NOT_TRAINED_NOT_TESTED',
                scope='frozen 2D adapter, endpoint diagnosis, reference population only')
    runner.save(root.parent/'verified_development_summary.json',result);print(json.dumps(result,indent=2))

def main():
    p=argparse.ArgumentParser();p.add_argument('--protocol',required=True)
    audit(json.loads(Path(p.parse_args().protocol).read_text()))

if __name__=='__main__':main()
