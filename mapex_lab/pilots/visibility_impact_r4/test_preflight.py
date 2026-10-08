"""Engineering fixtures only. None of these are scientific impact results."""
import argparse
from itertools import combinations
import json
from pathlib import Path
import tempfile
import numpy as np
from scipy.stats import hypergeom
from estimation import estimate,population_rate_interval,sample_flips
import runner

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--protocol',required=True)
    cfg=json.loads(Path(parser.parse_args().protocol).read_text())
    core,_,fast=runner.dependencies(cfg);checks=[]
    for M in range(1,11):
        for k in range(1,M+1):
            ci=[population_rate_interval(M,k,x) for x in range(k+1)]
            for H in range(M+1):
                p=sum(hypergeom.pmf(x,M,H,k) for x,(lo,hi) in enumerate(ci) if lo-1e-12<=H/M<=hi+1e-12)
                assert p >= .95-1e-10
    checks.append('hypergeometric coverage >=95% for every M<=10, sample size, true population count')
    assert population_rate_interval(100,6,0)[1]>0
    pop=np.array([.03,0.,.05,-.03]);rates=[];means=[]
    for ids in combinations(range(4),2):
        e=estimate(10,4,pop[list(ids)],np.zeros(2));rates.append(e['rate']);means.append(e['mean_effect'])
    assert abs(np.mean(rates)-.2)<1e-12
    assert abs(np.mean(means)-pop.sum()/10)<1e-12
    e=estimate(10,0,[],[]);assert e['rate']==0 and e['rate_interval']==[0,0]
    assert estimate(0,0,[],[])['rate'] is None
    try:estimate(10,4,[],[])
    except ValueError:pass
    else:raise AssertionError('Incomplete sample silently accepted')
    actions={0:['a','a'],1:['a','b'],2:['c','c'],3:['x','y']}
    s=sample_flips(actions,1,123);assert s['T']==4 and s['M']==2 and s['inclusion_probability']==.5
    checks.append('SRS estimator unbiased by exhaustive samples; zero-flip and incomplete cases handled')
    rng=np.random.default_rng(79)
    for _ in range(20):
        free=rng.random((35,50))>.25;start=(12,20);free[start]=True
        d,p=core.shortest_paths(free,start);df,pf=fast.shortest_paths(free,start)
        assert np.array_equal(d,df) and np.array_equal(p,pf)
    checks.append('accelerated BFS distances and every tied parent equal original FIFO BFS')
    with np.load(Path(cfg['assets'])/'kth_50052748.npz') as z:
        occupied=z['occupied'];domain=z['domain'];start=tuple(z['start'])
    sensor=runner.Sensor(cfg)
    for pose in [start,(50,100),(100,200),(180,300)]:
        if not core.clearance_free(~occupied,1.5)[pose]:continue
        world=core.GridWorld(occupied,.1,pose,20,2500,.15)
        visible=world.sense();assert np.array_equal(visible,sensor.visible(occupied,pose))
    checks.append('physical sensor and diagnostic first-hit renderer identical on real layout poses')
    # Execute a complete 150m path with known coverage: check integration and clone.
    truth=np.zeros((7,1510),dtype=bool);truth[[0,-1],:]=True;truth[:,[0,-1]]=True
    asset=dict(occupied=truth,domain=np.ones_like(truth),start=np.array([3,3]),layout='ENGINEERING_FULL_BUDGET')
    small=dict(cfg,sensor_range_m=1.,sensor_rays=50)
    observed=truth.astype(np.float32);trace=[[0.,1.]]
    w=core.GridWorld(truth,.1,(3,3),1,50,.15,observed)
    path=[(3,c) for c in range(3,1504)]
    assert runner.execute(w,path,asset,small,trace)
    class Fake:
        def __init__(self):self.inner=self;self.inference_s=0.;self.identity='ENGINEERING_ONLY';self.calls=0
        def predict(self,o):
            self.calls+=1;known=o!=.5
            ps=np.stack([np.zeros_like(o),np.zeros_like(o),np.ones_like(o)])
            ps[:,known]=o[known];m=ps.mean(axis=0);v=ps.var(axis=0,ddof=1)
            return ps,m,v
    result=runner.outcome(w,asset,small,Fake(),trace,'BUDGET_REACHED')
    assert abs(w.distance_m-150)<1e-9 and abs(result['C_bar']-1)<1e-10 and result['Q']==1
    checks.append('full 150m execution and coverage integration; strict IoU and known preservation')
    # Nontrivial closed-loop fixture: changed observations, candidates, branches.
    truth=np.zeros((80,140),dtype=bool);truth[[0,-1],:]=True;truth[:,[0,-1]]=True
    truth[:,70]=True;truth[25:55,70]=False
    truth[10:25,40]=True;truth[55:70,100]=True
    a=dict(occupied=truth,domain=np.ones_like(truth),start=np.array([40,20]),layout='ENGINEERING_REPLAY')
    fixture=dict(cfg,budget_m=20.,sensor_range_m=3.,sample_cap=2)
    engine=runner.Engine(core,fast,Fake(),fixture)
    probe=runner.world_from(core,a,fixture);_,mean,var=engine.predictor.predict(probe.observed)
    arrays=[probe.observed,mean,var,a['occupied']]
    before=[core.array_hash(x) for x in arrays]
    for x in arrays:x.setflags(write=False)
    engine.score(probe,mean,var,engine.candidates(probe),a['occupied'],True)
    assert before==[core.array_hash(x) for x in arrays]
    checks.append('scoring eight arms does not mutate observed map, P, U or truth')
    with tempfile.TemporaryDirectory() as tmp:
        root=Path(tmp);runner.save(root/'seal.json',dict(status='ENGINEERING_ONLY'))
        folder=root/a['layout'];ref,records=runner.reference(engine,a,folder)
        # Force an alternate first path in this fixture, so replay is tested
        # even when the numerical scorers naturally agree. This is not data.
        assert len(records[0]['candidates'])>=2
        alternate=next(i for i in range(len(records[0]['candidates'])) if i!=records[0]['chosen'][runner.BASE])
        records[0]['chosen']['native_GT_U']=alternate
        records[0]['actions']['native_GT_U']=records[0]['candidates'][alternate]['action_id']
        answer=runner.branches(engine,a,folder,ref,records)
        assert len(records)>0 and answer['status']=='COMPLETE_DEVELOPMENT_ONLY'
        assert len(answer['effects'])==len(runner.CONTRASTS)
        assert all(x['replay_parity'] for x in answer['effects'])
        assert answer['sampling']['T']==len(records)
        assert answer['sampling']['M']==1 and answer['sampling']['selected']==[0]
        again=runner.branches(engine,a,folder,ref,records)
        assert again['effects']==answer['effects']
    checks.append('closed-loop reference census and paired suffixes replay exactly on deterministic fixture')
    output=dict(status='ENGINEERING_PREFLIGHT_PASSED',impact_results=False,checks=checks)
    runner.save(Path(cfg['output']).parent/'engineering_preflight.json',output)
    print(json.dumps(output))

if __name__=='__main__':main()
