#!/usr/bin/env python3
"""G2-R3 Global Conditional Frontier Sequence native deployment.
Precommitted heuristic; same G1 LOCAL and weighted native MapEx A*.
"""
import sys,json,argparse,shutil
from pathlib import Path
sys.path.insert(0,'/home/dell/DELL_NATIVE2D')
import native2d,run_guard,parity_check
BASE=Path('/home/dell/DELL_NATIVE2D')
PROFILE='NR-G2SEQ'
RUNID='NR-G2SEQ-02'
native2d.SOURCE[PROFILE]=(native2d.PAIR,'native_opt_r41_3model_worker.py','native_opt_r41_lama3small_run04')
parity_check.SOURCES[PROFILE]=parity_check.SOURCES['NR-RN']
HELPER=BASE/'build/g2_global_order_r3.py'

def stage():
    dest=native2d.stage(PROFILE,RUNID)
    policy=dest/'room_native_integration.py'
    src=policy.read_text()
    marker='class RoomRun(object):'
    assert src.count(marker)==1
    src=src.replace(marker,'import g2_global_order_r3 as g2_r3\n\n'+marker,1)
    needle='''        self.scope_grid=plan_grid;self.active_mask=mask if mode=="LOCAL" else None'''
    inject='''        # G2-R3 GLOBAL only: LOCAL scoring/candidate/Room Node remain unchanged.
        r3_audit={"enabled":False,"reason":"LOCAL_UNMODIFIED"}
        if mode=="GLOBAL" and len(filtered)>=3:
            navcore=np.asarray(planner_grid[native_pad:native_pad+h,native_pad:native_pad+w],dtype=np.float32)
            r3_choice,r3_audit=g2_r3.select_global(
                np.asarray(robot,dtype=np.int32)-native_pad,
                np.asarray(filtered,dtype=np.int32)-native_pad,
                costs.copy(),navcore,core,meanraw,varraw,self.cell_m,
                int(pred_cfg['laser_range_m']*pred_cfg['pixel_per_meter']),1.0)
            if int(r3_choice)!=int(np.argmin(costs)):
                modified=costs.copy()
                minimum=float(np.min(costs))
                modified[r3_choice]=minimum-max(1.,abs(minimum))
                scored=list(scored);scored[1]=modified;scored=tuple(scored)
                costs=np.asarray(scored[1]).reshape(-1)
            print("G2_R3_GLOBAL",int(t),
                "old",int(r3_audit['baseline_index']),
                "new",int(r3_choice),
                "reason",r3_audit['reason'],
                "trials",int(r3_audit['trial_count']),
                "elapsed",r3_audit.get('elapsed_s'),flush=True)
        elif mode=="GLOBAL":
            r3_audit={"enabled":False,"reason":"FEWER_THAN_THREE_WFD"}
        self.scope_grid=plan_grid;self.active_mask=mask if mode=="LOCAL" else None'''
    assert src.count(needle)==1
    src=src.replace(needle,inject,1)
    log_field='''             "elapsed_lama_prediction_s":float(prediction_wall_time_s),'''
    assert src.count(log_field)==1
    src=src.replace(log_field,'''             "g2_r3_global":r3_audit,
             "elapsed_lama_prediction_s":float(prediction_wall_time_s),''',1)
    policy.write_text(src)
    shutil.copy2(str(HELPER),str(dest/HELPER.name))
    manifest=dest/'RUN_MANIFEST.json'
    payload=json.loads(manifest.read_text())
    payload.update(policy_code_modified=True,worker_path_thread_smoke_patch_only=False,
        algorithm='G2-R3 GLOBAL conditional near-before-far scheduling, same-epoch official WFD and LaMa, single native goal execution; fallback MapEx on ambiguous continuation',
        precommit_scoring='Compare current far target F to at most 6 strictly closer official WFD N with 2 distinct current WFD continuation E. On original native clearance-weighted A*: Cnear=SN+NF+FE; Cfar_proxy=SF+FE+fraction(predicted UNIQUE visible variance on NF vs baseline path)*max(0,FN+NE-FE). Switch ONLY if both E yield positive margin, added legs traversed only observed FREE, incremental unknown variance positive. GLOBAL only. No calibrated probability claim. Same-epoch ray proxy=180 rays per 50 grid cells, 20m lidar range.',
        no_future_or_gt=True,global_only=True,local_unchanged=True,environment_specific_thresholds=False,
        original_MapEx_global_frontier_fn_unchanged=True,original_MapEx_native_Astar_unchanged=True,
        shadow_source='/home/dell/DELL_NATIVE2D/analysis/g2_autonomous_research_20261018/NR_G2_R3_GLOBAL_CONDITIONAL_SHADOW.json',
        known_prelaunch_shadow='0 of 19 eligible D>0 GLOBAL decision snapshots satisfy conservative policy change condition')
    for q in [policy,dest/HELPER.name]:
        payload['staged_sha256'][q.name]=native2d.sha(q)
    manifest.write_text(json.dumps(payload,indent=2,sort_keys=True)+'\n')
    print('STAGED',dest,'HELPER_SHA',native2d.sha(dest/HELPER.name),'ROOM_SHA',native2d.sha(policy),flush=True)

def test():
    import numpy as np
    import pyastar2d
    sys.path.insert(0,str(BASE/'runs'/RUNID))
    import g2_global_order_r3 as R
    obs=np.zeros((80,100),dtype=np.float32)
    obs[10:70,65:75]=.5
    m=np.zeros_like(obs)+.1;v=np.zeros_like(obs)+.2
    nav=np.ones_like(obs,dtype=np.float32)
    pts=[[40,20],[45,20],[40,80],[33,50]]
    scores=np.array([-.1,-.3,-.7,-.15])
    x,z=R.select_global([40,4],pts,scores,nav,obs,m,v,.1,40,1.)
    assert x in range(4)
    assert z['no_future_or_gt_online'] and z['baseline_index']==2
    # Preserve baseline with no candidates, no unknown or no alternate E:
    x,z=R.select_global([40,4],pts[:2],scores[:2],nav,obs,m,v,.1,40,1.)
    assert x==1 and z['reason']=='FEWER_THAN_THREE_WFD'
    # Synthetic causal branch: N is OFF the S->F route. A near-only
    # predicted unknown pocket survives going directly to F; two distinct
    # continuation frontiers E are close to F. This must trigger N first.
    original=R._vis_one
    def mock(y,x,obs,mean,var,ray_count,reach):
        t=np.zeros(obs.shape,np.uint8)
        if y==25 and x==10:
            t[40:45,60:65]=1
        return t
    try:
        R._vis_one=mock
        fakeobs=np.zeros((80,100),np.float32)
        fakeobs[40:45,60:65]=.5
        x,z=R.select_global([30,4],[[30,50],[25,10],[30,53],[30,51]],
                  [-2.,-1.,-.9,-.8],np.ones_like(fakeobs),
                  fakeobs,np.zeros_like(fakeobs),np.ones_like(fakeobs),.1,40,.1)
        assert x==1 and z['decision']=='CHANGE_GLOBAL_GOAL',z
    finally:
        R._vis_one=original
    from pathlib import Path
    integration=(BASE/'runs'/RUNID/'room_native_integration.py').read_text()
    assert 'if mode=="GLOBAL" and len(filtered)>=3:' in integration
    assert 'elif mode=="GLOBAL"' in integration
    assert 'if mode=="LOCAL" and' not in integration[integration.index('# G2-R3 GLOBAL only'):integration.index('self.scope_grid=plan_grid;self.active_mask=mask if mode=="LOCAL" else None',integration.index('# G2-R3 GLOBAL only'))]
    print('G2_R3_UNIT_PASS','first',x,'choice',z['reason'],flush=True)

def compare(runid):
    from parity_check import load,evaluate,room_decision
    ref=BASE/'runs/NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob'
    dst=BASE/'runs'/runid/'native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob'
    rows=[]
    for d in (1,2):
        try:
            a,_=load(ref,d);b,_=load(dst,d);res=evaluate(a,b,d)
            x=room_decision(ref,d);y=room_decision(dst,d)
            if not y:return dict(result='PENDING',strict_pass=False)
            res['room_mode_pose_equal']=all(x.get(k)==y.get(k) for k in ('mode','room_id','robot_rc','native_tick'))
            rows.append(res)
        except Exception as e:return dict(result='PENDING',reason=str(e),strict_pass=False)
    okay=all(z['numerical_pass'] and z['room_mode_pose_equal'] for z in rows)
    return dict(result='D01_D02_NUMERIC_PARITY_PASS' if okay else 'PARITY_FAIL',
                numerical_pass=okay,strict_pass=False,rows=rows)
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['prepare','test','smoke','run'])
    cmd=parser.parse_args().action
    if cmd=='prepare':
        stage();test();return
    if cmd=='test':test();return
    print(json.dumps(native2d.preflight(RUNID,require_mem=True),indent=2))
    parity_check.compare_one=compare
    run_guard.guarded_run(RUNID,model_smoke=cmd=='smoke')
if __name__=='__main__':main()
