#!/usr/bin/env python3
"""Deploy G2-early-GLOBAL dynamic observed-door experiment on native New Room.
Derived from the immutable G2-SWEEP-R1 LOCAL: replaces GLOBAL selection ONLY
on the transition LOCAL -> GLOBAL after latest on-LiDAR door-aware reservation.
"""
import sys,argparse,json,shutil
from pathlib import Path
sys.path.insert(0,'/home/dell/DELL_NATIVE2D')
import native2d,run_guard,parity_check
BASE=Path('/home/dell/DELL_NATIVE2D')
PROFILE='NR-G2EARLYD'
RUNID='NR-G2EARLYD-02'
native2d.SOURCE[PROFILE]=(native2d.PAIR,'native_opt_r41_3model_worker.py','native_opt_r41_lama3small_run04')
parity_check.SOURCES[PROFILE]=parity_check.SOURCES['NR-RN']
HELPER=BASE/'build/g2_early_global_door_r1.py'
SWEEP=BASE/'runs/NR-G2SWEEP-02'
def stage():
    dst=native2d.stage(PROFILE,RUNID)
    # Stage the EXACT SAME LOCAL controller as frozen SWEEP-R1.
    parent=SWEEP/'room_native_integration.py'
    src=parent.read_text()
    assert 'G2_SWEEP_FIRST' in src and 'g2_room_sweep_r1' in src
    shutil.copy2(str(SWEEP/'g2_room_sweep_r1.py'),str(dst/'g2_room_sweep_r1.py'))
    src=src.replace('import g2_room_sweep_r1 as g2_sweep',
          'import g2_room_sweep_r1 as g2_sweep\nimport g2_early_global_door_r1 as g2_early',1)
    marker='''        self.g2_sweep_state=None\n        self.prior_mask'''
    assert src.count(marker)==1
    src=src.replace(marker,'''        self.g2_sweep_state=None
        self.early_global_reservation=None
        self.prior_mask''',1)
    marker='''        sweep_audit={"enabled":False,"reason":"GLOBAL_UNCHANGED"}
        if mode=="LOCAL":'''
    assert src.count(marker)==1
    src=src.replace(marker,'''        sweep_audit={"enabled":False,"reason":"GLOBAL_UNCHANGED"}
        global_precommit_audit={"reason":"NO_ACTIVE_LOCAL"}
        if mode=="LOCAL":
            # Independently run ORIGINAL GLOBAL MapEx scorer on current GLOBAL
            # WFD each LOCAL epoch: same-epoch LaMa, no GT or room-specific input.
            global_score=original_scorer(pts,robot,pose_list,native_pred,pred_cfg,
                             obs_map=padded_obs,mean_map=mean,var_map=var)
            global_costs=np.asarray(global_score[1]).reshape(-1)
            old=self.early_global_reservation
            if old is not None and old["room_id"]!=int(room_id):
                old=None
            self.early_global_reservation,global_precommit_audit=g2_early.choose_early(
                room_id,t,robot,pts,global_costs,mask,obs,planner_grid,
                native_pad,previous=old)
            print("G2_EARLY_DOOR_UPDATE",int(t),"room",int(room_id),
                  "reserved",self.early_global_reservation.get("goal_rc"),
                  "portal",self.early_global_reservation.get("portal_rc"),
                  "changed",global_precommit_audit["next_global_goal_changed"],
                  "newdoor",global_precommit_audit["new_or_changed_observed_portal"],
                  flush=True)
''',1)
    marker='''        else:
            self.g2_sweep_state=None
        self.scope_grid=plan_grid'''
    assert src.count(marker)==1
    src=src.replace(marker,'''        else:
            # Recompute normally if there is no valid current official WFD.
            # Only LOCAL->GLOBAL transition gets the stored early goal.
            if self.last_mode=="LOCAL":
                selected,global_precommit_audit=g2_early.release_on_global(
                    self.early_global_reservation,pts,costs,robot,planner_grid,
                    arrival_m=1.0,cell_m=self.cell_m)
                original_index=int(np.argmin(costs))
                if selected!=original_index:
                    revised=costs.copy()
                    revised[selected]=float(np.min(costs)-max(1.,abs(float(np.min(costs)))))
                    scored=list(scored);scored[1]=revised;scored=tuple(scored)
                    costs=np.asarray(scored[1]).reshape(-1)
                print("G2_EARLY_GLOBAL_RELEASE",int(t),
                      "original",original_index,"early",int(selected),
                      "reason",global_precommit_audit["reason"],flush=True)
            self.early_global_reservation=None
            self.g2_sweep_state=None
        self.scope_grid=plan_grid''',1)
    marker='''             "g2_room_sweep":sweep_audit,
             "elapsed_lama_prediction_s"'''
    assert src.count(marker)==1
    src=src.replace(marker,'''             "g2_room_sweep":sweep_audit,
             "g2_early_global_door":global_precommit_audit,
             "elapsed_lama_prediction_s"''',1)
    policy=dst/'room_native_integration.py'
    policy.write_text(src)
    shutil.copy2(str(HELPER),str(dst/HELPER.name))
    manifest=dst/'RUN_MANIFEST.json'
    x=json.loads(manifest.read_text())
    x.update(policy_code_modified=True,worker_path_thread_smoke_patch_only=False,
             experiment='G2 Early Next Global Reservation with Dynamic Observed Door Updates',
             design='Original MapEx+G1+SWEEP LOCAL unchanged. At each LOCAL epoch, score native GLOBAL official WFD with original visvarprob. Detect current G1 room 4-neighbor passable observed-FREE exterior portals. For each outside-room original WFD and each viable exit portal, reuse raw IG= -saved MapEx cost*current Euclidean(robot,goal), rank from exit portal by IG/Euclidean(portal,goal), check native A* does not reenter current room. Store current best and reassess every LOCAL LiDAR epoch including when new door appears. On first LOCAL->GLOBAL transition, if stored point still matches a current official reachable WFD within 1m, choose it; else original fresh GLOBAL MapEx. No forced obsolete goal. Original LOCAL PCA sweep and room exhaustion unchanged.',
             live_door_updates=True,source_local_run='NR-G2SWEEP-02',
             candidate_pool='Same epoch original GLOBAL MapEx WFD only',
             no_GT_no_hindsight_no_fixed_room_coords=True,
             global_after_LOCAL='Early door-anchored reservation if current official WFD remains valid; otherwise original MapEx',
             only_one_goal_committed_after_local=True)
    for q in (policy,dst/HELPER.name,dst/'g2_room_sweep_r1.py'):
        x['staged_sha256'][q.name]=native2d.sha(q)
    manifest.write_text(json.dumps(x,indent=2,sort_keys=True)+'\n')
    print('STAGED',dst,'roompolicy',native2d.sha(policy),
          'newhelper',native2d.sha(dst/HELPER.name),flush=True)
def test():
    import numpy as np
    sys.path.insert(0,str(BASE/'runs'/RUNID))
    import g2_early_global_door_r1 as s
    obs=np.ones((50,80),np.uint8)
    mask=np.zeros((50,80),bool)
    mask[10:40,30:50]=True
    obs[mask]=0
    obs[20:24,27:30]=0
    nav=np.ones(obs.shape,np.float32)
    goals=np.array([[21,7],[21,72]],np.int32)
    scores=np.array([-.60,-.80])
    old,a=s.choose_early(1,12,[25,35],goals,scores,mask,obs,nav,0,None)
    assert old['goal_rc']==[21,7] and a['n_portals']==1,(old,a)
    # New LiDAR reveals NEW passable boundary on the right of G1 room.
    # With right exterior WFD having higher IG than left (via scoring), the
    # closer virtual portal should update the NEXT GLOBAL goal.
    obs2=obs.copy();obs2[22:25,50:53]=0
    new,b=s.choose_early(1,14,[25,35],goals,scores,mask,obs2,nav,0,old)
    assert b['n_portals']==2 and b['new_or_changed_observed_portal'],b
    assert new['goal_rc']==[21,72],new
    res,diagnosis=s.release_on_global(new,goals,scores,[25,45],nav,arrival_m=1.,cell_m=.1)
    assert res==1 and diagnosis['reason']=='VALID_EARLY_GLOBAL_GOAL',diagnosis
    res,diagnosis=s.release_on_global(new,goals[:1],scores[:1],[25,45],nav)
    assert res==0 and diagnosis['reason']=='GOAL_NO_LONGER_CURRENT_WFD',diagnosis
    assert 'g2_early_global_door' in (BASE/'runs'/RUNID/'room_native_integration.py').read_text()
    print('EARLY_DOOR_UNIT_PASS 4 cases',flush=True)
def compare(runid):
    from parity_check import load,evaluate,room_decision
    ref=BASE/'runs/NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob'
    curr=BASE/'runs'/runid/'native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob'
    res=[]
    for idx in (1,2):
        try:
            a,_=load(ref,idx);b,_=load(curr,idx)
            z=evaluate(a,b,idx);da=room_decision(ref,idx);db=room_decision(curr,idx)
            if db is None:return dict(result='PENDING',strict_pass=False)
            z['scope_parity']=all(da.get(k)==db.get(k) for k in ('mode','room_id','robot_rc','native_tick'))
            res.append(z)
        except Exception as e:
            return dict(result='PENDING',strict_pass=False,reason=str(e))
    ok=all(x['numerical_pass'] and x['scope_parity'] for x in res)
    return dict(result='D01_D02_NUMERIC_PARITY_PASS' if ok else 'PARITY_FAIL',strict_pass=False,
                numerical_pass=ok,rows=res)
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('action',choices=['prepare','test','smoke','run'])
    cmd=ap.parse_args().action
    if cmd=='prepare':stage();test();return
    if cmd=='test':test();return
    print(json.dumps(native2d.preflight(RUNID,require_mem=True),indent=2),flush=True)
    parity_check.compare_one=compare
    run_guard.guarded_run(RUNID,model_smoke=cmd=='smoke')
if __name__=='__main__':main()
