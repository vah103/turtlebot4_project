#!/usr/bin/env python3
"""Run NR-G2SWEEP-02: G2 local directional sweep on original MapEx+G1 native2D.
Frozen before native launch; GLOBAL and parent LOCAL WFD detection/exits untouched.
"""
import sys,argparse,json,shutil
from pathlib import Path
sys.path.insert(0,'/home/dell/DELL_NATIVE2D')
import native2d,run_guard,parity_check
BASE=Path('/home/dell/DELL_NATIVE2D')
PROFILE='NR-G2SWEEP'
RUNID='NR-G2SWEEP-02'
native2d.SOURCE[PROFILE]=(native2d.PAIR,'native_opt_r41_3model_worker.py','native_opt_r41_lama3small_run04')
parity_check.SOURCES[PROFILE]=parity_check.SOURCES['NR-RN']
HELPER=BASE/'build/g2_room_sweep_r1.py'
POLICY_DETAILS='Room PCA longest axis, entry chooses cheaper native-Astar path to current end-frontier; keep fixed axis/direction per G1 room and select lowest signed frontier projection ahead; after no forward frontier, single residual reverse; after reverse exhausted MapEx fallback. Recompute at every native LiDAR decision, only one goal executed. No GT, hindsight or handpicked New Room coordinate.'
def stage():
    dst=native2d.stage(PROFILE,RUNID)
    policy=dst/'room_native_integration.py'
    src=policy.read_text()
    marker='class RoomRun(object):'
    assert src.count(marker)==1
    src=src.replace(marker,'import g2_room_sweep_r1 as g2_sweep\n\n'+marker,1)
    decl='''        self.epoch=0;self.last_mode="GLOBAL";self.room_id=None;self.next_room_id=1'''
    assert src.count(decl)==1
    src=src.replace(decl,decl+'\n        self.g2_sweep_state=None',1)
    needle='''        self.scope_grid=plan_grid;self.active_mask=mask if mode=="LOCAL" else None'''
    inject='''        sweep_audit={"enabled":False,"reason":"GLOBAL_UNCHANGED"}
        if mode=="LOCAL":
            navcore=np.asarray(nav[native_pad:native_pad+h,native_pad:native_pad+w],np.float32)
            current=np.asarray(robot,dtype=np.int32)-native_pad
            targets=np.asarray(filtered,dtype=np.int32)-native_pad
            original_costs=costs.copy()
            chosen,self.g2_sweep_state,sweep_audit=g2_sweep.select_local(
                current,targets,original_costs,navcore,mask,
                prior=self.g2_sweep_state,cell_m=self.cell_m,room_id=room_id)
            sweep_audit["original_costs"]=original_costs.astype(float).tolist()
            if chosen!=int(np.argmin(original_costs)):
                revised=original_costs.copy()
                revised[chosen]=float(np.min(original_costs)-max(1.,abs(float(np.min(original_costs)))))
                scored=list(scored);scored[1]=revised;scored=tuple(scored)
                costs=np.asarray(scored[1]).reshape(-1)
            print("G2_SWEEP_FIRST",int(t),"room",int(room_id),
                  "phase",sweep_audit.get("phase"),"first",int(chosen),
                  "original",int(np.argmin(original_costs)),
                  "direction",sweep_audit.get("sign"),
                  "reason",sweep_audit.get("reason"),flush=True)
        else:
            self.g2_sweep_state=None
        self.scope_grid=plan_grid;self.active_mask=mask if mode=="LOCAL" else None'''
    assert src.count(needle)==1
    src=src.replace(needle,inject,1)
    field='''             "elapsed_lama_prediction_s":float(prediction_wall_time_s),'''
    assert src.count(field)==1
    src=src.replace(field,'''             "g2_room_sweep":sweep_audit,
             "elapsed_lama_prediction_s":float(prediction_wall_time_s),''',1)
    policy.write_text(src)
    shutil.copy2(str(HELPER),str(dst/HELPER.name))
    manifest=dst/'RUN_MANIFEST.json'
    x=json.loads(manifest.read_text())
    x.update(policy_code_modified=True,worker_path_thread_smoke_patch_only=False,
             g2_design='G2_ROOM_SWEEP_R1',g2_scope='LOCAL_ONLY',
             scientific_precommit=POLICY_DETAILS,room_G1_algorithm='unchanged Run04 V4-R4.1',
             WFD_candidates='Original native G1 local WFD, current epoch',
             exit_rule='unchanged Run04: no reachable local WFD',
             global_policy='unchanged MapEx original',
             arrival_and_native_astar='unchanged Run04',
             axis_source='CURRENT_G1_ROOM_MASK_AT_ENTRY',
             no_GT_future_or_fixed_goal_coordinates=True,
             commit_sweep_direction=True,
             max_local_reversals=1)
    for q in (policy,dst/HELPER.name):x['staged_sha256'][q.name]=native2d.sha(q)
    manifest.write_text(json.dumps(x,indent=2,sort_keys=True)+'\n')
    print('STAGED',dst,'POLICY_SHA',native2d.sha(policy),
          'HELPER_SHA',native2d.sha(dst/HELPER.name),flush=True)
def test():
    import numpy as np
    sys.path.insert(0,str(BASE/'runs'/RUNID))
    import g2_room_sweep_r1 as s
    mask=np.zeros((70,120),np.bool_);mask[8:62,7:113]=1
    nav=np.ones(mask.shape,np.float32)
    # start nearer LEFT extreme, despite original MapEx cost favoring middle.
    pts=np.array([[45,104],[45,60],[45,17],[45,43]],np.int32)
    k,state,report=s.select_local([45,22],pts,[-1.,-50.,-3.,-2.],nav,mask,None,.1,1)
    assert k==2 and report['changed_goal'] and report['entry_ends']==[2,0],report
    assert report['baseline_index']==1
    # Current input changed: new frontiers alter, but sweep keeps same axis
    pts2=np.array([[45,101],[45,24],[45,60]],np.int32)
    k,state,report=s.select_local([45,17],pts2,[-12.,-1.,-3.],nav,mask,state,.1,1)
    assert k==1 and report['reason']=='FORWARD_SWEEP',report
    # Only behind remains -> exactly ONE cleanup reverse (not premature room exit)
    pts3=np.array([[45,20],[45,15]],np.int32)
    k,state,report=s.select_local([45,101],pts3,[-1.,-2.],nav,mask,state,.1,1)
    assert report['reason']=='ONE_CLEANUP_REVERSAL' and state['phase']==1,report
    # New confirmed room gets a NEW direction, no cross-room contamination.
    k,state,report=s.select_local([45,100],pts2,[-2.,-1.,-3.],nav,mask,state,.1,3)
    assert state['room_id']==3 and state['phase']==0
    src=(BASE/'runs'/RUNID/'room_native_integration.py').read_text()
    assert 'if mode=="LOCAL":' in src and 'self.g2_sweep_state=None' in src
    print('G2_SWEEP_UNIT_PASS 4 cases',flush=True)
def compare(runid):
    from parity_check import load,evaluate,room_decision
    ref=BASE/'runs/NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob'
    new=BASE/'runs'/runid/'native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob'
    checks=[]
    for d in (1,2):
        try:
            a,_=load(ref,d);b,_=load(new,d);c=evaluate(a,b,d)
            da=room_decision(ref,d);db=room_decision(new,d)
            if db is None:return dict(result='PENDING',strict_pass=False)
            c['room_and_robot_parity']=all(da.get(k)==db.get(k) for k in
                ('mode','room_id','robot_rc','native_tick'))
            checks.append(c)
        except Exception as e:return dict(result='PENDING',strict_pass=False,reason=str(e))
    ok=all(d['numerical_pass'] and d['room_and_robot_parity'] for d in checks)
    return dict(result='D01_D02_NUMERIC_PARITY_PASS' if ok else 'PARITY_FAIL',
                numerical_pass=ok,strict_pass=False,rows=checks)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=['prepare','test','smoke','run'])
    arg=ap.parse_args().action
    if arg=='prepare':
        stage();test();return
    if arg=='test':test();return
    print(json.dumps(native2d.preflight(RUNID,require_mem=True),indent=2),flush=True)
    parity_check.compare_one=compare
    run_guard.guarded_run(RUNID,model_smoke=(arg=='smoke'))
if __name__=='__main__':main()
