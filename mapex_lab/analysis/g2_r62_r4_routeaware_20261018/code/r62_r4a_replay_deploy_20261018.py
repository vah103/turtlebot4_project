#!/usr/bin/env python3
"""DELL-only G2-R6.2 native2D profile overlay. Does not modify native2d.py.

Policy diverges from COM1 on purpose; only D01/D02 pre-goal observation,
LaMa tensors and RoomNode mode/robot should match the immutable NR-RN-04.
The original resource watchdog/once-only locks/receipts remain in use.
"""
import argparse,datetime,json,os,sys
from pathlib import Path
sys.path.insert(0,"/home/dell/DELL_NATIVE2D")
import native2d
BASE=Path("/home/dell/DELL_NATIVE2D")
SRC=BASE/"build/g2_r62_r4a_r3_replay_20261018"
RUNID="NR-G2R62R4A-02"
PROFILE="NR-G2R62R4A"
native2d.SOURCE[PROFILE]=(SRC,"native_g2_r62_worker_template.py","native_g2_r62_r4a_replay01")
# R4-specific policy modifications are deliberately pinned here, without
# relaxing old baseline asset verification for all historic DELL runs.
ORIGINAL_SRC=BASE/"build/g2_r62_general_structural_r2_20261018"
ORIGINAL_ASSERT_ASSETS=native2d.assert_assets
R4_EXPECTED_HASHES={
    'room_native_integration.py':'49a6ec84ccb8f59b077344933bbd6bcce9a2c39c6015d0838c91feabf7a3dadf',
    'explore_g2_r62_lama3_smallroi.py':'7db4a542ba6eeeab9d68d4e7e41f49125cc5644c4f78e4cc27b0ac68279be1f3',
    'r4_exit_guard.py':'37890a0b790eafba0c677f0689d1e8b0f55aa4d6069b829161dfae4e037d066e',
    'r4_visibility_gate.py':'c3809e18fa5e2d1160bc9b3916b2a6ff94566919d7fc46d37f331af6e1b2ba07',
    'native_g2_r62_worker_template.py':'5e373b2c5e39c70b3321c1b12969c7e269a9c2b2c0edc438601535aebf511508',
    'g2_r61_sidecar.py':'dd787422e7756caa3585fea39676d0d25563fea6e176fd1f3052379949f024cf',
    'r3_frontier.py':'c308a2f3fccc333426df24d4100d61583d46d5d913730c4d59719083d562d4e3',
    'r3_plan_manager.py':'d8bbe7b761945463536d8a1edec92f9fbd3936cd66f73d9c23616809f8810a5d',
    'g2_r61_native_integration.py':'d5fc28fd3623927d7b71d4c74473af633a58c6ccb0b111492e81550d80c6a483',

}
def r4_assert_assets(source):
    baseline=ORIGINAL_ASSERT_ASSETS(ORIGINAL_SRC)
    r4={str(SRC/n):sha for n,sha in R4_EXPECTED_HASHES.items()}
    for name,expected in R4_EXPECTED_HASHES.items():
        actual=native2d.sha(SRC/name)
        if actual!=expected:
            raise RuntimeError("R4_POLICY_SHA_MISMATCH %s"%name)
    return dict(baseline,**r4)
native2d.assert_assets=r4_assert_assets



def comparator(runid):
    from parity_check import load,evaluate,room_decision
    a=BASE/"runs/NR-RN-04"/"native_opt_r41_lama3small_run04"/"R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob"
    b=BASE/"runs"/runid/"native_g2_r62_r4a_replay01"/"G2_R62_R4A_REPLAY01_visvarprob"
    results=[]
    for d in (1,2):
        try:
            ref,_=load(a,d); actual,_=load(b,d)
            v=evaluate(ref,actual,d)
            old_room=room_decision(a,d); new_room=room_decision(b,d)
            if old_room is None or new_room is None:
                # A D02 inference INPUT file is written before RoomNode
                # finishes. This is PENDING, not actual policy mismatch.
                return {"result":"PENDING_INCOMPLETE_SNAPSHOTS",
                        "reason":"WAIT_ROOM_DECISION_%02d"%d,
                        "strict_pass":False,"numerical_pass":False}
            v["room_mode_and_pose_equal"]=bool(old_room and new_room and
                all(old_room.get(k)==new_room.get(k)
                    for k in ("mode","robot_rc","native_tick","room_id")))
            results.append(v)
        except (OSError,KeyError,RuntimeError,FileNotFoundError) as exc:
            return {"result":"PENDING_INCOMPLETE_SNAPSHOTS","reason":str(exc),
                    "strict_pass":False,"numerical_pass":False}
    ok=all(x["numerical_pass"] and x["room_mode_and_pose_equal"] for x in results)
    return {"run_id":runid,"result":"D01_D02_NUMERIC_PARITY_PASS" if ok else "D01_D02_PARITY_FAIL",
            "numeric_only_pre_goal":True,"reference":"NR-RN-04, D01/D02 before G2-R62 new goal selection",
            "different_future_policy_allowed":True,
            "strict_pass":False,"numerical_pass":ok,"rows":results,
            "cannot_imply_G2R62_full_parity":True}

def main():
    p=argparse.ArgumentParser()
    p.add_argument("action",choices=["prepare","check","model-smoke","run"])
    a=p.parse_args()
    if a.action=="prepare":
        output=native2d.stage(PROFILE,RUNID)
        # Truthful native profile metadata: this IS changed policy source.
        manifest=output/"RUN_MANIFEST.json"
        md=json.loads(manifest.read_text())
        md["policy_code_modified"]=True
        md["worker_path_thread_smoke_patch_only"]=False
        md["r4_policy_pinned_hashes"]=R4_EXPECTED_HASHES
        manifest.write_text(json.dumps(md,indent=2,sort_keys=True)+"\n")
        print("STAGED",output)
        print(json.dumps(native2d.preflight(RUNID),indent=2))
        return
    proof=native2d.preflight(RUNID,require_mem=True)
    print(json.dumps(proof,indent=2),flush=True)
    if a.action=="check":return
    if a.action=="run":
        smoke=BASE/"runs"/RUNID/"MODEL_SMOKE_LATEST.json"
        if not smoke.exists() or json.loads(smoke.read_text()).get("outcome")!="MODEL_SMOKE_PASS":
            raise RuntimeError("MODEL_SMOKE_GATE_NOT_PASSED")
    import parity_check
    parity_check.compare_one=comparator
    from run_guard import guarded_run
    guarded_run(RUNID,model_smoke=(a.action=="model-smoke"))

if __name__=="__main__":
    try:main()
    except BaseException as exc:
        print("R62_DELL_DEPLOY_ERROR",type(exc).__name__,str(exc),file=sys.stderr)
        raise
