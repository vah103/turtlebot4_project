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
SRC=BASE/"build/g2_r62_general_structural_r2_20261018"
RUNID="NR-G2R62-03"
PROFILE="NR-G2R62"
native2d.SOURCE[PROFILE]=(SRC,"native_g2_r62_worker_template.py","native_g2_r62_general_r2_run01")

def comparator(runid):
    from parity_check import load,evaluate,room_decision
    a=BASE/"runs/NR-RN-04"/"native_opt_r41_lama3small_run04"/"R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob"
    b=BASE/"runs"/runid/"native_g2_r62_general_r2_run01"/"G2_R62_GENERAL_R2_RUN01_visvarprob"
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