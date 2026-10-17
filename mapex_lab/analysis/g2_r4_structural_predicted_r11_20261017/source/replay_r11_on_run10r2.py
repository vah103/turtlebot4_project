#!/usr/bin/env python3
"""Offline same-snapshot selector replay of R11 on frozen Run10-R2 observations.

NO simulator/robot motion. NO edits to frozen run. This replay is off-policy:
R11 goals differ from the goals that produced the saved later observations.
Therefore it is NOT causal evidence of exploration coverage or path savings.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from g2_r4_policy_sidecar import Service

def replay(src: Path, out: Path, selected=None):
    if out.exists():raise FileExistsError(out)
    p=src/"native_g2_r4_frontier_exit_run10r2"/"G2_R4_FRONTIER_EXIT_RUN10R2_visvarprob"
    root=p/"room_r41"
    files=sorted(root.glob("decisions/decision_[0-9]*.json"))
    files=[f for f in files if "global_fallback" not in f.name]
    if not files:raise FileNotFoundError("Run10-R2 frozen decision logs missing")
    service=Service()
    rows=[]
    for file in files:
        d=json.loads(file.read_text())
        did=int(d["decision"]);tick=int(d["native_tick"])
        if d["mode"]!="LOCAL" or (selected is not None and did not in selected):
            continue
        inp=root/"inference"/f"decision_{did:06d}_input.npz"
        sem=root/"inference"/f"decision_{did:06d}_semantic.npz"
        nav=p/"g2_r4"/f"native_scope_{did:06d}.npz"
        old=p/"g2_r4"/"decisions"/f"decision_{did:06d}.json"
        for f in [inp,sem,nav,old]:
            if not f.exists():raise FileNotFoundError(f)
        robot=np.array(d["robot_rc"],dtype=int)-500
        with np.load(inp,allow_pickle=False) as z:
            observed=z["observed"]
        with np.load(nav,allow_pickle=False) as z:
            safe=z["reachable_knownfree"].astype(bool)
        if observed[tuple(robot)] != 0 or not safe[tuple(robot)]:
            raise AssertionError(f"D{did}: robot not on native knownfree")
        req={"action":"CHOOSE","native_tick":tick,"room_id":int(d["room_id"]),
             "robot_core_rc":robot.astype(int).tolist(),
             "room_node_input":str(inp),"room_node_semantic":str(sem),
             "native_scope_npz":str(nav)}
        new=service.choose(req)
        old_dec=json.loads(old.read_text())
        current=new["goal_core_rc"]
        if current is not None:
            target=tuple(current)
            assert observed[target]==0 and safe[target],f"D{did}: unsafe goal"
        arb=new["R11_structural_arbitration"]
        row={
          "decision":did,"native_tick":tick,"room_id":int(d["room_id"]),
          "RoomNode_LOCAL_frontiers":int(d["local_candidate_count"]),
          "old_Run10_R4_goal":old_dec.get("R4_knownfree_baseline_goal_core_rc"),
          "old_Run10_executed_goal":old_dec.get("goal_core_rc"),
          "new_R11_original_R4_goal":new["R11_original_R4_knownfree_goal_core_rc"],
          "new_R11_final_executable_goal":current,
          "new_R11_predicted_target":new["predicted_target_core_rc"],
          "arbitration":arb,"evaluated_candidates":new["R11_candidate_diagnostics"],
          "backstop":new["run10_local_backstop"],
          "reposition_only":new["R11_reposition_only"],
          "no_wfd_used_as_goal":not new["run10_frontier_used_for_goal"],
          "snapshots_from_other_policy":True,
          "source_log_g2":str(old),
        }
        rows.append(row)
        print("D%02d t=%4d R10=%s R11=%s spec=%s %s" %(
            did,tick,row["old_Run10_executed_goal"],current,
            row["new_R11_predicted_target"],
            arb["reason"] if arb else "no predicted arbitration"),flush=True)
    result={
        "schema":"R11_SNAPSHOT_REPLAY_OFF_POLICY_V1",
        "source_frozen_run10r2":str(src),
        "has_ground_truth_or_future_state_in_input":False,
        "native_run_executed":False,
        "closed_loop_claim":False,
        "same_snapshot_wfd_only_exit_gate":True,
        "selected_decisions":sorted(selected) if selected is not None else "ALL_LOCAL",
        "local_decisions":len(rows),
        "R11_predicted_choices":sum(bool(x["new_R11_predicted_target"]) for x in rows),
        "R11_kept_original_R4":sum(x["arbitration"] is not None and
             x["arbitration"]["decision"]=="KEEP_R4_GOAL" for x in rows),
        "R11_reposition_only":sum(bool(x["reposition_only"]) for x in rows),
        "WFD_goal_violation":sum(not x["no_wfd_used_as_goal"] for x in rows),
        "results":rows,
    }
    if result["WFD_goal_violation"]>0:raise AssertionError("WFD as LOCAL goal")
    out.parent.mkdir(parents=True,exist_ok=True)
    tmp=out.with_suffix(out.suffix+".tmp")
    tmp.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    tmp.replace(out)
    print("WROTE",out,"COUNT",len(rows),
          "NEW_PRED",result["R11_predicted_choices"],
          "KEEP_R4",result["R11_kept_original_R4"],
          "REPOSITION_ONLY",result["R11_reposition_only"])
    return result

if __name__=="__main__":
    ap=argparse.ArgumentParser()
    ap.add_argument("--frozen-run",type=Path,required=True)
    ap.add_argument("--out",type=Path,required=True)
    ap.add_argument("--decisions",nargs="*",type=int,
                    help="Only for a limited diagnostic; omit for full chronological replay")
    a=ap.parse_args()
    replay(a.frozen_run.resolve(),a.out.resolve(),
           set(a.decisions) if a.decisions else None)
