"""D02 causal snapshot + synthetic follow-up test. NOT a closed-loop run."""
from pathlib import Path
import json
import time
import numpy as np

from trial_d02 import make
from r61_plan_manager import R61PlanManager

root=Path(__file__).resolve().parent
p,obs,nav,sem=make(2)
base=p.plan()
m=R61PlanManager(p,1,12)
start=time.perf_counter()
stable=m.on_scan(obs.copy(),(147,111),13,nav)
elapsed=time.perf_counter()-start
arrive=m.checkpoint(obs.copy(),tuple(base["goal_sequence_core_rc"][0]),14,nav,
                    source_goal=tuple(base["goal_sequence_core_rc"][0]))
# Synthetic counterfactual after D02: local new wall contradicting old LaMa.
p2,obs2,nav2,sem2=make(2)
p2.plan()
mm=R61PlanManager(p2,1,12)
new=obs2.copy()
pts=np.argwhere((obs2==1)&p2.room&(p2.mean<.25))
for y,x in pts[:40]:
    new[y,x]=2
changed=mm.on_scan(new,(147,111),13,nav2)
hold=mm.checkpoint(new,tuple(p2.plan_order[0]),13,nav2,
                   source_goal=tuple(p2.plan_order[0]))
result={
 "dataset":"New Room D02 frozen Run11 observed, same initial epoch as NR-RN-01",
 "warning":"all later observations here are synthetic counterfactuals, not G2-R6.1 actual robot LiDAR",
 "source_observed_sha256":base["input_observed_sha256"],
 "initial_viewpoints":base["goal_sequence_core_rc"],
 "initial_route_nominal_m":base["predicted_route_distance_m"],
 "actual_d02_newroom_scan_after_first_goal_available":False,
 "nochange_scan":{"event":stable["event"],"wall_seconds":round(elapsed,5)},
 "synthetic_arrival_with_same_observed_grid":{
    "action":arrive["action"],
    "source":arrive.get("source"),
    "new_lookahead":arrive.get("goal_rc"),
    "old_goals_not_blindly_acked":arrive.get("canceled_old_future_chain",False)},
 "synthetic_40_new_walls_predicted_free":{
    "event":changed["event"],"reason":changed["reason"],
    "first_goal_checkpoint_action":hold["action"],
    "future_goal_ack_count":len(mm.consumed)},
 "used_gt":False,
 "full_closed_loop_native_run":False
}
(root/"R61_D02_CAUSAL_AND_SYNTHETIC_PROBE.json").write_text(
  json.dumps(result,ensure_ascii=False,indent=2)+"\n")
print(json.dumps(result,ensure_ascii=False,indent=2))
