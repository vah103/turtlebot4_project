"""Real model, exact clone, actual sensor/control and termination replay technical gate."""
import json
from pathlib import Path
from engine import ART,SEAL,State,Predictor,execute
from native_adapter import atomic_json,planning_cost,sha_array,torch
from scoring import score_online
import numpy as np

np.random.seed(0)
torch.manual_seed(0)
m=SEAL["maps"][0]
with np.load(str(ART/(m["map_id"]+"_sealed.npz"))) as z:
    gt,component=z["gt"],z["component"]
state=State(gt,component,m["starts"][0]["padded_row_col"],dict(map_id=m["map_id"],start_id="PREFLIGHT"))
predictor=Predictor(ART/"preflight_controller")
(members,mean,variance),key,_=predictor.predict(state.mapper.obs_map)
state.meta["prediction_key"]=key
state.meta["prediction_digest"]=[sha_array(x) for x in [members,mean,variance]]
state.meta["release_reason"]="NO_ACTIVE_GOAL"
cost=planning_cost(state.mapper,"ALIGNED_SHARED")
a=score_online(state.mapper.obs_map,np.asarray(state.meta["pose"]),cost,members,mean,variance)
if a["chosen"] is None: raise RuntimeError("NO_REAL_MODEL_PREFLIGHT_GOAL")
goal=a["chosen"]["goal"]
path=a["chosen"]["path"]
limit=max(1,min(10,int(np.linalg.norm(goal-state.meta["pose"]))-9))
clone_file=ART/"controller_clone_fixture.npz"
state.save(clone_file)
left=State.load(clone_file,gt,component)
right=State.load(clone_file,gt,component)
b=score_online(right.mapper.obs_map,np.asarray(right.meta["pose"]),planning_cost(right.mapper,"ALIGNED_SHARED"),
               members.copy(),mean.copy(),variance.copy())
execute(left,predictor,first_goal=goal,move_ceiling=limit)
execute(right,predictor,move_ceiling=limit)
checks=dict(
    chosen_goal_equal=np.array_equal(goal,b["chosen"]["goal"]),
    full_astar_path_equal=np.array_equal(path,b["chosen"]["path"]),
    score_vectors_equal=[c["P0"] for c in a["pool"]]==[c["P0"] for c in b["pool"]],
    observations_equal=np.array_equal(left.mapper.obs_map,right.mapper.obs_map),
    accumulated_hits_equal=np.array_equal(left.mapper.accum_hit_points,right.mapper.accum_hit_points),
    pose_trace_equal=left.trace==right.trace,
    cost_maps_equal=np.array_equal(planning_cost(left.mapper,"ALIGNED_SHARED"),
                                   planning_cost(right.mapper,"ALIGNED_SHARED")),
    termination_equal=left.meta["terminal"]==right.meta["terminal"]=="BUDGET_EXHAUSTED",
    complete_state_equal=left.complete_hash()==right.complete_hash(),
    first_goal_intervention_equals_source_P0=left.complete_hash()==right.complete_hash(),
    expected_first_step=np.array_equal(left.trace[1]["pose"],path[1]),
    actual_moved_cells=left.meta["moves"]==limit)
result=dict(status="PASS" if all(checks.values()) else "FAIL",checks=checks,
            fixture_move_budget=limit,actual_model_members=3,scientific_trajectories_collected=0,
            technical_gate="serialized complete state + actual 4connected motion/source scan/termination")
atomic_json(ART/"controller_replay_parity.json",result)
print(json.dumps(result),flush=True)
if result["status"]!="PASS": raise RuntimeError("CLONE_REPLAY_FAILED")
