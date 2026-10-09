"""Scientific contract checks: complete weights, all scorers, alias keys and Q budget."""
import copy
import json
from pathlib import Path

from engine import ART,SEAL,MODEL_ID,State,Predictor,branch_metrics,replay_key
from native_adapter import (
    atomic_json,frozen_gain,native,planning_cost,sha_array,torch)
from scoring import diagnose_snapshot,score_online
import numpy as np

from lama_pred_utils import load_lama_model

checks={}
weight_audit=[]
for member in range(1,4):
    model_dir=Path(SEAL["models"][member-1]["checkpoint"]).parent.parent
    model=load_lama_model(str(model_dir),device="cpu")
    checkpoint=torch.load(str(model_dir/"models/best.ckpt"),map_location="cpu")
    weights=checkpoint["state_dict"]
    expected={k:v for k,v in model.state_dict().items() if k.startswith("generator.")}
    missing=[k for k in expected if k not in weights]
    equal=not missing and all(torch.equal(v,weights[k]) for k,v in expected.items())
    weight_audit.append(dict(member=member,generator_state_keys=len(expected),
                             missing_generator_keys=missing,loaded_generator_equals_checkpoint=equal))
    checks["G%d_complete_frozen_generator"%member]=equal and bool(expected)
    del checkpoint,weights,expected,model
    __import__("gc").collect()

np.random.seed(0)
torch.manual_seed(0)
info=SEAL["maps"][0]
with np.load(str(ART/(info["map_id"]+"_sealed.npz"))) as z:
    gt,component=z["gt"],z["component"]
state=State(gt,component,info["starts"][0]["padded_row_col"],dict(map_id=info["map_id"],start_id="TECHNICAL"))
predictor=Predictor(ART/"technical_contract_checks")
(members,mean,variance),key,_=predictor.predict(state.mapper.obs_map)
state.meta["prediction_key"]=key
state.meta["prediction_digest"]=[sha_array(x) for x in [members,mean,variance]]
online=score_online(state.mapper.obs_map,np.asarray(state.meta["pose"]),
                    planning_cost(state.mapper,"ALIGNED_SHARED"),members,mean,variance)
before=[sha_array(x) for x in [members,mean,variance,state.mapper.obs_map]]
choices=diagnose_snapshot(online,state.mapper.obs_map,np.asarray(state.meta["pose"]),
                          members,mean,variance,gt)
checks["all_five_selectors_available"]=all(choices.get(k) is not None for k in
                                          ["P0","P1","P4","M0","MAPEX_ALIGNED"])
checks["frozen_inputs_unchanged"]=before==[sha_array(x) for x in [members,mean,variance,state.mapper.obs_map]]
checks["dense_same_declared_native_poses"]=all(
    len(c["native_indices"])==len(c["path"][2::3]) for c in online["pool"])
# Direct parity for the endpoint MapEx path (different renderer from matched END).
centers=[c["goal"] for c in online["pool"]]
source=native.FrontierPlanner("mapex").score_frontiers(
    centers,np.asarray(state.meta["pose"]),[],np.zeros_like(mean),dict(
        laser_range_m=20,pixel_per_meter=10,num_laser=250),
    obs_map=state.mapper.obs_map,mean_map=mean,var_map=torch.from_numpy(variance))
checks["mapex_endpoint_source_scores_equal"]=np.allclose(
    -np.asarray(source[1]),[c["MAPEX_ALIGNED"] for c in online["pool"]],rtol=0,atol=0)
goal=choices["P0"]["goal"]
a=replay_key(state,goal)
checks["same_complete_state_goal_same_reuse_key"]=a==replay_key(state,goal.copy())
original=copy.deepcopy(state.meta)
state.meta["attempts"]+=1
checks["counter_change_breaks_alias"]=a!=replay_key(state,goal)
state.meta=copy.deepcopy(original)
state.meta["rng_numpy"][2]=(state.meta["rng_numpy"][2]+1)%624
checks["rng_change_breaks_alias"]=a!=replay_key(state,goal)
state.meta=copy.deepcopy(original)
state.meta["moves"]+=1
checks["remaining_budget_change_breaks_alias"]=a!=replay_key(state,goal)
state.meta=copy.deepcopy(original)
# Fixed 75m remaining budget and right-continuous carry-forward after one new scan.
trace=[dict(move=250,travel_m=25.,coverage=.1),
       dict(move=251,travel_m=25.1,coverage=.2)]
metric=branch_metrics(trace,250,"NO_REACHABLE_FRONTIER")
checks["Q_fixed_remaining_budget_carry_forward"]=abs(metric["Q"]-(.1*74.9/75.))<1e-12
checks["prefix_early_terminal_carry_forward"]=abs(metric["prefix_gain_10m"]-.1)<1e-12
checks["infrastructure_abort_missing_not_zero"]=branch_metrics(trace,250,"RESOURCE_ABORT")["Q"] is None
checks["collision_failure_retained"]=branch_metrics(trace,250,"COLLISION")["behavior_failure"]

result=dict(status="PASS" if all(checks.values()) else "FAIL",checks=checks,
            weight_audit=weight_audit,model_identity=MODEL_ID,
            five_selector_goals={k:None if v is None else v["goal"].tolist() for k,v in choices.items()},
            scientific_trajectories_collected=0,
            technical_fixture="initial real model snapshot + explicit analytic AUC fixture")
atomic_json(ART/"scientific_contract_checks.json",result)
print(json.dumps(result),flush=True)
if result["status"]!="PASS":
    raise RuntimeError("SCIENTIFIC_CONTRACT_CHECK_FAILED")
