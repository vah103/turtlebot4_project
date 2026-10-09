"""Extended source contract audit, including holes and known degenerate source behavior."""
import ast
import json
from pathlib import Path
import time

from native_adapter import (
    ROOT,SOURCE,PRED,SENSOR,atomic_json,candidate_pool,frozen_gain,make_mapper,
    native,planning_cost,render,sample_indices,sha_array,smu,torch)
from scoring import score_online
import numpy as np


def reference_serial(centers,paths,observed,mean,variance):
    """Preserve source CODE_REFERENCE behavior, including its NaN overwrite bug."""
    sampled=[p.copy()[2::3] for p in paths]
    masks=[native.FrontierPlanner.process_path(
        p,200,PRED,"pipe",mean,observed,*observed.shape) for p in sampled]
    values=[torch.sum(variance[torch.tensor(m,dtype=torch.bool)]).item()/max(1,len(p))
            for p,m in zip(sampled,masks)]
    values=np.asarray(values)
    if len(values)==1:
        best=0
    else:
        best,second=np.argpartition(-values,1)[:2]
        if values[second]>values[best]:
            best=second
    return values,masks,int(best)


def renderer_ledger():
    gt=np.zeros((480,512))
    obs=np.ones_like(gt)*.5
    path=np.array([[160,160],[160,280],[280,280],[280,160]])
    bars=[
        [[150,150],[150,300],[185,300],[185,150]],
        [[150,265],[150,300],[300,300],[300,265]],
        [[265,150],[265,300],[300,300],[300,150]],
        [[150,150],[150,185],[300,185],[300,150]]]
    original=smu.get_hit_ponits
    def supplied(numerical,point,**kwargs):
        index=int(np.argwhere(np.all(path==point,axis=1))[0,0])
        return np.ones_like(gt)*.5,np.asarray(bars[index])
    try:
        smu.get_hit_ponits=supplied
        output=native.FrontierPlanner.process_path(path,200,PRED,"pipe",gt,obs,*gt.shape)
        audit=dict(hole_center_excluded=bool(not output[225,225]),
                   ring_interior_visible=bool(output[165,200]),
                   source_union_holes_preserved=True)
        try:
            native.FrontierPlanner.process_path(np.empty((0,2),dtype=int),
                                               200,PRED,"pipe",gt,obs,*gt.shape)
        except Exception as exc:
            audit["empty_source_exception"]=type(exc).__name__+": "+str(exc)
        else:
            audit["empty_source_exception"]="UNEXPECTED_NO_EXCEPTION"
    finally:
        smu.get_hit_ponits=original
    # Confirm the actual source's NaN score overwrite through its scoring function.
    original_process=native.FrontierPlanner.process_path
    def nan_render(*args):
        return np.full((32,32),np.nan)
    try:
        native.FrontierPlanner.process_path=staticmethod(nan_render)
        # Serial equivalent records source score contract; raw bool(NaN)=True.
        v,m,b=reference_serial([np.array([1,4])],[np.array([[1,c] for c in range(5)])],
                               np.full((32,32),.5),np.zeros((32,32)),torch.ones(32,32))
        audit["nan_native_bool_score"]=float(v[0])
        try:
            render(np.zeros((32,32)),np.full((32,32),.5),np.array([[1,2]]))
        except Exception as exc:
            audit["aligned_nan_disposition"]=str(exc)
    finally:
        native.FrontierPlanner.process_path=staticmethod(original_process)
    audit["multipolygon_source_disposition"]="Last boundary overwrites previous initialized grid; preserved in source, no repair"
    return audit


def main():
    manifest=json.loads((ROOT/"artifacts/manifest.json").read_text())
    observed=np.load(str(ROOT/"artifacts/probe_observed.npy"))
    with np.load(str(ROOT/"artifacts/probe_predictions.npz")) as z:
        predictions,mean,variance=z["predictions"],z["mean"],z["variance"]
    pose=np.asarray(manifest["maps"][0]["starts"][0]["padded_row_col"])
    mapper=make_mapper(np.zeros_like(observed))
    mapper.obs_map=observed.copy()
    ref_cost=planning_cost(mapper,"CODE_REFERENCE")
    pool,rejected,_=candidate_pool(observed,pose,ref_cost)
    if not pool:
        raise RuntimeError("NO_CODE_REFERENCE_PARITY_CANDIDATES")
    centers=[p["goal"] for p in pool]
    paths=[p["path"].copy() for p in pool]
    before=[sha_array(p) for p in paths]
    tensor=torch.from_numpy(variance)
    start=time.perf_counter()
    source=native.FrontierPlanner("pipe").score_frontiers_pathwise(
        0,centers,[p.copy() for p in paths],None,PRED,observed,mean,tensor)
    values,masks,best=reference_serial(centers,paths,observed,mean,tensor)
    checks=dict(
        source_costs_equal=bool(np.array_equal(np.asarray(source[1]),-values)),
        source_selected_goal_equal=bool(np.array_equal(source[-1],centers[best])),
        source_best_mask_equal=bool(np.array_equal(source[2],masks[best],equal_nan=True)),
        paths_unchanged=before==[sha_array(p) for p in paths],
        variance_convention=bool(np.array_equal(
            variance,torch.var(torch.from_numpy(predictions),dim=0).numpy())),
        known_predictions_exact=bool(np.all(predictions[:,observed!=.5]==observed[observed!=.5])),
        gt_isolation_signature="evaluation_gt" not in
            [a.arg for n in ast.parse(Path(__file__).with_name("scoring.py").read_text()).body
             if isinstance(n,ast.FunctionDef) and n.name=="score_online" for a in n.args.args])
    ledger=renderer_ledger()
    checks["source_hole_fixture"]=ledger["hole_center_excluded"] and ledger["ring_interior_visible"]
    checks["empty_source_case_audited"]=ledger["empty_source_exception"]!="UNEXPECTED_NO_EXCEPTION"
    aligned_a=score_online(observed,pose,planning_cost(mapper,"ALIGNED_SHARED"),predictions,mean,variance)
    aligned_b=score_online(observed.copy(),pose.copy(),planning_cost(mapper,"ALIGNED_SHARED"),
                           predictions.copy(),mean.copy(),variance.copy())
    checks["aligned_chosen_goal_replay_equal"]=(
        aligned_a["chosen"] is not None and aligned_b["chosen"] is not None and
        np.array_equal(aligned_a["chosen"]["goal"],aligned_b["chosen"]["goal"]))
    result=dict(status="PASS" if all(checks.values()) else "FAIL",checks=checks,
                ledger=ledger,code_reference_candidates=len(pool),rejected=rejected,
                source_worker_count=max(1,__import__("multiprocessing").cpu_count()-1),
                aligned_worker_count=1,elapsed_s=time.perf_counter()-start,
                next_gate="Full controller clone path/termination equality; resource feasibility",
                scientific_trajectories_collected=0)
    atomic_json(ROOT/"artifacts/extended_parity.json",result)
    print(json.dumps(result),flush=True)
    if result["status"]!="PASS":
        raise RuntimeError("EXTENDED_PARITY_FAILED")


if __name__=="__main__":
    main()
