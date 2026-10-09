"""Sealed assets, source parity and bounded CPU feasibility probe for MX071 R2."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import platform
import resource
import subprocess
import sys
import time

from native_adapter import (
    ROOT, SOURCE, PRED, SENSOR, atomic_json, candidate_pool, frozen_gain,
    load_map, make_mapper, native, planning_cost, predict_member, render,
    sample_indices, sha_array, sha_file, torch)
import numpy as np
from scipy.ndimage import binary_dilation, label

MAPS = ["50052750", "50010535_PLAN2", "50015847", "50037765_PLAN3"]
ART = ROOT / "artifacts"


def revision(path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"]).decode().strip()


def geodesic(component, start):
    distances = np.full(component.shape, -1, dtype=np.int32)
    distances[tuple(start)] = 0
    queue = deque([tuple(start)])
    h, w = component.shape
    while queue:
        r, c = queue.popleft()
        for nr, nc in ((r-1,c),(r,c-1),(r,c+1),(r+1,c)):
            if 0 <= nr < h and 0 <= nc < w and component[nr,nc] and distances[nr,nc] < 0:
                distances[nr,nc] = distances[r,c] + 1
                queue.append((nr,nc))
    return distances


def starts_and_component(gt, valid):
    occ = binary_dilation(gt == 1, structure=np.ones((3,3)))
    components, _ = label((gt == 0) & (valid > 0) & ~occ)
    sizes = np.bincount(components.ravel())
    sizes[0] = 0
    component = components == int(np.argmax(sizes))
    points = np.argwhere(component)
    if not len(points):
        raise ValueError("EMPTY_REACHABLE_COMPONENT")
    corner = np.min(points, axis=0)
    first = points[np.argmin(np.sum((points-corner)**2,axis=1))]
    distances = geodesic(component, first)
    eligible = points[distances[points[:,0],points[:,1]] >= 200]
    flag = "GEODESIC_SEPARATION_AT_LEAST_20M"
    if len(eligible):
        centroid = points.mean(axis=0)
        second = eligible[np.argmin(np.sum((eligible-centroid)**2,axis=1))]
    else:
        second = points[np.argmax(distances[points[:,0],points[:,1]])]
        flag = "SHORT_START_SEPARATION"
    return [first, second], component, distances[tuple(second)] / 10.0, flag


def seal():
    ART.mkdir(parents=True, exist_ok=True)
    if (ART / "manifest.json").exists():
        raise RuntimeError("MANIFEST_ALREADY_SEALED: use the existing run or a new run root")
    manifest = dict(
        schema="MX071_R2_DELL_PREFLIGHT_V1",
        design_commit="e2987a65e23df7cd04fc60c19505492d771ce801",
        design_sha256="e85156fa594fb164d02a7d521d13772eb24bfa9062ceac58469fcd9942479b8f",
        pipe_commit=revision(SOURCE), mapex_commit=revision("/home/dell/MapEx"),
        mapex_lama_commit=revision("/home/dell/MapEx/lama"),
        pyastar_commit=revision(ROOT / "pyastar_source"),
        code_base_commit=revision(Path(__file__).parents[2]),
        user_execution_authorization="2026-10-09: the thoi ban chay tren dell nhe",
        stage=1, source_trajectories=8, checkpoint_targets_m=[20,40,60,80],
        branch_targets_m=[20,60], branch_selectors=["P0","P1","P4","M0","MAPEX_ALIGNED"],
        branch_request_ceiling=80, stage_2="CLOSED",
        travel_budget_m=100, successful_4connected_move_ceiling=1000,
        stuck_attempts=50, infrastructure_retry_ceiling=1,
        sensor=SENSOR, predicted_renderer=PRED,
        resource=dict(device="cpu", cuda=False, torch_threads=2,
                      torch_interop_threads=1, scoring_workers=1,
                      model_load="SEQUENTIAL_MEMBERS_EXACT_INPUT_NO_RESIZE",
                      probe_wall_ceiling_s=1200, process_rss_ceiling_mib=4608,
                      minimum_available_memory_mib=256, minimum_disk_free_mib=512),
        cohort="MAP_ID_ONLY", training_overlap="TRAIN_OVERLAP_UNVERIFIED",
        variance="torch.var(dim=0, unbiased=True), K=3, channel=0",
        tie_break="candidate ordering from native cluster label, first maximal score",
        goal_tolerance="Euclidean strictly less than 10 cells",
        native_samples="path[2::3]", native_denominator="max(1,len(sampled_path))",
        matched_samples="path[2::3] plus endpoint if absent, deduplicated",
        matched_path_denominator="full_4connected_path_m",
        adaptations=[
            "ALIGNED_UNKNOWN_BLOCKED_V1: call inflate_map(...,unknown_as_occ=True)",
            "ALIGNED_SINGLE_CELL_CONTROLLER_V1: one 4connected cell per successful move; scan every move",
            "ALIGNED_CANONICAL_FRAME_V1: crop actual centered LaMa pad back to input map; no resize",
            "ALIGNED_SERIAL_RENDER_V1: source process_path, one actual worker",
            "ALIGNED_FAIL_CLOSED_V1: reject empty/zero paths and nonfinite/nonboolean renders; log reason",
            "ALIGNED_STABLE_TIE_V1: deterministic first maximum in native cluster ordering",
            "CPU_SEQUENTIAL_MODELS_V1: no gradient, frozen members, exact input, cost includes reloads",
            "BUILD_RUNTIME_V1: system g++ linker and process-scoped libstdc++ preload; no system edits"],
        terminal_rules=["BUDGET_EXHAUSTED","NO_FRONTIER","NO_REACHABLE_FRONTIER",
                        "COLLISION","STUCK","PLANNER_ERROR","INFERENCE_ERROR","RESOURCE_ABORT"],
        maps=[], models=[], source_files={})
    assert manifest["pipe_commit"] == "e5bcb5ec4a9a13cbe14aa9f7850bc8b5989a2bb0"
    for map_id in MAPS:
        gt, valid = load_map(map_id)
        starts, component, separation, flag = starts_and_component(gt, valid)
        entry = dict(map_id=map_id, building_id="UNKNOWN", shape=list(gt.shape),
                     gt_hash=sha_array(gt), valid_hash=sha_array(valid),
                     component_hash=sha_array(component), coverage_denominator_cells=int(component.sum()),
                     start_separation_m=float(separation), separation_status=flag,
                     transform=dict(raw_grid_m=0.05, block_reduce=2,
                                    padded_map_grid_m=0.1, kth_pad_each_side=500,
                                    world_origin="downsampled raw top-left; x=col*0.1,y=row*0.1"),
                     starts=[dict(start_id="S%d"%(i+1), padded_row_col=p.tolist(),
                                  downsampled_raw_row_col=(p-500).tolist(),
                                  raw_block_top_left_row_col=((p-500)*2).tolist(),
                                  world_xy_m=[float(p[1]-500)/10,float(p[0]-500)/10])
                             for i,p in enumerate(starts)],
                     raw_asset_hashes={})
        for name in ["occ_map.npy","valid_space.npy"]:
            entry["raw_asset_hashes"][name] = sha_file(
                Path("/home/dell/MapEx/kth_test_maps")/map_id/name)
        np.savez_compressed(str(ART / (map_id+"_sealed.npz")), gt=gt, valid=valid, component=component)
        manifest["maps"].append(entry)
    expected = [
        "b37bfef69138806708d6e087b8db89836021f06db987cb3ef3db21fbf28422ca",
        "7880d164ccd88da58ddb0e6875f358bea02a232c452eff86b5e226957a489c26",
        "fcb43d1102d48ab6b14f5bfded859077d6da0c7efbfbda10d4acbb754c3d3bc1"]
    for i in range(1,4):
        path = Path("/home/dell/MapEx/pretrained_models/weights/lama_ensemble/train_%d" % i)
        digest = sha_file(path/"models/best.ckpt")
        if digest != expected[i-1]:
            raise RuntimeError("MODEL_CHECKPOINT_HASH_MISMATCH_G%d" % i)
        manifest["models"].append(dict(member=i, checkpoint=str(path/"models/best.ckpt"),
                                       sha256=digest, config_sha256=sha_file(path/"config.yaml")))
    for name in ["scripts/sim_utils.py","scripts/simple_mask_utils.py","scripts/lama_pred_utils.py",
                 "scripts/explore.py","configs/base.yaml","lama/saicinpainting/training/data/datasets.py"]:
        path = SOURCE/name
        if path.exists():
            manifest["source_files"][name] = sha_file(path)
    manifest["adapter_sha256"] = sha_file(Path(__file__).with_name("native_adapter.py"))
    atomic_json(ART/"manifest.json", manifest)
    print("SEALED",[(m["map_id"],m["shape"],m["starts"]) for m in manifest["maps"]],flush=True)


def parity():
    checks = {}
    gt = np.zeros((480,512))
    gt[[0,-1],:] = 1
    gt[:,[0,-1]] = 1
    gt[210:260,300] = 1  # internal occluder fixture
    mapper = make_mapper(gt)
    mapper.obs_map[:] = 0.5
    mapper.obs_map[140:340,160:360] = 0
    mapper.obs_map[210:260,300] = 1
    unknown = mapper.obs_map == .5
    ref = planning_cost(mapper,"CODE_REFERENCE")
    checks["helper_forces_unknown_finite"] = bool(np.isfinite(ref[unknown]).all())
    aligned = planning_cost(mapper,"ALIGNED_SHARED")
    checks["aligned_blocks_unknown"] = bool(np.isinf(aligned[unknown]).all())
    checks["reference_cost_parity"] = bool(np.array_equal(ref,mapper.inflate_map(mapper.obs_map,False)))
    path = np.asarray([[240,c] for c in range(220,261)],dtype=np.int32)
    original_hash = sha_array(path)
    poses = path[sample_indices(path)]
    prob = gt.astype(np.float32)
    raw = native.FrontierPlanner.process_path(poses.copy(),200,PRED,"pipe",prob,mapper.obs_map,*gt.shape)
    adapted = render(prob,mapper.obs_map,poses)
    checks["source_renderer_exact_mask"] = bool(np.array_equal(raw,adapted))
    checks["mask_excludes_known"] = bool(not adapted[~unknown].any())
    variance = torch.arange(gt.size,dtype=torch.float32).reshape(gt.shape)/(gt.size*100)
    native_score = torch.sum(variance[torch.tensor(raw,dtype=torch.bool)]).item()/max(1,len(poses))
    adapted_score = frozen_gain(variance.numpy(),unknown,adapted)/max(1,len(poses))
    checks["native_sample_count_score_exact"] = native_score == adapted_score
    checks["full_path_immutable"] = sha_array(path) == original_hash
    checks["native_endpoint_rule"] = int(sample_indices(path)[-1]) != len(path)-1
    checks["matched_endpoint_rule"] = int(sample_indices(path,True)[-1]) == len(path)-1
    a, b = make_mapper(gt), native.Mapper(gt,SENSOR,use_distance_transform_for_planning=True)
    a.observe_and_accumulate_given_pose([240,240])
    b.observe_and_accumulate_given_pose([240,240])
    checks["sensor_replay_equal"] = (
        np.array_equal(a.obs_map,b.obs_map) and np.array_equal(a.accum_hit_points,b.accum_hit_points))
    cost_a, cost_b = planning_cost(a,"ALIGNED_SHARED"), planning_cost(b,"ALIGNED_SHARED")
    checks["replay_cost_equal"] = bool(np.array_equal(cost_a,cost_b))
    checks["gt_free_occ_direction"] = bool(prob[0,0]==1 and prob[240,220]==0)
    result = dict(phase="PRELIMINARY_PARITY", checks=checks, all_pass=all(checks.values()),
                  source_renderer="UNCHANGED process_path incl Polygon.buffer(1), union, holes",
                  gaps=["Full chosen-goal/termination replay pending real predictor",
                        "Known source degenerate/NaN/MultiPolygon behavior requires ledger",
                        "No scientific trajectories collected"])
    atomic_json(ART/"preliminary_parity.json",result)
    print(json.dumps(result),flush=True)
    if not result["all_pass"]:
        raise RuntimeError("PRELIMINARY_PARITY_FAILED")


def probe():
    manifest = json.loads((ART/"manifest.json").read_text())
    selected = manifest["maps"][0]
    with np.load(str(ART/(selected["map_id"]+"_sealed.npz"))) as z:
        gt = z["gt"]
    mapper = make_mapper(gt)
    pose = selected["starts"][0]["padded_row_col"]
    mapper.observe_and_accumulate_given_pose(pose)
    observed = mapper.obs_map.astype(np.float32)
    np.save(str(ART/"probe_observed.npy"),observed)
    predictions, timing = [], []
    for member in [1,2,3]:
        print("MEMBER_START",member,time.time(),flush=True)
        prediction, measured = predict_member(observed,member)
        predictions.append(prediction)
        measured["maxrss_mib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.0
        timing.append(measured)
        atomic_json(ART/"probe_member_timings.json",timing)
        print("MEMBER_OK",json.dumps(measured),flush=True)
    stack = np.stack(predictions).astype(np.float32)
    # Native PIPE uses np.mean for mean, torch.var(unbiased=True) for variance.
    mean = np.mean(stack,axis=0)
    variance = torch.var(torch.from_numpy(stack),dim=0).numpy()
    np.savez_compressed(str(ART/"probe_predictions.npz"),predictions=stack,mean=mean,variance=variance,
                        observed_hash=sha_array(observed),manifest_hash=sha_file(ART/"manifest.json"))
    atomic_json(ART/"cpu_probe.json",dict(status="PASS",map_id=selected["map_id"],pose=pose,
                                         timings=timing,ensemble_inference_s=sum(t["inference_s"] for t in timing),
                                         maxrss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.0))
    print("ENSEMBLE_PROBE_OK",flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("phase",choices=["seal","parity","probe"])
    args = parser.parse_args()
    {"seal":seal,"parity":parity,"probe":probe}[args.phase]()
