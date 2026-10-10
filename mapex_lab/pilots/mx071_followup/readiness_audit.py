"""Read-only DELL source/identity audit and native MapEx fixture repeatability.

Fixtures use observation-only stand-ins; this program cannot start a scientific
trajectory, a paired branch, training, or a robot. Source harness provenance is
recorded in reference_fixture.py.
"""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

SOURCE = "/home/dell/MapEx"
SOURCE_PIN = "53636bd1c79153acc3c74a532837d78c926bae5e"
OLD = Path("/home/dell/mx071_dell_20261009/artifacts")
LIDAR = dict(laser_range_m=20, num_laser=2500, pixel_per_meter=10,
             dilate_diam_for_planning=3)
PRED_VIS = dict(laser_range_m=20, num_laser=250, pixel_per_meter=10)
START = (606, 621)
G_EXPECTED = "58c1841c10d4d97bc1f894ec8fe5b015b165277ddb2b9bb29eb688b21d4363b5"

def sha_file(path):
    h = hashlib.sha256()
    with open(str(path), "rb") as f:
        for chunk in iter(lambda: f.read(1024*1024), b""):
            h.update(chunk)
    return h.hexdigest()

def array_sha(array):
    import numpy as np
    a = np.ascontiguousarray(array)
    h = hashlib.sha256()
    h.update(str(a.dtype).encode())
    h.update(json.dumps(list(a.shape)).encode())
    h.update(a.tobytes())
    return h.hexdigest()

def source_modules():
    for path in [SOURCE, SOURCE+"/scripts", SOURCE+"/lama"]:
        if path not in sys.path:
            sys.path.insert(0, path)
    from scripts import sim_utils, simple_mask_utils
    import lama_pred_utils
    return sim_utils, simple_mask_utils, lama_pred_utils

def fixture_predict(obs):
    import torch
    import albumentations as A
    import cv2
    import numpy as np
    padded = A.PadIfNeeded(min_height=None, min_width=None,
                pad_height_divisor=16, pad_width_divisor=16,
                border_mode=cv2.BORDER_CONSTANT, value=0)(image=obs)["image"]
    unknown = padded == .5
    maps = [padded.copy() for _ in range(3)]
    for i, a in enumerate(maps):
        a[unknown] = [.2, .45, .7][i]
    stack = torch.stack([torch.from_numpy(a) for a in maps])
    rgb = np.repeat(maps[1][:,:,None], 3, axis=2)
    viz = np.clip(rgb*255, 0, 255).astype("uint8")
    return dict(G1=maps[0], G2=maps[1], G3=maps[2],
        mean=np.mean(stack.numpy(), axis=0),
        variance=torch.var(stack, dim=0).numpy(), alltrain_rgb=rgb,
        alltrain_viz=viz, pred_maputils=(viz[:,:,0] > 128).astype(np.float64),
        unknown=unknown, padded_obs=padded), dict(fixture_only=True)

def atomic_json(path, obj):
    p = Path(path)
    q = p.with_suffix(p.suffix+".tmp")
    q.write_text(json.dumps(obj, indent=2, sort_keys=True, allow_nan=False)+"\n")
    q.replace(p)

def main():
    import numpy as np
    import torch
    from reference_fixture import run_reference
    torch.set_num_threads(2)
    if torch.get_num_interop_threads() != 1:
        torch.set_num_interop_threads(1)
    out = Path(__file__).parent/"evidence/native_source_readiness.json"
    report = dict(status="RUNNING", scientific_ready=False,
        scientific_sources_started=0, scientific_branches_started=0,
        real_inference_run=False, source=SOURCE, source_pin=SOURCE_PIN,
        device=subprocess.check_output(["hostname"]).decode().strip(),
        fixture_harness_origin="turtlebot4_project@8bf7b96f388298361a1c0dd6c32d52041ffcb814",
        source_files={}, models={}, checks={})
    atomic_json(out, report)
    try:
        assert report["device"] == "dell", "WRONG_DEVICE"
        head = subprocess.check_output(["git", "-C", SOURCE, "rev-parse", "HEAD"]).decode().strip()
        assert head == SOURCE_PIN, "WRONG_SOURCE_PIN"
        for relative in ["scripts/explore.py", "scripts/sim_utils.py",
                         "scripts/simple_mask_utils.py", "scripts/lama_pred_utils.py"]:
            expected = subprocess.check_output(["git", "-C", SOURCE, "show", SOURCE_PIN+":"+relative])
            actual = Path(SOURCE, relative).read_bytes()
            assert actual == expected, "USED_SOURCE_FILE_DRIFT:"+relative
            report["source_files"][relative] = hashlib.sha256(actual).hexdigest()
        report["checks"]["used_source_bytes_match_pin"] = True
        lama_head = subprocess.check_output(["git", "-C", SOURCE+"/lama", "rev-parse", "HEAD"]).decode().strip()
        assert lama_head == "b61dcb33e063fe9586b50e2dc7b70f97d5046c1d"
        report["lama_pin"] = lama_head
        seal = json.loads((OLD/"manifest.json").read_text())
        weights = Path(SOURCE)/"pretrained_models/weights"
        report["models"]["G"] = dict(path=str(weights/"big_lama/models/best.ckpt"),
            sha256=sha_file(weights/"big_lama/models/best.ckpt"))
        assert report["models"]["G"]["sha256"] == G_EXPECTED, "ALLTRAIN_MODEL_MISMATCH"
        for i in [1,2,3]:
            path = weights/("lama_ensemble/train_%d/models/best.ckpt"%i)
            got = sha_file(path)
            expected = seal["models"][i-1]
            # Bind to the exact old sealed model entry; schema is inspected rather
            # than silently guessing an alternative identity.
            expected_sha = expected["sha256"]
            assert got == expected_sha, "ENSEMBLE_MODEL_MISMATCH"
            report["models"]["G%d"%i] = dict(path=str(path), sha256=got)
        report["checks"]["four_model_identities"] = True
        info = seal["maps"][0]
        assert info["map_id"] == "50052750"
        with np.load(str(OLD/"50052750_sealed.npz")) as z:
            gt, component = z["gt"].copy(), z["component"].copy()
        assert array_sha(gt) == info["gt_hash"]
        assert array_sha(component) == info["component_hash"]
        sim, _, lpu = source_modules()
        mapper = sim.Mapper(gt, LIDAR, use_distance_transform_for_planning=True)
        mapper.observe_and_accumulate_given_pose(np.asarray(START))
        requested = mapper.get_inflated_planning_maps(unknown_as_occ=True)
        effective_false = mapper.inflate_map(mapper.obs_map, unknown_as_occ=False)
        blocked = mapper.inflate_map(mapper.obs_map, unknown_as_occ=True)
        np.testing.assert_array_equal(requested, effective_false)
        assert not np.array_equal(requested, blocked)
        report["checks"]["source_unknown_false_behavior"] = True
        batch, _ = lpu.convert_obsimg_to_model_input(
            np.stack([mapper.obs_map]*3, axis=2),
            lpu.get_lama_transform("default_map_eval", (512,512)), "cpu")
        image = batch["image"][0,0].numpy()
        h,w = mapper.obs_map.shape
        top,left = (image.shape[0]-h)//2,(image.shape[1]-w)//2
        np.testing.assert_array_equal(image[top:top+h,left:left+w], mapper.obs_map)
        report["frame"] = dict(mapper_shape=list(mapper.obs_map.shape),
            model_shape=list(image.shape), center_pad_top=top, center_pad_left=left,
            source_candidate_origins_shifted=False,
            preserved_source_frame_not_mx071_cropped_frame=True)
        report["checks"]["source_input_frame"] = True
        before = array_sha(gt)
        start_time = time.monotonic()
        first = run_reference(gt, 4, START)
        second = run_reference(gt, 4, START)
        for name in ["poses","observations","epochs","step_goals",
                     "final_observation_sha256","final_pose_list"]:
            assert first[name] == second[name], "SOURCE_REPEATABILITY:"+name
        assert array_sha(gt) == before, "SOURCE_MUTATED_INPUT_GT"
        report["checks"]["direct_source_fixture_repeatability"] = True
        report["fixture"] = dict(steps_per_repeat=4, repeats=2,
            total_control_steps=len(first["poses"])+len(second["poses"])-2,
            wall_s=time.monotonic()-start_time, model="OBSERVATION_ONLY_STAND_IN",
            poses=first["poses"], epochs=first["epochs"],
            final_observation_sha256=first["final_observation_sha256"],
            source_terminal=first["source_terminal"])
        report["resources"] = dict(disk_free_bytes=shutil.disk_usage(str(out)).free,
            total_seven_question_output_bound="NOT_YET_CERTIFIED",
            runtime_and_peak_real_model_rss="NOT_TESTED_BY_FIXTURE")
        report["status"] = "SOURCE_IDENTITY_AND_FIXTURE_CHECKS_PASS"
        report["remaining_gates"] = [
            "Seven-question intervention contracts and branch ceiling",
            "Real four-model full-size resource/parity probe",
            "Native MapEx complete-state branch replay and commitment controls",
            "Lossless full-cohort storage and total time bounds"]
    except Exception as exc:
        import traceback
        report["status"] = "CHECK_FAILED"
        report["error"] = repr(exc)
        report["traceback"] = traceback.format_exc()
        raise
    finally:
        atomic_json(out, report)

if __name__ == "__main__":
    main()
