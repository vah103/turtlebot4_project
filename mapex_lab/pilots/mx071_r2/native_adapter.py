"""MX071 R2 native PIPE adapter; Python 3.6 compatible, ROS-free."""
import gc
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(os.environ.get("MX071_ROOT", "/home/dell/mx071_dell_20261009"))
SOURCE = ROOT / "pipe_source"
sys.path[:0] = [str(SOURCE), str(SOURCE / "scripts"), str(SOURCE / "lama")]
# Import order matches explore.py and resolves upstream's circular import.
from scripts import simple_mask_utils as smu
import sim_utils as native
import torch
import pyastar2d

SENSOR = dict(laser_range_m=20, pixel_per_meter=10, num_laser=2500,
              dilate_diam_for_planning=3)
PRED = dict(laser_range_m=20, pixel_per_meter=10, num_laser=250, pathwise_index=3)
torch.set_num_threads(2)
torch.set_num_interop_threads(1)


def sha_file(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as stream:
        for b in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(b)
    return digest.hexdigest()


def sha_array(array):
    a = np.ascontiguousarray(array)
    digest = hashlib.sha256()
    digest.update(str(a.dtype).encode())
    digest.update(json.dumps(list(a.shape)).encode())
    digest.update(a.tobytes())
    return digest.hexdigest()


def atomic_json(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False))
    temp.replace(path)


def load_map(map_id):
    base = Path("/home/dell/MapEx/kth_test_maps") / map_id
    return native.get_kth_occ_validspace_map(str(base / "occ_map.npy"),
                                           str(base / "valid_space.npy"))


def make_mapper(gt):
    mapper = native.Mapper(gt, SENSOR, use_distance_transform_for_planning=True)
    # Constant visualization-only arrays are never read by the used upstream methods.
    # Keep source obs, GT PyOMap, accumulated hits, sensor and inflation contracts exact.
    for name in ["prev_obs_map","prev_pred_map","curr_pred_map","combined_pred_map",
                 "combined_obs1_pred2_map","combined_obs_map"]:
        delattr(mapper, name)
    return mapper


def planning_cost(mapper, contract):
    if contract == "CODE_REFERENCE":
        return mapper.get_inflated_planning_maps(unknown_as_occ=True)
    if contract == "ALIGNED_SHARED":
        return mapper.inflate_map(mapper.obs_map, unknown_as_occ=True)
    raise ValueError(contract)


def candidate_pool(observed, pose, cost):
    planner = native.FrontierPlanner("pipe")
    centers, clusters, _ = planner.get_frontier_centers_given_obs_map(observed)
    pool, rejected = [], []
    for center in centers:
        center = np.asarray(center, dtype=np.int32)
        if not np.isfinite(cost[tuple(center)]):
            rejected.append(dict(goal=center.tolist(), reason="INFLATED_OR_UNKNOWN"))
            continue
        if np.linalg.norm(center - pose) < 10:
            rejected.append(dict(goal=center.tolist(), reason="WITHIN_GOAL_TOLERANCE"))
            continue
        path = pyastar2d.astar_path(cost, tuple(pose), tuple(center), allow_diagonal=False)
        if path is None:
            rejected.append(dict(goal=center.tolist(), reason="UNREACHABLE"))
            continue
        if len(path[2::3]) == 0:
            rejected.append(dict(goal=center.tolist(), reason="EMPTY_NATIVE_SAMPLES"))
            continue
        pool.append(dict(goal=center.copy(), path=path.copy(),
                         candidate_id="%d,%d" % tuple(center),
                         weighted_cost=float(np.sum(cost[path[1:, 0], path[1:, 1]])),
                         path_m=float(len(path) - 1) / 10.0))
    return pool, rejected, clusters


def sample_indices(path, matched=False):
    indices = list(range(2, len(path), 3))
    if matched and (not indices or indices[-1] != len(path) - 1):
        indices.append(len(path) - 1)
    return np.asarray(indices, dtype=np.int32)


def render(numerical_map, observed, poses):
    if len(poses) == 0:
        raise ValueError("EMPTY_SAMPLED_PATH: source renderer has undefined local state")
    output = native.FrontierPlanner.process_path(
        np.asarray(poses).copy(), 200, PRED, "pipe", numerical_map, observed,
        observed.shape[0], observed.shape[1])
    if not np.isfinite(output).all():
        raise ValueError("SOURCE_RENDERER_NONFINITE: ALIGNED_FAIL_CLOSED_V1")
    # Upstream normally returns a bool mask already intersected with unknown.
    if output.dtype != np.bool_:
        raise ValueError("SOURCE_RENDERER_NONBOOLEAN: ALIGNED_FAIL_CLOSED_V1")
    return output.copy()


def frozen_gain(variance, unknown, mask):
    # Use torch's native summation rather than numpy's different reduction order.
    weighted = torch.from_numpy(np.asarray(variance, dtype=np.float32))
    selected = torch.from_numpy(np.asarray(mask & unknown, dtype=np.bool_))
    return torch.sum(weighted[selected]).item()


def predict_member(observed, member_id):
    """Exact upstream transform/output channel, with bounded sequential model load."""
    from lama_pred_utils import (
        convert_obsimg_to_model_input, get_lama_transform, load_lama_model)
    model_dir = Path("/home/dell/MapEx/pretrained_models/weights/lama_ensemble") / (
        "train_%d" % member_id)
    start = time.perf_counter()
    model = load_lama_model(str(model_dir), device="cpu")
    model.eval()
    load_s = time.perf_counter() - start
    transform = get_lama_transform("default_map_eval", (512, 512))
    batch, mask = convert_obsimg_to_model_input(
        np.stack([observed] * 3, axis=2).astype(np.float32), transform, "cpu")
    h, w = observed.shape
    ph, pw = batch["image"].shape[-2:]
    top, left = (ph - h) // 2, (pw - w) // 2
    known_padded = batch["mask"][0, 0] == 0
    start = time.perf_counter()
    with torch.no_grad():
        output = model(batch)
        # Source PIPE takes RGB tensor channel 0, occupancy probability.
        padded = output["inpainted"][0, 0].detach().cpu()
        if not torch.isfinite(padded).all():
            raise ValueError("NONFINITE_MODEL_OUTPUT")
        if not torch.equal(padded[known_padded], batch["image"][0, 0][known_padded]):
            raise ValueError("MODEL_DID_NOT_PRESERVE_KNOWN_INPUT")
        prediction = padded[top:top+h, left:left+w].numpy().copy()
    inference_s = time.perf_counter() - start
    if prediction.shape != observed.shape:
        raise ValueError("CROP_GEOMETRY_MISMATCH")
    del output, padded, batch, mask, model
    gc.collect()
    return prediction, dict(member=member_id, load_s=load_s, inference_s=inference_s,
                            input_shape=[h, w], model_shape=[ph, pw],
                            pad_top=top, pad_left=left, channel=0,
                            transform="default_map_eval", resized=False)
