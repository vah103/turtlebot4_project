"""Same-pool MX071 scoring. Online P0 accepts no GT or evaluation component."""
import time

from native_adapter import (
    SENSOR, candidate_pool, frozen_gain, make_mapper, render, sample_indices, smu, torch)
import numpy as np


def score_online(observed, pose, cost, predictions, mean, variance):
    """The only online goal selector: source PIPE samples and sample-count denominator."""
    start = time.perf_counter()
    pool, rejected, clusters = candidate_pool(observed,pose,cost)
    unknown = observed == .5
    valid = []
    for candidate in pool:
        indices = sample_indices(candidate["path"])
        try:
            mask = render(mean,observed,candidate["path"][indices])
        except Exception as exc:
            rejected.append(dict(goal=candidate["goal"].tolist(),reason=str(exc)))
            continue
        candidate["native_indices"] = indices
        candidate["native_mask"] = mask
        candidate["P0"] = frozen_gain(variance,unknown,mask)/max(1,len(indices))
        valid.append(candidate)
    chosen = max(valid,key=lambda c:c["P0"]) if valid else None
    return dict(pool=valid,rejected=rejected,clusters=clusters,chosen=chosen,
                P0_compute_s=time.perf_counter()-start)


def _fractional_gain(variance,unknown,masks):
    # Keep U(mean/var) frozen, render K members, then average visibility.
    gains = [frozen_gain(variance,unknown,m) for m in masks]
    return sum(gains)/len(gains)


def diagnose_snapshot(online,observed,pose,predictions,mean,variance,evaluation_gt):
    """Offline only; GT changes scoring and never generates a frontier or a path."""
    unknown = observed == .5
    sensor = make_mapper(evaluation_gt)
    for candidate in online["pool"]:
        path = candidate["path"]
        native_idx = candidate["native_indices"]
        matched_idx = sample_indices(path,True)
        end_poses = path[-1:]
        p_native = path[native_idx]
        p_matched = path[matched_idx]
        denominator = max(1,len(native_idx))
        start = time.perf_counter()
        member_native = [render(member,observed,p_native) for member in predictions]
        candidate["P1"] = _fractional_gain(variance,unknown,member_native)/denominator
        candidate["P1_compute_s"] = time.perf_counter()-start
        start = time.perf_counter()
        gt_native = render(evaluation_gt,observed,p_native)
        candidate["P4"] = frozen_gain(variance,unknown,gt_native)/denominator
        candidate["P4_compute_s"] = time.perf_counter()-start
        start = time.perf_counter()
        end = render(mean,observed,end_poses)
        matched = render(mean,observed,p_matched)
        euclidean_pix = float(np.linalg.norm(path[-1]-pose))
        euclidean_m = euclidean_pix/10.
        physical_m = candidate["path_m"]
        end_gain = frozen_gain(variance,unknown,end)
        path_gain = frozen_gain(variance,unknown,matched)
        candidate.update(
            END_EUCLIDEAN=end_gain/euclidean_m,
            END_PATHCOST=end_gain/physical_m,
            PATH_EUCLIDEAN=path_gain/euclidean_m,
            PATH_PATHCOST=path_gain/physical_m,
            M0=path_gain/physical_m,
            matched_indices=matched_idx)
        member_matched = [render(member,observed,p_matched) for member in predictions]
        gt_matched = render(evaluation_gt,observed,p_matched)
        candidate["M1"] = _fractional_gain(variance,unknown,member_matched)/physical_m
        candidate["M4"] = frozen_gain(variance,unknown,gt_matched)/physical_m
        candidate["matched_compute_s"] = time.perf_counter()-start
        start = time.perf_counter()
        _,_,_,_,mapex_grid = smu.get_vis_mask(
            mean,tuple(path[-1]),laser_range=200,num_laser=250,
            raycast_mode="probabilistic",hit_prob_threshold=.8)
        mapex_mask = (mapex_grid==0) & unknown
        candidate["MAPEX_ALIGNED"] = (
            torch.tensor(frozen_gain(variance,unknown,mapex_mask),dtype=torch.float32) /
            np.float64(euclidean_pix)).item()
        candidate["MAPEX_compute_s"] = time.perf_counter()-start
        # Dense rays on exactly the native declared scored poses. No extra motion.
        start = time.perf_counter()
        dense = np.zeros(observed.shape,dtype=np.bool_)
        for point in p_native:
            scan = sensor.get_instant_obs_at_pose(point)
            for name in ["vis_ind","actual_hit_points"]:
                cells = scan[name]
                dense[cells[:,0],cells[:,1]] = True
        dense &= unknown
        candidate["gt_dense_compute_s"] = time.perf_counter()-start
        candidate["visibility_masks"] = dict(
            V_PRED_250=candidate["native_mask"],V_GT_MATCHED_250=gt_native,
            V_GT_DENSE_DECLARED_2500=dense,END_PRED_250=end,
            MATCHED_PATH_PRED_250=matched,MAPEX_ENDPOINT_PRED_250=mapex_mask)
        candidate["frozen_gains"] = {
            key:frozen_gain(variance,unknown,mask)
            for key,mask in candidate["visibility_masks"].items()}
        candidate["visibility_differences"] = dict(
            map_error_cells=int(np.count_nonzero(candidate["native_mask"] ^ gt_native)),
            renderer_ray_discrepancy_cells=int(np.count_nonzero(gt_native ^ dense)),
            executed_sensor="NOT_EVALUATED_UNTIL_ACTUAL_BRANCH",
            residual_label="EXECUTION_SENSOR_CONTRACT_GAP")
    choices = {}
    for key in ["P0","P1","P4","M0","MAPEX_ALIGNED"]:
        pool = [c for c in online["pool"] if key in c and np.isfinite(c[key])]
        choices[key] = max(pool,key=lambda c:c[key]) if pool else None
    return choices
