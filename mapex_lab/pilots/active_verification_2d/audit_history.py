"""Small historical sensor/structural-GT disagreement audit, without retuning.

This is a simulator-input diagnostic, not a replacement for any frozen MapEx
evaluation or a reinterpretation of Gate P/U/R.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

from .predictor import file_hash
from .run import write_csv, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    gt_path = args.data_root/"mapex_lab/ground_truth/new_room/generated/new_room_structural_gt_v2.npz"
    with np.load(gt_path, allow_pickle=False) as z:
        truth, domain = z["data"], z["evaluation_mask"]
        gt_res, ox, oy = float(z["resolution"]), float(z["origin_x"]), float(z["origin_y"])
    rows = []
    for run in ["mpx_001", "mpx_002"]:
        for decision in [6, 18, 29]:
            path = args.data_root/"mapex_lab/experiments/mapex"/run/"decisions"/("policy_decision_%06d" % decision)/"observed_map_raw.npz"
            with np.load(path, allow_pickle=False) as z:
                observed = z["data"]
                res = float(z["resolution"])
                if abs(float(z["origin_yaw"])) > 1e-8:
                    raise ValueError("Audit supports axis-aligned raw maps only")
                cc = np.floor((float(z["origin_x"])+(np.arange(observed.shape[1])+.5)*res-ox)/gt_res).astype(int)
                rr = np.floor((float(z["origin_y"])+(np.arange(observed.shape[0])+.5)*res-oy)/gt_res).astype(int)
                if min(rr.min(), cc.min()) < 0 or rr.max() >= truth.shape[0] or cc.max() >= truth.shape[1]:
                    raise ValueError("Raw map exceeds GT canvas")
                gt, mask = truth[np.ix_(rr, cc)], domain[np.ix_(rr, cc)]
                free = (observed == 0) & mask
                occupied = (observed > 0) & mask
                rows.append(dict(run=run, decision=decision, raw_sha256=file_hash(path), gt_sha256=file_hash(gt_path),
                                 raw_height=observed.shape[0], raw_width=observed.shape[1],
                                 observed_free_cells=int(free.sum()), observed_occupied_cells=int(occupied.sum()),
                                 observed_free_in_gt_occupied_fraction=float((gt[free] > 0).mean()),
                                 observed_occupied_in_gt_free_fraction=float((gt[occupied] == 0).mean())))
    args.output.mkdir(parents=True, exist_ok=True)
    write_csv(args.output/"historical_input_audit.csv", rows)
    write_json(args.output/"historical_input_audit.json", dict(
        status="DIAGNOSTIC_ONLY_NOT_FROZEN_BENCHMARK", rows=len(rows),
        interpretation="Recorded SLAM occupancy and structural solids differ, especially occupied surfaces. Do not splice these observations into a perfect-sensing physical world without explicitly modelling disagreement. This does not prove a coordinate-frame bug, does not calibrate or replace frozen metrics, and does not change accepted historical results.",
        pilot_choice="Generate shared warm states by ideal sensor rollout on static 2D assets; retain historical data unmodified."))
    print("Historical input audit", len(rows), "rows")


if __name__ == "__main__":
    main()
