"""Run G2-R6 against saved CAUSAL New Room D02 snapshot, without native simulation."""
import json
from pathlib import Path

import numpy as np

from g2_r6_watchman import R6Config, RoomWatchman

SRC = Path("/home/dell/mx072_g2_r4_structural_predicted_run11_20261017/"
           "native_g2_r4_structural_predicted_run11/"
           "G2_R4_STRUCTURAL_PREDICTED_RUN11_visvarprob")
OUT = Path(__file__).resolve().parent

def make(i):
    with np.load(SRC / "room_r41/inference" / f"decision_{i:06d}_input.npz") as f:
        obs, mean, variance = (f[k].copy() for k in ("observed", "mean", "variance"))
    with np.load(SRC / "room_r41/inference" / f"decision_{i:06d}_semantic.npz") as f:
        sem = f["semantic"].copy()
    with np.load(SRC / "g2_r4" / f"native_scope_{i:06d}.npz") as f:
        native = f["reachable_knownfree"].copy()
    log = json.loads((SRC / "g2_r4/decisions" /
                      f"decision_{i:06d}.json").read_text())
    obj = RoomWatchman(obs, mean, variance, sem,
                       log["robot_core_rc"], native, cfg=R6Config())
    return obj, obs, native, sem

def run():
    p, obs, native, sem = make(2)
    result = p.plan()
    public = {k: v for k, v in result.items() if k != "planned_grid_route_core_rc"}
    public["route_points_count"] = len(result["planned_grid_route_core_rc"])
    (OUT / "D02_G2_R6_OFFLINE.json").write_text(json.dumps(public, ensure_ascii=False, indent=2)+"\n")
    print(json.dumps(public, ensure_ascii=False, indent=2))
    if result["goal_sequence_core_rc"]:
        q = p.scan_update(obs, p.robot, native_walkable=native, semantic=sem)
        print("SAME_EPOCH_SCAN_FAST", json.dumps(q, ensure_ascii=False))
    return p

if __name__ == "__main__":
    run()
