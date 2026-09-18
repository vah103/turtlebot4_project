#!/usr/bin/env python3
"""Audit Way2 candidate-stop quality using observed SLAM occupancy only.

This is a development diagnostic for the existing MapEx runs. Unlike
``early_stopping_analysis.csv``, this audit does NOT fill unknown cells with the
MapEx ensemble mean. It converts each recorded ``canvas_map`` into an
observed-only occupied map:

    known occupied (>0) -> 1
    known free / unknown -> 0

and evaluates occupied IoU against the structural GT. The reference for loss is
the last evaluable recorded decision's observed-only map in the same run.

This helps distinguish genuine late SLAM-map improvement from fluctuation in the
prediction-assisted reconstructed IoU reference.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import numpy as np

import evaluate_mapex_run as evaluator


DEFAULT_GT_BY_ENVIRONMENT = {
    "new_room": "ground_truth/new_room/generated/new_room_structural_gt_v2.npz",
    "hospital": "ground_truth/hospital/generated/hospital_structural_gt_v1.npz",
}

DEFAULT_RULES = ((0.4, 2), (0.4, 3), (0.5, 2), (0.5, 3))


def read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def ffloat(v, default=math.nan):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except Exception:
        return default


def fint(v, default=-1):
    try:
        return int(float(v))
    except Exception:
        return default


def load_npz_data(path: Path, dtype=None):
    with np.load(path) as b:
        if "data" not in b.files:
            raise ValueError(f"{path} missing data array")
        x = np.asarray(b["data"])
    if dtype is not None:
        x = x.astype(dtype, copy=False)
    return x


def resolve_run(root: Path, value: str) -> Path:
    p = Path(value).expanduser()
    if p.is_dir():
        return p.resolve()
    p = root / "experiments" / "mapex" / value
    if p.is_dir():
        return p.resolve()
    raise FileNotFoundError(value)


def metadata(run_dir: Path):
    return json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))


def resolve_gt(mapex_lab: Path, meta: dict) -> Path:
    candidates = []
    if meta.get("structural_ground_truth_file"):
        candidates.append(Path(str(meta["structural_ground_truth_file"])).expanduser())
    env = str(meta.get("environment") or "new_room")
    rel = DEFAULT_GT_BY_ENVIRONMENT.get(env)
    if rel:
        candidates.append(mapex_lab / rel)
    for p in candidates:
        p = p.resolve()
        if p.is_file():
            return p
    raise FileNotFoundError(f"ground truth not found for {env}: {candidates}")


def observed_iou(canvas_path: Path, gt_occ: np.ndarray, mask: np.ndarray) -> float:
    observed = load_npz_data(canvas_path, dtype=np.int16)
    if observed.shape != mask.shape:
        raise ValueError(f"canvas {observed.shape} != GT {mask.shape}: {canvas_path}")
    # Unknown and known-free are non-occupied; only actually observed occupied cells count.
    pred = (observed > 0).astype(np.float32)
    return evaluator.occupied_iou(pred, gt_occ, mask)


def decision_ratios(run_dir: Path):
    rows = read_csv(run_dir / "candidates.csv")
    grouped = defaultdict(list)
    for r in rows:
        d = fint(r.get("decision_id"))
        if d >= 0:
            grouped[d].append(r)
    out = []
    for d in sorted(grouped):
        vals = []
        for r in grouped[d]:
            if fint(r.get("selectable"), 0) != 1:
                continue
            g = ffloat(r.get("information_gain"))
            c = ffloat(r.get("distance_m"))
            if math.isfinite(g) and math.isfinite(c) and c > 0:
                vals.append(g / c)
        if vals:
            out.append((d, max(vals)))
    return out


def stop_decision(run_dir: Path, lam: float, k: int):
    seq = decision_ratios(run_dir)
    streak = 0
    for d, r in seq:
        if r <= lam:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return d
    return None


def decision_quality(run_dir: Path, mapex_lab: Path):
    meta = metadata(run_dir)
    gt_path = resolve_gt(mapex_lab, meta)
    mask, gt_occ, _ = evaluator._load_ground_truth(gt_path)
    rows = []
    for d in read_csv(run_dir / "decisions.csv"):
        did = fint(d.get("decision_id"))
        canvas_rel = (d.get("canvas_map") or "").strip()
        if did < 0 or not canvas_rel:
            continue
        p = run_dir / canvas_rel
        if not p.is_file():
            continue
        obs = load_npz_data(p, dtype=np.int16)
        known = obs >= 0
        rows.append(
            {
                "decision_id": did,
                "observed_iou": observed_iou(p, gt_occ, mask),
                "known_fraction": float(np.count_nonzero(known) / known.size),
            }
        )
    rows.sort(key=lambda x: x["decision_id"])
    return rows


def median(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return statistics.median(xs) if xs else math.nan


def analyze_run(run_dir: Path, mapex_lab: Path, rules):
    meta = metadata(run_dir)
    env = str(meta.get("environment") or "unknown")
    quality = decision_quality(run_dir, mapex_lab)
    by_d = {x["decision_id"]: x for x in quality}
    if not quality:
        return [], None
    final = quality[-1]
    out = []
    for lam, k in rules:
        sd = stop_decision(run_dir, lam, k)
        if sd is None:
            out.append({
                "run_id": run_dir.name, "environment": env, "lambda": lam, "K": k,
                "triggered": False, "stop_decision": None,
                "stop_observed_iou": math.nan, "final_observed_iou": final["observed_iou"],
                "observed_iou_loss": math.nan, "stop_known_fraction": math.nan,
                "final_known_fraction": final["known_fraction"],
            })
            continue
        q = by_d.get(sd)
        if q is None:
            continue
        out.append({
            "run_id": run_dir.name,
            "environment": env,
            "lambda": lam,
            "K": k,
            "triggered": True,
            "stop_decision": sd,
            "stop_observed_iou": q["observed_iou"],
            "final_observed_iou": final["observed_iou"],
            "observed_iou_loss": final["observed_iou"] - q["observed_iou"],
            "stop_known_fraction": q["known_fraction"],
            "final_known_fraction": final["known_fraction"],
        })
    return out, {"run_id": run_dir.name, "environment": env, "final": final}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*", default=[*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"])
    args = ap.parse_args()

    mapex_lab = Path(__file__).resolve().parents[1]
    runs = [resolve_run(mapex_lab, r) for r in args.runs]
    all_rows = []
    finals = []
    for run in runs:
        rows, final = analyze_run(run, mapex_lab, DEFAULT_RULES)
        all_rows.extend(rows)
        if final:
            finals.append(final)

    print("Way2 observed-only IoU audit")
    for lam, k in DEFAULT_RULES:
        print(f"=== lambda={lam:g}, K={k} ===")
        subset = [r for r in all_rows if r["lambda"] == lam and r["K"] == k]
        for env in ("new_room", "hospital"):
            rows = [r for r in subset if r["environment"] == env]
            trig = [r for r in rows if r["triggered"]]
            losses = [r["observed_iou_loss"] for r in trig]
            over = sum(1 for x in losses if math.isfinite(x) and x > 0.01)
            worst = max(losses) if losses else math.nan
            print(
                f"{env}: trigger={len(trig)}/{len(rows)}, "
                f"median_observed_iou_loss={median(losses):.9f}, "
                f"worst={worst:.9f}, IoUloss>0.01={over}"
            )
            if env == "hospital":
                for r in trig:
                    print(
                        f"  {r['run_id']}: stop=d{r['stop_decision']}, "
                        f"obsIoU={r['stop_observed_iou']:.6f} -> final={r['final_observed_iou']:.6f}, "
                        f"loss={r['observed_iou_loss']:.6f}"
                    )

    # Focused trajectory for hpx_001 around the current candidate stop region.
    target = next((r for r in runs if r.name == "hpx_001"), None)
    if target is not None:
        q = decision_quality(target, mapex_lab)
        print("\n=== hpx_001 observed-only trajectory d36..d51 ===")
        for row in q:
            if 36 <= row["decision_id"] <= 51:
                print(
                    f"d{row['decision_id']:3d}: observed_iou={row['observed_iou']:.6f}, "
                    f"known_fraction={100*row['known_fraction']:.3f}%"
                )

    print("\nInterpretation: if reconstructed-IoU loss is large but observed-only IoU loss is small, the apparent failure is mainly prediction-reference instability. If observed-only loss is also large, the stop truly misses structural observations.")


if __name__ == "__main__":
    main()
