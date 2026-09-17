#!/usr/bin/env python3
"""Audit an online-deployable Way2 completion guard based on visible unknown cells.

Motivation
----------
The low-lambda sweep showed that the current one-step utility signal

    R_t = max_f information_gain(f) / distance_m(f)

can become small in Hospital while the observed SLAM map still gains meaningful
occupied-IoU later. A plausible reason is that uncertainty-weighted information
``information_gain`` can be small even when a frontier still exposes a sizeable
currently-unknown region.

This script does NOT change MapEx and does NOT choose a threshold. It audits two
online-available secondary signals at candidate stop decisions:

    Umax_t  = max_f visible_unknown_cells(f)
    Udmax_t = max_f visible_unknown_cells(f) / distance_m(f)

over recorded selectable candidates.

It labels each hypothetical stop using observed-only IoU loss, so we can inspect
whether unsafe stops (loss > 0.01) occupy a different visible-unknown regime from
safe stops. Existing runs are development/diagnostic only.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

import audit_way2_observed_quality as obsq


DEFAULT_RULES = (
    (0.10, 2),
    (0.15, 2),
    (0.20, 2),
    (0.30, 2),
    (0.40, 2),
    (0.40, 3),
)


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


def resolve_run(root: Path, value: str) -> Path:
    p = Path(value).expanduser()
    if p.is_dir():
        return p.resolve()
    p = root / "experiments" / "mapex" / value
    if p.is_dir():
        return p.resolve()
    raise FileNotFoundError(value)


def environment(run_dir: Path) -> str:
    import json

    try:
        data = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
        return str(data.get("environment") or "unknown")
    except Exception:
        return "unknown"


def candidate_state(run_dir: Path):
    rows = read_csv(run_dir / "candidates.csv")
    grouped = defaultdict(list)
    for row in rows:
        d = fint(row.get("decision_id"))
        if d >= 0:
            grouped[d].append(row)

    out = {}
    for d, decision_rows in grouped.items():
        parsed = []
        for row in decision_rows:
            if fint(row.get("selectable"), 0) != 1:
                continue
            g = ffloat(row.get("information_gain"))
            dist = ffloat(row.get("distance_m"))
            visible = ffloat(row.get("visible_unknown_cells"))
            if not math.isfinite(g) or not math.isfinite(dist) or dist <= 0:
                continue
            if not math.isfinite(visible):
                visible = 0.0
            parsed.append(
                {
                    "candidate_id": row.get("candidate_id", ""),
                    "ratio": g / dist,
                    "gain": g,
                    "distance": dist,
                    "visible": visible,
                    "visible_per_m": visible / dist,
                }
            )
        if not parsed:
            continue
        best_ratio = max(parsed, key=lambda x: x["ratio"])
        out[d] = {
            "R": best_ratio["ratio"],
            "best_ratio_candidate": best_ratio["candidate_id"],
            "candidate_count": len(parsed),
            "max_visible": max(x["visible"] for x in parsed),
            "max_visible_per_m": max(x["visible_per_m"] for x in parsed),
            "best_ratio_visible": best_ratio["visible"],
            "best_ratio_visible_per_m": best_ratio["visible_per_m"],
        }
    return out


def first_stop(states: dict[int, dict], lam: float, k: int):
    streak = 0
    for d in sorted(states):
        if states[d]["R"] <= lam:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return d
    return None


def median(values):
    vals = [v for v in values if math.isfinite(v)]
    return statistics.median(vals) if vals else math.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "runs",
        nargs="*",
        default=[*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"],
    )
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [resolve_run(root, x) for x in args.runs]

    records = []
    states_by_run = {}
    quality_by_run = {}
    for run in runs:
        states = candidate_state(run)
        states_by_run[run.name] = states
        q_rows = obsq.decision_quality(run, root)
        q_by_d = {row["decision_id"]: row for row in q_rows}
        final_iou = q_rows[-1]["observed_iou"] if q_rows else math.nan
        quality_by_run[run.name] = (q_by_d, final_iou)

        for lam, k in DEFAULT_RULES:
            d = first_stop(states, lam, k)
            if d is None or d not in q_by_d:
                continue
            loss = final_iou - q_by_d[d]["observed_iou"]
            state = states[d]
            records.append(
                {
                    "run_id": run.name,
                    "environment": environment(run),
                    "lambda": lam,
                    "K": k,
                    "decision": d,
                    "loss": loss,
                    "unsafe": loss > 0.01,
                    **state,
                }
            )

    safe = [r for r in records if not r["unsafe"]]
    unsafe = [r for r in records if r["unsafe"]]

    print("Way2 visible-unknown completion-guard audit")
    print(f"candidate stop records: safe={len(safe)}, unsafe={len(unsafe)}")
    for label, rows in (("safe", safe), ("unsafe", unsafe)):
        if not rows:
            continue
        print(
            f"{label}: max_visible median={median([r['max_visible'] for r in rows]):.1f}, "
            f"range=[{min(r['max_visible'] for r in rows):.1f}, {max(r['max_visible'] for r in rows):.1f}], "
            f"max_visible_per_m median={median([r['max_visible_per_m'] for r in rows]):.3f}, "
            f"range=[{min(r['max_visible_per_m'] for r in rows):.3f}, {max(r['max_visible_per_m'] for r in rows):.3f}]"
        )

    print("\nUnsafe stop details (observed IoU loss > 0.01):")
    for r in sorted(unsafe, key=lambda x: (x["run_id"], x["lambda"], x["K"])):
        print(
            f"{r['run_id']} lambda={r['lambda']:g} K={r['K']} d{r['decision']}: "
            f"loss={r['loss']:.6f}, R={r['R']:.3f}, "
            f"max_visible={r['max_visible']:.0f}, max_visible/m={r['max_visible_per_m']:.3f}, "
            f"bestR_visible={r['best_ratio_visible']:.0f}"
        )

    for run_id in ("hpx_001", "hpx_002"):
        states = states_by_run.get(run_id, {})
        if not states:
            continue
        print(f"\n=== {run_id} late trajectory ===")
        start = 40 if run_id == "hpx_001" else 38
        for d in sorted(states):
            if d < start:
                continue
            s = states[d]
            print(
                f"d{d:3d}: R={s['R']:.3f}, max_visible={s['max_visible']:.0f}, "
                f"max_visible/m={s['max_visible_per_m']:.3f}, candidates={s['candidate_count']}"
            )

    print(
        "\nInterpretation: a secondary visible-unknown guard is promising only if unsafe "
        "stops consistently retain larger online-visible unknown regions than safe stops. "
        "Do not choose a threshold from this audit alone."
    )


if __name__ == "__main__":
    main()
