#!/usr/bin/env python3
"""Inspect whether low Way2 utility in Hospital still precedes map-quality gains.

Default target: hpx_001.

Joins per-decision
  R_t = max_f information_gain / distance_m over selectable candidates
with offline map-quality metrics from early_stopping_analysis.csv.

Development diagnostic only. The quality metrics use the existing analyzer's
last-decision reconstructed reference and must not be treated as prospective
validation.
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


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
    raise FileNotFoundError(f"Cannot resolve run: {value}")


def ratios(run_dir: Path):
    grouped = defaultdict(list)
    for row in read_csv(run_dir / "candidates.csv"):
        did = fint(row.get("decision_id"))
        if did >= 0:
            grouped[did].append(row)

    out = {}
    for did, rows in grouped.items():
        vals = []
        for row in rows:
            if fint(row.get("selectable"), 0) != 1:
                continue
            g = ffloat(row.get("information_gain"))
            c = ffloat(row.get("distance_m"))
            if math.isfinite(g) and math.isfinite(c) and c > 0:
                vals.append(g / c)
        if vals:
            out[did] = max(vals)
    return out


def quality(run_dir: Path):
    path = run_dir / "early_stopping_analysis.csv"
    if not path.is_file():
        raise FileNotFoundError(
            f"Missing {path}. Run analyze_early_stopping.py for this run first."
        )
    out = {}
    for row in read_csv(path):
        did = fint(row.get("decision_id"))
        if did >= 0:
            out[did] = {
                "iou": ffloat(row.get("iou_if_stop")),
                "iou_loss": ffloat(row.get("iou_loss_vs_final")),
                "coverage": ffloat(row.get("coverage")),
                "tu_loss": ffloat(row.get("tu_loss_vs_final")),
                "time_saved": ffloat(row.get("time_saved_fraction")),
                "distance_saved": ffloat(row.get("distance_saved_fraction")),
                "final_reference_decision_id": fint(row.get("final_reference_decision_id")),
            }
    return out


def first_stop(seq, lam: float, k: int):
    streak = 0
    for did, r in seq:
        if r <= lam:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return did
    return None


def fmt(x, digits=3):
    if x is None or not math.isfinite(x):
        return "n/a"
    return f"{x:.{digits}f}"


def pct(x):
    if x is None or not math.isfinite(x):
        return "n/a"
    return f"{100*x:.2f}%"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", nargs="?", default="hpx_001")
    ap.add_argument("--start", type=int, default=36)
    ap.add_argument("--lambdas", nargs="+", type=float, default=[0.4, 0.5])
    ap.add_argument("--ks", nargs="+", type=int, default=[2, 3])
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    run_dir = resolve_run(root, args.run)
    rs = ratios(run_dir)
    qs = quality(run_dir)
    seq = sorted(rs.items())

    print(f"=== {run_dir.name}: R_t vs map quality ===")
    common = [did for did, _ in seq if did in qs and did >= args.start]
    if not common:
        print("No joined decisions in requested range.")
        return

    print("decision   R_t      IoU_if_stop  IoU_loss   coverage   time_saved  dist_saved")
    for did in common:
        q = qs[did]
        print(
            f"d{did:>3}      {rs[did]:>6.3f}   {q['iou']:>11.6f}  "
            f"{q['iou_loss']:>8.6f}  {pct(q['coverage']):>9}  "
            f"{pct(q['time_saved']):>10}  {pct(q['distance_saved']):>10}"
        )

    print("\nCandidate stops:")
    for lam in args.lambdas:
        for k in args.ks:
            stop = first_stop(seq, lam, k)
            q = qs.get(stop, {}) if stop is not None else {}
            print(
                f"lambda={lam:g}, K={k}: stop=d{stop if stop is not None else 'none'}, "
                f"R={fmt(rs.get(stop) if stop is not None else None)}, "
                f"IoU_loss={fmt(q.get('iou_loss'), 6)}, "
                f"coverage={pct(q.get('coverage'))}"
            )

    final_ref_ids = [q["final_reference_decision_id"] for q in qs.values() if q["final_reference_decision_id"] >= 0]
    final_ref = max(final_ref_ids) if final_ref_ids else None
    if final_ref in qs:
        qf = qs[final_ref]
        print(
            f"\nAnalyzer final reference: d{final_ref}, "
            f"IoU={fmt(qf['iou'], 6)}, coverage={pct(qf['coverage'])}"
        )

    # Quantify quality gained after each candidate stop without assuming monotonicity.
    print("\nPost-stop observed quality gain:")
    for lam in args.lambdas:
        for k in args.ks:
            stop = first_stop(seq, lam, k)
            if stop is None or stop not in qs:
                continue
            later = [(did, qs[did]) for did in sorted(qs) if did > stop and math.isfinite(qs[did]["iou"])]
            if not later:
                continue
            best_did, best_q = max(later, key=lambda x: x[1]["iou"])
            gain = best_q["iou"] - qs[stop]["iou"]
            print(
                f"lambda={lam:g}, K={k}: stop d{stop} -> best later d{best_did}, "
                f"IoU gain={gain:.6f}, later R={fmt(rs.get(best_did))}"
            )

    print(
        "\nInterpretation: if R_t remains below lambda while IoU/coverage still improve materially, "
        "the issue is not merely rebound/noise; the current G/C utility is not yet a sufficient "
        "proxy for completion in that Hospital run."
    )


if __name__ == "__main__":
    main()
