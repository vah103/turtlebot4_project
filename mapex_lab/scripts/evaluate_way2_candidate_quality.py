#!/usr/bin/env python3
"""Backtest map quality at hypothetical Way2 stop decisions.

Development/diagnostic only.

Frozen historical Way2 quantities:
  G = raw information_gain
  C = Euclidean distance_m
  R_t = max_f G/C over selectable candidates

This script evaluates focused candidate rules (default lambda 0.4/0.5, K 2/3),
finds the first stop decision, then joins that decision to the existing
``early_stopping_analysis.csv`` row for offline map-quality metrics.

Important caveat: ``iou_loss_vs_final`` in the existing analyzer is relative to
the reconstructed map at the last analyzed policy decision, not necessarily the
actual final observed SLAM map. Results are therefore development diagnostics,
not final validation claims.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


RUN_FIELDS = [
    "run_id", "environment", "lambda", "K", "triggered", "stop_decision",
    "stop_fraction", "coverage", "iou_if_stop", "iou_loss_vs_final",
    "tu_if_stop", "tu_loss_vs_final", "time_saved_fraction",
    "distance_saved_fraction", "final_reference_decision_id", "quality_status",
]

ENV_FIELDS = [
    "environment", "lambda", "K", "run_count", "trigger_count", "trigger_rate",
    "quality_count", "median_stop_fraction", "median_time_saved_fraction",
    "median_distance_saved_fraction", "median_iou_loss", "mean_iou_loss",
    "worst_iou_loss", "iou_loss_gt_0p01_count", "median_tu_loss", "worst_tu_loss",
]


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
    p = run_dir / "metadata.json"
    if p.is_file():
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            return str(d.get("environment") or "unknown")
        except Exception:
            pass
    return "unknown"


def decision_ratios(run_dir: Path):
    rows = read_csv(run_dir / "candidates.csv")
    grouped = defaultdict(list)
    for row in rows:
        did = fint(row.get("decision_id"))
        if did >= 0:
            grouped[did].append(row)

    out = []
    for did in sorted(grouped):
        ratios = []
        for row in grouped[did]:
            if fint(row.get("selectable"), 0) != 1:
                continue
            gain = ffloat(row.get("information_gain"))
            cost = ffloat(row.get("distance_m"))
            if math.isfinite(gain) and math.isfinite(cost) and cost > 0:
                ratios.append(gain / cost)
        if ratios:
            out.append((did, max(ratios)))
    return out


def find_stop(seq, lam: float, k: int):
    streak = 0
    for idx, (did, ratio) in enumerate(seq):
        if ratio <= lam:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return idx, did
    return None, None


def quality_rows(run_dir: Path):
    path = run_dir / "early_stopping_analysis.csv"
    if not path.is_file():
        return {}
    out = {}
    for row in read_csv(path):
        did = fint(row.get("decision_id"))
        if did >= 0:
            out[did] = row
    return out


def analyze(run_dir: Path, lam: float, k: int):
    seq = decision_ratios(run_dir)
    stop_idx, stop_did = find_stop(seq, lam, k)
    base = {
        "run_id": run_dir.name,
        "environment": environment(run_dir),
        "lambda": lam,
        "K": k,
        "triggered": stop_did is not None,
        "stop_decision": stop_did,
        "stop_fraction": ((stop_idx + 1) / len(seq)) if stop_idx is not None and seq else None,
        "coverage": None,
        "iou_if_stop": None,
        "iou_loss_vs_final": None,
        "tu_if_stop": None,
        "tu_loss_vs_final": None,
        "time_saved_fraction": None,
        "distance_saved_fraction": None,
        "final_reference_decision_id": None,
        "quality_status": "no_stop" if stop_did is None else "missing_quality_row",
    }
    if stop_did is None:
        return base

    row = quality_rows(run_dir).get(stop_did)
    if row is None:
        return base

    for key in (
        "coverage", "iou_if_stop", "iou_loss_vs_final", "tu_if_stop",
        "tu_loss_vs_final", "time_saved_fraction", "distance_saved_fraction",
    ):
        value = ffloat(row.get(key))
        base[key] = value if math.isfinite(value) else None
    final_ref = fint(row.get("final_reference_decision_id"))
    base["final_reference_decision_id"] = final_ref if final_ref >= 0 else None
    base["quality_status"] = "ok"
    return base


def finite(values):
    return [float(v) for v in values if v is not None and math.isfinite(float(v))]


def median(values):
    vals = finite(values)
    return statistics.median(vals) if vals else None


def mean(values):
    vals = finite(values)
    return statistics.mean(vals) if vals else None


def maximum(values):
    vals = finite(values)
    return max(vals) if vals else None


def environment_rows(run_rows):
    grouped = defaultdict(list)
    for row in run_rows:
        grouped[(row["environment"], row["lambda"], row["K"])].append(row)

    out = []
    for (env, lam, k), rows in sorted(grouped.items()):
        triggered = [r for r in rows if r["triggered"]]
        quality = [r for r in triggered if r["quality_status"] == "ok"]
        iou_losses = finite([r["iou_loss_vs_final"] for r in quality])
        tu_losses = finite([r["tu_loss_vs_final"] for r in quality])
        out.append({
            "environment": env,
            "lambda": lam,
            "K": k,
            "run_count": len(rows),
            "trigger_count": len(triggered),
            "trigger_rate": len(triggered) / len(rows) if rows else math.nan,
            "quality_count": len(quality),
            "median_stop_fraction": median([r["stop_fraction"] for r in triggered]),
            "median_time_saved_fraction": median([r["time_saved_fraction"] for r in quality]),
            "median_distance_saved_fraction": median([r["distance_saved_fraction"] for r in quality]),
            "median_iou_loss": median(iou_losses),
            "mean_iou_loss": mean(iou_losses),
            "worst_iou_loss": maximum(iou_losses),
            "iou_loss_gt_0p01_count": sum(v > 0.01 for v in iou_losses),
            "median_tu_loss": median(tu_losses),
            "worst_tu_loss": maximum(tu_losses),
        })
    return out


def fmt(v):
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if not math.isfinite(v):
            return "nan"
        return f"{v:.9f}"
    return str(v)


def pct(v):
    return "n/a" if v is None or not math.isfinite(v) else f"{100*v:.2f}%"


def write_csv(path: Path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for row in rows:
            w.writerow({field: fmt(row.get(field)) for field in fields})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "runs", nargs="*",
        default=[*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"],
    )
    ap.add_argument("--lambdas", nargs="+", type=float, default=[0.4, 0.5])
    ap.add_argument("--ks", nargs="+", type=int, default=[2, 3])
    ap.add_argument("--output-dir", type=Path, default=None)
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [resolve_run(root, x) for x in args.runs]
    results = [analyze(run, lam, k) for lam in args.lambdas for k in args.ks for run in runs]
    envs = environment_rows(results)

    out_dir = args.output_dir or (root / "experiments" / "mapex")
    out_dir.mkdir(parents=True, exist_ok=True)
    run_csv = out_dir / "way2_candidate_quality_runs.csv"
    env_csv = out_dir / "way2_candidate_quality_environments.csv"
    json_path = out_dir / "way2_candidate_quality.json"
    write_csv(run_csv, RUN_FIELDS, results)
    write_csv(env_csv, ENV_FIELDS, envs)
    json_path.write_text(json.dumps({
        "development_only": True,
        "quality_reference_caveat": (
            "iou_loss_vs_final uses the last analyzed policy-decision reconstruction, "
            "not necessarily the actual final observed SLAM map"
        ),
        "runs": results,
        "environments": envs,
    }, indent=2), encoding="utf-8")

    print("Way2 candidate-rule map-quality backtest")
    for lam in args.lambdas:
        print(f"=== lambda={lam:g} ===")
        for k in args.ks:
            print(f"K={k}")
            for env in ("new_room", "hospital"):
                matches = [r for r in envs if r["lambda"] == lam and r["K"] == k and r["environment"] == env]
                if not matches:
                    continue
                r = matches[0]
                print(
                    f"  {env}: trigger={r['trigger_count']}/{r['run_count']}, "
                    f"quality={r['quality_count']}/{r['trigger_count']}, "
                    f"time_saved={pct(r['median_time_saved_fraction'])}, "
                    f"distance_saved={pct(r['median_distance_saved_fraction'])}, "
                    f"median_iou_loss={fmt(r['median_iou_loss'])}, "
                    f"worst_iou_loss={fmt(r['worst_iou_loss'])}, "
                    f"IoUloss>0.01={r['iou_loss_gt_0p01_count']}, "
                    f"median_tu_loss={fmt(r['median_tu_loss'])}, "
                    f"worst_tu_loss={fmt(r['worst_tu_loss'])}"
                )
        print()

    print(f"Wrote {run_csv}")
    print(f"Wrote {env_csv}")
    print(f"Wrote {json_path}")
    print("Caveat: IoU loss is relative to the analyzer's last-decision reconstructed reference, not necessarily the actual final observed SLAM map.")


if __name__ == "__main__":
    main()
