#!/usr/bin/env python3
"""Sweep lower Way2 lambda values against observed-only and reconstructed quality.

Development/diagnostic only.

Frozen historical semantics:
  G = raw information_gain
  C = Euclidean distance_m
  R_t = max_f G/C over selectable candidates

This script intentionally focuses on lower lambdas after the 0.4/0.5 candidate
rules were shown to stop hpx_001 too early. It combines the existing stop replay
with early_stopping_analysis.csv and observed-only quality calculations.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path

import audit_way2_observed_quality as obsq


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
    for r in rows:
        did = fint(r.get("decision_id"))
        if did >= 0:
            grouped[did].append(r)
    out = []
    for did in sorted(grouped):
        vals = []
        for r in grouped[did]:
            if fint(r.get("selectable"), 0) != 1:
                continue
            g = ffloat(r.get("information_gain"))
            c = ffloat(r.get("distance_m"))
            if math.isfinite(g) and math.isfinite(c) and c > 0:
                vals.append(g / c)
        if vals:
            out.append((did, max(vals)))
    return out


def first_stop(run_dir: Path, lam: float, k: int):
    seq = decision_ratios(run_dir)
    streak = 0
    for i, (did, ratio) in enumerate(seq):
        if ratio <= lam:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return did, i, seq
    return None, None, seq


def early_rows(run_dir: Path):
    p = run_dir / "early_stopping_analysis.csv"
    if not p.is_file():
        return {}
    out = {}
    for r in read_csv(p):
        did = fint(r.get("decision_id"))
        if did >= 0:
            out[did] = r
    return out


def median(vals):
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    return statistics.median(vals) if vals else math.nan


def pct(v):
    return "n/a" if not math.isfinite(v) else f"{100*v:.2f}%"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "runs",
        nargs="*",
        default=[*(f"mpx_{i:03d}" for i in range(1,16)), "hpx_001", "hpx_002"],
    )
    ap.add_argument(
        "--lambdas",
        nargs="+",
        type=float,
        default=[0.05, 0.075, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4],
    )
    ap.add_argument("--ks", nargs="+", type=int, default=[2, 3])
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [resolve_run(root, x) for x in args.runs]

    # Reuse the observed-only audit implementation to obtain per-decision observed IoU.
    observed = {}
    for run in runs:
        quality = obsq.decision_quality(run, root)
        observed[run.name] = {
            "by_decision": {row["decision_id"]: row for row in quality},
            "final_observed_iou": quality[-1]["observed_iou"] if quality else math.nan,
        }

    all_results = []
    for lam in args.lambdas:
        for k in args.ks:
            for run in runs:
                did, idx, seq = first_stop(run, lam, k)
                env = environment(run)
                row = {
                    "run_id": run.name,
                    "environment": env,
                    "lambda": lam,
                    "K": k,
                    "triggered": did is not None,
                    "stop_decision": did,
                    "time_saved_fraction": math.nan,
                    "distance_saved_fraction": math.nan,
                    "reconstructed_iou_loss": math.nan,
                    "observed_iou_loss": math.nan,
                }
                if did is not None:
                    early = early_rows(run).get(did)
                    if early:
                        row["time_saved_fraction"] = ffloat(early.get("time_saved_fraction"))
                        row["distance_saved_fraction"] = ffloat(early.get("distance_saved_fraction"))
                        row["reconstructed_iou_loss"] = ffloat(early.get("iou_loss_vs_final"))
                    obs_run = observed[run.name]
                    stop = obs_run["by_decision"].get(did)
                    if stop is not None and math.isfinite(obs_run["final_observed_iou"]):
                        row["observed_iou_loss"] = obs_run["final_observed_iou"] - stop["observed_iou"]
                all_results.append(row)

    print("Way2 low-lambda quality sweep")
    for lam in args.lambdas:
        print(f"=== lambda={lam:g} ===")
        for k in args.ks:
            print(f"K={k}")
            subset = [r for r in all_results if r["lambda"] == lam and r["K"] == k]
            for env in ("new_room", "hospital"):
                rows = [r for r in subset if r["environment"] == env]
                trig = [r for r in rows if r["triggered"]]
                obs_losses = [r["observed_iou_loss"] for r in trig if math.isfinite(r["observed_iou_loss"])]
                rec_losses = [r["reconstructed_iou_loss"] for r in trig if math.isfinite(r["reconstructed_iou_loss"])]
                bad_obs = sum(v > 0.01 for v in obs_losses)
                bad_rec = sum(v > 0.01 for v in rec_losses)
                print(
                    f"  {env}: trigger={len(trig)}/{len(rows)}, "
                    f"time_saved={pct(median([r['time_saved_fraction'] for r in trig]))}, "
                    f"distance_saved={pct(median([r['distance_saved_fraction'] for r in trig]))}, "
                    f"median_obs_loss={median(obs_losses):.6f}, worst_obs_loss={max(obs_losses) if obs_losses else math.nan:.6f}, obs>0.01={bad_obs}, "
                    f"median_rec_loss={median(rec_losses):.6f}, worst_rec_loss={max(rec_losses) if rec_losses else math.nan:.6f}, rec>0.01={bad_rec}"
                )
            h1 = next((r for r in subset if r["run_id"] == "hpx_001"), None)
            if h1:
                if h1["triggered"]:
                    print(
                        f"  hpx_001: stop=d{h1['stop_decision']}, "
                        f"obs_loss={h1['observed_iou_loss']:.6f}, "
                        f"rec_loss={h1['reconstructed_iou_loss']:.6f}"
                    )
                else:
                    print("  hpx_001: no stop")
        print()

    print("Interpretation: look for a single lambda/K with cross-environment trigger coverage and no systematic >0.01 observed-IoU loss. Development-only; do not freeze from these runs alone.")


if __name__ == "__main__":
    main()
