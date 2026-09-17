#!/usr/bin/env python3
"""Focused inspection of the two strongest visible-unknown Way2 candidates.

Development/diagnostic only. This script intentionally does NOT perform another
threshold sweep. It compares two pre-existing candidates from the previous
coarse sweep:

    A: lambda=0.3, K=2, max_visible_per_m <= 10
    B: lambda=0.4, K=2, max_visible_per_m <= 10

For each run it reports stop decision, observed-only IoU loss, reconstructed IoU
loss, time/distance savings, and the guard value at the stop. It also prints the
runs where A and B differ, plus focused local trajectories for mpx_009,
hpx_001, and hpx_002.

The goal is to understand robustness/failure mode before changing any threshold.
Existing runs remain development-only and must not be used as independent final
validation.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import audit_way2_observed_quality as obsq
import audit_way2_visible_unknown_guard as vug
import sweep_way2_visible_unknown_guard as sweep


RULES = (
    ("A", 0.3, 2, "max_visible_per_m", 10.0),
    ("B", 0.4, 2, "max_visible_per_m", 10.0),
)


def read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def ffloat(value, default=math.nan):
    try:
        x = float(value)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def fint(value, default=-1):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def resolve_run(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_dir():
        return path.resolve()
    path = root / "experiments" / "mapex" / value
    if path.is_dir():
        return path.resolve()
    raise FileNotFoundError(value)


def early_rows(run_dir: Path):
    path = run_dir / "early_stopping_analysis.csv"
    if not path.is_file():
        return {}
    out = {}
    for row in read_csv(path):
        d = fint(row.get("decision_id"))
        if d >= 0:
            out[d] = row
    return out


def analyze_rule(run: Path, root: Path, states, quality_by_d, final_iou, early, label, lam, k, signal, threshold):
    stop = sweep.first_guarded_stop(states, lam, k, signal, threshold)
    rec = {
        "label": label,
        "lambda": lam,
        "K": k,
        "signal": signal,
        "threshold": threshold,
        "run_id": run.name,
        "environment": vug.environment(run),
        "stop": stop,
        "obs_loss": math.nan,
        "rec_loss": math.nan,
        "time_saved": math.nan,
        "distance_saved": math.nan,
        "guard": math.nan,
        "R": math.nan,
    }
    if stop is None:
        return rec
    state = states.get(stop)
    if state:
        rec["guard"] = state.get(signal, math.nan)
        rec["R"] = state.get("R", math.nan)
    q = quality_by_d.get(stop)
    if q is not None and math.isfinite(final_iou):
        rec["obs_loss"] = final_iou - q["observed_iou"]
    e = early.get(stop)
    if e:
        rec["rec_loss"] = ffloat(e.get("iou_loss_vs_final"))
        rec["time_saved"] = ffloat(e.get("time_saved_fraction"))
        rec["distance_saved"] = ffloat(e.get("distance_saved_fraction"))
    return rec


def fmt_pct(x):
    return "n/a" if not math.isfinite(x) else f"{100*x:.2f}%"


def fmt_float(x, digits=3):
    return "n/a" if not math.isfinite(x) else f"{x:.{digits}f}"


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

    all_records = []
    states_cache = {}
    for run in runs:
        states = vug.candidate_state(run)
        states_cache[run.name] = states
        quality = obsq.decision_quality(run, root)
        quality_by_d = {row["decision_id"]: row for row in quality}
        final_iou = quality[-1]["observed_iou"] if quality else math.nan
        early = early_rows(run)
        for rule in RULES:
            all_records.append(
                analyze_rule(
                    run,
                    root,
                    states,
                    quality_by_d,
                    final_iou,
                    early,
                    *rule,
                )
            )

    print("Way2 focused visible-unknown candidate inspection")
    for label, lam, k, signal, threshold in RULES:
        print(f"\n=== Rule {label}: lambda={lam:g}, K={k}, {signal}<={threshold:g} ===")
        subset = [r for r in all_records if r["label"] == label]
        for env in ("new_room", "hospital"):
            print(f"{env}:")
            for r in [x for x in subset if x["environment"] == env]:
                if r["stop"] is None:
                    print(f"  {r['run_id']}: no stop")
                    continue
                flag = " BAD" if math.isfinite(r["obs_loss"]) and r["obs_loss"] > 0.01 else ""
                print(
                    f"  {r['run_id']}: d{r['stop']} R={fmt_float(r['R'])} "
                    f"guard={fmt_float(r['guard'])} obs_loss={fmt_float(r['obs_loss'], 6)} "
                    f"rec_loss={fmt_float(r['rec_loss'], 6)} "
                    f"time={fmt_pct(r['time_saved'])} dist={fmt_pct(r['distance_saved'])}{flag}"
                )

    print("\n=== Runs where A and B differ ===")
    by_key = {(r["run_id"], r["label"]): r for r in all_records}
    differences = 0
    for run in runs:
        a = by_key[(run.name, "A")]
        b = by_key[(run.name, "B")]
        if a["stop"] == b["stop"]:
            continue
        differences += 1
        print(
            f"{run.name}: A stop={a['stop']} obs_loss={fmt_float(a['obs_loss'],6)} "
            f"time={fmt_pct(a['time_saved'])}; "
            f"B stop={b['stop']} obs_loss={fmt_float(b['obs_loss'],6)} "
            f"time={fmt_pct(b['time_saved'])}"
        )
    if differences == 0:
        print("A and B stop at exactly the same decisions on all runs.")

    print("\n=== Focused local trajectories ===")
    for run_id in ("mpx_009", "hpx_001", "hpx_002"):
        states = states_cache.get(run_id, {})
        if not states:
            continue
        stops = [
            by_key[(run_id, label)]["stop"]
            for label, *_ in RULES
            if by_key[(run_id, label)]["stop"] is not None
        ]
        if stops:
            lo = max(min(states), min(stops) - 4)
            hi = min(max(states), max(stops) + 4)
        else:
            hi = max(states)
            lo = max(min(states), hi - 8)
        print(f"{run_id}:")
        for d in sorted(states):
            if not (lo <= d <= hi):
                continue
            s = states[d]
            marks = []
            for label, *_ in RULES:
                if by_key[(run_id, label)]["stop"] == d:
                    marks.append(f"STOP-{label}")
            mark = (" " + ",".join(marks)) if marks else ""
            print(
                f"  d{d}: R={s['R']:.3f}, max_visible/m={s['max_visible_per_m']:.3f}, "
                f"max_visible={s['max_visible']:.0f}, candidates={s['candidate_count']}{mark}"
            )

    print(
        "\nInterpretation: prefer a candidate only if its extra savings come from stable "
        "earlier stops rather than additional quality failures. The single development "
        "violation near 0.01 should be inspected, not tuned away by threshold search."
    )


if __name__ == "__main__":
    main()
