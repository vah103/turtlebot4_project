#!/usr/bin/env python3
"""Sensitivity of the adaptive Way2 confirmation rule to candidate-count cutoff.

Development/diagnostic only. Does not change online MapEx.

Base condition:
    R_t <= lambda
    AND max_visible_per_m <= threshold

Adaptive confirmation:
    - require two consecutive base-valid decisions;
    - at the second valid decision, STOP if candidate_count <= cutoff;
    - otherwise require one additional consecutive valid decision (effectively K=3).

Cutoff=0 behaves like fixed K=3 because candidate_count is positive on evaluable states.
A very large cutoff behaves like fixed K=2. The purpose is to test whether the
previous cutoff=1 result is robust rather than a single-value accident.
"""
from __future__ import annotations

import argparse
import math
import statistics
from pathlib import Path

import audit_way2_visible_unknown_guard as vug
import evaluate_way2_adaptive_confirmation as base

DEFAULT_LAMBDAS = [0.3, 0.4]
DEFAULT_THRESHOLD = 10.0
DEFAULT_CUTOFFS = [0, 1, 2, 3, 999]


def first_stop(states: dict[int, dict], lam: float, threshold: float, cutoff: int):
    streak = 0
    require_third = False
    for d in sorted(states):
        state = states[d]
        if not base.base_valid(state, lam, threshold):
            streak = 0
            require_third = False
            continue

        streak += 1
        if streak < 2:
            continue

        if streak == 2:
            if int(state.get("candidate_count", 0)) <= cutoff:
                return d
            require_third = True
            continue

        if require_third and streak >= 3:
            return d

    return None


def median(values):
    vals = [x for x in values if x is not None and math.isfinite(x)]
    return statistics.median(vals) if vals else math.nan


def pct(x):
    return "n/a" if x is None or not math.isfinite(x) else f"{100.0*x:.2f}%"


def label_for_cutoff(cutoff: int):
    if cutoff == 0:
        return "cutoff=0 (fixed K=3 equivalent)"
    if cutoff >= 999:
        return "cutoff=999 (fixed K=2 equivalent)"
    return f"cutoff={cutoff}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "runs",
        nargs="*",
        default=[*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"],
    )
    ap.add_argument("--lambdas", nargs="+", type=float, default=DEFAULT_LAMBDAS)
    ap.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD)
    ap.add_argument("--cutoffs", nargs="+", type=int, default=DEFAULT_CUTOFFS)
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [base.resolve_run(root, x) for x in args.runs]
    states = {run.name: vug.candidate_state(run) for run in runs}

    print("Way2 adaptive candidate-count sensitivity")
    print(
        "Rule: after two consecutive base-valid decisions, stop when "
        "candidate_count<=cutoff; otherwise require a third confirmation."
    )

    for lam in args.lambdas:
        print(f"\n=== lambda={lam:g}, guard_threshold={args.threshold:g} ===")
        for cutoff in args.cutoffs:
            rows = []
            for run in runs:
                s = states[run.name]
                stop = first_stop(s, lam, args.threshold, cutoff)
                rows.append(base.evaluate_stop(run, root, s, stop))

            print(label_for_cutoff(cutoff))
            for env in ("new_room", "hospital"):
                erows = [r for r in rows if r["environment"] == env]
                trig = [r for r in erows if r["triggered"]]
                losses = [r["loss"] for r in trig if math.isfinite(r["loss"])]
                print(
                    f"  {env}: trigger={len(trig)}/{len(erows)}, "
                    f"bad={sum(x > 0.01 for x in losses)}, "
                    f"worst={max(losses) if losses else math.nan:.6f}, "
                    f"median_loss={median(losses):.6f}, "
                    f"time_saved={pct(median([r['time_saved'] for r in trig]))}, "
                    f"distance_saved={pct(median([r['distance_saved'] for r in trig]))}"
                )

            for run_id in ("mpx_009", "hpx_001", "hpx_002"):
                r = next((x for x in rows if x["run_id"] == run_id), None)
                if r is None:
                    continue
                if not r["triggered"]:
                    print(f"  {run_id}: no stop")
                else:
                    print(
                        f"  {run_id}: d{r['stop']} cand={int(r['candidate_count'])} "
                        f"loss={r['loss']:.6f} time={pct(r['time_saved'])} "
                        f"dist={pct(r['distance_saved'])}"
                    )

    print(
        "\nInterpretation: cutoff=1 is credible only if nearby cutoffs show a sensible "
        "trade-off rather than a sharp one-off improvement. Do not freeze the cutoff "
        "from the 17 development runs alone."
    )


if __name__ == "__main__":
    main()
