#!/usr/bin/env python3
"""Backtest an adaptive confirmation guard for Way2 early stopping.

Development/diagnostic only. This script does not change online MapEx.

Motivation
----------
The focused mpx_009 audit showed that the current guarded K=2 rule stops at d26
with observed-IoU loss 0.010516, while one more confirming decision (d27)
reduces that loss to approximately zero. A blanket K=3 rule is too conservative
for Hospital because hpx_001 then fails to trigger.

This script tests a simple online-deployable adaptive confirmation rule:

  base condition B_t:
      R_t = max_f information_gain / distance_m <= lambda
      AND max_f visible_unknown_cells / distance_m <= U_threshold

  stop logic:
      - require two consecutive evaluable decisions satisfying B_t;
      - if the second decision has only one selectable candidate, STOP;
      - if it still has multiple selectable candidates, require one additional
        consecutive confirming decision satisfying B_t, then STOP.

The candidate-count condition is intended as a structural confidence signal,
not as a tuned per-run exception: multiple selectable frontiers mean several
online alternatives remain; a single candidate is closer to an exhausted
frontier set.

Observed-only IoU is used strictly for offline evaluation.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

import audit_way2_observed_quality as obsq
import audit_way2_visible_unknown_guard as vug


DEFAULT_LAMBDAS = [0.3, 0.4]
DEFAULT_THRESHOLD = 10.0


def read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def ffloat(value, default=math.nan):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


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
    out = {}
    if not path.is_file():
        return out
    for row in read_csv(path):
        d = fint(row.get("decision_id"))
        if d >= 0:
            out[d] = row
    return out


def base_valid(state: dict, lam: float, threshold: float) -> bool:
    return (
        math.isfinite(state.get("R", math.nan))
        and math.isfinite(state.get("max_visible_per_m", math.nan))
        and state["R"] <= lam
        and state["max_visible_per_m"] <= threshold
    )


def first_fixed_k_stop(states: dict[int, dict], lam: float, threshold: float, k: int):
    streak = 0
    for d in sorted(states):
        if base_valid(states[d], lam, threshold):
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return d
    return None


def first_adaptive_stop(states: dict[int, dict], lam: float, threshold: float):
    """K=2 normally; require K=3 when second-state candidate_count > 1."""
    streak = 0
    require_third = False
    for d in sorted(states):
        state = states[d]
        if not base_valid(state, lam, threshold):
            streak = 0
            require_third = False
            continue

        streak += 1
        if streak < 2:
            continue

        if streak == 2:
            if int(state.get("candidate_count", 0)) <= 1:
                return d
            require_third = True
            continue

        if require_third and streak >= 3:
            return d

    return None


def median(values):
    vals = [v for v in values if v is not None and math.isfinite(v)]
    return statistics.median(vals) if vals else math.nan


def pct(value):
    return "n/a" if value is None or not math.isfinite(value) else f"{100.0 * value:.2f}%"


def evaluate_stop(run: Path, root: Path, states: dict[int, dict], stop: int | None):
    quality = obsq.decision_quality(run, root)
    q_by_d = {r["decision_id"]: r for r in quality}
    final_iou = quality[-1]["observed_iou"] if quality else math.nan
    early = early_rows(run)

    result = {
        "run_id": run.name,
        "environment": vug.environment(run),
        "stop": stop,
        "triggered": stop is not None,
        "loss": math.nan,
        "time_saved": math.nan,
        "distance_saved": math.nan,
        "candidate_count": math.nan,
        "R": math.nan,
        "guard": math.nan,
    }
    if stop is None:
        return result

    q = q_by_d.get(stop)
    if q is not None and math.isfinite(final_iou):
        result["loss"] = final_iou - q["observed_iou"]
    e = early.get(stop)
    if e:
        result["time_saved"] = ffloat(e.get("time_saved_fraction"))
        result["distance_saved"] = ffloat(e.get("distance_saved_fraction"))
    state = states.get(stop)
    if state:
        result["candidate_count"] = state.get("candidate_count", math.nan)
        result["R"] = state.get("R", math.nan)
        result["guard"] = state.get("max_visible_per_m", math.nan)
    return result


def summarize(rows):
    out = {}
    for env in ("new_room", "hospital"):
        env_rows = [r for r in rows if r["environment"] == env]
        trig = [r for r in env_rows if r["triggered"]]
        losses = [r["loss"] for r in trig if math.isfinite(r["loss"])]
        out[env] = {
            "runs": len(env_rows),
            "trigger": len(trig),
            "bad": sum(x > 0.01 for x in losses),
            "worst": max(losses) if losses else math.nan,
            "median_loss": median(losses),
            "median_time": median([r["time_saved"] for r in trig]),
            "median_distance": median([r["distance_saved"] for r in trig]),
        }
    return out


def print_summary(label: str, summary: dict):
    print(label)
    for env in ("new_room", "hospital"):
        s = summary[env]
        print(
            f"  {env}: trigger={s['trigger']}/{s['runs']}, bad={s['bad']}, "
            f"median_loss={s['median_loss']:.6f}, worst={s['worst']:.6f}, "
            f"time_saved={pct(s['median_time'])}, distance_saved={pct(s['median_distance'])}"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "runs",
        nargs="*",
        default=[*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"],
    )
    parser.add_argument("--lambdas", nargs="+", type=float, default=DEFAULT_LAMBDAS)
    parser.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [resolve_run(root, x) for x in args.runs]
    state_cache = {run.name: vug.candidate_state(run) for run in runs}

    print("Way2 adaptive-confirmation backtest")
    print(
        "Base condition: R<=lambda AND max_visible_per_m<=threshold; "
        "adaptive rule uses K=2 when candidate_count<=1, else requires K=3."
    )

    for lam in args.lambdas:
        print(f"\n=== lambda={lam:g}, threshold={args.threshold:g} ===")
        variants = []
        for label, stop_fn in (
            ("fixed K=2", lambda s: first_fixed_k_stop(s, lam, args.threshold, 2)),
            ("fixed K=3", lambda s: first_fixed_k_stop(s, lam, args.threshold, 3)),
            ("adaptive 2->3", lambda s: first_adaptive_stop(s, lam, args.threshold)),
        ):
            rows = []
            for run in runs:
                states = state_cache[run.name]
                rows.append(evaluate_stop(run, root, states, stop_fn(states)))
            variants.append((label, rows))
            print_summary(label, summarize(rows))

        adaptive_rows = next(rows for label, rows in variants if label == "adaptive 2->3")
        print("  adaptive run details:")
        for r in adaptive_rows:
            if r["triggered"]:
                flag = " BAD" if math.isfinite(r["loss"]) and r["loss"] > 0.01 else ""
                print(
                    f"    {r['run_id']}: d{r['stop']} cand={int(r['candidate_count'])} "
                    f"R={r['R']:.3f} guard={r['guard']:.3f} "
                    f"loss={r['loss']:.6f} time={pct(r['time_saved'])} "
                    f"dist={pct(r['distance_saved'])}{flag}"
                )
            else:
                print(f"    {r['run_id']}: no stop")

    print(
        "\nInterpretation: the adaptive rule is useful only if it removes the near-threshold "
        "mpx_009 failure while preserving the safe late hpx_001 stop and reasonable "
        "cross-environment trigger coverage. Existing runs remain development-only."
    )


if __name__ == "__main__":
    main()
