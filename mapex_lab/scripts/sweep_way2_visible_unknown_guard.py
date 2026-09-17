#!/usr/bin/env python3
"""Sweep online-deployable visible-unknown guards for Way2.

Development/diagnostic only. This does not change online MapEx and does not
freeze any threshold.

Base historical Way2 semantics stay fixed:

    G_t(f) = information_gain(f)
    C_t(f) = distance_m(f)
    R_t    = max_f G_t(f) / C_t(f)

A hypothetical stop is allowed only when the base utility condition and one
secondary completion guard both hold for K consecutive evaluable decisions:

    R_t <= lambda
    AND guard_t <= threshold

Two guard signals are tested separately over selectable frontiers:

    Umax_t  = max visible_unknown_cells
    Udmax_t = max visible_unknown_cells / distance_m

Observed-only occupied IoU is used only for offline evaluation, never as a
stopping input. Existing mpx/hpx runs are development data only.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

import audit_way2_observed_quality as obsq
import audit_way2_visible_unknown_guard as vug


DEFAULT_LAMBDAS = [0.10, 0.15, 0.20, 0.30, 0.40]
DEFAULT_KS = [2, 3]
DEFAULT_U_THRESHOLDS = [15, 20, 25, 30, 40, 50, 75, 100, 125, 150, 175, 200]
DEFAULT_UD_THRESHOLDS = [1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 7.5, 10.0, 15.0, 20.0]


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


def environment(run_dir: Path) -> str:
    return vug.environment(run_dir)


def early_rows(run_dir: Path):
    path = run_dir / "early_stopping_analysis.csv"
    if not path.is_file():
        return {}
    out = {}
    for row in read_csv(path):
        decision = fint(row.get("decision_id"))
        if decision >= 0:
            out[decision] = row
    return out


def first_guarded_stop(states, lam: float, k: int, signal: str, threshold: float):
    streak = 0
    for decision in sorted(states):
        state = states[decision]
        signal_value = state[signal]
        valid = (
            math.isfinite(state["R"])
            and math.isfinite(signal_value)
            and state["R"] <= lam
            and signal_value <= threshold
        )
        if valid:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return decision
    return None


def median(values):
    vals = [v for v in values if v is not None and math.isfinite(v)]
    return statistics.median(vals) if vals else math.nan


def pct(value):
    return "n/a" if value is None or not math.isfinite(value) else f"{100.0 * value:.1f}%"


def analyze_configuration(runs, root, cache, lam, k, signal, threshold):
    rows = []
    for run in runs:
        states = cache[run.name]["states"]
        stop = first_guarded_stop(states, lam, k, signal, threshold)
        quality_by_d = cache[run.name]["quality_by_d"]
        final_iou = cache[run.name]["final_iou"]
        early = cache[run.name]["early"]

        record = {
            "run_id": run.name,
            "environment": environment(run),
            "lambda": lam,
            "K": k,
            "signal": signal,
            "threshold": threshold,
            "triggered": stop is not None,
            "stop_decision": stop,
            "observed_iou_loss": math.nan,
            "time_saved_fraction": math.nan,
            "distance_saved_fraction": math.nan,
        }
        if stop is not None:
            q = quality_by_d.get(stop)
            if q is not None and math.isfinite(final_iou):
                record["observed_iou_loss"] = final_iou - q["observed_iou"]
            e = early.get(stop)
            if e:
                record["time_saved_fraction"] = ffloat(e.get("time_saved_fraction"))
                record["distance_saved_fraction"] = ffloat(e.get("distance_saved_fraction"))
        rows.append(record)

    summary = {
        "lambda": lam,
        "K": k,
        "signal": signal,
        "threshold": threshold,
    }
    for env in ("new_room", "hospital"):
        env_rows = [r for r in rows if r["environment"] == env]
        triggered = [r for r in env_rows if r["triggered"]]
        losses = [r["observed_iou_loss"] for r in triggered if math.isfinite(r["observed_iou_loss"])]
        summary[f"{env}_runs"] = len(env_rows)
        summary[f"{env}_trigger"] = len(triggered)
        summary[f"{env}_bad"] = sum(loss > 0.01 for loss in losses)
        summary[f"{env}_worst_loss"] = max(losses) if losses else math.nan
        summary[f"{env}_median_loss"] = median(losses)
        summary[f"{env}_median_time_saved"] = median([r["time_saved_fraction"] for r in triggered])
        summary[f"{env}_median_distance_saved"] = median([r["distance_saved_fraction"] for r in triggered])

    summary["total_trigger"] = summary["new_room_trigger"] + summary["hospital_trigger"]
    summary["total_bad"] = summary["new_room_bad"] + summary["hospital_bad"]
    finite_worst = [summary["new_room_worst_loss"], summary["hospital_worst_loss"]]
    finite_worst = [x for x in finite_worst if math.isfinite(x)]
    summary["worst_loss"] = max(finite_worst) if finite_worst else math.nan
    return rows, summary


def print_candidate(label, s):
    print(
        f"{label}: lambda={s['lambda']:g} K={s['K']} "
        f"{s['signal']}<={s['threshold']:g} | "
        f"NR trigger={s['new_room_trigger']}/{s['new_room_runs']} bad={s['new_room_bad']} "
        f"time={pct(s['new_room_median_time_saved'])} dist={pct(s['new_room_median_distance_saved'])} | "
        f"H trigger={s['hospital_trigger']}/{s['hospital_runs']} bad={s['hospital_bad']} "
        f"time={pct(s['hospital_median_time_saved'])} dist={pct(s['hospital_median_distance_saved'])} | "
        f"worst_loss={s['worst_loss']:.6f}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "runs",
        nargs="*",
        default=[*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"],
    )
    parser.add_argument("--lambdas", nargs="+", type=float, default=DEFAULT_LAMBDAS)
    parser.add_argument("--ks", nargs="+", type=int, default=DEFAULT_KS)
    parser.add_argument("--u-thresholds", nargs="+", type=float, default=DEFAULT_U_THRESHOLDS)
    parser.add_argument("--ud-thresholds", nargs="+", type=float, default=DEFAULT_UD_THRESHOLDS)
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [resolve_run(root, value) for value in args.runs]

    cache = {}
    for run in runs:
        states = vug.candidate_state(run)
        quality_rows = obsq.decision_quality(run, root)
        quality_by_d = {row["decision_id"]: row for row in quality_rows}
        final_iou = quality_rows[-1]["observed_iou"] if quality_rows else math.nan
        cache[run.name] = {
            "states": states,
            "quality_by_d": quality_by_d,
            "final_iou": final_iou,
            "early": early_rows(run),
        }

    summaries = []
    for lam in args.lambdas:
        for k in args.ks:
            for threshold in args.u_thresholds:
                _, summary = analyze_configuration(
                    runs, root, cache, lam, k, "max_visible", threshold
                )
                summaries.append(summary)
            for threshold in args.ud_thresholds:
                _, summary = analyze_configuration(
                    runs, root, cache, lam, k, "max_visible_per_m", threshold
                )
                summaries.append(summary)

    print("Way2 visible-unknown guard sweep")
    print(
        "Rule: stop after K consecutive evaluable decisions satisfying "
        "R<=lambda AND guard<=threshold."
    )

    safe = [s for s in summaries if s["total_bad"] == 0]
    safe.sort(
        key=lambda s: (
            -s["hospital_trigger"],
            -s["new_room_trigger"],
            -s["total_trigger"],
            -(s["new_room_median_time_saved"] if math.isfinite(s["new_room_median_time_saved"]) else -1),
        )
    )

    print("\n=== Zero observed-IoU-loss violations (>0.01) ===")
    if not safe:
        print("No swept configuration has zero >0.01 observed-IoU-loss violations.")
    else:
        for i, summary in enumerate(safe[:15], start=1):
            print_candidate(f"#{i}", summary)

    cross_env = [
        s for s in summaries
        if s["hospital_trigger"] == s["hospital_runs"]
        and s["new_room_trigger"] >= 10
    ]
    cross_env.sort(
        key=lambda s: (
            s["total_bad"],
            s["worst_loss"] if math.isfinite(s["worst_loss"]) else math.inf,
            -s["new_room_trigger"],
        )
    )

    print("\n=== Cross-environment coverage candidates (Hospital 2/2, New Room >=10/15) ===")
    if not cross_env:
        print("No swept configuration meets the requested trigger coverage.")
    else:
        for i, summary in enumerate(cross_env[:15], start=1):
            print_candidate(f"#{i}", summary)

    # Focus on hpx_001 to show when the guard actually delays the known failure.
    print("\n=== hpx_001 guarded stop examples ===")
    hpx = next((run for run in runs if run.name == "hpx_001"), None)
    if hpx is not None:
        states = cache[hpx.name]["states"]
        quality_by_d = cache[hpx.name]["quality_by_d"]
        final_iou = cache[hpx.name]["final_iou"]
        examples = [
            (0.4, 2, "max_visible", 30.0),
            (0.4, 2, "max_visible", 50.0),
            (0.4, 2, "max_visible", 100.0),
            (0.4, 2, "max_visible_per_m", 2.0),
            (0.4, 2, "max_visible_per_m", 3.0),
            (0.4, 2, "max_visible_per_m", 5.0),
        ]
        for lam, k, signal, threshold in examples:
            stop = first_guarded_stop(states, lam, k, signal, threshold)
            if stop is None:
                print(f"lambda={lam:g} K={k} {signal}<={threshold:g}: no stop")
                continue
            q = quality_by_d.get(stop)
            loss = final_iou - q["observed_iou"] if q is not None else math.nan
            print(
                f"lambda={lam:g} K={k} {signal}<={threshold:g}: "
                f"stop=d{stop}, observed_iou_loss={loss:.6f}"
            )

    print(
        "\nInterpretation: a useful completion guard should reduce unsafe stops without "
        "collapsing cross-environment trigger coverage. Treat every result here as "
        "development-only; do not freeze a threshold until the trade-off is reviewed."
    )


if __name__ == "__main__":
    main()
