#!/usr/bin/env python3
"""Local sensitivity sweep for the full Way2 adaptive completion rule.

Development/diagnostic only. This script does not modify online MapEx and must
not be used to claim independent validation on the existing 17 runs.

Frozen historical semantics:
    G_t(f) = information_gain(f)
    C_t(f) = distance_m(f)
    R_t    = max_f G_t(f) / C_t(f)

Candidate completion condition:
    B_t := R_t <= lambda
           AND max_f visible_unknown_cells(f) / distance_m(f) <= U_threshold

Adaptive persistence rule (candidate_count cutoff fixed to 1 for this audit):
    - after two consecutive B_t states, stop if selectable candidate_count <= 1;
    - otherwise require a third consecutive B_t state.

The purpose of this script is to test whether the promising candidate lies in a
stable local neighborhood of lambda and U_threshold, rather than only at the
single swept point lambda in {0.3, 0.4}, U_threshold=10.
"""
from __future__ import annotations

import argparse
import math
import statistics
from pathlib import Path

import audit_way2_visible_unknown_guard as vug
import evaluate_way2_adaptive_confirmation as adaptive


DEFAULT_LAMBDAS = [0.25, 0.30, 0.35, 0.40]
DEFAULT_THRESHOLDS = [7.5, 10.0, 12.5, 15.0]
CANDIDATE_COUNT_CUTOFF = 1


def median(values):
    vals = [v for v in values if v is not None and math.isfinite(v)]
    return statistics.median(vals) if vals else math.nan


def pct(value):
    return "n/a" if value is None or not math.isfinite(value) else f"{100.0 * value:.2f}%"


def resolve_run(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_dir():
        return path.resolve()
    path = root / "experiments" / "mapex" / value
    if path.is_dir():
        return path.resolve()
    raise FileNotFoundError(value)


def first_adaptive_stop(states: dict[int, dict], lam: float, threshold: float):
    streak = 0
    require_third = False
    for decision in sorted(states):
        state = states[decision]
        if not adaptive.base_valid(state, lam, threshold):
            streak = 0
            require_third = False
            continue

        streak += 1
        if streak < 2:
            continue

        if streak == 2:
            if int(state.get("candidate_count", 0)) <= CANDIDATE_COUNT_CUTOFF:
                return decision
            require_third = True
            continue

        if require_third and streak >= 3:
            return decision

    return None


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


def evaluate_config(runs, root, states_by_run, lam: float, threshold: float):
    rows = []
    for run in runs:
        states = states_by_run[run.name]
        stop = first_adaptive_stop(states, lam, threshold)
        rows.append(adaptive.evaluate_stop(run, root, states, stop))
    return rows, summarize(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "runs",
        nargs="*",
        default=[*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"],
    )
    parser.add_argument("--lambdas", nargs="+", type=float, default=DEFAULT_LAMBDAS)
    parser.add_argument(
        "--thresholds", nargs="+", type=float, default=DEFAULT_THRESHOLDS
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [resolve_run(root, value) for value in args.runs]
    states_by_run = {run.name: vug.candidate_state(run) for run in runs}

    records = []
    for lam in args.lambdas:
        for threshold in args.thresholds:
            rows, summary = evaluate_config(
                runs, root, states_by_run, lam, threshold
            )
            nr = summary["new_room"]
            hosp = summary["hospital"]
            records.append(
                {
                    "lambda": lam,
                    "threshold": threshold,
                    "rows": rows,
                    "summary": summary,
                    "total_bad": nr["bad"] + hosp["bad"],
                    "total_trigger": nr["trigger"] + hosp["trigger"],
                    "cross_env_full_hospital": hosp["trigger"] == hosp["runs"],
                }
            )

    print("Way2 adaptive-rule local neighborhood sensitivity")
    print(
        "Rule: R<=lambda AND max_visible_per_m<=threshold; after two valid states, "
        "stop if candidate_count<=1, otherwise require a third confirmation."
    )
    print("Existing runs are development data only.\n")

    for lam in args.lambdas:
        print(f"=== lambda={lam:g} ===")
        for rec in [r for r in records if r["lambda"] == lam]:
            threshold = rec["threshold"]
            nr = rec["summary"]["new_room"]
            hosp = rec["summary"]["hospital"]
            print(
                f"U/m<={threshold:g}: "
                f"NR {nr['trigger']}/{nr['runs']} bad={nr['bad']} worst={nr['worst']:.6f} "
                f"time={pct(nr['median_time'])} dist={pct(nr['median_distance'])} | "
                f"H {hosp['trigger']}/{hosp['runs']} bad={hosp['bad']} worst={hosp['worst']:.6f} "
                f"time={pct(hosp['median_time'])} dist={pct(hosp['median_distance'])}"
            )
        print()

    robust = [
        rec
        for rec in records
        if rec["total_bad"] == 0
        and rec["cross_env_full_hospital"]
        and rec["summary"]["new_room"]["trigger"] >= 10
    ]
    robust.sort(
        key=lambda rec: (
            -rec["summary"]["new_room"]["trigger"],
            -rec["summary"]["new_room"]["median_time"],
            rec["lambda"],
            rec["threshold"],
        )
    )

    print("=== Locally robust candidates ===")
    if not robust:
        print(
            "No configuration in this local neighborhood has zero >0.01 observed-IoU "
            "violations, Hospital 2/2 trigger, and New Room >=10/15 trigger."
        )
    else:
        for i, rec in enumerate(robust, 1):
            nr = rec["summary"]["new_room"]
            hosp = rec["summary"]["hospital"]
            print(
                f"#{i}: lambda={rec['lambda']:g}, U/m<={rec['threshold']:g} | "
                f"NR {nr['trigger']}/{nr['runs']} time={pct(nr['median_time'])} "
                f"dist={pct(nr['median_distance'])} worst={nr['worst']:.6f} | "
                f"H {hosp['trigger']}/{hosp['runs']} time={pct(hosp['median_time'])} "
                f"dist={pct(hosp['median_distance'])} worst={hosp['worst']:.6f}"
            )

    print("\n=== Stop-decision stability for robust candidates ===")
    if robust:
        run_names = [run.name for run in runs]
        for run_name in run_names:
            stops = []
            for rec in robust:
                row = next(r for r in rec["rows"] if r["run_id"] == run_name)
                stops.append(row["stop"] if row["triggered"] else None)
            unique = sorted({str(x) for x in stops})
            if len(unique) > 1:
                print(f"{run_name}: stops={stops}")

    print(
        "\nInterpretation: prefer a candidate only if zero-bad behavior and cross-environment "
        "coverage persist across neighboring lambda/guard values. If only one exact point "
        "works, treat it as likely development overfit and do not freeze it."
    )


if __name__ == "__main__":
    main()
