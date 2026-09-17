#!/usr/bin/env python3
"""Inspect focused Way2 lambda candidates from sensitivity outputs.

Reads way2_lambda_sensitivity_runs.csv produced by analyze_way2_lambda.py and
prints per-run details for selected lambda values (default: 0.4 and 0.5).
This is a diagnostic helper only; it does not freeze lambda.
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path


def _finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return math.nan
    return number if math.isfinite(number) else math.nan


def _int_value(value, default=0):
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _pct(value):
    number = _finite_float(value)
    return "n/a" if not math.isfinite(number) else f"{100.0 * number:.1f}%"


def _num(value, digits=3):
    number = _finite_float(value)
    return "n/a" if not math.isfinite(number) else f"{number:.{digits}f}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Inspect focused Way2 lambda candidates from sensitivity CSV."
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help=(
            "Path to way2_lambda_sensitivity_runs.csv. Default: "
            "mapex_lab/experiments/mapex/way2_lambda_sensitivity_runs.csv"
        ),
    )
    parser.add_argument(
        "--lambdas",
        type=float,
        nargs="+",
        default=[0.4, 0.5],
        help="Focused lambda values (default: 0.4 0.5).",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    csv_path = args.csv or (
        root / "experiments" / "mapex" / "way2_lambda_sensitivity_runs.csv"
    )
    if not csv_path.is_file():
        raise FileNotFoundError(
            f"Missing {csv_path}. Run analyze_way2_lambda.py first."
        )

    with csv_path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    focus = sorted(set(float(value) for value in args.lambdas))

    for lambda_value in focus:
        selected = [
            row
            for row in rows
            if math.isclose(
                _finite_float(row.get("lambda_value")),
                lambda_value,
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        ]
        if not selected:
            print(f"lambda={lambda_value:g}: no rows found")
            continue

        print(f"\n=== lambda={lambda_value:g} ===")
        for environment in ("new_room", "hospital"):
            env_rows = [row for row in selected if row.get("environment") == environment]
            if not env_rows:
                continue
            print(f"\n{environment}:")
            for row in sorted(env_rows, key=lambda item: item.get("run_id", "")):
                trigger = _int_value(row.get("stop_triggered"), 0) == 1
                if not trigger:
                    print(f"  {row.get('run_id')}: no stop")
                    continue

                rebounds = _int_value(row.get("post_stop_rebound_count"), 0)
                print(
                    f"  {row.get('run_id')}: "
                    f"stop_d={row.get('stop_decision_id') or 'n/a'}, "
                    f"stop={_pct(row.get('stop_fraction_of_evaluable'))} of evaluable, "
                    f"time_saved={_pct(row.get('time_saved_fraction'))}, "
                    f"distance_saved={_pct(row.get('distance_saved_fraction'))}, "
                    f"rebounds={rebounds}, "
                    f"rebound_fraction={_pct(row.get('post_stop_rebound_fraction'))}, "
                    f"max_post_R={_num(row.get('max_post_stop_ratio'))}, "
                    f"max_excess={_num(row.get('max_post_stop_excess'))}"
                )

            triggered = [row for row in env_rows if _int_value(row.get("stop_triggered"), 0) == 1]
            rebound_runs = [
                row for row in triggered if _int_value(row.get("post_stop_rebound_count"), 0) > 0
            ]
            print(
                f"  summary: trigger={len(triggered)}/{len(env_rows)}, "
                f"rebound_runs={len(rebound_runs)}/{len(triggered) if triggered else 0}"
            )

    print(
        "\nInterpretation: prioritize lambda values that stop consistently across both "
        "environments while keeping post-stop rebounds rare and small. Do not freeze "
        "lambda from savings alone."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
