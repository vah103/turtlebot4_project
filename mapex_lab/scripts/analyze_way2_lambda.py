#!/usr/bin/env python3
"""Sensitivity analysis for the Way2 utility threshold lambda.

This script uses the frozen historical-replay semantics:

    G_t(f) = recorded information_gain
    C_t(f) = recorded Euclidean distance_m
    R_t    = max_f G_t(f) / C_t(f), over selectable candidates

The Way2 stopping condition is equivalent to R_t <= lambda.  To reduce a
single noisy crossing, the default guard requires K=2 distinct recorded
candidate states with R_t <= lambda.

The historical 17 runs are development/diagnostic data only.  The script does
NOT pick a final lambda.  It evaluates a declared sensitivity grid and reports:
- first guarded stop decision;
- estimated time/distance savings at that decision;
- later rebounds R_t > lambda after the hypothetical stop;
- maximum post-stop R_t and excess above lambda.

Time/distance savings are reconstructed from metrics.csv at or immediately
before the stop decision time.  They are development diagnostics, not
prospective validation results.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

from analyze_way2_utility import analyze_run, _finite_float, _int_value, _resolve_run


DEFAULT_LAMBDAS = (0.10, 0.20, 0.30, 0.40, 0.50, 0.75, 1.00, 1.50, 2.00)

RUN_FIELDS = [
    "run_id",
    "environment",
    "lambda_value",
    "guard_k",
    "evaluable_decisions",
    "stop_triggered",
    "stop_decision_id",
    "stop_evaluable_index",
    "stop_fraction_of_evaluable",
    "stop_time_s",
    "full_time_s",
    "time_saved_s",
    "time_saved_fraction",
    "stop_distance_m",
    "full_distance_m",
    "distance_saved_m",
    "distance_saved_fraction",
    "post_stop_evaluable_decisions",
    "post_stop_rebound_count",
    "post_stop_rebound_fraction",
    "max_post_stop_ratio",
    "max_post_stop_excess",
    "last_ratio_before_end",
]

ENV_FIELDS = [
    "environment",
    "lambda_value",
    "guard_k",
    "run_count",
    "trigger_count",
    "trigger_rate",
    "median_stop_fraction",
    "median_time_saved_fraction",
    "median_distance_saved_fraction",
    "runs_with_any_rebound",
    "rebound_run_rate",
    "median_post_stop_rebound_fraction",
    "median_max_post_stop_excess",
]


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return ""
    return f"{number:.9f}"


def _median(values: list[float]) -> float | None:
    finite = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.median(finite) if finite else None


def _metrics_at_stop(run_dir: Path, stop_time_s: float | None) -> dict[str, float]:
    path = run_dir / "metrics.csv"
    if not path.is_file():
        return {
            "stop_distance_m": math.nan,
            "full_time_s": math.nan,
            "full_distance_m": math.nan,
        }

    rows = _read_csv(path)
    parsed = []
    for row in rows:
        time_s = _finite_float(row.get("time_s"))
        distance_m = _finite_float(row.get("distance_m"))
        if math.isfinite(time_s) and math.isfinite(distance_m):
            parsed.append((time_s, distance_m))

    if not parsed:
        return {
            "stop_distance_m": math.nan,
            "full_time_s": math.nan,
            "full_distance_m": math.nan,
        }

    parsed.sort(key=lambda item: item[0])
    full_time_s, full_distance_m = parsed[-1]

    stop_distance_m = math.nan
    if stop_time_s is not None and math.isfinite(stop_time_s):
        prior = [item for item in parsed if item[0] <= stop_time_s]
        if prior:
            stop_distance_m = prior[-1][1]
        else:
            stop_distance_m = parsed[0][1]

    return {
        "stop_distance_m": stop_distance_m,
        "full_time_s": full_time_s,
        "full_distance_m": full_distance_m,
    }


def _analyze_lambda_for_run(run_dir: Path, lambda_value: float, guard_k: int) -> dict:
    rows, summary = analyze_run(
        run_dir,
        lambda_value=lambda_value,
        guard_k=guard_k,
        gain_normalization="raw",
        variance_reference=None,
    )

    evaluable = [row for row in rows if row.get("status") == "ok"]
    stop_row = next((row for row in evaluable if row.get("guarded_stop") == "1"), None)

    stop_triggered = stop_row is not None
    stop_decision_id = (
        _int_value(stop_row.get("decision_id"), -1) if stop_row is not None else None
    )
    stop_time_s = (
        _finite_float(stop_row.get("time_s")) if stop_row is not None else math.nan
    )

    stop_evaluable_index = None
    post_rows: list[dict[str, str]] = []
    if stop_row is not None:
        for index, row in enumerate(evaluable, start=1):
            if row is stop_row:
                stop_evaluable_index = index
                post_rows = evaluable[index:]
                break

    stop_fraction = (
        stop_evaluable_index / len(evaluable)
        if stop_evaluable_index is not None and evaluable
        else math.nan
    )

    rebounds = []
    post_ratios = []
    for row in post_rows:
        ratio = _finite_float(row.get("lambda_break_even"))
        if not math.isfinite(ratio):
            continue
        post_ratios.append(ratio)
        if ratio > lambda_value:
            rebounds.append(ratio)

    post_rebound_fraction = (
        len(rebounds) / len(post_ratios) if post_ratios else 0.0
    )
    max_post_ratio = max(post_ratios) if post_ratios else math.nan
    max_post_excess = (
        max_post_ratio - lambda_value if math.isfinite(max_post_ratio) else math.nan
    )

    metrics = _metrics_at_stop(
        run_dir,
        stop_time_s if math.isfinite(stop_time_s) else None,
    )
    full_time_s = metrics["full_time_s"]
    full_distance_m = metrics["full_distance_m"]
    stop_distance_m = metrics["stop_distance_m"]

    time_saved_s = (
        full_time_s - stop_time_s
        if stop_triggered and math.isfinite(full_time_s) and math.isfinite(stop_time_s)
        else math.nan
    )
    time_saved_fraction = (
        time_saved_s / full_time_s
        if math.isfinite(time_saved_s) and full_time_s > 0.0
        else math.nan
    )
    distance_saved_m = (
        full_distance_m - stop_distance_m
        if stop_triggered
        and math.isfinite(full_distance_m)
        and math.isfinite(stop_distance_m)
        else math.nan
    )
    distance_saved_fraction = (
        distance_saved_m / full_distance_m
        if math.isfinite(distance_saved_m) and full_distance_m > 0.0
        else math.nan
    )

    last_ratio = (
        _finite_float(evaluable[-1].get("lambda_break_even")) if evaluable else math.nan
    )

    return {
        "run_id": run_dir.name,
        "environment": str(summary.get("environment") or "unknown"),
        "lambda_value": lambda_value,
        "guard_k": guard_k,
        "evaluable_decisions": len(evaluable),
        "stop_triggered": stop_triggered,
        "stop_decision_id": stop_decision_id,
        "stop_evaluable_index": stop_evaluable_index,
        "stop_fraction_of_evaluable": stop_fraction,
        "stop_time_s": stop_time_s,
        "full_time_s": full_time_s,
        "time_saved_s": time_saved_s,
        "time_saved_fraction": time_saved_fraction,
        "stop_distance_m": stop_distance_m,
        "full_distance_m": full_distance_m,
        "distance_saved_m": distance_saved_m,
        "distance_saved_fraction": distance_saved_fraction,
        "post_stop_evaluable_decisions": len(post_ratios),
        "post_stop_rebound_count": len(rebounds),
        "post_stop_rebound_fraction": post_rebound_fraction,
        "max_post_stop_ratio": max_post_ratio,
        "max_post_stop_excess": max_post_excess,
        "last_ratio_before_end": last_ratio,
    }


def _environment_rows(run_rows: list[dict]) -> list[dict]:
    grouped: dict[tuple[str, float, int], list[dict]] = defaultdict(list)
    for row in run_rows:
        key = (row["environment"], float(row["lambda_value"]), int(row["guard_k"]))
        grouped[key].append(row)

    output = []
    for (environment, lambda_value, guard_k), rows in sorted(grouped.items()):
        triggered = [row for row in rows if row["stop_triggered"]]
        rebound_runs = [row for row in triggered if row["post_stop_rebound_count"] > 0]

        output.append(
            {
                "environment": environment,
                "lambda_value": lambda_value,
                "guard_k": guard_k,
                "run_count": len(rows),
                "trigger_count": len(triggered),
                "trigger_rate": len(triggered) / len(rows) if rows else math.nan,
                "median_stop_fraction": _median(
                    [row["stop_fraction_of_evaluable"] for row in triggered]
                ),
                "median_time_saved_fraction": _median(
                    [row["time_saved_fraction"] for row in triggered]
                ),
                "median_distance_saved_fraction": _median(
                    [row["distance_saved_fraction"] for row in triggered]
                ),
                "runs_with_any_rebound": len(rebound_runs),
                "rebound_run_rate": (
                    len(rebound_runs) / len(triggered) if triggered else math.nan
                ),
                "median_post_stop_rebound_fraction": _median(
                    [row["post_stop_rebound_fraction"] for row in triggered]
                ),
                "median_max_post_stop_excess": _median(
                    [row["max_post_stop_excess"] for row in triggered]
                ),
            }
        )
    return output


def _write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: _fmt(row.get(field)) for field in fields})


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sensitivity analysis for Way2 lambda with raw G, Euclidean C, K guard."
    )
    parser.add_argument(
        "runs",
        nargs="+",
        help="Run IDs (e.g. mpx_001) or explicit MapEx run directories.",
    )
    parser.add_argument(
        "--lambdas",
        type=float,
        nargs="+",
        default=list(DEFAULT_LAMBDAS),
        help=(
            "Declared sensitivity grid. Default: "
            + " ".join(str(value) for value in DEFAULT_LAMBDAS)
        ),
    )
    parser.add_argument(
        "--guard-k",
        type=int,
        default=2,
        help="Distinct non-positive states required before stop (default: 2).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory. Default: mapex_lab/experiments/mapex.",
    )
    args = parser.parse_args()

    if args.guard_k < 1:
        parser.error("--guard-k must be >= 1")
    if not args.lambdas:
        parser.error("at least one lambda is required")
    if any(value < 0.0 or not math.isfinite(value) for value in args.lambdas):
        parser.error("all lambda values must be finite and >= 0")

    lambdas = sorted(set(float(value) for value in args.lambdas))
    root = Path(__file__).resolve().parents[1]
    output_dir = args.output_dir or (root / "experiments" / "mapex")

    run_dirs = [_resolve_run(root, value) for value in args.runs]
    run_rows = []
    for run_dir in run_dirs:
        for lambda_value in lambdas:
            result = _analyze_lambda_for_run(run_dir, lambda_value, args.guard_k)
            run_rows.append(result)

    env_rows = _environment_rows(run_rows)

    run_csv = output_dir / "way2_lambda_sensitivity_runs.csv"
    env_csv = output_dir / "way2_lambda_sensitivity_environments.csv"
    json_path = output_dir / "way2_lambda_sensitivity.json"

    _write_csv(run_csv, RUN_FIELDS, run_rows)
    _write_csv(env_csv, ENV_FIELDS, env_rows)
    json_path.write_text(
        json.dumps(
            {
                "development_only": True,
                "gain_definition": "recorded raw information_gain",
                "cost_definition": "recorded Euclidean distance_m",
                "guard_k": args.guard_k,
                "lambda_grid": lambdas,
                "interpretation": {
                    "stop_condition": "R_t=max_f(G/C) <= lambda for K distinct states",
                    "rebound": "a later evaluable decision has R_t > lambda",
                    "time_distance_savings": (
                        "reconstructed from metrics.csv at/before hypothetical stop time; "
                        "diagnostic only"
                    ),
                    "selection_rule": (
                        "this script does not choose a final lambda; inspect cross-environment "
                        "trigger timing, savings, and rebound risk before freezing one"
                    ),
                },
                "environments": env_rows,
                "runs": run_rows,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    print("Way2 lambda sensitivity (K=%d)" % args.guard_k)
    print("lambda grid:", ", ".join(f"{value:g}" for value in lambdas))
    print("\nEnvironment summary:")
    for row in env_rows:
        trigger_rate = row["trigger_rate"]
        time_saved = row["median_time_saved_fraction"]
        distance_saved = row["median_distance_saved_fraction"]
        rebound_rate = row["rebound_run_rate"]
        print(
            f"  {row['environment']:9s} lambda={row['lambda_value']:.3g}: "
            f"trigger={row['trigger_count']}/{row['run_count']} "
            f"({trigger_rate*100:.1f}%), "
            f"median time saved={time_saved*100:.1f}% "
            if time_saved is not None and math.isfinite(time_saved)
            else f"  {row['environment']:9s} lambda={row['lambda_value']:.3g}: "
                 f"trigger={row['trigger_count']}/{row['run_count']} "
                 f"({trigger_rate*100:.1f}%), median time saved=n/a "
        , end="")
        if distance_saved is not None and math.isfinite(distance_saved):
            print(f"distance saved={distance_saved*100:.1f}% ", end="")
        else:
            print("distance saved=n/a ", end="")
        if rebound_rate is not None and math.isfinite(rebound_rate):
            print(f"rebound runs={rebound_rate*100:.1f}%")
        else:
            print("rebound runs=n/a")

    print(f"\nWrote {run_csv}")
    print(f"Wrote {env_csv}")
    print(f"Wrote {json_path}")
    print(
        "Conclusion: this is sensitivity analysis only. Do not freeze lambda until "
        "cross-environment stop timing and rebound risk have been reviewed."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
