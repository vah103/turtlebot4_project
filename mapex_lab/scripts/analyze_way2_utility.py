#!/usr/bin/env python3
"""Offline Way2 utility replay for recorded MapEx runs.

This tool does not change the online MapEx controller. It replays the per-frontier
quantities already stored in ``candidates.csv`` and computes the one-step Way2
utility

    V_t(f) = G_t(f) - lambda * C_t(f)

over candidates that were selectable by the recorded online policy.

Baseline semantics:
- G_t(f): recorded ``information_gain`` from MapEx;
- C_t(f): recorded Euclidean ``distance_m``;
- eligibility: recorded ``selectable == 1``;
- planner-path cost is intentionally NOT reconstructed from ``plans.csv`` because
  current recordings do not contain a planner path for every candidate frontier.

Without ``--lambda-value`` the script reports the per-decision break-even boundary

    lambda_break_even = max_f G_t(f) / C_t(f)

At lambda < lambda_break_even at least one selectable frontier has positive utility;
at lambda >= lambda_break_even the best one-step utility is non-positive (equality
at the boundary). This permits formulation audit without prematurely freezing a
lambda.

The script also summarizes early-vs-late break-even behavior for every run and,
when multiple runs are supplied, compares the scale across environments.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Iterable


OUTPUT_FIELDS = [
    "run_id",
    "environment",
    "decision_id",
    "time_s",
    "candidate_total",
    "candidate_selectable",
    "gain_normalization",
    "variance_reference",
    "cost_definition",
    "recorded_selected_candidate_id",
    "best_ratio_candidate_id",
    "best_ratio_gain",
    "best_ratio_cost_m",
    "lambda_break_even",
    "ratio_matches_recorded_selection",
    "lambda_value",
    "best_utility_candidate_id",
    "best_utility_gain",
    "best_utility_cost_m",
    "max_utility",
    "utility_nonpositive",
    "guard_state_counted",
    "guard_streak",
    "guard_k",
    "guarded_stop",
    "status",
]

AGGREGATE_FIELDS = [
    "run_id",
    "environment",
    "decision_count",
    "evaluable_decision_count",
    "lambda_break_even_min",
    "lambda_break_even_max",
    "lambda_break_even_median",
    "lambda_break_even_first",
    "lambda_break_even_last",
    "lambda_break_even_early_median",
    "lambda_break_even_late_median",
    "lambda_late_to_early_ratio",
    "lambda_late_change_fraction",
    "score_formula_mismatch_count",
    "selection_mismatch_count",
    "selection_audit_count",
    "first_guarded_stop_decision_id",
]


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _finite_float(value, default=math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _int_value(value, default=0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


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
    if math.isnan(number):
        return "nan"
    if math.isinf(number):
        return "inf" if number > 0 else "-inf"
    return f"{number:.9f}"


def _resolve_run(root: Path, value: str) -> Path:
    direct = Path(value).expanduser()
    if direct.is_dir():
        return direct.resolve()

    candidate = root / "experiments" / "mapex" / value
    if candidate.is_dir():
        return candidate.resolve()

    raise FileNotFoundError(
        f"Cannot resolve run '{value}'. Expected a run directory or {candidate}"
    )


def _load_metadata(run_dir: Path) -> dict:
    path = run_dir / "metadata.json"
    if not path.is_file():
        return {}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return value if isinstance(value, dict) else {}


def _load_decision_times(run_dir: Path) -> dict[int, float]:
    path = run_dir / "decisions.csv"
    if not path.is_file():
        return {}
    result: dict[int, float] = {}
    for row in _read_csv(path):
        decision_id = _int_value(row.get("decision_id"), -1)
        if decision_id < 0:
            continue
        result[decision_id] = _finite_float(row.get("time_s"))
    return result


def _validate_candidate_schema(fieldnames: Iterable[str] | None, path: Path) -> None:
    fields = set(fieldnames or [])
    required = {
        "decision_id",
        "candidate_id",
        "information_gain",
        "distance_m",
        "selectable",
    }
    missing = sorted(required - fields)
    if missing:
        raise ValueError(
            f"{path} is missing required Way2 replay fields: {', '.join(missing)}"
        )


def _normalized_gain(
    raw_gain: float,
    mode: str,
    variance_reference: float | None,
) -> float:
    if mode == "raw":
        return raw_gain
    if mode == "variance-reference":
        assert variance_reference is not None
        return raw_gain / variance_reference
    raise ValueError(f"Unsupported gain normalization: {mode}")


def _decision_signature(rows: list[dict[str, str]]) -> tuple:
    """Stable signature so the K guard counts distinct recorded states only."""
    signature_rows = []
    for row in rows:
        signature_rows.append(
            (
                row.get("candidate_id", ""),
                row.get("row", ""),
                row.get("col", ""),
                row.get("information_gain", ""),
                row.get("distance_m", ""),
                row.get("selectable", ""),
                row.get("status", ""),
            )
        )
    return tuple(sorted(signature_rows))


def _break_even_summary(values: list[float]) -> dict[str, float]:
    finite = [float(v) for v in values if math.isfinite(float(v))]
    if not finite:
        return {
            "min": math.nan,
            "max": math.nan,
            "median": math.nan,
            "first": math.nan,
            "last": math.nan,
            "early_median": math.nan,
            "late_median": math.nan,
            "late_to_early_ratio": math.nan,
            "late_change_fraction": math.nan,
        }

    quartile_n = max(1, math.ceil(len(finite) * 0.25))
    early = finite[:quartile_n]
    late = finite[-quartile_n:]
    early_median = statistics.median(early)
    late_median = statistics.median(late)
    ratio = late_median / early_median if early_median != 0.0 else math.nan
    change = (
        (late_median - early_median) / early_median
        if early_median != 0.0
        else math.nan
    )
    return {
        "min": min(finite),
        "max": max(finite),
        "median": statistics.median(finite),
        "first": finite[0],
        "last": finite[-1],
        "early_median": early_median,
        "late_median": late_median,
        "late_to_early_ratio": ratio,
        "late_change_fraction": change,
    }


def analyze_run(
    run_dir: Path,
    lambda_value: float | None,
    guard_k: int,
    gain_normalization: str,
    variance_reference: float | None,
) -> tuple[list[dict[str, str]], dict]:
    candidates_path = run_dir / "candidates.csv"
    if not candidates_path.is_file():
        raise FileNotFoundError(f"Missing candidates.csv: {candidates_path}")

    metadata = _load_metadata(run_dir)
    environment = str(metadata.get("environment") or "unknown")

    with candidates_path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        _validate_candidate_schema(reader.fieldnames, candidates_path)
        candidate_rows = list(reader)

    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in candidate_rows:
        decision_id = _int_value(row.get("decision_id"), -1)
        if decision_id >= 0:
            grouped[decision_id].append(row)

    decision_times = _load_decision_times(run_dir)
    output_rows: list[dict[str, str]] = []
    guard_streak = 0
    last_counted_nonpositive_signature = None
    first_guarded_stop_decision_id: int | None = None
    score_formula_mismatch_count = 0
    selection_mismatch_count = 0
    selection_audit_count = 0

    for decision_id in sorted(grouped):
        all_candidates = grouped[decision_id]
        state_signature = _decision_signature(all_candidates)
        selectable = [
            row
            for row in all_candidates
            if _int_value(row.get("selectable"), 0) == 1
        ]

        parsed = []
        for row in selectable:
            raw_gain = _finite_float(row.get("information_gain"))
            cost = _finite_float(row.get("distance_m"))
            if not math.isfinite(raw_gain) or not math.isfinite(cost) or cost <= 0.0:
                continue

            recorded_score = _finite_float(row.get("score"))
            raw_ratio = raw_gain / cost
            if math.isfinite(recorded_score) and not math.isclose(
                raw_ratio,
                recorded_score,
                rel_tol=1e-6,
                abs_tol=1e-6,
            ):
                score_formula_mismatch_count += 1

            gain = _normalized_gain(raw_gain, gain_normalization, variance_reference)
            parsed.append(
                {
                    "candidate_id": row.get("candidate_id", ""),
                    "gain": gain,
                    "cost": cost,
                    "ratio": gain / cost,
                }
            )

        selected_rows = [
            row for row in all_candidates if _int_value(row.get("selected"), 0) == 1
        ]
        recorded_selected_candidate_id = (
            selected_rows[0].get("candidate_id", "") if len(selected_rows) == 1 else ""
        )

        if not parsed:
            guard_streak = 0
            last_counted_nonpositive_signature = None
            output_rows.append(
                {
                    "run_id": run_dir.name,
                    "environment": environment,
                    "decision_id": str(decision_id),
                    "time_s": _fmt(decision_times.get(decision_id, math.nan)),
                    "candidate_total": str(len(all_candidates)),
                    "candidate_selectable": "0",
                    "gain_normalization": gain_normalization,
                    "variance_reference": _fmt(variance_reference),
                    "cost_definition": "euclidean_distance_m",
                    "recorded_selected_candidate_id": recorded_selected_candidate_id,
                    "best_ratio_candidate_id": "",
                    "best_ratio_gain": "",
                    "best_ratio_cost_m": "",
                    "lambda_break_even": "",
                    "ratio_matches_recorded_selection": "",
                    "lambda_value": _fmt(lambda_value),
                    "best_utility_candidate_id": "",
                    "best_utility_gain": "",
                    "best_utility_cost_m": "",
                    "max_utility": "",
                    "utility_nonpositive": "",
                    "guard_state_counted": "0",
                    "guard_streak": "0",
                    "guard_k": str(guard_k),
                    "guarded_stop": "0",
                    "status": "no_selectable_candidate",
                }
            )
            continue

        best_ratio = max(parsed, key=lambda item: item["ratio"])
        lambda_break_even = best_ratio["ratio"]

        selection_match = ""
        if recorded_selected_candidate_id:
            selection_audit_count += 1
            matches = str(best_ratio["candidate_id"]) == str(
                recorded_selected_candidate_id
            )
            selection_match = "1" if matches else "0"
            if not matches:
                selection_mismatch_count += 1

        best_utility = None
        max_utility = math.nan
        utility_nonpositive = None
        guard_state_counted = False
        guarded_stop = False

        if lambda_value is not None:
            for item in parsed:
                item["utility"] = item["gain"] - lambda_value * item["cost"]
            best_utility = max(parsed, key=lambda item: item["utility"])
            max_utility = float(best_utility["utility"])
            utility_nonpositive = max_utility <= 0.0

            if utility_nonpositive:
                if state_signature != last_counted_nonpositive_signature:
                    guard_streak += 1
                    guard_state_counted = True
                    last_counted_nonpositive_signature = state_signature
            else:
                guard_streak = 0
                last_counted_nonpositive_signature = None

            guarded_stop = guard_streak >= guard_k
            if guarded_stop and first_guarded_stop_decision_id is None:
                first_guarded_stop_decision_id = decision_id
        else:
            guard_streak = 0
            last_counted_nonpositive_signature = None

        output_rows.append(
            {
                "run_id": run_dir.name,
                "environment": environment,
                "decision_id": str(decision_id),
                "time_s": _fmt(decision_times.get(decision_id, math.nan)),
                "candidate_total": str(len(all_candidates)),
                "candidate_selectable": str(len(parsed)),
                "gain_normalization": gain_normalization,
                "variance_reference": _fmt(variance_reference),
                "cost_definition": "euclidean_distance_m",
                "recorded_selected_candidate_id": recorded_selected_candidate_id,
                "best_ratio_candidate_id": str(best_ratio["candidate_id"]),
                "best_ratio_gain": _fmt(best_ratio["gain"]),
                "best_ratio_cost_m": _fmt(best_ratio["cost"]),
                "lambda_break_even": _fmt(lambda_break_even),
                "ratio_matches_recorded_selection": selection_match,
                "lambda_value": _fmt(lambda_value),
                "best_utility_candidate_id": (
                    "" if best_utility is None else str(best_utility["candidate_id"])
                ),
                "best_utility_gain": (
                    "" if best_utility is None else _fmt(best_utility["gain"])
                ),
                "best_utility_cost_m": (
                    "" if best_utility is None else _fmt(best_utility["cost"])
                ),
                "max_utility": "" if best_utility is None else _fmt(max_utility),
                "utility_nonpositive": (
                    ""
                    if utility_nonpositive is None
                    else ("1" if utility_nonpositive else "0")
                ),
                "guard_state_counted": "1" if guard_state_counted else "0",
                "guard_streak": str(guard_streak),
                "guard_k": str(guard_k),
                "guarded_stop": "1" if guarded_stop else "0",
                "status": "ok",
            }
        )

    evaluable = [row for row in output_rows if row["status"] == "ok"]
    break_even_values = [
        _finite_float(row["lambda_break_even"])
        for row in evaluable
        if row["lambda_break_even"] not in ("", "nan")
    ]
    trend = _break_even_summary(break_even_values)
    summary = {
        "run_id": run_dir.name,
        "environment": environment,
        "source": str(candidates_path),
        "decision_count": len(output_rows),
        "evaluable_decision_count": len(evaluable),
        "gain_definition": "recorded MapEx information_gain",
        "gain_normalization": gain_normalization,
        "variance_reference": variance_reference,
        "cost_definition": "recorded Euclidean distance_m",
        "eligibility_definition": "recorded policy selectable == 1",
        "eligibility_limit": (
            "selectable means not filtered/suppressed by the recorded policy; "
            "it is not an independent planner-path verification for every candidate"
        ),
        "planner_path_cost_supported_by_current_candidate_recording": False,
        "score_formula_mismatch_count": score_formula_mismatch_count,
        "selection_audit_count": selection_audit_count,
        "selection_mismatch_count": selection_mismatch_count,
        "lambda_break_even": trend,
        "lambda_value": lambda_value,
        "guard_k": guard_k,
        "guard_distinct_state_semantics": (
            "count only non-positive utility decisions with a different recorded "
            "candidate-state signature"
        ),
        "first_guarded_stop_decision_id": first_guarded_stop_decision_id,
        "development_only": True,
        "notes": (
            "Offline Way2 formulation diagnostic only. This does not modify online "
            "MapEx and must not be presented as independent validation."
        ),
    }
    return output_rows, summary


def _summary_row(summary: dict) -> dict[str, str]:
    trend = summary["lambda_break_even"]
    return {
        "run_id": str(summary["run_id"]),
        "environment": str(summary["environment"]),
        "decision_count": str(summary["decision_count"]),
        "evaluable_decision_count": str(summary["evaluable_decision_count"]),
        "lambda_break_even_min": _fmt(trend["min"]),
        "lambda_break_even_max": _fmt(trend["max"]),
        "lambda_break_even_median": _fmt(trend["median"]),
        "lambda_break_even_first": _fmt(trend["first"]),
        "lambda_break_even_last": _fmt(trend["last"]),
        "lambda_break_even_early_median": _fmt(trend["early_median"]),
        "lambda_break_even_late_median": _fmt(trend["late_median"]),
        "lambda_late_to_early_ratio": _fmt(trend["late_to_early_ratio"]),
        "lambda_late_change_fraction": _fmt(trend["late_change_fraction"]),
        "score_formula_mismatch_count": str(summary["score_formula_mismatch_count"]),
        "selection_mismatch_count": str(summary["selection_mismatch_count"]),
        "selection_audit_count": str(summary["selection_audit_count"]),
        "first_guarded_stop_decision_id": _fmt(summary["first_guarded_stop_decision_id"]),
    }


def _environment_summary(summaries: list[dict]) -> dict[str, dict]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for summary in summaries:
        grouped[str(summary["environment"])].append(summary)

    result = {}
    for environment, items in sorted(grouped.items()):
        run_medians = [
            float(item["lambda_break_even"]["median"])
            for item in items
            if math.isfinite(float(item["lambda_break_even"]["median"]))
        ]
        early = [
            float(item["lambda_break_even"]["early_median"])
            for item in items
            if math.isfinite(float(item["lambda_break_even"]["early_median"]))
        ]
        late = [
            float(item["lambda_break_even"]["late_median"])
            for item in items
            if math.isfinite(float(item["lambda_break_even"]["late_median"]))
        ]
        result[environment] = {
            "run_count": len(items),
            "run_median_lambda_median": statistics.median(run_medians) if run_medians else None,
            "run_median_early_lambda": statistics.median(early) if early else None,
            "run_median_late_lambda": statistics.median(late) if late else None,
        }
    return result


def _write_run_outputs(run_dir: Path, rows: list[dict[str, str]], summary: dict) -> None:
    csv_path = run_dir / "way2_utility_replay.csv"
    json_path = run_dir / "way2_utility_summary.json"

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")


def _write_aggregate_outputs(output_dir: Path, summaries: list[dict]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "way2_utility_aggregate.csv"
    json_path = output_dir / "way2_utility_aggregate.json"

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=AGGREGATE_FIELDS)
        writer.writeheader()
        writer.writerows(_summary_row(summary) for summary in summaries)

    payload = {
        "run_count": len(summaries),
        "environments": _environment_summary(summaries),
        "runs": summaries,
        "interpretation": {
            "late_to_early_ratio_lt_1": (
                "late-run break-even lambda is lower than early-run break-even lambda"
            ),
            "cross_environment_goal": (
                "inspect whether New Room and Hospital occupy a compatible lambda scale "
                "before any single lambda is frozen"
            ),
        },
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Replay Way2 one-step frontier utility from recorded MapEx candidates."
    )
    parser.add_argument(
        "runs",
        nargs="+",
        help="Run IDs (e.g. mpx_001) or explicit run directories.",
    )
    parser.add_argument(
        "--lambda-value",
        type=float,
        default=None,
        help=(
            "Declared information-gain/travel-cost exchange rate. If omitted, "
            "only per-decision break-even lambda values are reported."
        ),
    )
    parser.add_argument(
        "--guard-k",
        type=int,
        default=2,
        help=(
            "Consecutive distinct non-positive-utility decision states required "
            "to stop (default: 2)."
        ),
    )
    parser.add_argument(
        "--gain-normalization",
        choices=("raw", "variance-reference"),
        default="raw",
        help="Gain scaling used only for offline formulation analysis.",
    )
    parser.add_argument(
        "--variance-reference",
        type=float,
        default=None,
        help=(
            "Declared theoretical variance reference when "
            "--gain-normalization variance-reference is used."
        ),
    )
    parser.add_argument(
        "--aggregate-output-dir",
        type=Path,
        default=None,
        help=(
            "Directory for multi-run aggregate CSV/JSON. Default: experiments/mapex."
        ),
    )
    parser.add_argument(
        "--no-write",
        action="store_true",
        help="Analyze and print summaries without writing replay files.",
    )
    args = parser.parse_args()

    if args.guard_k < 1:
        parser.error("--guard-k must be >= 1")
    if args.lambda_value is not None and args.lambda_value < 0.0:
        parser.error("--lambda-value must be >= 0")
    if args.gain_normalization == "variance-reference":
        if args.variance_reference is None or args.variance_reference <= 0.0:
            parser.error(
                "--variance-reference must be > 0 with "
                "--gain-normalization variance-reference"
            )
    elif args.variance_reference is not None:
        parser.error(
            "--variance-reference is only valid with "
            "--gain-normalization variance-reference"
        )

    root = Path(__file__).resolve().parents[1]
    summaries = []
    for value in args.runs:
        run_dir = _resolve_run(root, value)
        rows, summary = analyze_run(
            run_dir=run_dir,
            lambda_value=args.lambda_value,
            guard_k=args.guard_k,
            gain_normalization=args.gain_normalization,
            variance_reference=args.variance_reference,
        )
        summaries.append(summary)
        if not args.no_write:
            _write_run_outputs(run_dir, rows, summary)

        trend = summary["lambda_break_even"]
        print(
            "WAY2 REPLAY: "
            f"run={run_dir.name}, env={summary['environment']}, "
            f"decisions={summary['decision_count']}, "
            f"evaluable={summary['evaluable_decision_count']}, "
            f"score_mismatch={summary['score_formula_mismatch_count']}, "
            f"selection_mismatch={summary['selection_mismatch_count']}/"
            f"{summary['selection_audit_count']}, "
            f"lambda_range={trend['min']:.6f}..{trend['max']:.6f}, "
            f"early_median={trend['early_median']:.6f}, "
            f"late_median={trend['late_median']:.6f}, "
            f"late/early={trend['late_to_early_ratio']:.6f}, "
            f"lambda={args.lambda_value}, guard_k={args.guard_k}, "
            f"first_guarded_stop={summary['first_guarded_stop_decision_id']}"
        )

    if len(summaries) > 1:
        env_summary = _environment_summary(summaries)
        for environment, values in env_summary.items():
            print(
                "WAY2 ENV: "
                f"env={environment}, runs={values['run_count']}, "
                f"median_of_run_medians={values['run_median_lambda_median']}, "
                f"median_early={values['run_median_early_lambda']}, "
                f"median_late={values['run_median_late_lambda']}"
            )

        if not args.no_write:
            aggregate_dir = (
                args.aggregate_output_dir.resolve()
                if args.aggregate_output_dir is not None
                else root / "experiments" / "mapex"
            )
            _write_aggregate_outputs(aggregate_dir, summaries)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
