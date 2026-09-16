#!/usr/bin/env python3
"""Offline Way2 utility replay for recorded MapEx runs.

This tool does not change the online MapEx controller. It replays the per-frontier
quantities already stored in ``candidates.csv`` and computes the one-step Way2
utility

    V_t(f) = G_t(f) - lambda * C_t(f)

over candidates that were genuinely selectable by the recorded online policy.

Current baseline semantics:
- G_t(f): recorded ``information_gain`` from MapEx;
- C_t(f): recorded Euclidean ``distance_m``;
- eligibility: recorded ``selectable == 1``;
- planner-path cost is intentionally NOT reconstructed from ``plans.csv`` because
  current recordings do not contain a planner path for every candidate frontier.

Without ``--lambda-value`` the script still produces the per-decision break-even
lambda

    lambda_break_even = max_f G_t(f) / C_t(f)

which is the largest lambda for which at least one selectable frontier has
positive utility. This is useful for formulation audit without prematurely
freezing a lambda.

A normalized gain can be inspected with ``--gain-normalization variance-reference``.
That simply divides the recorded summed information gain by a declared theoretical
variance reference. No normalization constant is assumed by default.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Iterable


OUTPUT_FIELDS = [
    "run_id",
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
    """Stable signature used so the K guard counts distinct recorded states only."""
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
    summary = {
        "run_id": run_dir.name,
        "source": str(candidates_path),
        "decision_count": len(output_rows),
        "evaluable_decision_count": len(evaluable),
        "gain_definition": "recorded MapEx information_gain",
        "gain_normalization": gain_normalization,
        "variance_reference": variance_reference,
        "cost_definition": "recorded Euclidean distance_m",
        "eligibility_definition": "recorded selectable == 1",
        "planner_path_cost_supported_by_current_candidate_recording": False,
        "score_formula_mismatch_count": score_formula_mismatch_count,
        "selection_audit_count": selection_audit_count,
        "selection_mismatch_count": selection_mismatch_count,
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


def _write_outputs(run_dir: Path, rows: list[dict[str, str]], summary: dict) -> None:
    csv_path = run_dir / "way2_utility_replay.csv"
    json_path = run_dir / "way2_utility_summary.json"

    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")


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
        "--no-write",
        action="store_true",
        help=(
            "Analyze and print summaries without writing replay files into run "
            "directories."
        ),
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
    for value in args.runs:
        run_dir = _resolve_run(root, value)
        rows, summary = analyze_run(
            run_dir=run_dir,
            lambda_value=args.lambda_value,
            guard_k=args.guard_k,
            gain_normalization=args.gain_normalization,
            variance_reference=args.variance_reference,
        )
        if not args.no_write:
            _write_outputs(run_dir, rows, summary)

        break_even = [
            _finite_float(row["lambda_break_even"])
            for row in rows
            if row["lambda_break_even"] not in ("", "nan")
        ]
        finite_break_even = [value for value in break_even if math.isfinite(value)]
        range_text = "n/a"
        if finite_break_even:
            range_text = (
                f"{min(finite_break_even):.6f}..{max(finite_break_even):.6f}"
            )

        stop_text = summary["first_guarded_stop_decision_id"]
        print(
            "WAY2 REPLAY: "
            f"run={run_dir.name}, decisions={summary['decision_count']}, "
            f"evaluable={summary['evaluable_decision_count']}, "
            f"score_mismatch={summary['score_formula_mismatch_count']}, "
            f"selection_mismatch={summary['selection_mismatch_count']}/"
            f"{summary['selection_audit_count']}, "
            f"lambda_break_even_range={range_text}, "
            f"lambda={args.lambda_value}, guard_k={args.guard_k}, "
            f"first_guarded_stop={stop_text}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
