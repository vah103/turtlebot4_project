#!/usr/bin/env python3
"""Audit the raw Way2 gain signal before freezing normalization or lambda.

This script is intentionally diagnostic. It does not modify the online MapEx
controller and it does not choose a stopping lambda.

For each recorded decision it inspects the best selectable frontier under the
current MapEx ratio G/C, where:
    G = recorded information_gain
    C = recorded Euclidean distance_m

The audit is designed to answer three questions:
1. Are extreme G/C values caused by unusually large gain, unusually small
   distance, or simply a very large visible unknown region?
2. Is raw information_gain on a comparable scale across environments?
3. Why do some recorded decisions have no selectable frontier?

Because MapEx information_gain is a sum over predicted-visible unknown cells,
the diagnostic quantity

    gain_per_visible_unknown = information_gain / visible_unknown_cells

is also reported. It is only a diagnostic decomposition, NOT a proposed
replacement gain and NOT an automatic normalization.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path


AUDIT_FIELDS = [
    "run_id",
    "environment",
    "decision_id",
    "candidate_total",
    "candidate_selectable",
    "status",
    "best_candidate_id",
    "best_gain",
    "best_distance_m",
    "best_lambda_break_even",
    "best_visible_unknown_cells",
    "best_gain_per_visible_unknown",
    "recorded_selected_candidate_id",
    "execution_suppressed_count",
    "planner_suppressed_count",
    "suppressed_planner_blocked_count",
    "distance_ineligible_count",
    "below_1m_count",
    "candidate_status_counts",
]

NO_SELECTABLE_FIELDS = [
    "run_id",
    "environment",
    "decision_id",
    "candidate_total",
    "execution_suppressed_count",
    "planner_suppressed_count",
    "suppressed_planner_blocked_count",
    "distance_ineligible_count",
    "below_1m_count",
    "candidate_status_counts",
    "diagnostic_reason",
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


def _median(values: list[float]) -> float | None:
    finite = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.median(finite) if finite else None


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


def _load_environment(run_dir: Path) -> str:
    path = run_dir / "metadata.json"
    if not path.is_file():
        return "unknown"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return "unknown"
    if not isinstance(payload, dict):
        return "unknown"
    return str(payload.get("environment") or "unknown")


def _decision_ids_from_decisions_csv(run_dir: Path) -> set[int]:
    path = run_dir / "decisions.csv"
    if not path.is_file():
        return set()
    result: set[int] = set()
    for row in _read_csv(path):
        decision_id = _int_value(row.get("decision_id"), -1)
        if decision_id >= 0:
            result.add(decision_id)
    return result


def _validate_schema(fieldnames: list[str] | None, path: Path) -> None:
    fields = set(fieldnames or [])
    required = {
        "decision_id",
        "candidate_id",
        "information_gain",
        "distance_m",
        "visible_unknown_cells",
        "selectable",
    }
    missing = sorted(required - fields)
    if missing:
        raise ValueError(
            f"{path} is missing required gain-audit fields: {', '.join(missing)}"
        )


def _suppression_summary(rows: list[dict[str, str]]) -> dict:
    statuses = Counter(str(row.get("status") or "unknown") for row in rows)
    execution_suppressed = sum(
        _int_value(row.get("execution_suppressed"), 0) == 1 for row in rows
    )
    planner_suppressed = sum(
        _int_value(row.get("planner_suppressed"), 0) == 1 for row in rows
    )
    planner_blocked = sum(
        _int_value(row.get("suppressed_planner_blocked"), 0) == 1 for row in rows
    )
    distance_ineligible = sum(
        _int_value(row.get("distance_eligible"), 1) == 0 for row in rows
    )
    below_1m = sum(_int_value(row.get("below_1m"), 0) == 1 for row in rows)

    return {
        "execution_suppressed_count": execution_suppressed,
        "planner_suppressed_count": planner_suppressed,
        "suppressed_planner_blocked_count": planner_blocked,
        "distance_ineligible_count": distance_ineligible,
        "below_1m_count": below_1m,
        "candidate_status_counts": json.dumps(dict(sorted(statuses.items()))),
    }


def _no_selectable_reason(rows: list[dict[str, str]], summary: dict) -> str:
    if not rows:
        return "no_candidate_rows"
    if summary["execution_suppressed_count"] == len(rows):
        return "all_execution_suppressed"
    if summary["planner_suppressed_count"] == len(rows):
        return "all_planner_suppressed"
    if summary["distance_ineligible_count"] == len(rows):
        return "all_distance_ineligible"
    if (
        summary["execution_suppressed_count"]
        or summary["planner_suppressed_count"]
        or summary["distance_ineligible_count"]
    ):
        return "mixed_filter_or_suppression"
    return "no_selectable_flag_without_obvious_filter"


def analyze_run(run_dir: Path) -> tuple[list[dict[str, str]], dict, list[dict]]:
    candidates_path = run_dir / "candidates.csv"
    if not candidates_path.is_file():
        raise FileNotFoundError(f"Missing candidates.csv: {candidates_path}")

    with candidates_path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        _validate_schema(reader.fieldnames, candidates_path)
        candidate_rows = list(reader)

    environment = _load_environment(run_dir)

    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in candidate_rows:
        decision_id = _int_value(row.get("decision_id"), -1)
        if decision_id >= 0:
            grouped[decision_id].append(row)

    decision_ids = set(grouped) | _decision_ids_from_decisions_csv(run_dir)

    audit_rows: list[dict[str, str]] = []
    no_selectable_rows: list[dict] = []

    best_gain_values: list[float] = []
    best_distance_values: list[float] = []
    best_lambda_values: list[float] = []
    best_visible_values: list[float] = []
    best_gain_per_visible_values: list[float] = []

    for decision_id in sorted(decision_ids):
        all_rows = grouped.get(decision_id, [])
        suppression = _suppression_summary(all_rows)
        selectable = [
            row
            for row in all_rows
            if _int_value(row.get("selectable"), 0) == 1
        ]

        parsed = []
        for row in selectable:
            gain = _finite_float(row.get("information_gain"))
            distance = _finite_float(row.get("distance_m"))
            visible = _finite_float(row.get("visible_unknown_cells"))
            if (
                not math.isfinite(gain)
                or not math.isfinite(distance)
                or distance <= 0.0
                or not math.isfinite(visible)
                or visible < 0.0
            ):
                continue

            parsed.append(
                {
                    "candidate_id": str(row.get("candidate_id", "")),
                    "gain": gain,
                    "distance": distance,
                    "lambda_break_even": gain / distance,
                    "visible": visible,
                    "gain_per_visible": (
                        gain / visible if visible > 0.0 else math.nan
                    ),
                }
            )

        selected_rows = [
            row for row in all_rows if _int_value(row.get("selected"), 0) == 1
        ]
        selected_candidate_id = (
            str(selected_rows[0].get("candidate_id", ""))
            if len(selected_rows) == 1
            else ""
        )

        if parsed:
            best = max(parsed, key=lambda item: item["lambda_break_even"])
            status = "ok"

            best_gain_values.append(best["gain"])
            best_distance_values.append(best["distance"])
            best_lambda_values.append(best["lambda_break_even"])
            best_visible_values.append(best["visible"])
            if math.isfinite(best["gain_per_visible"]):
                best_gain_per_visible_values.append(best["gain_per_visible"])

            best_candidate_id = best["candidate_id"]
            best_gain = _fmt(best["gain"])
            best_distance = _fmt(best["distance"])
            best_lambda = _fmt(best["lambda_break_even"])
            best_visible = _fmt(best["visible"])
            best_gain_per_visible = _fmt(best["gain_per_visible"])
        else:
            status = "no_selectable_candidate"
            best_candidate_id = ""
            best_gain = ""
            best_distance = ""
            best_lambda = ""
            best_visible = ""
            best_gain_per_visible = ""

            diagnostic_reason = _no_selectable_reason(all_rows, suppression)
            no_selectable_rows.append(
                {
                    "run_id": run_dir.name,
                    "environment": environment,
                    "decision_id": decision_id,
                    "candidate_total": len(all_rows),
                    **suppression,
                    "diagnostic_reason": diagnostic_reason,
                }
            )

        audit_rows.append(
            {
                "run_id": run_dir.name,
                "environment": environment,
                "decision_id": str(decision_id),
                "candidate_total": str(len(all_rows)),
                "candidate_selectable": str(len(parsed)),
                "status": status,
                "best_candidate_id": best_candidate_id,
                "best_gain": best_gain,
                "best_distance_m": best_distance,
                "best_lambda_break_even": best_lambda,
                "best_visible_unknown_cells": best_visible,
                "best_gain_per_visible_unknown": best_gain_per_visible,
                "recorded_selected_candidate_id": selected_candidate_id,
                **{key: str(value) for key, value in suppression.items()},
            }
        )

    evaluable_rows = [row for row in audit_rows if row["status"] == "ok"]
    max_lambda_row = (
        max(
            evaluable_rows,
            key=lambda row: _finite_float(row["best_lambda_break_even"], -math.inf),
        )
        if evaluable_rows
        else None
    )

    summary = {
        "run_id": run_dir.name,
        "environment": environment,
        "decision_count": len(audit_rows),
        "evaluable_decision_count": len(evaluable_rows),
        "no_selectable_decision_count": len(no_selectable_rows),
        "no_selectable_decision_ids": [
            row["decision_id"] for row in no_selectable_rows
        ],
        "best_frontier_medians": {
            "information_gain": _median(best_gain_values),
            "distance_m": _median(best_distance_values),
            "lambda_break_even": _median(best_lambda_values),
            "visible_unknown_cells": _median(best_visible_values),
            "gain_per_visible_unknown": _median(best_gain_per_visible_values),
        },
        "max_lambda_break_even_decision": (
            None
            if max_lambda_row is None
            else {
                "decision_id": _int_value(max_lambda_row["decision_id"], -1),
                "candidate_id": max_lambda_row["best_candidate_id"],
                "information_gain": _finite_float(max_lambda_row["best_gain"]),
                "distance_m": _finite_float(max_lambda_row["best_distance_m"]),
                "lambda_break_even": _finite_float(
                    max_lambda_row["best_lambda_break_even"]
                ),
                "visible_unknown_cells": _finite_float(
                    max_lambda_row["best_visible_unknown_cells"]
                ),
                "gain_per_visible_unknown": _finite_float(
                    max_lambda_row["best_gain_per_visible_unknown"]
                ),
            }
        ),
        "diagnostic_note": (
            "gain_per_visible_unknown is a decomposition diagnostic only; it is "
            "not automatically proposed as the Way2 gain normalization."
        ),
        "development_only": True,
    }
    return audit_rows, summary, no_selectable_rows


def _environment_summary(audit_rows: list[dict[str, str]]) -> dict[str, dict]:
    grouped: dict[str, list[dict[str, str]]] = defaultdict(list)
    for row in audit_rows:
        if row["status"] == "ok":
            grouped[row["environment"]].append(row)

    result: dict[str, dict] = {}
    for environment, rows in sorted(grouped.items()):
        gains = [_finite_float(row["best_gain"]) for row in rows]
        distances = [_finite_float(row["best_distance_m"]) for row in rows]
        lambdas = [_finite_float(row["best_lambda_break_even"]) for row in rows]
        visibles = [_finite_float(row["best_visible_unknown_cells"]) for row in rows]
        gain_per_visible = [
            _finite_float(row["best_gain_per_visible_unknown"]) for row in rows
        ]

        result[environment] = {
            "decision_count": len(rows),
            "median_best_information_gain": _median(gains),
            "median_best_distance_m": _median(distances),
            "median_lambda_break_even": _median(lambdas),
            "max_lambda_break_even": (
                max(v for v in lambdas if math.isfinite(v))
                if any(math.isfinite(v) for v in lambdas)
                else None
            ),
            "median_best_visible_unknown_cells": _median(visibles),
            "median_best_gain_per_visible_unknown": _median(gain_per_visible),
        }
    return result


def _top_outliers(audit_rows: list[dict[str, str]], top_k: int) -> list[dict]:
    evaluable = [row for row in audit_rows if row["status"] == "ok"]
    ranked = sorted(
        evaluable,
        key=lambda row: _finite_float(row["best_lambda_break_even"], -math.inf),
        reverse=True,
    )
    output = []
    for rank, row in enumerate(ranked[:top_k], start=1):
        output.append(
            {
                "rank": rank,
                "run_id": row["run_id"],
                "environment": row["environment"],
                "decision_id": _int_value(row["decision_id"], -1),
                "candidate_id": row["best_candidate_id"],
                "information_gain": _finite_float(row["best_gain"]),
                "distance_m": _finite_float(row["best_distance_m"]),
                "lambda_break_even": _finite_float(row["best_lambda_break_even"]),
                "visible_unknown_cells": _finite_float(
                    row["best_visible_unknown_cells"]
                ),
                "gain_per_visible_unknown": _finite_float(
                    row["best_gain_per_visible_unknown"]
                ),
            }
        )
    return output


def _write_csv(path: Path, fieldnames: list[str], rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Audit Way2 raw information gain before normalization/lambda freeze."
    )
    parser.add_argument(
        "runs",
        nargs="+",
        help="Run IDs (e.g. mpx_001) or explicit MapEx run directories.",
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=10,
        help="Number of largest G/C decisions to report (default: 10).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory. Default: mapex_lab/experiments/mapex.",
    )
    parser.add_argument(
        "--no-write",
        action="store_true",
        help="Print diagnostics without writing audit CSV/JSON files.",
    )
    args = parser.parse_args()

    if args.top_k < 1:
        parser.error("--top-k must be >= 1")

    root = Path(__file__).resolve().parents[1]
    all_rows: list[dict[str, str]] = []
    summaries: list[dict] = []
    no_selectable_rows: list[dict] = []

    for value in args.runs:
        run_dir = _resolve_run(root, value)
        rows, summary, no_selectable = analyze_run(run_dir)
        all_rows.extend(rows)
        summaries.append(summary)
        no_selectable_rows.extend(no_selectable)

        outlier = summary["max_lambda_break_even_decision"]
        if outlier is None:
            outlier_text = "n/a"
        else:
            outlier_text = (
                f"d{outlier['decision_id']}: "
                f"G={outlier['information_gain']:.6f}, "
                f"C={outlier['distance_m']:.6f}, "
                f"G/C={outlier['lambda_break_even']:.6f}, "
                f"visible={outlier['visible_unknown_cells']:.0f}, "
                f"G/visible={outlier['gain_per_visible_unknown']:.6f}"
            )

        print(
            "WAY2 G AUDIT: "
            f"run={summary['run_id']}, env={summary['environment']}, "
            f"evaluable={summary['evaluable_decision_count']}/"
            f"{summary['decision_count']}, "
            f"no_selectable={summary['no_selectable_decision_count']}, "
            f"max={outlier_text}"
        )

    environment_summary = _environment_summary(all_rows)
    for environment, values in environment_summary.items():
        print(
            "WAY2 G ENV: "
            f"env={environment}, decisions={values['decision_count']}, "
            f"median_G={values['median_best_information_gain']}, "
            f"median_C={values['median_best_distance_m']}, "
            f"median_visible={values['median_best_visible_unknown_cells']}, "
            f"median_G_per_visible={values['median_best_gain_per_visible_unknown']}, "
            f"median_G_over_C={values['median_lambda_break_even']}, "
            f"max_G_over_C={values['max_lambda_break_even']}"
        )

    top_outliers = _top_outliers(all_rows, args.top_k)
    print(f"WAY2 G TOP {len(top_outliers)}:")
    for item in top_outliers:
        print(
            f"  #{item['rank']} {item['run_id']} d{item['decision_id']} "
            f"G={item['information_gain']:.6f} "
            f"C={item['distance_m']:.6f} "
            f"G/C={item['lambda_break_even']:.6f} "
            f"visible={item['visible_unknown_cells']:.0f} "
            f"G/visible={item['gain_per_visible_unknown']:.6f}"
        )

    if no_selectable_rows:
        print(f"WAY2 G NO-SELECTABLE: {len(no_selectable_rows)} decisions")
        for row in no_selectable_rows:
            print(
                f"  {row['run_id']} d{row['decision_id']}: "
                f"candidates={row['candidate_total']}, "
                f"reason={row['diagnostic_reason']}, "
                f"exec_supp={row['execution_suppressed_count']}, "
                f"planner_supp={row['planner_suppressed_count']}, "
                f"distance_ineligible={row['distance_ineligible_count']}, "
                f"statuses={row['candidate_status_counts']}"
            )

    if not args.no_write:
        output_dir = (
            args.output_dir.resolve()
            if args.output_dir is not None
            else root / "experiments" / "mapex"
        )
        _write_csv(output_dir / "way2_gain_audit.csv", AUDIT_FIELDS, all_rows)
        _write_csv(
            output_dir / "way2_gain_no_selectable.csv",
            NO_SELECTABLE_FIELDS,
            no_selectable_rows,
        )
        payload = {
            "run_count": len(summaries),
            "runs": summaries,
            "environments": environment_summary,
            "top_lambda_break_even_outliers": top_outliers,
            "no_selectable_decision_count": len(no_selectable_rows),
            "interpretation_guardrail": (
                "This audit diagnoses raw G, C, visible-region size, and filtering. "
                "It does not choose lambda and does not declare G/visible to be the "
                "final Way2 gain."
            ),
        }
        (output_dir / "way2_gain_audit_summary.json").write_text(
            json.dumps(payload, indent=2),
            encoding="utf-8",
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
