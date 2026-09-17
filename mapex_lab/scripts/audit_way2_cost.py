#!/usr/bin/env python3
"""Audit the travel-cost data available for Way2.

The purpose of this script is narrow: determine whether the historical MapEx
runs contain enough planner-path information to replace the current Euclidean
candidate cost in an offline Way2 replay.

Current recorder semantics:
- candidates.csv stores distance_m for every recorded candidate frontier.
- plans.csv stores /plan diagnostics for the currently active navigation goal.
- /plan can update multiple times while the robot moves, so the planner cost
  closest to the decision state is the first usable main-goal plan recorded for
  that decision.

The audit therefore measures:
1. Euclidean-distance coverage over selectable candidates.
2. Planner-path coverage over selected decisions.
3. Planner-path coverage over all selectable candidates.
4. For selected decisions with a usable initial planner path, the ratio
       initial_planner_path_length / selected_euclidean_distance.

This script is diagnostic only. It does not change the online MapEx policy and
it does not choose the final Way2 travel-cost definition or lambda.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


RUN_FIELDS = [
    "run_id",
    "environment",
    "decision_count",
    "selectable_candidate_count",
    "selectable_with_euclidean_count",
    "euclidean_candidate_coverage",
    "selected_decision_count",
    "selected_with_first_usable_plan_count",
    "selected_planner_coverage",
    "all_candidate_planner_coverage_upper_bound",
    "median_initial_planner_path_m",
    "median_selected_euclidean_m",
    "median_planner_over_euclidean",
    "p90_planner_over_euclidean",
    "max_planner_over_euclidean",
    "multi_plan_selected_decision_count",
]

MATCH_FIELDS = [
    "run_id",
    "environment",
    "decision_id",
    "candidate_id",
    "euclidean_distance_m",
    "first_usable_plan_time_s",
    "first_usable_plan_length_m",
    "planner_over_euclidean",
    "usable_plan_updates_for_decision",
    "frontier_match_error_m",
]


def _read_csv(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        rows = list(reader)
        return rows, list(reader.fieldnames or [])


def _finite_float(value, default=math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _int_value(value, default=-1) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _is_true(value) -> bool:
    return _int_value(value, 0) == 1


def _fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, int):
        return str(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if not math.isfinite(number):
        return "nan"
    return f"{number:.9f}"


def _median(values: list[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    return statistics.median(finite) if finite else None


def _percentile(values: list[float], q: float) -> float | None:
    finite = sorted(v for v in values if math.isfinite(v))
    if not finite:
        return None
    if len(finite) == 1:
        return finite[0]
    pos = (len(finite) - 1) * q
    lo = int(math.floor(pos))
    hi = int(math.ceil(pos))
    if lo == hi:
        return finite[lo]
    weight = pos - lo
    return finite[lo] * (1.0 - weight) + finite[hi] * weight


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


def _validate_schema(path: Path, fields: list[str], required: set[str]) -> None:
    missing = sorted(required - set(fields))
    if missing:
        raise ValueError(f"{path} missing required fields: {', '.join(missing)}")


def analyze_run(run_dir: Path, match_tolerance_m: float) -> tuple[dict, list[dict]]:
    candidates_path = run_dir / "candidates.csv"
    plans_path = run_dir / "plans.csv"
    if not candidates_path.is_file():
        raise FileNotFoundError(f"Missing {candidates_path}")
    if not plans_path.is_file():
        raise FileNotFoundError(f"Missing {plans_path}")

    candidates, candidate_fields = _read_csv(candidates_path)
    plans, plan_fields = _read_csv(plans_path)

    _validate_schema(
        candidates_path,
        candidate_fields,
        {"decision_id", "candidate_id", "x", "y", "distance_m", "selectable", "selected"},
    )
    _validate_schema(
        plans_path,
        plan_fields,
        {
            "time_s",
            "decision_id",
            "path_length_m",
            "frontier_x",
            "frontier_y",
            "endpoint_error_m",
            "usable",
        },
    )

    environment = _load_environment(run_dir)
    decision_ids = {
        _int_value(row.get("decision_id"))
        for row in candidates
        if _int_value(row.get("decision_id")) >= 0
    }

    selectable = [row for row in candidates if _is_true(row.get("selectable"))]
    selectable_with_euclidean = [
        row
        for row in selectable
        if math.isfinite(_finite_float(row.get("distance_m")))
        and _finite_float(row.get("distance_m")) > 0.0
    ]
    selected = [row for row in candidates if _is_true(row.get("selected"))]

    selected_by_decision: dict[int, dict[str, str]] = {}
    for row in selected:
        decision_id = _int_value(row.get("decision_id"))
        if decision_id >= 0:
            selected_by_decision.setdefault(decision_id, row)

    usable_plans_by_decision: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in plans:
        decision_id = _int_value(row.get("decision_id"))
        path_length = _finite_float(row.get("path_length_m"))
        if decision_id < 0 or not _is_true(row.get("usable")):
            continue
        if not math.isfinite(path_length) or path_length <= 0.0:
            continue
        usable_plans_by_decision[decision_id].append(row)

    for rows in usable_plans_by_decision.values():
        rows.sort(key=lambda row: _finite_float(row.get("time_s"), math.inf))

    match_rows: list[dict] = []
    ratios: list[float] = []
    path_lengths: list[float] = []
    euclidean_distances: list[float] = []
    matched_selected_decisions = 0
    multi_plan_selected_decisions = 0

    for decision_id, candidate in sorted(selected_by_decision.items()):
        candidate_x = _finite_float(candidate.get("x"))
        candidate_y = _finite_float(candidate.get("y"))
        euclidean = _finite_float(candidate.get("distance_m"))
        usable_rows = usable_plans_by_decision.get(decision_id, [])

        matched_plans: list[tuple[float, dict[str, str]]] = []
        for plan in usable_rows:
            frontier_x = _finite_float(plan.get("frontier_x"))
            frontier_y = _finite_float(plan.get("frontier_y"))
            if not all(
                math.isfinite(v)
                for v in (candidate_x, candidate_y, frontier_x, frontier_y)
            ):
                continue
            match_error = math.hypot(frontier_x - candidate_x, frontier_y - candidate_y)
            if match_error <= match_tolerance_m:
                matched_plans.append((match_error, plan))

        if not matched_plans:
            continue

        # rows were already time-sorted; keep the earliest matching usable main plan.
        first_match_error, first_plan = matched_plans[0]
        planner_length = _finite_float(first_plan.get("path_length_m"))
        if not math.isfinite(planner_length) or planner_length <= 0.0:
            continue

        ratio = (
            planner_length / euclidean
            if math.isfinite(euclidean) and euclidean > 0.0
            else math.nan
        )

        matched_selected_decisions += 1
        if len(matched_plans) > 1:
            multi_plan_selected_decisions += 1
        if math.isfinite(ratio):
            ratios.append(ratio)
        if math.isfinite(planner_length):
            path_lengths.append(planner_length)
        if math.isfinite(euclidean):
            euclidean_distances.append(euclidean)

        match_rows.append(
            {
                "run_id": run_dir.name,
                "environment": environment,
                "decision_id": decision_id,
                "candidate_id": candidate.get("candidate_id", ""),
                "euclidean_distance_m": _fmt(euclidean),
                "first_usable_plan_time_s": _fmt(
                    _finite_float(first_plan.get("time_s"))
                ),
                "first_usable_plan_length_m": _fmt(planner_length),
                "planner_over_euclidean": _fmt(ratio),
                "usable_plan_updates_for_decision": len(matched_plans),
                "frontier_match_error_m": _fmt(first_match_error),
            }
        )

    selectable_count = len(selectable)
    selected_count = len(selected_by_decision)
    euclidean_coverage = (
        len(selectable_with_euclidean) / selectable_count if selectable_count else 0.0
    )
    selected_planner_coverage = (
        matched_selected_decisions / selected_count if selected_count else 0.0
    )

    # Historical /plan rows belong to the active goal. Even under the generous
    # assumption that every matched selected frontier counts as a candidate with
    # planner cost, this is only an upper bound on all-candidate coverage.
    all_candidate_planner_coverage_upper_bound = (
        matched_selected_decisions / selectable_count if selectable_count else 0.0
    )

    summary = {
        "run_id": run_dir.name,
        "environment": environment,
        "decision_count": len(decision_ids),
        "selectable_candidate_count": selectable_count,
        "selectable_with_euclidean_count": len(selectable_with_euclidean),
        "euclidean_candidate_coverage": euclidean_coverage,
        "selected_decision_count": selected_count,
        "selected_with_first_usable_plan_count": matched_selected_decisions,
        "selected_planner_coverage": selected_planner_coverage,
        "all_candidate_planner_coverage_upper_bound": all_candidate_planner_coverage_upper_bound,
        "median_initial_planner_path_m": _median(path_lengths),
        "median_selected_euclidean_m": _median(euclidean_distances),
        "median_planner_over_euclidean": _median(ratios),
        "p90_planner_over_euclidean": _percentile(ratios, 0.90),
        "max_planner_over_euclidean": max(ratios) if ratios else None,
        "multi_plan_selected_decision_count": multi_plan_selected_decisions,
    }
    return summary, match_rows


def _environment_summary(run_summaries: list[dict]) -> dict[str, dict]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in run_summaries:
        grouped[row["environment"]].append(row)

    result: dict[str, dict] = {}
    for environment, rows in sorted(grouped.items()):
        selectable = sum(int(row["selectable_candidate_count"]) for row in rows)
        euclidean = sum(int(row["selectable_with_euclidean_count"]) for row in rows)
        selected = sum(int(row["selected_decision_count"]) for row in rows)
        planner = sum(int(row["selected_with_first_usable_plan_count"]) for row in rows)
        ratio_medians = [
            row["median_planner_over_euclidean"]
            for row in rows
            if row["median_planner_over_euclidean"] is not None
        ]
        result[environment] = {
            "run_count": len(rows),
            "selectable_candidate_count": selectable,
            "euclidean_candidate_coverage": euclidean / selectable if selectable else 0.0,
            "selected_decision_count": selected,
            "selected_planner_coverage": planner / selected if selected else 0.0,
            "all_candidate_planner_coverage_upper_bound": (
                planner / selectable if selectable else 0.0
            ),
            "median_of_run_median_planner_over_euclidean": _median(ratio_medians),
        }
    return result


def _write_csv(path: Path, fieldnames: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _fmt(row.get(key)) for key in fieldnames})


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Audit historical Way2 Euclidean/planner travel-cost coverage."
    )
    parser.add_argument(
        "runs",
        nargs="+",
        help="Run IDs (e.g. mpx_001) or explicit MapEx run directories.",
    )
    parser.add_argument(
        "--match-tolerance-m",
        type=float,
        default=0.10,
        help="Max frontier-coordinate error when joining a plan to the selected candidate (default: 0.10 m).",
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
        help="Print the audit without writing CSV/JSON outputs.",
    )
    args = parser.parse_args()

    if args.match_tolerance_m < 0.0:
        parser.error("--match-tolerance-m must be >= 0")

    root = Path(__file__).resolve().parents[1]
    run_summaries: list[dict] = []
    matches: list[dict] = []

    for value in args.runs:
        run_dir = _resolve_run(root, value)
        summary, run_matches = analyze_run(run_dir, args.match_tolerance_m)
        run_summaries.append(summary)
        matches.extend(run_matches)
        print(
            f"{summary['run_id']} ({summary['environment']}): "
            f"euclidean={summary['euclidean_candidate_coverage']:.1%} of selectable candidates, "
            f"planner={summary['selected_planner_coverage']:.1%} of selected decisions, "
            f"all-candidate planner upper bound={summary['all_candidate_planner_coverage_upper_bound']:.1%}, "
            f"median path/euclid={summary['median_planner_over_euclidean']}"
        )

    environment_summary = _environment_summary(run_summaries)
    total_selectable = sum(row["selectable_candidate_count"] for row in run_summaries)
    total_euclidean = sum(row["selectable_with_euclidean_count"] for row in run_summaries)
    total_selected = sum(row["selected_decision_count"] for row in run_summaries)
    total_planner = sum(
        row["selected_with_first_usable_plan_count"] for row in run_summaries
    )

    aggregate = {
        "run_count": len(run_summaries),
        "selectable_candidate_count": total_selectable,
        "euclidean_candidate_coverage": (
            total_euclidean / total_selectable if total_selectable else 0.0
        ),
        "selected_decision_count": total_selected,
        "selected_planner_coverage": (
            total_planner / total_selected if total_selected else 0.0
        ),
        "all_candidate_planner_coverage_upper_bound": (
            total_planner / total_selectable if total_selectable else 0.0
        ),
        "environment_summary": environment_summary,
        "interpretation": {
            "euclidean": (
                "distance_m is recorded per candidate and is suitable for historical all-candidate replay when coverage is complete."
            ),
            "planner": (
                "plans.csv records active-goal /plan diagnostics. It can describe the selected frontier, but it does not provide planner path length for every candidate at every decision."
            ),
            "decision_rule": (
                "Do not replace historical all-candidate C with planner path length unless all candidate frontiers are explicitly planned and recorded."
            ),
        },
        "development_only": True,
    }

    print("\nEnvironment summary:")
    for environment, summary in environment_summary.items():
        print(
            f"  {environment}: euclidean={summary['euclidean_candidate_coverage']:.1%}, "
            f"selected planner={summary['selected_planner_coverage']:.1%}, "
            f"all-candidate planner upper bound={summary['all_candidate_planner_coverage_upper_bound']:.1%}, "
            f"median(run median path/euclid)={summary['median_of_run_median_planner_over_euclidean']}"
        )

    if total_selectable and total_planner < total_selectable:
        print(
            "\nConclusion: historical planner-path data are not complete enough for a fair all-candidate Way2 replay. "
            "Use recorded Euclidean distance_m as the historical C baseline."
        )
    else:
        print(
            "\nConclusion: planner-path coverage appears complete; inspect per-run joins before freezing C."
        )

    if not args.no_write:
        output_dir = (
            args.output_dir.expanduser().resolve()
            if args.output_dir is not None
            else (root / "experiments" / "mapex").resolve()
        )
        output_dir.mkdir(parents=True, exist_ok=True)
        _write_csv(output_dir / "way2_cost_audit_runs.csv", RUN_FIELDS, run_summaries)
        _write_csv(output_dir / "way2_cost_audit_matches.csv", MATCH_FIELDS, matches)
        (output_dir / "way2_cost_audit_summary.json").write_text(
            json.dumps(aggregate, indent=2, allow_nan=False), encoding="utf-8"
        )
        print(f"Wrote audit outputs to {output_dir}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
