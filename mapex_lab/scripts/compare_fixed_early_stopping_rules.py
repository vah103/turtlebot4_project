#!/usr/bin/env python3
"""Compare selected fixed MapEx early-stopping rules on recorded runs.

This script is intentionally simpler than threshold sweep / LOOCV. It answers:

    If one fixed rule were chosen now, how would that same rule perform
    on every recorded MapEx run?

Default candidate rules:
- P95 <= 0.23, K=1
- P95 <= 0.25, K=2
- P95 <= 0.20, K=1

The implementation reuses the exact online-state deduplication, stop selection,
per-run detail, and aggregate-summary semantics from
``sweep_early_stopping_thresholds.py``.

Ground-truth-derived IoU/TU losses are evaluation outputs only. They never
participate in selecting the stopping point.

Outputs:
- ``early_stopping_fixed_rule_comparison.csv``: one aggregate row per rule.
- ``early_stopping_fixed_rule_details.csv``: one row per rule/run pair.

No pandas dependency is required.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path

import sweep_early_stopping_thresholds as sweep


SUMMARY_FIELDS = sweep.SUMMARY_FIELDS + [
    "all_runs_triggered",
    "all_runs_iou_loss_le_0_005",
    "all_runs_iou_loss_le_0_01",
    "all_runs_tu_loss_le_0",
]

DETAIL_FIELDS = sweep.DETAIL_FIELDS + [
    "iou_loss_le_0_005",
    "iou_loss_le_0_01",
    "tu_loss_le_0",
]


def _parse_rule(value: str) -> tuple[float, int]:
    """Parse THRESHOLD:K, e.g. 0.23:1."""
    try:
        threshold_text, persistence_text = value.split(":", 1)
        threshold = float(threshold_text)
        persistence = int(persistence_text)
    except (ValueError, TypeError) as exc:
        raise argparse.ArgumentTypeError(
            f"invalid rule {value!r}; expected THRESHOLD:K, e.g. 0.23:1"
        ) from exc

    if not math.isfinite(threshold):
        raise argparse.ArgumentTypeError("rule threshold must be finite")
    if persistence <= 0:
        raise argparse.ArgumentTypeError("rule persistence K must be > 0")
    return threshold, persistence


def _write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: sweep._fmt(row.get(field)) for field in fields})


def _augment_detail(detail: dict) -> dict:
    out = dict(detail)
    iou_loss = float(detail["iou_loss_vs_final"])
    tu_loss = float(detail["tu_loss_vs_final"])
    out["iou_loss_le_0_005"] = iou_loss <= 0.005
    out["iou_loss_le_0_01"] = iou_loss <= 0.01
    out["tu_loss_le_0"] = tu_loss <= 0.0
    return out


def _augment_summary(summary: dict, details: list[dict]) -> dict:
    out = dict(summary)
    run_count = len(details)
    out["all_runs_triggered"] = int(summary["triggered_runs"]) == run_count
    out["all_runs_iou_loss_le_0_005"] = all(
        float(row["iou_loss_vs_final"]) <= 0.005 for row in details
    )
    out["all_runs_iou_loss_le_0_01"] = all(
        float(row["iou_loss_vs_final"]) <= 0.01 for row in details
    )
    out["all_runs_tu_loss_le_0"] = all(
        float(row["tu_loss_vs_final"]) <= 0.0 for row in details
    )
    return out


def _rule_label(threshold: float, persistence: int) -> str:
    return f"P95<={threshold:.2f},K={persistence}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare fixed P95 early-stopping rules over recorded MapEx runs."
    )
    mapex_lab = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--experiments-dir",
        default=str(mapex_lab / "experiments" / "mapex"),
        help="Directory containing mpx_* run directories.",
    )
    parser.add_argument(
        "--rule",
        dest="rules",
        action="append",
        type=_parse_rule,
        default=None,
        metavar="THRESHOLD:K",
        help=(
            "Fixed rule to compare; may be repeated. "
            "Default: 0.23:1, 0.25:2, 0.20:1."
        ),
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Aggregate comparison CSV output path.",
    )
    parser.add_argument(
        "--details-output",
        default=None,
        help="Per-rule/per-run CSV output path.",
    )
    parser.add_argument(
        "--keep-duplicate-states",
        action="store_true",
        help="Do not collapse consecutive exact duplicate online states.",
    )
    args = parser.parse_args()

    rules = args.rules or [(0.23, 1), (0.25, 2), (0.20, 1)]
    # Preserve command-line order while removing accidental duplicates.
    rules = list(dict.fromkeys(rules))

    experiments_dir = Path(args.experiments_dir).expanduser().resolve()
    runs = sweep._discover_runs(experiments_dir)
    if not runs:
        parser.error(
            f"no mpx_*/early_stopping_analysis.csv files found in {experiments_dir}"
        )

    run_rows: dict[str, list[dict[str, str]]] = {}
    raw_counts: dict[str, int] = {}
    used_counts: dict[str, int] = {}
    for run_id, path in runs:
        rows = sweep._read_csv(path)
        raw_counts[run_id] = len(rows)
        if not args.keep_duplicate_states:
            rows = sweep._deduplicate_consecutive_states(rows)
        used_counts[run_id] = len(rows)
        run_rows[run_id] = rows

    summary_rows: list[dict] = []
    detail_rows: list[dict] = []
    json_rules: list[dict] = []

    for threshold, persistence in rules:
        details = [
            _augment_detail(
                sweep._detail_for_run(run_id, rows, threshold, persistence)
            )
            for run_id, rows in run_rows.items()
        ]
        summary = _augment_summary(
            sweep._summarize(details, threshold, persistence), details
        )
        detail_rows.extend(details)
        summary_rows.append(summary)

        worst_run = max(
            details,
            key=lambda row: float(row["iou_loss_vs_final"]),
        )
        best_saving_run = max(
            details,
            key=lambda row: float(row["distance_saved_fraction"]),
        )
        failures_0_01 = [
            row["run_id"]
            for row in details
            if float(row["iou_loss_vs_final"]) > 0.01
        ]
        failures_0_005 = [
            row["run_id"]
            for row in details
            if float(row["iou_loss_vs_final"]) > 0.005
        ]

        json_rules.append(
            {
                "rule": _rule_label(threshold, persistence),
                "p95_threshold": threshold,
                "persistence": persistence,
                "triggered_runs": int(summary["triggered_runs"]),
                "run_count": int(summary["run_count"]),
                "mean_distance_saved_fraction": float(
                    summary["mean_distance_saved_fraction"]
                ),
                "mean_time_saved_fraction": float(
                    summary["mean_time_saved_fraction"]
                ),
                "mean_iou_loss_vs_final": float(summary["mean_iou_loss_vs_final"]),
                "worst_iou_loss_vs_final": float(
                    summary["worst_iou_loss_vs_final"]
                ),
                "worst_iou_run": worst_run["run_id"],
                "worst_tu_loss_vs_final": float(
                    summary["worst_tu_loss_vs_final"]
                ),
                "iou_loss_gt_0_005_runs": failures_0_005,
                "iou_loss_gt_0_01_runs": failures_0_01,
                "all_runs_iou_loss_le_0_005": bool(
                    summary["all_runs_iou_loss_le_0_005"]
                ),
                "all_runs_iou_loss_le_0_01": bool(
                    summary["all_runs_iou_loss_le_0_01"]
                ),
                "all_runs_tu_loss_le_0": bool(summary["all_runs_tu_loss_le_0"]),
                "best_distance_saving_run": best_saving_run["run_id"],
                "best_distance_saved_fraction": float(
                    best_saving_run["distance_saved_fraction"]
                ),
            }
        )

    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else experiments_dir / "early_stopping_fixed_rule_comparison.csv"
    )
    details_output = (
        Path(args.details_output).expanduser().resolve()
        if args.details_output
        else experiments_dir / "early_stopping_fixed_rule_details.csv"
    )
    _write_csv(output, SUMMARY_FIELDS, summary_rows)
    _write_csv(details_output, DETAIL_FIELDS, detail_rows)

    # A compact ranking is useful for terminal review, but no rule is declared
    # "validated" here. The ranking only summarizes the recorded 10-run set.
    ranked = sorted(
        json_rules,
        key=lambda row: (
            len(row["iou_loss_gt_0_01_runs"]),
            row["worst_iou_loss_vs_final"],
            -row["mean_distance_saved_fraction"],
        ),
    )

    selected_rule_counts = Counter(row["rule"] for row in ranked)
    print(
        json.dumps(
            {
                "status": "ok",
                "experiments_dir": str(experiments_dir),
                "run_count": len(runs),
                "runs": [run_id for run_id, _path in runs],
                "fixed_rules": [
                    {"p95_threshold": threshold, "persistence": persistence}
                    for threshold, persistence in rules
                ],
                "duplicate_states_collapsed": not args.keep_duplicate_states,
                "decision_rows_raw": raw_counts,
                "decision_states_used": used_counts,
                "output": str(output),
                "details_output": str(details_output),
                "comparison": json_rules,
                "ranking_by_recorded_set_safety_then_saving": [
                    row["rule"] for row in ranked
                ],
                "note": (
                    "Ranking describes only the current recorded 10-run dataset; "
                    "it is not independent validation."
                ),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
