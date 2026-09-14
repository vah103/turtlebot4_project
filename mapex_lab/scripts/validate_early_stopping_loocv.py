#!/usr/bin/env python3
"""Leave-one-run-out validation for MapEx early-stopping rule selection.

For each fold, one recorded MapEx run is held out as test data. Candidate rules
are evaluated only on the remaining training runs, and the best eligible rule
is selected using training data alone. The selected rule is then applied to the
held-out run using only the deployable P95 uncertainty signal.

Default eligibility on the training folds mirrors the current research screen:
- the rule must trigger on every training run;
- worst IoU loss vs the recorded final reference must be <= 0.01;
- worst TU loss vs the recorded final reference must be <= 0.

Among eligible rules, selection maximizes mean distance saved fraction, with
mean time saved fraction as the first tie-breaker and lower worst IoU loss as
the second tie-breaker.

The script reuses the exact threshold, persistence, duplicate-state collapsing,
and stop-selection semantics from ``sweep_early_stopping_thresholds.py``.

Outputs:
- ``early_stopping_loocv.csv``: one row per held-out run.

No pandas dependency is required.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter
from pathlib import Path

import sweep_early_stopping_thresholds as sweep


OUTPUT_FIELDS = [
    "held_out_run",
    "train_run_count",
    "eligible_rule_count",
    "selected_p95_threshold",
    "selected_persistence",
    "train_mean_distance_saved_fraction",
    "train_mean_time_saved_fraction",
    "train_mean_iou_loss_vs_final",
    "train_worst_iou_loss_vs_final",
    "train_worst_tu_loss_vs_final",
    "test_triggered",
    "test_stop_decision_id",
    "test_stop_p95",
    "test_stop_coverage",
    "test_distance_saved_m",
    "test_distance_saved_fraction",
    "test_time_saved_s",
    "test_time_saved_fraction",
    "test_iou_loss_vs_final",
    "test_tu_loss_vs_final",
    "test_iou_loss_within_limit",
    "test_tu_loss_within_limit",
]


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


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: _fmt(row.get(field)) for field in OUTPUT_FIELDS})


def _mean(values: list[float]) -> float:
    return float(statistics.mean(values)) if values else math.nan


def _rule_key(row: dict) -> tuple[float, float, float, int, float]:
    """Sort key for choosing one rule from eligible training rules.

    Larger distance/time savings are better. Lower worst IoU loss is better.
    If rules remain tied, prefer persistence (stability) and then the lower
    threshold (more conservative uncertainty requirement).
    """
    return (
        float(row["mean_distance_saved_fraction"]),
        float(row["mean_time_saved_fraction"]),
        -float(row["worst_iou_loss_vs_final"]),
        int(row["persistence"]),
        -float(row["p95_threshold"]),
    )


def _select_training_rule(
    train_rows: dict[str, list[dict[str, str]]],
    thresholds: list[float],
    persistence_values: list[int],
    max_iou_loss: float,
    max_tu_loss: float,
) -> tuple[dict, int]:
    candidates: list[dict] = []
    train_count = len(train_rows)

    for persistence in persistence_values:
        for threshold in thresholds:
            details = [
                sweep._detail_for_run(run_id, rows, threshold, persistence)
                for run_id, rows in train_rows.items()
            ]
            summary = sweep._summarize(details, threshold, persistence)
            if int(summary["triggered_runs"]) != train_count:
                continue
            if float(summary["worst_iou_loss_vs_final"]) > max_iou_loss:
                continue
            if float(summary["worst_tu_loss_vs_final"]) > max_tu_loss:
                continue
            candidates.append(summary)

    if not candidates:
        raise RuntimeError(
            "No eligible rule in this fold. Relax the validation constraints "
            "or expand the threshold/persistence search grid."
        )

    return max(candidates, key=_rule_key), len(candidates)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Leave-one-run-out validation for MapEx early stopping."
    )
    mapex_lab = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--experiments-dir",
        default=str(mapex_lab / "experiments" / "mapex"),
        help="Directory containing mpx_* run directories.",
    )
    parser.add_argument("--threshold-min", type=float, default=0.10)
    parser.add_argument("--threshold-max", type=float, default=0.30)
    parser.add_argument("--threshold-step", type=float, default=0.01)
    parser.add_argument(
        "--persistence",
        type=int,
        nargs="+",
        default=[1, 2, 3],
        help="Candidate persistence values (default: 1 2 3).",
    )
    parser.add_argument(
        "--max-train-iou-loss",
        type=float,
        default=0.01,
        help="Maximum allowed worst training IoU loss (default: 0.01).",
    )
    parser.add_argument(
        "--max-train-tu-loss",
        type=float,
        default=0.0,
        help="Maximum allowed worst training TU loss (default: 0).",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="LOOCV CSV output path.",
    )
    parser.add_argument(
        "--keep-duplicate-states",
        action="store_true",
        help="Do not collapse consecutive exact duplicate online states.",
    )
    args = parser.parse_args()

    if args.max_train_iou_loss < 0:
        parser.error("--max-train-iou-loss must be >= 0")
    if args.max_train_tu_loss < 0:
        parser.error("--max-train-tu-loss must be >= 0")
    if any(value <= 0 for value in args.persistence):
        parser.error("all --persistence values must be > 0")

    experiments_dir = Path(args.experiments_dir).expanduser().resolve()
    discovered = sweep._discover_runs(experiments_dir)
    if len(discovered) < 2:
        parser.error(
            f"need at least 2 mpx_*/early_stopping_analysis.csv runs in "
            f"{experiments_dir}"
        )

    run_rows: dict[str, list[dict[str, str]]] = {}
    raw_counts: dict[str, int] = {}
    used_counts: dict[str, int] = {}
    for run_id, path in discovered:
        rows = sweep._read_csv(path)
        raw_counts[run_id] = len(rows)
        if not args.keep_duplicate_states:
            rows = sweep._deduplicate_consecutive_states(rows)
        used_counts[run_id] = len(rows)
        run_rows[run_id] = rows

    thresholds = sweep._thresholds(
        args.threshold_min,
        args.threshold_max,
        args.threshold_step,
    )
    persistence_values = sorted(set(args.persistence))

    fold_rows: list[dict] = []
    for held_out_run, test_rows in run_rows.items():
        train_rows = {
            run_id: rows
            for run_id, rows in run_rows.items()
            if run_id != held_out_run
        }
        selected, eligible_count = _select_training_rule(
            train_rows,
            thresholds,
            persistence_values,
            args.max_train_iou_loss,
            args.max_train_tu_loss,
        )

        threshold = float(selected["p95_threshold"])
        persistence = int(selected["persistence"])
        test = sweep._detail_for_run(
            held_out_run,
            test_rows,
            threshold,
            persistence,
        )

        test_iou_loss = float(test["iou_loss_vs_final"])
        test_tu_loss = float(test["tu_loss_vs_final"])
        fold_rows.append(
            {
                "held_out_run": held_out_run,
                "train_run_count": len(train_rows),
                "eligible_rule_count": eligible_count,
                "selected_p95_threshold": threshold,
                "selected_persistence": persistence,
                "train_mean_distance_saved_fraction": selected[
                    "mean_distance_saved_fraction"
                ],
                "train_mean_time_saved_fraction": selected[
                    "mean_time_saved_fraction"
                ],
                "train_mean_iou_loss_vs_final": selected[
                    "mean_iou_loss_vs_final"
                ],
                "train_worst_iou_loss_vs_final": selected[
                    "worst_iou_loss_vs_final"
                ],
                "train_worst_tu_loss_vs_final": selected[
                    "worst_tu_loss_vs_final"
                ],
                "test_triggered": bool(test["triggered"]),
                "test_stop_decision_id": test["stop_decision_id"],
                "test_stop_p95": test["stop_p95"],
                "test_stop_coverage": test["stop_coverage"],
                "test_distance_saved_m": test["distance_saved_m"],
                "test_distance_saved_fraction": test[
                    "distance_saved_fraction"
                ],
                "test_time_saved_s": test["time_saved_s"],
                "test_time_saved_fraction": test["time_saved_fraction"],
                "test_iou_loss_vs_final": test_iou_loss,
                "test_tu_loss_vs_final": test_tu_loss,
                "test_iou_loss_within_limit": (
                    test_iou_loss <= args.max_train_iou_loss
                ),
                "test_tu_loss_within_limit": (
                    test_tu_loss <= args.max_train_tu_loss
                ),
            }
        )

    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else experiments_dir / "early_stopping_loocv.csv"
    )
    _write_csv(output, fold_rows)

    test_distance_fraction = [
        float(row["test_distance_saved_fraction"]) for row in fold_rows
    ]
    test_time_fraction = [
        float(row["test_time_saved_fraction"]) for row in fold_rows
    ]
    test_iou_loss = [float(row["test_iou_loss_vs_final"]) for row in fold_rows]
    test_tu_loss = [float(row["test_tu_loss_vs_final"]) for row in fold_rows]
    selected_counts = Counter(
        (
            float(row["selected_p95_threshold"]),
            int(row["selected_persistence"]),
        )
        for row in fold_rows
    )

    selected_rules = [
        {
            "p95_threshold": threshold,
            "persistence": persistence,
            "fold_count": count,
        }
        for (threshold, persistence), count in sorted(
            selected_counts.items(),
            key=lambda item: (-item[1], item[0][0], item[0][1]),
        )
    ]

    print(
        json.dumps(
            {
                "status": "ok",
                "experiments_dir": str(experiments_dir),
                "run_count": len(run_rows),
                "fold_count": len(fold_rows),
                "threshold_count": len(thresholds),
                "persistence": persistence_values,
                "duplicate_states_collapsed": not args.keep_duplicate_states,
                "selection_constraints": {
                    "trigger_all_training_runs": True,
                    "max_train_iou_loss": args.max_train_iou_loss,
                    "max_train_tu_loss": args.max_train_tu_loss,
                    "objective": "maximize_mean_distance_saved_fraction",
                },
                "decision_rows_raw": raw_counts,
                "decision_states_used": used_counts,
                "output": str(output),
                "selected_rules": selected_rules,
                "held_out_summary": {
                    "triggered_runs": sum(
                        bool(row["test_triggered"]) for row in fold_rows
                    ),
                    "mean_distance_saved_fraction": _mean(
                        test_distance_fraction
                    ),
                    "mean_time_saved_fraction": _mean(test_time_fraction),
                    "mean_iou_loss_vs_final": _mean(test_iou_loss),
                    "worst_iou_loss_vs_final": max(test_iou_loss),
                    "iou_loss_gt_0_005_runs": sum(
                        value > 0.005 for value in test_iou_loss
                    ),
                    "iou_loss_gt_0_01_runs": sum(
                        value > 0.01 for value in test_iou_loss
                    ),
                    "mean_tu_loss_vs_final": _mean(test_tu_loss),
                    "worst_tu_loss_vs_final": max(test_tu_loss),
                    "tu_loss_gt_0_runs": sum(
                        value > 0.0 for value in test_tu_loss
                    ),
                    "all_held_out_runs_within_iou_limit": all(
                        bool(row["test_iou_loss_within_limit"])
                        for row in fold_rows
                    ),
                    "all_held_out_runs_within_tu_limit": all(
                        bool(row["test_tu_loss_within_limit"])
                        for row in fold_rows
                    ),
                },
                "folds": [
                    {
                        "held_out_run": row["held_out_run"],
                        "selected_p95_threshold": row[
                            "selected_p95_threshold"
                        ],
                        "selected_persistence": row[
                            "selected_persistence"
                        ],
                        "test_stop_decision_id": row[
                            "test_stop_decision_id"
                        ],
                        "test_distance_saved_fraction": row[
                            "test_distance_saved_fraction"
                        ],
                        "test_time_saved_fraction": row[
                            "test_time_saved_fraction"
                        ],
                        "test_iou_loss_vs_final": row[
                            "test_iou_loss_vs_final"
                        ],
                        "test_tu_loss_vs_final": row[
                            "test_tu_loss_vs_final"
                        ],
                    }
                    for row in fold_rows
                ],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
