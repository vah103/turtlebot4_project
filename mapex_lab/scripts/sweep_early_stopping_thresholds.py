#!/usr/bin/env python3
"""Sweep offline MapEx early-stopping rules over recorded runs.

The script consumes each run's ``early_stopping_analysis.csv`` and evaluates
rules of the form:

    unknown_variance_p95 <= threshold for K consecutive decision states

This is an offline analysis only. Ground-truth-derived quantities (IoU/TU loss)
are used only to evaluate a rule after its stopping point has been selected from
the deployable P95 uncertainty signal.

By default, consecutive exact duplicate online states are collapsed before the
persistence test. This prevents stationary tail rows from satisfying K merely
because the recorder emitted the same final state repeatedly.

Outputs:
- ``early_stopping_threshold_sweep.csv``: one aggregate row per rule.
- ``early_stopping_threshold_sweep_details.csv``: one row per rule/run pair.

No pandas dependency is required.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from decimal import Decimal
from pathlib import Path


SUMMARY_FIELDS = [
    "p95_threshold",
    "persistence",
    "run_count",
    "triggered_runs",
    "trigger_rate",
    "mean_time_saved_s",
    "mean_time_saved_fraction",
    "median_time_saved_fraction",
    "mean_distance_saved_m",
    "mean_distance_saved_fraction",
    "median_distance_saved_fraction",
    "mean_iou_loss_vs_final",
    "worst_iou_loss_vs_final",
    "best_iou_gain_vs_final",
    "iou_loss_gt_0_005_runs",
    "iou_loss_gt_0_01_runs",
    "mean_tu_loss_vs_final",
    "worst_tu_loss_vs_final",
    "tu_loss_gt_0_runs",
]

DETAIL_FIELDS = [
    "p95_threshold",
    "persistence",
    "run_id",
    "triggered",
    "stop_decision_id",
    "stop_p95",
    "stop_mean_variance",
    "stop_coverage",
    "time_saved_s",
    "time_saved_fraction",
    "distance_saved_m",
    "distance_saved_fraction",
    "iou_loss_vs_final",
    "tu_loss_vs_final",
]

# Only online/deployable quantities are used for duplicate-state collapsing.
_DUPLICATE_STATE_FIELDS = (
    "distance_m",
    "unknown_cells",
    "unknown_mean_variance",
    "unknown_total_variance",
    "unknown_variance_p95",
)


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _finite_float(value, default=math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _mean(values: list[float]) -> float:
    return float(statistics.mean(values)) if values else math.nan


def _median(values: list[float]) -> float:
    return float(statistics.median(values)) if values else math.nan


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


def _write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: _fmt(row.get(field)) for field in fields})


def _thresholds(start: float, stop: float, step: float) -> list[float]:
    if step <= 0:
        raise ValueError("--threshold-step must be > 0")
    if stop < start:
        raise ValueError("--threshold-max must be >= --threshold-min")

    current = Decimal(str(start))
    end = Decimal(str(stop))
    increment = Decimal(str(step))
    values: list[float] = []
    while current <= end:
        values.append(float(current))
        current += increment
    return values


def _deduplicate_consecutive_states(
    rows: list[dict[str, str]],
) -> list[dict[str, str]]:
    out: list[dict[str, str]] = []
    previous = None
    for row in rows:
        signature = tuple(row.get(field, "") for field in _DUPLICATE_STATE_FIELDS)
        if signature == previous:
            continue
        out.append(row)
        previous = signature
    return out


def _select_stop(
    rows: list[dict[str, str]],
    threshold: float,
    persistence: int,
) -> dict[str, str] | None:
    streak = 0
    for row in rows:
        p95 = _finite_float(row.get("unknown_variance_p95"))
        if math.isfinite(p95) and p95 <= threshold:
            streak += 1
            if streak >= persistence:
                return row
        else:
            streak = 0
    return None


def _detail_for_run(
    run_id: str,
    rows: list[dict[str, str]],
    threshold: float,
    persistence: int,
) -> dict:
    stop = _select_stop(rows, threshold, persistence)
    if stop is None:
        # The rule never fired: exploration continues normally, so there is no
        # early-stop saving or early-stop quality loss for this run.
        return {
            "p95_threshold": threshold,
            "persistence": persistence,
            "run_id": run_id,
            "triggered": False,
            "stop_decision_id": None,
            "stop_p95": math.nan,
            "stop_mean_variance": math.nan,
            "stop_coverage": math.nan,
            "time_saved_s": 0.0,
            "time_saved_fraction": 0.0,
            "distance_saved_m": 0.0,
            "distance_saved_fraction": 0.0,
            "iou_loss_vs_final": 0.0,
            "tu_loss_vs_final": 0.0,
        }

    return {
        "p95_threshold": threshold,
        "persistence": persistence,
        "run_id": run_id,
        "triggered": True,
        "stop_decision_id": int(stop["decision_id"]),
        "stop_p95": _finite_float(stop.get("unknown_variance_p95")),
        "stop_mean_variance": _finite_float(stop.get("unknown_mean_variance")),
        "stop_coverage": _finite_float(stop.get("coverage")),
        "time_saved_s": _finite_float(stop.get("time_saved_s"), 0.0),
        "time_saved_fraction": _finite_float(stop.get("time_saved_fraction"), 0.0),
        "distance_saved_m": _finite_float(stop.get("distance_saved_m"), 0.0),
        "distance_saved_fraction": _finite_float(
            stop.get("distance_saved_fraction"), 0.0
        ),
        "iou_loss_vs_final": _finite_float(stop.get("iou_loss_vs_final"), 0.0),
        "tu_loss_vs_final": _finite_float(stop.get("tu_loss_vs_final"), 0.0),
    }


def _summarize(details: list[dict], threshold: float, persistence: int) -> dict:
    triggered = sum(bool(row["triggered"]) for row in details)
    run_count = len(details)

    time_s = [float(row["time_saved_s"]) for row in details]
    time_f = [float(row["time_saved_fraction"]) for row in details]
    dist_m = [float(row["distance_saved_m"]) for row in details]
    dist_f = [float(row["distance_saved_fraction"]) for row in details]
    iou_loss = [float(row["iou_loss_vs_final"]) for row in details]
    tu_loss = [float(row["tu_loss_vs_final"]) for row in details]

    return {
        "p95_threshold": threshold,
        "persistence": persistence,
        "run_count": run_count,
        "triggered_runs": triggered,
        "trigger_rate": triggered / run_count if run_count else math.nan,
        "mean_time_saved_s": _mean(time_s),
        "mean_time_saved_fraction": _mean(time_f),
        "median_time_saved_fraction": _median(time_f),
        "mean_distance_saved_m": _mean(dist_m),
        "mean_distance_saved_fraction": _mean(dist_f),
        "median_distance_saved_fraction": _median(dist_f),
        "mean_iou_loss_vs_final": _mean(iou_loss),
        # Positive loss is degradation, so max() is the worst run.
        "worst_iou_loss_vs_final": max(iou_loss) if iou_loss else math.nan,
        # Negative loss means the early-stop reconstruction beat final reference.
        "best_iou_gain_vs_final": min(iou_loss) if iou_loss else math.nan,
        "iou_loss_gt_0_005_runs": sum(value > 0.005 for value in iou_loss),
        "iou_loss_gt_0_01_runs": sum(value > 0.01 for value in iou_loss),
        "mean_tu_loss_vs_final": _mean(tu_loss),
        "worst_tu_loss_vs_final": max(tu_loss) if tu_loss else math.nan,
        "tu_loss_gt_0_runs": sum(value > 0.0 for value in tu_loss),
    }


def _discover_runs(experiments_dir: Path) -> list[tuple[str, Path]]:
    runs: list[tuple[str, Path]] = []
    for run_dir in sorted(experiments_dir.glob("mpx_*")):
        analysis = run_dir / "early_stopping_analysis.csv"
        if run_dir.is_dir() and analysis.is_file():
            runs.append((run_dir.name, analysis))
    return runs


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Sweep P95 early-stopping thresholds over recorded MapEx runs."
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
        help="Required consecutive low-P95 decision states (default: 1 2 3).",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Aggregate CSV output path.",
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

    if any(value <= 0 for value in args.persistence):
        parser.error("all --persistence values must be > 0")

    experiments_dir = Path(args.experiments_dir).expanduser().resolve()
    runs = _discover_runs(experiments_dir)
    if not runs:
        parser.error(
            f"no mpx_*/early_stopping_analysis.csv files found in {experiments_dir}"
        )

    run_rows: dict[str, list[dict[str, str]]] = {}
    raw_counts: dict[str, int] = {}
    used_counts: dict[str, int] = {}
    for run_id, path in runs:
        rows = _read_csv(path)
        raw_counts[run_id] = len(rows)
        if not args.keep_duplicate_states:
            rows = _deduplicate_consecutive_states(rows)
        used_counts[run_id] = len(rows)
        run_rows[run_id] = rows

    thresholds = _thresholds(
        args.threshold_min,
        args.threshold_max,
        args.threshold_step,
    )
    persistence_values = sorted(set(args.persistence))

    summary_rows: list[dict] = []
    detail_rows: list[dict] = []
    for persistence in persistence_values:
        for threshold in thresholds:
            current_details = [
                _detail_for_run(
                    run_id,
                    rows,
                    threshold,
                    persistence,
                )
                for run_id, rows in run_rows.items()
            ]
            detail_rows.extend(current_details)
            summary_rows.append(
                _summarize(current_details, threshold, persistence)
            )

    output = (
        Path(args.output).expanduser().resolve()
        if args.output
        else experiments_dir / "early_stopping_threshold_sweep.csv"
    )
    details_output = (
        Path(args.details_output).expanduser().resolve()
        if args.details_output
        else experiments_dir / "early_stopping_threshold_sweep_details.csv"
    )
    _write_csv(output, SUMMARY_FIELDS, summary_rows)
    _write_csv(details_output, DETAIL_FIELDS, detail_rows)

    top_by_distance = sorted(
        summary_rows,
        key=lambda row: (
            -float(row["mean_distance_saved_fraction"]),
            float(row["worst_iou_loss_vs_final"]),
        ),
    )[:10]

    print(
        json.dumps(
            {
                "status": "ok",
                "experiments_dir": str(experiments_dir),
                "run_count": len(runs),
                "runs": [run_id for run_id, _path in runs],
                "threshold_count": len(thresholds),
                "persistence": persistence_values,
                "rule_count": len(summary_rows),
                "duplicate_states_collapsed": not args.keep_duplicate_states,
                "decision_rows_raw": raw_counts,
                "decision_states_used": used_counts,
                "output": str(output),
                "details_output": str(details_output),
                "top_10_by_mean_distance_saved_fraction": [
                    {
                        "p95_threshold": row["p95_threshold"],
                        "persistence": row["persistence"],
                        "triggered_runs": row["triggered_runs"],
                        "mean_distance_saved_fraction": row[
                            "mean_distance_saved_fraction"
                        ],
                        "mean_time_saved_fraction": row[
                            "mean_time_saved_fraction"
                        ],
                        "mean_iou_loss_vs_final": row["mean_iou_loss_vs_final"],
                        "worst_iou_loss_vs_final": row[
                            "worst_iou_loss_vs_final"
                        ],
                        "worst_tu_loss_vs_final": row["worst_tu_loss_vs_final"],
                    }
                    for row in top_by_distance
                ],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
