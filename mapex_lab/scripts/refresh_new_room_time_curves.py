#!/usr/bin/env python3
"""Regenerate canonical New Room baseline Coverage and IoU curves.

Historical recorder processes can remain alive after exploration has stopped, so
``snapshots.csv`` and ``metrics.csv`` may contain a stationary recorder tail.
Canonical run completion is taken from ``summary.json``:

- time cutoff: ``total_time_s``
- distance cutoff: ``total_distance_m``

Raw run logs are intentionally preserved.

Coverage semantics
------------------
Coverage is observed from the saved SLAM canvas.  Canonical main-figure curves
span the full baseline range; after a run completes, its final Coverage is held
constant.  Strict common-support variants are retained separately.

IoU semantics
-------------
Occupied IoU is only available at evaluable policy decisions with an offline
Big-LaMa prediction.  Canonical main-figure IoU curves also span the full
baseline range, but they do not invent new predictions after the last evaluable
decision.  Instead, each run holds its last evaluable/final IoU until its
canonical exploration stop and after completion.  ``*_active_n`` records how
many runs are still exploring at each x value.  Strict common-support IoU curves
remain available separately with no post-evaluation hold.

Outputs:
- analysis/new_room_coverage_time_curve.csv
  Canonical full Coverage-vs-time curve through the longest baseline run.
- analysis/new_room_coverage_distance_curve.csv
  Canonical full Coverage-vs-distance curve through the longest baseline run.
- analysis/new_room_coverage_time_common_curve.csv
  Strict common-support Coverage-vs-time curve, no extrapolation.
- analysis/new_room_coverage_distance_common_curve.csv
  Strict common-support Coverage-vs-distance curve, no extrapolation.
- analysis/new_room_iou_time_curve.csv
  Canonical full IoU-vs-time curve; hold last evaluable IoU after each run's
  last valid evaluation checkpoint.
- analysis/new_room_iou_distance_curve.csv
  Canonical full IoU-vs-distance curve with the same hold-last-evaluable rule.
- analysis/new_room_iou_time_common_curve.csv
  Strict common-support IoU-vs-time curve, no extrapolation.
- analysis/new_room_iou_distance_common_curve.csv
  Strict common-support IoU-vs-distance curve, no extrapolation.
- analysis/new_room_time_cutoff_audit.csv
  Per-run audit of raw recorder/evaluation ends versus canonical stop time.

The legacy ``new_room_coverage_time_full_curve.csv`` alias is removed when this
script runs; ``new_room_coverage_time_curve.csv`` is the canonical full curve.
"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
EVAL_PROFILE = "new_room_v2"
BASELINE_NF = [f"nf_{i:03d}" for i in range(1, 11)]
BASELINE_MAPEX = [f"mpx_{i:03d}" for i in range(1, 11)]
EPS = 1e-6


def run_dir(method: str, run_id: str) -> Path:
    folder = "nearest" if method == "NF" else "mapex"
    return ROOT / "experiments" / folder / run_id


def read_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        raise RuntimeError(f"refusing to write empty CSV: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def finite_float(value, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f"invalid {label}: {value!r}") from exc
    if not math.isfinite(result):
        raise RuntimeError(f"non-finite {label}: {value!r}")
    return result


def baseline_infos() -> list[dict]:
    infos = []
    for method, ids in (("NF", BASELINE_NF), ("MapEx", BASELINE_MAPEX)):
        for run_id in ids:
            path = run_dir(method, run_id)
            summary = read_json(path / "summary.json")
            stop_s = finite_float(summary.get("total_time_s"), f"{run_id}.total_time_s")
            stop_m = finite_float(
                summary.get("total_distance_m"), f"{run_id}.total_distance_m"
            )
            final_coverage = finite_float(
                summary.get("final_coverage"), f"{run_id}.final_coverage"
            )
            final_iou = finite_float(
                summary.get("final_occupied_iou"), f"{run_id}.final_occupied_iou"
            )
            if stop_s <= 0.0:
                raise RuntimeError(
                    f"non-positive exploration stop time for {run_id}: {stop_s}"
                )
            if stop_m <= 0.0:
                raise RuntimeError(
                    f"non-positive exploration stop distance for {run_id}: {stop_m}"
                )
            infos.append(
                {
                    "method": method,
                    "run_id": run_id,
                    "path": path,
                    "stop_s": stop_s,
                    "stop_m": stop_m,
                    "final_coverage": final_coverage,
                    "final_iou": final_iou,
                }
            )
    return infos


def series_from_csv(
    path: Path,
    x_key: str,
    y_key: str,
    *,
    max_x: float | None = None,
    endpoint_y: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    points: list[tuple[float, float]] = []
    for row in read_csv(path):
        try:
            x = float(row[x_key])
            y = float(row[y_key])
        except (KeyError, TypeError, ValueError):
            continue
        if not (math.isfinite(x) and math.isfinite(y)):
            continue
        if max_x is not None and x > max_x + EPS:
            continue
        points.append((x, y))

    if endpoint_y is not None:
        if max_x is None:
            raise RuntimeError("endpoint_y requires max_x")
        points.append((float(max_x), float(endpoint_y)))

    if not points:
        raise RuntimeError(f"no usable {x_key}/{y_key} samples in {path}")

    dedup: dict[float, float] = {}
    for x, y in sorted(points):
        dedup[x] = y

    xs = np.asarray(sorted(dedup), dtype=np.float64)
    ys = np.asarray([dedup[x] for x in xs], dtype=np.float64)
    if xs[0] > 0.0:
        xs = np.insert(xs, 0, 0.0)
        ys = np.insert(ys, 0, ys[0])
    return xs, ys


def common_curve(
    infos: list[dict],
    *,
    source_name: str,
    x_key: str,
    y_key: str,
    stop_key: str,
    step: float,
    cutoff_source: str,
    endpoint_info_key: str | None = None,
) -> list[dict]:
    """Strict no-extrapolation common-support curve across all 20 runs."""

    series: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    for info in infos:
        endpoint_y = info[endpoint_info_key] if endpoint_info_key else None
        series[(info["method"], info["run_id"])] = series_from_csv(
            info["path"] / source_name,
            x_key,
            y_key,
            max_x=info[stop_key],
            endpoint_y=endpoint_y,
        )

    common_max = min(float(xs[-1]) for xs, _ in series.values())
    grid_max = math.floor(common_max / step) * step
    grid = np.arange(0.0, grid_max + step * 0.25, step)

    output = []
    for x in grid:
        row = {
            x_key: float(x),
            "evaluation_profile": EVAL_PROFILE,
            "common_support_exact_max": common_max,
            "cutoff_source": cutoff_source,
        }
        for method, ids in (("NF", BASELINE_NF), ("MapEx", BASELINE_MAPEX)):
            vals = [
                float(np.interp(x, *series[(method, run_id)])) for run_id in ids
            ]
            prefix = method.lower()
            row[f"{prefix}_mean"] = float(np.mean(vals))
            row[f"{prefix}_std"] = float(np.std(vals, ddof=1))
            row[f"{prefix}_n"] = len(vals)
        output.append(row)
    return output


def full_coverage_curve(
    infos: list[dict],
    *,
    x_key: str,
    stop_key: str,
    step: float,
    cutoff_source: str,
) -> list[dict]:
    """Completion-aware Coverage curve through the longest completed run."""

    series: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    info_by_key = {}
    for info in infos:
        key = (info["method"], info["run_id"])
        info_by_key[key] = info
        series[key] = series_from_csv(
            info["path"] / "snapshots.csv",
            x_key,
            "coverage",
            max_x=info[stop_key],
            endpoint_y=info["final_coverage"],
        )

    max_stop = max(float(info[stop_key]) for info in infos)
    grid_max = math.floor(max_stop / step) * step
    grid = np.arange(0.0, grid_max + step * 0.25, step)
    max_key = "max_stop_exact_s" if x_key == "time_s" else "max_stop_exact_m"

    output = []
    for x in grid:
        row = {
            x_key: float(x),
            "evaluation_profile": EVAL_PROFILE,
            max_key: max_stop,
            "cutoff_source": cutoff_source,
            "post_completion_semantics": "hold_final_coverage",
        }
        for method, ids in (("NF", BASELINE_NF), ("MapEx", BASELINE_MAPEX)):
            vals = []
            active_n = 0
            for run_id in ids:
                key = (method, run_id)
                info = info_by_key[key]
                if x <= float(info[stop_key]) + EPS:
                    vals.append(float(np.interp(x, *series[key])))
                    active_n += 1
                else:
                    vals.append(float(info["final_coverage"]))
            prefix = method.lower()
            row[f"{prefix}_mean"] = float(np.mean(vals))
            row[f"{prefix}_std"] = float(np.std(vals, ddof=1))
            row[f"{prefix}_n"] = len(vals)
            row[f"{prefix}_active_n"] = active_n
        output.append(row)
    return output


def full_iou_curve(
    infos: list[dict],
    *,
    x_key: str,
    stop_key: str,
    step: float,
    cutoff_source: str,
) -> list[dict]:
    """Completion-aware IoU curve without inventing post-evaluation predictions.

    Each run is interpolated only between real evaluable decision checkpoints.
    Once x passes that run's last evaluable checkpoint, the run's final IoU
    (which is the last evaluable IoU stored in summary.json) is carried forward.
    The x-axis still spans the canonical exploration range from summary.json.
    """

    series: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    info_by_key: dict[tuple[str, str], dict] = {}
    last_eval_x: dict[tuple[str, str], float] = {}

    for info in infos:
        key = (info["method"], info["run_id"])
        info_by_key[key] = info
        xs, ys = series_from_csv(
            info["path"] / "evaluation.csv",
            x_key,
            "occupied_iou",
            max_x=info[stop_key],
            endpoint_y=None,
        )
        # The summary final IoU must represent the last evaluable decision.
        if not math.isclose(
            float(ys[-1]), float(info["final_iou"]), rel_tol=1e-7, abs_tol=1e-9
        ):
            raise RuntimeError(
                f"{info['run_id']} final_occupied_iou={info['final_iou']} does not "
                f"match last evaluation {float(ys[-1])} for {x_key}"
            )
        series[key] = (xs, ys)
        last_eval_x[key] = float(xs[-1])

    max_stop = max(float(info[stop_key]) for info in infos)
    grid_max = math.floor(max_stop / step) * step
    grid = np.arange(0.0, grid_max + step * 0.25, step)
    max_key = "max_stop_exact_s" if x_key == "time_s" else "max_stop_exact_m"
    last_eval_key = (
        "max_last_evaluable_exact_s" if x_key == "time_s" else "max_last_evaluable_exact_m"
    )
    max_last_eval = max(last_eval_x.values())

    output = []
    for x in grid:
        row = {
            x_key: float(x),
            "evaluation_profile": EVAL_PROFILE,
            max_key: max_stop,
            last_eval_key: max_last_eval,
            "cutoff_source": cutoff_source,
            "post_evaluation_semantics": "hold_last_evaluable_iou",
            "final_iou_source": "summary.final_occupied_iou",
        }
        for method, ids in (("NF", BASELINE_NF), ("MapEx", BASELINE_MAPEX)):
            vals = []
            active_n = 0
            for run_id in ids:
                key = (method, run_id)
                info = info_by_key[key]
                xs, ys = series[key]
                if x <= last_eval_x[key] + EPS:
                    vals.append(float(np.interp(x, xs, ys)))
                else:
                    vals.append(float(info["final_iou"]))
                if x <= float(info[stop_key]) + EPS:
                    active_n += 1
            prefix = method.lower()
            row[f"{prefix}_mean"] = float(np.mean(vals))
            row[f"{prefix}_std"] = float(np.std(vals, ddof=1))
            row[f"{prefix}_n"] = len(vals)
            row[f"{prefix}_active_n"] = active_n
        output.append(row)
    return output


def last_finite_x(path: Path, key: str) -> float:
    values = []
    for row in read_csv(path):
        try:
            x = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(x):
            values.append(x)
    return max(values) if values else math.nan


def cutoff_audit(infos: list[dict]) -> list[dict]:
    output = []
    for info in infos:
        stop_s = float(info["stop_s"])
        snapshot_last = last_finite_x(info["path"] / "snapshots.csv", "time_s")
        metrics_last = last_finite_x(info["path"] / "metrics.csv", "time_s")
        evaluation_last = last_finite_x(info["path"] / "evaluation.csv", "time_s")
        evaluation_last_m = last_finite_x(
            info["path"] / "evaluation.csv", "distance_m"
        )
        output.append(
            {
                "method": info["method"],
                "run_id": info["run_id"],
                "evaluation_profile": EVAL_PROFILE,
                "canonical_stop_time_s": stop_s,
                "cutoff_source": "summary.total_time_s",
                "snapshots_last_time_s": snapshot_last,
                "snapshots_tail_s": max(0.0, snapshot_last - stop_s),
                "metrics_last_time_s": metrics_last,
                "metrics_tail_s": max(0.0, metrics_last - stop_s),
                "evaluation_last_time_s": evaluation_last,
                "evaluation_tail_s": max(0.0, evaluation_last - stop_s),
                "canonical_stop_distance_m": float(info["stop_m"]),
                "evaluation_last_distance_m": evaluation_last_m,
                "final_coverage": info["final_coverage"],
                "final_occupied_iou": info["final_iou"],
            }
        )
    return output


def main() -> None:
    infos = baseline_infos()
    if len(infos) != 20:
        raise RuntimeError(f"expected 20 baseline runs, found {len(infos)}")

    analysis = ROOT / "analysis"

    # Strict common-support Coverage curves.
    coverage_time_common = common_curve(
        infos,
        source_name="snapshots.csv",
        x_key="time_s",
        y_key="coverage",
        stop_key="stop_s",
        step=10.0,
        cutoff_source="summary.total_time_s",
        endpoint_info_key="final_coverage",
    )
    coverage_distance_common = common_curve(
        infos,
        source_name="snapshots.csv",
        x_key="distance_m",
        y_key="coverage",
        stop_key="stop_m",
        step=5.0,
        cutoff_source="summary.total_distance_m",
        endpoint_info_key="final_coverage",
    )
    write_csv(
        analysis / "new_room_coverage_time_common_curve.csv", coverage_time_common
    )
    write_csv(
        analysis / "new_room_coverage_distance_common_curve.csv",
        coverage_distance_common,
    )

    # Canonical full Coverage curves used for the main figures.
    coverage_time_full = full_coverage_curve(
        infos,
        x_key="time_s",
        stop_key="stop_s",
        step=10.0,
        cutoff_source="summary.total_time_s",
    )
    coverage_distance_full = full_coverage_curve(
        infos,
        x_key="distance_m",
        stop_key="stop_m",
        step=5.0,
        cutoff_source="summary.total_distance_m",
    )
    write_csv(analysis / "new_room_coverage_time_curve.csv", coverage_time_full)
    write_csv(
        analysis / "new_room_coverage_distance_curve.csv", coverage_distance_full
    )

    # Strict common-support IoU curves: real decision-level evaluations only.
    iou_time_common = common_curve(
        infos,
        source_name="evaluation.csv",
        x_key="time_s",
        y_key="occupied_iou",
        stop_key="stop_s",
        step=10.0,
        cutoff_source="summary.total_time_s",
    )
    iou_distance_common = common_curve(
        infos,
        source_name="evaluation.csv",
        x_key="distance_m",
        y_key="occupied_iou",
        stop_key="stop_m",
        step=5.0,
        cutoff_source="summary.total_distance_m",
    )
    write_csv(analysis / "new_room_iou_time_common_curve.csv", iou_time_common)
    write_csv(
        analysis / "new_room_iou_distance_common_curve.csv", iou_distance_common
    )

    # Canonical full IoU curves used for the main figures.
    iou_time_full = full_iou_curve(
        infos,
        x_key="time_s",
        stop_key="stop_s",
        step=10.0,
        cutoff_source="summary.total_time_s",
    )
    iou_distance_full = full_iou_curve(
        infos,
        x_key="distance_m",
        stop_key="stop_m",
        step=5.0,
        cutoff_source="summary.total_distance_m",
    )
    write_csv(analysis / "new_room_iou_time_curve.csv", iou_time_full)
    write_csv(analysis / "new_room_iou_distance_curve.csv", iou_distance_full)

    audit = cutoff_audit(infos)
    write_csv(analysis / "new_room_time_cutoff_audit.csv", audit)

    legacy_full = analysis / "new_room_coverage_time_full_curve.csv"
    if legacy_full.exists():
        legacy_full.unlink()

    print("Refreshed canonical New Room Coverage and IoU curves from 20 baseline runs.")
    print(
        "Coverage time full: "
        f"0-{coverage_time_full[-1]['time_s']} s grid; exact max stop "
        f"{coverage_time_full[-1]['max_stop_exact_s']} s"
    )
    print(
        "Coverage distance full: "
        f"0-{coverage_distance_full[-1]['distance_m']} m grid; exact max stop "
        f"{coverage_distance_full[-1]['max_stop_exact_m']} m"
    )
    print(
        "IoU time full: "
        f"0-{iou_time_full[-1]['time_s']} s grid; exact max stop "
        f"{iou_time_full[-1]['max_stop_exact_s']} s"
    )
    print(
        "IoU distance full: "
        f"0-{iou_distance_full[-1]['distance_m']} m grid; exact max stop "
        f"{iou_distance_full[-1]['max_stop_exact_m']} m"
    )
    print(
        "Coverage time common: "
        f"0-{coverage_time_common[-1]['time_s']} s grid; exact common support "
        f"{coverage_time_common[-1]['common_support_exact_max']} s"
    )
    print(
        "Coverage distance common: "
        f"0-{coverage_distance_common[-1]['distance_m']} m grid; exact common support "
        f"{coverage_distance_common[-1]['common_support_exact_max']} m"
    )
    print(
        "IoU time common: "
        f"0-{iou_time_common[-1]['time_s']} s grid; exact common support "
        f"{iou_time_common[-1]['common_support_exact_max']} s"
    )
    print(
        "IoU distance common: "
        f"0-{iou_distance_common[-1]['distance_m']} m grid; exact common support "
        f"{iou_distance_common[-1]['common_support_exact_max']} m"
    )
    print(
        "Raw snapshots.csv/metrics.csv/evaluation.csv were not modified; raw "
        "evidence remains available for audit."
    )


if __name__ == "__main__":
    main()
