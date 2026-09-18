#!/usr/bin/env python3
"""Regenerate canonical New Room baseline Coverage and IoU curves.

Canonical run completion comes from summary.json:
- time cutoff: total_time_s
- distance cutoff: total_distance_m

Raw snapshots.csv, metrics.csv and evaluation.csv are never modified.

Main-figure Coverage curves span the full baseline range and hold final Coverage
once a run completes. Main-figure IoU curves span the same canonical range, but
only interpolate between real decision-level evaluations; after a run's last
evaluable decision they hold summary.final_occupied_iou.

Strict common-support variants are retained separately.

Important distance-axis rule:
multiple policy decisions/snapshots can occur at exactly the same cumulative
travel distance while the robot is stationary. For duplicate x values, the
LAST CSV row wins (chronological last observation), not the numerically largest
y value.
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
    infos: list[dict] = []
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
                raise RuntimeError(f"non-positive stop time for {run_id}: {stop_s}")
            if stop_m <= 0.0:
                raise RuntimeError(f"non-positive stop distance for {run_id}: {stop_m}")
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
    """Return monotone-x interpolation arrays.

    CSV row order is chronological. If several rows share the same x (common on
    distance axes while the robot is stationary), the last CSV row is retained.
    Only after deduplication are x values sorted for np.interp.
    """

    dedup: dict[float, float] = {}
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
        # Intentionally overwrite: chronological last row wins for duplicate x.
        dedup[x] = y

    if endpoint_y is not None:
        if max_x is None:
            raise RuntimeError("endpoint_y requires max_x")
        # Canonical summary endpoint deliberately overrides any raw sample at x.
        dedup[float(max_x)] = float(endpoint_y)

    if not dedup:
        raise RuntimeError(f"no usable {x_key}/{y_key} samples in {path}")

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

    output: list[dict] = []
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
    series: dict[tuple[str, str], tuple[np.ndarray, np.ndarray]] = {}
    info_by_key: dict[tuple[str, str], dict] = {}
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

    output: list[dict] = []
    for x in grid:
        row = {
            x_key: float(x),
            "evaluation_profile": EVAL_PROFILE,
            max_key: max_stop,
            "cutoff_source": cutoff_source,
            "post_completion_semantics": "hold_final_coverage",
        }
        for method, ids in (("NF", BASELINE_NF), ("MapEx", BASELINE_MAPEX)):
            vals: list[float] = []
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
    """Full IoU curve; carry the last real evaluable IoU forward."""

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
        )
        # This guards the key semantic assumption used by hold-last-evaluable.
        if not math.isclose(
            float(ys[-1]), float(info["final_iou"]), rel_tol=1e-7, abs_tol=1e-9
        ):
            raise RuntimeError(
                f"{info['run_id']} final_occupied_iou={info['final_iou']} does not "
                f"match chronological last evaluation {float(ys[-1])} for {x_key}"
            )
        series[key] = (xs, ys)
        last_eval_x[key] = float(xs[-1])

    max_stop = max(float(info[stop_key]) for info in infos)
    grid_max = math.floor(max_stop / step) * step
    grid = np.arange(0.0, grid_max + step * 0.25, step)
    max_key = "max_stop_exact_s" if x_key == "time_s" else "max_stop_exact_m"
    last_eval_key = (
        "max_last_evaluable_exact_s"
        if x_key == "time_s"
        else "max_last_evaluable_exact_m"
    )
    max_last_eval = max(last_eval_x.values())

    output: list[dict] = []
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
            vals: list[float] = []
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
    values: list[float] = []
    for row in read_csv(path):
        try:
            x = float(row[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(x):
            values.append(x)
    return max(values) if values else math.nan


def cutoff_audit(infos: list[dict]) -> list[dict]:
    output: list[dict] = []
    for info in infos:
        stop_s = float(info["stop_s"])
        snapshot_last = last_finite_x(info["path"] / "snapshots.csv", "time_s")
        metrics_last = last_finite_x(info["path"] / "metrics.csv", "time_s")
        evaluation_last = last_finite_x(info["path"] / "evaluation.csv", "time_s")
        evaluation_last_m = last_finite_x(info["path"] / "evaluation.csv", "distance_m")
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
    write_csv(analysis / "new_room_coverage_time_common_curve.csv", coverage_time_common)
    write_csv(
        analysis / "new_room_coverage_distance_common_curve.csv",
        coverage_distance_common,
    )

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
    write_csv(analysis / "new_room_coverage_distance_curve.csv", coverage_distance_full)

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
    write_csv(analysis / "new_room_iou_distance_common_curve.csv", iou_distance_common)

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

    write_csv(analysis / "new_room_time_cutoff_audit.csv", cutoff_audit(infos))

    legacy_full = analysis / "new_room_coverage_time_full_curve.csv"
    if legacy_full.exists():
        legacy_full.unlink()

    print("Refreshed canonical New Room Coverage and IoU curves from 20 baseline runs.")
    print(
        f"Coverage time full: 0-{coverage_time_full[-1]['time_s']} s grid; "
        f"exact max stop {coverage_time_full[-1]['max_stop_exact_s']} s"
    )
    print(
        f"Coverage distance full: 0-{coverage_distance_full[-1]['distance_m']} m grid; "
        f"exact max stop {coverage_distance_full[-1]['max_stop_exact_m']} m"
    )
    print(
        f"IoU time full: 0-{iou_time_full[-1]['time_s']} s grid; "
        f"exact max stop {iou_time_full[-1]['max_stop_exact_s']} s"
    )
    print(
        f"IoU distance full: 0-{iou_distance_full[-1]['distance_m']} m grid; "
        f"exact max stop {iou_distance_full[-1]['max_stop_exact_m']} m"
    )
    print(
        f"Coverage time common: 0-{coverage_time_common[-1]['time_s']} s grid; "
        f"exact common support {coverage_time_common[-1]['common_support_exact_max']} s"
    )
    print(
        f"Coverage distance common: 0-{coverage_distance_common[-1]['distance_m']} m grid; "
        f"exact common support {coverage_distance_common[-1]['common_support_exact_max']} m"
    )
    print(
        f"IoU time common: 0-{iou_time_common[-1]['time_s']} s grid; "
        f"exact common support {iou_time_common[-1]['common_support_exact_max']} s"
    )
    print(
        f"IoU distance common: 0-{iou_distance_common[-1]['distance_m']} m grid; "
        f"exact common support {iou_distance_common[-1]['common_support_exact_max']} m"
    )
    print(
        "Raw snapshots.csv/metrics.csv/evaluation.csv were not modified; raw "
        "evidence remains available for audit."
    )


if __name__ == "__main__":
    main()
