#!/usr/bin/env python3
"""D1 Phase 0 / Gate P: offline MapEx prediction-fidelity analysis.

This script evaluates whether MapEx's decision-time LaMa predictions are useful
for early stopping before any online D1 stop rule is implemented.

Primary reference implemented here: ``first-later-observed``.
For each policy decision t:
  1. load the exact ROS OccupancyGrid used by MapEx at t;
  2. keep only cells that were unknown at t;
  3. crop G1/G2/G3/mean predictions back to the runtime grid;
  4. map those cell centers into later policy-decision OccupancyGrids;
  5. use the first later decision where each cell becomes known as its target;
  6. compare prediction at t against that first later observation.

The final raw map is retained only as a secondary diagnostic reference.

No final/future information is used as an online feature. Future information is
used only as the offline Gate-P target.

Outputs:
  gate_p_decisions.csv
  gate_p_runs.csv
  gate_p_summary.json
  figures/*.png

The canonical run identifier is the run directory name (mpx_001, ...), not the
historical ``metadata.json`` run_id field, because older metadata may contain a
provenance naming mismatch.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


PREDICTION_THRESHOLD = 0.5
PREDICTORS = ("mean", "g1", "g2", "g3")
DEFAULT_RUNS = tuple(f"mpx_{index:03d}" for index in range(1, 11))
STAGE_BOUNDS = (1.0 / 3.0, 2.0 / 3.0)


@dataclass(frozen=True)
class RawGrid:
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float

    @property
    def shape(self) -> tuple[int, int]:
        return int(self.data.shape[0]), int(self.data.shape[1])


def _scalar(npz: np.lib.npyio.NpzFile, key: str, default=None):
    if key not in npz.files:
        if default is not None:
            return default
        raise KeyError(f"Missing NPZ key: {key}")
    value = npz[key]
    if np.asarray(value).shape == ():
        return np.asarray(value).item()
    if np.asarray(value).size == 1:
        return np.asarray(value).reshape(()).item()
    return value


def load_raw_grid(path: Path) -> RawGrid:
    if not path.is_file():
        raise FileNotFoundError(path)
    with np.load(path, allow_pickle=False) as npz:
        data = np.asarray(npz["data"], dtype=np.int16)
        resolution = float(_scalar(npz, "resolution"))
        origin_x = float(_scalar(npz, "origin_x"))
        origin_y = float(_scalar(npz, "origin_y"))
        origin_yaw = float(_scalar(npz, "origin_yaw", 0.0))

        if data.ndim != 2:
            raise ValueError(f"{path}: expected 2-D occupancy grid, got {data.shape}")
        if "height" in npz.files and int(_scalar(npz, "height")) != data.shape[0]:
            raise ValueError(f"{path}: height metadata does not match data shape")
        if "width" in npz.files and int(_scalar(npz, "width")) != data.shape[1]:
            raise ValueError(f"{path}: width metadata does not match data shape")
        if not math.isfinite(resolution) or resolution <= 0.0:
            raise ValueError(f"{path}: invalid resolution {resolution}")

    return RawGrid(
        data=data,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=origin_yaw,
    )


def load_runtime_prediction(path: Path, expected_shape: tuple[int, int]) -> np.ndarray:
    """Load one saved padded prediction and crop it to the decision-time map."""
    if not path.is_file():
        raise FileNotFoundError(path)

    with np.load(path, allow_pickle=False) as npz:
        padded = np.asarray(npz["data"], dtype=np.float32)
        if padded.ndim != 2:
            raise ValueError(f"{path}: expected 2-D prediction, got {padded.shape}")

        source_h = int(_scalar(npz, "source_height"))
        source_w = int(_scalar(npz, "source_width"))
        pad_top = int(_scalar(npz, "pad_top"))
        pad_left = int(_scalar(npz, "pad_left"))

        if (source_h, source_w) != expected_shape:
            raise ValueError(
                f"{path}: prediction source shape {(source_h, source_w)} "
                f"!= observed map shape {expected_shape}"
            )
        if pad_top < 0 or pad_left < 0:
            raise ValueError(f"{path}: negative padding metadata")
        if pad_top + source_h > padded.shape[0] or pad_left + source_w > padded.shape[1]:
            raise ValueError(
                f"{path}: crop {(pad_top, pad_left, source_h, source_w)} "
                f"does not fit padded prediction {padded.shape}"
            )

        runtime = padded[
            pad_top : pad_top + source_h,
            pad_left : pad_left + source_w,
        ].astype(np.float32, copy=False)

    if runtime.shape != expected_shape:
        raise ValueError(
            f"{path}: cropped prediction shape {runtime.shape} != {expected_shape}"
        )
    return runtime


def _read_csv(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def _load_json(path: Path) -> dict:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _resolve_run_path(run_dir: Path, value: str) -> Path:
    value = (value or "").strip()
    if not value:
        raise ValueError(f"{run_dir}: empty artifact path")
    path = Path(value)
    return path if path.is_absolute() else run_dir / path


def find_final_raw_map(run_dir: Path) -> Path:
    """Resolve the final raw map from snapshots.csv, with a strict fallback."""
    snapshots_path = run_dir / "snapshots.csv"
    if snapshots_path.is_file():
        rows = _read_csv(snapshots_path)
        final_rows = [
            row for row in rows if row.get("event", "").strip().lower() == "final"
        ]
        if final_rows:
            value = final_rows[-1].get("raw_map_file", "")
            path = _resolve_run_path(run_dir, value)
            if path.is_file():
                return path

    matches = sorted((run_dir / "maps").glob("snapshot_*_final_raw.npz"))
    if len(matches) != 1:
        raise RuntimeError(
            f"{run_dir}: expected exactly one final raw map; found {len(matches)}"
        )
    return matches[0]


def _cell_centers_world(
    grid: RawGrid,
    rows: np.ndarray,
    cols: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert grid cells to world coordinates, supporting non-zero origin yaw."""
    local_x = (cols.astype(np.float64) + 0.5) * grid.resolution
    local_y = (rows.astype(np.float64) + 0.5) * grid.resolution

    if abs(grid.origin_yaw) < 1e-12:
        return grid.origin_x + local_x, grid.origin_y + local_y

    c = math.cos(grid.origin_yaw)
    s = math.sin(grid.origin_yaw)
    world_x = grid.origin_x + c * local_x - s * local_y
    world_y = grid.origin_y + s * local_x + c * local_y
    return world_x, world_y


def _world_to_cells(
    grid: RawGrid,
    world_x: np.ndarray,
    world_y: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Map world coordinates into a raw occupancy grid."""
    dx = world_x - grid.origin_x
    dy = world_y - grid.origin_y

    if abs(grid.origin_yaw) < 1e-12:
        local_x = dx
        local_y = dy
    else:
        c = math.cos(grid.origin_yaw)
        s = math.sin(grid.origin_yaw)
        # inverse rotation R(-yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy

    # Cell centers should be safely inside their cells. The tiny epsilon only
    # protects exact floating-point boundary cases when origins differ by an
    # integer number of cells.
    eps = 1e-10
    cols = np.floor(local_x / grid.resolution + eps).astype(np.int64)
    rows = np.floor(local_y / grid.resolution + eps).astype(np.int64)

    height, width = grid.shape
    inside = (
        (rows >= 0)
        & (rows < height)
        & (cols >= 0)
        & (cols < width)
    )
    return rows, cols, inside


def _safe_div(numerator: float, denominator: float) -> float:
    if denominator <= 0:
        return math.nan
    return float(numerator / denominator)


def classification_metrics(
    prediction: np.ndarray,
    truth_occupied: np.ndarray,
) -> dict[str, float | int]:
    """Binary fidelity metrics with occupied=1 and free=0."""
    values = np.asarray(prediction, dtype=np.float64)
    truth_occ = np.asarray(truth_occupied, dtype=bool)

    if values.shape != truth_occ.shape:
        raise ValueError("Prediction/truth vector shape mismatch")
    if values.size == 0:
        return {
            "n": 0,
            "tp_occ": 0,
            "tn_free": 0,
            "fp_occ": 0,
            "fn_occ": 0,
            "accuracy": math.nan,
            "free_precision": math.nan,
            "free_recall": math.nan,
            "occupied_precision": math.nan,
            "occupied_recall": math.nan,
            "free_iou": math.nan,
            "occupied_iou": math.nan,
            "macro_iou": math.nan,
            "mae": math.nan,
            "out_of_range_fraction": math.nan,
        }

    if not np.all(np.isfinite(values)):
        raise ValueError("Prediction contains NaN/Inf on evaluated cells")

    pred_occ = values >= PREDICTION_THRESHOLD
    pred_free = ~pred_occ
    truth_free = ~truth_occ

    tp_occ = int(np.count_nonzero(pred_occ & truth_occ))
    tn_free = int(np.count_nonzero(pred_free & truth_free))
    fp_occ = int(np.count_nonzero(pred_occ & truth_free))
    fn_occ = int(np.count_nonzero(pred_free & truth_occ))
    n = int(values.size)

    accuracy = _safe_div(tp_occ + tn_free, n)
    occupied_precision = _safe_div(tp_occ, tp_occ + fp_occ)
    occupied_recall = _safe_div(tp_occ, tp_occ + fn_occ)
    free_precision = _safe_div(tn_free, tn_free + fn_occ)
    free_recall = _safe_div(tn_free, tn_free + fp_occ)
    occupied_iou = _safe_div(tp_occ, tp_occ + fp_occ + fn_occ)
    free_iou = _safe_div(tn_free, tn_free + fp_occ + fn_occ)
    valid_ious = [x for x in (free_iou, occupied_iou) if math.isfinite(x)]
    macro_iou = float(np.mean(valid_ious)) if valid_ious else math.nan

    truth_float = truth_occ.astype(np.float64)
    mae = float(np.mean(np.abs(values - truth_float)))
    out_of_range_fraction = float(np.mean((values < 0.0) | (values > 1.0)))

    return {
        "n": n,
        "tp_occ": tp_occ,
        "tn_free": tn_free,
        "fp_occ": fp_occ,
        "fn_occ": fn_occ,
        "accuracy": accuracy,
        "free_precision": free_precision,
        "free_recall": free_recall,
        "occupied_precision": occupied_precision,
        "occupied_recall": occupied_recall,
        "free_iou": free_iou,
        "occupied_iou": occupied_iou,
        "macro_iou": macro_iou,
        "mae": mae,
        "out_of_range_fraction": out_of_range_fraction,
    }


def metrics_from_counts(tp_occ: int, tn_free: int, fp_occ: int, fn_occ: int) -> dict[str, float]:
    n = tp_occ + tn_free + fp_occ + fn_occ
    occupied_precision = _safe_div(tp_occ, tp_occ + fp_occ)
    occupied_recall = _safe_div(tp_occ, tp_occ + fn_occ)
    free_precision = _safe_div(tn_free, tn_free + fn_occ)
    free_recall = _safe_div(tn_free, tn_free + fp_occ)
    occupied_iou = _safe_div(tp_occ, tp_occ + fp_occ + fn_occ)
    free_iou = _safe_div(tn_free, tn_free + fp_occ + fn_occ)
    valid_ious = [x for x in (free_iou, occupied_iou) if math.isfinite(x)]
    return {
        "accuracy": _safe_div(tp_occ + tn_free, n),
        "free_precision": free_precision,
        "free_recall": free_recall,
        "occupied_precision": occupied_precision,
        "occupied_recall": occupied_recall,
        "free_iou": free_iou,
        "occupied_iou": occupied_iou,
        "macro_iou": float(np.mean(valid_ious)) if valid_ious else math.nan,
    }


def _stage(progress: float) -> str:
    if progress < STAGE_BOUNDS[0]:
        return "early"
    if progress < STAGE_BOUNDS[1]:
        return "mid"
    return "late"


def _nanmean(values: Iterable[float]) -> float:
    arr = np.asarray([float(v) for v in values], dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    return float(np.mean(arr)) if arr.size else math.nan


def _nanstd(values: Iterable[float]) -> float:
    arr = np.asarray([float(v) for v in values], dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    return float(np.std(arr, ddof=1)) if arr.size > 1 else (0.0 if arr.size == 1 else math.nan)


def _add_prefixed_metrics(row: dict, prefix: str, metrics: dict) -> None:
    for key, value in metrics.items():
        row[f"{prefix}_{key}"] = value


def _float_or_nan(value) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return math.nan
    return result if math.isfinite(result) else math.nan


def _percentile_or_nan(values: np.ndarray, percentile: float) -> float:
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return math.nan
    return float(np.percentile(values, percentile))


def first_later_observation_targets(
    observed: RawGrid,
    unknown_rows: np.ndarray,
    unknown_cols: np.ndarray,
    decision_index: int,
    decision_table: list[dict[str, str]],
    decision_grids: list[RawGrid],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Find the first later policy-decision map where each source cell is known.

    decision_index is 1-based. A target is never taken from the current
    decision. Cells that remain unknown through all later policy decisions keep
    label/reveal metadata at the sentinel values.
    """
    count = int(unknown_rows.size)
    labels = np.full(count, -1, dtype=np.int16)
    reveal_decision_ids = np.full(count, -1, dtype=np.int64)
    decisions_until_reveal = np.full(count, -1, dtype=np.int64)
    time_until_reveal_s = np.full(count, np.nan, dtype=np.float64)

    if count == 0 or decision_index >= len(decision_table):
        return labels, reveal_decision_ids, decisions_until_reveal, time_until_reveal_s

    world_x, world_y = _cell_centers_world(observed, unknown_rows, unknown_cols)
    unresolved = np.ones(count, dtype=bool)
    current_time_s = _float_or_nan(decision_table[decision_index - 1].get("time_s"))

    # decision_index is 1-based, therefore list position decision_index is the
    # immediately following policy decision.
    for future_pos in range(decision_index, len(decision_table)):
        future_grid = decision_grids[future_pos]
        future_rows, future_cols, inside = _world_to_cells(
            future_grid,
            world_x,
            world_y,
        )

        candidate_mask = unresolved & inside
        if not np.any(candidate_mask):
            continue

        candidate_indices = np.flatnonzero(candidate_mask)
        candidate_labels = future_grid.data[
            future_rows[candidate_indices],
            future_cols[candidate_indices],
        ]
        known_local = candidate_labels >= 0
        if not np.any(known_local):
            continue

        revealed_indices = candidate_indices[known_local]
        revealed_labels = candidate_labels[known_local]
        labels[revealed_indices] = revealed_labels

        future_row = decision_table[future_pos]
        reveal_decision_ids[revealed_indices] = int(future_row["decision_id"])
        decisions_until_reveal[revealed_indices] = (future_pos + 1) - decision_index

        future_time_s = _float_or_nan(future_row.get("time_s"))
        if math.isfinite(current_time_s) and math.isfinite(future_time_s):
            time_until_reveal_s[revealed_indices] = future_time_s - current_time_s

        unresolved[revealed_indices] = False
        if not np.any(unresolved):
            break

    return labels, reveal_decision_ids, decisions_until_reveal, time_until_reveal_s


def analyze_decision(
    run_dir: Path,
    decision_row: dict[str, str],
    decision_table: list[dict[str, str]],
    decision_grids: list[RawGrid],
    final_grid: RawGrid,
    decision_index: int,
    total_decisions: int,
    metadata: dict,
    late_n: int,
) -> dict:
    decision_id = int(decision_row["decision_id"])
    raw_map_path = _resolve_run_path(run_dir, decision_row["raw_map"])
    observed = decision_grids[decision_index - 1]

    unknown_mask = observed.data < 0
    unknown_rows, unknown_cols = np.nonzero(unknown_mask)
    unknown_count = int(unknown_rows.size)

    (
        first_labels,
        reveal_decision_ids,
        decisions_until_reveal,
        time_until_reveal_s,
    ) = first_later_observation_targets(
        observed=observed,
        unknown_rows=unknown_rows,
        unknown_cols=unknown_cols,
        decision_index=decision_index,
        decision_table=decision_table,
        decision_grids=decision_grids,
    )

    first_observed = first_labels >= 0
    eval_rows = unknown_rows[first_observed]
    eval_cols = unknown_cols[first_observed]
    truth_labels = first_labels[first_observed]
    truth_occupied = truth_labels > 0

    reveal_steps_eval = decisions_until_reveal[first_observed]
    reveal_time_eval = time_until_reveal_s[first_observed]
    reveal_ids_eval = reveal_decision_ids[first_observed]

    # Secondary diagnostic only: compare the same decision-time unknown cells
    # against the final raw map. This is not the primary Gate-P target.
    world_x, world_y = _cell_centers_world(observed, unknown_rows, unknown_cols)
    final_rows, final_cols, final_inside = _world_to_cells(
        final_grid,
        world_x,
        world_y,
    )
    final_labels = np.full(unknown_count, -1, dtype=np.int16)
    if np.any(final_inside):
        final_labels[final_inside] = final_grid.data[
            final_rows[final_inside],
            final_cols[final_inside],
        ]
    final_observed = final_inside & (final_labels >= 0)
    final_eval_rows = unknown_rows[final_observed]
    final_eval_cols = unknown_cols[final_observed]
    final_truth_labels = final_labels[final_observed]
    final_truth_occupied = final_truth_labels > 0

    both_known = first_observed & final_observed
    first_vs_final_count = int(np.count_nonzero(both_known))
    if first_vs_final_count:
        first_vs_final_agreement = float(
            np.mean((first_labels[both_known] > 0) == (final_labels[both_known] > 0))
        )
    else:
        first_vs_final_agreement = math.nan

    progress = float(decision_index / max(total_decisions, 1))
    row: dict[str, object] = {
        "run_id": run_dir.name,
        "metadata_run_id": metadata.get("run_id", ""),
        "environment": metadata.get("environment", ""),
        "termination_reason": metadata.get("termination_reason", ""),
        "decision_id": decision_id,
        "decision_index": decision_index,
        "total_decisions": total_decisions,
        "progress_fraction": progress,
        "stage": _stage(progress),
        "decisions_to_end": total_decisions - decision_index,
        "is_last_n": int((total_decisions - decision_index) < late_n),
        "sim_time_s": decision_row.get("time_s", ""),
        "observed_unknown_cells": unknown_count,
        "evaluated_cell_count": int(truth_labels.size),
        "evaluated_free_count": int(np.count_nonzero(truth_labels == 0)),
        "evaluated_occupied_count": int(np.count_nonzero(truth_labels > 0)),
        "future_observed_fraction_of_unknown": _safe_div(
            int(truth_labels.size),
            unknown_count,
        ),
        "first_reveal_decision_id": (
            int(np.min(reveal_ids_eval)) if reveal_ids_eval.size else ""
        ),
        "last_reveal_decision_id": (
            int(np.max(reveal_ids_eval)) if reveal_ids_eval.size else ""
        ),
        "mean_decisions_until_reveal": (
            float(np.mean(reveal_steps_eval)) if reveal_steps_eval.size else math.nan
        ),
        "median_decisions_until_reveal": (
            float(np.median(reveal_steps_eval)) if reveal_steps_eval.size else math.nan
        ),
        "p90_decisions_until_reveal": _percentile_or_nan(reveal_steps_eval, 90.0),
        "mean_time_until_reveal_s": _nanmean(reveal_time_eval),
        "median_time_until_reveal_s": _percentile_or_nan(reveal_time_eval, 50.0),
        "p90_time_until_reveal_s": _percentile_or_nan(reveal_time_eval, 90.0),
        "revealed_next_decision_fraction": _safe_div(
            int(np.count_nonzero(reveal_steps_eval == 1)),
            int(reveal_steps_eval.size),
        ),
        "finalref_evaluated_cell_count": int(final_truth_labels.size),
        "first_vs_final_common_cell_count": first_vs_final_count,
        "first_vs_final_class_agreement": first_vs_final_agreement,
        "raw_map": str(raw_map_path.relative_to(run_dir)),
    }

    predictor_paths = {
        "g1": decision_row.get("g1_map", ""),
        "g2": decision_row.get("g2_map", ""),
        "g3": decision_row.get("g3_map", ""),
        "mean": decision_row.get("mean_map", ""),
    }
    for predictor in PREDICTORS:
        path = _resolve_run_path(run_dir, predictor_paths[predictor])
        runtime_prediction = load_runtime_prediction(path, observed.shape)

        primary_values = runtime_prediction[eval_rows, eval_cols]
        primary_metrics = classification_metrics(primary_values, truth_occupied)
        _add_prefixed_metrics(row, predictor, primary_metrics)

        final_values = runtime_prediction[final_eval_rows, final_eval_cols]
        final_metrics = classification_metrics(final_values, final_truth_occupied)
        _add_prefixed_metrics(row, f"finalref_{predictor}", final_metrics)

    return row

def aggregate_run(
    run_dir: Path,
    decision_rows: list[dict],
    metadata: dict,
    final_map: Path,
    late_n: int,
) -> dict:
    row: dict[str, object] = {
        "run_id": run_dir.name,
        "metadata_run_id": metadata.get("run_id", ""),
        "metadata_run_id_matches_directory": int(metadata.get("run_id") == run_dir.name),
        "environment": metadata.get("environment", ""),
        "termination_reason": metadata.get("termination_reason", ""),
        "final_coverage": metadata.get("final_coverage", ""),
        "decisions_total": len(decision_rows),
        "decisions_evaluated": sum(int(d["evaluated_cell_count"]) > 0 for d in decision_rows),
        "prediction_target_pairs": sum(int(d["evaluated_cell_count"]) for d in decision_rows),
        "late_n": late_n,
        "late_decisions_evaluated": sum(
            int(d["evaluated_cell_count"]) > 0 and int(d["is_last_n"]) == 1
            for d in decision_rows
        ),
        "mean_decisions_until_reveal_decision_macro": _nanmean(
            d["mean_decisions_until_reveal"] for d in decision_rows
        ),
        "mean_time_until_reveal_s_decision_macro": _nanmean(
            d["mean_time_until_reveal_s"] for d in decision_rows
        ),
        "first_vs_final_class_agreement_decision_macro": _nanmean(
            d["first_vs_final_class_agreement"] for d in decision_rows
        ),
        "final_reference_map": str(final_map.relative_to(run_dir)),
    }

    for predictor in PREDICTORS:
        tp = sum(int(d[f"{predictor}_tp_occ"]) for d in decision_rows)
        tn = sum(int(d[f"{predictor}_tn_free"]) for d in decision_rows)
        fp = sum(int(d[f"{predictor}_fp_occ"]) for d in decision_rows)
        fn = sum(int(d[f"{predictor}_fn_occ"]) for d in decision_rows)
        micro = metrics_from_counts(tp, tn, fp, fn)

        for key, value in micro.items():
            row[f"{predictor}_{key}_micro"] = value

        for metric_name in (
            "accuracy",
            "free_precision",
            "free_recall",
            "occupied_precision",
            "occupied_recall",
            "free_iou",
            "occupied_iou",
            "macro_iou",
            "mae",
            "out_of_range_fraction",
        ):
            row[f"{predictor}_{metric_name}_decision_macro"] = _nanmean(
                d[f"{predictor}_{metric_name}"] for d in decision_rows
            )

        late_rows = [
            d for d in decision_rows
            if int(d["is_last_n"]) == 1 and int(d["evaluated_cell_count"]) > 0
        ]
        for metric_name in ("accuracy", "macro_iou", "mae"):
            row[f"{predictor}_late_{metric_name}_decision_macro"] = _nanmean(
                d[f"{predictor}_{metric_name}"] for d in late_rows
            )

        # Secondary final-map reference is summarized separately so it cannot be
        # confused with the primary first-later-observed Gate-P result.
        for metric_name in ("accuracy", "macro_iou", "mae"):
            row[f"finalref_{predictor}_{metric_name}_decision_macro"] = _nanmean(
                d[f"finalref_{predictor}_{metric_name}"] for d in decision_rows
            )

    return row

def _json_number(value):
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        value = float(value)
        return value if math.isfinite(value) else None
    return value


def build_summary(
    run_rows: list[dict],
    decision_rows: list[dict],
    requested_runs: list[str],
    late_n: int,
) -> dict:
    metadata_mismatches = [
        {
            "directory_run_id": row["run_id"],
            "metadata_run_id": row["metadata_run_id"],
        }
        for row in run_rows
        if not int(row["metadata_run_id_matches_directory"])
    ]

    summary: dict[str, object] = {
        "gate": "D1 Phase 0 / Gate P",
        "reference": "first_later_policy_decision_observation",
        "prediction_threshold": PREDICTION_THRESHOLD,
        "late_n_decisions": late_n,
        "requested_runs": requested_runs,
        "runs_analyzed": len(run_rows),
        "decisions_analyzed": len(decision_rows),
        "decisions_with_later_observed_targets": sum(
            int(row["evaluated_cell_count"]) > 0 for row in decision_rows
        ),
        "prediction_target_pairs": sum(
            int(row["evaluated_cell_count"]) for row in decision_rows
        ),
        "metadata_run_id_mismatches": metadata_mismatches,
        "predictors": {},
        "stage_summary": {},
        "notes": [
            "Primary targets are cells unknown at decision t that become known in a later policy-decision map.",
            "Each cell uses its first later known observation as the primary target.",
            "The final raw map is retained only as a secondary diagnostic reference.",
            "Future data are offline targets only and are never runtime D1 inputs.",
            "Run directory names are canonical run IDs for this analysis.",
            "Run-level macro summaries are reported to avoid treating all spatial cells as independent runs.",
        ],
    }

    predictor_summary = {}
    for predictor in PREDICTORS:
        run_accuracy = [
            row[f"{predictor}_accuracy_decision_macro"] for row in run_rows
        ]
        run_iou = [
            row[f"{predictor}_macro_iou_decision_macro"] for row in run_rows
        ]
        run_mae = [
            row[f"{predictor}_mae_decision_macro"] for row in run_rows
        ]
        run_late_accuracy = [
            row[f"{predictor}_late_accuracy_decision_macro"] for row in run_rows
        ]
        run_late_iou = [
            row[f"{predictor}_late_macro_iou_decision_macro"] for row in run_rows
        ]
        run_late_mae = [
            row[f"{predictor}_late_mae_decision_macro"] for row in run_rows
        ]

        tp = sum(int(row[f"{predictor}_tp_occ"]) for row in decision_rows)
        tn = sum(int(row[f"{predictor}_tn_free"]) for row in decision_rows)
        fp = sum(int(row[f"{predictor}_fp_occ"]) for row in decision_rows)
        fn = sum(int(row[f"{predictor}_fn_occ"]) for row in decision_rows)
        overall_micro = metrics_from_counts(tp, tn, fp, fn)

        predictor_summary[predictor] = {
            "run_macro_accuracy_mean": _nanmean(run_accuracy),
            "run_macro_accuracy_std": _nanstd(run_accuracy),
            "run_macro_macro_iou_mean": _nanmean(run_iou),
            "run_macro_macro_iou_std": _nanstd(run_iou),
            "run_macro_mae_mean": _nanmean(run_mae),
            "run_macro_mae_std": _nanstd(run_mae),
            "late_run_macro_accuracy_mean": _nanmean(run_late_accuracy),
            "late_run_macro_accuracy_std": _nanstd(run_late_accuracy),
            "late_run_macro_macro_iou_mean": _nanmean(run_late_iou),
            "late_run_macro_macro_iou_std": _nanstd(run_late_iou),
            "late_run_macro_mae_mean": _nanmean(run_late_mae),
            "late_run_macro_mae_std": _nanstd(run_late_mae),
            "overall_prediction_pair_micro": overall_micro,
        }
    summary["predictors"] = predictor_summary

    stage_summary = {}
    for stage in ("early", "mid", "late"):
        stage_rows = [row for row in decision_rows if row["stage"] == stage]
        stage_summary[stage] = {}
        for predictor in PREDICTORS:
            stage_summary[stage][predictor] = {
                "decisions": len(stage_rows),
                "accuracy_decision_macro": _nanmean(
                    row[f"{predictor}_accuracy"] for row in stage_rows
                ),
                "macro_iou_decision_macro": _nanmean(
                    row[f"{predictor}_macro_iou"] for row in stage_rows
                ),
                "mae_decision_macro": _nanmean(
                    row[f"{predictor}_mae"] for row in stage_rows
                ),
            }
    summary["stage_summary"] = stage_summary

    def clean(value):
        if isinstance(value, dict):
            return {key: clean(val) for key, val in value.items()}
        if isinstance(value, list):
            return [clean(val) for val in value]
        return _json_number(value)

    return clean(summary)


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError(f"No rows to write: {path}")
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="raise")
        writer.writeheader()
        writer.writerows(rows)


def make_figures(decision_rows: list[dict], run_rows: list[dict], output_dir: Path) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not installed; skipping figures")
        return []

    figure_dir = output_dir / "gate_p_plots"
    figure_dir.mkdir(parents=True, exist_ok=True)
    written: list[str] = []

    by_run: dict[str, list[dict]] = {}
    for row in decision_rows:
        by_run.setdefault(str(row["run_id"]), []).append(row)

    def line_plot(metric: str, ylabel: str, filename: str) -> None:
        fig, ax = plt.subplots(figsize=(8.5, 5.0))
        for run_id, rows in sorted(by_run.items()):
            rows = sorted(rows, key=lambda item: int(item["decision_index"]))
            x = [float(item["progress_fraction"]) for item in rows]
            y = [float(item[metric]) for item in rows]
            ax.plot(x, y, marker=".", linewidth=1.0, label=run_id)
        ax.set_xlabel("Exploration progress (decision index / total decisions)")
        ax.set_ylabel(ylabel)
        ax.set_xlim(0.0, 1.02)
        ax.grid(True, alpha=0.25)
        ax.legend(ncol=2, fontsize=8)
        fig.tight_layout()
        path = figure_dir / filename
        fig.savefig(path, dpi=160)
        plt.close(fig)
        written.append(str(path))

    line_plot(
        "mean_accuracy",
        "Mean-prediction accuracy",
        "accuracy_vs_decision_progress.png",
    )
    line_plot(
        "mean_macro_iou",
        "Mean-prediction macro IoU",
        "macro_iou_vs_decision_progress.png",
    )
    line_plot(
        "mean_mae",
        "Mean-prediction absolute error",
        "mae_vs_decision_progress.png",
    )

    fig, ax = plt.subplots(figsize=(9.0, 5.0))
    labels = [str(row["run_id"]) for row in run_rows]
    values = [
        float(row["mean_late_accuracy_decision_macro"])
        for row in run_rows
    ]
    ax.bar(labels, values)
    ax.set_xlabel("Run")
    ax.set_ylabel("Last-N mean-prediction accuracy")
    ax.set_ylim(0.0, 1.0)
    ax.tick_params(axis="x", rotation=45)
    ax.grid(True, axis="y", alpha=0.25)
    fig.tight_layout()
    path = figure_dir / "late_stage_accuracy_by_run.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    written.append(str(path))

    return written


def parse_args() -> argparse.Namespace:
    script_path = Path(__file__).resolve()
    mapex_lab_root = script_path.parents[2]

    parser = argparse.ArgumentParser(
        description=(
            "D1 Gate P: evaluate MapEx prediction fidelity on cells unknown at "
            "decision t and observed later in the same baseline run."
        )
    )
    parser.add_argument(
        "--experiments-root",
        type=Path,
        default=mapex_lab_root / "experiments" / "mapex",
        help="Directory containing MapEx run folders.",
    )
    parser.add_argument(
        "--runs",
        nargs="+",
        default=list(DEFAULT_RUNS),
        help="Run directories to analyze (default: mpx_001 ... mpx_010).",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=script_path.parent / "results" / "gate_p",
        help="Output directory for CSV, JSON and figures.",
    )
    parser.add_argument(
        "--late-n",
        type=int,
        default=10,
        help="Number of final policy decisions used for late-stage summaries.",
    )
    parser.add_argument(
        "--no-figures",
        action="store_true",
        help="Write CSV/JSON only.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.late_n <= 0:
        raise ValueError("--late-n must be > 0")

    experiments_root = args.experiments_root.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    all_decisions: list[dict] = []
    all_runs: list[dict] = []

    print(f"[Gate P] experiments root: {experiments_root}")
    print(f"[Gate P] output dir:       {output_dir}")
    print(f"[Gate P] runs:             {', '.join(args.runs)}")
    print(f"[Gate P] threshold:        {PREDICTION_THRESHOLD}")
    print("[Gate P] reference:        first later policy-decision observation")

    for run_name in args.runs:
        run_dir = experiments_root / run_name
        if not run_dir.is_dir():
            raise FileNotFoundError(f"Run directory not found: {run_dir}")

        metadata = _load_json(run_dir / "metadata.json")
        metadata_run_id = metadata.get("run_id")
        if metadata_run_id and metadata_run_id != run_name:
            print(
                f"[WARN] {run_name}: metadata run_id={metadata_run_id!r}; "
                "using directory name as canonical run ID"
            )

        decision_table = _read_csv(run_dir / "decisions.csv")
        decision_table = sorted(
            decision_table,
            key=lambda row: int(row["decision_id"]),
        )
        if not decision_table:
            raise RuntimeError(f"{run_dir}: decisions.csv has no decisions")

        final_map_path = find_final_raw_map(run_dir)
        final_grid = load_raw_grid(final_map_path)

        # Preload exact decision-time raw grids once per run. Gate P then uses
        # future policy-decision maps to find the first saved observation where
        # each decision-time unknown cell becomes known.
        decision_grids = [
            load_raw_grid(_resolve_run_path(run_dir, row["raw_map"]))
            for row in decision_table
        ]

        run_decisions: list[dict] = []
        total = len(decision_table)
        for index, decision_row in enumerate(decision_table, start=1):
            result = analyze_decision(
                run_dir=run_dir,
                decision_row=decision_row,
                decision_table=decision_table,
                decision_grids=decision_grids,
                final_grid=final_grid,
                decision_index=index,
                total_decisions=total,
                metadata=metadata,
                late_n=args.late_n,
            )
            run_decisions.append(result)

        run_row = aggregate_run(
            run_dir=run_dir,
            decision_rows=run_decisions,
            metadata=metadata,
            final_map=final_map_path,
            late_n=args.late_n,
        )
        all_decisions.extend(run_decisions)
        all_runs.append(run_row)

        print(
            f"[Gate P] {run_name}: {total} decisions, "
            f"{run_row['prediction_target_pairs']} prediction-target pairs, "
            f"mean accuracy={run_row['mean_accuracy_decision_macro']:.4f}, "
            f"late accuracy={run_row['mean_late_accuracy_decision_macro']:.4f}"
        )

    decision_csv = output_dir / "gate_p_decisions.csv"
    run_csv = output_dir / "gate_p_runs.csv"
    summary_json = output_dir / "gate_p_summary.json"

    _write_csv(decision_csv, all_decisions)
    _write_csv(run_csv, all_runs)

    summary = build_summary(
        run_rows=all_runs,
        decision_rows=all_decisions,
        requested_runs=list(args.runs),
        late_n=args.late_n,
    )
    summary_json.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    figures = []
    if not args.no_figures:
        figures = make_figures(all_decisions, all_runs, output_dir)

    print(f"[Gate P] wrote {decision_csv}")
    print(f"[Gate P] wrote {run_csv}")
    print(f"[Gate P] wrote {summary_json}")
    for figure in figures:
        print(f"[Gate P] wrote {figure}")

    mean_summary = summary["predictors"]["mean"]
    print(
        "[Gate P] mean predictor across runs: "
        f"accuracy={mean_summary['run_macro_accuracy_mean']:.4f} ± "
        f"{mean_summary['run_macro_accuracy_std']:.4f}, "
        f"late accuracy={mean_summary['late_run_macro_accuracy_mean']:.4f} ± "
        f"{mean_summary['late_run_macro_accuracy_std']:.4f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
