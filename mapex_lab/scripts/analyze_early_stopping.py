#!/usr/bin/env python3
"""Offline per-decision analysis for MapEx early-stopping research.

This script does not change the online MapEx policy. It replays each recorded
MapEx policy decision with saved ensemble-mean/variance predictions and asks:

    "If exploration stopped immediately before executing this decision's goal,
     what map quality would we have, and how much time/distance would be saved?"

Primary outputs are written to ``early_stopping_analysis.csv`` inside the run
directory (or to ``--output`` when supplied).

Stopping-signal quantities are deliberately computed only from information that
was available online at the decision:
- the recorded observed raw map;
- the recorded ensemble variance map.

Ground truth / ROI are used only for offline evaluation metrics such as Coverage,
occupied IoU, and Topological Understanding (TU). No stopping threshold is
assumed by default. ``high_uncertainty_fraction`` is only populated when
``--high-variance-threshold`` is explicitly provided.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Iterable

import numpy as np

import evaluate_mapex_run as evaluator


DEFAULT_GT_BY_ENVIRONMENT = {
    "new_room": "ground_truth/new_room/generated/new_room_structural_gt_v2.npz",
    "hospital": "ground_truth/hospital/generated/hospital_structural_gt_v1.npz",
}

DEFAULT_ROI_BY_ENVIRONMENT = {
    "new_room": "ground_truth/new_room/generated/new_room_connected_free_v2.npy",
    "hospital": "ground_truth/hospital/generated/hospital_connected_free_v1.npy",
}

OUTPUT_FIELDS = [
    "decision_id",
    "time_s",
    "distance_m",
    "coverage",
    "known_fraction",
    "unknown_cells",
    "unknown_mean_variance",
    "unknown_total_variance",
    "unknown_max_variance",
    "unknown_variance_p90",
    "unknown_variance_p95",
    "high_variance_threshold",
    "high_uncertainty_fraction",
    "iou_if_stop",
    "tu_if_stop",
    "tu_succeeded",
    "tu_failed",
    "time_saved_s",
    "distance_saved_m",
    "time_saved_fraction",
    "distance_saved_fraction",
    "iou_loss_vs_final",
    "tu_loss_vs_final",
    "final_reference_decision_id",
    "raw_map",
    "mean_map",
    "variance_map",
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


def _fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, (np.integer, int)):
        return str(int(value))
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(number):
        return "nan"
    if math.isinf(number):
        return "inf" if number > 0 else "-inf"
    return f"{number:.9f}"


def _load_npz_data(path: Path, dtype=None) -> np.ndarray:
    with np.load(path) as bundle:
        if "data" not in bundle.files:
            raise ValueError(f"{path} does not contain a 'data' array")
        data = np.asarray(bundle["data"])
    if dtype is not None:
        data = data.astype(dtype, copy=False)
    return data


def _scalar(bundle, key: str, default=None):
    if key not in bundle.files:
        return default
    value = np.asarray(bundle[key])
    if value.size != 1:
        raise ValueError(f"metadata '{key}' in NPZ must be scalar")
    return value.reshape(-1)[0].item()


def _variance_on_observed_grid(
    variance_path: Path,
    observed_raw: np.ndarray,
) -> np.ndarray:
    """Crop padded MapEx variance back to the exact recorded runtime map grid."""
    with np.load(variance_path) as bundle:
        variance = np.asarray(bundle["data"], dtype=np.float32)
        source_h = int(_scalar(bundle, "source_height"))
        source_w = int(_scalar(bundle, "source_width"))
        pad_top = int(_scalar(bundle, "pad_top", 0))
        pad_left = int(_scalar(bundle, "pad_left", 0))

    if observed_raw.shape != (source_h, source_w):
        raise ValueError(
            "observed/variance source shape mismatch: "
            f"observed={observed_raw.shape}, variance_source={(source_h, source_w)}"
        )
    if variance.ndim != 2:
        raise ValueError(f"variance map must be 2-D, got {variance.shape}")

    row_end = pad_top + source_h
    col_end = pad_left + source_w
    if (
        pad_top < 0
        or pad_left < 0
        or row_end > variance.shape[0]
        or col_end > variance.shape[1]
    ):
        raise ValueError(
            "invalid variance padding metadata: "
            f"variance={variance.shape}, source={(source_h, source_w)}, "
            f"pad={(pad_top, pad_left)}"
        )
    return variance[pad_top:row_end, pad_left:col_end]


def _uncertainty_metrics(
    observed_raw: np.ndarray,
    variance_source: np.ndarray,
    high_threshold: float | None,
) -> dict[str, float | int]:
    """Compute deployable uncertainty statistics on currently-unknown cells."""
    unknown = observed_raw < 0
    values = np.asarray(variance_source[unknown], dtype=np.float64)
    values = values[np.isfinite(values)]

    if values.size == 0:
        return {
            "unknown_cells": 0,
            "unknown_mean_variance": math.nan,
            "unknown_total_variance": 0.0,
            "unknown_max_variance": math.nan,
            "unknown_variance_p90": math.nan,
            "unknown_variance_p95": math.nan,
            "high_uncertainty_fraction": math.nan,
        }

    high_fraction = math.nan
    if high_threshold is not None:
        high_fraction = float(np.count_nonzero(values >= high_threshold) / values.size)

    return {
        "unknown_cells": int(values.size),
        "unknown_mean_variance": float(np.mean(values)),
        "unknown_total_variance": float(np.sum(values)),
        "unknown_max_variance": float(np.max(values)),
        "unknown_variance_p90": float(np.percentile(values, 90)),
        "unknown_variance_p95": float(np.percentile(values, 95)),
        "high_uncertainty_fraction": high_fraction,
    }


def _reconstructed_canvas(mean_path: Path, observed_canvas_path: Path) -> np.ndarray:
    """Observed evidence + MapEx ensemble mean for the remaining unknown area."""
    prediction = evaluator._prediction_to_canvas(mean_path)
    observed = _load_npz_data(observed_canvas_path, dtype=np.int16)
    if observed.shape != prediction.shape:
        raise ValueError(
            f"observed canvas {observed.shape} != prediction canvas {prediction.shape}"
        )

    reconstructed = np.asarray(prediction, dtype=np.float32).copy()
    known = observed >= 0
    # MapEx's ROS-map conversion treats every known occupancy value >0 as occupied.
    reconstructed[known] = (observed[known] > 0).astype(np.float32)
    return reconstructed


def _coverage_and_known_fraction(
    observed_canvas_path: Path,
    roi: np.ndarray | None,
) -> tuple[float, float]:
    observed = _load_npz_data(observed_canvas_path, dtype=np.int16)
    known = observed >= 0
    known_fraction = float(np.count_nonzero(known) / known.size)
    if roi is None:
        return math.nan, known_fraction
    if roi.shape != observed.shape:
        raise ValueError(f"ROI shape {roi.shape} != observed canvas {observed.shape}")
    denominator = int(np.count_nonzero(roi))
    if denominator <= 0:
        return math.nan, known_fraction
    coverage = float(np.count_nonzero(known & roi) / denominator)
    return coverage, known_fraction


def _resolve_recorded_or_default_path(
    explicit: str | None,
    metadata_value,
    environment: str,
    defaults: dict[str, str],
    mapex_lab: Path,
    label: str,
) -> Path:
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    if metadata_value:
        candidates.append(Path(str(metadata_value)).expanduser())
    relative = defaults.get(environment)
    if relative:
        candidates.append(mapex_lab / relative)

    for path in candidates:
        resolved = path.resolve()
        if resolved.is_file():
            return resolved

    rendered = ", ".join(str(path) for path in candidates) or "<none>"
    raise FileNotFoundError(f"Could not resolve {label}; tried: {rendered}")


def _load_summary(run_dir: Path) -> dict:
    path = run_dir / "summary.json"
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def _total_from_summary_or_trajectory(
    summary: dict,
    trajectory: Iterable[dict[str, str]],
) -> tuple[float, float]:
    total_time = _finite_float(summary.get("total_time_s"))
    total_distance = _finite_float(summary.get("total_distance_m"))

    trajectory = list(trajectory)
    if not math.isfinite(total_time):
        times = [_finite_float(row.get("time_s")) for row in trajectory]
        times = [value for value in times if math.isfinite(value)]
        total_time = max(times) if times else math.nan
    if not math.isfinite(total_distance):
        distances = [
            _finite_float(row.get("cumulative_distance_m")) for row in trajectory
        ]
        distances = [value for value in distances if math.isfinite(value)]
        total_distance = max(distances) if distances else math.nan
    return total_time, total_distance


def _write_output(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def analyze_run(
    run_dir: str | Path,
    *,
    output_path: str | Path | None = None,
    ground_truth_path: str | None = None,
    roi_path: str | None = None,
    tu_goal_count: int = evaluator.TU_GOAL_COUNT,
    tu_seed: int = evaluator.TU_RANDOM_SEED,
    high_variance_threshold: float | None = None,
) -> dict:
    run_dir = Path(run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(run_dir)
    if high_variance_threshold is not None and high_variance_threshold < 0:
        raise ValueError("--high-variance-threshold must be >= 0")
    if tu_goal_count <= 0:
        raise ValueError("--tu-goals must be > 0")

    mapex_lab = Path(__file__).resolve().parents[1]
    metadata_path = run_dir / "metadata.json"
    decisions_path = run_dir / "decisions.csv"
    if not metadata_path.is_file():
        raise FileNotFoundError(metadata_path)
    if not decisions_path.is_file():
        raise FileNotFoundError(decisions_path)

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("method") not in (None, "mapex"):
        raise ValueError(
            f"Expected a MapEx run, metadata method={metadata.get('method')!r}"
        )
    environment = str(metadata.get("environment") or "new_room")

    gt_path = _resolve_recorded_or_default_path(
        ground_truth_path,
        metadata.get("structural_ground_truth_file"),
        environment,
        DEFAULT_GT_BY_ENVIRONMENT,
        mapex_lab,
        "structural ground truth",
    )
    resolved_roi_path = _resolve_recorded_or_default_path(
        roi_path,
        metadata.get("evaluation_roi_file"),
        environment,
        DEFAULT_ROI_BY_ENVIRONMENT,
        mapex_lab,
        "evaluation ROI",
    )

    evaluation_mask, gt_occ, gt_free = evaluator._load_ground_truth(gt_path)
    roi = np.load(resolved_roi_path).astype(bool)
    if roi.shape != evaluation_mask.shape:
        raise ValueError(f"ROI shape {roi.shape} != GT shape {evaluation_mask.shape}")

    start_x = metadata.get("evaluation_start_x")
    start_y = metadata.get("evaluation_start_y")
    if start_x is None or start_y is None:
        raise ValueError("metadata lacks evaluation_start_x/evaluation_start_y")
    start = evaluator._world_to_cell(float(start_x), float(start_y))
    if not evaluator._inside(gt_free.shape, start) or not gt_free[start]:
        raise ValueError(f"evaluation start cell {start} is not GT free")

    valid_goals = roi & gt_free
    goals = evaluator._sample_goals(valid_goals, int(tu_goal_count), int(tu_seed))

    decisions = _read_csv(decisions_path)
    trajectory_path = run_dir / "trajectory.csv"
    trajectory = _read_csv(trajectory_path) if trajectory_path.is_file() else []
    summary = _load_summary(run_dir)
    total_time, total_distance = _total_from_summary_or_trajectory(summary, trajectory)

    rows: list[dict] = []
    skipped: list[dict] = []
    planner_used = None

    for decision in decisions:
        decision_id_raw = (decision.get("decision_id") or "").strip()
        raw_rel = (decision.get("raw_map") or "").strip()
        canvas_rel = (decision.get("canvas_map") or "").strip()
        mean_rel = (decision.get("mean_map") or "").strip()
        variance_rel = (decision.get("variance_map") or "").strip()
        if not all((decision_id_raw, raw_rel, canvas_rel, mean_rel, variance_rel)):
            skipped.append(
                {
                    "decision_id": decision_id_raw,
                    "reason": "missing raw/canvas/mean/variance path",
                }
            )
            continue

        try:
            decision_id = int(decision_id_raw)
            decision_time = float(decision["time_s"])
            raw_path = run_dir / raw_rel
            canvas_path = run_dir / canvas_rel
            mean_path = run_dir / mean_rel
            variance_path = run_dir / variance_rel
            for required in (raw_path, canvas_path, mean_path, variance_path):
                if not required.is_file():
                    raise FileNotFoundError(required)

            observed_raw = _load_npz_data(raw_path, dtype=np.int16)
            variance_source = _variance_on_observed_grid(variance_path, observed_raw)
            uncertainty = _uncertainty_metrics(
                observed_raw,
                variance_source,
                high_variance_threshold,
            )

            reconstructed = _reconstructed_canvas(mean_path, canvas_path)
            iou = evaluator.occupied_iou(reconstructed, gt_occ, evaluation_mask)
            tu, tu_succeeded, tu_failed, planner = evaluator.topological_understanding(
                reconstructed,
                evaluation_mask,
                gt_occ,
                start,
                goals,
            )
            planner_used = planner

            coverage, known_fraction = _coverage_and_known_fraction(canvas_path, roi)
            distance = evaluator._distance_at_time(trajectory, decision_time)

            time_saved = (
                max(0.0, total_time - decision_time)
                if math.isfinite(total_time)
                else math.nan
            )
            distance_saved = (
                max(0.0, total_distance - distance)
                if math.isfinite(total_distance) and math.isfinite(distance)
                else math.nan
            )
            time_saved_fraction = (
                time_saved / total_time
                if math.isfinite(time_saved) and math.isfinite(total_time) and total_time > 0
                else math.nan
            )
            distance_saved_fraction = (
                distance_saved / total_distance
                if math.isfinite(distance_saved)
                and math.isfinite(total_distance)
                and total_distance > 0
                else math.nan
            )

            rows.append(
                {
                    "decision_id": decision_id,
                    "time_s": decision_time,
                    "distance_m": distance,
                    "coverage": coverage,
                    "known_fraction": known_fraction,
                    **uncertainty,
                    "high_variance_threshold": (
                        high_variance_threshold
                        if high_variance_threshold is not None
                        else math.nan
                    ),
                    "iou_if_stop": iou,
                    "tu_if_stop": tu,
                    "tu_succeeded": tu_succeeded,
                    "tu_failed": tu_failed,
                    "time_saved_s": time_saved,
                    "distance_saved_m": distance_saved,
                    "time_saved_fraction": time_saved_fraction,
                    "distance_saved_fraction": distance_saved_fraction,
                    "iou_loss_vs_final": math.nan,
                    "tu_loss_vs_final": math.nan,
                    "final_reference_decision_id": "",
                    "raw_map": raw_rel,
                    "mean_map": mean_rel,
                    "variance_map": variance_rel,
                }
            )
        except Exception as exc:
            skipped.append({"decision_id": decision_id_raw, "reason": str(exc)})

    if not rows:
        details = "; ".join(
            f"decision {item['decision_id']}: {item['reason']}" for item in skipped[:5]
        )
        raise RuntimeError(
            "No evaluable MapEx decisions. Ensure the run was recorded with "
            f"saved predictions. {details}"
        )

    rows.sort(key=lambda row: int(row["decision_id"]))
    final_reference = rows[-1]
    final_decision_id = int(final_reference["decision_id"])
    final_iou = float(final_reference["iou_if_stop"])
    final_tu = float(final_reference["tu_if_stop"])
    for row in rows:
        row["iou_loss_vs_final"] = final_iou - float(row["iou_if_stop"])
        row["tu_loss_vs_final"] = final_tu - float(row["tu_if_stop"])
        row["final_reference_decision_id"] = final_decision_id

    serializable_rows = []
    for row in rows:
        serializable_rows.append(
            {field: _fmt(row.get(field)) for field in OUTPUT_FIELDS}
        )

    output = (
        Path(output_path).expanduser().resolve()
        if output_path
        else run_dir / "early_stopping_analysis.csv"
    )
    _write_output(output, serializable_rows)

    return {
        "status": "ok",
        "run_dir": str(run_dir),
        "output": str(output),
        "environment": environment,
        "evaluated_decisions": len(rows),
        "skipped_decisions": len(skipped),
        "final_reference_decision_id": final_decision_id,
        "final_reference_occupied_iou": final_iou,
        "final_reference_tu": final_tu,
        "total_time_s": total_time,
        "total_distance_m": total_distance,
        "tu_goal_count": int(tu_goal_count),
        "tu_seed": int(tu_seed),
        "tu_planner": planner_used,
        "high_variance_threshold": high_variance_threshold,
        "ground_truth": str(gt_path),
        "roi": str(resolved_roi_path),
        "skipped": skipped,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze MapEx early-stopping trade-offs per policy decision."
    )
    parser.add_argument("run_dir", help="Recorded mapex_lab/experiments/mapex/<run_id> directory")
    parser.add_argument(
        "--output",
        default=None,
        help="Output CSV path (default: <run_dir>/early_stopping_analysis.csv)",
    )
    parser.add_argument("--ground-truth", default=None)
    parser.add_argument("--roi", default=None)
    parser.add_argument("--tu-goals", type=int, default=evaluator.TU_GOAL_COUNT)
    parser.add_argument("--tu-seed", type=int, default=evaluator.TU_RANDOM_SEED)
    parser.add_argument(
        "--high-variance-threshold",
        type=float,
        default=None,
        help=(
            "Optional diagnostic variance threshold. When omitted, "
            "high_uncertainty_fraction remains NaN; v1 does not assume a stopping threshold."
        ),
    )
    args = parser.parse_args()

    result = analyze_run(
        args.run_dir,
        output_path=args.output,
        ground_truth_path=args.ground_truth,
        roi_path=args.roi,
        tu_goal_count=args.tu_goals,
        tu_seed=args.tu_seed,
        high_variance_threshold=args.high_variance_threshold,
    )
    print(json.dumps(result, indent=2, allow_nan=True))


if __name__ == "__main__":
    main()
