#!/usr/bin/env python3
"""Offline occupied-IoU and TU evaluation for recorded Nearest-Frontier runs.

Paper-style NF evaluation uses the same MapEx whole-training LaMa predictor as
other methods. Run ``predict_alltrain_offline.py`` first; this evaluator then
compares those predictions against the same structural GT/ROI contract used by
the MapEx evaluator.

Observed SLAM canvases are still retained by the recorder for replay/audit, but
they are no longer used as the primary IoU/TU source here.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import evaluate_mapex_run as base


ROS_FLOAT_ABS_TOL = 1e-6


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_observed_canvas(path: Path) -> np.ndarray:
    """Legacy/audit helper for the recorded fixed-canvas SLAM map."""
    with np.load(path) as bundle:
        data = np.asarray(bundle["data"], dtype=np.int16)
        resolution = float(bundle["resolution"].reshape(-1)[0])
        origin_x = float(bundle["origin_x"].reshape(-1)[0])
        origin_y = float(bundle["origin_y"].reshape(-1)[0])

    if data.shape != (base.CANVAS_H, base.CANVAS_W):
        raise ValueError(
            f"observed canvas shape must be {(base.CANVAS_H, base.CANVAS_W)}, got {data.shape}"
        )
    if not math.isclose(resolution, base.CANVAS_RES, abs_tol=ROS_FLOAT_ABS_TOL):
        raise ValueError(
            f"observed canvas resolution must be {base.CANVAS_RES}, got {resolution}"
        )
    if not math.isclose(origin_x, base.CANVAS_X, abs_tol=ROS_FLOAT_ABS_TOL) or not math.isclose(
        origin_y, base.CANVAS_Y, abs_tol=ROS_FLOAT_ABS_TOL
    ):
        raise ValueError("observed canvas origin does not match canonical canvas")
    return data


def _occupied_iou_observed(
    observed: np.ndarray,
    gt_occ: np.ndarray,
    mask: np.ndarray,
) -> float:
    pred_occ = (observed > base.GT_OCC_THRESHOLD) & mask
    gt_occ = gt_occ & mask
    inter = np.count_nonzero(pred_occ & gt_occ)
    union = np.count_nonzero(pred_occ | gt_occ)
    return float(inter / union) if union else 0.0


def _topological_understanding_observed(
    observed: np.ndarray,
    mask: np.ndarray,
    gt_occ: np.ndarray,
    start: tuple[int, int],
    goals: np.ndarray,
):
    pred_free = mask & (observed >= 0) & (observed <= base.GT_OCC_THRESHOLD)
    collision_mask = gt_occ | ~mask
    try:
        import pyastar2d  # noqa: F401
    except Exception:
        planner = "four_neighbor_bfs_fallback"
        succeeded, failed = base._tu_bfs(pred_free, collision_mask, start, goals)
    else:
        planner = "pyastar2d_astar_allow_diagonal_false"
        succeeded, failed = base._tu_pyastar(pred_free, collision_mask, start, goals)
    total = succeeded + failed
    score = float(succeeded / total) if total else math.nan
    return score, succeeded, failed, planner


def _read_policy_decisions(path: Path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _attach_alltrain_predictions(run_dir: Path, decisions):
    """Attach paper-style all-training predictions to every NF decision."""
    manifest_path = run_dir / "evaluation" / "alltrain" / "manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError(
            f"missing {manifest_path}; run predict_alltrain_offline.py first"
        )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    decisions_path = run_dir / "policy_decisions.csv"
    expected_hash = _sha256(decisions_path)
    if manifest.get("decisions_sha256") != expected_hash:
        raise ValueError("alltrain manifest belongs to a different policy_decisions.csv")
    if manifest.get("decision_log") not in (None, "policy_decisions.csv"):
        raise ValueError("alltrain manifest was not generated from policy_decisions.csv")

    entries = manifest.get("decisions", [])
    indexed = {int(entry["decision_id"]): entry for entry in entries}
    if len(indexed) != len(entries) or len(indexed) != len(decisions):
        raise ValueError("incomplete or duplicate alltrain prediction decisions")

    attached = []
    for decision in decisions:
        decision_id = int(decision["policy_decision_id"])
        if decision_id not in indexed:
            raise ValueError(f"alltrain manifest lacks NF decision {decision_id}")
        entry = indexed[decision_id]
        raw_rel = (decision.get("raw_map") or "").strip()
        if entry.get("raw_map") != raw_rel:
            raise ValueError(f"alltrain raw-map association mismatch at decision {decision_id}")
        raw_path = run_dir / raw_rel
        if not raw_path.is_file():
            raise FileNotFoundError(raw_path)
        if _sha256(raw_path) != entry.get("raw_sha256"):
            raise ValueError(f"raw map changed since alltrain inference at decision {decision_id}")
        alltrain_rel = entry.get("alltrain_map", "")
        if not alltrain_rel or not (run_dir / alltrain_rel).is_file():
            raise ValueError(f"missing alltrain prediction at decision {decision_id}")
        attached.append(dict(decision, alltrain_map=alltrain_rel))
    return attached, manifest


def _write_csv(path: Path, rows) -> None:
    fields = [
        "decision_id",
        "time_s",
        "distance_m",
        "occupied_iou",
        "tu",
        "tu_succeeded",
        "tu_failed",
        "tu_total",
        "prediction_map",
        "prediction_source",
        "observed_map",
        "observed_occupied_iou",
        "observed_tu",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def evaluate_run(
    run_dir: str | Path,
    ground_truth_path: str | Path,
    roi_path: str | Path,
    goal_count: int = base.TU_GOAL_COUNT,
    seed: int = base.TU_RANDOM_SEED,
) -> dict:
    """Evaluate one recorded Nearest run using all-training LaMa predictions."""
    run_dir = Path(run_dir).expanduser().resolve()
    gt_path = Path(ground_truth_path).expanduser().resolve()
    roi_path = Path(roi_path).expanduser().resolve()

    if not run_dir.is_dir():
        raise FileNotFoundError(run_dir)
    if not gt_path.is_file():
        return base._skip(
            run_dir,
            "skipped_missing_ground_truth",
            "canonical structural ground truth is missing",
            expected_ground_truth=str(gt_path),
        )

    decisions_path = run_dir / "policy_decisions.csv"
    metadata_path = run_dir / "metadata.json"
    if not decisions_path.is_file():
        return base._skip(run_dir, "skipped_missing_decisions", "policy_decisions.csv is missing")
    if not metadata_path.is_file():
        return base._skip(run_dir, "skipped_missing_metadata", "metadata.json is missing")

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    start_x = metadata.get("evaluation_start_x")
    start_y = metadata.get("evaluation_start_y")
    if start_x is None or start_y is None:
        return base._skip(
            run_dir,
            "skipped_missing_start_pose",
            "metadata lacks evaluation_start_x/evaluation_start_y",
        )
    start = base._world_to_cell(float(start_x), float(start_y))

    try:
        mask, gt_occ, gt_free = base._load_ground_truth(gt_path)
    except Exception as exc:
        return base._skip(run_dir, "skipped_invalid_ground_truth", str(exc), ground_truth=str(gt_path))
    if not base._inside(gt_free.shape, start) or not gt_free[start]:
        return base._skip(
            run_dir,
            "skipped_invalid_start_pose",
            f"start cell {start} is not GT free",
        )

    if roi_path.is_file():
        roi = np.load(roi_path).astype(bool)
        if roi.shape != gt_free.shape:
            return base._skip(
                run_dir,
                "skipped_invalid_roi",
                f"ROI shape {roi.shape} != GT {gt_free.shape}",
            )
        valid_goals = roi & gt_free
        goal_source = str(roi_path)
    else:
        valid_goals = base._connected_component(gt_free, start)
        goal_source = "connected_ground_truth_free_component"

    try:
        goals = base._sample_goals(valid_goals, int(goal_count), int(seed))
    except Exception as exc:
        return base._skip(run_dir, "skipped_goal_sampling_failed", str(exc))

    evaluation_dir = run_dir / "evaluation"
    evaluation_dir.mkdir(exist_ok=True)
    np.save(evaluation_dir / "tu_goals.npy", goals)

    decisions = _read_policy_decisions(decisions_path)
    try:
        decisions, manifest = _attach_alltrain_predictions(run_dir, decisions)
    except Exception as exc:
        return base._skip(
            run_dir,
            "skipped_missing_or_invalid_alltrain_predictions",
            str(exc),
        )

    trajectory = (
        base._read_csv(run_dir / "trajectory.csv")
        if (run_dir / "trajectory.csv").is_file()
        else []
    )

    output = []
    errors = []
    planner_used = None
    observed_planner_used = None
    for decision in decisions:
        decision_id = int(decision["policy_decision_id"])
        alltrain_rel = decision["alltrain_map"]
        observed_rel = (decision.get("canvas_map") or "").strip()
        try:
            prediction = base._prediction_to_canvas(run_dir / alltrain_rel)
            iou = base.occupied_iou(prediction, gt_occ, mask)
            tu, succeeded, failed, planner = base.topological_understanding(
                prediction, mask, gt_occ, start, goals
            )
            planner_used = planner

            observed_iou = ""
            observed_tu = ""
            if observed_rel and (run_dir / observed_rel).is_file():
                observed = _load_observed_canvas(run_dir / observed_rel)
                observed_iou = _occupied_iou_observed(observed, gt_occ, mask)
                observed_tu_value, _, _, observed_planner = _topological_understanding_observed(
                    observed, mask, gt_occ, start, goals
                )
                observed_tu = observed_tu_value
                observed_planner_used = observed_planner

            decision_time = float(decision["sim_time_s"])
            output.append(
                {
                    "decision_id": decision_id,
                    "time_s": f"{decision_time:.9f}",
                    "distance_m": f"{base._distance_at_time(trajectory, decision_time):.9f}",
                    "occupied_iou": f"{iou:.9f}",
                    "tu": f"{tu:.9f}",
                    "tu_succeeded": succeeded,
                    "tu_failed": failed,
                    "tu_total": succeeded + failed,
                    "prediction_map": alltrain_rel,
                    "prediction_source": "alltrain",
                    "observed_map": observed_rel,
                    "observed_occupied_iou": "" if observed_iou == "" else f"{float(observed_iou):.9f}",
                    "observed_tu": "" if observed_tu == "" else f"{float(observed_tu):.9f}",
                }
            )
        except Exception as exc:
            errors.append({"decision_id": decision_id, "error": str(exc)})

    if not output:
        return base._skip(
            run_dir,
            "skipped_no_evaluable_alltrain_predictions",
            "no saved all-training prediction could be evaluated",
            ground_truth=str(gt_path),
            errors=errors,
        )

    _write_csv(run_dir / "evaluation.csv", output)
    backfilled = base._backfill_metrics(run_dir / "metrics.csv", output)
    last = output[-1]
    payload = {
        "status": "ok",
        "source": "nearest_alltrain_prediction",
        "prediction_source": "alltrain",
        "prediction_manifest": str(run_dir / "evaluation" / "alltrain" / "manifest.json"),
        "prediction_checkpoint": manifest.get("checkpoint"),
        "ground_truth": str(gt_path),
        "ground_truth_sha256": base._sha256(gt_path),
        "evaluation_canvas": {
            "id": "hospital_canvas_v1",
            "resolution_m": base.CANVAS_RES,
            "width": base.CANVAS_W,
            "height": base.CANVAS_H,
            "origin_x": base.CANVAS_X,
            "origin_y": base.CANVAS_Y,
        },
        "occupied_iou": {
            "class": "occupied",
            "prediction_threshold": base.PRED_OCC_THRESHOLD,
            "unknown_unrepresented_prediction": "free",
            "domain": "structural_ground_truth_evaluation_mask",
        },
        "tu": {
            "goal_count": int(goal_count),
            "seed": int(seed),
            "goal_source": goal_source,
            "start_world_xy": [float(start_x), float(start_y)],
            "start_canvas_row_col": [int(start[0]), int(start[1])],
            "planner": planner_used,
            "connectivity": 4,
            "traversable": "predicted_free",
            "success": "predicted-map path exists and does not intersect GT occupied cells",
        },
        "observed_map_diagnostics": {
            "retained": True,
            "planner": observed_planner_used,
            "columns": ["observed_occupied_iou", "observed_tu"],
        },
        "evaluated_decisions": len(output),
        "metrics_csv_backfilled_step_hold": backfilled,
        "final_decision_id": int(last["decision_id"]),
        "final_occupied_iou": float(last["occupied_iou"]),
        "final_tu": float(last["tu"]),
        "occupied_iou_auc_time": base._auc(output, "time_s", "occupied_iou"),
        "occupied_iou_auc_distance": base._auc(output, "distance_m", "occupied_iou"),
        "tu_auc_time": base._auc(output, "time_s", "tu"),
        "tu_auc_distance": base._auc(output, "distance_m", "tu"),
        "errors": errors,
    }
    base._write_eval_json(run_dir, payload)
    base._update_summary(
        run_dir,
        {
            "offline_evaluation_status": "ok",
            "offline_evaluation_reason": None,
            "offline_evaluation_source": "nearest_alltrain_prediction",
            "final_occupied_iou": payload["final_occupied_iou"],
            "final_tu": payload["final_tu"],
            "occupied_iou_auc_time": payload["occupied_iou_auc_time"],
            "occupied_iou_auc_distance": payload["occupied_iou_auc_distance"],
            "tu_auc_time": payload["tu_auc_time"],
            "tu_auc_distance": payload["tu_auc_distance"],
            "offline_metric_last_decision_id": payload["final_decision_id"],
            "tu_goal_count": int(goal_count),
            "tu_random_seed": int(seed),
            "tu_planner": planner_used,
            "prediction_source": "alltrain",
        },
    )
    return payload


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--ground-truth", required=True)
    parser.add_argument("--roi", required=True)
    parser.add_argument("--tu-goals", type=int, default=base.TU_GOAL_COUNT)
    parser.add_argument("--tu-seed", type=int, default=base.TU_RANDOM_SEED)
    args = parser.parse_args()
    result = evaluate_run(
        args.run_dir,
        args.ground_truth,
        args.roi,
        args.tu_goals,
        args.tu_seed,
    )
    print(json.dumps(result, indent=2, allow_nan=True))


if __name__ == "__main__":
    main()
