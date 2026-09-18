#!/usr/bin/env python3
"""Offline occupied-IoU and Topological Understanding evaluation for one MapEx run.

The evaluator is kept out of the online ROS loop. ``mapex_run.py`` invokes
``evaluate_run`` only after recorder files are closed, so TU computation does
not affect benchmark timing.

Canonical Hospital structural-GT contract:
- default file: ground_truth/hospital/generated/hospital_structural_gt_v1.npz
- data: HxW occupancy labels; <0 outside/unscored, 0 free, >50 occupied
- optional evaluation_mask: explicit building-footprint mask
- resolution/origin/shape must match hospital_canvas_v1

MapEx-style metrics:
- occupied IoU thresholds the all-training prediction at > 0.5 (legacy CSVs: ensemble mean);
- TU uses 100 deterministic free-space goals, 4-connected predicted-map paths,
  and succeeds only when a path exists and does not intersect GT occupied cells.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
from collections import deque
from pathlib import Path

import numpy as np

CANVAS_RES = 0.05
CANVAS_W = 1504
CANVAS_H = 2123
CANVAS_X = -25.6
CANVAS_Y = -60.1
PRED_OCC_THRESHOLD = 0.5
GT_OCC_THRESHOLD = 50
TU_GOAL_COUNT = 100
TU_RANDOM_SEED = 2025
DEFAULT_GT_RELATIVE = Path(
    "ground_truth/hospital/generated/hospital_structural_gt_v1.npz"
)
DEFAULT_ROI_RELATIVE = Path(
    "ground_truth/hospital/generated/hospital_connected_free_v1.npy"
)


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _scalar(npz, key: str, default=None):
    if key not in npz.files:
        return default
    value = np.asarray(npz[key])
    if value.size != 1:
        raise ValueError(f"metadata '{key}' must be scalar")
    return value.reshape(-1)[0].item()


def _load_ground_truth(path: Path):
    explicit_mask = None
    if path.suffix.lower() == ".npy":
        data = np.load(path)
        resolution, origin_x, origin_y = CANVAS_RES, CANVAS_X, CANVAS_Y
    elif path.suffix.lower() == ".npz":
        with np.load(path) as bundle:
            if "data" in bundle.files:
                data = np.asarray(bundle["data"])
            elif "occupancy" in bundle.files:
                data = np.asarray(bundle["occupancy"])
            else:
                raise ValueError("ground truth needs 'data' or 'occupancy'")
            if "evaluation_mask" in bundle.files:
                explicit_mask = np.asarray(bundle["evaluation_mask"]).astype(bool)
            resolution = float(_scalar(bundle, "resolution", CANVAS_RES))
            origin_x = float(_scalar(bundle, "origin_x", CANVAS_X))
            origin_y = float(_scalar(bundle, "origin_y", CANVAS_Y))
    else:
        raise ValueError(f"unsupported GT format: {path.suffix}")

    if data.shape != (CANVAS_H, CANVAS_W):
        raise ValueError(
            f"GT shape must be {(CANVAS_H, CANVAS_W)}, got {data.shape}"
        )
    if not math.isclose(resolution, CANVAS_RES, abs_tol=1e-9):
        raise ValueError(
            f"GT resolution must be {CANVAS_RES}, got {resolution}"
        )
    if not math.isclose(origin_x, CANVAS_X, abs_tol=1e-9) or not math.isclose(
        origin_y, CANVAS_Y, abs_tol=1e-9
    ):
        raise ValueError("GT origin does not match hospital_canvas_v1")

    if explicit_mask is not None:
        if explicit_mask.shape != data.shape:
            raise ValueError("evaluation_mask shape mismatch")
        evaluation_mask = explicit_mask
    elif np.issubdtype(data.dtype, np.signedinteger) or np.issubdtype(
        data.dtype, np.floating
    ):
        evaluation_mask = data >= 0
    else:
        evaluation_mask = np.ones(data.shape, dtype=bool)

    occupied = (data > GT_OCC_THRESHOLD) & evaluation_mask
    free = evaluation_mask & ~occupied
    if not np.any(evaluation_mask) or not np.any(free):
        raise ValueError("GT evaluation mask/free space is empty")
    return evaluation_mask, occupied, free


def _prediction_to_canvas(path: Path) -> np.ndarray:
    with np.load(path) as bundle:
        mean = np.asarray(bundle["data"], dtype=np.float32)
        source_h = int(_scalar(bundle, "source_height"))
        source_w = int(_scalar(bundle, "source_width"))
        pad_top = int(_scalar(bundle, "pad_top", 0))
        pad_left = int(_scalar(bundle, "pad_left", 0))
        resolution = float(_scalar(bundle, "resolution"))
        origin_x = _scalar(bundle, "origin_x")
        origin_y = _scalar(bundle, "origin_y")

    if origin_x is None or origin_y is None:
        raise ValueError(
            "prediction lacks origin_x/origin_y; rerun with updated mapex_run.py"
        )
    r1, c1 = pad_top + source_h, pad_left + source_w
    if (
        mean.ndim != 2
        or pad_top < 0
        or pad_left < 0
        or r1 > mean.shape[0]
        or c1 > mean.shape[1]
    ):
        raise ValueError("invalid prediction array/padding metadata")
    source = mean[pad_top:r1, pad_left:c1]

    ratio_f = resolution / CANVAS_RES
    ratio = int(round(ratio_f))
    if ratio < 1 or not math.isclose(ratio_f, ratio, abs_tol=1e-6):
        raise ValueError(
            "prediction resolution is incompatible with canonical canvas"
        )
    if ratio > 1:
        source = np.repeat(np.repeat(source, ratio, axis=0), ratio, axis=1)

    # Official MapEx IoU treats unknown/unrepresented prediction as free.
    canvas = np.zeros((CANVAS_H, CANVAS_W), dtype=np.float32)
    c0 = int(round((float(origin_x) - CANVAS_X) / CANVAS_RES))
    r0 = int(round((float(origin_y) - CANVAS_Y) / CANVAS_RES))
    sr, sc = max(0, -r0), max(0, -c0)
    dr, dc = max(0, r0), max(0, c0)
    nr = min(source.shape[0] - sr, CANVAS_H - dr)
    nc = min(source.shape[1] - sc, CANVAS_W - dc)
    if nr > 0 and nc > 0:
        canvas[dr : dr + nr, dc : dc + nc] = source[
            sr : sr + nr, sc : sc + nc
        ]
    return canvas


def occupied_iou(
    prediction: np.ndarray,
    gt_occ: np.ndarray,
    mask: np.ndarray,
) -> float:
    pred_occ = (prediction > PRED_OCC_THRESHOLD) & mask
    gt_occ = gt_occ & mask
    inter = np.count_nonzero(pred_occ & gt_occ)
    union = np.count_nonzero(pred_occ | gt_occ)
    return float(inter / union) if union else 0.0


def _world_to_cell(x: float, y: float) -> tuple[int, int]:
    row = int(round((y - CANVAS_Y) / CANVAS_RES))
    col = int(round((x - CANVAS_X) / CANVAS_RES))
    return row, col


def _inside(shape, cell) -> bool:
    row, col = cell
    return 0 <= row < shape[0] and 0 <= col < shape[1]


def _connected_component(
    mask: np.ndarray,
    start: tuple[int, int],
) -> np.ndarray:
    out = np.zeros(mask.shape, dtype=bool)
    if not _inside(mask.shape, start) or not mask[start]:
        return out
    h, w = mask.shape
    queue = deque([start])
    out[start] = True
    while queue:
        row, col = queue.popleft()
        for nr, nc in (
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ):
            if (
                0 <= nr < h
                and 0 <= nc < w
                and mask[nr, nc]
                and not out[nr, nc]
            ):
                out[nr, nc] = True
                queue.append((nr, nc))
    return out


def _sample_goals(valid: np.ndarray, count: int, seed: int) -> np.ndarray:
    flat = np.flatnonzero(valid)
    if flat.size < count:
        raise ValueError(
            f"TU needs {count} goals, only {flat.size} valid cells exist"
        )
    rng = np.random.default_rng(seed)
    chosen = rng.choice(flat, size=count, replace=False)
    rows, cols = np.unravel_index(chosen, valid.shape)
    return np.column_stack((rows, cols)).astype(np.int32)


def _tu_pyastar(pred_free, collision_mask, start, goals):
    import pyastar2d  # type: ignore

    costs = np.where(pred_free, 1.0, np.inf).astype(np.float32)
    succeeded = 0
    for goal_array in goals:
        goal = (int(goal_array[0]), int(goal_array[1]))
        if not pred_free[start] or not pred_free[goal]:
            continue
        try:
            path = pyastar2d.astar_path(
                costs,
                np.asarray(start, dtype=np.int32),
                np.asarray(goal, dtype=np.int32),
                allow_diagonal=False,
            )
        except Exception:
            path = None
        if (
            path is not None
            and len(path)
            and not np.any(collision_mask[path[:, 0], path[:, 1]])
        ):
            succeeded += 1
    return succeeded, len(goals) - succeeded


def _bfs_parent(
    pred_free: np.ndarray,
    start: tuple[int, int],
) -> np.ndarray:
    h, w = pred_free.shape
    parent = np.full(h * w, -1, dtype=np.int32)
    if not _inside(pred_free.shape, start) or not pred_free[start]:
        return parent
    start_flat = start[0] * w + start[1]
    parent[start_flat] = start_flat
    queue = deque([start_flat])
    while queue:
        current = queue.popleft()
        row, col = divmod(current, w)
        if row > 0:
            nxt = current - w
            if pred_free[row - 1, col] and parent[nxt] == -1:
                parent[nxt] = current
                queue.append(nxt)
        if row + 1 < h:
            nxt = current + w
            if pred_free[row + 1, col] and parent[nxt] == -1:
                parent[nxt] = current
                queue.append(nxt)
        if col > 0:
            nxt = current - 1
            if pred_free[row, col - 1] and parent[nxt] == -1:
                parent[nxt] = current
                queue.append(nxt)
        if col + 1 < w:
            nxt = current + 1
            if pred_free[row, col + 1] and parent[nxt] == -1:
                parent[nxt] = current
                queue.append(nxt)
    return parent


def _tu_bfs(pred_free, collision_mask, start, goals):
    _h, w = pred_free.shape
    parent = _bfs_parent(pred_free, start)
    start_flat = start[0] * w + start[1]
    succeeded = 0
    for goal in goals:
        current = int(goal[0]) * w + int(goal[1])
        if current < 0 or current >= parent.size or parent[current] == -1:
            continue
        valid = True
        while True:
            row, col = divmod(current, w)
            if collision_mask[row, col]:
                valid = False
                break
            if current == start_flat:
                break
            nxt = int(parent[current])
            if nxt < 0 or nxt == current:
                valid = False
                break
            current = nxt
        if valid:
            succeeded += 1
    return succeeded, len(goals) - succeeded


def topological_understanding(
    prediction,
    mask,
    gt_occ,
    start,
    goals,
):
    pred_free = mask & ~(prediction > PRED_OCC_THRESHOLD)
    collision_mask = gt_occ | ~mask
    try:
        import pyastar2d  # noqa: F401
    except Exception:
        planner = "four_neighbor_bfs_fallback"
        succeeded, failed = _tu_bfs(
            pred_free, collision_mask, start, goals
        )
    else:
        planner = "pyastar2d_astar_allow_diagonal_false"
        succeeded, failed = _tu_pyastar(
            pred_free, collision_mask, start, goals
        )
    total = succeeded + failed
    score = float(succeeded / total) if total else math.nan
    return score, succeeded, failed, planner


def _read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _distance_at_time(trajectory, time_s):
    points = []
    for row in trajectory:
        try:
            points.append(
                (
                    float(row["time_s"]),
                    float(row["cumulative_distance_m"]),
                )
            )
        except (KeyError, TypeError, ValueError):
            pass
    if not points:
        return math.nan
    points.sort()
    times = [point[0] for point in points]
    index = max(0, bisect.bisect_right(times, time_s) - 1)
    return points[index][1]


def _auc(rows, x_key, y_key):
    points = []
    for row in rows:
        try:
            x, y = float(row[x_key]), float(row[y_key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    if len(points) < 2:
        return math.nan
    points.sort()
    dedup = {}
    for x, y in points:
        dedup[x] = y
    xs = np.asarray(sorted(dedup), dtype=np.float64)
    ys = np.asarray([dedup[x] for x in xs], dtype=np.float64)
    return float(np.trapz(ys, xs)) if len(xs) >= 2 else math.nan


def _write_csv(path: Path, rows):
    fields = [
        "decision_id",
        "time_s",
        "distance_m",
        "occupied_iou",
        "tu",
        "tu_succeeded",
        "tu_failed",
        "tu_total",
        "mean_map",
        "prediction_map",
        "prediction_source",
        "ensemble_mean_occupied_iou",
        "ensemble_mean_tu",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _backfill_metrics(path: Path, evaluations) -> bool:
    if not path.is_file() or not evaluations:
        return False
    rows = _read_csv(path)
    if not rows:
        return False
    evaluations = sorted(
        evaluations,
        key=lambda row: float(row["time_s"]),
    )
    times = [float(row["time_s"]) for row in evaluations]
    for row in rows:
        try:
            current_time = float(row["time_s"])
        except (KeyError, TypeError, ValueError):
            continue
        index = bisect.bisect_right(times, current_time) - 1
        if index >= 0:
            row["occupied_iou"] = (
                f"{float(evaluations[index]['occupied_iou']):.9f}"
            )
            row["tu"] = f"{float(evaluations[index]['tu']):.9f}"
    tmp = path.with_suffix(".csv.tmp")
    with tmp.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    tmp.replace(path)
    return True


def _update_summary(run_dir: Path, updates: dict):
    path = run_dir / "summary.json"
    payload = (
        json.loads(path.read_text(encoding="utf-8"))
        if path.is_file()
        else {}
    )
    payload.update(updates)
    path.write_text(
        json.dumps(payload, indent=2, allow_nan=True),
        encoding="utf-8",
    )


def _write_eval_json(run_dir: Path, payload: dict):
    (run_dir / "evaluation.json").write_text(
        json.dumps(payload, indent=2, allow_nan=True),
        encoding="utf-8",
    )


def _skip(run_dir: Path, status: str, reason: str, **extra):
    payload = {"status": status, "reason": reason, **extra}
    _write_eval_json(run_dir, payload)
    _update_summary(
        run_dir,
        {
            "offline_evaluation_status": status,
            "offline_evaluation_reason": reason,
            "final_occupied_iou": None,
            "final_tu": None,
        },
    )
    return payload


def attach_offline_predictions(run_dir, decisions):
    """Require a complete manifest tied to this exact decisions.csv."""
    manifest_path = run_dir / "evaluation" / "alltrain" / "manifest.json"
    if not manifest_path.is_file():
        return decisions
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("decisions_sha256") != _sha256(run_dir / "decisions.csv"):
        raise ValueError("Offline alltrain manifest belongs to different decisions.csv")
    entries = manifest["decisions"]
    indexed = {int(entry["decision_id"]): entry for entry in entries}
    if len(indexed) != len(entries) or len(indexed) != len(decisions):
        raise ValueError("Incomplete or duplicate offline prediction decisions")
    attached = []
    for decision in decisions:
        entry = indexed[int(decision["decision_id"])]
        if entry["raw_map"] != decision.get("raw_map"):
            raise ValueError("Offline raw-map association mismatch")
        if _sha256(run_dir / entry["raw_map"]) != entry["raw_sha256"]:
            raise ValueError("Raw map changed since offline inference")
        if not (run_dir / entry["alltrain_map"]).is_file():
            raise ValueError("Missing offline alltrain prediction")
        attached.append(dict(decision, alltrain_map=entry["alltrain_map"]))
    return attached


def evaluate_run(
    run_dir: str | Path,
    ground_truth_path: str | Path | None = None,
    goal_count: int = TU_GOAL_COUNT,
    seed: int = TU_RANDOM_SEED,
) -> dict:
    """Evaluate one recorded MapEx run and backfill IoU/TU into metrics.csv."""
    run_dir = Path(run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise FileNotFoundError(run_dir)
    mapex_lab = Path(__file__).resolve().parents[1]
    gt_path = (
        Path(ground_truth_path).expanduser().resolve()
        if ground_truth_path
        else mapex_lab / DEFAULT_GT_RELATIVE
    )
    if not gt_path.is_file():
        return _skip(
            run_dir,
            "skipped_missing_ground_truth",
            "canonical structural ground truth is missing",
            expected_ground_truth=str(gt_path),
        )

    decisions_path = run_dir / "decisions.csv"
    metadata_path = run_dir / "metadata.json"
    if not decisions_path.is_file():
        return _skip(
            run_dir,
            "skipped_missing_decisions",
            "decisions.csv is missing",
        )
    if not metadata_path.is_file():
        return _skip(
            run_dir,
            "skipped_missing_metadata",
            "metadata.json is missing",
        )
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    start_x = metadata.get("evaluation_start_x")
    start_y = metadata.get("evaluation_start_y")
    if start_x is None or start_y is None:
        return _skip(
            run_dir,
            "skipped_missing_start_pose",
            "metadata lacks evaluation_start_x/evaluation_start_y",
        )
    start = _world_to_cell(float(start_x), float(start_y))

    try:
        mask, gt_occ, gt_free = _load_ground_truth(gt_path)
    except Exception as exc:
        return _skip(
            run_dir,
            "skipped_invalid_ground_truth",
            str(exc),
            ground_truth=str(gt_path),
        )
    if not _inside(gt_free.shape, start) or not gt_free[start]:
        return _skip(
            run_dir,
            "skipped_invalid_start_pose",
            f"start cell {start} is not GT free",
        )

    roi_path = mapex_lab / DEFAULT_ROI_RELATIVE
    if roi_path.is_file():
        roi = np.load(roi_path).astype(bool)
        if roi.shape != gt_free.shape:
            return _skip(
                run_dir,
                "skipped_invalid_roi",
                f"ROI shape {roi.shape} != GT {gt_free.shape}",
            )
        valid_goals = roi & gt_free
        goal_source = str(roi_path)
    else:
        valid_goals = _connected_component(gt_free, start)
        goal_source = "connected_ground_truth_free_component"
    try:
        goals = _sample_goals(valid_goals, int(goal_count), int(seed))
    except Exception as exc:
        return _skip(
            run_dir,
            "skipped_goal_sampling_failed",
            str(exc),
        )

    evaluation_dir = run_dir / "evaluation"
    evaluation_dir.mkdir(exist_ok=True)
    np.save(evaluation_dir / "tu_goals.npy", goals)
    decisions = attach_offline_predictions(run_dir, _read_csv(decisions_path))
    trajectory = (
        _read_csv(run_dir / "trajectory.csv")
        if (run_dir / "trajectory.csv").is_file()
        else []
    )

    prediction_field = "alltrain_map" if decisions and "alltrain_map" in decisions[0] else "mean_map"
    prediction_source = "alltrain" if prediction_field == "alltrain_map" else prediction_field
    output = []
    errors = []
    planner_used = None
    for decision in decisions:
        mean_rel = (decision.get(prediction_field) or "").strip()
        if not mean_rel:
            continue
        prediction_path = run_dir / mean_rel
        if not prediction_path.is_file():
            errors.append(
                {
                    "decision_id": decision.get("decision_id"),
                    "error": f"missing prediction {prediction_path}",
                }
            )
            continue
        try:
            canvas = _prediction_to_canvas(prediction_path)
            iou = occupied_iou(canvas, gt_occ, mask)
            tu, succeeded, failed, planner = topological_understanding(
                canvas,
                mask,
                gt_occ,
                start,
                goals,
            )
            secondary_iou, secondary_tu = "", ""
            if prediction_field == "alltrain_map" and decision.get("mean_map"):
                try:
                    mean_canvas = _prediction_to_canvas(run_dir / decision["mean_map"])
                    secondary_iou = occupied_iou(mean_canvas, gt_occ, mask)
                    secondary_tu = topological_understanding(
                        mean_canvas, mask, gt_occ, start, goals)[0]
                except Exception as exc:
                    errors.append({"decision_id": decision.get("decision_id"),
                                   "error": "secondary ensemble mean: " + str(exc)})
            planner_used = planner
            decision_time = float(decision["time_s"])
            output.append(
                {
                    "decision_id": int(decision["decision_id"]),
                    "time_s": f"{decision_time:.9f}",
                    "distance_m": (
                        f"{_distance_at_time(trajectory, decision_time):.9f}"
                    ),
                    "occupied_iou": f"{iou:.9f}",
                    "tu": f"{tu:.9f}",
                    "tu_succeeded": succeeded,
                    "tu_failed": failed,
                    "tu_total": succeeded + failed,
                    "mean_map": decision.get("mean_map", ""),
                    "prediction_map": mean_rel,
                    "prediction_source": prediction_source,
                    "ensemble_mean_occupied_iou": secondary_iou,
                    "ensemble_mean_tu": secondary_tu,
                }
            )
        except Exception as exc:
            errors.append(
                {
                    "decision_id": decision.get("decision_id"),
                    "error": str(exc),
                }
            )

    if not output:
        return _skip(
            run_dir,
            "skipped_no_evaluable_predictions",
            "no saved mean prediction could be evaluated",
            ground_truth=str(gt_path),
            errors=errors,
        )

    _write_csv(run_dir / "evaluation.csv", output)
    backfilled = _backfill_metrics(run_dir / "metrics.csv", output)
    last = output[-1]
    payload = {
        "status": "ok",
        "prediction_source": prediction_source,
        "ground_truth": str(gt_path),
        "ground_truth_sha256": _sha256(gt_path),
        "evaluation_canvas": {
            "id": "hospital_canvas_v1",
            "resolution_m": CANVAS_RES,
            "width": CANVAS_W,
            "height": CANVAS_H,
            "origin_x": CANVAS_X,
            "origin_y": CANVAS_Y,
        },
        "occupied_iou": {
            "class": "occupied",
            "prediction_threshold": PRED_OCC_THRESHOLD,
            "domain": "structural_ground_truth_evaluation_mask",
            "unrepresented_prediction_cells": "treated_as_free",
        },
        "tu": {
            "goal_count": int(goal_count),
            "seed": int(seed),
            "goal_source": goal_source,
            "start_world_xy": [float(start_x), float(start_y)],
            "start_canvas_row_col": [int(start[0]), int(start[1])],
            "planner": planner_used,
            "connectivity": 4,
            "success": (
                "predicted path exists and does not intersect GT occupied cells"
            ),
        },
        "evaluated_decisions": len(output),
        "metrics_csv_backfilled_step_hold": backfilled,
        "final_decision_id": int(last["decision_id"]),
        "final_occupied_iou": float(last["occupied_iou"]),
        "final_tu": float(last["tu"]),
        "occupied_iou_auc_time": _auc(
            output, "time_s", "occupied_iou"
        ),
        "occupied_iou_auc_distance": _auc(
            output, "distance_m", "occupied_iou"
        ),
        "tu_auc_time": _auc(output, "time_s", "tu"),
        "tu_auc_distance": _auc(output, "distance_m", "tu"),
        "errors": errors,
    }
    _write_eval_json(run_dir, payload)
    _update_summary(
        run_dir,
        {
            "offline_evaluation_status": "ok",
            "offline_evaluation_reason": None,
            "final_occupied_iou": payload["final_occupied_iou"],
            "final_tu": payload["final_tu"],
            "occupied_iou_auc_time": payload["occupied_iou_auc_time"],
            "occupied_iou_auc_distance": payload[
                "occupied_iou_auc_distance"
            ],
            "tu_auc_time": payload["tu_auc_time"],
            "tu_auc_distance": payload["tu_auc_distance"],
            "offline_metric_last_decision_id": payload[
                "final_decision_id"
            ],
            "tu_goal_count": int(goal_count),
            "tu_random_seed": int(seed),
            "tu_planner": planner_used,
            "prediction_source": prediction_source,
        },
    )
    return payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir")
    parser.add_argument("--ground-truth", default=None)
    parser.add_argument("--roi", default=None, help="Connected-free ROI for the active environment")
    parser.add_argument("--tu-goals", type=int, default=TU_GOAL_COUNT)
    parser.add_argument("--tu-seed", type=int, default=TU_RANDOM_SEED)
    args = parser.parse_args()
    global DEFAULT_ROI_RELATIVE
    if args.roi:
        DEFAULT_ROI_RELATIVE = Path(args.roi).expanduser().resolve()
    result = evaluate_run(
        args.run_dir,
        args.ground_truth,
        args.tu_goals,
        args.tu_seed,
    )
    print(json.dumps(result, indent=2, allow_nan=True))


if __name__ == "__main__":
    main()
