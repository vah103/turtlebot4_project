#!/usr/bin/env python3
"""Evaluate R003 common paper500 samples with fixed masks/goals."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from predict_alltrain_offline import sha256
from r003_paper500 import (
    EVAL_ID,
    MAX_STEPS,
    coverage,
    occupied_iou,
    project_runtime_observed,
    project_runtime_prediction,
    reduce_observed,
    reduce_prediction,
    select_final_sample,
    topological_understanding,
)


COMMON_STEP_SPACING = 10


def _rows(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _load_raw(path: Path):
    with np.load(path, allow_pickle=False) as bundle:
        return {
            "data": np.asarray(bundle["data"]),
            "resolution": float(bundle["resolution"]),
            "origin_x": float(bundle["origin_x"]),
            "origin_y": float(bundle["origin_y"]),
            "origin_yaw": float(bundle["origin_yaw"]) if "origin_yaw" in bundle else 0.0,
        }


def _terminal_kind(event: str) -> str:
    mapping = {
        "budget_cutoff": "budget_cutoff",
        "natural_completion": "natural_completion",
        "abnormal_final": "algorithmic_failure",
    }
    try:
        return mapping[event]
    except KeyError as exc:
        raise ValueError(f"unsupported authoritative final event: {event}") from exc


def _dedupe_curve_steps(rows: list[dict]) -> list[dict]:
    """Use the last recorded sample at each adapted step, deterministically."""
    by_step: dict[int, dict] = {}
    for row in rows:
        by_step[int(row["progress_step"])] = dict(row)
    return [by_step[step] for step in sorted(by_step)]


def build_curve_support(
    evaluated_rows: list[dict],
    final_row: dict,
) -> tuple[list[dict], list[dict], list[dict]]:
    """Build canonical raw/held curve support through the authoritative final.

    Post-cancellation audit samples are never part of the metric curve. When a
    run legitimately completes early, the final metric is explicitly held at
    subsequent common k=10 support through k=500. Failed runs never hold.
    """
    final_id = int(final_row["sample_id"])
    terminal_prefix: list[dict] = []
    found = False
    for row in evaluated_rows:
        terminal_prefix.append(dict(row))
        if int(row["sample_id"]) == final_id:
            found = True
            break
    if not found:
        raise ValueError(f"authoritative final sample {final_id} was not evaluated")

    raw_support = _dedupe_curve_steps(terminal_prefix)
    if not raw_support:
        raise ValueError("curve support is empty")

    final_step = int(final_row["progress_step"])
    if final_step < 0 or final_step > MAX_STEPS:
        raise ValueError(f"invalid final adapted step: {final_step}")

    held_rows: list[dict] = []
    if final_row["event"] == "natural_completion" and final_step < MAX_STEPS:
        first_hold = ((final_step // COMMON_STEP_SPACING) + 1) * COMMON_STEP_SPACING
        for step in range(first_hold, MAX_STEPS + 1, COMMON_STEP_SPACING):
            held = dict(final_row)
            held["sample_id"] = f"hold_from_{final_id}_k{step:04d}"
            held["event"] = "natural_completion_hold"
            held["progress_step"] = step
            held["held"] = True
            held["held_from_sample_id"] = final_id
            held_rows.append(held)

    curve_rows = raw_support + held_rows
    steps = [int(row["progress_step"]) for row in curve_rows]
    if any(b <= a for a, b in zip(steps, steps[1:])):
        raise AssertionError("curve support must be strictly increasing after deduplication")
    return raw_support, held_rows, curve_rows


def trapezoidal_auc(rows: list[dict], metric: str) -> dict:
    """Deterministic trapezoidal area on the adapted-step axis."""
    if not rows:
        raise ValueError("cannot integrate empty curve")
    x = [int(row["progress_step"]) for row in rows]
    y = [float(row[metric]) for row in rows]
    if not all(math.isfinite(value) for value in y):
        raise ValueError(f"nonfinite {metric} value in curve")
    if any(b <= a for a, b in zip(x, x[1:])):
        raise ValueError("integration support must be strictly increasing")

    area = 0.0
    for x0, x1, y0, y1 in zip(x, x[1:], y, y[1:]):
        area += 0.5 * (y0 + y1) * (x1 - x0)
    return {
        "area": float(area),
        "axis": "adapted_step",
        "units": "metric_x_adapted_step",
        "rule": "deterministic_trapezoidal",
        "start_step": int(x[0]),
        "end_step": int(x[-1]),
        "point_count": len(x),
    }


def _endpoint_at_500(curve_rows: list[dict]) -> dict | None:
    if not curve_rows or int(curve_rows[-1]["progress_step"]) != MAX_STEPS:
        return None
    row = curve_rows[-1]
    return {
        "progress_step": MAX_STEPS,
        "coverage": float(row["coverage"]),
        "occupied_iou": float(row["occupied_iou"]),
        "tu": float(row["tu"]),
        "held": bool(row["held"]),
        "source_sample_id": row["held_from_sample_id"] or row["sample_id"],
    }


def evaluate(run_dir: Path, profile_path: Path, goals_path: Path) -> dict:
    samples_path = run_dir / "paper500_snapshots.csv"
    prediction_manifest_path = run_dir / "evaluation" / "alltrain_snapshots" / "manifest.json"
    samples = _rows(samples_path)
    final = select_final_sample(samples)
    manifest = json.loads(prediction_manifest_path.read_text(encoding="utf-8"))
    if manifest.get("source_kind") != "paper500_snapshots":
        raise ValueError("alltrain manifest is not snapshot-backed")
    if manifest.get("decisions_sha256") != sha256(samples_path):
        raise ValueError("snapshot manifest changed after inference")
    indexed = {int(item["sample_id"]): item for item in manifest["decisions"]}
    if len(indexed) != len(samples):
        raise ValueError("alltrain snapshot prediction count mismatch")

    with np.load(profile_path, allow_pickle=False) as profile:
        occupied = profile["occupied"].astype(bool)
        valid = profile["valid_space"].astype(bool)
        evaluation = profile["evaluation_mask"].astype(bool)
        start = (int(profile["start_row"]), int(profile["start_col"]))
    goals = np.load(goals_path, allow_pickle=False).astype(np.int32)

    output: list[dict] = []
    errors: list[dict] = []
    for sample in samples:
        sid = int(sample["sample_id"])
        try:
            entry = indexed[sid]
            raw_rel = sample["raw_map_file"]
            raw_path = run_dir / raw_rel
            if entry["raw_map"] != raw_rel or entry["raw_sha256"] != sha256(raw_path):
                raise ValueError("raw-map provenance mismatch")

            raw = _load_raw(raw_path)
            observed05, observed_support = project_runtime_observed(**raw)
            observed10 = reduce_observed(observed05)

            pred_path = run_dir / entry["alltrain_map"]
            with np.load(pred_path, allow_pickle=False) as bundle:
                prediction = np.asarray(bundle["data"], dtype=np.float32)
                top = int(bundle["pad_top"])
                left = int(bundle["pad_left"])
                h = int(bundle["source_height"])
                w = int(bundle["source_width"])
                unpadded = prediction[top:top+h, left:left+w]
                prediction05, prediction_support = project_runtime_prediction(
                    unpadded,
                    float(bundle["resolution"]),
                    float(bundle["origin_x"]),
                    float(bundle["origin_y"]),
                    0.0,
                )

            prediction10 = reduce_prediction(prediction05)
            cov = coverage(observed10, valid)
            iou = occupied_iou(prediction10, occupied, evaluation)
            tu = topological_understanding(
                prediction10,
                occupied,
                evaluation,
                start,
                goals,
            )
            if tu["status"] != "ok":
                raise RuntimeError(f"TU evaluation error: {tu}")

            output.append({
                "sample_id": sid,
                "event": sample["event"],
                "progress_step": int(sample["progress_step"]),
                "distance_m": float(sample["distance_m"]),
                "coverage": cov["coverage"],
                "known_fraction": cov["known_fraction"],
                "occupied_iou": iou["value"],
                "empty_iou_union": iou["empty_union"],
                "tu": tu["value"],
                "tu_success": tu["success"],
                "tu_predicted_occupied": tu["predicted_occupied"],
                "tu_no_path": tu["no_path"],
                "tu_gt_collision": tu["gt_collision"],
                "observed_support_fraction": float(observed_support.mean()),
                "prediction_support_fraction": float(prediction_support.mean()),
                "held": False,
                "held_from_sample_id": "",
            })
        except Exception as exc:  # explicit per-sample error accounting
            errors.append({"sample_id": sid, "error": str(exc)})

    curve_rows: list[dict] = []
    if errors:
        payload = {
            "status": "evaluation_error",
            "errors": errors,
            "evaluated": len(output),
        }
    else:
        final_row = next(
            row for row in output
            if int(row["sample_id"]) == int(final["sample_id"])
        )
        raw_support, held_support, curve_rows = build_curve_support(output, final_row)
        termination_kind = _terminal_kind(str(final_row["event"]))

        coverage_auc = trapezoidal_auc(curve_rows, "coverage")
        occupied_iou_auc = trapezoidal_auc(curve_rows, "occupied_iou")
        tu_auc = trapezoidal_auc(curve_rows, "tu")
        tu_auc["project_added"] = True

        final_sample_id = int(final_row["sample_id"])
        raw_record_count_through_terminal = sum(
            1 for row in output
            if int(row["sample_id"]) <= final_sample_id
        )
        post_terminal_audit_count = sum(
            1 for row in output
            if int(row["sample_id"]) > final_sample_id
        )

        payload = {
            "status": "ok",
            "profile_id": EVAL_ID,
            "prediction_source": "alltrain_snapshots",
            "checkpoint": manifest["checkpoint"],
            "sample_count": len(output),
            "raw_support_count": len(raw_support),
            "raw_record_count_through_terminal": raw_record_count_through_terminal,
            "held_support_count": len(held_support),
            "curve_support_count": len(curve_rows),
            "post_terminal_audit_count": post_terminal_audit_count,
            "termination": {
                "event": final_row["event"],
                "kind": termination_kind,
                "final_progress_step": int(final_row["progress_step"]),
                "hold_applied": bool(held_support),
            },
            "curve_support": {
                "axis": "adapted_step",
                "common_step_spacing": COMMON_STEP_SPACING,
                "steps": [int(row["progress_step"]) for row in curve_rows],
                "raw_support_count": len(raw_support),
                "raw_record_count_through_terminal": raw_record_count_through_terminal,
                "held_support_count": len(held_support),
                "duplicate_step_policy": "last_recorded_sample_wins",
                "post_terminal_samples": "audit_only_excluded_from_curve",
                "natural_completion_hold": (
                    "explicit_k10_hold_to_500"
                    if held_support
                    else "not_applied"
                ),
                "algorithmic_failure_hold": "forbidden",
            },
            "auc": {
                "coverage": coverage_auc,
                "occupied_iou": occupied_iou_auc,
                "tu_project_added": tu_auc,
            },
            "endpoint_at_500": _endpoint_at_500(curve_rows),
            "final": final_row,
            "errors": [],
        }

    evaluation_dir = run_dir / "evaluation" / "paper500"
    evaluation_dir.mkdir(parents=True, exist_ok=True)
    (evaluation_dir / "evaluation.json").write_text(
        json.dumps(payload, indent=2) + "\n",
        encoding="utf-8",
    )

    if output:
        with (evaluation_dir / "samples.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(output[0]))
            writer.writeheader()
            writer.writerows(output)

    if curve_rows:
        with (evaluation_dir / "curve.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=list(curve_rows[0]))
            writer.writeheader()
            writer.writerows(curve_rows)

    return payload


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    default_dir = (
        root
        / "ground_truth"
        / "new_room"
        / "generated"
        / "r003_paper500"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--profile",
        type=Path,
        default=default_dir / f"{EVAL_ID}.npz",
    )
    parser.add_argument(
        "--goals",
        type=Path,
        default=default_dir / f"{EVAL_ID}_tu_goals.npy",
    )
    args = parser.parse_args()
    print(json.dumps(
        evaluate(
            args.run_dir.resolve(),
            args.profile.resolve(),
            args.goals.resolve(),
        ),
        indent=2,
    ))


if __name__ == "__main__":
    main()
