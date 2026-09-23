#!/usr/bin/env python3
"""Evaluate R003 common paper1000 samples with fixed masks/goals."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from predict_alltrain_offline import sha256
from r003_paper1000 import (
    EVAL_ID,
    coverage,
    occupied_iou,
    project_runtime_observed,
    project_runtime_prediction,
    reduce_observed,
    reduce_prediction,
    select_final_sample,
    topological_understanding,
)


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


def evaluate(run_dir: Path, profile_path: Path, goals_path: Path) -> dict:
    samples_path = run_dir / "paper1000_snapshots.csv"
    prediction_manifest_path = run_dir / "evaluation" / "alltrain_snapshots" / "manifest.json"
    samples = _rows(samples_path)
    final = select_final_sample(samples)
    manifest = json.loads(prediction_manifest_path.read_text(encoding="utf-8"))
    if manifest.get("source_kind") != "paper1000_snapshots":
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
    output = []
    errors = []
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
                    unpadded, float(bundle["resolution"]), float(bundle["origin_x"]),
                    float(bundle["origin_y"]), 0.0,
                )
            prediction10 = reduce_prediction(prediction05)
            cov = coverage(observed10, valid)
            iou = occupied_iou(prediction10, occupied, evaluation)
            tu = topological_understanding(prediction10, occupied, evaluation, start, goals)
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
            })
        except Exception as exc:  # explicit per-sample error accounting
            errors.append({"sample_id": sid, "error": str(exc)})
    if errors:
        payload = {"status": "evaluation_error", "errors": errors, "evaluated": len(output)}
    else:
        final_row = next(row for row in output if row["sample_id"] == int(final["sample_id"]))
        payload = {
            "status": "ok",
            "profile_id": EVAL_ID,
            "prediction_source": "alltrain_snapshots",
            "checkpoint": manifest["checkpoint"],
            "sample_count": len(output),
            "raw_support_count": len(output),
            "held_support_count": 0,
            "final": final_row,
            "errors": [],
        }
    evaluation_dir = run_dir / "evaluation" / "paper1000"
    evaluation_dir.mkdir(parents=True, exist_ok=True)
    (evaluation_dir / "evaluation.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    if output:
        with (evaluation_dir / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(output[0]))
            writer.writeheader(); writer.writerows(output)
    return payload


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    default_dir = root / "ground_truth" / "new_room" / "generated" / "r003_paper1000"
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--profile", type=Path, default=default_dir / f"{EVAL_ID}.npz")
    parser.add_argument("--goals", type=Path, default=default_dir / f"{EVAL_ID}_tu_goals.npy")
    args = parser.parse_args()
    print(json.dumps(evaluate(args.run_dir.resolve(), args.profile.resolve(), args.goals.resolve()), indent=2))


if __name__ == "__main__":
    main()
