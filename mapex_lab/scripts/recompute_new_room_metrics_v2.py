#!/usr/bin/env python3
"""Destructively migrate New Room derived evaluation metrics from v1 to v2.

Raw experiment evidence is never deleted: maps, predictions, decisions, trajectory,
goals, plans, and runtime configuration remain untouched. Derived evaluation
artifacts are rewritten in place so the repository has one active set of New
Room numbers after the migration.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import hashlib
import json
import math
import statistics
import subprocess
from pathlib import Path

import numpy as np

import analyze_early_stopping
import evaluate_mapex_profiled
import evaluate_nf_profiled
from generate_new_room_ground_truth import generate as generate_new_room_ground_truth

ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = ROOT.parent
GT_REL = Path("ground_truth/new_room/generated/new_room_structural_gt_v2.npz")
ROI_REL = Path("ground_truth/new_room/generated/new_room_connected_free_v2.npy")
GT_PATH = ROOT / GT_REL
ROI_PATH = ROOT / ROI_REL
GT_ID = "new_room_structural_gt_v2"
ROI_ID = "new_room_connected_free_v2"
EVAL_PROFILE = "new_room_v2"
BASELINE_NF = [f"nf_{i:03d}" for i in range(1, 11)]
BASELINE_MAPEX = [f"mpx_{i:03d}" for i in range(1, 11)]


def read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows, fields=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        if not rows:
            raise ValueError(f"cannot infer CSV fields for empty output: {path}")
        fields = list(rows[0].keys())
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def write_json(path: Path, payload):
    path.write_text(json.dumps(payload, indent=2, allow_nan=True) + "\n", encoding="utf-8")


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_head():
    try:
        return subprocess.check_output(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"], text=True
        ).strip()
    except Exception:
        return None


def generate_v2_ground_truth():
    return generate_new_room_ground_truth(
        sdf_path=(ROOT / "map/new_room.sdf").resolve(),
        output_dir=(ROOT / "ground_truth/new_room/generated").resolve(),
        z_slice_m=0.20,
        spawn_x=0.0,
        spawn_y=3.0,
        spawn_yaw=0.0,
    )


def discover_new_room_runs():
    runs = []
    for parent in (ROOT / "experiments/nearest", ROOT / "experiments/mapex"):
        if not parent.is_dir():
            continue
        for run in sorted(p for p in parent.iterdir() if p.is_dir()):
            meta_path = run / "metadata.json"
            if not meta_path.is_file():
                continue
            try:
                meta = load_json(meta_path)
            except Exception:
                continue
            if str(meta.get("environment") or "") == "new_room":
                runs.append(run)
    return runs


def coverage_from_canvas(path: Path, roi: np.ndarray):
    with np.load(path) as bundle:
        data = np.asarray(bundle["data"])
    if data.shape != roi.shape:
        raise ValueError(f"canvas shape {data.shape} != ROI {roi.shape}: {path}")
    denominator = int(np.count_nonzero(roi))
    return float(np.count_nonzero((data >= 0) & roi) / denominator)


def fmt_float(value):
    return "nan" if not math.isfinite(value) else f"{value:.9f}"


def rewrite_snapshots(run: Path, roi: np.ndarray):
    path = run / "snapshots.csv"
    rows = read_csv(path)
    if not rows:
        raise RuntimeError(f"no snapshots: {path}")
    checkpoints = []
    for row in rows:
        rel = (row.get("canvas_map_file") or "").strip()
        if not rel:
            raise RuntimeError(f"snapshot missing canvas_map_file: {path}")
        coverage = coverage_from_canvas(run / rel, roi)
        row["coverage"] = fmt_float(coverage)
        checkpoints.append((float(row["time_s"]), coverage))
    write_csv(path, rows, list(rows[0].keys()))
    final_rows = [r for r in rows if (r.get("event") or "").strip() == "final"]
    final_row = final_rows[-1] if final_rows else rows[-1]
    return sorted(checkpoints), float(final_row["coverage"])


def clear_and_backfill_metrics(run: Path, checkpoints):
    path = run / "metrics.csv"
    rows = read_csv(path)
    if not rows:
        raise RuntimeError(f"no metrics: {path}")
    times = [x[0] for x in checkpoints]
    values = [x[1] for x in checkpoints]
    for row in rows:
        index = bisect.bisect_right(times, float(row["time_s"])) - 1
        row["coverage"] = "nan" if index < 0 else fmt_float(values[index])
        if "occupied_iou" in row:
            row["occupied_iou"] = "nan"
        if "tu" in row:
            row["tu"] = "nan"
    write_csv(path, rows, list(rows[0].keys()))


def clear_old_evaluation_fields(run: Path, final_coverage: float, roi_n: int):
    meta_path = run / "metadata.json"
    summary_path = run / "summary.json"
    metadata = load_json(meta_path)
    summary = load_json(summary_path)
    migration = {
        "evaluation_profile": EVAL_PROFILE,
        "source_evaluation_roi_id": "new_room_connected_free_v1",
        "source_structural_ground_truth_id": "new_room_structural_gt_v1",
        "reason": "correct Gazebo-world to SLAM-start frame alignment using spawn (0,3,0)",
        "coverage_source": "saved canvas snapshots; metrics.csv uses latest-snapshot step hold",
        "git_head_when_migrated": git_head(),
    }
    metadata.update({
        "evaluation_profile_version": EVAL_PROFILE,
        "evaluation_roi_id": ROI_ID,
        "evaluation_roi_denominator": roi_n,
        "evaluation_roi_file": ROI_REL.as_posix(),
        "structural_ground_truth_id": GT_ID,
        "structural_ground_truth_file": GT_REL.as_posix(),
        "structural_ground_truth_exists_at_start": True,
        "final_coverage": final_coverage,
        "evaluation_migration": migration,
    })
    summary.update({
        "evaluation_profile_version": EVAL_PROFILE,
        "evaluation_roi_id": ROI_ID,
        "evaluation_roi_denominator": roi_n,
        "structural_ground_truth_id": GT_ID,
        "final_coverage": final_coverage,
        "offline_evaluation_status": "pending_v2_recompute",
        "offline_evaluation_reason": None,
        "final_occupied_iou": None,
        "final_tu": None,
        "occupied_iou_auc_time": None,
        "occupied_iou_auc_distance": None,
        "tu_auc_time": None,
        "tu_auc_distance": None,
        "evaluation_migration": migration,
    })
    write_json(meta_path, metadata)
    write_json(summary_path, summary)
    for stale in (run / "evaluation.csv", run / "evaluation.json", run / "evaluation/tu_goals.npy"):
        if stale.is_file():
            stale.unlink()
    return metadata


def normalize_evaluation_json(run: Path):
    path = run / "evaluation.json"
    if not path.is_file():
        return
    payload = load_json(path)
    payload["evaluation_profile_version"] = EVAL_PROFILE
    payload["ground_truth"] = GT_REL.as_posix()
    payload["ground_truth_sha256"] = sha256(GT_PATH)
    if isinstance(payload.get("tu"), dict):
        payload["tu"]["goal_source"] = ROI_REL.as_posix()
    write_json(path, payload)


def evaluate_quality(run: Path, metadata):
    method = str(metadata.get("method") or "")
    if method.startswith("nearest") or run.parent.name == "nearest":
        result = evaluate_nf_profiled.evaluate_run(
            run, GT_PATH, ROI_PATH, allow_observed_fallback=False
        )
    else:
        result = evaluate_mapex_profiled.evaluate_run(run, GT_PATH, ROI_PATH)
    if result.get("status") != "ok":
        raise RuntimeError(f"offline evaluation failed for {run.name}: {result}")
    normalize_evaluation_json(run)
    return result


def regenerate_early_stopping(run: Path, metadata):
    if str(metadata.get("method") or "") != "mapex":
        return False
    if not (run / "decisions.csv").is_file():
        return False
    analyze_early_stopping.analyze_run(
        run, ground_truth_path=str(GT_PATH), roi_path=str(ROI_PATH)
    )
    return True


def update_way2_online_runs():
    path = ROOT / "analysis/way2_online_runs.csv"
    if not path.is_file():
        return
    rows = read_csv(path)
    for row in rows:
        if row.get("environment") != "new_room":
            continue
        summary_path = ROOT / "experiments/mapex" / row["run_id"] / "summary.json"
        if not summary_path.is_file():
            continue
        summary = load_json(summary_path)
        row["final_coverage"] = str(summary.get("final_coverage", ""))
        row["final_occupied_iou"] = str(summary.get("final_occupied_iou", ""))
        row["final_tu"] = str(summary.get("final_tu", ""))
    write_csv(path, rows, list(rows[0].keys()))


def value(summary, key):
    try:
        return float(summary.get(key))
    except (TypeError, ValueError):
        return math.nan


def baseline_run_dir(method, run_id):
    folder = "nearest" if method == "NF" else "mapex"
    return ROOT / "experiments" / folder / run_id


def build_baseline_run_table():
    rows = []
    for method, ids in (("NF", BASELINE_NF), ("MapEx", BASELINE_MAPEX)):
        for run_id in ids:
            summary = load_json(baseline_run_dir(method, run_id) / "summary.json")
            rows.append({
                "method": method,
                "run_id": run_id,
                "evaluation_profile": EVAL_PROFILE,
                "final_coverage": value(summary, "final_coverage"),
                "final_known_fraction": value(summary, "final_known_fraction"),
                "total_distance_m": value(summary, "total_distance_m"),
                "total_time_s": value(summary, "total_time_s"),
                "main_success_rate": value(summary, "main_success_rate"),
                "final_occupied_iou": value(summary, "final_occupied_iou"),
                "final_tu": value(summary, "final_tu"),
            })
    return rows


def finite(values):
    return [x for x in values if math.isfinite(x)]


def build_baseline_summary(run_rows):
    metrics = [
        ("Coverage", "final_coverage"),
        ("Distance (m)", "total_distance_m"),
        ("Time (s)", "total_time_s"),
        ("Success rate", "main_success_rate"),
        ("Occupied IoU", "final_occupied_iou"),
        ("TU", "final_tu"),
    ]
    output = []
    for label, key in metrics:
        row = {"metric": label, "evaluation_profile": EVAL_PROFILE}
        for method in ("NF", "MapEx"):
            vals = finite([float(r[key]) for r in run_rows if r["method"] == method])
            row[f"{method.lower()}_mean"] = statistics.fmean(vals)
            row[f"{method.lower()}_std"] = statistics.stdev(vals) if len(vals) > 1 else 0.0
        output.append(row)
    return output


def series_from_csv(path: Path, x_key, y_key):
    points = []
    for row in read_csv(path):
        try:
            x, y = float(row[x_key]), float(row[y_key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append((x, y))
    if not points:
        raise RuntimeError(f"no usable {x_key}/{y_key} samples in {path}")
    dedup = {}
    for x, y in sorted(points):
        dedup[x] = y
    xs = np.asarray(sorted(dedup), dtype=np.float64)
    ys = np.asarray([dedup[x] for x in xs], dtype=np.float64)
    if xs[0] > 0.0:
        xs = np.insert(xs, 0, 0.0)
        ys = np.insert(ys, 0, ys[0])
    return xs, ys


def build_curve(source_name, y_key, x_key, step):
    series = {}
    for method, ids in (("NF", BASELINE_NF), ("MapEx", BASELINE_MAPEX)):
        for run_id in ids:
            series[(method, run_id)] = series_from_csv(
                baseline_run_dir(method, run_id) / source_name, x_key, y_key
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
        }
        for method in ("NF", "MapEx"):
            ids = BASELINE_NF if method == "NF" else BASELINE_MAPEX
            vals = [
                float(np.interp(x, *series[(method, run_id)]))
                for run_id in ids
            ]
            row[f"{method.lower()}_mean"] = float(np.mean(vals))
            row[f"{method.lower()}_std"] = float(np.std(vals, ddof=1))
            row[f"{method.lower()}_n"] = len(vals)
        output.append(row)
    return output


def regenerate_baseline_aggregates():
    run_rows = build_baseline_run_table()
    write_csv(ROOT / "analysis/new_room_baseline_runs.csv", run_rows)
    write_csv(ROOT / "analysis/new_room_baseline_summary.csv", build_baseline_summary(run_rows))
    for axis, key, step in (("time", "time_s", 10.0), ("distance", "distance_m", 5.0)):
        write_csv(
            ROOT / f"analysis/new_room_coverage_{axis}_curve.csv",
            build_curve("snapshots.csv", "coverage", key, step),
        )
        write_csv(
            ROOT / f"analysis/new_room_iou_{axis}_curve.csv",
            build_curve("evaluation.csv", "occupied_iou", key, step),
        )


def update_status():
    path = ROOT / "STATUS.md"
    if not path.is_file():
        return
    text = path.read_text(encoding="utf-8")
    text = text.replace(
        "Historical New Room coverage values remain recorded as v1 until offline backfill from saved canvases is validated; time, distance, trajectory, navigation outcomes, MapEx scoring, and Way2 R/U are unaffected.",
        "Committed New Room derived Coverage/IoU/TU metrics have been backfilled from saved artifacts using the frame-correct v2 evaluator; time, distance, trajectory, navigation outcomes, MapEx scoring, and Way2 R/U are unchanged.",
    )
    text = text.replace(
        "Existing New Room run summaries/CSV coverage were produced with `new_room_connected_free_v1`, whose structural geometry was in Gazebo world coordinates rather than the SLAM-start frame. Treat those absolute coverage values as superseded/pending v2 backfill; do not compare v1 New Room coverage numerically with Hospital coverage.",
        "New Room run summaries/CSV evaluation metrics now use `new_room_connected_free_v2` / `new_room_structural_gt_v2`; v1-derived Coverage/IoU/TU values were replaced from saved artifacts and are no longer the active repository results.",
    )
    text = text.replace(
        "1. Generate and visually validate `new_room_connected_free_v2` against a saved New Room final canvas (recommended sanity run: `mpx_w2_002`), then backfill historical New Room coverage/coverage curves from saved canvas snapshots before interpreting absolute coverage.",
        "1. Treat `new_room_connected_free_v2` / `new_room_structural_gt_v2` as the canonical New Room evaluation profile and regenerate aggregates after adding new runs.",
    )
    for run_id in ("mpx_w2_001", "mpx_w2_002"):
        summary_path = ROOT / "experiments/mapex" / run_id / "summary.json"
        if not summary_path.is_file():
            continue
        coverage = value(load_json(summary_path), "final_coverage")
        lines = text.splitlines()
        for i, line in enumerate(lines):
            if f"`{run_id}`" not in line or not line.lstrip().startswith("|"):
                continue
            parts = line.split("|")
            if len(parts) >= 9:
                parts[4] = f" {coverage:.6f} "
                lines[i] = "|".join(parts)
            break
        text = "\n".join(lines) + ("\n" if text.endswith("\n") else "")
    path.write_text(text, encoding="utf-8")


def audit_no_active_v1(runs):
    problems = []
    for run in runs:
        metadata = load_json(run / "metadata.json")
        summary = load_json(run / "summary.json")
        evaluation = load_json(run / "evaluation.json")
        checks = {
            "metadata.evaluation_roi_id": metadata.get("evaluation_roi_id"),
            "metadata.structural_ground_truth_id": metadata.get("structural_ground_truth_id"),
            "summary.evaluation_roi_id": summary.get("evaluation_roi_id"),
            "summary.structural_ground_truth_id": summary.get("structural_ground_truth_id"),
            "evaluation.evaluation_profile_version": evaluation.get("evaluation_profile_version"),
        }
        for key, current in checks.items():
            if current is None:
                continue
            if "connected_free_v1" in str(current) or "structural_gt_v1" in str(current):
                problems.append(f"{run.name}: {key}={current}")
    if problems:
        raise RuntimeError("active v1 references remain:\n" + "\n".join(problems))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-early-stopping", action="store_true")
    args = parser.parse_args()

    gt_summary = generate_v2_ground_truth()
    roi = np.load(ROI_PATH).astype(bool)
    roi_n = int(np.count_nonzero(roi))
    if roi_n != int(gt_summary["connected_free_roi_cells"]):
        raise RuntimeError("ROI denominator mismatch after generation")

    runs = discover_new_room_runs()
    if not runs:
        raise RuntimeError("no New Room runs found")

    print(f"Migrating {len(runs)} New Room runs to {EVAL_PROFILE}")
    failures = []
    for index, run in enumerate(runs, 1):
        print(f"[{index:02d}/{len(runs):02d}] {run.parent.name}/{run.name}")
        try:
            checkpoints, final_coverage = rewrite_snapshots(run, roi)
            clear_and_backfill_metrics(run, checkpoints)
            metadata = clear_old_evaluation_fields(run, final_coverage, roi_n)
            result = evaluate_quality(run, metadata)
            if not args.skip_early_stopping:
                regenerate_early_stopping(run, metadata)
            print(
                f"  coverage={final_coverage:.6f} "
                f"IoU={float(result['final_occupied_iou']):.6f} "
                f"TU={float(result['final_tu']):.6f}"
            )
        except Exception as exc:
            failures.append(f"{run}: {exc}")
            print(f"  ERROR: {exc}")

    if failures:
        raise RuntimeError("migration incomplete:\n" + "\n".join(failures))

    update_way2_online_runs()
    regenerate_baseline_aggregates()
    update_status()
    audit_no_active_v1(runs)

    report = {
        "evaluation_profile": EVAL_PROFILE,
        "ground_truth": GT_REL.as_posix(),
        "ground_truth_sha256": sha256(GT_PATH),
        "roi": ROI_REL.as_posix(),
        "roi_cells": roi_n,
        "runs_migrated": [str(run.relative_to(ROOT)) for run in runs],
        "coverage_metrics_semantics": "snapshots exact; metrics.csv latest-snapshot step hold",
        "iou_tu_semantics": "recomputed by current profiled evaluators with v2 GT/ROI",
        "raw_experiment_data_modified": False,
    }
    write_json(ROOT / "analysis/new_room_v2_migration_report.json", report)
    print("Migration completed successfully.")
    print("Review: git -C", REPO_ROOT, "status --short")


if __name__ == "__main__":
    main()
