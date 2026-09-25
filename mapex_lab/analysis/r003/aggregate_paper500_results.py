#!/usr/bin/env python3
"""Aggregate the accepted R003 paper500 cohort without mutating source runs."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import statistics
import subprocess
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


DATA_SHA = "7f01b4a725dbc4e7dd788f78ad9ac0992f311068"
CHECKPOINT_SHA256 = "58c1841c10d4d97bc1f894ec8fe5b015b165277ddb2b9bb29eb688b21d4363b5"
PROFILE_ID = "new_room_mapex_eval_010_v1"
PREDICTION_SOURCE = "alltrain_snapshots"
STEPS = tuple(range(0, 501, 10))
RUNS = tuple(
    [("NF", "nearest", f"nf_p500_{index:03d}") for index in range(1, 11)]
    + [("MapEx", "mapex", f"mpx_p500_{index:03d}") for index in range(1, 11)]
)
METRICS = (
    ("coverage_auc", "Coverage AUC", "metric x adapted step", False),
    ("occupied_iou_auc", "Occupied-IoU AUC", "metric x adapted step", False),
    ("tu_auc_project_added", "TU AUC", "metric x adapted step", True),
    ("coverage_at_500", "Coverage@500", "fraction", False),
    ("occupied_iou_at_500", "Occupied-IoU@500", "fraction", False),
    ("tu_at_500", "TU@500", "fraction", False),
)
CURVE_METRICS = (
    ("coverage", "Coverage", "Coverage vs adapted step", "coverage_vs_adapted_step.png"),
    ("occupied_iou", "Occupied IoU", "Occupied-IoU vs adapted step", "occupied_iou_vs_adapted_step.png"),
    ("tu", "TU", "Topological Understanding vs adapted step", "tu_vs_adapted_step.png"),
)


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def write_csv(path: Path, rows, fieldnames=None):
    rows = list(rows)
    if fieldnames is None:
        fieldnames = []
        for row in rows:
            for key in row:
                if key not in fieldnames:
                    fieldnames.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def git_head(path: Path):
    return subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()


def require_data_identity(actual, expected=DATA_SHA):
    if actual != expected:
        raise ValueError(f"data identity mismatch: {actual} != {expected}")


def validate_run_ids(run_ids):
    run_ids = list(run_ids)
    expected = [run_id for _, _, run_id in RUNS]
    if len(run_ids) != len(set(run_ids)):
        raise ValueError("duplicate run ID")
    if run_ids != expected:
        missing = sorted(set(expected) - set(run_ids))
        extra = sorted(set(run_ids) - set(expected))
        raise ValueError(f"official run universe mismatch: missing={missing}, extra={extra}")


def validate_steps(steps):
    steps = list(steps)
    if len(steps) != len(set(steps)):
        raise ValueError("duplicate adapted step")
    if steps != list(STEPS):
        raise ValueError("common adapted-step support mismatch")


def trapezoid(steps, values):
    if len(steps) != len(values) or len(steps) < 2:
        raise ValueError("invalid AUC vectors")
    return sum((values[index] + values[index + 1]) * 0.5
               * (steps[index + 1] - steps[index])
               for index in range(len(steps) - 1))


def require_close(actual, expected, label, atol=1e-10):
    if not (math.isfinite(actual) and math.isfinite(expected)
            and math.isclose(actual, expected, rel_tol=0.0, abs_tol=atol)):
        raise ValueError(f"{label} mismatch: {actual} != {expected}")


def summarize(values):
    values = [float(value) for value in values]
    if len(values) < 2 or not all(math.isfinite(value) for value in values):
        raise ValueError("summary requires at least two finite run values")
    return {"n": len(values), "mean": statistics.fmean(values),
            "sample_std": statistics.stdev(values)}


def input_paths(data_root: Path):
    paths = []
    for _, parent, run_id in RUNS:
        directory = data_root / "mapex_lab/experiments" / parent / run_id / "evaluation/paper500"
        paths.extend(directory / name for name in ("evaluation.json", "curve.csv", "samples.csv"))
    return paths


def fingerprint_inputs(data_root: Path):
    return {str(path.relative_to(data_root)): sha256(path) for path in input_paths(data_root)}


def load_run(data_root: Path, algorithm, parent, run_id):
    directory = data_root / "mapex_lab/experiments" / parent / run_id / "evaluation/paper500"
    missing = [name for name in ("evaluation.json", "curve.csv", "samples.csv")
               if not (directory / name).is_file()]
    if missing:
        raise ValueError(f"{run_id}: missing evaluator files {missing}")
    evaluation = json.loads((directory / "evaluation.json").read_text(encoding="utf-8"))
    curve = read_csv(directory / "curve.csv")
    samples = read_csv(directory / "samples.csv")
    if evaluation.get("status") != "ok" or evaluation.get("errors") != []:
        raise ValueError(f"{run_id}: evaluator status/errors mismatch")
    if evaluation.get("profile_id") != PROFILE_ID:
        raise ValueError(f"{run_id}: profile mismatch")
    if evaluation.get("prediction_source") != PREDICTION_SOURCE:
        raise ValueError(f"{run_id}: prediction source mismatch")
    if evaluation.get("checkpoint", {}).get("sha256") != CHECKPOINT_SHA256:
        raise ValueError(f"{run_id}: checkpoint mismatch")
    if evaluation.get("sample_count") != 53 or len(samples) != 53:
        raise ValueError(f"{run_id}: sample count mismatch")
    if evaluation.get("termination", {}).get("final_progress_step") != 500:
        raise ValueError(f"{run_id}: terminal step mismatch")
    steps = [int(row["progress_step"]) for row in curve]
    validate_steps(steps)
    endpoint = evaluation.get("endpoint_at_500") or {}
    if endpoint.get("progress_step") != 500:
        raise ValueError(f"{run_id}: endpoint missing")
    terminal = curve[-1]
    for field in ("coverage", "occupied_iou", "tu"):
        require_close(float(terminal[field]), float(endpoint[field]), f"{run_id} endpoint {field}", 1e-12)
    auc_keys = {"coverage": "coverage", "occupied_iou": "occupied_iou", "tu": "tu_project_added"}
    recomputed = {}
    for field, key in auc_keys.items():
        values = [float(row[field]) for row in curve]
        area = trapezoid(steps, values)
        saved = float(evaluation["auc"][key]["area"])
        require_close(area, saved, f"{run_id} AUC {field}")
        recomputed[field] = area
    scalar = {
        "algorithm": algorithm, "run_id": run_id, "data_commit": DATA_SHA,
        "profile_id": PROFILE_ID, "prediction_source": PREDICTION_SOURCE,
        "checkpoint_sha256": CHECKPOINT_SHA256, "evaluator_status": "ok",
        "sample_count": 53, "curve_point_count": len(curve),
        "adapted_step_start": steps[0], "adapted_step_end": steps[-1],
        "auc_rule": "deterministic_trapezoidal",
        "coverage_auc": float(evaluation["auc"]["coverage"]["area"]),
        "occupied_iou_auc": float(evaluation["auc"]["occupied_iou"]["area"]),
        "tu_auc_project_added": float(evaluation["auc"]["tu_project_added"]["area"]),
        "coverage_at_500": float(endpoint["coverage"]),
        "occupied_iou_at_500": float(endpoint["occupied_iou"]),
        "tu_at_500": float(endpoint["tu"]),
    }
    curve_rows = [{"algorithm": algorithm, "run_id": run_id,
                   "adapted_step": int(row["progress_step"]),
                   "coverage": float(row["coverage"]),
                   "occupied_iou": float(row["occupied_iou"]), "tu": float(row["tu"])}
                  for row in curve]
    return scalar, curve_rows


def group_summaries(per_run):
    rows = []
    for algorithm in ("NF", "MapEx"):
        selected = [row for row in per_run if row["algorithm"] == algorithm]
        if len(selected) != 10:
            raise ValueError(f"{algorithm}: expected n=10")
        for key, label, units, project_added in METRICS:
            summary = summarize(row[key] for row in selected)
            rows.append({"algorithm": algorithm, "metric": key, "display_label": label,
                         "units": units, "project_added": project_added, **summary})
    return rows


def curve_summaries(curves):
    indexed = defaultdict(list)
    for row in curves:
        indexed[(row["algorithm"], row["adapted_step"])].append(row)
    rows = []
    for algorithm in ("NF", "MapEx"):
        for step in STEPS:
            selected = indexed[(algorithm, step)]
            if len(selected) != 10:
                raise ValueError(f"{algorithm} step {step}: expected n=10")
            item = {"algorithm": algorithm, "adapted_step": step, "n": len(selected)}
            for key, _, _, _ in CURVE_METRICS:
                summary = summarize(row[key] for row in selected)
                item[f"{key}_mean"] = summary["mean"]
                item[f"{key}_sample_std"] = summary["sample_std"]
            rows.append(item)
    return rows


def format_pm(mean, std):
    return f"{mean:.6f} ± {std:.6f}"


def write_tab_ready(output: Path, per_run, summaries, curve_rows):
    by_summary = {(row["algorithm"], row["metric"]): row for row in summaries}
    summary_rows = []
    for key, label, units, project_added in METRICS:
        nf = by_summary[("NF", key)]; mapex = by_summary[("MapEx", key)]
        summary_rows.append({"section": "Primary AUC" if key.endswith("auc") or "_auc_" in key else "Secondary endpoint",
                             "metric": label, "units": units, "project_added": project_added,
                             "nf_n": nf["n"], "nf_mean": nf["mean"], "nf_sample_std": nf["sample_std"],
                             "nf_mean_pm_std": format_pm(nf["mean"], nf["sample_std"]),
                             "mapex_n": mapex["n"], "mapex_mean": mapex["mean"],
                             "mapex_sample_std": mapex["sample_std"],
                             "mapex_mean_pm_std": format_pm(mapex["mean"], mapex["sample_std"])})
    write_csv(output / "tabs/Summary.csv", summary_rows)
    for algorithm in ("NF", "MapEx"):
        selected = [row for row in per_run if row["algorithm"] == algorithm]
        run_ids = [row["run_id"] for row in selected]
        wide = []
        for key, label, units, project_added in METRICS:
            values = [row[key] for row in selected]
            summary = summarize(values)
            wide.append({"metric": label, "metric_key": key, "units": units,
                         "project_added": project_added,
                         **{run_id: value for run_id, value in zip(run_ids, values)},
                         "Mean": summary["mean"], "Sample Std": summary["sample_std"]})
        write_csv(output / f"tabs/{algorithm}.csv", wide,
                  ["metric", "metric_key", "units", "project_added", *run_ids, "Mean", "Sample Std"])
    by_curve = {(row["algorithm"], row["adapted_step"]): row for row in curve_rows}
    for key, label, _, _ in CURVE_METRICS:
        rows = []
        for step in STEPS:
            nf = by_curve[("NF", step)]; mapex = by_curve[("MapEx", step)]
            rows.append({"adapted_step": step, "nf_mean": nf[f"{key}_mean"],
                         "nf_sample_std": nf[f"{key}_sample_std"], "nf_n": nf["n"],
                         "mapex_mean": mapex[f"{key}_mean"],
                         "mapex_sample_std": mapex[f"{key}_sample_std"], "mapex_n": mapex["n"]})
        write_csv(output / f"tabs/{label.replace(' ', '_')}_Curves.csv", rows)


def plot_curves(output: Path, rows):
    indexed = {(row["algorithm"], row["adapted_step"]): row for row in rows}
    colors = {"NF": "#2f5d7c", "MapEx": "#d97706"}
    for key, label, title, filename in CURVE_METRICS:
        fig, ax = plt.subplots(figsize=(7.4, 4.6))
        for algorithm in ("NF", "MapEx"):
            means = [indexed[(algorithm, step)][f"{key}_mean"] for step in STEPS]
            stds = [indexed[(algorithm, step)][f"{key}_sample_std"] for step in STEPS]
            lower = [mean - std for mean, std in zip(means, stds)]
            upper = [mean + std for mean, std in zip(means, stds)]
            ax.plot(STEPS, means, color=colors[algorithm], linewidth=2.0, label=f"{algorithm} mean")
            ax.fill_between(STEPS, lower, upper, color=colors[algorithm], alpha=0.18,
                            label=f"{algorithm} ± sample std")
        ax.set(xlabel="Adapted step", ylabel=label, title=title, xlim=(0, 500))
        ax.grid(alpha=0.25)
        ax.legend(frameon=False, fontsize=8, ncol=2)
        fig.tight_layout()
        path = output / "figures" / filename
        path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, dpi=180)
        plt.close(fig)


def output_hashes(output: Path, excluded=()):
    excluded = set(excluded)
    return {str(path.relative_to(output)): sha256(path) for path in sorted(output.rglob("*"))
            if path.is_file() and str(path.relative_to(output)) not in excluded}


def aggregate(data_root: Path, output: Path, implementation_sha: str, test_log: Path | None = None):
    actual_data_sha = git_head(data_root)
    require_data_identity(actual_data_sha)
    validate_run_ids(run_id for _, _, run_id in RUNS)
    before = fingerprint_inputs(data_root)
    per_run, all_curves = [], []
    for run_spec in RUNS:
        scalar, curves = load_run(data_root, *run_spec)
        per_run.append(scalar); all_curves.extend(curves)
    summaries = group_summaries(per_run)
    curve_rows = curve_summaries(all_curves)
    temporary = output.with_name(output.name + ".tmp")
    if output.exists() or temporary.exists():
        raise ValueError(f"refusing existing output/temporary root: {output}")
    temporary.mkdir(parents=True)
    try:
        write_csv(temporary / "per_run_metrics.csv", per_run)
        write_csv(temporary / "group_summary.csv", summaries)
        write_csv(temporary / "curve_mean_std.csv", curve_rows)
        write_tab_ready(temporary, per_run, summaries, curve_rows)
        plot_curves(temporary, curve_rows)
        atomic_json(temporary / "input_hashes.json", {"schema": "r003_paper500_input_hashes_v1",
                    "data_commit": actual_data_sha, "file_count": len(before), "hashes": before})
        layout = {"schema": "r003_paper500_tab_layout_v1", "destination": "new standalone Google Sheets file",
                  "historical_workbook_mutated": False,
                  "tabs": [
                      {"name": "Summary", "source": "tabs/Summary.csv", "charts": [item[3] for item in CURVE_METRICS]},
                      {"name": "NF", "source": "tabs/NF.csv"},
                      {"name": "MapEx", "source": "tabs/MapEx.csv"},
                      {"name": "Coverage Curves", "source": "tabs/Coverage_Curves.csv"},
                      {"name": "IoU Curves", "source": "tabs/Occupied_IoU_Curves.csv"},
                      {"name": "TU Curves", "source": "tabs/TU_Curves.csv"}],
                  "presentation": {"primary": ["Coverage AUC", "Occupied-IoU AUC", "TU AUC (project-added)"],
                                   "secondary": ["Coverage@500", "Occupied-IoU@500", "TU@500"],
                                   "curve_axis": "adapted step 0..500", "uncertainty": "across-run sample standard deviation"}}
        atomic_json(temporary / "tab_layout_manifest.json", layout)
        if test_log:
            shutil.copy2(test_log, temporary / "test.log")
        (temporary / "execution.log").write_text(
            f"{utc_now()} aggregate START data={actual_data_sha} implementation={implementation_sha}\n"
            f"{utc_now()} validated 20 runs, 60 inputs, 102 group-step rows\n"
            f"{utc_now()} aggregate COMPLETE\n", encoding="utf-8")
        completion = (
            "# R003 Paper500 Results Completion\n\n"
            f"- Status: COMPLETE_PENDING_REVIEW\n- Data commit: `{actual_data_sha}`\n"
            f"- Implementation commit: `{implementation_sha}`\n- Cohort: 10 NF + 10 MapEx\n"
            "- Support: adapted step 0,10,...,500\n- Aggregation: equal-run mean and sample standard deviation\n"
            "- Scientific interpretation: not performed\n- Historical workbook modified: no\n")
        (temporary / "R003_PAPER500_RESULTS_COMPLETION.md").write_text(completion, encoding="utf-8")
        after = fingerprint_inputs(data_root)
        if before != after:
            raise ValueError("source run artifacts changed during aggregation")
        hashes = output_hashes(temporary, excluded={"aggregate_manifest.json"})
        manifest = {"schema": "r003_paper500_aggregate_manifest_v1", "status": "complete_pending_review",
                    "generated_at": utc_now(), "implementation_sha": implementation_sha,
                    "data_commit": actual_data_sha, "run_count": len(per_run), "nf_count": 10,
                    "mapex_count": 10, "curve_support": list(STEPS), "curve_rows": len(curve_rows),
                    "auc_rule": "deterministic_trapezoidal", "group_aggregation": "equal-run mean and sample std",
                    "source_artifacts_unchanged": True, "historical_workbook_mutated": False,
                    "scientific_interpretation": None, "artifact_count": len(hashes), "hashes": hashes}
        atomic_json(temporary / "aggregate_manifest.json", manifest)
        temporary.replace(output)
        return manifest
    except Exception:
        raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--implementation-sha", required=True)
    parser.add_argument("--test-log", type=Path)
    args = parser.parse_args()
    result = aggregate(args.data_root.resolve(), args.output.resolve(), args.implementation_sha,
                       args.test_log.resolve() if args.test_log else None)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
