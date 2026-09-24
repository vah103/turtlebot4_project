#!/usr/bin/env python3
"""Run the frozen R004 topology diagnostic with atomic run checkpoints."""
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import traceback
from datetime import datetime, timezone
from pathlib import Path

from mapex_lab.analysis.r004 import evaluate_topology_traversability as topology


METHOD_ID = "R004_TOPOLOGY_TRAVERSABILITY_METHOD_V2"
RUN_FILES = (
    "topology_diagnostic_decisions.csv",
    "source_pose_alignment.csv",
    "overlay_manifest.csv",
    "missed_free_clearance_distribution.csv",
    "fragmentation_inventory.json",
    "run_result.json",
)
FINAL_FILES = (
    "topology_diagnostic_decisions.csv",
    "topology_diagnostic_run_bins.csv",
    "topology_diagnostic_summary.json",
    "source_pose_alignment_inventory.csv",
    "topology_overlay_manifest.csv",
    "missed_free_clearance_distribution.csv",
    "fragmentation_inventory.json",
    "topology_provenance.json",
    "figures/navigable_missed_free_vs_progress.png",
    "figures/reachable_future_free_retention_vs_progress.png",
    "figures/reachable_pair_connectivity_retention_vs_progress.png",
    "figures/largest_piece_and_component_count_vs_progress.png",
    "figures/support_context_vs_progress.png",
    "figures/missed_free_clearance_distribution.png",
)
ALLOWED_FILES = {
    "mapex_lab/analysis/r004/evaluate_topology_traversability.py",
    "mapex_lab/analysis/r004/test_evaluate_topology_traversability.py",
    "mapex_lab/analysis/r004/run_topology_low_touch.py",
}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def git(repo: Path, *args):
    return subprocess.check_output(["git", *args], cwd=repo, text=True).strip()


def append_log(path: Path, message: str):
    line = f"{utc_now()} {message}"
    print(line, flush=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(line + "\n")


def inventory(directory: Path, excluded=()):
    excluded = set(excluded)
    return {str(path.relative_to(directory)): topology.sha256(path)
            for path in sorted(directory.rglob("*"))
            if path.is_file() and str(path.relative_to(directory)) not in excluded}


def read_rows(path: Path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def verify_run(directory: Path, run_id: str, expected_decisions=None):
    manifest_path = directory / "completion_manifest.json"
    if not manifest_path.is_file():
        raise ValueError(f"{run_id}: missing completion manifest")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("status") != "complete" or manifest.get("run_id") != run_id:
        raise ValueError(f"{run_id}: invalid completion manifest identity/status")
    if manifest.get("accepted_base_sha") != topology.ACCEPTED_BASE_SHA:
        raise ValueError(f"{run_id}: accepted-base mismatch")
    for relative, expected_hash in manifest.get("output_hashes", {}).items():
        path = directory / relative
        if not path.is_file() or topology.sha256(path) != expected_hash:
            raise ValueError(f"{run_id}: output hash mismatch: {relative}")
    rows = read_rows(directory / "topology_diagnostic_decisions.csv")
    if expected_decisions is not None and len(rows) != expected_decisions:
        raise ValueError(f"{run_id}: decision row count mismatch")
    overlays = read_rows(directory / "overlay_manifest.csv")
    for row in overlays:
        if not (directory / row["file"]).is_file():
            raise ValueError(f"{run_id}: missing overlay {row['file']}")
    late_fragmented = sum(int(row["fragmented"]) and float(row["decision_progress"]) >= 0.75 for row in rows)
    barrier_count = sum(row["kind"] == "false_barrier" for row in overlays)
    if late_fragmented != barrier_count:
        raise ValueError(f"{run_id}: late-fragmentation/overlay mismatch")
    return manifest


def preflight_repo(repo: Path):
    head = git(repo, "rev-parse", "HEAD")
    subprocess.run(["git", "merge-base", "--is-ancestor", topology.ACCEPTED_BASE_SHA, head],
                   cwd=repo, check=True)
    changed = set(filter(None, git(repo, "diff", "--name-only", f"{topology.ACCEPTED_BASE_SHA}..HEAD").splitlines()))
    unexpected = sorted(changed - ALLOWED_FILES)
    if unexpected:
        raise ValueError(f"implementation exceeds allowed surface: {unexpected}")
    dirty = git(repo, "status", "--short", "--untracked-files=no")
    if dirty:
        raise ValueError(f"tracked working tree is dirty: {dirty}")
    return head, sorted(changed)


def run_tests(repo: Path, log_path: Path):
    command = [sys.executable, "-m", "unittest",
               "mapex_lab.analysis.r004.test_evaluate_topology_traversability",
               "mapex_lab.analysis.r004.test_evaluate_prediction_vs_final_observed",
               "mapex_lab.analysis.r004.test_evaluate_free_error_diagnostic"]
    completed = subprocess.run(command, cwd=repo, text=True, capture_output=True)
    with log_path.open("a", encoding="utf-8") as stream:
        stream.write(completed.stdout)
        stream.write(completed.stderr)
    if completed.returncode:
        raise RuntimeError("focused/base test suite failed")
    match = re.search(r"Ran (\d+) tests", completed.stderr)
    if not match:
        raise RuntimeError("test suite passed but test count was unavailable")
    return {"command": command, "tests_run": int(match.group(1)), "status": "PASS"}


def initial_state(repo, data_root, output_root, implementation_sha):
    return {"schema": "r004_topology_batch_state_v1", "method_id": METHOD_ID,
            "accepted_base_sha": topology.ACCEPTED_BASE_SHA,
            "implementation_sha": implementation_sha, "hostname": socket.gethostname(),
            "repo_path": str(repo), "data_root": str(data_root), "output_root": str(output_root),
            "run_order": list(topology.base.RUNS), "completed_validated_runs": [],
            "next_incomplete_run": topology.base.RUNS[0], "active_run": None,
            "failed_run": None, "aggregation_status": "pending", "status": "running",
            "updated_at": utc_now()}


def save_state(path: Path, state, **updates):
    state.update(updates, updated_at=utc_now())
    topology.atomic_json(path, state)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--output-root", type=Path)
    args = parser.parse_args()
    repo = args.repo.resolve()
    data_root = (args.data_root or repo).resolve()
    output_root = (args.output_root or repo / "mapex_lab/analysis/r004/results/topology_traversability_v1").resolve()
    if output_root.exists() and any(output_root.iterdir()) and not (output_root / "batch_state.json").is_file():
        raise ValueError(f"refusing non-empty output root without batch state: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    log_path = output_root / "execution.log"
    state_path = output_root / "batch_state.json"
    state = None
    try:
        implementation_sha, changed = preflight_repo(repo)
        append_log(log_path, f"preflight identity accepted_base={topology.ACCEPTED_BASE_SHA} implementation={implementation_sha}")
        preflight = topology.preflight(data_root)
        preflight.update({"hostname": socket.gethostname(), "repo_path": str(repo),
                          "output_root": str(output_root), "implementation_sha": implementation_sha,
                          "implementation_files": changed})
        topology.atomic_json(output_root / "preflight.json", preflight)
        if state_path.exists():
            state = json.loads(state_path.read_text(encoding="utf-8"))
            for key, expected in (("accepted_base_sha", topology.ACCEPTED_BASE_SHA),
                                  ("implementation_sha", implementation_sha),
                                  ("output_root", str(output_root))):
                if state.get(key) != expected:
                    raise ValueError(f"resume state mismatch: {key}")
        else:
            state = initial_state(repo, data_root, output_root, implementation_sha)
            topology.atomic_json(state_path, state)
        tests = run_tests(repo, log_path)
        topology.atomic_json(output_root / "test_summary.json", tests)
        append_log(log_path, "focused/base tests PASS")

        completed = []
        for run_id in topology.base.RUNS:
            final_dir = output_root / "runs" / run_id
            tmp_dir = output_root / "runs" / f"{run_id}.tmp"
            input_decisions = len(topology.base.read_csv(
                data_root / "mapex_lab/experiments/mapex" / run_id / "decisions.csv"))
            if final_dir.exists():
                verify_run(final_dir, run_id, input_decisions)
                completed.append(run_id)
                continue
            if tmp_dir.exists():
                shutil.rmtree(tmp_dir)
            tmp_dir.mkdir(parents=True)
            save_state(state_path, state, active_run=run_id, failed_run=None,
                       completed_validated_runs=completed, next_incomplete_run=run_id)
            append_log(log_path, f"{run_id} START decisions={input_decisions}")
            result = topology.analyze_run(data_root, run_id, tmp_dir)
            for name in RUN_FILES:
                if not (tmp_dir / name).is_file():
                    raise ValueError(f"{run_id}: missing required output {name}")
            output_hashes = inventory(tmp_dir)
            manifest = {"schema": "r004_topology_run_completion_v1", "status": "complete",
                        "method_id": METHOD_ID, "accepted_base_sha": topology.ACCEPTED_BASE_SHA,
                        "implementation_sha": implementation_sha, "run_id": run_id,
                        "run_path": result["run_path"], "decision_count": result["decision_count"],
                        "source_available_count": result["source_available_count"],
                        "source_unavailable_count": result["source_unavailable_count"],
                        "footprint_valid": True, "resolution_valid": True,
                        "input_fingerprints": result["input_fingerprints"],
                        "output_inventory": sorted(output_hashes), "output_hashes": output_hashes,
                        "validation": "PASS", "completed_at": utc_now()}
            topology.atomic_json(tmp_dir / "completion_manifest.json", manifest)
            verify_run(tmp_dir, run_id, input_decisions)
            final_dir.parent.mkdir(parents=True, exist_ok=True)
            os.replace(tmp_dir, final_dir)
            completed.append(run_id)
            next_run = topology.base.RUNS[len(completed)] if len(completed) < len(topology.base.RUNS) else None
            save_state(state_path, state, active_run=None, completed_validated_runs=completed,
                       next_incomplete_run=next_run)
            append_log(log_path, f"{run_id} COMPLETE source={result['source_available_count']}/{result['decision_count']}")
            if run_id == "mpx_001":
                append_log(log_path, "mpx_001 automatic smoke gate PASS")

        save_state(state_path, state, aggregation_status="running", active_run=None,
                   completed_validated_runs=completed, next_incomplete_run=None)
        summary = topology.aggregate(output_root)
        for name in FINAL_FILES:
            if not (output_root / name).is_file():
                raise ValueError(f"missing final output: {name}")
        if summary["run_count"] != len(topology.base.RUNS):
            raise ValueError("final run count mismatch")
        tmp_dirs = list((output_root / "runs").glob("*.tmp"))
        if tmp_dirs:
            raise ValueError(f"unresolved temporary run directories: {tmp_dirs}")
        completion = {"schema": "r004_topology_completion_summary_v1", "status": "complete",
                      "hostname": socket.gethostname(), "repo_path": str(repo),
                      "data_root": str(data_root), "output_root": str(output_root),
                      "accepted_base_sha": topology.ACCEPTED_BASE_SHA,
                      "implementation_sha": implementation_sha, "branch": git(repo, "branch", "--show-current"),
                      "completed_runs": completed, "decision_count": summary["decision_count"],
                      "source_available_count": summary["source_available_count"],
                      "source_unavailable_count": summary["source_unavailable_count"],
                      "artifact_manifest": "artifact_manifest.json", "completed_at": utc_now()}
        topology.atomic_json(output_root / "completion_summary.json", completion)
        save_state(state_path, state, aggregation_status="complete", status="complete",
                   active_run=None, failed_run=None)
        append_log(log_path, "batch COMPLETE")
        final_inventory = inventory(output_root, excluded={"artifact_manifest.json"})
        artifact_manifest = {"schema": "r004_topology_artifact_manifest_v1",
                             "accepted_base_sha": topology.ACCEPTED_BASE_SHA,
                             "implementation_sha": implementation_sha,
                             "artifact_count": len(final_inventory), "hashes": final_inventory,
                             "generated_at": utc_now()}
        topology.atomic_json(output_root / "artifact_manifest.json", artifact_manifest)
    except Exception as error:
        append_log(log_path, f"FAIL {type(error).__name__}: {error}")
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(traceback.format_exc())
        if state is not None:
            save_state(state_path, state, status="failed", aggregation_status=state.get("aggregation_status", "pending"),
                       failed_run=state.get("active_run"))
        raise


if __name__ == "__main__":
    main()
