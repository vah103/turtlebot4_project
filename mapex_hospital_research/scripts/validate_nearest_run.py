#!/usr/bin/env python3
"""Validate that a Hospital Nearest run is complete enough for offline research."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[1]
CANVAS_SHAPE = (2123, 1504)


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(value: str, label: str, errors: list[str]) -> float | None:
    try:
        return float(value)
    except Exception:  # noqa: BLE001
        errors.append(f"invalid numeric value for {label}: {value!r}")
        return None


def check_npz(path: Path, errors: list[str], *, canvas: bool = False) -> None:
    if not path.exists():
        errors.append(f"missing snapshot file: {path}")
        return
    try:
        with np.load(path, allow_pickle=False) as data:
            if "data" not in data:
                errors.append(f"{path}: missing data array")
                return
            arr = data["data"]
            if arr.ndim != 2:
                errors.append(f"{path}: data is not 2-D")
            if canvas and tuple(arr.shape) != CANVAS_SHAPE:
                errors.append(f"{path}: canvas shape {arr.shape} != {CANVAS_SHAPE}")
    except Exception as exc:  # noqa: BLE001
        errors.append(f"cannot read {path}: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="nearest_pilot_007")
    parser.add_argument("--allow-running", action="store_true")
    args = parser.parse_args()

    run_dir = WORKSPACE / "experiments" / "nearest" / args.run_id
    errors: list[str] = []
    warnings: list[str] = []
    official = args.run_id.startswith("nearest_") and not args.run_id.startswith(
        "nearest_pilot_"
    )

    if not run_dir.is_dir():
        print(f"NEAREST RUN VALIDATION: FAIL\n- missing run directory: {run_dir}")
        return 1

    required = [
        "metadata.json",
        "metrics.csv",
        "trajectory.csv",
        "decisions.csv",
        "policy_decisions.csv",
        "candidates.csv",
        "snapshots.csv",
    ]
    for name in required:
        if not (run_dir / name).exists():
            errors.append(f"missing {name}")

    metadata: dict = {}
    metadata_path = run_dir / "metadata.json"
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            errors.append(f"invalid metadata.json: {exc}")

    termination = metadata.get("termination_reason")
    if not termination and not args.allow_running:
        errors.append("run has no termination_reason; use --allow-running for live checks")

    if metadata.get("exploration_start_source") != "first_policy_decision_before_compute":
        errors.append("benchmark clock was not started before first policy computation")

    seed_policy = str(metadata.get("sim_seed_policy", ""))
    if "intentionally_uncontrolled_gazebo_default" not in seed_policy:
        errors.append("metadata missing explicit simulator seed policy")

    dirty = metadata.get("git_dirty_at_recorder_start")
    if official and dirty is not False:
        errors.append("official run must start from a clean git worktree")
    elif dirty:
        warnings.append("pilot started from a dirty git worktree")

    hashes = metadata.get("config_sha256")
    required_hashes = {
        "nearest_official_policy",
        "nearest_hospital_adapted",
        "official_recorder",
        "nearest_config",
        "experiment_protocol",
        "installed_hospital_flat_stack",
        "installed_hospital_slam",
        "installed_hospital_nav2_override",
        "installed_frontier_config",
        "installed_hospital_world",
    }
    if not isinstance(hashes, dict):
        errors.append("metadata missing config_sha256 provenance")
    else:
        for key in sorted(required_hashes):
            if not hashes.get(key):
                errors.append(f"metadata config_sha256 missing {key}")

    metrics = read_csv(run_dir / "metrics.csv")
    trajectory = read_csv(run_dir / "trajectory.csv")
    decisions = read_csv(run_dir / "decisions.csv")
    policy_decisions = read_csv(run_dir / "policy_decisions.csv")
    candidates = read_csv(run_dir / "candidates.csv")
    snapshots = read_csv(run_dir / "snapshots.csv")

    if not metrics:
        errors.append("metrics.csv has no samples")
    prev_t = -math.inf
    prev_d = -math.inf
    for index, row in enumerate(metrics, start=1):
        t = as_float(row.get("time_s", ""), f"metrics row {index} time_s", errors)
        d = as_float(row.get("distance_m", ""), f"metrics row {index} distance_m", errors)
        c = as_float(row.get("coverage", ""), f"metrics row {index} coverage", errors)
        k = as_float(
            row.get("known_fraction", ""),
            f"metrics row {index} known_fraction",
            errors,
        )
        if t is not None:
            if t + 1e-9 < prev_t:
                errors.append(f"metrics time is not monotonic at row {index}")
            prev_t = t
        if d is not None:
            if d + 1e-6 < prev_d:
                errors.append(f"metrics cumulative distance decreased at row {index}")
            prev_d = d
        if c is not None and not 0.0 <= c <= 1.0:
            errors.append(f"coverage outside [0,1] at row {index}: {c}")
        if k is not None and not 0.0 <= k <= 1.0:
            errors.append(f"known_fraction outside [0,1] at row {index}: {k}")

    if not trajectory:
        errors.append("trajectory.csv has no samples")

    if not policy_decisions:
        errors.append("no policy decisions logged")

    policy_by_id = {
        row.get("policy_decision_id", ""): row
        for row in policy_decisions
        if row.get("policy_decision_id", "")
    }
    candidate_groups: dict[str, list[dict[str, str]]] = {}
    for row in candidates:
        pid = row.get("policy_decision_id", "")
        candidate_groups.setdefault(pid, []).append(row)
        if not row.get("candidate_id", ""):
            errors.append(f"{pid}: candidate row missing candidate_id")
        if row.get("status") == "rejected_lt_1m":
            errors.append(f"{pid}: below-1m candidate was incorrectly rejected")

    for pid, row in policy_by_id.items():
        decision_dir = run_dir / "decisions" / pid
        required_decision_files = [
            decision_dir / "decision.json",
            decision_dir / "candidates.csv",
            decision_dir / "observed_map_raw.npz",
            decision_dir / "observed_map_canvas.npz",
        ]
        for path in required_decision_files:
            if not path.exists():
                errors.append(f"{pid}: missing {path.name}")
        check_npz(decision_dir / "observed_map_raw.npz", errors)
        check_npz(
            decision_dir / "observed_map_canvas.npz", errors, canvas=True
        )

        count = int(float(row.get("candidate_count", "0") or 0))
        if count > 0 and not candidate_groups.get(pid):
            errors.append(f"{pid}: candidate_count={count} but no candidate rows")

    for row in decisions:
        did = row.get("decision_id", "")
        pid = row.get("policy_decision_id", "")
        cid = row.get("selected_candidate_id", "")
        if not pid or pid not in policy_by_id:
            errors.append(f"{did}: missing/unknown policy_decision_id")
        if not cid:
            errors.append(f"{did}: missing selected_candidate_id")
        elif pid:
            matches = [
                item
                for item in candidate_groups.get(pid, [])
                if item.get("candidate_id") == cid
            ]
            if not matches:
                errors.append(f"{did}: selected candidate {pid}/{cid} not found")
            elif not any(str(item.get("selected", "")).lower() == "true" for item in matches):
                errors.append(f"{did}: linked candidate is not marked selected")
        if row.get("coverage", "") == "":
            errors.append(f"{did}: missing coverage")
        if row.get("robot_map_x", "") == "" or row.get("robot_map_y", "") == "":
            errors.append(f"{did}: missing map-frame robot pose")
        if termination and row.get("result") == "PENDING":
            errors.append(f"{did}: terminated run still has PENDING result")
        if row.get("result") == "FAILED" and not row.get("failure_reason", ""):
            errors.append(f"{did}: FAILED goal missing failure_reason")

    if not snapshots:
        errors.append("snapshots.csv is empty")
    for row in snapshots:
        raw_rel = row.get("raw_map_file", "")
        canvas_rel = row.get("canvas_map_file", "")
        if not raw_rel or not canvas_rel:
            errors.append(f"snapshot {row.get('snapshot_id')}: missing file reference")
            continue
        check_npz(run_dir / raw_rel, errors)
        check_npz(run_dir / canvas_rel, errors, canvas=True)

    if termination and not any(row.get("event") == "final" for row in snapshots):
        errors.append("terminated run has no final recorder snapshot")

    if termination and policy_decisions:
        terminal_outcomes = {
            "exhausted_no_ranked_candidate",
            "no_nav2_reachable_ranked_candidate",
        }
        if not any(row.get("outcome") in terminal_outcomes for row in policy_decisions):
            warnings.append(
                "terminated run has no explicit exhausted/no-Nav2-reachable policy state; "
                "inspect termination_reason if this was an interruption/policy error"
            )

    if errors:
        print("NEAREST RUN VALIDATION: FAIL")
        for error in errors:
            print(f"- {error}")
        if warnings:
            print("Warnings:")
            for warning in warnings:
                print(f"- {warning}")
        return 1

    print("NEAREST RUN VALIDATION: PASS")
    print(f"- metrics samples: {len(metrics)}")
    print(f"- selected decisions: {len(decisions)}")
    print(f"- policy decisions: {len(policy_decisions)}")
    print(f"- candidate rows: {len(candidates)}")
    print(f"- periodic/final snapshots: {len(snapshots)}")
    print(f"- termination: {termination or 'RUNNING'}")
    print("- benchmark t=0: pre-compute")
    print("- Hospital below-1m adaptation: consistent")
    print("- candidate linkage + snapshot integrity: OK")
    print("- provenance: OK")
    if warnings:
        print("Warnings:")
        for warning in warnings:
            print(f"- {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
