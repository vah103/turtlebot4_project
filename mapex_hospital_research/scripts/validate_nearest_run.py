#!/usr/bin/env python3
"""Validate that a Hospital Nearest run is complete enough for offline research."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[1]
CANVAS_SHAPE = (2123, 1504)


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="nearest_pilot_006")
    parser.add_argument("--allow-running", action="store_true")
    args = parser.parse_args()

    run_dir = WORKSPACE / "experiments" / "nearest" / args.run_id
    errors: list[str] = []
    warnings: list[str] = []

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

    metadata = {}
    metadata_path = run_dir / "metadata.json"
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            errors.append(f"invalid metadata.json: {exc}")

    termination = metadata.get("termination_reason")
    if not termination and not args.allow_running:
        errors.append("run has no termination_reason; use --allow-running for live checks")

    decisions = read_csv(run_dir / "decisions.csv")
    policy_decisions = read_csv(run_dir / "policy_decisions.csv")
    candidates = read_csv(run_dir / "candidates.csv")
    snapshots = read_csv(run_dir / "snapshots.csv")

    if not decisions:
        errors.append("no selected-goal rows in decisions.csv")
    if not policy_decisions:
        errors.append("no policy decisions logged")
    if not candidates:
        errors.append("no candidate rows logged")

    policy_ids = {row.get("policy_decision_id", "") for row in policy_decisions}
    policy_ids.discard("")

    for row in decisions:
        pid = row.get("policy_decision_id", "")
        if not pid:
            errors.append(f"{row.get('decision_id')}: missing policy_decision_id")
        elif pid not in policy_ids:
            errors.append(
                f"{row.get('decision_id')}: policy_decision_id {pid} not in policy_decisions.csv"
            )
        if row.get("coverage", "") == "":
            errors.append(f"{row.get('decision_id')}: missing coverage")
        if row.get("robot_map_x", "") == "" or row.get("robot_map_y", "") == "":
            warnings.append(f"{row.get('decision_id')}: map-frame robot pose missing")

    for pid in sorted(policy_ids):
        decision_dir = run_dir / "decisions" / pid
        for name in (
            "decision.json",
            "candidates.csv",
            "observed_map_raw.npz",
            "observed_map_canvas.npz",
        ):
            if not (decision_dir / name).exists():
                errors.append(f"{pid}: missing {name}")

        raw_path = decision_dir / "observed_map_raw.npz"
        if raw_path.exists():
            try:
                with np.load(raw_path, allow_pickle=False) as data:
                    raw = data["data"]
                    for key in (
                        "resolution",
                        "width",
                        "height",
                        "origin_x",
                        "origin_y",
                        "origin_yaw",
                        "frame_id",
                        "source_stamp_s",
                    ):
                        if key not in data:
                            errors.append(f"{pid}: raw map missing field {key}")
                    if raw.ndim != 2:
                        errors.append(f"{pid}: raw map is not 2-D")
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{pid}: cannot read raw map: {exc}")

        canvas_path = decision_dir / "observed_map_canvas.npz"
        if canvas_path.exists():
            try:
                with np.load(canvas_path, allow_pickle=False) as data:
                    canvas = data["data"]
                    if tuple(canvas.shape) != CANVAS_SHAPE:
                        errors.append(
                            f"{pid}: canvas shape {canvas.shape} != {CANVAS_SHAPE}"
                        )
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{pid}: cannot read canvas map: {exc}")

    candidate_policy_ids = {row.get("policy_decision_id", "") for row in candidates}
    for pid in policy_ids:
        if pid not in candidate_policy_ids:
            errors.append(f"{pid}: no rows in global candidates.csv")

    # Hospital adaptation deliberately allows candidates below the original MapEx
    # 1 m threshold. Flag accidental reintroduction of the old rejection status.
    for row in candidates:
        if row.get("status") == "rejected_lt_1m":
            errors.append(
                f"{row.get('policy_decision_id')}: below-1m candidate was rejected; "
                "Hospital adaptation is not active"
            )

    if not snapshots:
        errors.append("snapshots.csv is empty")
    elif termination and not any(row.get("event") == "final" for row in snapshots):
        errors.append("terminated run has no final snapshot")

    config_hashes = metadata.get("config_sha256")
    if not isinstance(config_hashes, dict) or not config_hashes:
        errors.append("metadata missing config_sha256 provenance")
    elif any(value in (None, "") for value in config_hashes.values()):
        warnings.append("one or more config_sha256 entries are missing")

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
    print(f"- selected decisions: {len(decisions)}")
    print(f"- policy decisions: {len(policy_decisions)}")
    print(f"- candidate rows: {len(candidates)}")
    print(f"- periodic/final snapshots: {len(snapshots)}")
    print(f"- termination: {termination or 'RUNNING'}")
    print("- Hospital below-1m adaptation: consistent")
    if warnings:
        print("Warnings:")
        for warning in warnings:
            print(f"- {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
