#!/usr/bin/env python3
"""Fail-closed MX060 acquisition-only recorder and launch guards."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from mx060_contract import (
    COHORT_ID, LAYOUT_SEEDS, RUN_SEEDS, acquisition_only_allows, metadata_only,
    retry_disposition, run_id, sha256_file, validate_decision_sequence,
    write_canonical_json,
)

MANIFEST_SCHEMA = "MX060_COM1_PILOT_MANIFEST_V2"
READY_CLASSIFICATION = "MX060_COM1_PILOT_PREFLIGHT_READY_FOR_COLLECTION"
MANDATORY_PREDICTIONS = ("G1", "G2", "G3", "ensemble_mean", "ensemble_variance")
MANDATORY_MAPS = ("observed_raw", "observed_fixed_canvas")


def _load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _atomic_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            data = json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii") + b"\n"
            stream.write(data); stream.flush(); os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try: os.unlink(temporary)
        except FileNotFoundError: pass
        raise


def machine_id_sha256() -> str:
    raw = Path("/etc/machine-id").read_text(encoding="utf-8").strip().encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def verify_clean(repo_root: Path) -> None:
    result = subprocess.run(
        ["git", "-C", str(repo_root), "status", "--porcelain", "--untracked-files=no"],
        check=True, text=True, stdout=subprocess.PIPE,
    )
    if result.stdout.strip():
        raise RuntimeError("MX060_DIRTY_SCIENTIFIC_WORKTREE")


def verify_launch(repo_root: Path, manifest_path: Path, ready_path: Path) -> dict[str, Any]:
    verify_clean(repo_root)
    manifest = _load(manifest_path)
    ready = _load(ready_path)
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("manifest_state") != "FROZEN_PRE_OUTCOME":
        raise RuntimeError("MX060_MANIFEST_NOT_FROZEN")
    if ready.get("classification") != READY_CLASSIFICATION:
        raise RuntimeError("MX060_COLLECTION_BLOCKED_NO_READY_TOKEN")
    if ready.get("pilot_manifest_sha256") != sha256_file(manifest_path):
        raise RuntimeError("MX060_MANIFEST_MUTATION_DETECTED")
    binding_path = repo_root / manifest["com1_binding_path"]
    if sha256_file(binding_path) != manifest["com1_binding_sha256"]:
        raise RuntimeError("MX060_COM1_BINDING_MUTATION_DETECTED")
    binding = _load(binding_path)
    if binding.get("authorized_machine_label") != "COM1" or binding.get("machine_id_sha256") != machine_id_sha256():
        raise RuntimeError("MX060_COM1_BINDING_MISMATCH")
    if manifest.get("acquisition_only") is not True:
        raise RuntimeError("MX060_ACQUISITION_ONLY_REQUIRED")
    return manifest


def verify_run_slot(manifest: dict[str, Any], layout_seed: int, run_seed: int) -> dict[str, Any]:
    if layout_seed not in LAYOUT_SEEDS or run_seed not in RUN_SEEDS:
        raise RuntimeError("MX060_SEED_DOMAIN_VIOLATION")
    expected = run_id(layout_seed, run_seed)
    matches = [row for row in manifest.get("runs", []) if row.get("run_id") == expected]
    if len(matches) != 1:
        raise RuntimeError("MX060_RUN_SLOT_NOT_FROZEN")
    return matches[0]


def validate_bundle(bundle: dict[str, Any], files_root: Path) -> dict[str, Any]:
    required = {
        "decision_id", "ordinal", "predecessor_decision_id", "successor_decision_id",
        "decision_exploration_time_s", "clock_source", "recorder_use_sim_time",
        "cumulative_path_m", "odometry_topic", "odometry_frame", "maps", "predictions",
        "r_b1_source", "f1_source", "f2_source", "f4_source", "runtime_current_only",
    }
    if required - bundle.keys():
        raise RuntimeError("MX060_DECISION_BUNDLE_INCOMPLETE")
    if bundle["clock_source"] != "ROS_/clock" or bundle["recorder_use_sim_time"] is not True:
        raise RuntimeError("MX060_SCIENTIFIC_CLOCK_PROVENANCE_INVALID")
    if bundle.get("contains_evaluator_only_values") is not False:
        raise RuntimeError("MX060_FUTURE_LEAKAGE_OR_TARGET_EXPOSURE")
    for group, names in (("maps", MANDATORY_MAPS), ("predictions", MANDATORY_PREDICTIONS)):
        if set(bundle[group]) != set(names):
            raise RuntimeError("MX060_DECISION_BUNDLE_INCOMPLETE")
        for name in names:
            record = bundle[group][name]
            path = files_root / record["path"]
            if not path.is_file() or sha256_file(path) != record["sha256"]:
                raise RuntimeError("MX060_DECISION_BUNDLE_HASH_MISMATCH")
    return bundle


def freeze_sequence_manifest(path: Path, bundles: list[dict[str, Any]]) -> None:
    validate_decision_sequence(bundles)
    _atomic_json(path, {
        "schema_version": "MX060_ACCEPTED_DECISION_SEQUENCE_V1",
        "cohort_id": COHORT_ID,
        "decisions": bundles,
        "target_values_materialized": False,
    })


def content_neutral_inventory(paths: list[Path], root: Path) -> dict[str, Any]:
    rows = []
    for path in paths:
        relative = path.resolve().relative_to(root.resolve()).as_posix()
        rows.append({"path": relative, "bytes": path.stat().st_size, "sha256": sha256_file(path)})
    result = {"schema_version": "MX060_CONTENT_NEUTRAL_INTEGRITY_V1", "files": rows}
    metadata_only(result)
    return result


def reserve_activation(original_manifest: Path, compatibility: Path, lost_seed: int,
                       reason: str, attempts: list[str], retries_used: int,
                       output: Path) -> None:
    if output.exists():
        raise RuntimeError("MX060_RESERVE_ACTIVATION_APPEND_ONLY")
    if retries_used > 2 or len(attempts) < 2:
        raise RuntimeError("MX060_RESERVE_ACTIVATION_INVALID")
    comp = _load(compatibility)
    entry = comp.get("lost_layout_entries", {}).get(str(lost_seed))
    if not entry or entry.get("status") != "PASS" or entry.get("reserve_seed") != 60004:
        raise RuntimeError("MX060_PILOT_TECHNICAL_INSUFFICIENT")
    write_canonical_json(output, {
        "schema_version": "MX060_RESERVE_ACTIVATION_V1",
        "original_manifest_path": original_manifest.as_posix(),
        "original_manifest_sha256": sha256_file(original_manifest),
        "compatibility_path": compatibility.as_posix(),
        "compatibility_sha256": sha256_file(compatibility),
        "excluded_layout_seed": lost_seed, "technical_failure_reason": reason,
        "quarantined_attempts": attempts, "pilot_global_retry_count": retries_used,
        "reserve_seed": 60004, "reserve_run_ids": [run_id(60004, 1), run_id(60004, 2)],
    })


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--ready-token", type=Path, required=True)
    parser.add_argument("--layout-seed", type=int, required=True)
    parser.add_argument("--run-seed", type=int, required=True)
    parser.add_argument("--action", default="ordinary_exploration")
    args = parser.parse_args()
    if not acquisition_only_allows(args.action):
        raise RuntimeError("MX060_ACQUISITION_ONLY_SUPPRESSION")
    manifest = verify_launch(args.repo_root.resolve(), args.manifest.resolve(), args.ready_token.resolve())
    print(json.dumps(verify_run_slot(manifest, args.layout_seed, args.run_seed), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

