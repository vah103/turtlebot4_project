#!/usr/bin/env python3
"""Ordered COM1 MX060 P1-P20 preflight; stops before generation on P1-P3 failure."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import shutil
import socket
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from mx060_contract import (
    COHORT_ID, ENSEMBLE_SHA256, MX048_CONTRACT_BLOB, MX048_CONTRACT_COMMIT,
    MX060_METHOD_BLOB, MX060_METHOD_COMMIT, MX061_METHOD_BLOB, MX061_METHOD_COMMIT,
    sha256_file, write_canonical_json,
)

HANDOFF_COMMIT = "0f7d2002dcd882e242f8584e1f6b6d77302ce65f"
HANDOFF_PATH = "company/operations/HO-20261005-MX064-PM-ENGINEER-MX060-R2-IMPLEMENT-PREFLIGHT.md"
METHOD_PATH = "company/projects/mapex/MX060_MINIMAL_COM1_NEW_EVIDENCE_COHORT_METHOD_R2.md"
MX061_PATH = "company/projects/mapex/MX061_CONTINUATION_VALUE_SEMANTICS_METHOD_R2.md"
RESULT_REL = Path("mapex_lab/analysis/mx060_preflight_results")


def run(command: list[str], check: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, check=check, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)


def git(root: Path, *args: str) -> str:
    return run(["git", "-C", str(root), *args]).stdout.strip()


def version(command: list[str]) -> str:
    try:
        return next((line.strip() for line in run(command, check=False).stdout.splitlines() if line.strip()), "UNAVAILABLE")
    except Exception as exc:
        return f"UNAVAILABLE:{type(exc).__name__}"


def machine_hash() -> str:
    return hashlib.sha256(Path("/etc/machine-id").read_text(encoding="utf-8").strip().encode()).hexdigest()


def blob_at(governance: Path, commit: str, path: str) -> str:
    return git(governance, "rev-parse", f"{commit}:{path}")


def require_ancestor(governance: Path, commit: str) -> None:
    proc = run(["git", "-C", str(governance), "merge-base", "--is-ancestor", commit, "main"], check=False)
    if proc.returncode:
        raise RuntimeError(f"MX060_AUTHORITY_NOT_ANCESTOR:{commit}")


def p1_authority(governance: Path, repo: Path) -> dict[str, Any]:
    for commit in (
        MX060_METHOD_COMMIT, "4771d974b4b768484c6d05c0090e6784c3cbf0d4",
        "7e4ba8f1747527179e73e1a230a0f661fd0e2f66",
        "d2046166bb53d49cc6c0a99648e27ec566922972", HANDOFF_COMMIT,
    ):
        require_ancestor(governance, commit)
    checks = {
        "mx060_method_blob": blob_at(governance, MX060_METHOD_COMMIT, METHOD_PATH) == MX060_METHOD_BLOB,
        "mx061_method_blob": blob_at(governance, MX061_METHOD_COMMIT, MX061_PATH) == MX061_METHOD_BLOB,
        "mx048_contract_blob": blob_at(governance, MX048_CONTRACT_COMMIT, "company/projects/mapex/MX048_MX045_SHADOW_ACQUISITION_CONTRACT_R1.md") == MX048_CONTRACT_BLOB,
        "execution_handoff_present": blob_at(governance, HANDOFF_COMMIT, HANDOFF_PATH) != "",
        "scientific_run_count_zero": not (repo / "mapex_lab/experiments/mx060_com1_pilot").exists(),
    }
    return {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks}


def _hashes(repo: Path, paths: list[str]) -> dict[str, str]:
    return {path: sha256_file(repo / path) for path in paths if (repo / path).is_file()}


def p2_binding(repo: Path, output: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    dirty = bool(git(repo, "status", "--porcelain", "--untracked-files=no"))
    free = shutil.disk_usage(repo).free
    configs = [
        "ros2_ws/src/frontier_exploration/config/hospital_slam.yaml",
        "ros2_ws/src/frontier_exploration/config/hospital_slam_no_loop.yaml",
        "ros2_ws/src/frontier_exploration/config/nav2_hospital_override.yaml",
    ]
    files = {
        "runner_blob": ".run_core",
        "recorder_blob": "mapex_lab/scripts/mapex_run.py",
        "explorer_policy_blob": "ros2_ws/src/frontier_exploration/frontier_exploration/exploration_manager.py",
        "lama_bridge_blob": "mapex_lab/scripts/mapex_lama_bridge.py",
        "lama_worker_blob": "mapex_lab/scripts/mapex_lama_worker.py",
        "generator_blob": "mapex_lab/scripts/mx060_generator.py",
        "preflight_blob": "mapex_lab/scripts/run_mx060_preflight.py",
    }
    binding: dict[str, Any] = {
        "schema_version": "MX060_COM1_BINDING_V1", "cohort_id": COHORT_ID,
        "authorized_machine_label": "COM1", "machine_id_sha256": machine_hash(),
        "hostname_runtime_only": socket.gethostname(), "os": platform.platform(),
        "kernel": platform.release(), "ros_distribution": os.environ.get("ROS_DISTRO", "UNAVAILABLE"),
        "gazebo_version": version(["gz", "sim", "--versions"]),
        "python_runtimes": {"active": sys.version.splitlines()[0], "system": version(["/usr/bin/python3", "--version"])},
        "mapex_worker_runtime": version([sys.executable, "--version"]),
        "technical_repository_commit": git(repo, "rev-parse", "HEAD"), "dirty": dirty,
        "acquisition_volume_path": str(repo.resolve()), "free_disk_bytes": free,
        "navigation_slam_config_hashes": _hashes(repo, configs),
        "created_utc": datetime.now(timezone.utc).isoformat(),
    }
    for key, relative in files.items():
        binding[key] = git(repo, "hash-object", str(repo / relative)) if (repo / relative).is_file() else "MISSING"
    write_canonical_json(output, binding)
    checks = {
        "authorized_machine_label": socket.gethostname().lower() == "com1",
        "machine_id_hashed_not_raw": len(binding["machine_id_sha256"]) == 64,
        "clean_scientific_worktree": not dirty,
        "free_disk_gte_5gb": free >= 5_000_000_000,
        "runner_recorder_generator_present": all(binding[key] != "MISSING" for key in ("runner_blob", "recorder_blob", "generator_blob")),
    }
    return binding, {"status": "PASS" if all(checks.values()) else "FAIL", "checks": checks}


def p3_ensemble(weight_paths: list[Path]) -> dict[str, Any]:
    by_hash: dict[str, str] = {}
    for path in weight_paths:
        if path.is_file():
            by_hash[sha256_file(path)] = str(path.resolve())
    members = {
        member: {"expected_sha256": expected, "path": by_hash.get(expected), "status": "PASS" if expected in by_hash else "FAIL"}
        for member, expected in ENSEMBLE_SHA256.items()
    }
    return {"status": "PASS" if all(row["status"] == "PASS" for row in members.values()) else "FAIL", "members": members}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--governance-repo", type=Path, required=True)
    parser.add_argument("--weight", type=Path, action="append", default=[])
    parser.add_argument("--result-dir", type=Path)
    args = parser.parse_args()
    repo = args.repo_root.resolve(); governance = args.governance_repo.resolve()
    result_dir = (args.result_dir or (repo / RESULT_REL)).resolve(); result_dir.mkdir(parents=True, exist_ok=True)
    report: dict[str, Any] = {
        "schema_version": "MX060_PREFLIGHT_P1_P20_V1", "cohort_id": COHORT_ID,
        "ordered_execution": True, "scientific_runs_executed": 0,
        "checks": {f"P{i}": {"status": "NOT_RUN_FAIL_CLOSED"} for i in range(1, 21)},
    }
    try:
        report["checks"]["P1"] = p1_authority(governance, repo)
        if report["checks"]["P1"]["status"] != "PASS": raise RuntimeError("MX060_P1_AUTHORITY_FAIL")
        _, report["checks"]["P2"] = p2_binding(repo, result_dir / "MX060_COM1_BINDING.json")
        if report["checks"]["P2"]["status"] != "PASS": raise RuntimeError("MX060_P2_COM1_BINDING_FAIL")
        report["checks"]["P3"] = p3_ensemble(args.weight)
        if report["checks"]["P3"]["status"] != "PASS": raise RuntimeError("MX060_P3_ENSEMBLE_IDENTITY_FAIL")
        raise RuntimeError("MX060_PREFLIGHT_IMPLEMENTATION_STOPS_BEFORE_P4_PENDING_EXPLICIT_COMPLETION")
    except Exception as exc:
        report["overall_status"] = "BLOCKED"
        report["blocker"] = str(exc)
        report["ready_token_created"] = False
        report["worlds_generated"] = False
        report["scientific_runs_executed"] = 0
        write_canonical_json(result_dir / "MX060_PREFLIGHT_P1_P20.json", report)
        note = (
            "# MX064 durable progress\n\n"
            f"- status: BLOCKED\n- blocker: `{exc}`\n"
            f"- technical commit: `{git(repo, 'rev-parse', 'HEAD')}`\n"
            "- completed: exact authority reconciliation, COM1 binding, ordered fail-closed P1-P3\n"
            "- worlds/seeds generated: none\n- scientific runs: none\n- ready token: not created\n"
            "- resume: provide exact G1/G2/G3 checkpoint files, then rerun `python3 mapex_lab/scripts/run_mx060_preflight.py --governance-repo <chat-gpt> --weight <G1> --weight <G2> --weight <G3>`\n"
        )
        (result_dir / "MX064_DURABLE_PROGRESS.md").write_text(note, encoding="utf-8")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
