#!/usr/bin/env python3
"""Resume-safe sequential driver for the four prospective MX029 Hospital runs."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import signal
import shutil
import subprocess
import sys
import time
from pathlib import Path


RUN_IDS = tuple(f"hpx_{index:03d}" for index in range(2, 6))
GT_SHA256 = "080c7d708f12ae71c1ed1881bfd630dbb491401d95831f613e3116cb9926dce1"
GT_SEMANTIC_DIGEST = "d27ba692b313ba784729ea1fd10b2963c20fea9e4465d27081ece9c8757cf4ea"
ROI_SHA256 = "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"
HEADLESS = True
USE_RVIZ = True
MAP_STALL_TIMEOUT_S = 180.0
MAP_STALL_MIN_DECISIONS = 5


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def git(root: Path, *args: str) -> str:
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).strip()


def frozen_identity(root: Path) -> dict:
    lab = root / "mapex_lab"
    gt_dir = lab / "analysis/d1/results/mx018_hospital_gt_recovery_v1/gt_v2"
    paths = {
        "hospital_gt_v2": gt_dir / "hospital_structural_gt_v2.npz",
        "hospital_gt_v2_roi": gt_dir / "hospital_connected_free_v2.npy",
        "hospital_gt_v2_semantic": gt_dir / "gt_v2_semantic_manifest.canonical.json",
        "mapex_run": lab / "scripts/mapex_run.py",
        "mapex_policy": lab / "scripts/mapex.py",
        "nf_shared_execution": lab / "scripts/nf_basic.py",
        "mapex_config": lab / "config/mapex.yaml",
        "nav2_override": lab / "config/nav2.yaml",
        "runtime_launch": lab / "launch/slam.launch.py",
        "hospital_scale": lab / "scripts/hospital_scale.py",
        "hospital_world": lab / "map/hospital_aws_flat.sdf",
        "execution_runner": root / ".run_core",
        "batch_driver": lab / "scripts/run_mx029_hospital_batch.py",
    }
    identity = {
        "source_revision": git(root, "rev-parse", "HEAD"),
        "source_branch": git(root, "branch", "--show-current"),
        "configuration": {name: sha256(path) for name, path in paths.items()},
        "gt_v2": {
            "sha256": GT_SHA256,
            "semantic_digest": GT_SEMANTIC_DIGEST,
            "roi_sha256": ROI_SHA256,
            "binding_timing": "before_execution",
        },
        "run_ids": list(RUN_IDS),
        "provenance_label": "PROSPECTIVE_GT_V2",
        "experimental_early_stop_enabled": False,
        "execution_mode": {
            "headless": HEADLESS,
            "use_rviz": USE_RVIZ,
            "map_stall_timeout_s": MAP_STALL_TIMEOUT_S,
            "map_stall_min_decisions": MAP_STALL_MIN_DECISIONS,
        },
        "sim_seed": None,
        "sim_seed_policy": "intentionally_uncontrolled_gazebo_default_multiple_run_statistics",
    }
    if identity["configuration"]["hospital_gt_v2"] != GT_SHA256:
        raise RuntimeError("Hospital GT-v2 mismatch")
    if identity["configuration"]["hospital_gt_v2_roi"] != ROI_SHA256:
        raise RuntimeError("Hospital GT-v2 ROI mismatch")
    if identity["configuration"]["hospital_gt_v2_semantic"] != GT_SEMANTIC_DIGEST:
        raise RuntimeError("Hospital GT-v2 semantic mismatch")
    return identity


def inventory_run(run: Path, frozen: dict) -> dict:
    metadata = load_json(run / "metadata.json")
    summary = load_json(run / "summary.json")
    required = {
        "metadata.json", "summary.json", "decisions.csv", "policy_decisions.csv",
        "candidates.csv", "trajectory.csv", "metrics.csv", "snapshots.csv",
        "goals.csv", "plans.csv", "runtime_mapex.yaml", "runtime_nav2_merged.yaml",
    }
    files = {str(path.relative_to(run)) for path in run.rglob("*") if path.is_file()}
    missing = sorted(required - files)
    decision_count = int(metadata.get("policy_decisions") or 0)
    prediction_counts = {
        suffix: len(list((run / "predictions").glob(f"decision_*_{suffix}.npz")))
        for suffix in ("g1", "g2", "g3", "mean", "variance")
    }
    valid = (
        summary.get("termination_reason") == "exploration_complete"
        and metadata.get("termination_reason") == "exploration_complete"
        and metadata.get("git_commit") == frozen["source_revision"]
        and metadata.get("structural_ground_truth_sha256") == GT_SHA256
        and metadata.get("structural_ground_truth_semantic_digest") == GT_SEMANTIC_DIGEST
        and metadata.get("evaluation_roi_sha256") == ROI_SHA256
        and metadata.get("ground_truth_binding_timing") == "before_execution"
        and metadata.get("cohort_provenance_label") == "PROSPECTIVE_GT_V2"
        and metadata.get("experimental_early_stop_enabled") is False
        and metadata.get("execution_headless") is HEADLESS
        and metadata.get("execution_use_rviz") is USE_RVIZ
        and decision_count > 0
        and not missing
        and all(count == decision_count for count in prediction_counts.values())
    )
    return {
        "status": "COMPLETED" if valid else "TECHNICAL_ABORT",
        "termination_reason": summary.get("termination_reason"),
        "decision_count": decision_count,
        "missing_required_files": missing,
        "prediction_counts": prediction_counts,
        "artifact_count": len(files),
        "metadata_sha256": sha256(run / "metadata.json"),
        "summary_sha256": sha256(run / "summary.json"),
    }


def archive_abort(run: Path, evidence: Path, run_id: str, attempt: int) -> Path:
    target = evidence / "technical_aborts" / f"{run_id}_attempt{attempt:02d}"
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise RuntimeError(f"abort archive already exists: {target}")
    shutil.move(str(run), str(target))
    return target


def read_decision_progress(path: Path) -> tuple[int, str | None]:
    if not path.is_file():
        return 0, None
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        return 0, None
    return len(rows), rows[-1].get("map_generation")


def stop_runner(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=45)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=15)
        return
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait()


def execute_run(command: list[str], root: Path, run: Path, stream) -> tuple[int, str | None]:
    env = os.environ.copy()
    env["MAPEX_RUN_HEADLESS"] = "True" if HEADLESS else "False"
    env["MAPEX_RUN_USE_RVIZ"] = "True" if USE_RVIZ else "False"
    process = subprocess.Popen(
        command,
        cwd=root,
        env=env,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    observed_generation = None
    generation_changed_at = time.monotonic()
    decisions_at_change = 0
    abort_reason = None
    try:
        while process.poll() is None:
            time.sleep(10)
            count, generation = read_decision_progress(run / "decisions.csv")
            if generation is None:
                continue
            if generation != observed_generation:
                observed_generation = generation
                generation_changed_at = time.monotonic()
                decisions_at_change = count
                continue
            if (
                count - decisions_at_change >= MAP_STALL_MIN_DECISIONS
                and time.monotonic() - generation_changed_at >= MAP_STALL_TIMEOUT_S
            ):
                abort_reason = (
                    f"map_generation_stalled:{generation}:"
                    f"decisions={count - decisions_at_change}:"
                    f"wall_s={time.monotonic() - generation_changed_at:.1f}"
                )
                stop_runner(process)
                break
    except BaseException:
        stop_runner(process)
        raise
    return process.wait(), abort_reason


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--max-attempts", type=int, default=2)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--roll-forward-zero-completed", action="store_true")
    parser.add_argument("--stop-after-run-id", choices=RUN_IDS)
    args = parser.parse_args()
    root = args.repo_root.resolve()
    evidence = root / "mapex_lab/analysis/d1/results/mx029_hospital_5run"
    state_path = evidence / "run_manifest.json"
    preflight = load_json(evidence / "preflight.json")
    if preflight.get("verdict") != "HPX001_DATA_REUSE_PASS":
        raise RuntimeError("MX029 batch requires HPX001_DATA_REUSE_PASS")
    frozen = frozen_identity(root)
    state = load_json(state_path) if state_path.is_file() else {
        "schema": "mx029_hospital_batch_state_v1",
        "task": "MX029",
        "preflight_verdict": preflight["verdict"],
        "legacy_run": {"run_id": "hpx_001", "label": "LEGACY_RETROSPECTIVE_GT_V2"},
        "frozen": frozen,
        "runs": {run_id: {"status": "PENDING", "attempts": 0} for run_id in RUN_IDS},
        "technical_aborts": [],
    }
    if state["frozen"] != frozen:
        completed = [run_id for run_id, record in state["runs"].items() if record["status"] == "COMPLETED"]
        if not args.roll_forward_zero_completed or completed:
            raise RuntimeError(
                "frozen MX029 identity changed after cohort start; "
                f"completed={completed or 'none'}"
            )
        state.setdefault("superseded_frozen_identities", []).append(
            {
                "frozen": state["frozen"],
                "superseded_at_unix": time.time(),
                "reason": "technical_aborts_before_any_completed_prospective_run",
            }
        )
        state["frozen"] = frozen
        for record in state["runs"].values():
            record["status"] = "PENDING"
    atomic_json(state_path, state)
    if args.prepare_only:
        print(json.dumps(state, indent=2, sort_keys=True))
        return 0

    for run_id in RUN_IDS:
        record = state["runs"][run_id]
        if record["status"] == "COMPLETED":
            if args.stop_after_run_id == run_id:
                state["status"] = "STOPPED_AFTER_REQUESTED_RUN"
                state["stopped_after_run_id"] = run_id
                atomic_json(state_path, state)
                return 0
            continue
        run = root / "mapex_lab/experiments/mapex" / run_id
        if run.exists() and record["status"] in {"RUNNING", "PENDING_RETRY"}:
            recovered = inventory_run(run, frozen)
            if recovered["status"] == "COMPLETED":
                record.update(status="COMPLETED", inventory=recovered, recovered_after_interruption=True)
                atomic_json(state_path, state)
                continue
            attempt = max(1, int(record["attempts"]))
            archive = archive_abort(run, evidence, run_id, attempt)
            abort = {
                "run_id": run_id, "attempt": attempt, "returncode": None,
                "inventory": recovered, "log": record.get("log"),
                "archived_run": str(archive.relative_to(root)),
                "classification": "INTERRUPTED_OR_AMBIGUOUS_RECOVERY",
            }
            state["technical_aborts"].append(abort)
            record.update(status="PENDING_RETRY", last_abort=abort)
            atomic_json(state_path, state)
        while record["attempts"] < args.max_attempts:
            record["attempts"] += 1
            attempt = record["attempts"]
            run = root / "mapex_lab/experiments/mapex" / run_id
            if run.exists():
                raise RuntimeError(f"unclassified run directory exists: {run}")
            log = evidence / "logs" / f"{run_id}_attempt{attempt:02d}.log"
            log.parent.mkdir(parents=True, exist_ok=True)
            record.update(status="RUNNING", started_at_unix=time.time(), log=str(log.relative_to(root)))
            atomic_json(state_path, state)
            command = [
                str(root / "run"), "--method", "mapex", "--environment", "hospital",
                "--record", "yes", "--run-id", run_id,
            ]
            with log.open("wb") as stream:
                returncode, monitor_abort_reason = execute_run(command, root, run, stream)
            inventory = inventory_run(run, frozen) if run.is_dir() else {
                "status": "TECHNICAL_ABORT", "termination_reason": "run_directory_missing"
            }
            record.update(
                status=inventory["status"], returncode=returncode,
                completed_at_unix=time.time(), inventory=inventory,
            )
            if inventory["status"] == "COMPLETED" and returncode == 0:
                atomic_json(state_path, state)
                break
            archive = archive_abort(run, evidence, run_id, attempt) if run.is_dir() else None
            abort = {
                "run_id": run_id, "attempt": attempt, "returncode": returncode,
                "inventory": inventory, "log": str(log.relative_to(root)),
                "archived_run": None if archive is None else str(archive.relative_to(root)),
                "monitor_abort_reason": monitor_abort_reason,
            }
            state["technical_aborts"].append(abort)
            record.update(status="PENDING_RETRY", last_abort=abort)
            atomic_json(state_path, state)
        if record["status"] != "COMPLETED":
            return 1
        if args.stop_after_run_id == run_id:
            state["status"] = "STOPPED_AFTER_REQUESTED_RUN"
            state["stopped_after_run_id"] = run_id
            atomic_json(state_path, state)
            print(json.dumps(state, indent=2, sort_keys=True))
            return 0
    state["status"] = "COLLECTION_COMPLETE"
    state["completed_at_unix"] = time.time()
    atomic_json(state_path, state)
    print(json.dumps(state, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
