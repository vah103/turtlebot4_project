#!/usr/bin/env python3
"""Resume-safe driver for the 20 official R003 paper500 runs."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time


WATCHDOG_EXIT_CODE = 42
TERMINAL_BLOCKED_STATUSES = {
    "blocked_repeated_technical_invalidity",
    "blocked_ambiguous_failure",
    "blocked_postprocess_failure",
    "blocked_manual_technical_invalidity",
    "recovery_required",
}


def official_matrix() -> list[dict[str, str]]:
    return [
        {"method": method, "run_id": f"{prefix}_p500_{index:03d}"}
        for method, prefix in (("nf", "nf"), ("mapex", "mpx"))
        for index in range(1, 11)
    ]


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def source_sha(root: Path) -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=root, check=True,
        capture_output=True, text=True,
    ).stdout.strip()


def effective_ros_env(base: dict[str, str] | None = None) -> dict[str, str]:
    env = dict(os.environ if base is None else base)
    env["ROS_DOMAIN_ID"] = env.get("MAPEX_SIM_ROS_DOMAIN_ID", "42")
    env["ROS_LOCALHOST_ONLY"] = env.get("MAPEX_SIM_LOCALHOST_ONLY", "1")
    return env


def resume_blocker(record: dict) -> str | None:
    status = record.get("status")
    if status in {None, "complete", "retry_pending_same_id"}:
        return None
    if status in TERMINAL_BLOCKED_STATUSES:
        return str(status)
    if status in {"running", "technical_invalid"}:
        return "recovery_required"
    return "recovery_required"


def invalid_deletion_decision(attempt: int, max_technical_retries: int) -> str:
    if int(attempt) <= int(max_technical_retries):
        return "delete_exact_run_dir_then_retry_same_official_id"
    return "delete_exact_run_dir_then_block_repeated_invalidity"


def first_unfinished(matrix: list[dict], state: dict) -> dict | None:
    complete = {
        run_id for run_id, item in state.get("runs", {}).items()
        if item.get("status") == "complete"
    }
    return next((item for item in matrix if item["run_id"] not in complete), None)


def run_dir(root: Path, item: dict) -> Path:
    method_dir = "nearest" if item["method"] == "nf" else "mapex"
    return root / "mapex_lab" / "experiments" / method_dir / item["run_id"]


def delete_invalid_run(path: Path, root: Path, expected_id: str) -> None:
    allowed = {
        (root / "mapex_lab" / "experiments" / "nearest").resolve(),
        (root / "mapex_lab" / "experiments" / "mapex").resolve(),
    }
    resolved_parent = path.parent.resolve()
    if resolved_parent not in allowed or path.name != expected_id or "_p500_" not in path.name:
        raise RuntimeError(f"refusing unsafe invalid-run deletion: {path}")
    if path.exists():
        shutil.rmtree(path)


def run_one(
    root: Path, item: dict, attempt: int, evidence_dir: Path,
) -> tuple[str, Path, Path]:
    run_id = item["run_id"]
    log = evidence_dir / "logs" / f"{run_id}_attempt{attempt:02d}.log"
    watchdog_evidence = evidence_dir / "invalid_attempts" / f"{run_id}_attempt{attempt:02d}.json"
    log.parent.mkdir(parents=True, exist_ok=True)
    command = [
        str(root / "run"), "--paper500", "--method", item["method"],
        "--environment", "new_room", "--record", "yes", "--run-id", run_id,
    ]
    watchdog = [
        "/usr/bin/python3", str(root / "mapex_lab" / "scripts" / "r003_deadlock_watchdog.py"),
        "--run-id", run_id, "--evidence", str(watchdog_evidence),
    ]
    ros_env = effective_ros_env()
    with log.open("a", encoding="utf-8") as stream:
        stream.write("COMMAND " + " ".join(command) + "\n")
        stream.flush()
        runner = subprocess.Popen(
            command, cwd=root, stdout=stream, stderr=subprocess.STDOUT,
            start_new_session=True, env=ros_env,
        )
        watcher = subprocess.Popen(
            watchdog, cwd=root, stdout=stream, stderr=subprocess.STDOUT,
            start_new_session=True, env=ros_env,
        )
        technical_deadlock = False
        try:
            while runner.poll() is None:
                watcher_status = watcher.poll()
                if watcher_status == WATCHDOG_EXIT_CODE:
                    technical_deadlock = True
                    os.killpg(runner.pid, signal.SIGINT)
                    break
                if watcher_status not in (None, 0):
                    os.killpg(runner.pid, signal.SIGINT)
                    raise RuntimeError(f"watchdog failed with status {watcher_status}")
                time.sleep(2.0)
            runner_status = runner.wait(timeout=45)
        finally:
            if watcher.poll() is None:
                os.killpg(watcher.pid, signal.SIGTERM)
                try:
                    watcher.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(watcher.pid, signal.SIGKILL)
        if technical_deadlock:
            return "technical_deadlock", log, watchdog_evidence
        if runner_status != 0:
            return f"ambiguous_runner_failure:{runner_status}", log, watchdog_evidence
    return "runtime_complete", log, watchdog_evidence


def paper500_integrity_faults(target: Path) -> list[str]:
    for filename in ("summary.json", "metadata.json"):
        path = target / filename
        if not path.is_file():
            continue
        payload = json.loads(path.read_text(encoding="utf-8"))
        faults = payload.get("paper500_integrity_faults", [])
        if faults:
            return [str(item) for item in faults]
    return []


def checkpoint_invalid_attempt(
    *,
    path: Path,
    run_id: str,
    method: str,
    attempt: int,
    host: str,
    git_sha: str,
    classification: str,
    reason: str,
    log: Path,
    watchdog_evidence: Path | None,
    runtime_dir: Path,
    deletion_decision: str,
    details: dict | None = None,
) -> dict:
    watchdog = None
    if watchdog_evidence is not None:
        if not watchdog_evidence.is_file():
            raise RuntimeError("watchdog evidence missing before invalid-run deletion")
        watchdog = {
            "path": str(watchdog_evidence),
            "sha256": sha256(watchdog_evidence),
        }
    payload = {
        "schema": "r003_paper500_invalid_attempt_v1",
        "run_id": run_id,
        "method": method,
        "attempt": int(attempt),
        "execution_host": host,
        "source_sha": git_sha,
        "classification": classification,
        "reason": reason,
        "log": str(log),
        "watchdog_evidence": watchdog,
        "runtime_dir": str(runtime_dir),
        "deletion_decision": deletion_decision,
        "checkpointed_at": time.time(),
    }
    if details:
        payload["details"] = details
    atomic_json(path, payload)
    if not path.is_file():
        raise RuntimeError("invalid-attempt checkpoint missing before deletion")
    return payload


def invalidate_and_delete(
    *,
    state: dict,
    state_path: Path,
    record: dict,
    item: dict,
    target: Path,
    root: Path,
    evidence_dir: Path,
    log: Path,
    classification: str,
    reason: str,
    watchdog_evidence: Path | None,
    deletion_decision: str,
    details: dict | None = None,
) -> Path:
    attempt = int(record["attempts"])
    evidence_path = (
        evidence_dir / "invalid_attempts" /
        f"{item['run_id']}_attempt{attempt:02d}_checkpoint.json"
    )
    payload = checkpoint_invalid_attempt(
        path=evidence_path,
        run_id=item["run_id"],
        method=item["method"],
        attempt=attempt,
        host=state["execution_host"],
        git_sha=state["source_sha"],
        classification=classification,
        reason=reason,
        log=log,
        watchdog_evidence=watchdog_evidence,
        runtime_dir=target,
        deletion_decision=deletion_decision,
        details=details,
    )
    record.update(
        status="technical_invalid",
        invalidity_classification=classification,
        invalidity_reason=reason,
        invalidity_evidence=str(evidence_path),
        invalidity_evidence_sha256=sha256(evidence_path),
        deletion_decision=payload["deletion_decision"],
        deletion_completed=False,
    )
    atomic_json(state_path, state)
    delete_invalid_run(target, root, item["run_id"])
    record["deletion_completed"] = True
    atomic_json(state_path, state)
    return evidence_path


def postprocess(root: Path, item: dict, checkpoint: Path, inference_python: str, device: str) -> None:
    target = run_dir(root, item)
    subprocess.run([
        inference_python,
        str(root / "mapex_lab" / "scripts" / "predict_alltrain_offline.py"),
        str(target), "--paper500-snapshots", "--checkpoint", str(checkpoint),
        "--device", device,
    ], cwd=root, check=True)
    subprocess.run([
        sys.executable,
        str(root / "mapex_lab" / "scripts" / "evaluate_r003_paper500.py"),
        str(target),
    ], cwd=root, check=True)
    evaluation = json.loads(
        (target / "evaluation" / "paper500" / "evaluation.json").read_text(encoding="utf-8")
    )
    if evaluation.get("status") != "ok":
        raise RuntimeError(f"paper500 evaluation failed for {item['run_id']}")


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--state", type=Path, default=root / "mapex_lab/results/r003_paper500/batch_state.json")
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--inference-python", default=os.environ.get("MAPEX_LAMA_PYTHON", "python3"))
    parser.add_argument("--device", default=os.environ.get("MAPEX_DEVICE", "cpu"))
    parser.add_argument("--max-technical-retries", type=int, default=1)
    args = parser.parse_args()
    matrix = official_matrix()
    if args.dry_run:
        print(json.dumps({"count": len(matrix), "runs": matrix}, indent=2))
        return 0
    if args.checkpoint is None:
        parser.error("--checkpoint is required for official execution")
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file():
        parser.error(f"checkpoint does not exist: {checkpoint}")

    state_path = args.state.resolve()
    current_sha = source_sha(root)
    current_host = socket.gethostname()
    state = json.loads(state_path.read_text()) if state_path.is_file() else {
        "schema": "r003_paper500_batch_v2",
        "matrix": matrix,
        "execution_host": current_host,
        "source_sha": current_sha,
        "runs": {},
    }
    if state.get("schema") != "r003_paper500_batch_v2":
        raise RuntimeError("unsupported batch state schema; manual recovery required")
    if state.get("matrix") != matrix:
        raise RuntimeError("batch matrix differs from persisted state")
    if state.get("execution_host") != current_host:
        raise RuntimeError("batch execution host differs from persisted state")
    if state.get("source_sha") != current_sha:
        raise RuntimeError("batch source SHA differs from persisted state")
    evidence_dir = state_path.parent
    while (item := first_unfinished(matrix, state)) is not None:
        run_id = item["run_id"]
        record = state["runs"].setdefault(run_id, {"attempts": 0})
        blocker = resume_blocker(record)
        if blocker:
            if blocker == "recovery_required":
                record["status"] = blocker
                atomic_json(state_path, state)
            print(f"batch blocked at {run_id}: {blocker}", file=sys.stderr)
            return 5
        target = run_dir(root, item)
        if target.exists():
            raise RuntimeError(f"unfinished run directory requires manual inspection: {target}")
        record.update(status="running", attempts=int(record["attempts"]) + 1)
        atomic_json(state_path, state)
        outcome, log, watchdog_evidence = run_one(
            root, item, record["attempts"], evidence_dir
        )
        record.update(last_outcome=outcome, log=str(log))
        if outcome == "technical_deadlock":
            retry_allowed = record["attempts"] <= args.max_technical_retries
            invalidate_and_delete(
                state=state,
                state_path=state_path,
                record=record,
                item=item,
                target=target,
                root=root,
                evidence_dir=evidence_dir,
                log=log,
                classification="watchdog_confirmed_physical_deadlock",
                reason="frozen watchdog exited with technical-deadlock status",
                watchdog_evidence=watchdog_evidence,
                deletion_decision=invalid_deletion_decision(
                    record["attempts"], args.max_technical_retries
                ),
            )
            if not retry_allowed:
                record["status"] = "blocked_repeated_technical_invalidity"
                atomic_json(state_path, state)
                return 3
            record["status"] = "retry_pending_same_id"
            atomic_json(state_path, state)
            continue
        if outcome != "runtime_complete":
            record["status"] = "blocked_ambiguous_failure"
            atomic_json(state_path, state)
            return 4
        integrity_faults = paper500_integrity_faults(target)
        if integrity_faults:
            retry_allowed = record["attempts"] <= args.max_technical_retries
            invalidate_and_delete(
                state=state,
                state_path=state_path,
                record=record,
                item=item,
                target=target,
                root=root,
                evidence_dir=evidence_dir,
                log=log,
                classification="recorder_integrity_fault",
                reason="paper500 runtime reported integrity faults",
                watchdog_evidence=None,
                deletion_decision=invalid_deletion_decision(
                    record["attempts"], args.max_technical_retries
                ),
                details={"paper500_integrity_faults": integrity_faults},
            )
            if not retry_allowed:
                record["status"] = "blocked_repeated_technical_invalidity"
                atomic_json(state_path, state)
                return 3
            record["status"] = "retry_pending_same_id"
            atomic_json(state_path, state)
            continue
        try:
            postprocess(root, item, checkpoint, args.inference_python, args.device)
        except Exception as exc:
            record.update(status="blocked_postprocess_failure", error=str(exc))
            atomic_json(state_path, state)
            raise
        record.update(status="complete", completed_at=time.time())
        atomic_json(state_path, state)
    return 0


if __name__ == "__main__":
    sys.exit(main())
