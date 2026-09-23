#!/usr/bin/env python3
"""Resume-safe driver for the 20 official R003 paper500 runs."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


WATCHDOG_EXIT_CODE = 42


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


def run_one(root: Path, item: dict, attempt: int, evidence_dir: Path) -> tuple[str, Path]:
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
    with log.open("a", encoding="utf-8") as stream:
        stream.write("COMMAND " + " ".join(command) + "\n")
        stream.flush()
        runner = subprocess.Popen(
            command, cwd=root, stdout=stream, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        watcher = subprocess.Popen(
            watchdog, cwd=root, stdout=stream, stderr=subprocess.STDOUT,
            start_new_session=True,
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
            return "technical_deadlock", log
        if runner_status != 0:
            return f"ambiguous_runner_failure:{runner_status}", log
    return "runtime_complete", log


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
    state = json.loads(state_path.read_text()) if state_path.is_file() else {
        "schema": "r003_paper500_batch_v1", "matrix": matrix, "runs": {},
    }
    if state.get("matrix") != matrix:
        raise RuntimeError("batch matrix differs from persisted state")
    evidence_dir = state_path.parent
    while (item := first_unfinished(matrix, state)) is not None:
        run_id = item["run_id"]
        record = state["runs"].setdefault(run_id, {"attempts": 0})
        target = run_dir(root, item)
        if target.exists():
            raise RuntimeError(f"unfinished run directory requires manual inspection: {target}")
        record.update(status="running", attempts=int(record["attempts"]) + 1)
        atomic_json(state_path, state)
        outcome, log = run_one(root, item, record["attempts"], evidence_dir)
        record.update(last_outcome=outcome, log=str(log))
        if outcome == "technical_deadlock":
            record["status"] = "technical_invalid"
            atomic_json(state_path, state)
            delete_invalid_run(target, root, run_id)
            if record["attempts"] > args.max_technical_retries:
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

