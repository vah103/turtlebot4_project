#!/usr/bin/env python3
"""Low-touch, fold-checkpointed runner for W027 Gate-U scoring."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import pandas as pd

from d1_gate_u import (METHOD_ID, RUNS, bootstrap, classify, json_dump, score_fold,
                       sha256_file, validate_input)


def now() -> str:
    return datetime.now(timezone.utc).isoformat()


def implementation_hash(root: Path) -> str:
    digest = hashlib.sha256()
    for name in ("d1_gate_u.py", "run_gate_u_low_touch.py"):
        digest.update((root / name).read_bytes())
    return digest.hexdigest()


def validate_fold_cache(target: Path, expected_identity: dict, expected_run: str) -> dict:
    manifest_path = target / "completion_manifest.json"
    summary_path = target / "fold_summary.json"
    decisions_path = target / "per_decision.csv"
    for required in (manifest_path, summary_path, decisions_path):
        if not required.is_file():
            raise RuntimeError(f"missing required fold artifact: {required}")
    try:
        saved = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError) as exc:
        raise RuntimeError(f"invalid fold manifest: {expected_run}") from exc
    checks = {
        "identity": saved.get("identity") == expected_identity,
        "fold": saved.get("fold") == expected_run,
        "fold_summary_sha256": saved.get("fold_summary_sha256") == sha256_file(summary_path),
        "per_decision_sha256": saved.get("per_decision_sha256") == sha256_file(decisions_path),
    }
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        raise RuntimeError(f"resume identity/hash mismatch: {expected_run}: {','.join(failed)}")
    return saved


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-folds", type=int)
    parser.add_argument("--skip-tests", action="store_true")
    args = parser.parse_args()
    script_root = Path(__file__).resolve().parent
    args.output.mkdir(parents=True, exist_ok=True)
    log_path = args.output / "execution.log"
    def log(message: str) -> None:
        with log_path.open("a", encoding="utf-8") as stream:
            stream.write(f"{now()} {message}\n")
    input_sha = sha256_file(args.input)
    impl_sha = implementation_hash(script_root)
    frame = pd.read_csv(args.input, low_memory=False)
    preflight = validate_input(frame, input_sha)
    identity = {"method_id": METHOD_ID, "input_sha256": input_sha, "implementation_sha256": impl_sha}
    preflight.update(identity)
    json_dump(preflight, args.output / "preflight.json")
    log(f"PREFLIGHT_OK rows={len(frame)} input={input_sha} implementation={impl_sha}")
    if not args.skip_tests:
        test_log = args.output / "tests.log"
        with test_log.open("w", encoding="utf-8") as stream:
            proc = subprocess.run(["python3", "-m", "unittest", "-v", "test_d1_gate_u.py"], cwd=script_root, stdout=stream, stderr=subprocess.STDOUT)
        if proc.returncode:
            log("TESTS_FAILED")
            return proc.returncode
        log("TESTS_OK")
    folds_root = args.output / "folds"
    folds_root.mkdir(exist_ok=True)
    processed = 0
    for run_id in RUNS:
        target = folds_root / f"holdout_{run_id}"
        manifest = target / "completion_manifest.json"
        if manifest.exists():
            validate_fold_cache(target, identity, run_id)
            log(f"FOLD_VALIDATED_SKIP {run_id}")
            continue
        if args.max_folds is not None and processed >= args.max_folds:
            break
        temp = folds_root / f"holdout_{run_id}.tmp"
        if temp.exists():
            shutil.rmtree(temp)
        temp.mkdir()
        fold, rows = score_fold(frame, run_id)
        json_dump(fold, temp / "fold_summary.json")
        rows.to_csv(temp / "per_decision.csv", index=False)
        json_dump({"identity": identity, "fold": run_id, "fold_summary_sha256": sha256_file(temp / "fold_summary.json"),
                   "per_decision_sha256": sha256_file(temp / "per_decision.csv"), "completed_at": now()}, temp / "completion_manifest.json")
        os.replace(temp, target)
        log(f"FOLD_COMPLETE {run_id} selected={fold['selected_candidate']} status={fold['selection_status']}")
        processed += 1
    completed = []
    decision_tables = []
    for run_id in RUNS:
        target = folds_root / f"holdout_{run_id}"
        if not (target / "completion_manifest.json").exists():
            continue
        completed.append(json.loads((target / "fold_summary.json").read_text()))
        decision_tables.append(pd.read_csv(target / "per_decision.csv", low_memory=False))
    json_dump({"identity": identity, "completed_folds": [f["held_out_run"] for f in completed],
               "next_fold": next((r for r in RUNS if r not in {f['held_out_run'] for f in completed}), None)}, args.output / "batch_state.json")
    if len(completed) != 10:
        log(f"PARTIAL completed={len(completed)}")
        return 0
    final_tmp = args.output / "final.tmp"
    final = args.output / "final"
    if final_tmp.exists(): shutil.rmtree(final_tmp)
    if final.exists(): shutil.rmtree(final)
    final_tmp.mkdir()
    summary = classify(completed)
    summary["bootstrap"] = bootstrap(completed)
    late = [f for f in completed if f.get("late_primary_rows", 0) >= 3]
    summary["late_primary_support"] = {"contributing_runs": len(late), "status": "SUPPORTED" if len(late) >= 5 else "LATE_PRIMARY_INSUFFICIENT"}
    json_dump(summary, final_tmp / "gate_u_summary.json")
    json_dump(completed, final_tmp / "per_fold_results.json")
    pd.concat(decision_tables, ignore_index=True).to_csv(final_tmp / "per_decision_results.csv", index=False)
    json_dump({"status": "COMPLETE_PENDING_REVIEW", "identity": identity,
               "folds": len(completed), "decision_rows": sum(len(x) for x in decision_tables),
               "selected_candidates": sorted({f["selected_candidate"] for f in completed}),
               "historical_outcome": summary["outcome"], "reason_chain": summary["reason_chain"],
               "test_log": "../tests.log", "execution_log": "../execution.log"},
              final_tmp / "completion_summary.json")
    artifacts = {}
    for path in sorted(final_tmp.iterdir()): artifacts[path.name] = sha256_file(path)
    json_dump({"identity": identity, "artifacts": artifacts, "completed_at": now()}, final_tmp / "artifact_manifest.json")
    os.replace(final_tmp, final)
    log(f"FINAL_COMPLETE outcome={summary['outcome']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
