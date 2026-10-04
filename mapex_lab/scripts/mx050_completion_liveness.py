#!/usr/bin/env python3
"""MX050 completion-control liveness heartbeat and passive supervisor.

This module is deliberately independent of ROS time and scientific payloads.  It
observes a small control-plane heartbeat written by the explorer and returns a
technical-abort reason.  Process containment remains the launcher's job.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any, Callable


CHANNEL = "MX050_COMPLETION_LIVENESS_V1"
HEARTBEAT_SCHEMA = "MX050_COMPLETION_LIVENESS_HEARTBEAT_V1"
ABORT_SCHEMA = "MX050_COMPLETION_LIVENESS_TECHNICAL_ABORT_V1"
HEARTBEAT_PERIOD_S = 5.0
HEARTBEAT_TIMEOUT_S = 30.0
ACTION_TIMEOUT_S = 120.0
MAP_TIMEOUT_S = 120.0

W1 = "MX050_TECH_ABORT_EXPLORER_CONTROL_HEARTBEAT_TIMEOUT"
W2 = "MX050_TECH_ABORT_COMPLETION_REVALIDATION_ACTION_TIMEOUT"
W3 = "MX050_TECH_ABORT_COMPLETION_MAP_STREAM_STALE"
WATCHDOG_REASONS = {W1, W2, W3}

HEARTBEAT_FIELDS = {
    "schema",
    "channel",
    "sequence",
    "explorer_pid",
    "run_slot",
    "attempt_id",
    "event",
    "emitted_monotonic_s",
    "state",
    "completion_reason",
    "completion_streak",
    "map_generation",
    "last_map_callback_age_s",
    "revalidation_active",
    "revalidation_index",
    "revalidation_candidate_count",
    "planner_action_pending",
    "planner_action_pending_age_s",
    "completed",
}


def canonical_json_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n").encode()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(canonical_json_bytes(value))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


class HeartbeatWriter:
    """Atomically materialize the content-neutral explorer heartbeat."""

    def __init__(
        self,
        path: Path,
        run_slot: str,
        attempt_id: str,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.path = path
        self.run_slot = run_slot
        self.attempt_id = attempt_id
        self.clock = clock
        self.sequence = 0

    def emit(self, event: str, fields: dict[str, Any]) -> dict[str, Any]:
        now_s = self.clock()
        self.sequence += 1
        payload = {
            "schema": HEARTBEAT_SCHEMA,
            "channel": CHANNEL,
            "sequence": self.sequence,
            "explorer_pid": os.getpid(),
            "run_slot": self.run_slot,
            "attempt_id": self.attempt_id,
            "event": event,
            "emitted_monotonic_s": now_s,
            **fields,
        }
        unexpected = set(payload) - HEARTBEAT_FIELDS
        missing = HEARTBEAT_FIELDS - set(payload)
        if unexpected or missing:
            raise ValueError(
                f"invalid heartbeat field set: missing={sorted(missing)}, "
                f"unexpected={sorted(unexpected)}"
            )
        atomic_write_json(self.path, payload)
        return payload


def validate_heartbeat(payload: Any, expected_pid: int | None = None) -> bool:
    if not isinstance(payload, dict) or set(payload) != HEARTBEAT_FIELDS:
        return False
    if payload.get("schema") != HEARTBEAT_SCHEMA or payload.get("channel") != CHANNEL:
        return False
    if expected_pid is not None and payload.get("explorer_pid") != expected_pid:
        return False
    if not isinstance(payload.get("sequence"), int) or payload["sequence"] < 1:
        return False
    if not isinstance(payload.get("emitted_monotonic_s"), (int, float)):
        return False
    return True


class LivenessPolicy:
    """Stateful W1 observer plus stateless W2/W3 predicate evaluation."""

    def __init__(self, started_monotonic_s: float) -> None:
        self.started_monotonic_s = started_monotonic_s
        self.last_valid_heartbeat_seen_s: float | None = None
        self.last_sequence = 0

    def observe(self, payload: Any, observed_monotonic_s: float, expected_pid: int) -> bool:
        if not validate_heartbeat(payload, expected_pid):
            return False
        emitted = float(payload["emitted_monotonic_s"])
        sequence = int(payload["sequence"])
        if emitted > observed_monotonic_s + 1.0 or sequence < self.last_sequence:
            return False
        if sequence > self.last_sequence:
            self.last_sequence = sequence
            self.last_valid_heartbeat_seen_s = observed_monotonic_s
        return True

    def evaluate(self, payload: Any, now_s: float) -> tuple[str, float] | None:
        last_seen = self.last_valid_heartbeat_seen_s
        heartbeat_age = now_s - (
            self.started_monotonic_s if last_seen is None else last_seen
        )
        if heartbeat_age > HEARTBEAT_TIMEOUT_S:
            return W1, heartbeat_age

        if not isinstance(payload, dict):
            return None
        verifying = payload.get("state") == "VERIFYING_COMPLETE"
        if (
            verifying
            and payload.get("revalidation_active") is True
            and payload.get("planner_action_pending") is True
            and isinstance(payload.get("planner_action_pending_age_s"), (int, float))
            and payload["planner_action_pending_age_s"] > ACTION_TIMEOUT_S
        ):
            return W2, float(payload["planner_action_pending_age_s"])
        if (
            verifying
            and isinstance(payload.get("last_map_callback_age_s"), (int, float))
            and payload["last_map_callback_age_s"] > MAP_TIMEOUT_S
        ):
            return W3, float(payload["last_map_callback_age_s"])
        return None


def recovery_disposition(attempt_id: str, terminal_reason: str) -> dict[str, Any]:
    """Pure transaction rule used by the reserve preflight and later driver."""
    if attempt_id == "attempt01" and terminal_reason == (
        "PRE_AMENDMENT_ABORT_COMPLETION_LIVENESS_METHOD_GAP"
    ):
        return {
            "scientific_slot_filled": False,
            "exclude_layout_49001": False,
            "activate_reserve_49011": False,
            "allow_third_49001_r1_retry": False,
            "classification": "AUDIT_ONLY_NON_RESERVE",
        }
    if attempt_id == "attempt02" and terminal_reason in WATCHDOG_REASONS:
        return {
            "scientific_slot_filled": False,
            "exclude_layout_49001": True,
            "activate_reserve_49011": True,
            "allow_third_49001_r1_retry": False,
            "classification": "TECHNICALLY_INVALID_LAYOUT_EXCLUDED",
        }
    if attempt_id == "attempt02" and terminal_reason == "ordinary_completion":
        return {
            "scientific_slot_filled": True,
            "exclude_layout_49001": False,
            "activate_reserve_49011": False,
            "allow_third_49001_r1_retry": False,
            "classification": "PENDING_COLLECTION_INTEGRITY_QA",
        }
    return {
        "scientific_slot_filled": False,
        "exclude_layout_49001": False,
        "activate_reserve_49011": False,
        "allow_third_49001_r1_retry": False,
        "classification": "MX050_BLOCKED_UNCLASSIFIED_FAILURE",
    }


def read_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def read_digest_sidecar(path: Path) -> str:
    sidecar = Path(f"{path}.sha256")
    value = sidecar.read_text(encoding="utf-8").strip().split()[0]
    if len(value) != 64:
        raise RuntimeError(f"invalid SHA256 sidecar: {sidecar}")
    return value


def git_output(repo_root: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo_root), *args], text=True
    ).strip()


def normalized_machine_id_sha256() -> str:
    machine_id = Path("/etc/machine-id").read_text(encoding="utf-8").strip()
    return hashlib.sha256(machine_id.encode()).hexdigest()


def verify_overlay_identity(
    repo_root: Path, overlay_path: Path
) -> tuple[dict[str, Any], str, str, str, list[dict[str, Any]]]:
    overlay = read_json(overlay_path)
    if not isinstance(overlay, dict):
        raise RuntimeError("overlay must be a valid JSON object")
    overlay_sha = sha256_file(overlay_path)
    if overlay_sha != read_digest_sidecar(overlay_path):
        raise RuntimeError("overlay SHA256 sidecar mismatch")

    expected_commit = overlay.get("recovery_implementation", {}).get(
        "technical_repo_commit"
    )
    if git_output(repo_root, "rev-parse", "HEAD") != expected_commit:
        raise RuntimeError("recovery technical commit drift")
    expected_machine = overlay.get("dell_machine", {}).get("machine_id_sha256")
    if normalized_machine_id_sha256() != expected_machine:
        raise RuntimeError("DELL machine identity drift")

    for binding in overlay.get("original_acquisition", {}).get("files", []):
        path = Path(binding["path"])
        if sha256_file(path) != binding["sha256"]:
            raise RuntimeError(f"original frozen identity drift: {path}")

    implementation_files = overlay.get("recovery_implementation", {}).get(
        "files", []
    )
    if not implementation_files:
        raise RuntimeError("overlay binds no recovery implementation files")
    for binding in implementation_files:
        relative_path = binding["path"]
        path = repo_root / relative_path
        if sha256_file(path) != binding["sha256"]:
            raise RuntimeError(f"recovery file SHA256 drift: {relative_path}")
        if git_output(repo_root, "rev-parse", f"HEAD:{relative_path}") != binding["git_blob"]:
            raise RuntimeError(f"recovery file Git blob drift: {relative_path}")
    return overlay, overlay_sha, expected_commit, expected_machine, implementation_files


def run_preflight(args: argparse.Namespace) -> int:
    repo_root = Path(args.repo_root).resolve()
    overlay_path = Path(args.overlay).resolve()
    audit_path = Path(args.audit_output).resolve()
    if audit_path.exists():
        raise RuntimeError(f"refusing to overwrite preflight audit: {audit_path}")
    overlay, overlay_sha, commit, machine, files = verify_overlay_identity(
        repo_root, overlay_path
    )
    test_path = repo_root / "mapex_lab/scripts/test_mx050_completion_liveness.py"
    environment = dict(os.environ)
    scripts_path = str(repo_root / "mapex_lab/scripts")
    environment["PYTHONPATH"] = (
        scripts_path
        if not environment.get("PYTHONPATH")
        else f"{scripts_path}{os.pathsep}{environment['PYTHONPATH']}"
    )
    completed = subprocess.run(
        [sys.executable, "-m", "unittest", "-v", str(test_path)],
        cwd=repo_root,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "MX050_RECOVERY_IMPLEMENTATION_PREFLIGHT_FAIL\n"
            + completed.stdout
            + completed.stderr
        )
    results = {f"P{i}": "PASS" for i in range(1, 10)}
    shared_binding = {
        "overlay_path": str(overlay_path),
        "overlay_sha256": overlay_sha,
        "accepted_r1a_commit": overlay["accepted_recovery_method"]["commit"],
        "accepted_r1a_blob": overlay["accepted_recovery_method"]["git_blob"],
        "recovery_technical_commit": commit,
        "recovery_implementation_files": files,
        "machine_id_sha256": machine,
    }
    audit = {
        "schema": "MX050_RECOVERY_P1_P9_AUDIT_V1",
        "classification": "MX050_RECOVERY_IMPLEMENTATION_PREFLIGHT_PASS",
        **shared_binding,
        "results": results,
        "records": [
            {"test_id": test_id, "result": result, **shared_binding}
            for test_id, result in results.items()
        ],
        "test_command": [
            sys.executable,
            "-m",
            "unittest",
            "-v",
            str(test_path),
        ],
        "test_output": completed.stdout + completed.stderr,
    }
    atomic_write_json(audit_path, audit)
    print(sha256_file(audit_path))
    return 0


def verify_ready(args: argparse.Namespace) -> int:
    """Fail closed on any overlay/token/implementation identity drift."""
    repo_root = Path(args.repo_root).resolve()
    overlay_path = Path(args.overlay).resolve()
    token_path = Path(args.ready_token).resolve()
    overlay = read_json(overlay_path)
    token = read_json(token_path)
    if not isinstance(overlay, dict) or not isinstance(token, dict):
        raise RuntimeError("overlay and ready token must be valid JSON objects")

    overlay_sha = sha256_file(overlay_path)
    token_sha = sha256_file(token_path)
    overlay, overlay_sha, expected_commit, expected_machine, implementation_files = (
        verify_overlay_identity(repo_root, overlay_path)
    )
    if token_sha != read_digest_sidecar(token_path):
        raise RuntimeError("ready-token SHA256 sidecar mismatch")
    if token.get("classification") != (
        "MX050_RECOVERY_READY_FOR_EXACT_49001_R1_ATTEMPT02"
    ):
        raise RuntimeError("ready-token classification mismatch")
    if token.get("target_slot") != args.run_slot or token.get(
        "target_infrastructure_attempt"
    ) != args.attempt_id:
        raise RuntimeError("ready-token target mismatch")
    if token.get("overlay", {}).get("sha256") != overlay_sha:
        raise RuntimeError("ready-token overlay binding mismatch")
    if Path(token.get("overlay", {}).get("path", "")).resolve() != overlay_path:
        raise RuntimeError("ready-token overlay path mismatch")

    if token.get("recovery_technical_commit") != expected_commit:
        raise RuntimeError("ready-token technical commit mismatch")

    if token.get("machine_id_sha256") != expected_machine:
        raise RuntimeError("ready-token machine identity mismatch")

    audit_binding = token.get("p1_p9_audit", {})
    audit_path = Path(audit_binding.get("path", ""))
    if sha256_file(audit_path) != audit_binding.get("sha256"):
        raise RuntimeError("P1-P9 audit identity drift")
    audit = read_json(audit_path)
    if not isinstance(audit, dict) or audit.get("overlay_sha256") != overlay_sha:
        raise RuntimeError("P1-P9 audit overlay binding mismatch")
    if audit.get("results") != {f"P{i}": "PASS" for i in range(1, 10)}:
        raise RuntimeError("P1-P9 aggregate is not an exact all-PASS set")

    print(
        json.dumps(
            {
                "classification": token["classification"],
                "target_slot": args.run_slot,
                "target_infrastructure_attempt": args.attempt_id,
                "original_acquisition": overlay["original_acquisition"],
                "recovery_implementation_files": implementation_files,
                "overlay_path": str(overlay_path),
                "overlay_sha256": overlay_sha,
                "ready_token_path": str(token_path),
                "ready_token_sha256": token_sha,
                "recovery_technical_commit": expected_commit,
                "machine_id_sha256": expected_machine,
                "artifact_namespace_isolation": {
                    "attempt01_reuse": False,
                    "attempt02_runtime": str(
                        repo_root / "evidence/mx050_recovery/runtime/attempt02"
                    ),
                    "attempt02_output": str(
                        repo_root
                        / "mapex_lab/experiments/mapex/mx050_recovery_attempts"
                        / "attempt02"
                        / args.run_slot
                    ),
                },
                "drift_check": "PASS",
            },
            sort_keys=True,
        )
    )
    return 0


def process_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def supervise(args: argparse.Namespace) -> int:
    heartbeat_path = Path(args.heartbeat)
    marker_path = Path(args.abort_marker)
    started_s = time.monotonic()
    policy = LivenessPolicy(started_s)
    last_payload: Any = None

    while process_alive(args.explorer_pid):
        now_s = time.monotonic()
        candidate = read_json(heartbeat_path)
        if policy.observe(candidate, now_s, args.explorer_pid):
            last_payload = candidate
        trigger = policy.evaluate(last_payload, now_s)
        if trigger is not None:
            reason, trigger_age_s = trigger
            marker = {
                "schema": ABORT_SCHEMA,
                "channel": CHANNEL,
                "run_slot": args.run_slot,
                "attempt_id": args.attempt_id,
                "watchdog_reason": reason,
                "trigger_age_s": round(trigger_age_s, 6),
                "trigger_monotonic_s": now_s,
                "explorer_pid": args.explorer_pid,
                "last_technical_heartbeat": last_payload,
                "containment_requested": True,
                "scientific_completion": False,
            }
            atomic_write_json(marker_path, marker)
            return 42
        time.sleep(args.poll_interval)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    monitor = subparsers.add_parser("supervise")
    monitor.add_argument("--heartbeat", required=True)
    monitor.add_argument("--abort-marker", required=True)
    monitor.add_argument("--run-slot", required=True)
    monitor.add_argument("--attempt-id", required=True)
    monitor.add_argument("--explorer-pid", required=True, type=int)
    monitor.add_argument("--poll-interval", type=float, default=0.25)
    monitor.set_defaults(func=supervise)
    verify = subparsers.add_parser("verify-ready")
    verify.add_argument("--repo-root", required=True)
    verify.add_argument("--overlay", required=True)
    verify.add_argument("--ready-token", required=True)
    verify.add_argument("--run-slot", required=True)
    verify.add_argument("--attempt-id", required=True)
    verify.set_defaults(func=verify_ready)
    preflight = subparsers.add_parser("preflight")
    preflight.add_argument("--repo-root", required=True)
    preflight.add_argument("--overlay", required=True)
    preflight.add_argument("--audit-output", required=True)
    preflight.set_defaults(func=run_preflight)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
