#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import shlex
from pathlib import Path
from typing import Any

from mx049_generator import (
    GENERATOR_ID, LAYOUT_SEEDS, RUN_SEEDS, COHORT_ID, NAMESPACE,
    MX045_METHOD_COMMIT, MX045_METHOD_BLOB,
    MX048_CONTRACT_COMMIT, MX048_CONTRACT_BLOB,
    MX049_ADDENDUM_COMMIT, MX049_ADDENDUM_BLOB,
    split_role, canonical_json_bytes, write_canonical_json, sha256_file, git_value,
)

SEAL_ERROR = "CONFIRMATION_NUMERIC_CONTENT_SEALED"
MANIFEST_SCHEMA = "MX049_FRESH_DELL_ACQ_MANIFEST_V1"
OLD_SOURCE_MANIFEST_SHA256 = "6d5515c8055eba05b22a85de600bac3e7b04694f2c92655e9e62c4b38fc89caa"
DENYLIST_REL = "MX049_OLD_MX046_GEOMETRY_DENYLIST.json"
DELL_BINDING_REL = "MX049_DELL_BINDING.json"
PASS_TOKEN_REL = "MX049_DELL_PREFLIGHT_PASS_TOKEN.json"
PASS_CLASSIFICATION = "MX049_DELL_PREFLIGHT_PASS_READY_FOR_MX050_COLLECTION"
COMPONENT_TAGS = ("gazebo", "exploration", "planner", "sensor")
COMPONENT_STATUSES = (
    "NOT_RANDOM",
    "UNSEEDED_RUNTIME_NONDETERMINISM",
    "SEED_INTERFACE_PRESENT_NOT_VERIFIED",
    "SEEDED_EFFECTIVE",
)

def derive_seed32(layout_seed: int, run_seed: int, tag: str) -> int:
    if tag not in COMPONENT_TAGS:
        raise ValueError(f"invalid component tag {tag}")
    key = f"MX048_RUN_V1|{layout_seed}|{run_seed}|{tag}".encode("ascii")
    d = hashlib.sha256(key).digest()
    return 1 + (int.from_bytes(d[:4], "big") % 2147483646)

def run_id(layout_seed: int, run_seed: int) -> str:
    return f"mx049_fresh_dell_l{layout_seed}_r{run_seed}"

def should_run_offline_evaluator(acquisition_only: bool) -> bool:
    return not acquisition_only

def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))

def _canonical_rel(path: str) -> str:
    p = Path(path)
    if p.is_absolute() or ".." in p.parts or "\\" in path:
        raise ValueError(f"noncanonical path: {path}")
    return p.as_posix()

def build_manifest(repo_root: Path, generated: list[dict], created_utc: str) -> dict:
    repo_root = repo_root.resolve()
    technical_commit = git_value(repo_root, "rev-parse", "HEAD")
    generator_path = repo_root / "mapex_lab/scripts/mx049_generator.py"
    generator_blob = git_value(repo_root, "hash-object", str(generator_path))
    layouts = []
    by_seed = {int(x["layout_seed"]): x for x in generated}
    for seed in LAYOUT_SEEDS:
        g = by_seed[seed]
        split, role = split_role(seed)
        if g["status"] != "ACCEPTED":
            layout = {
                "layout_seed": seed, "split": split, "primary_or_reserve": role,
                "planned_status": "PLANNED_PRIMARY" if role == "primary" else "DORMANT_RESERVE",
                "generator_attempt_status": g["status"], "accepted_attempt": None,
                "geometry_parameter_digest": None,
                "world_path": None, "world_sha256": None,
                "layout_config_path": None, "layout_config_sha256": None,
                "world_identity_sha256": None, "gt_binding_path": None,
                "gt_sha256": None, "roi_sha256": None, "runs": [],
            }
            layouts.append(layout)
            continue
        cfg = _load_json(repo_root / g["layout_config_path"])
        binding = _load_json(repo_root / g["gt_binding_path"])
        runs = []
        for r in RUN_SEEDS:
            seeds = {tag: derive_seed32(seed, r, tag) for tag in COMPONENT_TAGS}
            runs.append({
                "run_id": run_id(seed, r), "run_seed": r,
                "gazebo_seed": seeds["gazebo"],
                "exploration_seed": seeds["exploration"],
                "planner_seed": seeds["planner"],
                "sensor_seed": seeds["sensor"],
                "expected_world_identity_sha256": g["world_identity_sha256"],
                "expected_layout_config_sha256": g["layout_config_sha256"],
            })
        layouts.append({
            "layout_seed": seed, "split": split, "primary_or_reserve": role,
            "planned_status": "PLANNED_PRIMARY" if role == "primary" else "DORMANT_RESERVE",
            "generator_attempt_status": g["status"],
            "accepted_attempt": g["accepted_attempt"],
            "geometry_parameter_digest": g["geometry_parameter_digest"],
            "world_path": _canonical_rel(g["world_path"]),
            "world_sha256": g["world_sha256"],
            "layout_config_path": _canonical_rel(g["layout_config_path"]),
            "layout_config_sha256": g["layout_config_sha256"],
            "world_identity_sha256": g["world_identity_sha256"],
            "gt_binding_path": _canonical_rel(g["gt_binding_path"]),
            "gt_sha256": binding["gt_sha256"],
            "roi_sha256": binding["roi_sha256"],
            "runs": runs,
        })
    return {
        "schema_version": MANIFEST_SCHEMA,
        "cohort_id": COHORT_ID,
        "namespace": NAMESPACE,
        "mx045_method_commit": MX045_METHOD_COMMIT,
        "mx045_method_blob": MX045_METHOD_BLOB,
        "mx048_contract_commit": MX048_CONTRACT_COMMIT,
        "mx048_contract_blob": MX048_CONTRACT_BLOB,
        "mx049_addendum_commit": MX049_ADDENDUM_COMMIT,
        "mx049_addendum_blob": MX049_ADDENDUM_BLOB,
        "old_mx046_geometry_denylist_path": DENYLIST_REL,
        "old_mx046_geometry_denylist_sha256": sha256_file(repo_root / DENYLIST_REL),
        "old_source_manifest_sha256": OLD_SOURCE_MANIFEST_SHA256,
        "dell_binding_path": DELL_BINDING_REL,
        "dell_binding_sha256": sha256_file(repo_root / DELL_BINDING_REL),
        "authorized_machine_label": "DELL",
        "cross_cohort_geometry_disjoint": True,
        "generator_id": GENERATOR_ID,
        "generator_commit": technical_commit,
        "generator_blob": generator_blob,
        "technical_repo_commit": technical_commit,
        "created_utc": created_utc,
        "manifest_state": "FROZEN_PRE_OUTCOME",
        "layouts": layouts,
    }

def find_layout(manifest: dict, layout_seed: int) -> dict:
    found = [x for x in manifest["layouts"] if int(x["layout_seed"]) == int(layout_seed)]
    if len(found) != 1:
        raise ValueError(f"manifest layout missing/duplicate: {layout_seed}")
    return found[0]

def find_run(layout: dict, seed: int) -> dict:
    found = [x for x in layout["runs"] if int(x["run_seed"]) == int(seed)]
    if len(found) != 1:
        raise ValueError(f"manifest run missing/duplicate: {seed}")
    return found[0]

def verify_layout_identity(repo_root: Path, layout: dict) -> None:
    world = repo_root / _canonical_rel(layout["world_path"])
    cfg = repo_root / _canonical_rel(layout["layout_config_path"])
    binding = repo_root / _canonical_rel(layout["gt_binding_path"])
    if sha256_file(world) != layout["world_sha256"]:
        raise RuntimeError("MX048_WORLD_MUTATION_DETECTED")
    if sha256_file(cfg) != layout["layout_config_sha256"]:
        raise RuntimeError("MX048_LAYOUT_CONFIG_MUTATION_DETECTED")
    b = _load_json(binding)
    if b["world_sha256"] != layout["world_sha256"]:
        raise RuntimeError("MX048_GT_WORLD_BINDING_MISMATCH")
    if b["world_identity_sha256"] != layout["world_identity_sha256"]:
        raise RuntimeError("MX048_GT_WORLD_IDENTITY_MISMATCH")
    if sha256_file(repo_root / b["gt_path"]) != layout["gt_sha256"]:
        raise RuntimeError("MX048_GT_HASH_MISMATCH")
    if sha256_file(repo_root / b["roi_path"]) != layout["roi_sha256"]:
        raise RuntimeError("MX048_ROI_HASH_MISMATCH")

def _verify_collection_token(repo_root: Path, manifest_path: Path, manifest: dict, machine_id_sha256: str) -> None:
    token_path = repo_root / PASS_TOKEN_REL
    if not token_path.is_file():
        raise RuntimeError("MX049_COLLECTION_BLOCKED_NO_FRESH_PASS_TOKEN")
    token = _load_json(token_path)
    if (
        token.get("classification") != PASS_CLASSIFICATION
        or token.get("cohort_id") != COHORT_ID
        or token.get("fresh_manifest_path") != manifest_path.relative_to(repo_root).as_posix()
        or token.get("fresh_manifest_sha256") != sha256_file(manifest_path)
        or token.get("dell_binding_sha256") != manifest.get("dell_binding_sha256")
        or token.get("old_mx046_geometry_denylist_sha256") != manifest.get("old_mx046_geometry_denylist_sha256")
        or token.get("cross_cohort_geometry_disjoint") is not True
        or token.get("machine_id_sha256") != machine_id_sha256
        or token.get("scientific_run_started_before_token") is not False
        or token.get("confirmation_state") != "SEALED_RAW_ONLY"
    ):
        raise RuntimeError("MX049_COLLECTION_BLOCKED_INVALID_FRESH_PASS_TOKEN")

def resolve_run(
    repo_root: Path,
    manifest_path: Path,
    layout_seed: int,
    run_seed_value: int,
    require_pass_token: bool = False,
) -> dict:
    manifest_path = manifest_path.resolve()
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("manifest_state") != "FROZEN_PRE_OUTCOME":
        raise RuntimeError("MX049_MANIFEST_NOT_FROZEN")
    binding_path = repo_root / _canonical_rel(manifest["dell_binding_path"])
    if sha256_file(binding_path) != manifest["dell_binding_sha256"]:
        raise RuntimeError("MX049_DELL_BINDING_MUTATION_DETECTED")
    dell_binding = _load_json(binding_path)
    current_machine_id_sha256 = hashlib.sha256(
        Path("/etc/machine-id").read_text(encoding="utf-8").strip().encode("utf-8")
    ).hexdigest()
    if (
        dell_binding.get("authorized_machine_label") != "DELL"
        or dell_binding.get("machine_id_sha256") != current_machine_id_sha256
    ):
        raise RuntimeError("MX049_NON_DELL_EXECUTION_BLOCK")
    if require_pass_token:
        _verify_collection_token(repo_root, manifest_path, manifest, current_machine_id_sha256)
    layout = find_layout(manifest, layout_seed)
    verify_layout_identity(repo_root, layout)
    cfg = _load_json(repo_root / _canonical_rel(layout["layout_config_path"]))
    run = find_run(layout, run_seed_value)
    if run["expected_world_identity_sha256"] != layout["world_identity_sha256"]:
        raise RuntimeError("MX048_RUN_WORLD_IDENTITY_MISMATCH")
    split = layout["split"]
    return {
        "run_id": run["run_id"],
        "layout_seed": layout_seed,
        "run_seed": run_seed_value,
        "split": split,
        "primary_or_reserve": layout["primary_or_reserve"],
        "world_path": str((repo_root / layout["world_path"]).resolve()),
        "world_rel": layout["world_path"],
        "world_sha256": layout["world_sha256"],
        "layout_config_path": str((repo_root / layout["layout_config_path"]).resolve()),
        "layout_config_rel": layout["layout_config_path"],
        "layout_config_sha256": layout["layout_config_sha256"],
        "world_identity_sha256": layout["world_identity_sha256"],
        "gt_binding_path": str((repo_root / layout["gt_binding_path"]).resolve()),
        "gt_binding_rel": layout["gt_binding_path"],
        "spawn_x_m": float(cfg["spawn"]["x_cm"]) / 100.0,
        "spawn_y_m": float(cfg["spawn"]["y_cm"]) / 100.0,
        "spawn_yaw_rad": float(cfg["spawn"]["yaw_millirad"]) / 1000.0,
        "gazebo_seed": run["gazebo_seed"],
        "exploration_seed": run["exploration_seed"],
        "planner_seed": run["planner_seed"],
        "sensor_seed": run["sensor_seed"],
        "confirmation_state": "SEALED_RAW_ONLY" if split == "confirmation" else "DEVELOPMENT_RAW",
        "manifest_sha256": sha256_file(manifest_path),
        "cohort_id": COHORT_ID,
        "machine_id_sha256": current_machine_id_sha256,
        "authorized_machine_label": "DELL",
        "technical_repo_commit": dell_binding["technical_repo_commit"],
        "runner_blob": dell_binding["runner_blob"],
        "recorder_blob": dell_binding["recorder_blob"],
        "generator_blob": dell_binding["generator_blob"],
    }

def shell_exports(resolved: dict) -> str:
    mapping = {
        "MX045_RUN_ID": resolved["run_id"],
        "MX045_LAYOUT_SEED": resolved["layout_seed"],
        "MX045_RUN_SEED": resolved["run_seed"],
        "MX045_SPLIT": resolved["split"],
        "MX045_PRIMARY_OR_RESERVE": resolved["primary_or_reserve"],
        "MX045_WORLD_PATH": resolved["world_path"],
        "MX045_WORLD_REL": resolved["world_rel"],
        "MX045_WORLD_SHA256": resolved["world_sha256"],
        "MX045_LAYOUT_CONFIG_PATH": resolved["layout_config_path"],
        "MX045_LAYOUT_CONFIG_REL": resolved["layout_config_rel"],
        "MX045_LAYOUT_CONFIG_SHA256": resolved["layout_config_sha256"],
        "MX045_WORLD_IDENTITY_SHA256": resolved["world_identity_sha256"],
        "MX045_GT_BINDING_PATH": resolved["gt_binding_path"],
        "MX045_GT_BINDING_REL": resolved["gt_binding_rel"],
        "MX045_SPAWN_X_M": resolved["spawn_x_m"],
        "MX045_SPAWN_Y_M": resolved["spawn_y_m"],
        "MX045_SPAWN_YAW_RAD": resolved["spawn_yaw_rad"],
        "MX045_GAZEBO_SEED": resolved["gazebo_seed"],
        "MX045_EXPLORATION_SEED": resolved["exploration_seed"],
        "MX045_PLANNER_SEED": resolved["planner_seed"],
        "MX045_SENSOR_SEED": resolved["sensor_seed"],
        "MX045_CONFIRMATION_STATE": resolved["confirmation_state"],
        "MX045_MANIFEST_SHA256": resolved["manifest_sha256"],
        "MX049_COHORT_ID": resolved["cohort_id"],
        "MX049_MACHINE_ID_SHA256": resolved["machine_id_sha256"],
        "MX049_AUTHORIZED_MACHINE_LABEL": resolved["authorized_machine_label"],
        "MX049_TECHNICAL_REPO_COMMIT": resolved["technical_repo_commit"],
        "MX049_RUNNER_BLOB": resolved["runner_blob"],
        "MX049_RECORDER_BLOB": resolved["recorder_blob"],
        "MX049_GENERATOR_BLOB": resolved["generator_blob"],
    }
    return "\n".join(f"export {k}={shlex.quote(str(v))}" for k, v in mapping.items())

OUTCOME_NUMERIC_KINDS = {
    "coverage", "known_area", "delta_known_area", "occupancy_map",
    "scientific_numeric", "gt", "roi", "target", "evaluator",
}

def require_unsealed(kind: str, unseal_record: Path | None = None, expected: dict[str, Any] | None = None) -> None:
    if kind not in OUTCOME_NUMERIC_KINDS:
        return
    if unseal_record is None or not unseal_record.is_file():
        raise PermissionError(SEAL_ERROR)
    validate_unseal_record(unseal_record, expected or {})

def validate_unseal_record(path: Path, expected: dict[str, Any]) -> dict:
    rec = _load_json(path)
    required = [
        "contract_id", "mx045_method_commit", "mx045_method_blob",
        "mx048_contract_commit", "mx048_contract_blob",
        "cohort_id", "mx049_addendum_commit", "mx049_addendum_blob",
        "fresh_manifest_path", "fresh_manifest_sha256", "fresh_dell_binding_sha256",
        "mx046_collection_integrity_qa_accept_reference",
        "development_freeze_path", "development_freeze_sha256",
        "user_pm_authorization_reference", "confirmation_run_ids",
        "acquisition_manifest_sha256", "confirmation_gt_binding_hashes",
        "unseal_actor", "utc_timestamp", "unseal_once",
    ]
    missing = [k for k in required if k not in rec]
    if missing:
        raise PermissionError(SEAL_ERROR)
    if (
        rec["contract_id"] != GENERATOR_ID
        or rec["cohort_id"] != COHORT_ID
        or rec["mx049_addendum_commit"] != MX049_ADDENDUM_COMMIT
        or rec["mx049_addendum_blob"] != MX049_ADDENDUM_BLOB
        or rec["unseal_once"] is not True
    ):
        raise PermissionError(SEAL_ERROR)
    if not rec["development_freeze_sha256"]:
        raise PermissionError(SEAL_ERROR)
    for k, v in expected.items():
        if rec.get(k) != v:
            raise PermissionError(SEAL_ERROR)
    return rec

def consume_unseal_once(path: Path, consumption_marker: Path, expected: dict[str, Any]) -> dict:
    rec = validate_unseal_record(path, expected)
    unseal_sha = sha256_file(path)
    if consumption_marker.exists():
        old = _load_json(consumption_marker)
        if old.get("unseal_sha256") != unseal_sha:
            raise PermissionError(SEAL_ERROR)
        return rec
    write_canonical_json(consumption_marker, {"unseal_sha256": unseal_sha, "consumed": True})
    return rec


def metadata_for_csv(path: Path) -> dict:
    with path.open("r", encoding="utf-8", newline="") as f:
        header = f.readline().rstrip("\r\n").split(",") if path.stat().st_size else []
        row_count = sum(1 for _ in f)
    return {
        "artifact_relative_path": path.name,
        "exists": path.exists(),
        "byte_size": path.stat().st_size,
        "sha256": sha256_file(path),
        "schema_version": "csv",
        "header": header,
        "row_count": row_count,
    }

def metadata_for_binary(path: Path, schema_version: str) -> dict:
    return {
        "artifact_relative_path": path.name,
        "exists": path.exists(),
        "byte_size": path.stat().st_size,
        "sha256": sha256_file(path),
        "schema_version": schema_version,
    }

def reserve_decision(split: str, events: list[dict]) -> dict:
    technical = [e for e in events if e.get("technical_invalid") is True]
    scientific = [e for e in events if e.get("scientific_weakness") is True]
    if len(technical) == 0:
        return {"action": "NO_RESERVE", "split": split, "scientific_weakness_count": len(scientific)}
    primary_layouts = sorted({int(e["layout_seed"]) for e in technical if e.get("primary_or_reserve") == "primary"})
    if len(primary_layouts) > 1:
        return {"action": "GLOBAL_INSUFFICIENT_BLOCK", "split": split, "invalid_primary_layouts": primary_layouts}
    failed = primary_layouts[0] if primary_layouts else None
    if failed is None:
        return {"action": "GLOBAL_INSUFFICIENT_BLOCK", "split": split, "reason": "reserve_failed"}
    reserve = 49011 if split == "development" else 49012
    return {"action": "ACTIVATE_WHOLE_LAYOUT_RESERVE", "split": split, "excluded_layout": failed, "reserve_layout": reserve, "required_run_seeds": [1,2]}

def run_provenance_from_args(args: argparse.Namespace) -> dict:
    component_seeds = {
        "gazebo": int(args.gazebo_seed), "exploration": int(args.exploration_seed),
        "planner": int(args.planner_seed), "sensor": int(args.sensor_seed),
    }
    actual_world_sha = sha256_file(Path(args.actual_world))
    actual_layout_sha = sha256_file(Path(args.layout_config))
    if args.expected_world_sha256 and actual_world_sha != args.expected_world_sha256:
        raise RuntimeError("MX049_RECORDER_WORLD_HASH_MISMATCH")
    if args.expected_layout_config_sha256 and actual_layout_sha != args.expected_layout_config_sha256:
        raise RuntimeError("MX049_RECORDER_LAYOUT_CONFIG_HASH_MISMATCH")
    return {
        "run_id": args.run_id, "layout_seed": int(args.layout_seed), "run_seed": int(args.run_seed),
        "split": args.split, "primary_or_reserve": args.primary_or_reserve,
        "world_identity_sha256": args.world_identity_sha256,
        "environment_world": args.actual_world_rel,
        "environment_world_resolved_runtime": str(Path(args.actual_world).resolve()),
        "actual_world_sha256": actual_world_sha,
        "expected_world_sha256": args.expected_world_sha256,
        "layout_config_path": args.layout_config_rel,
        "layout_config_resolved_runtime": str(Path(args.layout_config).resolve()),
        "layout_config_sha256": actual_layout_sha,
        "expected_layout_config_sha256": args.expected_layout_config_sha256,
        "gt_binding_path": args.gt_binding_rel,
        "gt_binding_path_resolved_runtime": str(Path(args.gt_binding).resolve()),
        "launch_spawn": {"x_m": float(args.spawn_x_m), "y_m": float(args.spawn_y_m), "yaw_rad": float(args.spawn_yaw_rad)},
        "component_seeds": component_seeds,
        "acquisition_only": bool(args.acquisition_only),
        "confirmation_state": args.confirmation_state,
        "manifest_sha256": args.manifest_sha256,
        "telemetry_schema_version": "MX048_RAW_TELEMETRY_V1",
        "cohort_id": args.cohort_id,
        "producer_machine_id_sha256": args.machine_id_sha256,
        "creation_write_host_label": args.authorized_machine_label,
        "technical_repo_commit": args.technical_repo_commit,
        "runner_blob": args.runner_blob,
        "recorder_blob": args.recorder_blob,
        "generator_blob": args.generator_blob,
    }

def add_run_provenance_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--actual-world")
    parser.add_argument("--actual-world-rel")
    parser.add_argument("--expected-world-sha256")
    parser.add_argument("--layout-config")
    parser.add_argument("--layout-config-rel")
    parser.add_argument("--expected-layout-config-sha256")
    parser.add_argument("--world-identity-sha256")
    parser.add_argument("--gt-binding")
    parser.add_argument("--gt-binding-rel")
    parser.add_argument("--layout-seed", type=int)
    parser.add_argument("--run-seed", type=int)
    parser.add_argument("--split")
    parser.add_argument("--primary-or-reserve")
    parser.add_argument("--gazebo-seed", type=int)
    parser.add_argument("--exploration-seed", type=int)
    parser.add_argument("--planner-seed", type=int)
    parser.add_argument("--sensor-seed", type=int)
    parser.add_argument("--manifest-sha256")
    parser.add_argument("--spawn-x-m", type=float, default=0.0)
    parser.add_argument("--spawn-y-m", type=float, default=3.0)
    parser.add_argument("--spawn-yaw-rad", type=float, default=0.0)
    parser.add_argument("--confirmation-state", default="DEVELOPMENT_RAW")
    parser.add_argument("--acquisition-only", action="store_true")
    parser.add_argument("--cohort-id")
    parser.add_argument("--machine-id-sha256")
    parser.add_argument("--authorized-machine-label")
    parser.add_argument("--technical-repo-commit")
    parser.add_argument("--runner-blob")
    parser.add_argument("--recorder-blob")
    parser.add_argument("--generator-blob")

def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("derive")
    p.add_argument("--layout-seed", type=int, required=True)
    p.add_argument("--run-seed", type=int, required=True)
    p = sub.add_parser("resolve-run")
    p.add_argument("--repo-root", type=Path, required=True)
    p.add_argument("--manifest", type=Path, required=True)
    p.add_argument("--layout-seed", type=int, required=True)
    p.add_argument("--run-seed", type=int, required=True)
    p.add_argument("--shell", action="store_true")
    p.add_argument("--require-pass-token", action="store_true")
    p = sub.add_parser("guard")
    p.add_argument("--kind", required=True)
    p.add_argument("--unseal-record", type=Path)
    args = parser.parse_args()
    if args.cmd == "derive":
        print(json.dumps({t:derive_seed32(args.layout_seed,args.run_seed,t) for t in COMPONENT_TAGS}, sort_keys=True))
    elif args.cmd == "resolve-run":
        out = resolve_run(
            args.repo_root.resolve(), args.manifest, args.layout_seed, args.run_seed,
            require_pass_token=args.require_pass_token,
        )
        print(shell_exports(out) if args.shell else json.dumps(out, indent=2, sort_keys=True))
    elif args.cmd == "guard":
        require_unsealed(args.kind, args.unseal_record)
        print("UNSEALED_ACCESS_GRANTED")

if __name__ == "__main__":
    main()
