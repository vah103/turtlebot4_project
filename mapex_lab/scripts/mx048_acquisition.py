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

from mx048_generator import (
    GENERATOR_ID, LAYOUT_SEEDS, RUN_SEEDS,
    MX045_METHOD_COMMIT, MX045_METHOD_BLOB,
    MX048_CONTRACT_COMMIT, MX048_CONTRACT_BLOB,
    split_role, canonical_json_bytes, write_canonical_json, sha256_file, git_value,
)

SEAL_ERROR = "CONFIRMATION_NUMERIC_CONTENT_SEALED"
MANIFEST_SCHEMA = "MX048_ACQ_MANIFEST_V1"
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
    return f"mx045_l{layout_seed}_r{run_seed}"

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
    generator_path = repo_root / "mapex_lab/scripts/mx048_generator.py"
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
        "mx045_method_commit": MX045_METHOD_COMMIT,
        "mx045_method_blob": MX045_METHOD_BLOB,
        "mx048_contract_commit": MX048_CONTRACT_COMMIT,
        "mx048_contract_blob": MX048_CONTRACT_BLOB,
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

def resolve_run(repo_root: Path, manifest_path: Path, layout_seed: int, run_seed_value: int) -> dict:
    manifest_path = manifest_path.resolve()
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("manifest_state") != "FROZEN_PRE_OUTCOME":
        raise RuntimeError("MX048_MANIFEST_NOT_FROZEN")
    layout = find_layout(manifest, layout_seed)
    verify_layout_identity(repo_root, layout)
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
        "gazebo_seed": run["gazebo_seed"],
        "exploration_seed": run["exploration_seed"],
        "planner_seed": run["planner_seed"],
        "sensor_seed": run["sensor_seed"],
        "confirmation_state": "SEALED_RAW_ONLY" if split == "confirmation" else "DEVELOPMENT_RAW",
        "manifest_sha256": sha256_file(manifest_path),
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
        "MX045_GAZEBO_SEED": resolved["gazebo_seed"],
        "MX045_EXPLORATION_SEED": resolved["exploration_seed"],
        "MX045_PLANNER_SEED": resolved["planner_seed"],
        "MX045_SENSOR_SEED": resolved["sensor_seed"],
        "MX045_CONFIRMATION_STATE": resolved["confirmation_state"],
        "MX045_MANIFEST_SHA256": resolved["manifest_sha256"],
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
        "mx046_collection_integrity_qa_accept_reference",
        "development_freeze_path", "development_freeze_sha256",
        "user_pm_authorization_reference", "confirmation_run_ids",
        "acquisition_manifest_sha256", "confirmation_gt_binding_hashes",
        "unseal_actor", "utc_timestamp", "unseal_once",
    ]
    missing = [k for k in required if k not in rec]
    if missing:
        raise PermissionError(SEAL_ERROR)
    if rec["contract_id"] != GENERATOR_ID or rec["unseal_once"] is not True:
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
    reserve = 45011 if split == "development" else 45012
    return {"action": "ACTIVATE_WHOLE_LAYOUT_RESERVE", "split": split, "excluded_layout": failed, "reserve_layout": reserve, "required_run_seeds": [1,2]}

def run_provenance_from_args(args: argparse.Namespace) -> dict:
    component_seeds = {
        "gazebo": int(args.gazebo_seed), "exploration": int(args.exploration_seed),
        "planner": int(args.planner_seed), "sensor": int(args.sensor_seed),
    }
    return {
        "run_id": args.run_id, "layout_seed": int(args.layout_seed), "run_seed": int(args.run_seed),
        "split": args.split, "primary_or_reserve": args.primary_or_reserve,
        "world_identity_sha256": args.world_identity_sha256,
        "environment_world": args.actual_world_rel,
        "environment_world_resolved_runtime": str(Path(args.actual_world).resolve()),
        "actual_world_sha256": sha256_file(Path(args.actual_world)),
        "layout_config_path": args.layout_config_rel,
        "layout_config_resolved_runtime": str(Path(args.layout_config).resolve()),
        "layout_config_sha256": sha256_file(Path(args.layout_config)),
        "gt_binding_path": args.gt_binding_rel,
        "gt_binding_path_resolved_runtime": str(Path(args.gt_binding).resolve()),
        "component_seeds": component_seeds,
        "acquisition_only": bool(args.acquisition_only),
        "confirmation_state": args.confirmation_state,
        "manifest_sha256": args.manifest_sha256,
        "telemetry_schema_version": "MX048_RAW_TELEMETRY_V1",
    }

def add_run_provenance_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--actual-world")
    parser.add_argument("--actual-world-rel")
    parser.add_argument("--layout-config")
    parser.add_argument("--layout-config-rel")
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
    parser.add_argument("--confirmation-state", default="DEVELOPMENT_RAW")
    parser.add_argument("--acquisition-only", action="store_true")

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
    p = sub.add_parser("guard")
    p.add_argument("--kind", required=True)
    p.add_argument("--unseal-record", type=Path)
    args = parser.parse_args()
    if args.cmd == "derive":
        print(json.dumps({t:derive_seed32(args.layout_seed,args.run_seed,t) for t in COMPONENT_TAGS}, sort_keys=True))
    elif args.cmd == "resolve-run":
        out = resolve_run(args.repo_root.resolve(), args.manifest, args.layout_seed, args.run_seed)
        print(shell_exports(out) if args.shell else json.dumps(out, indent=2, sort_keys=True))
    elif args.cmd == "guard":
        require_unsealed(args.kind, args.unseal_record)
        print("UNSEALED_ACCESS_GRANTED")

if __name__ == "__main__":
    main()
