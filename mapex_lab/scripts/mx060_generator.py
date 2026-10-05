#!/usr/bin/env python3
"""MX060 namespace adapter for the definition-identical MX048 generator."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import mx049_generator as core
from mx060_contract import COHORT_ID, GENERATOR_ID, LAYOUT_SEEDS, NAMESPACE, sha256_file

GENERATED_WORLD_REL = "mapex_lab/map/generated/mx060_com1_pilot/layout_{seed}/world.sdf"
GEOMETRY_REL = "mapex_lab/map/generated/mx060_com1_pilot/layout_{seed}/geometry_params.json"
LAYOUT_CONFIG_REL = "mapex_lab/map/generated/mx060_com1_pilot/layout_{seed}/layout_config.json"
GEN_AUDIT_REL = "mapex_lab/map/generated/mx060_com1_pilot/layout_{seed}/generation_audit.json"
GEN_RUNTIME_REL = "mapex_lab/map/generated/mx060_com1_pilot/layout_{seed}/generation_runtime_provenance.json"
GT_REL = "mapex_lab/ground_truth/mx060_com1_pilot/layout_{seed}/sealed/structural_gt_v2.npz"
ROI_REL = "mapex_lab/ground_truth/mx060_com1_pilot/layout_{seed}/sealed/connected_free_roi_v2.npy"
GT_SUMMARY_REL = "mapex_lab/ground_truth/mx060_com1_pilot/layout_{seed}/sealed/structural_gt_v2_summary.json"
GT_BINDING_REL = "mapex_lab/ground_truth/mx060_com1_pilot/layout_{seed}/sealed/gt_binding.json"


def _role(seed: int) -> tuple[str, str]:
    if seed not in LAYOUT_SEEDS:
        raise ValueError("MX060_SEED_DOMAIN_VIOLATION")
    return ("pilot", "reserve" if seed == 60004 else "primary")


def _bind_core() -> None:
    # Only cohort/identity paths differ; draw order, PRNG and V1-V9 remain MX048.
    for name, value in {
        "GENERATOR_ID": GENERATOR_ID, "COHORT_ID": COHORT_ID, "NAMESPACE": NAMESPACE,
        "LAYOUT_SEEDS": LAYOUT_SEEDS, "PRIMARY_DEVELOPMENT": LAYOUT_SEEDS[:3],
        "PRIMARY_CONFIRMATION": (), "RESERVE_DEVELOPMENT": (60004,), "RESERVE_CONFIRMATION": (),
        "GENERATED_WORLD_REL": GENERATED_WORLD_REL, "GEOMETRY_REL": GEOMETRY_REL,
        "LAYOUT_CONFIG_REL": LAYOUT_CONFIG_REL, "GEN_AUDIT_REL": GEN_AUDIT_REL,
        "GEN_RUNTIME_REL": GEN_RUNTIME_REL, "GT_REL": GT_REL, "ROI_REL": ROI_REL,
        "GT_SUMMARY_REL": GT_SUMMARY_REL, "GT_BINDING_REL": GT_BINDING_REL,
    }.items():
        setattr(core, name, value)
    core.split_role = _role


def generate_set(repo_root: Path, seeds: tuple[int, ...], denylist: set[str]) -> list[dict]:
    if any(seed not in LAYOUT_SEEDS for seed in seeds):
        raise ValueError("MX060_SEED_DOMAIN_VIOLATION")
    _bind_core()
    root = repo_root.resolve()
    generator_blob = core.git_value(root, "hash-object", str(Path(__file__).resolve()))
    commit = core.git_value(root, "rev-parse", "HEAD")
    gt_blob = core.git_value(root, "hash-object", str(root / "mapex_lab/scripts/generate_new_room_ground_truth.py"))
    accepted: set[str] = set()
    return [core.generate_one(root, seed, accepted, commit, generator_blob, gt_blob, denylist) for seed in sorted(seeds)]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--seed", type=int, action="append")
    parser.add_argument("--denylist", type=Path, required=True)
    parser.add_argument("--audit-csv", type=Path)
    args = parser.parse_args()
    seeds = tuple(args.seed) if args.seed else LAYOUT_SEEDS
    doc = json.loads(args.denylist.read_text(encoding="utf-8"))
    denylist = {str(x["geometry_parameter_digest"]) for x in doc["generated_geometry_entries"]}
    results = generate_set(args.repo_root, seeds, denylist)
    if args.audit_csv:
        args.audit_csv.parent.mkdir(parents=True, exist_ok=True)
        keys = sorted({key for row in results for key in row})
        with args.audit_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=keys)
            writer.writeheader(); writer.writerows(results)
    print(json.dumps(results, indent=2, sort_keys=True))
    if any(row["status"] != "ACCEPTED" for row in results):
        raise SystemExit(2)


if __name__ == "__main__":
    main()

