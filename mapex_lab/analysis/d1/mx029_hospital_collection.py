#!/usr/bin/env python3
"""Integrity-only preflight and inventory helpers for MX029 Hospital collection."""
from __future__ import annotations

import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import numpy as np


GT_SHA256 = "080c7d708f12ae71c1ed1881bfd630dbb491401d95831f613e3116cb9926dce1"
GT_SEMANTIC_DIGEST = "d27ba692b313ba784729ea1fd10b2963c20fea9e4465d27081ece9c8757cf4ea"
ROI_SHA256 = "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"
MX018_REVISION = "27fad5306f6a2868f93a269b82719fb189d77ebd"
GT_RELATIVE = Path(
    "mapex_lab/analysis/d1/results/mx018_hospital_gt_recovery_v1/gt_v2"
)
REQUIRED_DECISION_ARTIFACTS = (
    "raw_map", "canvas_map", "g1_map", "g2_map", "g3_map", "mean_map", "variance_map"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def scalar(archive: np.lib.npyio.NpzFile, key: str):
    return np.asarray(archive[key]).reshape(()).item()


def local_path(run: Path, relative: str) -> Path:
    path = (run / relative).resolve()
    if run.resolve() not in path.parents:
        raise ValueError(f"artifact escapes run directory: {relative}")
    return path


def load_runtime_artifacts(run: Path, row: dict[str, str]) -> tuple[np.ndarray, list[np.ndarray], np.ndarray]:
    decision_id = int(row["decision_id"])
    with np.load(local_path(run, row["raw_map"]), allow_pickle=False) as archive:
        raw = np.asarray(archive["data"])
        raw_meta = {
            name: scalar(archive, name)
            for name in ("resolution", "origin_x", "origin_y", "source_stamp_s")
        }
        if raw.ndim != 2 or raw.shape != (int(scalar(archive, "height")), int(scalar(archive, "width"))):
            raise ValueError(f"decision {decision_id}: invalid raw map geometry")

    members: list[np.ndarray] = []
    variance = None
    for field, member in zip(
        ("g1_map", "g2_map", "g3_map", "mean_map", "variance_map"),
        ("G1", "G2", "G3", "mean", "variance"),
    ):
        path = local_path(run, row[field])
        if path.name != f"decision_{decision_id:06d}_{member.lower()}.npz":
            raise ValueError(f"decision {decision_id}: wrong {member} identity")
        with np.load(path, allow_pickle=False) as archive:
            height, width, top, left = (
                int(scalar(archive, key))
                for key in ("source_height", "source_width", "pad_top", "pad_left")
            )
            data = np.asarray(archive["data"])
            cropped = data[top:top + height, left:left + width]
            if cropped.shape != raw.shape or not np.all(np.isfinite(cropped)):
                raise ValueError(f"decision {decision_id}: invalid {member} crop")
            if scalar(archive, "member") != member:
                raise ValueError(f"decision {decision_id}: wrong {member} metadata")
            for key, expected in raw_meta.items():
                prediction_key = "source_map_stamp_s" if key == "source_stamp_s" else key
                if scalar(archive, prediction_key) != expected:
                    raise ValueError(f"decision {decision_id}: {member} {key} mismatch")
            if member in {"G1", "G2", "G3"}:
                members.append(cropped)
            elif member == "variance":
                variance = cropped
    if variance is None:
        raise ValueError(f"decision {decision_id}: missing variance")
    return raw, members, variance


def check(condition: bool, evidence: dict, reason: str = "") -> dict:
    return {"status": "PASS" if condition else "FAIL", "reason": reason, "evidence": evidence}


def audit_hpx001(repo_root: Path) -> dict:
    run = repo_root / "mapex_lab/experiments/mapex/hpx_001"
    gt = repo_root / GT_RELATIVE
    metadata = json.loads((run / "metadata.json").read_text(encoding="utf-8"))
    summary = json.loads((run / "summary.json").read_text(encoding="utf-8"))
    decisions = read_csv(run / "decisions.csv")
    policy = read_csv(run / "policy_decisions.csv")
    trajectory = read_csv(run / "trajectory.csv")
    ids = [int(row["decision_id"]) for row in decisions]
    policy_ids = [int(row["policy_decision_id"]) for row in policy]

    missing: list[str] = []
    runtime_errors: list[str] = []
    variance_failures: list[int] = []
    candidate_errors: list[str] = []
    selection_reasons = Counter()
    for row in decisions:
        decision_id = int(row["decision_id"])
        for field in REQUIRED_DECISION_ARTIFACTS:
            path = local_path(run, row[field])
            if not path.is_file():
                missing.append(row[field])
        try:
            raw, members, variance = load_runtime_artifacts(run, row)
            recomputed = np.var(np.stack(members), axis=0, ddof=1)
            recomputed[raw >= 0] = 0
            if not np.allclose(variance, recomputed, rtol=1e-5, atol=1e-7):
                variance_failures.append(decision_id)
        except (KeyError, OSError, ValueError) as exc:
            runtime_errors.append(f"d{decision_id}:{type(exc).__name__}:{exc}")

        policy_id = int(row["mapex_policy_decision_id"])
        table_path = run / f"decisions/policy_decision_{policy_id:06d}/candidates.csv"
        table = read_csv(table_path)
        selected = [candidate for candidate in table if candidate["selected"] == "1"]
        if len(table) != int(row["candidate_total"]):
            candidate_errors.append(f"d{decision_id}:candidate_total")
        if row["outcome"] == "selected" and len(selected) != 1:
            candidate_errors.append(f"d{decision_id}:selected_cardinality")
        if row["outcome"] != "selected" and selected:
            candidate_errors.append(f"d{decision_id}:unexpected_selection")
        selection_reasons[row["outcome"]] += 1

    json_files = sorted(run.rglob("*.json"))
    csv_files = sorted(run.rglob("*.csv"))
    npz_files = sorted(run.rglob("*.npz"))
    parse_errors: list[str] = []
    for path in json_files:
        try:
            json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            parse_errors.append(f"{path.relative_to(run)}:{exc}")
    for path in csv_files:
        try:
            read_csv(path)
        except (OSError, ValueError, csv.Error) as exc:
            parse_errors.append(f"{path.relative_to(run)}:{exc}")
    for path in npz_files:
        try:
            with np.load(path, allow_pickle=False) as archive:
                for name in archive.files:
                    if np.asarray(archive[name]).dtype.hasobject:
                        raise ValueError(f"object dtype in {name}")
        except (OSError, ValueError, KeyError) as exc:
            parse_errors.append(f"{path.relative_to(run)}:{exc}")

    provenance_keys = {
        "git_commit", "protocol_version", "runtime_profile", "runtime_launch_file",
        "runtime_map_resolution_m", "mapex_reference_checkout_commit", "ensemble_checkpoints",
        "sim_seed_policy", "config_sha256", "termination_reason",
    }
    config_keys = {
        "mapex_run", "mapex_evaluator", "mapex_profiled_evaluator", "mapex_policy",
        "mapex_lama_bridge", "mapex_lama_worker", "nf_run_recorder_base",
        "nf_basic_shared_execution", "mapex_config", "runtime_mapex", "nav2_override",
        "runtime_nav2_merged", "runtime_launch", "hospital_scale", "hospital_world",
    }
    checks = {
        "decision_trajectory_and_ids": check(
            ids == list(range(1, 57))
            and policy_ids == list(range(1, 57))
            and len(trajectory) == 3638
            and all(math.isfinite(float(row["time_s"])) for row in trajectory)
            and all(row.get("robot_x") and row.get("robot_y") for row in policy),
            {"decision_count": len(ids), "policy_decision_count": len(policy), "trajectory_rows": len(trajectory)},
        ),
        "raw_maps_and_time": check(
            not missing and not runtime_errors and all(math.isfinite(float(row["time_s"])) for row in decisions),
            {"raw_map_count": len(decisions), "time_s_count": len(decisions), "errors": runtime_errors},
        ),
        "saved_mapex_predictions": check(
            not missing and not runtime_errors,
            {"decision_count": len(decisions), "artifacts_per_decision": list(REQUIRED_DECISION_ARTIFACTS)},
        ),
        "original_uncertainty_inputs": check(
            not variance_failures and not runtime_errors,
            {"variance_parity_pass": len(decisions) - len(variance_failures), "variance_parity_failures": variance_failures},
        ),
        "candidate_frontier_policy_metadata": check(
            not candidate_errors,
            {"candidate_rows": len(read_csv(run / "candidates.csv")), "outcomes": dict(selection_reasons), "errors": candidate_errors},
        ),
        "runtime_config_model_policy_world_launch_provenance": check(
            provenance_keys <= metadata.keys()
            and config_keys <= metadata.get("config_sha256", {}).keys()
            and len(metadata.get("ensemble_checkpoints", [])) == 3
            and summary.get("termination_reason") == "exploration_complete",
            {
                "run_git_commit": metadata.get("git_commit"),
                "git_dirty_at_recorder_start": metadata.get("git_dirty_at_recorder_start"),
                "mapex_reference_commit": metadata.get("mapex_reference_checkout_commit"),
                "config_hash_count": len(metadata.get("config_sha256", {})),
                "checkpoint_hashes_present": all(item.get("sha256") for item in metadata.get("ensemble_checkpoints", [])),
                "termination_reason": summary.get("termination_reason"),
            },
        ),
        "hospital_gt_v2_alignment_inputs": check(
            sha256(gt / "hospital_structural_gt_v2.npz") == GT_SHA256
            and sha256(gt / "hospital_connected_free_v2.npy") == ROI_SHA256
            and json.loads((gt / "gt_v2_semantic_manifest.json").read_text())["semantic_digest"] == GT_SEMANTIC_DIGEST
            and json.loads((gt / "alignment_validation.json").read_text())["all_decisions_pass"],
            {
                "source_revision": MX018_REVISION,
                "gt_sha256": GT_SHA256,
                "semantic_digest": GT_SEMANTIC_DIGEST,
                "alignment_decisions": 56,
            },
        ),
        "file_integrity_and_future_survey_inputs": check(
            not parse_errors and not missing and not runtime_errors and not candidate_errors,
            {
                "file_count": sum(1 for path in run.rglob("*") if path.is_file()),
                "json_files": len(json_files), "csv_files": len(csv_files), "npz_files": len(npz_files),
                "parse_errors": parse_errors,
            },
        ),
    }
    verdict = "HPX001_DATA_REUSE_PASS" if all(item["status"] == "PASS" for item in checks.values()) else "HPX001_DATA_REUSE_PARTIAL"
    return {
        "schema": "mx029_hpx001_preflight_v1",
        "task": "MX029",
        "run_id": "hpx_001",
        "verdict": verdict,
        "cohort_label": "LEGACY_RETROSPECTIVE_GT_V2",
        "scientific_pooling_authorized": False,
        "checks": checks,
    }


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = audit_hpx001(args.repo_root.resolve())
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    print(payload, end="")
    return 0 if report["verdict"] == "HPX001_DATA_REUSE_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
