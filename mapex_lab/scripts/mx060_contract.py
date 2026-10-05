#!/usr/bin/env python3
"""Frozen, outcome-blind MX060 Method R2 contract primitives."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Iterable

COHORT_ID = "MX060_COM1_PILOT_V1"
NAMESPACE = "mx060_com1_pilot"
LAYOUT_SEEDS = (60001, 60002, 60003, 60004)
RUN_SEEDS = (1, 2)
ACTIVE_SUBSETS = (
    (60001, 60002, 60003),
    (60001, 60002, 60004),
    (60001, 60003, 60004),
    (60002, 60003, 60004),
)
COMPONENT_TAGS = ("gazebo", "exploration", "planner", "sensor")

MX060_METHOD_COMMIT = "f693e7d90ac0f8d597a1d3cd7a36c3f051a9ef92"
MX060_METHOD_BLOB = "ef24d94f9bcee64a240d437362d6914075e9514e"
MX061_METHOD_COMMIT = "fbc4bb6ddb416ea23907aa7b2169342aad9b4af3"
MX061_METHOD_BLOB = "ca9aa37d1c00a03f59414c9fc236fcd2ff775b07"
MX048_CONTRACT_COMMIT = "928be24d768f821f20155864d641e78bf9b31dcd"
MX048_CONTRACT_BLOB = "731c53555318efa079629485b3622d54a2660c84"
GENERATOR_ID = "MX048_ORTHO_NEW_ROOM_V1"

ENSEMBLE_SHA256 = {
    "G1": "b37bfef69138806708d6e087b8db89836021f06db987cb3ef3db21fbf28422ca",
    "G2": "7880d164ccd88da58ddb0e6875f358bea02a232c452eff86b5e226957a489c26",
    "G3": "fcb43d1102d48ab6b14f5bfded859077d6da0c7efbfbda10d4acbb754c3d3bc1",
}

PARAMETER_ORDER = (
    "y_top_cm", "y_bottom_cm",
    "top_left_door_center_x_cm", "top_left_door_width_cm",
    "top_center_door_center_x_cm", "top_center_door_width_cm",
    "top_right_door_center_x_cm", "top_right_door_width_cm",
    "bottom_left_door_center_x_cm", "bottom_left_door_width_cm",
    "bottom_center_door_center_x_cm", "bottom_center_door_width_cm",
    "bottom_right_door_center_x_cm", "bottom_right_door_width_cm",
    "x_top_left_cm", "x_top_right_cm", "x_bottom_left_cm", "x_bottom_right_cm",
    "top_left_divider_door_center_y_cm", "top_left_divider_door_width_cm",
    "top_right_divider_door_center_y_cm", "top_right_divider_door_width_cm",
    "bottom_left_divider_door_center_y_cm", "bottom_left_divider_door_width_cm",
    "bottom_right_divider_door_center_y_cm", "bottom_right_divider_door_width_cm",
) + tuple(
    key
    for index in range(1, 9)
    for key in (
        f"obs{index:02d}_width_x_cm", f"obs{index:02d}_width_y_cm",
        f"obs{index:02d}_x_cm", f"obs{index:02d}_y_cm",
    )
)
PARTITION_KEYS = (
    "y_top_cm", "y_bottom_cm", "x_top_left_cm", "x_top_right_cm",
    "x_bottom_left_cm", "x_bottom_right_cm",
)
DOOR_KEYS = tuple(k for k in PARAMETER_ORDER if "door" in k)
OBSTACLE_KEYS = tuple(k for k in PARAMETER_ORDER if k.startswith("obs"))


def canonical_json_bytes(value: Any) -> bytes:
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True) + "\n").encode("ascii")


def write_canonical_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_json_bytes(value))


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def derive_seed32(layout_seed: int, run_seed: int, tag: str) -> int:
    if layout_seed not in LAYOUT_SEEDS or run_seed not in RUN_SEEDS or tag not in COMPONENT_TAGS:
        raise ValueError("MX060_SEED_DOMAIN_VIOLATION")
    key = f"MX060_RUN_V1|{layout_seed}|{run_seed}|{tag}".encode("ascii")
    return 1 + (int.from_bytes(hashlib.sha256(key).digest()[:4], "big") % 2147483646)


def run_id(layout_seed: int, run_seed: int) -> str:
    derive_seed32(layout_seed, run_seed, "gazebo")
    return f"mx060_com1_l{layout_seed}_r{run_seed}"


def pairwise_diversity(a: dict[str, int], b: dict[str, int]) -> dict[str, Any]:
    missing = [k for k in PARAMETER_ORDER if k not in a or k not in b]
    if missing:
        raise ValueError(f"MX060_GEOMETRY_PARAMETER_MISSING:{','.join(missing)}")
    partition = sum(a[k] != b[k] for k in PARTITION_KEYS)
    doors = sum(a[k] != b[k] for k in DOOR_KEYS)
    obstacles = sum(a[k] != b[k] for k in OBSTACLE_KEYS)
    overall = sum(a[k] != b[k] for k in PARAMETER_ORDER)
    checks = {
        "partition_differences_gte_1": partition >= 1,
        "door_differences_gte_4": doors >= 4,
        "obstacle_differences_gte_8": obstacles >= 8,
        "ordered_parameter_differences_gte_16": overall >= 16,
    }
    return {
        "partition_differences": partition,
        "door_differences": doors,
        "obstacle_differences": obstacles,
        "ordered_parameter_differences": overall,
        "checks": checks,
        "status": "PASS" if all(checks.values()) else "FAIL",
    }


def select_active_subset(layouts: dict[int, dict[str, Any]]) -> tuple[int, int, int]:
    for subset in ACTIVE_SUBSETS:
        if all(layouts.get(seed, {}).get("admissible") is True for seed in subset):
            pairs = ((subset[0], subset[1]), (subset[0], subset[2]), (subset[1], subset[2]))
            if all(layouts[a]["pairwise"][str(b)]["status"] == "PASS" for a, b in pairs):
                return subset
    raise RuntimeError("MX060_PREFLIGHT_BLOCK_LAYOUT_SET")


def acquisition_only_allows(action: str) -> bool:
    return action in {"record_runtime_source", "record_content_neutral_health", "ordinary_exploration"}


def metadata_only(value: Any, path: str = "root") -> None:
    forbidden = {"min", "max", "mean", "histogram", "coverage", "cv_q1", "cv_q3", "t4_full", "quality", "cost"}
    if isinstance(value, dict):
        for key, child in value.items():
            if str(key).lower() in forbidden:
                raise RuntimeError(f"MX060_SEALED_NUMERIC_PAYLOAD:{path}.{key}")
            metadata_only(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            metadata_only(child, f"{path}[{index}]")


def retry_disposition(attempt: int, global_retries_used: int, technical: bool, requires_change: bool) -> str:
    if not technical:
        return "RETAIN_NO_RETRY"
    if requires_change:
        return "BLOCK_RECOVERY_REVIEW"
    if attempt == 1 and global_retries_used < 2:
        return "ALLOW_EXACT_ATTEMPT02"
    if attempt == 1:
        return "EXCLUDE_LAYOUT_OR_TECHNICAL_INSUFFICIENT"
    if attempt == 2:
        return "EXCLUDE_WHOLE_LAYOUT"
    raise ValueError("MX060_ATTEMPT_DOMAIN_VIOLATION")


def validate_decision_sequence(rows: Iterable[dict[str, Any]]) -> None:
    sequence = list(rows)
    for index, row in enumerate(sequence):
        if row.get("ordinal") != index or row.get("decision_id") is None:
            raise RuntimeError("MX060_DECISION_SEQUENCE_INVALID")
        expected_prev = "RUN_START" if index == 0 else sequence[index - 1]["decision_id"]
        expected_next = "TERMINAL" if index == len(sequence) - 1 else sequence[index + 1]["decision_id"]
        if row.get("predecessor_decision_id") != expected_prev or row.get("successor_decision_id") != expected_next:
            raise RuntimeError("MX060_DECISION_SEQUENCE_INVALID")
        if not isinstance(row.get("decision_exploration_time_s"), (int, float)):
            raise RuntimeError("MX060_SCIENTIFIC_CLOCK_INVALID")
        if not isinstance(row.get("cumulative_path_m"), (int, float)):
            raise RuntimeError("MX060_ODOMETRY_COST_INVALID")
        if index:
            if row["decision_exploration_time_s"] <= sequence[index - 1]["decision_exploration_time_s"]:
                raise RuntimeError("MX060_SCIENTIFIC_CLOCK_INVALID")
            if row["cumulative_path_m"] < sequence[index - 1]["cumulative_path_m"]:
                raise RuntimeError("MX060_ODOMETRY_COST_INVALID")
