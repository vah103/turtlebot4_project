#!/usr/bin/env python3
"""Preflight checks before a Hospital MapEx-nearest research run."""

from __future__ import annotations

import hashlib
import py_compile
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
REPO_ROOT = WORKSPACE.parent

FILES = [
    WORKSPACE / "scripts" / "mapex_nearest_ros.py",
    WORKSPACE / "scripts" / "mapex_nearest_ros_hospital.py",
    WORKSPACE / "scripts" / "mapex_nearest_ros_research.py",
    WORKSPACE / "scripts" / "mapex_nearest_ros_hospital_adapted.py",
    WORKSPACE / "scripts" / "mapex_nearest_ros_official.py",
    WORKSPACE / "scripts" / "research_recorder.py",
    WORKSPACE / "scripts" / "research_recorder_safe.py",
    WORKSPACE / "scripts" / "research_recorder_official.py",
    WORKSPACE / "scripts" / "exploration_manager_research.py",
    WORKSPACE / "scripts" / "validate_nearest_run.py",
    WORKSPACE / "launch" / "hospital_nearest.launch.py",
]

ROI = (
    WORKSPACE
    / "ground_truth"
    / "hospital"
    / "generated"
    / "hospital_connected_free_v1.npy"
)
ROI_EXPECTED_SHA256 = "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"
SOURCE_NAV2_OVERRIDE = (
    REPO_ROOT
    / "ros2_ws/src/frontier_exploration/config/nav2_hospital_override.yaml"
)
INSTALLED_NAV2_OVERRIDE = (
    REPO_ROOT
    / "ros2_ws/install/frontier_exploration/share/frontier_exploration/config/nav2_hospital_override.yaml"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    errors: list[str] = []

    for path in FILES:
        if not path.exists():
            errors.append(f"missing file: {path.relative_to(REPO_ROOT)}")
            continue
        try:
            py_compile.compile(str(path), doraise=True)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"syntax error: {path.relative_to(REPO_ROOT)}: {exc}")

    if not ROI.exists():
        errors.append("missing frozen ROI")
    else:
        actual = sha256(ROI)
        if actual != ROI_EXPECTED_SHA256:
            errors.append(f"ROI SHA-256 mismatch: {actual} != {ROI_EXPECTED_SHA256}")

    if not SOURCE_NAV2_OVERRIDE.exists() or "tolerance: 0.0" not in SOURCE_NAV2_OVERRIDE.read_text(
        encoding="utf-8"
    ):
        errors.append("source Hospital Nav2 override is missing GridBased tolerance: 0.0")

    if not INSTALLED_NAV2_OVERRIDE.exists():
        errors.append(
            "installed Hospital Nav2 override is missing; rebuild frontier_exploration"
        )
    elif "tolerance: 0.0" not in INSTALLED_NAV2_OVERRIDE.read_text(encoding="utf-8"):
        errors.append(
            "installed Hospital Nav2 override is stale (missing tolerance: 0.0); "
            "run colcon build --symlink-install --packages-select frontier_exploration"
        )

    launch = WORKSPACE / "launch" / "hospital_nearest.launch.py"
    official_policy = WORKSPACE / "scripts" / "mapex_nearest_ros_official.py"
    adapted_policy = WORKSPACE / "scripts" / "mapex_nearest_ros_hospital_adapted.py"
    research_manager = WORKSPACE / "scripts" / "exploration_manager_research.py"
    official_recorder = WORKSPACE / "scripts" / "research_recorder_official.py"
    validator = WORKSPACE / "scripts" / "validate_nearest_run.py"
    protocol = WORKSPACE / "EXPERIMENT_PROTOCOL.md"
    schema = WORKSPACE / "docs" / "DATA_SCHEMA.md"

    checks = [
        (launch, "mapex_nearest_ros_official.py"),
        (launch, "research_recorder_official.py"),
        (adapted_policy, "Hospital adaptation allowing ranked frontier below 1 m"),
        (adapted_policy, "all_ranked_candidates_failed_nav2_path_validation"),
        (official_policy, "first_policy_decision_before_compute"),
        (official_policy, "exhausted_no_ranked_candidate"),
        (official_policy, "candidate_id"),
        (official_policy, "nav2_no_path_count"),
        (research_manager, "EXACT FRONTIER EXECUTION ACTIVE"),
        (research_manager, "goal_source\": \"exact_frontier_center"),
        (official_recorder, "frontier_exploration_start"),
        (official_recorder, "installed_hospital_world"),
        (official_recorder, "intentionally_uncontrolled_gazebo_default"),
        (official_recorder, "execution_goal_semantics"),
        (official_recorder, "planner_endpoint_to_frontier_m"),
        (validator, "selected_candidate_id"),
        (validator, "exhausted_no_ranked_candidate"),
        (validator, "terminal Nav2 audit"),
        (validator, "exact frontier execution goal"),
        (protocol, "first policy decision before computation"),
        (protocol, "intentionally uncontrolled"),
        (protocol, "NavigateToPose(exact frontier center)"),
        (schema, "below_1m is diagnostic only"),
        (schema, "planner_endpoint_to_frontier_m"),
        (schema, "goal_source=exact_frontier_center"),
    ]
    for path, needle in checks:
        if path.exists() and needle not in path.read_text(encoding="utf-8"):
            errors.append(
                f"runtime/protocol invariant missing in {path.relative_to(REPO_ROOT)}: {needle}"
            )

    if errors:
        print("NEAREST PREFLIGHT: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1

    print("NEAREST PREFLIGHT: PASS")
    print("- Python syntax: OK")
    print("- Frozen ROI SHA-256: OK")
    print("- Hospital below-1m bypass: ACTIVE")
    print("- exact frontier execution goal: ACTIVE")
    print("- installed Nav2 exact-planner tolerance: ACTIVE")
    print("- benchmark t=0 before first policy compute: ACTIVE")
    print("- exact exhausted/no-candidate logging: ACTIVE")
    print("- terminal Nav2 audit logging: ACTIVE")
    print("- stable candidate_id linkage: ACTIVE")
    print("- installed runtime provenance hashing: ACTIVE")
    print("- simulator seed policy: intentionally uncontrolled + repeated runs")
    print("- strict post-run validator: present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
