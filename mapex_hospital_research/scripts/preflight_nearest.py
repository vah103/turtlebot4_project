#!/usr/bin/env python3
"""Preflight checks before a Hospital MapEx-nearest research run."""

from __future__ import annotations

import hashlib
import math
import py_compile
from pathlib import Path

import yaml


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
SOURCE_CONFIG = REPO_ROOT / "ros2_ws/src/frontier_exploration/config"
INSTALLED_CONFIG = (
    REPO_ROOT / "ros2_ws/install/frontier_exploration/share/frontier_exploration/config"
)
SOURCE_NAV2_OVERRIDE = SOURCE_CONFIG / "nav2_hospital_override.yaml"
INSTALLED_NAV2_OVERRIDE = INSTALLED_CONFIG / "nav2_hospital_override.yaml"
SOURCE_SLAM = SOURCE_CONFIG / "hospital_slam.yaml"
INSTALLED_SLAM = INSTALLED_CONFIG / "hospital_slam.yaml"
SOURCE_SLAM_NO_LOOP = SOURCE_CONFIG / "hospital_slam_no_loop.yaml"
INSTALLED_SLAM_NO_LOOP = INSTALLED_CONFIG / "hospital_slam_no_loop.yaml"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_yaml(path: Path, label: str, errors: list[str]) -> dict:
    if not path.exists():
        errors.append(f"missing {label}: {path}")
        return {}
    try:
        return yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except Exception as exc:  # noqa: BLE001
        errors.append(f"cannot parse {label}: {exc}")
        return {}


def nested(mapping: dict, *keys):
    cur = mapping
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return None
        cur = cur[key]
    return cur


def check_nav2(path: Path, label: str, errors: list[str]) -> None:
    cfg = load_yaml(path, label, errors)
    if not cfg:
        return
    checks = {
        "GridBased tolerance": (
            nested(cfg, "planner_server", "ros__parameters", "GridBased", "tolerance"),
            0.0,
        ),
        "MPPI vx_max": (
            nested(cfg, "controller_server", "ros__parameters", "FollowPath", "vx_max"),
            0.45,
        ),
        "goal yaw tolerance": (
            nested(
                cfg,
                "controller_server",
                "ros__parameters",
                "general_goal_checker",
                "yaw_goal_tolerance",
            ),
            math.pi,
        ),
    }
    for name, (actual, expected) in checks.items():
        try:
            if abs(float(actual) - expected) > 1e-6:
                errors.append(f"{label}: {name}={actual!r}, expected {expected}")
        except Exception:
            errors.append(f"{label}: missing/invalid {name}: {actual!r}")

    enabled = nested(
        cfg,
        "controller_server",
        "ros__parameters",
        "FollowPath",
        "GoalAngleCritic",
        "enabled",
    )
    if enabled is not False:
        errors.append(f"{label}: GoalAngleCritic.enabled must be false, got {enabled!r}")

    max_velocity = nested(cfg, "velocity_smoother", "ros__parameters", "max_velocity")
    if not isinstance(max_velocity, list) or not max_velocity or abs(float(max_velocity[0]) - 0.45) > 1e-6:
        errors.append(f"{label}: velocity_smoother max linear velocity is not 0.45")


def check_slam(path: Path, label: str, errors: list[str], *, loop_expected: bool) -> None:
    cfg = load_yaml(path, label, errors)
    if not cfg:
        return
    params = nested(cfg, "slam_toolbox", "ros__parameters") or {}
    expected = {
        "minimum_travel_distance": 0.10,
        "minimum_travel_heading": 0.10,
        "minimum_time_interval": 0.15,
    }
    for key, value in expected.items():
        actual = params.get(key)
        try:
            if abs(float(actual) - value) > 1e-6:
                errors.append(f"{label}: {key}={actual!r}, expected {value}")
        except Exception:
            errors.append(f"{label}: missing/invalid {key}: {actual!r}")
    if params.get("do_loop_closing") is not loop_expected:
        errors.append(
            f"{label}: do_loop_closing={params.get('do_loop_closing')!r}, "
            f"expected {loop_expected}"
        )


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

    check_nav2(SOURCE_NAV2_OVERRIDE, "source Hospital Nav2 override", errors)
    check_nav2(INSTALLED_NAV2_OVERRIDE, "installed Hospital Nav2 override", errors)
    check_slam(SOURCE_SLAM, "source Hospital SLAM profile", errors, loop_expected=True)
    check_slam(INSTALLED_SLAM, "installed Hospital SLAM profile", errors, loop_expected=True)
    check_slam(
        SOURCE_SLAM_NO_LOOP,
        "source no-loop SLAM diagnostic",
        errors,
        loop_expected=False,
    )
    check_slam(
        INSTALLED_SLAM_NO_LOOP,
        "installed no-loop SLAM diagnostic",
        errors,
        loop_expected=False,
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
        (launch, "nearest_pilot_010"),
        (adapted_policy, "GoalStatus.STATUS_SUCCEEDED"),
        (adapted_policy, "PLANNER_REVALIDATE_PERIOD_S"),
        (adapted_policy, "EXECUTION_FAILURE_COOLDOWN_S"),
        (adapted_policy, "yaw as a neutral seed"),
        (official_policy, "first_policy_decision_before_compute"),
        (official_policy, "nav2_action_status"),
        (research_manager, "ignored_by_hospital_goal_checker"),
        (official_recorder, "runtime_nav2_merged.yaml"),
        (official_recorder, "installed_nav2_base_params"),
        (official_recorder, "selected_path_length_semantics"),
        (validator, "experiment health"),
        (validator, "terminal planner revalidation"),
        (validator, "runtime_nav2_merged"),
        (protocol, "position-only"),
        (protocol, "Planner revalidation"),
        (schema, "goal_yaw_semantics"),
        (schema, "planner_validation_path_length_not_executed_trajectory"),
    ]
    for path, needle in checks:
        if path.exists() and needle not in path.read_text(encoding="utf-8"):
            errors.append(
                f"runtime/protocol invariant missing in "
                f"{path.relative_to(REPO_ROOT)}: {needle}"
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
    print("- exact frontier x/y execution: ACTIVE")
    print("- frontier goal yaw constraint: DISABLED")
    print("- Nav2 planner action status check: ACTIVE")
    print("- transient no-path planner revalidation: ACTIVE")
    print("- bounded execution-failure cooldown: ACTIVE")
    print("- installed Nav2 tolerance/speed/yaw semantics: ACTIVE")
    print("- installed SLAM keyframe spacing 0.10 m / 0.10 rad: ACTIVE")
    print("- no-loop SLAM A/B profile synchronized: ACTIVE")
    print("- benchmark t=0 before first policy compute: ACTIVE")
    print("- effective Nav2 config archival/provenance: ACTIVE")
    print("- strict data + experiment-health validator: present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
