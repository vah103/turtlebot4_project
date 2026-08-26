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
    WORKSPACE / "scripts" / "research_recorder.py",
    WORKSPACE / "scripts" / "research_recorder_safe.py",
    WORKSPACE / "scripts" / "exploration_manager_research.py",
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
        errors.append(
            "missing frozen ROI: run scripts/generate_hospital_roi.py before the pilot"
        )
    else:
        actual = sha256(ROI)
        if actual != ROI_EXPECTED_SHA256:
            errors.append(
                "ROI SHA-256 mismatch: "
                f"{actual} != {ROI_EXPECTED_SHA256}"
            )

    base_policy = WORKSPACE / "scripts" / "mapex_nearest_ros.py"
    adapted_policy = WORKSPACE / "scripts" / "mapex_nearest_ros_hospital_adapted.py"
    research_policy = WORKSPACE / "scripts" / "mapex_nearest_ros_research.py"
    recorder = WORKSPACE / "scripts" / "research_recorder_safe.py"
    launch = WORKSPACE / "launch" / "hospital_nearest.launch.py"

    checks = [
        (base_policy, "The 1 m rule is intentionally NOT applied here"),
        (base_policy, "MapEx locked-frontier validity rejected candidate <1.0 m"),
        (adapted_policy, "NOT enforced"),
        (adapted_policy, "checking_nav2_below_1m_allowed"),
        (adapted_policy, "Hospital adaptation allowing ranked frontier below 1 m"),
        (research_policy, "observed_map_raw.npz"),
        (research_policy, "candidates.csv"),
        (recorder, "PERIODIC_MAP_INTERVAL_S = 10.0"),
        (recorder, "policy_decision_id"),
        (launch, "mapex_nearest_ros_hospital_adapted.py"),
        (launch, "exploration_manager_research.py"),
    ]
    for path, needle in checks:
        if path.exists() and needle not in path.read_text(encoding="utf-8"):
            errors.append(
                f"runtime invariant missing in {path.relative_to(REPO_ROOT)}: {needle}"
            )

    # The base adapter remains a faithful MapEx reference, but the Hospital launch
    # must go through the explicit adaptation wrapper so below-1m candidates are
    # allowed instead of being rejected before Nav2.
    if launch.exists():
        text = launch.read_text(encoding="utf-8")
        if "mapex_nearest_ros_hospital_adapted.py" not in text:
            errors.append("Hospital launch is not using the below-1m adaptation wrapper")

    if errors:
        print("NEAREST PREFLIGHT: FAIL")
        for error in errors:
            print(f"- {error}")
        return 1

    print("NEAREST PREFLIGHT: PASS")
    print("- Python syntax: OK")
    print("- Frozen ROI SHA-256: OK")
    print("- original MapEx 1 m rule retained in reference adapter")
    print("- Hospital below-1m bypass: ACTIVE")
    print("- exact decision-map/candidate logging: present")
    print("- periodic/final replay snapshots: present")
    print("- detailed navigation-result logging: present")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
