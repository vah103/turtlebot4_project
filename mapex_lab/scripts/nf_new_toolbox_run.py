#!/usr/bin/env python3
"""Run canonical Nearest-Frontier recorder with launch/new_toolbox.launch.py.

This wrapper does not change the Nearest-Frontier policy. It only registers the
Adaptive Temporal Anchor V1 runtime provenance profile used by
launch/new_toolbox.launch.py before delegating to nf_run.py.
"""

import sys

from nf_run import RUNTIME_PROFILES, main


RUNTIME_PROFILES["new_toolbox"] = {
    "environment": "new_room",
    "id": "new_room_new_toolbox_adaptive_v1",
    "note": (
        "New Room benchmark launched with launch/new_toolbox.launch.py using "
        "vendored SLAM Toolbox + Adaptive Temporal Anchor V1 and the Nav2 "
        "settings embedded/merged by that launch file."
    ),
    "launch_relative": "launch/new_toolbox.launch.py",
    "slam_relative": None,
    "extra_hashes": {
        "new_toolbox_run_wrapper": "scripts/nf_new_toolbox_run.py",
        "adaptive_ceres_solver": "../slam_toolbox/solvers/ceres_solver.cpp",
    },
}


def _ensure_profile_arg() -> None:
    has_runtime_profile = any(
        arg == "--runtime-profile" or arg.startswith("--runtime-profile=")
        for arg in sys.argv[1:]
    )
    if not has_runtime_profile:
        sys.argv.extend(["--runtime-profile", "new_toolbox"])


if __name__ == "__main__":
    _ensure_profile_arg()
    main()
