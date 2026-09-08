#!/usr/bin/env python3
"""Run canonical Nearest-Frontier recorder with launch/oldmap_toolbox.launch.py.

This wrapper does not change the Nearest-Frontier policy. It only registers the
Old-Map-First V2 SLAM runtime provenance profile before delegating to nf_run.py.
"""

import sys

from nf_run import RUNTIME_PROFILES, main


RUNTIME_PROFILES["oldmap_toolbox"] = {
    "environment": "new_room",
    "id": "new_room_oldmap_toolbox_v2",
    "note": (
        "New Room benchmark launched with launch/oldmap_toolbox.launch.py using "
        "scan_buffer_size=30; old>new weighted initial and near-chain local scan "
        "matching (min confidence=0.25, decay=70), first scan retained as trusted "
        "history, 0.5 m historical keyframes within 3.0 m (max 40); first-pose "
        "hard anchor; local Ceres edge gap<=5 with 5x->1x decay 70; Karto loop "
        "matching and loop edges remain unweighted/1x; release disabled."
    ),
    "launch_relative": "launch/oldmap_toolbox.launch.py",
    "slam_relative": None,
    "extra_hashes": {
        "oldmap_run_wrapper": "scripts/nf_oldmap_toolbox_run.py",
        "oldmap_mapper": "../slam_toolbox/include/slam_toolbox/oldmap_mapper.hpp",
        "slam_mapper_glue": "../slam_toolbox/src/slam_mapper.cpp",
        "adaptive_ceres_solver": "../slam_toolbox/solvers/ceres_solver.cpp",
    },
}


def _ensure_profile_arg() -> None:
    has_runtime_profile = any(
        arg == "--runtime-profile" or arg.startswith("--runtime-profile=")
        for arg in sys.argv[1:]
    )
    if not has_runtime_profile:
        sys.argv.extend(["--runtime-profile", "oldmap_toolbox"])


if __name__ == "__main__":
    _ensure_profile_arg()
    main()
