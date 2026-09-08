#!/usr/bin/env python3
"""Run canonical Nearest-Frontier recorder with launch/oldmap_toolbox.launch.py.

This wrapper does not change the Nearest-Frontier policy. It only registers the
old-map-first temporal pose-graph runtime provenance profile before delegating
to nf_run.py.
"""

import sys

from nf_run import RUNTIME_PROFILES, main


RUNTIME_PROFILES["oldmap_toolbox"] = {
    "environment": "new_room",
    "id": "new_room_oldmap_toolbox_v1",
    "note": (
        "New Room benchmark launched with launch/oldmap_toolbox.launch.py using "
        "scan_buffer_size=30, first-pose hard anchoring, local edge gap<=5, "
        "temporal weight 5x->1x with decay 70 nodes, and release disabled."
    ),
    "launch_relative": "launch/oldmap_toolbox.launch.py",
    "slam_relative": None,
    "extra_hashes": {
        "oldmap_run_wrapper": "scripts/nf_oldmap_toolbox_run.py",
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
