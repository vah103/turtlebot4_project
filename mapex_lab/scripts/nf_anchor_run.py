#!/usr/bin/env python3
"""Run canonical Nearest-Frontier with the temporal-anchor SLAM Toolbox profile.

This wrapper does not change the Nearest-Frontier exploration policy. It only
registers the runtime provenance for launch/toolbox_anchor.launch.py and defaults
--runtime-profile to toolbox_anchor so recorded metadata matches the actual SLAM
variant used for the run.
"""

import sys

from nf_run import RUNTIME_PROFILES, main


RUNTIME_PROFILES["toolbox_anchor"] = {
    "environment": "new_room",
    "id": "new_room_toolbox_temporal_anchor",
    "note": (
        "New Room benchmark launched with launch/toolbox_anchor.launch.py. "
        "It keeps the conservative Toolbox scan/loop settings and enables the "
        "experimental Ceres temporal-anchor weighting: early local constraints "
        "receive higher information weight that decays toward 1.0; long-gap "
        "loop-closure edges are not strengthened."
    ),
    "launch_relative": "launch/toolbox_anchor.launch.py",
    "slam_relative": None,
    "extra_hashes": {
        "temporal_anchor_patch": "src/slam/temporal_anchor_ceres.patch",
        "anchor_run_wrapper": "scripts/nf_anchor_run.py",
    },
}


def _ensure_anchor_profile_arg() -> None:
    has_runtime_profile = any(
        arg == "--runtime-profile" or arg.startswith("--runtime-profile=")
        for arg in sys.argv[1:]
    )
    if not has_runtime_profile:
        sys.argv.extend(["--runtime-profile", "toolbox_anchor"])


if __name__ == "__main__":
    _ensure_anchor_profile_arg()
    main()
