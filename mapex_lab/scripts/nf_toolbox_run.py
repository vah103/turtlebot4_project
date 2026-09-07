#!/usr/bin/env python3
"""Run canonical nf_basic.py recorder with the tuned SLAM Toolbox profile.

This wrapper does not change the Nearest-Frontier policy. It only registers the
runtime provenance profile for launch/toolbox.launch.py before delegating to
nf_run.py. Toolbox is injected as the default runtime profile when the caller
does not provide --runtime-profile explicitly.
"""

import sys

from nf_run import RUNTIME_PROFILES, main


RUNTIME_PROFILES["toolbox"] = {
    "environment": "new_room",
    "id": "new_room_toolbox_tuned_loop",
    "note": (
        "New Room benchmark launched with launch/toolbox.launch.py using "
        "upstream SLAM Toolbox online_async plus the tuned loop-closure and "
        "scan-cadence overrides embedded in that launch file."
    ),
    "launch_relative": "launch/toolbox.launch.py",
    "slam_relative": None,
    "extra_hashes": {
        "toolbox_run_wrapper": "scripts/nf_toolbox_run.py",
    },
}


def _ensure_toolbox_profile_arg() -> None:
    has_runtime_profile = any(
        arg == "--runtime-profile" or arg.startswith("--runtime-profile=")
        for arg in sys.argv[1:]
    )
    if not has_runtime_profile:
        sys.argv.extend(["--runtime-profile", "toolbox"])


if __name__ == "__main__":
    _ensure_toolbox_profile_arg()
    main()
