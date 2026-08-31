#!/usr/bin/env python3
"""Prepare the currently configured scaled Hospital world.

Normally this script does not need to be run manually: the Hospital simulation
launch prepares the scaled world automatically. It is kept as a quick preview /
pre-generation command.

To change the long-term scale, edit only:
    ros2_ws/src/frontier_exploration/frontier_exploration/hospital_scale.py
    HOSPITAL_SCALE = ...
"""

from __future__ import annotations

import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_SOURCE = REPO_ROOT / "ros2_ws/src/frontier_exploration"
sys.path.insert(0, str(PACKAGE_SOURCE))

from frontier_exploration.hospital_scale import (  # noqa: E402
    HOSPITAL_SCALE,
    prepare_scaled_hospital,
)


def main() -> int:
    source_world = (
        REPO_ROOT
        / "ros2_ws/src/frontier_exploration/worlds/hospital_aws_flat.sdf"
    )
    source_models = (
        Path.home()
        / ".cache/turtlebot4_project/hospital_world/models"
    )

    try:
        hospital = prepare_scaled_hospital(
            source_world=source_world,
            source_models_dir=source_models,
            scale=HOSPITAL_SCALE,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"Configured Hospital scale: x{hospital.scale:g}")
    print(f"World: {hospital.world}")
    print(f"Models: {hospital.models_dir}")
    print(
        "Default spawn: "
        f"x={hospital.spawn_x:g}, y={hospital.spawn_y:g}, yaw={hospital.spawn_yaw:g}"
    )
    print("Normal Hospital launch files use this scale automatically.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
