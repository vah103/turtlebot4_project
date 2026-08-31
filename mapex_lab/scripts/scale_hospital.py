#!/usr/bin/env python3
"""Preview/pre-generate the currently configured Hospital scale.

Normal Hospital launch files do this automatically. To change the long-term
scale, edit only ``HOSPITAL_SCALE`` in ``mapex_lab/scripts/hospital_scale.py``.
All world/model/generated paths remain under ``mapex_lab/map``.
"""

from __future__ import annotations

import sys

from hospital_scale import HOSPITAL_SCALE, prepare_scaled_hospital


def main() -> int:
    try:
        hospital = prepare_scaled_hospital()
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    print(f"Configured Hospital scale: x{HOSPITAL_SCALE:g}")
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
