#!/usr/bin/env python3
"""Generate Hospital structural GT and connected-free ROI from the active 1.0x world.

The heavy lifting lives in ``hospital_ground_truth_core.py`` and intentionally
reuses the same trimesh/OpenCV/scipy rasterization semantics that originally
froze Hospital ROI v1.  The script does not silently rewrite the frozen spec:
it reports whether the active world still reproduces the v1 cell count/SHA.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import hospital_ground_truth_core as core  # noqa: E402
import hospital_scale  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate Hospital 1.0x structural GT + connected-free ROI"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=core.OUTPUT_DIR,
    )
    args = parser.parse_args()

    hospital = hospital_scale.prepare_scaled_hospital()
    if not math.isclose(hospital.scale, 1.0, abs_tol=1e-12):
        print(
            json.dumps(
                {
                    "status": "error",
                    "error": f"Hospital GT is canonical 1.0x only; HOSPITAL_SCALE={hospital.scale}",
                },
                indent=2,
            )
        )
        raise SystemExit(2)

    try:
        summary = core.generate(args.output_dir.expanduser().resolve())
    except Exception as exc:
        print(json.dumps({"status": "error", "error": str(exc)}, indent=2))
        raise SystemExit(2) from exc

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
