#!/usr/bin/env python3
"""Validated entry point for the Hospital MapEx benchmark.

The original benchmark implementation is kept in
``hospital_mapex_benchmark_legacy.py``.  This wrapper adds two things that are
important for the Hospital experiment:

1. robust discovery of the flat-world elevator blocker boxes; and
2. a structural-GT sanity check against the committed final Hospital SLAM
   snapshot (001415) before an expensive benchmark is allowed to continue.

The old "valid space touches bounding box" message is therefore treated as a
clipping note, not as a pass/fail criterion by itself.  The actual gate uses
wall agreement and free-space conflict, matching the diagnostics used in the
Hospital analysis report.
"""

from __future__ import annotations

import importlib.util
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

HERE = Path(__file__).resolve().parent
LEGACY_PATH = HERE / "hospital_mapex_benchmark_legacy.py"

_spec = importlib.util.spec_from_file_location("hospital_mapex_benchmark_legacy", LEGACY_PATH)
if _spec is None or _spec.loader is None:
    raise RuntimeError(f"Could not load legacy benchmark: {LEGACY_PATH}")
legacy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(legacy)


# ---------------------------------------------------------------------------
# Robust world parsing
# ---------------------------------------------------------------------------
def find_world_model_geometry(
    sdf_path: Path,
) -> Tuple[Tuple[float, float, float], List[Tuple[float, float, float, float, float]]]:
    """Return wall pose and every elevator blocker box from the flat world.

    Older revisions used names such as ``elevator_opening_blocker_left`` while
    a later revision used ``elevator_blocker_left``.  Geometry, not an exact
    naming convention, should decide whether the boxes enter structural GT.
    """
    root = ET.parse(sdf_path).getroot()
    wall_pose: Optional[Tuple[float, float, float]] = None

    for include in root.findall(".//include"):
        uri = (include.findtext("uri") or "").strip()
        if uri.endswith("aws_robomaker_hospital_floor_01_walls"):
            x, y, _z, _r, _p, yaw = legacy.parse_pose(include.findtext("pose"))
            wall_pose = (x, y, yaw)
            break

    if wall_pose is None:
        raise RuntimeError(f"Could not find Hospital wall include in {sdf_path}")

    blockers: List[Tuple[float, float, float, float, float]] = []
    for model in root.findall(".//model"):
        name = model.attrib.get("name", "").lower()
        if not ("elevator" in name and "blocker" in name):
            continue
        x, y, _z, _r, _p, yaw = legacy.parse_pose(model.findtext("pose"))
        size_text = model.findtext(".//collision/geometry/box/size")
        if not size_text:
            continue
        size = [float(v) for v in size_text.split()]
        if len(size) < 2:
            continue
        blockers.append((x, y, yaw, size[0], size[1]))

    return wall_pose, blockers


legacy.find_world_model_geometry = find_world_model_geometry


# ---------------------------------------------------------------------------
# Better interpretation of the bounding-box diagnostic
# ---------------------------------------------------------------------------
_original_log = legacy.log


def log(msg: str) -> None:
    if msg.startswith("WARNING: connected free space touches the configured Hospital bounding box"):
        _original_log(
            "NOTE: valid space touches the configured clipping bounds; "
            "this is not a failure by itself. Running structural-GT validation next."
        )
    else:
        _original_log(msg)


legacy.log = log


FINAL_REFERENCE = (
    legacy.PROJECT_ROOT
    / "data"
    / "lama_runs"
    / "hospital_flat_lama_01_001"
    / "lama_eval_20"
    / "model_input"
    / "001415.png"
)

# Deliberately looser than the values observed in the Hospital report
# (wall match 78.7--94.4%, free-space conflict <2%).  These are guard rails,
# not numbers to optimize against.
MIN_WALL_MATCH = 0.70
MAX_FREE_CONFLICT = 0.05


def _structural_map_as_lama_image(occ_raw, target_shape):
    """Convert raw 0.05 m structural map to the exact LaMa image geometry.

    Raw structural convention: 0=occupied/invalid, 254=free.
    LaMa convention: 0=occupied, 255=free, 127=unknown.

    Downsampling preserves obstacles with a 2x2 minimum.  The image is then
    flipped to top-down PNG order and padded on +Y (the top of the PNG), exactly
    like ``lama_dataset_preprocessor.py``.
    """
    np = legacy.np
    source_h, source_w = occ_raw.shape
    block = int(round(0.10 / legacy.CANVAS_RESOLUTION))
    if block != 2:
        raise RuntimeError(f"Unexpected Hospital downsample factor: {block}")

    pad_h = (-source_h) % block
    pad_w = (-source_w) % block
    padded = np.pad(
        occ_raw,
        ((0, pad_h), (0, pad_w)),
        mode="constant",
        constant_values=254,
    )
    reduced = padded.reshape(
        padded.shape[0] // block,
        block,
        padded.shape[1] // block,
        block,
    ).min(axis=(1, 3))

    top_down = reduced[::-1, :]
    target_h, target_w = target_shape
    if top_down.shape[1] > target_w or top_down.shape[0] > target_h:
        raise RuntimeError(
            f"Structural map {top_down.shape} is larger than reference {target_shape}."
        )

    output = np.full((target_h, target_w), 127, dtype=np.uint8)
    pad_top = target_h - top_down.shape[0]
    output[pad_top : pad_top + top_down.shape[0], : top_down.shape[1]] = np.where(
        top_down > 127, 255, 0
    ).astype(np.uint8)
    return output


def validate_structural_gt(map_dir: Path, metadata: dict) -> dict:
    np = legacy.np
    cv2 = legacy.require_import("cv2", "python3 -m pip install opencv-python")

    if not FINAL_REFERENCE.exists():
        raise RuntimeError(
            "Committed Hospital reference snapshot is missing: "
            f"{FINAL_REFERENCE}. Run from a complete turtlebot4_project checkout."
        )

    reference = cv2.imread(str(FINAL_REFERENCE), cv2.IMREAD_GRAYSCALE)
    if reference is None:
        raise RuntimeError(f"Could not read GT validation reference: {FINAL_REFERENCE}")

    occ_raw = np.load(map_dir / "occ_map.npy")
    structural = _structural_map_as_lama_image(occ_raw, reference.shape)

    reference_occ = reference <= 64
    reference_free = reference >= 192
    structural_occ = structural <= 64

    occ_count = int(reference_occ.sum())
    free_count = int(reference_free.sum())
    if occ_count == 0 or free_count == 0:
        raise RuntimeError("Reference snapshot contains no usable occupied/free cells.")

    wall_match = float((structural_occ & reference_occ).sum() / occ_count)
    free_conflict = float((structural_occ & reference_free).sum() / free_count)

    start_rc = legacy.metric_xy_to_raw_rc(
        np.array([[0.0, 0.0]], dtype=np.float64)
    )[0]
    sr, sc = int(start_rc[0]), int(start_rc[1])
    start_is_free = bool(
        0 <= sr < occ_raw.shape[0]
        and 0 <= sc < occ_raw.shape[1]
        and occ_raw[sr, sc] > 127
    )

    passed = bool(
        wall_match >= MIN_WALL_MATCH
        and free_conflict <= MAX_FREE_CONFLICT
        and start_is_free
    )

    validation = {
        "passed": passed,
        "reference": str(FINAL_REFERENCE.relative_to(legacy.PROJECT_ROOT)),
        "wall_match": wall_match,
        "free_space_conflict": free_conflict,
        "start_is_free": start_is_free,
        "thresholds": {
            "min_wall_match": MIN_WALL_MATCH,
            "max_free_space_conflict": MAX_FREE_CONFLICT,
        },
        "note": (
            "Boundary-touch is recorded separately as a clipping diagnostic; "
            "it is not by itself a GT failure."
        ),
    }
    metadata["validation"] = validation

    metadata_path = map_dir / "hospital_map_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    status = "PASS" if passed else "FAIL"
    legacy.log(
        f"GT validation: {status} | wall_match={wall_match * 100:.1f}% | "
        f"free_conflict={free_conflict * 100:.2f}% | start_free={start_is_free}"
    )

    if not passed:
        raise RuntimeError(
            "Hospital structural GT failed validation. "
            f"Need wall_match >= {MIN_WALL_MATCH * 100:.0f}% and "
            f"free_conflict <= {MAX_FREE_CONFLICT * 100:.0f}%. "
            "Do not run the benchmark until this is fixed."
        )

    return metadata


_original_build_hospital_map = legacy.build_hospital_map


def build_hospital_map(*args, **kwargs):
    map_dir, start_pose, metadata = _original_build_hospital_map(*args, **kwargs)
    metadata = validate_structural_gt(map_dir, metadata)
    return map_dir, start_pose, metadata


legacy.build_hospital_map = build_hospital_map


if __name__ == "__main__":
    try:
        raise SystemExit(legacy.main())
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        raise SystemExit(130)
    except Exception as exc:
        print(f"\nERROR: {exc}", file=sys.stderr)
        raise
