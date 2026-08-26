#!/usr/bin/env python3
"""Validated entry point for the Hospital MapEx benchmark.

The full benchmark implementation lives in ``hospital_mapex_benchmark_legacy.py``.
This wrapper adds Hospital-specific safety checks without changing the upstream
MapEx exploration/metric pipeline:

1. normalize the AWS Hospital COLLADA wall mesh into Gazebo coordinates;
2. robustly discover the flat-world elevator blockers; and
3. validate structural GT against the committed final Hospital SLAM snapshot.

Important: ``occ_map.npy`` intentionally stores every cell outside valid-space as
occupied, as expected by the MapEx KTH loader. That representation must NOT be
used directly for wall validation. Validation below rebuilds an explicit wall
raster from the collision mesh and blocker boxes.
"""

from __future__ import annotations

import importlib.util
import inspect
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

HERE = Path(__file__).resolve().parent
LEGACY_PATH = HERE / "hospital_mapex_benchmark_legacy.py"

_spec = importlib.util.spec_from_file_location(
    "hospital_mapex_benchmark_legacy", LEGACY_PATH
)
if _spec is None or _spec.loader is None:
    raise RuntimeError(f"Could not load legacy benchmark: {LEGACY_PATH}")
legacy = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(legacy)


# ---------------------------------------------------------------------------
# AWS Hospital COLLADA normalization
# ---------------------------------------------------------------------------
def _local_xml_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _collada_asset_metadata(mesh_path: Path) -> Tuple[float, str]:
    """Return COLLADA unit scale (metres/unit) and declared up axis."""
    root = ET.parse(mesh_path).getroot()
    unit_scale = 1.0
    up_axis = "Z_UP"

    asset = next(
        (node for node in root if _local_xml_name(node.tag) == "asset"),
        None,
    )
    if asset is not None:
        for child in asset:
            name = _local_xml_name(child.tag)
            if name == "unit":
                unit_scale = float(child.attrib.get("meter", "1.0"))
            elif name == "up_axis" and child.text:
                up_axis = child.text.strip().upper()

    if unit_scale <= 0.0:
        raise RuntimeError(
            f"Invalid COLLADA unit scale {unit_scale}: {mesh_path}"
        )
    return unit_scale, up_axis


def load_wall_section_polylines(mesh_path: Path, z_slice: float):
    """Load the AWS wall mesh in Gazebo metres and take a horizontal section.

    The source Hospital DAE declares ``unit meter=0.01`` and ``Y_UP``. Depending
    on the trimesh/pycollada versions, those asset transforms may or may not be
    applied automatically. We therefore inspect the loaded extents and only
    apply the missing transforms:

      COLLADA (X,Y,Z) -> Gazebo (X,-Z,Y)

    This is the same coordinate convention already used by hospital_canvas.py.
    """
    np = legacy.np
    trimesh = legacy.require_import(
        "trimesh", "python3 -m pip install trimesh pycollada"
    )

    loaded = trimesh.load(str(mesh_path), force="scene")
    if isinstance(loaded, trimesh.Scene):
        meshes = []
        for node_name in loaded.graph.nodes_geometry:
            transform, geom_name = loaded.graph[node_name]
            geom = loaded.geometry[geom_name].copy()
            geom.apply_transform(transform)
            meshes.append(geom)
        if not meshes:
            raise RuntimeError("Hospital wall DAE contains no geometry.")
        mesh = trimesh.util.concatenate(meshes)
    else:
        mesh = loaded.copy()

    unit_scale, up_axis = _collada_asset_metadata(mesh_path)
    extents = np.asarray(mesh.extents, dtype=np.float64)
    if extents.shape != (3,) or not np.all(np.isfinite(extents)):
        raise RuntimeError(f"Invalid Hospital wall mesh extents: {extents}")

    # Some COLLADA loaders honor <unit meter=...>; some return authoring units.
    # The real Hospital floor is tens of metres wide, not thousands. Apply the
    # declared unit scale only when the loaded geometry is clearly unscaled.
    applied_scale = False
    if unit_scale != 1.0 and float(np.max(extents)) > 200.0:
        mesh.apply_scale(unit_scale)
        extents = np.asarray(mesh.extents, dtype=np.float64)
        applied_scale = True

    # For this AWS asset, after metric scaling the vertical span is ~3 m and the
    # second planar span is ~25 m. If Y is still the short/vertical dimension,
    # trimesh has preserved COLLADA Y_UP and we must rotate into Gazebo Z_UP.
    applied_axis = False
    if up_axis == "Y_UP" and extents[1] < 0.5 * extents[2]:
        transform = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        mesh.apply_transform(transform)
        extents = np.asarray(mesh.extents, dtype=np.float64)
        applied_axis = True
    elif up_axis not in ("Y_UP", "Z_UP"):
        raise RuntimeError(
            f"Unsupported COLLADA up_axis {up_axis!r}: {mesh_path}"
        )

    # Sanity check after normalization. A bad units/axis interpretation is much
    # better caught here than after an expensive benchmark run.
    if float(np.max(extents[:2])) > 100.0 or float(extents[2]) > 10.0:
        raise RuntimeError(
            "Hospital wall mesh normalization produced implausible extents "
            f"{extents.tolist()} m (unit_scale={unit_scale}, up_axis={up_axis})."
        )

    if applied_scale or applied_axis:
        legacy.log(
            "Normalized Hospital DAE: "
            f"unit_scale={'applied' if applied_scale else 'already metric'}, "
            f"Y_UP->Z_UP={'applied' if applied_axis else 'already applied'}, "
            f"extents_m={[round(float(v), 3) for v in extents]}"
        )

    section = mesh.section(
        plane_origin=np.array([0.0, 0.0, z_slice]),
        plane_normal=np.array([0.0, 0.0, 1.0]),
    )
    if section is None:
        raise RuntimeError(f"No wall-mesh section found at z={z_slice:.3f} m.")

    polylines = [
        np.asarray(p, dtype=np.float64)[:, :2]
        for p in section.discrete
        if len(p) >= 2
    ]
    segment_count = int(sum(max(0, len(p) - 1) for p in polylines))
    entity_count = len(section.entities)
    return polylines, entity_count, segment_count


# Replace the legacy loader before either GT building or validation runs.
legacy.load_wall_section_polylines = load_wall_section_polylines


# ---------------------------------------------------------------------------
# Robust world parsing
# ---------------------------------------------------------------------------
def find_world_model_geometry(
    sdf_path: Path,
) -> Tuple[
    Tuple[float, float, float],
    List[Tuple[float, float, float, float, float]],
]:
    """Return wall pose and every elevator blocker box from the flat world."""
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
# Boundary-touch is diagnostic only
# ---------------------------------------------------------------------------
_original_log = legacy.log


def log(msg: str) -> None:
    if msg.startswith(
        "WARNING: connected free space touches the configured Hospital bounding box"
    ):
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

# Guard rails. The Hospital analysis observed substantially better agreement.
MIN_WALL_MATCH = 0.70
MAX_FREE_CONFLICT = 0.05
WALL_MATCH_TOLERANCE_PIXELS = 2  # 0.20 m at the 0.10 m validation scale.


def _build_explicit_wall_mask(
    aws_root: Path,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
    z_slice: float,
    wall_thickness_cells: int,
    include_flat_blockers: bool,
):
    """Rasterize only physical Hospital walls/blockers on the 0.05 m canvas."""
    np = legacy.np
    cv2 = legacy.require_import("cv2", "python3 -m pip install opencv-python")

    mesh_path = (
        aws_root
        / "models"
        / "aws_robomaker_hospital_floor_01_walls"
        / "meshes"
        / "aws_robomaker_hospital_floor_01_walls_collision.dae"
    )
    if not mesh_path.exists():
        raise RuntimeError(f"Hospital collision mesh missing: {mesh_path}")

    (wall_x, wall_y, wall_yaw), blockers = find_world_model_geometry(
        legacy.HOSPITAL_WORLD_SDF
    )
    polylines, _entity_count, _segment_count = load_wall_section_polylines(
        mesh_path, z_slice
    )

    obstacle = np.zeros(
        (legacy.CANVAS_HEIGHT, legacy.CANVAS_WIDTH), dtype=np.uint8
    )
    for poly in polylines:
        poly_world = legacy.transform_xy(poly, wall_x, wall_y, wall_yaw)
        poly_slam = legacy.world_to_slam(
            poly_world, spawn_x, spawn_y, spawn_yaw
        )
        rc = legacy.metric_xy_to_raw_rc(poly_slam)
        legacy.rasterize_polyline(obstacle, rc, wall_thickness_cells, cv2)

    if include_flat_blockers:
        for x, y, yaw, sx, sy in blockers:
            corners_world = legacy.rectangle_corners(x, y, yaw, sx, sy)
            corners_slam = legacy.world_to_slam(
                corners_world, spawn_x, spawn_y, spawn_yaw
            )
            rc = legacy.metric_xy_to_raw_rc(corners_slam)
            polygon = np.stack([rc[:, 1], rc[:, 0]], axis=1).astype(np.int32)
            cv2.fillPoly(obstacle, [polygon], 255)

    # Same crack-closing dilation used by the GT builder.
    kernel = np.ones((3, 3), dtype=np.uint8)
    obstacle_closed = cv2.dilate(obstacle, kernel, iterations=1)
    return obstacle_closed > 0


def _wall_mask_as_lama_image(wall_raw, target_shape):
    """Convert 0.05 m wall mask to the final LaMa top-down image geometry."""
    np = legacy.np
    source_h, source_w = wall_raw.shape
    block = int(round(0.10 / legacy.CANVAS_RESOLUTION))
    if block != 2:
        raise RuntimeError(f"Unexpected Hospital downsample factor: {block}")

    # The dataset preprocessor uses ceil-sized 0.10 m cells. For a binary
    # obstacle mask, any occupied source cell makes the target cell occupied.
    pad_h = (-source_h) % block
    pad_w = (-source_w) % block
    padded = np.pad(
        wall_raw.astype(np.uint8),
        ((0, pad_h), (0, pad_w)),
        mode="constant",
        constant_values=0,
    )
    reduced = padded.reshape(
        padded.shape[0] // block,
        block,
        padded.shape[1] // block,
        block,
    ).max(axis=(1, 3)).astype(bool)

    top_down = reduced[::-1, :]
    target_h, target_w = target_shape
    if top_down.shape[1] > target_w or top_down.shape[0] > target_h:
        raise RuntimeError(
            f"Structural wall map {top_down.shape} is larger than "
            f"reference {target_shape}."
        )

    output = np.zeros((target_h, target_w), dtype=bool)
    pad_top = target_h - top_down.shape[0]
    output[
        pad_top : pad_top + top_down.shape[0],
        : top_down.shape[1],
    ] = top_down
    return output


def validate_structural_gt(
    map_dir: Path,
    metadata: dict,
    *,
    aws_root: Path,
    spawn_x: float,
    spawn_y: float,
    spawn_yaw: float,
    z_slice: float,
    wall_thickness_cells: int,
    include_flat_blockers: bool,
) -> dict:
    np = legacy.np
    cv2 = legacy.require_import("cv2", "python3 -m pip install opencv-python")

    if not FINAL_REFERENCE.exists():
        raise RuntimeError(
            "Committed Hospital reference snapshot is missing: "
            f"{FINAL_REFERENCE}. Run from a complete turtlebot4_project checkout."
        )

    reference = cv2.imread(str(FINAL_REFERENCE), cv2.IMREAD_GRAYSCALE)
    if reference is None:
        raise RuntimeError(
            f"Could not read GT validation reference: {FINAL_REFERENCE}"
        )

    wall_raw = _build_explicit_wall_mask(
        aws_root=aws_root,
        spawn_x=spawn_x,
        spawn_y=spawn_y,
        spawn_yaw=spawn_yaw,
        z_slice=z_slice,
        wall_thickness_cells=wall_thickness_cells,
        include_flat_blockers=include_flat_blockers,
    )
    structural_occ = _wall_mask_as_lama_image(wall_raw, reference.shape)

    reference_occ = reference <= 64
    reference_free = reference >= 192

    occ_count = int(reference_occ.sum())
    free_count = int(reference_free.sum())
    structural_count = int(structural_occ.sum())
    if occ_count == 0 or free_count == 0 or structural_count == 0:
        raise RuntimeError(
            "Validation reference/structural wall map contains no usable cells."
        )

    # SLAM and mesh rasterization need a small spatial tolerance. The metric
    # remains reference-wall recall: an observed occupied cell is matched when
    # a structural wall lies within 0.20 m.
    kernel = np.ones((3, 3), dtype=np.uint8)
    structural_near = cv2.dilate(
        structural_occ.astype(np.uint8),
        kernel,
        iterations=WALL_MATCH_TOLERANCE_PIXELS,
    ).astype(bool)
    reference_near = cv2.dilate(
        reference_occ.astype(np.uint8),
        kernel,
        iterations=WALL_MATCH_TOLERANCE_PIXELS,
    ).astype(bool)

    wall_match = float((structural_near & reference_occ).sum() / occ_count)
    wall_precision = float(
        (structural_occ & reference_near).sum() / structural_count
    )
    # Only explicit physical wall cells are tested against observed free space.
    # Do NOT use MapEx occ_map.npy here: outside-valid is intentionally 0 there.
    free_conflict = float(
        (structural_occ & reference_free).sum() / structural_count
    )

    occ_raw = np.load(map_dir / "occ_map.npy")
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
        "reference": str(
            FINAL_REFERENCE.relative_to(legacy.PROJECT_ROOT)
        ),
        "wall_match": wall_match,
        "wall_precision": wall_precision,
        "free_space_conflict": free_conflict,
        "start_is_free": start_is_free,
        "explicit_wall_cells_0p10m": structural_count,
        "thresholds": {
            "min_wall_match": MIN_WALL_MATCH,
            "max_free_space_conflict": MAX_FREE_CONFLICT,
            "wall_match_tolerance_pixels": WALL_MATCH_TOLERANCE_PIXELS,
        },
        "note": (
            "Validation uses explicit collision-mesh walls plus flat-world "
            "elevator blockers. Outside-valid cells in MapEx occ_map.npy are "
            "not treated as physical walls."
        ),
    }
    metadata["validation"] = validation

    metadata_path = map_dir / "hospital_map_metadata.json"
    metadata_path.write_text(
        json.dumps(metadata, indent=2), encoding="utf-8"
    )

    status = "PASS" if passed else "FAIL"
    legacy.log(
        f"GT validation: {status} | wall_match={wall_match * 100:.1f}% | "
        f"wall_precision={wall_precision * 100:.1f}% | "
        f"free_conflict={free_conflict * 100:.2f}% | "
        f"start_free={start_is_free}"
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
_build_signature = inspect.signature(_original_build_hospital_map)


def build_hospital_map(*args, **kwargs):
    bound = _build_signature.bind(*args, **kwargs)
    params = bound.arguments

    map_dir, start_pose, metadata = _original_build_hospital_map(
        *args, **kwargs
    )
    metadata = validate_structural_gt(
        map_dir,
        metadata,
        aws_root=Path(params["aws_root"]),
        spawn_x=float(params["spawn_x"]),
        spawn_y=float(params["spawn_y"]),
        spawn_yaw=float(params["spawn_yaw"]),
        z_slice=float(params["z_slice"]),
        wall_thickness_cells=int(params["wall_thickness_cells"]),
        include_flat_blockers=bool(params["include_flat_blockers"]),
    )
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
