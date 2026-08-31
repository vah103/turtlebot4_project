"""Automatic scaling for the flat AWS Hospital world.

The Hospital geometry is scaled uniformly in X/Y/Z by ``HOSPITAL_SCALE``.
The TurtleBot4 itself is not scaled. Its spawn position intentionally follows
a different rule: X stays at the original value, while only Y is multiplied by
the Hospital scale.
"""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


MAPEX_LAB_ROOT = Path(__file__).resolve().parents[1]
MAP_ROOT = MAPEX_LAB_ROOT / "map"
SOURCE_WORLD = MAP_ROOT / "hospital_aws_flat.sdf"
SOURCE_MODELS_DIR = MAP_ROOT / "models"
GENERATED_ROOT = MAP_ROOT / "generated"

# Single source of truth for Hospital geometry scale.
# 1.0 = original size; 0.6 = 60% size in X, Y and Z.
HOSPITAL_SCALE = 0.6
SCALE_MODE = "uniform_map_spawn_y_only"

# Reference spawn in the original (1.0x) Hospital.
# Spawn X is NOT scaled; spawn Y follows HOSPITAL_SCALE.
# At 0.6 this gives (0.0, 7.2).
ORIGINAL_SPAWN_X = 0.0
ORIGINAL_SPAWN_Y = 12.0
ORIGINAL_SPAWN_YAW = -1.57

_LOCAL_MODELS = (
    "aws_robomaker_hospital_floor_01_walls",
    "aws_robomaker_hospital_curtain_closed_01",
)


@dataclass(frozen=True)
class ScaledHospital:
    scale: float
    world: Path
    models_dir: Path
    spawn_x: float
    spawn_y: float
    spawn_yaw: float


def _fmt(value: float) -> str:
    if abs(value) < 5e-13:
        value = 0.0
    return f"{value:.12g}"


def _tag(scale: float) -> str:
    return (
        f"{scale:.6f}"
        .rstrip("0")
        .rstrip(".")
        .replace("-", "m")
        .replace(".", "p")
    )


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _scale_numbers(text: str | None, scale: float, count: int) -> str | None:
    if text is None:
        return None
    values = text.split()
    if len(values) != count:
        raise ValueError(f"Expected {count} numbers, got {text!r}")
    return " ".join(_fmt(float(v) * scale) for v in values)


def _scale_pose_uniform(text: str | None, scale: float) -> str | None:
    if text is None:
        return None
    parts = text.split()
    if len(parts) not in (6, 7):
        raise ValueError(f"Unsupported <pose>: {text!r}")
    values = [float(v) for v in parts]
    values[0] *= scale
    values[1] *= scale
    values[2] *= scale
    return " ".join(_fmt(v) for v in values)


def _scale_tree_uniform(root: ET.Element, scale: float) -> None:
    """Scale all Hospital geometry uniformly in X/Y/Z."""
    for pose in root.iter("pose"):
        pose.text = _scale_pose_uniform(pose.text, scale)

    for mesh in root.iter("mesh"):
        scale_el = mesh.find("scale")
        if scale_el is None:
            scale_el = ET.SubElement(mesh, "scale")
            scale_el.text = f"{_fmt(scale)} {_fmt(scale)} {_fmt(scale)}"
        else:
            scale_el.text = _scale_numbers(scale_el.text, scale, 3)

    for box in root.iter("box"):
        size = box.find("size")
        if size is not None:
            size.text = _scale_numbers(size.text, scale, 3)

    for plane in root.iter("plane"):
        size = plane.find("size")
        if size is not None:
            size.text = _scale_numbers(size.text, scale, 2)

    for sphere in root.iter("sphere"):
        radius = sphere.find("radius")
        if radius is not None and radius.text:
            radius.text = _fmt(float(radius.text) * scale)

    for shape in list(root.iter("cylinder")) + list(root.iter("capsule")):
        for name in ("radius", "length"):
            el = shape.find(name)
            if el is not None and el.text:
                el.text = _fmt(float(el.text) * scale)


def _write_tree(tree: ET.ElementTree, path: Path) -> None:
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _rewrite_model_uris(root: ET.Element, old: str, new: str) -> None:
    prefix = f"model://{old}/"
    replacement = f"model://{new}/"
    for uri in root.iter("uri"):
        if uri.text and uri.text.startswith(prefix):
            uri.text = replacement + uri.text[len(prefix):]


def _write_clean_model_config(path: Path, generated_name: str) -> None:
    # Do not parse the upstream curtain model.config: its description contains
    # an unescaped '&', which is invalid XML. A minimal valid config is enough.
    root = ET.Element("model")
    ET.SubElement(root, "name").text = generated_name
    ET.SubElement(root, "version").text = "1.0"
    sdf = ET.SubElement(root, "sdf", {"version": "1.6"})
    sdf.text = "model.sdf"
    _write_tree(ET.ElementTree(root), path)


def _make_scaled_model(
    source: Path,
    output_models: Path,
    original_name: str,
    generated_name: str,
    scale: float,
) -> None:
    target = output_models / generated_name
    shutil.copytree(source, target)

    sdf_path = target / "model.sdf"
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"No <model> in {sdf_path}")

    model.set("name", generated_name)
    _scale_tree_uniform(root, scale)
    _rewrite_model_uris(root, original_name, generated_name)
    _write_tree(tree, sdf_path)
    _write_clean_model_config(target / "model.config", generated_name)


def prepare_scaled_hospital(scale: float = HOSPITAL_SCALE) -> ScaledHospital:
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError(f"HOSPITAL_SCALE must be finite and > 0, got {scale!r}")

    source_world = SOURCE_WORLD.resolve()
    source_models = SOURCE_MODELS_DIR.resolve()
    output_root = GENERATED_ROOT.resolve()

    if not source_world.is_file():
        raise FileNotFoundError(f"Hospital world not found: {source_world}")

    for name in _LOCAL_MODELS:
        if not (source_models / name / "model.sdf").is_file():
            raise FileNotFoundError(
                f"Missing Hospital asset {source_models / name}. "
                "Run mapex_lab/scripts/setup_hospital_world_assets.sh first."
            )

    # IMPORTANT: geometry is uniformly scaled, but robot spawn uses Y-only
    # scaling by user choice.
    spawn_x = ORIGINAL_SPAWN_X
    spawn_y = ORIGINAL_SPAWN_Y * scale

    if abs(scale - 1.0) < 1e-12:
        return ScaledHospital(
            scale,
            source_world,
            source_models,
            spawn_x,
            spawn_y,
            ORIGINAL_SPAWN_YAW,
        )

    tag = _tag(scale)
    # Dedicated namespace avoids reusing either the old uniform cache or the
    # temporary Y-only-map cache.
    generated_dir = output_root / f"hospital_uniform_spawn_y_scale_{tag}"
    output_models = generated_dir / "models"
    output_world = generated_dir / f"hospital_aws_flat_uniform_scale_{tag}.sdf"
    manifest_path = generated_dir / "manifest.json"

    fingerprint = {
        "world_sha256": _sha256(source_world),
        "models": {
            name: _sha256(source_models / name / "model.sdf")
            for name in _LOCAL_MODELS
        },
    }

    if manifest_path.is_file() and output_world.is_file() and output_models.is_dir():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            if (
                manifest.get("scale") == scale
                and manifest.get("scale_mode") == SCALE_MODE
                and manifest.get("source_fingerprint") == fingerprint
            ):
                return ScaledHospital(
                    scale,
                    output_world,
                    output_models,
                    spawn_x,
                    spawn_y,
                    ORIGINAL_SPAWN_YAW,
                )
        except (OSError, ValueError, json.JSONDecodeError):
            pass

    shutil.rmtree(generated_dir, ignore_errors=True)
    output_models.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(source_world)
    root = tree.getroot()
    _scale_tree_uniform(root, scale)

    mapping: dict[str, str] = {}
    for include in root.iter("include"):
        uri = include.find("uri")
        if uri is None or not uri.text:
            continue
        original = uri.text.strip().removeprefix("model://")
        if original not in _LOCAL_MODELS:
            continue

        generated = f"{original}_uniform_scale_{tag}"
        _make_scaled_model(
            source_models / original,
            output_models,
            original,
            generated,
            scale,
        )
        uri.text = f"model://{generated}"
        mapping[original] = generated

    missing = sorted(set(_LOCAL_MODELS) - set(mapping))
    if missing:
        shutil.rmtree(generated_dir, ignore_errors=True)
        raise ValueError(f"Hospital world is missing expected models: {missing}")

    _write_tree(tree, output_world)
    manifest_path.write_text(
        json.dumps(
            {
                "scale": scale,
                "scale_mode": SCALE_MODE,
                "source_world": str(source_world),
                "generated_world": str(output_world),
                "source_fingerprint": fingerprint,
                "model_mapping": mapping,
                "spawn": {
                    "x": spawn_x,
                    "y": spawn_y,
                    "yaw": ORIGINAL_SPAWN_YAW,
                },
                "robot_scaled": False,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    return ScaledHospital(
        scale,
        output_world,
        output_models,
        spawn_x,
        spawn_y,
        ORIGINAL_SPAWN_YAW,
    )
