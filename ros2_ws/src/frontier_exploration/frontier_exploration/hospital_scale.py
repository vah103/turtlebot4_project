"""Automatic uniform scaling for the flat AWS Hospital simulation world.

Edit only ``HOSPITAL_SCALE`` when a different long-term Hospital scale is
wanted. The normal Hospital launch files consume this value automatically.
The TurtleBot4 itself is intentionally not scaled.
"""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


# Single source of truth for the simulation geometry scale.
# Keep 1.0 for the original Hospital. Example: 0.7 means 70% linear size.
HOSPITAL_SCALE = 1.0

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
    text = f"{scale:.6f}".rstrip("0").rstrip(".")
    return text.replace("-", "m").replace(".", "p")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _scale_numbers(text: str | None, scale: float, count: int) -> str | None:
    if text is None:
        return None
    values = text.split()
    if len(values) != count:
        raise ValueError(f"Expected {count} numbers, got {text!r}")
    return " ".join(_fmt(float(value) * scale) for value in values)


def _scale_pose(text: str | None, scale: float) -> str | None:
    if text is None:
        return None
    parts = text.split()
    if len(parts) not in (6, 7):
        raise ValueError(f"Unsupported <pose>: {text!r}")
    values = [float(value) for value in parts]
    values[0] *= scale
    values[1] *= scale
    values[2] *= scale
    return " ".join(_fmt(value) for value in values)


def _scale_tree(root: ET.Element, scale: float) -> None:
    for pose in root.iter("pose"):
        pose.text = _scale_pose(pose.text, scale)

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
            element = shape.find(name)
            if element is not None and element.text:
                element.text = _fmt(float(element.text) * scale)


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
    _scale_tree(root, scale)
    _rewrite_model_uris(root, original_name, generated_name)
    _write_tree(tree, sdf_path)

    config_path = target / "model.config"
    if config_path.is_file():
        config_tree = ET.parse(config_path)
        name = config_tree.getroot().find("name")
        if name is not None:
            name.text = generated_name
        _write_tree(config_tree, config_path)


def prepare_scaled_hospital(
    source_world: str | Path,
    source_models_dir: str | Path,
    output_root: str | Path | None = None,
    scale: float = HOSPITAL_SCALE,
) -> ScaledHospital:
    """Return the world/models to launch, generating the scaled copy if needed."""
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError(f"HOSPITAL_SCALE must be finite and > 0, got {scale!r}")

    source_world = Path(source_world).expanduser().resolve()
    source_models = Path(source_models_dir).expanduser().resolve()
    if not source_world.is_file():
        raise FileNotFoundError(f"Hospital world not found: {source_world}")

    spawn_x = ORIGINAL_SPAWN_X * scale
    spawn_y = ORIGINAL_SPAWN_Y * scale

    # 1.0 is exactly the current/original environment; avoid needless copies.
    if abs(scale - 1.0) < 1e-12:
        return ScaledHospital(
            scale=scale,
            world=source_world,
            models_dir=source_models,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_yaw=ORIGINAL_SPAWN_YAW,
        )

    for model_name in _LOCAL_MODELS:
        if not (source_models / model_name / "model.sdf").is_file():
            raise FileNotFoundError(
                f"Missing Hospital asset {source_models / model_name}. "
                "Run setup_hospital_world_assets.sh first."
            )

    if output_root is None:
        output_root = Path.home() / ".cache/turtlebot4_project/scaled_hospital"
    output_root = Path(output_root).expanduser().resolve()

    tag = _tag(scale)
    generated_dir = output_root / f"hospital_scale_{tag}"
    output_models = generated_dir / "models"
    output_world = generated_dir / f"hospital_aws_flat_scale_{tag}.sdf"
    manifest_path = generated_dir / "manifest.json"

    source_fingerprint = {
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
                and manifest.get("source_fingerprint") == source_fingerprint
            ):
                return ScaledHospital(
                    scale=scale,
                    world=output_world,
                    models_dir=output_models,
                    spawn_x=spawn_x,
                    spawn_y=spawn_y,
                    spawn_yaw=ORIGINAL_SPAWN_YAW,
                )
        except (OSError, ValueError, json.JSONDecodeError):
            pass

    shutil.rmtree(generated_dir, ignore_errors=True)
    output_models.mkdir(parents=True, exist_ok=True)

    tree = ET.parse(source_world)
    root = tree.getroot()
    _scale_tree(root, scale)

    mapping: dict[str, str] = {}
    for include in root.iter("include"):
        uri = include.find("uri")
        if uri is None or not uri.text:
            continue
        original_name = uri.text.strip().removeprefix("model://")
        if original_name not in _LOCAL_MODELS:
            continue

        generated_name = f"{original_name}_scale_{tag}"
        _make_scaled_model(
            source_models / original_name,
            output_models,
            original_name,
            generated_name,
            scale,
        )
        uri.text = f"model://{generated_name}"
        mapping[original_name] = generated_name

    missing = sorted(set(_LOCAL_MODELS) - set(mapping))
    if missing:
        shutil.rmtree(generated_dir, ignore_errors=True)
        raise ValueError(f"Hospital world is missing expected models: {missing}")

    _write_tree(tree, output_world)
    manifest_path.write_text(
        json.dumps(
            {
                "scale": scale,
                "source_world": str(source_world),
                "generated_world": str(output_world),
                "source_fingerprint": source_fingerprint,
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
        scale=scale,
        world=output_world,
        models_dir=output_models,
        spawn_x=spawn_x,
        spawn_y=spawn_y,
        spawn_yaw=ORIGINAL_SPAWN_YAW,
    )
