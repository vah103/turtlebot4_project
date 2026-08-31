#!/usr/bin/env python3
"""
Generate a uniformly scaled copy of the flat AWS Hospital simulation world.

Default usage:
    python3 mapex_lab/scripts/scale_hospital.py 0.5

Optional custom world:
    python3 mapex_lab/scripts/scale_hospital.py 0.75 --input path/to/world.sdf

The source world and cached model assets are never modified.
Only Python standard-library modules are required.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


LOCAL_MODEL_URIS = {
    "model://aws_robomaker_hospital_floor_01_walls":
        "aws_robomaker_hospital_floor_01_walls",
    "model://aws_robomaker_hospital_curtain_closed_01":
        "aws_robomaker_hospital_curtain_closed_01",
}

DEFAULT_SPAWN_X = 0.0
DEFAULT_SPAWN_Y = 12.0
DEFAULT_SPAWN_YAW = -1.57


def _fmt(value: float) -> str:
    if abs(value) < 5e-13:
        value = 0.0
    return f"{value:.12g}"


def _scale_numbers(text: str | None, scale: float, count: int) -> str | None:
    if text is None:
        return None
    parts = text.split()
    if len(parts) != count:
        raise ValueError(f"Expected {count} numbers, got {len(parts)} in {text!r}")
    return " ".join(_fmt(float(value) * scale) for value in parts)


def _scale_pose_text(text: str | None, scale: float) -> str | None:
    if text is None:
        return None
    parts = text.split()
    if len(parts) not in (6, 7):
        raise ValueError(f"Unsupported <pose> format: {text!r}")
    values = [float(value) for value in parts]
    values[0] *= scale
    values[1] *= scale
    values[2] *= scale
    return " ".join(_fmt(value) for value in values)


def _scale_pose_elements(root: ET.Element, scale: float) -> None:
    for pose in root.iter("pose"):
        pose.text = _scale_pose_text(pose.text, scale)


def _scale_geometry(root: ET.Element, scale: float) -> None:
    # Scale meshes through SDF so the original DAE/STL files stay untouched.
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

    for sphere in root.iter("sphere"):
        radius = sphere.find("radius")
        if radius is not None and radius.text:
            radius.text = _fmt(float(radius.text) * scale)

    for cylinder in root.iter("cylinder"):
        for name in ("radius", "length"):
            element = cylinder.find(name)
            if element is not None and element.text:
                element.text = _fmt(float(element.text) * scale)

    for capsule in root.iter("capsule"):
        for name in ("radius", "length"):
            element = capsule.find(name)
            if element is not None and element.text:
                element.text = _fmt(float(element.text) * scale)

    for plane in root.iter("plane"):
        size = plane.find("size")
        if size is not None:
            size.text = _scale_numbers(size.text, scale, 2)


def _rewrite_model_uris(root: ET.Element, old_name: str, new_name: str) -> None:
    prefix = f"model://{old_name}/"
    replacement = f"model://{new_name}/"
    for uri in root.iter("uri"):
        if uri.text and uri.text.startswith(prefix):
            uri.text = replacement + uri.text[len(prefix):]


def _indent_and_write(tree: ET.ElementTree, path: Path) -> None:
    ET.indent(tree, space="  ")
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(path, encoding="utf-8", xml_declaration=True)


def _scale_model_copy(
    source_dir: Path,
    output_models_dir: Path,
    original_name: str,
    generated_name: str,
    scale: float,
) -> None:
    if not source_dir.is_dir():
        raise FileNotFoundError(
            f"Missing Hospital model asset: {source_dir}\n"
            "Run setup_hospital_world_assets.sh first."
        )

    target_dir = output_models_dir / generated_name
    shutil.copytree(source_dir, target_dir)

    sdf_path = target_dir / "model.sdf"
    if not sdf_path.is_file():
        raise FileNotFoundError(f"Missing model.sdf in {source_dir}")

    tree = ET.parse(sdf_path)
    root = tree.getroot()
    model = root.find("model")
    if model is None:
        raise ValueError(f"No <model> element in {sdf_path}")

    model.set("name", generated_name)
    _scale_pose_elements(root, scale)
    _scale_geometry(root, scale)
    _rewrite_model_uris(root, original_name, generated_name)
    _indent_and_write(tree, sdf_path)

    config_path = target_dir / "model.config"
    if config_path.is_file():
        config_tree = ET.parse(config_path)
        config_root = config_tree.getroot()
        name_el = config_root.find("name")
        if name_el is not None:
            name_el.text = generated_name
        _indent_and_write(config_tree, config_path)


def _tag_for_scale(scale: float) -> str:
    text = f"{scale:.6f}".rstrip("0").rstrip(".")
    return text.replace("-", "m").replace(".", "p")


def _repo_root() -> Path:
    # .../mapex_lab/scripts/scale_hospital.py -> repository root
    return Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    repo_root = _repo_root()
    default_world = (
        repo_root
        / "ros2_ws/src/frontier_exploration/worlds/hospital_aws_flat.sdf"
    )
    default_assets = (
        Path.home()
        / ".cache/turtlebot4_project/hospital_world/models"
    )
    default_output = (
        Path.home()
        / ".cache/turtlebot4_project/scaled_hospital"
    )

    parser = argparse.ArgumentParser(
        description=(
            "Create a uniformly scaled copy of the flat AWS Hospital SDF world "
            "without modifying the source world or source model assets."
        )
    )
    parser.add_argument(
        "scale",
        type=float,
        help="Uniform scale factor, for example 0.5, 0.75 or 1.2.",
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=default_world,
        help=f"Input Hospital world SDF (default: {default_world}).",
    )
    parser.add_argument(
        "--assets-root",
        type=Path,
        default=default_assets,
        help=f"Directory containing Hospital model folders (default: {default_assets}).",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=default_output,
        help=f"Parent directory for generated worlds (default: {default_output}).",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing generated directory for the same scale.",
    )
    parser.add_argument(
        "--spawn-x",
        type=float,
        default=DEFAULT_SPAWN_X,
        help=(
            "Reference robot spawn x to scale in the printed launch hint "
            f"(default: {DEFAULT_SPAWN_X})."
        ),
    )
    parser.add_argument(
        "--spawn-y",
        type=float,
        default=DEFAULT_SPAWN_Y,
        help=(
            "Reference robot spawn y to scale in the printed launch hint "
            f"(default: {DEFAULT_SPAWN_Y})."
        ),
    )
    parser.add_argument(
        "--spawn-yaw",
        type=float,
        default=DEFAULT_SPAWN_YAW,
        help=(
            "Reference robot spawn yaw; yaw is not scaled "
            f"(default: {DEFAULT_SPAWN_YAW})."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    scale = float(args.scale)
    if not math.isfinite(scale) or scale <= 0.0:
        print("ERROR: scale must be a finite number > 0.", file=sys.stderr)
        return 2

    input_world = args.input.expanduser().resolve()
    assets_root = args.assets_root.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()

    if not input_world.is_file():
        print(f"ERROR: input world not found: {input_world}", file=sys.stderr)
        return 2

    tag = _tag_for_scale(scale)
    generated_dir = output_root / f"hospital_scale_{tag}"
    output_models_dir = generated_dir / "models"
    output_world = generated_dir / f"hospital_aws_flat_scale_{tag}.sdf"

    if generated_dir.exists():
        if not args.overwrite:
            print(
                f"ERROR: output already exists: {generated_dir}\n"
                "Use --overwrite to regenerate it.",
                file=sys.stderr,
            )
            return 2
        shutil.rmtree(generated_dir)

    generated_dir.mkdir(parents=True, exist_ok=True)
    output_models_dir.mkdir(parents=True, exist_ok=True)

    try:
        tree = ET.parse(input_world)
        root = tree.getroot()

        # Uniform scale of all explicit world positions and inline geometry.
        # Physics, gravity, material values and angular values stay unchanged.
        _scale_pose_elements(root, scale)
        _scale_geometry(root, scale)

        generated_models: dict[str, str] = {}
        for include in root.iter("include"):
            uri_el = include.find("uri")
            if uri_el is None or not uri_el.text:
                continue
            uri = uri_el.text.strip()
            original_name = LOCAL_MODEL_URIS.get(uri)
            if original_name is None:
                continue

            generated_name = f"{original_name}_scale_{tag}"
            _scale_model_copy(
                source_dir=assets_root / original_name,
                output_models_dir=output_models_dir,
                original_name=original_name,
                generated_name=generated_name,
                scale=scale,
            )
            uri_el.text = f"model://{generated_name}"
            generated_models[original_name] = generated_name

        missing = sorted(set(LOCAL_MODEL_URIS.values()) - set(generated_models))
        if missing:
            raise ValueError(
                "Input world does not contain the expected local Hospital model "
                f"includes: {', '.join(missing)}"
            )

        _indent_and_write(tree, output_world)
    except Exception as exc:
        shutil.rmtree(generated_dir, ignore_errors=True)
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    spawn_x = float(args.spawn_x) * scale
    spawn_y = float(args.spawn_y) * scale
    spawn_yaw = float(args.spawn_yaw)

    manifest = {
        "source_world": str(input_world),
        "scale": scale,
        "generated_world": str(output_world),
        "generated_models_dir": str(output_models_dir),
        "source_assets_root": str(assets_root),
        "model_mapping": generated_models,
        "reference_spawn": {
            "source": {
                "x": float(args.spawn_x),
                "y": float(args.spawn_y),
                "yaw": spawn_yaw,
            },
            "scaled": {
                "x": spawn_x,
                "y": spawn_y,
                "yaw": spawn_yaw,
            },
        },
        "notes": [
            "Robot geometry is not scaled.",
            "SLAM/Nav2/ROI/canvas parameters are not changed by this generator.",
            "A scaled world is a different benchmark environment unless every "
            "compared method uses the same scale and evaluation setup.",
        ],
    }
    manifest_path = generated_dir / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"Scaled Hospital generated successfully: x{_fmt(scale)}")
    print(f"World:    {output_world}")
    print(f"Models:   {output_models_dir}")
    print(f"Manifest: {manifest_path}")
    print()
    print("Use the generated world with Gazebo/Nav2:")
    print(
        f'export GZ_SIM_RESOURCE_PATH="{output_models_dir}:'
        '${GZ_SIM_RESOURCE_PATH:-}"'
    )
    print(
        "ros2 launch frontier_exploration tb4_simulation_safe.launch.py "
        f'world:="{output_world}" '
        f"x_pose:={_fmt(spawn_x)} "
        f"y_pose:={_fmt(spawn_y)} "
        f"yaw:={_fmt(spawn_yaw)}"
    )
    print()
    print(
        "NOTE: Hospital geometry and positions are uniformly scaled, while "
        "the TurtleBot4 itself remains at its original physical size."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
