#!/usr/bin/env python3
"""Visualize saved MapEx ensemble prediction maps for one decision.

Examples
--------
From the repository root::

    python3 mapex_lab/scripts/view_mapex_predictions.py mapex_submap_001 10

Or pass the run directory directly::

    python3 mapex_lab/scripts/view_mapex_predictions.py \
        mapex_lab/experiments/mapex/mapex_submap_001 10

The script opens G1, G2, G3, ensemble mean, and ensemble variance side by side.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np


MAP_NAMES = ("g1", "g2", "g3", "mean", "variance")
MAP_TITLES = {
    "g1": "G1",
    "g2": "G2",
    "g3": "G3",
    "mean": "Mean Map",
    "variance": "Variance Map",
}


def resolve_run_dir(run_arg: str) -> Path:
    """Resolve either a run id or an explicit run directory."""
    raw = Path(run_arg).expanduser()
    if raw.is_dir():
        return raw.resolve()

    repo_root = Path(__file__).resolve().parents[2]
    candidate = repo_root / "mapex_lab" / "experiments" / "mapex" / run_arg
    if candidate.is_dir():
        return candidate.resolve()

    raise FileNotFoundError(
        f"MapEx run not found: {run_arg}\n"
        f"Expected run id under {repo_root / 'mapex_lab/experiments/mapex'} "
        "or an explicit run directory."
    )


def prediction_path(prediction_dir: Path, decision_id: int, name: str) -> Path:
    return prediction_dir / f"decision_{decision_id:06d}_{name}.npz"


def available_decisions(prediction_dir: Path) -> list[int]:
    """Return decision ids for which a mean prediction file exists."""
    ids: list[int] = []
    for path in prediction_dir.glob("decision_*_mean.npz"):
        stem = path.stem
        try:
            ids.append(int(stem.split("_")[1]))
        except (IndexError, ValueError):
            continue
    return sorted(set(ids))


def load_prediction(path: Path) -> tuple[np.ndarray, dict[str, object]]:
    """Load the saved prediction array and lightweight metadata."""
    with np.load(path, allow_pickle=False) as archive:
        if "data" not in archive.files:
            raise KeyError(f"Missing 'data' key in {path}")

        data = np.asarray(archive["data"], dtype=np.float32)
        metadata: dict[str, object] = {}
        for key in (
            "member",
            "resolution",
            "source_height",
            "source_width",
            "pad_top",
            "pad_left",
            "origin_x",
            "origin_y",
            "environment",
        ):
            if key not in archive.files:
                continue
            value = archive[key]
            metadata[key] = value.item() if value.ndim == 0 else value.tolist()

    if data.ndim != 2:
        raise ValueError(f"Expected 2-D map in {path}, got shape {data.shape}")
    return data, metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Show G1/G2/G3, Mean Map and Variance Map saved by a MapEx run."
        )
    )
    parser.add_argument(
        "run",
        help=(
            "Run id such as mapex_submap_001, or a path to the MapEx run directory."
        ),
    )
    parser.add_argument("decision", type=int, help="Decision id, e.g. 1, 10, 28, 34")
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional output image path. The interactive window is still shown.",
    )
    parser.add_argument(
        "--no-flip",
        action="store_true",
        help=(
            "Do not vertically flip arrays before plotting. By default image origin is "
            "shown at the lower-left for map-style viewing."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.decision <= 0:
        print("ERROR: decision must be >= 1", file=sys.stderr)
        return 2

    try:
        run_dir = resolve_run_dir(args.run)
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    prediction_dir = run_dir / "predictions"
    if not prediction_dir.is_dir():
        print(
            f"ERROR: prediction directory not found: {prediction_dir}\n"
            "This run may have been recorded without --save-predictions.",
            file=sys.stderr,
        )
        return 2

    paths = {
        name: prediction_path(prediction_dir, args.decision, name)
        for name in MAP_NAMES
    }
    missing = [path for path in paths.values() if not path.is_file()]
    if missing:
        available = available_decisions(prediction_dir)
        print(
            "ERROR: missing prediction file(s):\n  "
            + "\n  ".join(str(path) for path in missing),
            file=sys.stderr,
        )
        if available:
            print(
                f"Available decisions: {available[0]}..{available[-1]} "
                f"({len(available)} mean-map files)",
                file=sys.stderr,
            )
        return 2

    loaded: dict[str, np.ndarray] = {}
    metadata: dict[str, object] = {}
    for name, path in paths.items():
        data, current_metadata = load_prediction(path)
        loaded[name] = data
        if not metadata:
            metadata = current_metadata

    shapes = {name: value.shape for name, value in loaded.items()}
    if len(set(shapes.values())) != 1:
        print(f"ERROR: prediction shapes do not match: {shapes}", file=sys.stderr)
        return 2

    fig, axes = plt.subplots(1, 5, figsize=(20, 4.8), constrained_layout=True)

    for ax, name in zip(axes, MAP_NAMES):
        data = loaded[name]
        origin = "upper" if args.no_flip else "lower"

        if name == "variance":
            image = ax.imshow(data, origin=origin)
        else:
            image = ax.imshow(data, cmap="gray", vmin=0.0, vmax=1.0, origin=origin)

        ax.set_title(MAP_TITLES[name])
        ax.set_xlabel(f"min={np.nanmin(data):.4f}  max={np.nanmax(data):.4f}")
        ax.set_xticks([])
        ax.set_yticks([])
        fig.colorbar(image, ax=ax, fraction=0.046, pad=0.03)

    resolution = metadata.get("resolution")
    environment = metadata.get("environment", "unknown")
    shape = next(iter(loaded.values())).shape
    resolution_text = "?" if resolution is None else f"{float(resolution):.3f} m/cell"
    fig.suptitle(
        f"MapEx predictions | run={run_dir.name} | decision={args.decision} | "
        f"env={environment} | shape={shape} | resolution={resolution_text}"
    )

    print(f"Run       : {run_dir.name}")
    print(f"Decision  : {args.decision}")
    print(f"Shape     : {shape}")
    print(f"Resolution: {resolution_text}")
    print(f"Files     : {prediction_dir}")
    for name in MAP_NAMES:
        data = loaded[name]
        print(
            f"{MAP_TITLES[name]:12s}: min={np.nanmin(data):.6f}, "
            f"max={np.nanmax(data):.6f}, mean={np.nanmean(data):.6f}"
        )

    if args.save is not None:
        output_path = args.save.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=180, bbox_inches="tight")
        print(f"Saved     : {output_path}")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
