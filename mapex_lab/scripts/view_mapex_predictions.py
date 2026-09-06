#!/usr/bin/env python3
"""Visualize the observed SLAM map and saved MapEx predictions for one decision.

Examples
--------
From the repository root::

    python3 mapex_lab/scripts/view_mapex_predictions.py mapex_submap_001 10

Or pass the run directory directly::

    python3 mapex_lab/scripts/view_mapex_predictions.py \
        mapex_lab/experiments/mapex/mapex_submap_001 10

The script opens the observed map at that exact decision together with G1, G2,
G3, ensemble mean, and ensemble variance.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np


PREDICTION_NAMES = ("g1", "g2", "g3", "mean", "variance")
DISPLAY_NAMES = ("observed", "g1", "g2", "g3", "mean", "variance")
MAP_TITLES = {
    "observed": "Observed Map",
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


def raw_map_path(run_dir: Path, decision_id: int) -> Path:
    return run_dir / "decision_maps" / f"decision_{decision_id:06d}_raw.npz"


def available_decisions(prediction_dir: Path) -> list[int]:
    """Return decision ids for which a mean prediction file exists."""
    ids: list[int] = []
    for path in prediction_dir.glob("decision_*_mean.npz"):
        try:
            ids.append(int(path.stem.split("_")[1]))
        except (IndexError, ValueError):
            continue
    return sorted(set(ids))


def load_npz_map(path: Path) -> tuple[np.ndarray, dict[str, object]]:
    """Load a saved 2-D map and lightweight metadata."""
    with np.load(path, allow_pickle=False) as archive:
        if "data" not in archive.files:
            raise KeyError(f"Missing 'data' key in {path}")

        data = np.asarray(archive["data"])
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


def ros_occupancy_to_mapex(raw: np.ndarray) -> np.ndarray:
    """ROS OccupancyGrid labels -> MapEx display values: free=0, unknown=.5, occupied=1."""
    observed = np.full(raw.shape, 0.5, dtype=np.float32)
    observed[raw == 0] = 0.0
    observed[raw > 0] = 1.0
    return observed


def align_observed_to_prediction(
    observed: np.ndarray,
    prediction_shape: tuple[int, int],
    metadata: dict[str, object],
) -> np.ndarray:
    """Pad the raw observed map exactly as the saved LaMa prediction input was padded."""
    if observed.shape == prediction_shape:
        return observed.astype(np.float32, copy=False)

    aligned = np.full(prediction_shape, 0.5, dtype=np.float32)
    pad_top = int(metadata.get("pad_top", 0) or 0)
    pad_left = int(metadata.get("pad_left", 0) or 0)

    source_h = int(metadata.get("source_height", observed.shape[0]) or observed.shape[0])
    source_w = int(metadata.get("source_width", observed.shape[1]) or observed.shape[1])
    source_h = min(source_h, observed.shape[0])
    source_w = min(source_w, observed.shape[1])

    dst_h = max(0, min(source_h, prediction_shape[0] - pad_top))
    dst_w = max(0, min(source_w, prediction_shape[1] - pad_left))
    if dst_h <= 0 or dst_w <= 0:
        raise ValueError(
            "Cannot align observed map with prediction grid: "
            f"observed={observed.shape}, prediction={prediction_shape}, "
            f"pad_top={pad_top}, pad_left={pad_left}"
        )

    aligned[pad_top : pad_top + dst_h, pad_left : pad_left + dst_w] = observed[
        :dst_h, :dst_w
    ]
    return aligned


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Show the observed SLAM map plus G1/G2/G3, Mean Map and Variance Map "
            "saved by a MapEx run."
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
        for name in PREDICTION_NAMES
    }
    observed_path = raw_map_path(run_dir, args.decision)
    missing = [path for path in paths.values() if not path.is_file()]
    if not observed_path.is_file():
        missing.insert(0, observed_path)

    if missing:
        available = available_decisions(prediction_dir)
        print(
            "ERROR: missing map file(s):\n  "
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
        data, current_metadata = load_npz_map(path)
        loaded[name] = np.asarray(data, dtype=np.float32)
        if not metadata:
            metadata = current_metadata

    prediction_shapes = {name: value.shape for name, value in loaded.items()}
    if len(set(prediction_shapes.values())) != 1:
        print(
            f"ERROR: prediction shapes do not match: {prediction_shapes}",
            file=sys.stderr,
        )
        return 2

    prediction_shape = next(iter(loaded.values())).shape
    raw_observed, raw_metadata = load_npz_map(observed_path)
    observed = ros_occupancy_to_mapex(raw_observed)
    try:
        loaded["observed"] = align_observed_to_prediction(
            observed,
            prediction_shape,
            metadata,
        )
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    fig, axes = plt.subplots(1, 6, figsize=(24, 4.8), constrained_layout=True)
    origin = "upper" if args.no_flip else "lower"

    for ax, name in zip(axes, DISPLAY_NAMES):
        data = loaded[name]
        if name == "variance":
            image = ax.imshow(data, origin=origin)
        else:
            image = ax.imshow(data, cmap="gray", vmin=0.0, vmax=1.0, origin=origin)

        ax.set_title(MAP_TITLES[name])
        ax.set_xlabel(f"min={np.nanmin(data):.4f}  max={np.nanmax(data):.4f}")
        ax.set_xticks([])
        ax.set_yticks([])
        fig.colorbar(image, ax=ax, fraction=0.046, pad=0.03)

    resolution = metadata.get("resolution", raw_metadata.get("resolution"))
    environment = metadata.get("environment", "unknown")
    resolution_text = "?" if resolution is None else f"{float(resolution):.3f} m/cell"
    fig.suptitle(
        f"MapEx decision view | run={run_dir.name} | decision={args.decision} | "
        f"env={environment} | prediction shape={prediction_shape} | "
        f"resolution={resolution_text}"
    )

    print(f"Run          : {run_dir.name}")
    print(f"Decision     : {args.decision}")
    print(f"Observed raw : {observed_path}")
    print(f"Raw shape    : {raw_observed.shape}")
    print(f"Display shape: {prediction_shape}")
    print(f"Resolution   : {resolution_text}")
    for name in DISPLAY_NAMES:
        data = loaded[name]
        print(
            f"{MAP_TITLES[name]:12s}: min={np.nanmin(data):.6f}, "
            f"max={np.nanmax(data):.6f}, mean={np.nanmean(data):.6f}"
        )

    if args.save is not None:
        output_path = args.save.expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=180, bbox_inches="tight")
        print(f"Saved        : {output_path}")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
