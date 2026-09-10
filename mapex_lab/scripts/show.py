#!/usr/bin/env python3
"""Show one recorded MapEx decision as a 2x3 map panel.

Layout:
    obs | mean | var
    g1  | g2   | g3

Known cells from the observed SLAM map are kept unchanged in every prediction
panel. Only cells that were unknown in the observed map are colored by the
corresponding MapEx prediction. The variance panel similarly colors only the
unknown part of the observed map.

Examples:
    python3 mapex_lab/scripts/show.py
    python3 mapex_lab/scripts/show.py --run-id run_01
    python3 mapex_lab/scripts/show.py --run-id run_01 --decision 12
    python3 mapex_lab/scripts/show.py --run-id run_01 --decision 12 --save /tmp/mapex_12.png
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MAPEX_RUNS = ROOT / "experiments" / "mapex"


def _latest_run() -> Path:
    if not MAPEX_RUNS.is_dir():
        raise FileNotFoundError(f"MapEx experiment directory not found: {MAPEX_RUNS}")
    runs = [path for path in MAPEX_RUNS.iterdir() if path.is_dir()]
    if not runs:
        raise FileNotFoundError(f"No MapEx runs found under: {MAPEX_RUNS}")
    return max(runs, key=lambda path: path.stat().st_mtime)


def _resolve_run(run_id: str | None) -> Path:
    if not run_id:
        return _latest_run()

    supplied = Path(run_id).expanduser()
    if supplied.is_dir():
        return supplied.resolve()

    run = MAPEX_RUNS / run_id
    if not run.is_dir():
        raise FileNotFoundError(
            f"MapEx run not found: {run_id}\n"
            f"Expected either a run directory or a name under {MAPEX_RUNS}"
        )
    return run.resolve()


def _decision_rows(run: Path) -> list[dict[str, str]]:
    path = run / "decisions.csv"
    if not path.is_file():
        raise FileNotFoundError(f"Missing decisions.csv: {path}")
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))

    # Only decisions with the full saved prediction ensemble can be displayed.
    required = ("raw_map", "g1_map", "g2_map", "g3_map", "mean_map", "variance_map")
    rows = [row for row in rows if all((row.get(key) or "").strip() for key in required)]
    if not rows:
        raise RuntimeError(
            "No decision contains observed + G1/G2/G3/mean/variance. "
            "Make sure the run used prediction saving."
        )
    return rows


def _select_decision(run: Path, decision: str) -> dict[str, str]:
    rows = _decision_rows(run)
    if decision == "latest":
        return max(rows, key=lambda row: int(row["decision_id"]))

    wanted = int(decision)
    for row in rows:
        if int(row["decision_id"]) == wanted:
            return row
    available = [int(row["decision_id"]) for row in rows]
    raise ValueError(
        f"Decision {wanted} has no complete six-map record. "
        f"Available range: {min(available)}..{max(available)}"
    )


def _load_npz(run: Path, relative_path: str) -> tuple[np.ndarray, dict[str, object]]:
    path = run / relative_path
    if not path.is_file():
        raise FileNotFoundError(f"Missing recorded map: {path}")
    with np.load(path, allow_pickle=False) as archive:
        if "data" not in archive:
            raise KeyError(f"NPZ has no 'data' array: {path}")
        data = np.asarray(archive["data"])
        meta = {key: archive[key].item() if archive[key].ndim == 0 else archive[key] for key in archive.files if key != "data"}
    return data, meta


def _crop_prediction(pred: np.ndarray, meta: dict[str, object], observed_shape: tuple[int, int]) -> np.ndarray:
    """Crop a padded LaMa prediction back to the source observed-map extent."""
    source_h, source_w = observed_shape
    if pred.shape == observed_shape:
        return pred.astype(np.float32, copy=False)

    meta_h = int(meta.get("source_height", source_h))
    meta_w = int(meta.get("source_width", source_w))
    if (meta_h, meta_w) != observed_shape:
        raise ValueError(
            f"Prediction metadata source shape {(meta_h, meta_w)} does not match "
            f"observed shape {observed_shape}"
        )

    pad_top = int(meta.get("pad_top", max(0, (pred.shape[0] - source_h) // 2)))
    pad_left = int(meta.get("pad_left", max(0, (pred.shape[1] - source_w) // 2)))
    cropped = pred[pad_top : pad_top + source_h, pad_left : pad_left + source_w]
    if cropped.shape != observed_shape:
        raise ValueError(
            f"Could not align prediction shape {pred.shape} to observed shape {observed_shape}; "
            f"crop produced {cropped.shape}"
        )
    return cropped.astype(np.float32, copy=False)


def _observed_rgb(observed: np.ndarray) -> np.ndarray:
    """ROS occupancy map: unknown gray, free white, occupied black."""
    rgb = np.full((*observed.shape, 3), 0.65, dtype=np.float32)
    rgb[observed == 0] = 1.0
    rgb[observed > 0] = 0.0
    return rgb


def _prediction_overlay(observed: np.ndarray, prediction: np.ndarray) -> np.ndarray:
    """Keep observed cells unchanged and color prediction only over unknown cells."""
    rgb = _observed_rgb(observed)
    unknown = observed < 0
    values = np.clip(prediction.astype(np.float32), 0.0, 1.0)
    colors = plt.get_cmap("coolwarm")(values)[..., :3]
    rgb[unknown] = colors[unknown]
    return rgb


def _variance_overlay(observed: np.ndarray, variance: np.ndarray) -> np.ndarray:
    """Keep observed cells unchanged and heat-map variance only over unknown cells."""
    rgb = _observed_rgb(observed)
    unknown = observed < 0
    values = np.maximum(variance.astype(np.float32), 0.0)
    finite_unknown = values[unknown & np.isfinite(values)]
    if finite_unknown.size:
        vmax = float(np.percentile(finite_unknown, 99.0))
        if vmax <= 0.0:
            vmax = float(np.max(finite_unknown))
    else:
        vmax = 0.0
    normalized = np.zeros_like(values, dtype=np.float32) if vmax <= 0.0 else np.clip(values / vmax, 0.0, 1.0)
    colors = plt.get_cmap("magma")(normalized)[..., :3]
    rgb[unknown] = colors[unknown]
    return rgb


def main() -> None:
    parser = argparse.ArgumentParser(description="Show six maps from one recorded MapEx decision")
    parser.add_argument(
        "--run-id",
        default=None,
        help="MapEx run name or run directory. Default: most recently modified MapEx run.",
    )
    parser.add_argument(
        "--decision",
        default="latest",
        help="Decision id to show, or 'latest' (default).",
    )
    parser.add_argument(
        "--save",
        default=None,
        help="Optional PNG/PDF output path. The interactive window is still shown.",
    )
    args = parser.parse_args()

    run = _resolve_run(args.run_id)
    row = _select_decision(run, args.decision)
    decision_id = int(row["decision_id"])

    observed, _ = _load_npz(run, row["raw_map"])
    if observed.ndim != 2:
        raise ValueError(f"Observed map must be 2-D, got {observed.shape}")

    arrays: dict[str, np.ndarray] = {}
    for name, column in (
        ("g1", "g1_map"),
        ("g2", "g2_map"),
        ("g3", "g3_map"),
        ("mean", "mean_map"),
        ("var", "variance_map"),
    ):
        pred, meta = _load_npz(run, row[column])
        if pred.ndim != 2:
            raise ValueError(f"{name} map must be 2-D, got {pred.shape}")
        arrays[name] = _crop_prediction(pred, meta, observed.shape)

    panels = [
        ("obs", _observed_rgb(observed)),
        ("mean", _prediction_overlay(observed, arrays["mean"])),
        ("var", _variance_overlay(observed, arrays["var"])),
        ("g1", _prediction_overlay(observed, arrays["g1"])),
        ("g2", _prediction_overlay(observed, arrays["g2"])),
        ("g3", _prediction_overlay(observed, arrays["g3"])),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(14, 9), constrained_layout=True)
    for ax, (title, image) in zip(axes.flat, panels):
        # ROS occupancy-grid row 0 starts at the map origin, so origin='lower'
        # displays the map in the same orientation as map coordinates.
        ax.imshow(image, origin="lower", interpolation="nearest")
        ax.set_title(title)
        ax.set_axis_off()

    fig.suptitle(f"{run.name} | decision {decision_id:06d}")

    if args.save:
        output = Path(args.save).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output, dpi=180, bbox_inches="tight")
        print(f"Saved: {output}")

    print(f"Run: {run}")
    print(f"Decision: {decision_id}")
    plt.show()


if __name__ == "__main__":
    main()
