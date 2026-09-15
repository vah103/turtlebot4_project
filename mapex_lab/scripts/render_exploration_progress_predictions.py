#!/usr/bin/env python3
"""Render NF vs MapEx exploration progress with MapEx prediction overlay.

This figure is intended to mirror the qualitative role of Fig. 5 in the MapEx
paper while staying faithful to the data recorded by this project:

- Nearest Frontier row: observed SLAM map only.
- MapEx row: observed SLAM map + ensemble-mean predicted occupied structure,
  but ONLY inside cells that are still unknown in the observed map.

Default representative runs / checkpoints used by the thesis comparison:
- NF    nf_004 : decisions 9, 19, 30   (~100, ~300, ~500 s)
- MapEx mpx_004: decisions 7, 15, 20   (~100, ~300, ~500 s)

The prediction overlay does not rerun LaMa. It uses the saved
``decision_XXXXXX_mean.npz`` files from the recorded MapEx run.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from view_mapex_predictions import (
    align_observed_to_prediction,
    load_npz_map,
    ros_occupancy_to_mapex,
)


DEFAULT_TIMES = (100, 300, 500)
DEFAULT_NF_DECISIONS = (9, 19, 30)
DEFAULT_MAPEX_DECISIONS = (7, 15, 20)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _run_dir(method: str, run: str) -> Path:
    path = _repo_root() / "mapex_lab" / "experiments" / method / run
    if not path.is_dir():
        raise FileNotFoundError(path)
    return path


def _raw_path(run_dir: Path, decision_id: int) -> Path:
    """Return the saved observed raw map for either recorder layout.

    MapEx runs currently have the compact ``decision_maps`` layout, while the
    NF benchmark runs keep the observed map inside
    ``decisions/policy_decision_XXXXXX``. Supporting both makes the renderer
    work on the existing recorded benchmark without copying or regenerating
    data.
    """
    candidates = (
        run_dir / "decision_maps" / f"decision_{decision_id:06d}_raw.npz",
        run_dir
        / "decisions"
        / f"policy_decision_{decision_id:06d}"
        / "observed_map_raw.npz",
    )
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(
        "Observed map not found for decision "
        f"{decision_id} in {run_dir}. Checked:\n  "
        + "\n  ".join(str(path) for path in candidates)
    )


def _mean_path(run_dir: Path, decision_id: int) -> Path:
    return run_dir / "predictions" / f"decision_{decision_id:06d}_mean.npz"


def _observed_rgb(raw: np.ndarray) -> np.ndarray:
    """Render ROS occupancy labels as paper-style observed/unknown colors."""
    rgb = np.empty((*raw.shape, 3), dtype=np.float32)
    # Unknown area: light gray.
    rgb[:] = (0.76, 0.76, 0.76)
    # Observed free space: white.
    rgb[raw == 0] = (1.0, 1.0, 1.0)
    # Observed occupied: black.
    rgb[raw > 0] = (0.0, 0.0, 0.0)
    return rgb


def _mapex_rgb(raw: np.ndarray, mean_path: Path, threshold: float) -> np.ndarray:
    """Observed map plus blue predicted occupied cells in currently unknown area."""
    mean, metadata = load_npz_map(mean_path)
    observed = ros_occupancy_to_mapex(raw)
    aligned = align_observed_to_prediction(observed, mean.shape, metadata)

    rgb = np.empty((*mean.shape, 3), dtype=np.float32)
    rgb[:] = (0.76, 0.76, 0.76)  # currently unknown

    known_free = np.isclose(aligned, 0.0)
    known_occ = np.isclose(aligned, 1.0)
    unknown = np.isclose(aligned, 0.5)

    rgb[known_free] = (1.0, 1.0, 1.0)
    rgb[known_occ] = (0.0, 0.0, 0.0)

    # The ensemble mean preserves known observed cells. Therefore this mask
    # isolates only model-predicted occupied structure in unobserved space.
    predicted_occ = unknown & (mean > threshold)
    rgb[predicted_occ] = (0.08, 0.22, 0.95)
    return rgb


def _parse_triplet(values: list[int], name: str) -> tuple[int, int, int]:
    if len(values) != 3:
        raise ValueError(f"{name} requires exactly 3 integers")
    return tuple(int(v) for v in values)  # type: ignore[return-value]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Render a 2x3 NF vs MapEx progress figure with MapEx predicted-map overlay."
    )
    parser.add_argument("--nf-run", default="nf_004")
    parser.add_argument("--mapex-run", default="mpx_004")
    parser.add_argument("--times", nargs=3, type=int, default=list(DEFAULT_TIMES))
    parser.add_argument(
        "--nf-decisions", nargs=3, type=int, default=list(DEFAULT_NF_DECISIONS)
    )
    parser.add_argument(
        "--mapex-decisions",
        nargs=3,
        type=int,
        default=list(DEFAULT_MAPEX_DECISIONS),
    )
    parser.add_argument(
        "--prediction-threshold",
        type=float,
        default=0.5,
        help="Ensemble-mean occupied threshold for blue predicted structure.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("mapex_lab/figures/exploration_progress_prediction.png"),
    )
    parser.add_argument("--dpi", type=int, default=240)
    parser.add_argument(
        "--show",
        action="store_true",
        help="Also open an interactive Matplotlib window.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    times = _parse_triplet(args.times, "--times")
    nf_decisions = _parse_triplet(args.nf_decisions, "--nf-decisions")
    mapex_decisions = _parse_triplet(args.mapex_decisions, "--mapex-decisions")

    nf_dir = _run_dir("nearest", args.nf_run)
    mapex_dir = _run_dir("mapex", args.mapex_run)

    fig, axes = plt.subplots(2, 3, figsize=(15.0, 7.2))
    fig.subplots_adjust(left=0.12, right=0.995, top=0.88, bottom=0.08, wspace=0.03, hspace=0.05)

    for col, (target_t, nf_id, mapex_id) in enumerate(
        zip(times, nf_decisions, mapex_decisions)
    ):
        nf_raw, _ = load_npz_map(_raw_path(nf_dir, nf_id))
        mapex_raw, _ = load_npz_map(_raw_path(mapex_dir, mapex_id))

        nf_rgb = _observed_rgb(nf_raw)
        mapex_rgb = _mapex_rgb(
            mapex_raw,
            _mean_path(mapex_dir, mapex_id),
            float(args.prediction_threshold),
        )

        ax_nf = axes[0, col]
        ax_mx = axes[1, col]
        ax_nf.imshow(nf_rgb, origin="lower", interpolation="nearest")
        ax_mx.imshow(mapex_rgb, origin="lower", interpolation="nearest")

        ax_nf.set_title(f"t = {target_t} s", fontsize=14, fontweight="bold", pad=8)
        for ax in (ax_nf, ax_mx):
            ax.set_xticks([])
            ax.set_yticks([])
            for spine in ax.spines.values():
                spine.set_linewidth(0.8)
                spine.set_edgecolor("0.35")

    axes[0, 0].set_ylabel("Nearest Frontier", fontsize=14, fontweight="bold", rotation=0, labelpad=68, va="center")
    axes[1, 0].set_ylabel("MapEx", fontsize=14, fontweight="bold", rotation=0, labelpad=68, va="center")

    fig.suptitle(
        "Exploration Progress over Time - representative runs",
        fontsize=17,
        fontweight="bold",
    )

    # Compact legend matching the intended paper-style semantics.
    from matplotlib.patches import Patch

    legend_handles = [
        Patch(facecolor="white", edgecolor="black", label="Observed Map"),
        Patch(facecolor=(0.08, 0.22, 0.95), edgecolor="black", label="Predicted Map (MapEx only)"),
        Patch(facecolor=(0.76, 0.76, 0.76), edgecolor="black", label="Unknown Area"),
    ]
    fig.legend(
        handles=legend_handles,
        loc="lower center",
        ncol=3,
        frameon=False,
        fontsize=11,
        bbox_to_anchor=(0.57, 0.0),
    )

    output = args.output.expanduser()
    if not output.is_absolute():
        output = _repo_root() / output
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=int(args.dpi), bbox_inches="tight", facecolor="white")
    print(f"Saved: {output}")
    print(f"NF:    {args.nf_run} decisions {nf_decisions}")
    print(f"MapEx: {args.mapex_run} decisions {mapex_decisions}")
    print(f"Times: {times}")
    print(f"Predicted occupied threshold: {args.prediction_threshold}")

    if args.show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
