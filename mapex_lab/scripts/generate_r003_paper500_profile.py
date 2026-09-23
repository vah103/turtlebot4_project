#!/usr/bin/env python3
"""Generate R003 paper500 masks, fixed TU goals, manifest and audit overlays."""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from r003_paper500 import EVAL_ID, build_profile, reduce_observed


def _git_commit(repo: Path) -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo, check=True, capture_output=True, text=True
    ).stdout.strip()


def render_audits(output_dir: Path, observed_path: Path | None) -> list[str]:
    profile_path = output_dir / f"{EVAL_ID}.npz"
    with np.load(profile_path) as bundle:
        occupied = bundle["occupied"].astype(bool)
        valid = bundle["valid_space"].astype(bool)
        start = (int(bundle["start_row"]), int(bundle["start_col"]))
    audit = np.full(occupied.shape, 0.15, dtype=np.float32)
    audit[valid] = 0.85
    audit[occupied] = 0.0
    paths = []
    fig, ax = plt.subplots(figsize=(12, 8), constrained_layout=True)
    ax.imshow(audit, origin="lower", cmap="gray", vmin=0, vmax=1)
    ax.scatter([start[1]], [start[0]], c="tab:red", marker="*", s=80, label="start (0,0)")
    ax.set_title("R003 structural GT (black) / valid-space (white)")
    ax.legend(loc="upper right")
    path = output_dir / "r003_gt_valid_overlay.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    paths.append(str(path))
    if observed_path is not None:
        with np.load(observed_path) as bundle:
            observed = np.asarray(bundle["data"])
        observed10 = reduce_observed(observed)
        observed_occ = observed10 > 0
        gt_occ = occupied & valid
        rgb = np.full((*occupied.shape, 3), (0.12, 0.12, 0.12), dtype=np.float32)
        rgb[valid] = (0.86, 0.86, 0.86)
        rgb[gt_occ & observed_occ] = (0.10, 0.60, 0.25)  # match
        rgb[gt_occ & ~observed_occ] = (0.02, 0.02, 0.02)  # GT only
        rgb[~gt_occ & observed_occ & valid] = (0.90, 0.12, 0.12)  # observed only
        fig, axes = plt.subplots(1, 2, figsize=(16, 8), constrained_layout=True)
        axes[0].imshow(audit, origin="lower", cmap="gray", vmin=0, vmax=1)
        axes[0].set_title("R003 GT / valid-space")
        axes[1].imshow(rgb, origin="lower")
        axes[1].set_title("Aligned observed overlay: green match, black GT-only, red observed-only")
        for ax in axes:
            ax.set_xlim(100, 420)
            ax.set_ylim(440, 700)
        path = output_dir / "r003_gt_vs_observed.png"
        fig.savefig(path, dpi=180)
        plt.close(fig)
        paths.append(str(path))
    return paths


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdf", type=Path, default=root / "map" / "new_room.sdf")
    parser.add_argument(
        "--output-dir", type=Path,
        default=root / "ground_truth" / "new_room" / "generated" / "r003_paper500",
    )
    parser.add_argument("--observed-map", type=Path)
    args = parser.parse_args()
    output = args.output_dir.expanduser().resolve()
    manifest = build_profile(args.sdf.expanduser().resolve(), output, git_commit=_git_commit(root.parent))
    manifest["audit_overlays"] = render_audits(output, args.observed_map)
    manifest_path = output / f"{EVAL_ID}_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
