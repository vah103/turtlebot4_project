"""Prepare the three preselected new layouts with V1 KTH preprocessing."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

from .core import FOUR, clearance_free
from .predictor import file_hash
from .run import write_json

BASE = Path(__file__).resolve().parent


def convert(raw, valid):
    # Conservatively pad odd dimensions: occupied outside the source image.
    h, w = raw.shape
    pad = ((0, h % 2), (0, w % 2))
    occupied = np.pad(raw == 0, pad, constant_values=True)
    valid = np.pad(valid > 0, pad, constant_values=False)
    hh, ww = occupied.shape[0]//2, occupied.shape[1]//2
    occupied = occupied.reshape(hh, 2, ww, 2).any(axis=(1, 3))
    valid = valid.reshape(hh, 2, ww, 2).any(axis=(1, 3))
    occupied |= ~valid
    labels, _ = ndi.label(~occupied, structure=FOUR)
    counts = np.bincount(labels.ravel()); counts[0] = 0
    if counts.max() == 0:
        raise ValueError("No connected free component")
    domain = labels == counts.argmax()
    start = np.unravel_index(ndi.distance_transform_edt(domain).argmax(), domain.shape)
    if not clearance_free(~occupied, 1.5)[start]:
        raise ValueError("Maximum-clearance start is not footprint feasible")
    return dict(occupied=occupied, domain=domain, resolution=np.array(.1), start=np.asarray(start))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mapex-root", type=Path, default=Path.home()/"MapEx")
    args = parser.parse_args()
    cfg = json.loads((BASE/"protocol_v2.json").read_text())
    # Reproduce a V1 asset before creating new assets; no predictions/results used.
    root = args.mapex_root/"kth_test_maps"
    old = convert(np.load(root/"50052751/occ_map.npy"), np.load(root/"50052751/valid_space.npy"))
    with np.load(BASE/"assets/kth_50052751.npz") as z:
        if not all(np.array_equal(old[k], z[k]) for k in old):
            raise RuntimeError("Preprocessing does not reproduce V1")
    sources = []
    for layout in cfg["confirmation_layouts"]:
        folder = root/layout.removeprefix("kth_")
        source, valid_source = folder/"occ_map.npy", folder/"valid_space.npy"
        raw, valid = np.load(source), np.load(valid_source)
        asset = convert(raw, valid)
        path = BASE/"assets"/(layout+".npz")
        if path.exists():
            raise RuntimeError("New asset already exists; do not overwrite")
        np.savez_compressed(path, **asset)
        sources.append(dict(layout=layout, source=str(source), sha256=file_hash(source),
                            valid_source=str(valid_source), valid_source_sha256=file_hash(valid_source),
                            raw_shape=list(raw.shape), shape=list(asset["occupied"].shape),
                            start=asset["start"].tolist(), asset_sha256=file_hash(path),
                            preprocess="V1: 2x2 any occupied, any valid (>0); outside valid occupied; largest 4-connected free domain, max-clearance start. Odd dimensions padded occupied/invalid."))
        print("NEW_ASSET", layout, asset["occupied"].shape, flush=True)
    write_json(BASE/"assets/sources_v2.json", sources)


if __name__ == "__main__":
    main()
