"""Prepare geometry with system Python; consume frozen arrays with model Python.

Uses the exact pinned GT generator, including its pose parser and tolerances.
The model environment never imports the Python 3.10+ generator.
"""
import argparse
import importlib.util
import math
from decimal import Decimal, ROUND_FLOOR
from pathlib import Path
import numpy as np
from .contract import (SDF_SHA, GENERATOR_SHA, SHAPE_RAW, SHAPE_MAPPER,
                       array_sha, atomic_json, file_sha)

def to_raw_cell(x, y):
    def index(v):
        return int(((Decimal(str(v)) + Decimal('13.15')) / Decimal('0.10'))
                   .to_integral_value(rounding=ROUND_FLOOR))
    return index(y), index(x)

def prepare(repo, output):
    repo, output = Path(repo), Path(output)
    output.mkdir(parents=True, exist_ok=False)
    sdf = repo / 'mapex_lab/map/new_room.sdf'
    script = repo / 'mapex_lab/scripts/generate_new_room_ground_truth.py'
    assert file_sha(sdf) == SDF_SHA
    assert file_sha(script) == GENERATOR_SHA
    spec = importlib.util.spec_from_file_location('mx072_pinned_gt', str(script))
    g = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(g)
    boxes = g._boxes_world_to_slam(g._load_collision_boxes(sdf, g.STRUCTURE_MODEL, 0.20), 0, 3, 0)
    occupied2 = g._rasterize_boxes(boxes)
    domain2, bounds = g._building_mask(boxes)
    occupied2 &= domain2
    data2 = np.full(domain2.shape, -1, dtype=np.int16)
    data2[domain2] = 0
    data2[occupied2] = 100
    roi2, seed = g._connected_free(domain2 & ~occupied2, 0, 0)
    frozen = repo / 'mapex_lab/ground_truth/new_room/generated'
    np.testing.assert_array_equal(data2, np.load(str(frozen / 'new_room_structural_gt_v2.npz'))['data'])
    np.testing.assert_array_equal(roi2, np.load(str(frozen / 'new_room_connected_free_v2.npy')))
    counts = [len(boxes), int(domain2.sum()), int(occupied2.sum()),
              int((domain2 & ~occupied2).sum()), int(roi2.sum())]
    assert counts == [66, 211696, 23990, 187706, 187690]
    assert tuple(seed) == (1202, 512)
    np.testing.assert_allclose(bounds, [-13.09, 13.09, -13.09, 7.09], atol=1e-12, rtol=0)
    rr, cc = SHAPE_RAW
    yy, xx = np.meshgrid(-13.10+np.arange(rr)*0.10,
                         -13.10+np.arange(cc)*0.10, indexing='ij')
    occupied1 = np.zeros(SHAPE_RAW, bool)
    for b in boxes:
        dx, dy = xx-b['x'], yy-b['y']
        c, s = math.cos(b['yaw']), math.sin(b['yaw'])
        occupied1 |= ((np.abs(c*dx+s*dy) <= b['sx']/2+1e-10) &
                      (np.abs(-s*dx+c*dy) <= b['sy']/2+1e-10))
    raw = occupied1.astype(np.float32)
    mapper_gt = np.pad(raw, 500, mode='constant', constant_values=0)
    assert mapper_gt.shape == SHAPE_MAPPER and raw[131, 131] == 0
    support = domain2[939:1345, 249:775].reshape(rr, 2, cc, 2).all(axis=(1, 3))
    p2 = occupied2[939:1345, 249:775].reshape(rr, 2, cc, 2).any(axis=(1, 3)).astype(np.float32)
    assert int(support.sum()) == 52461
    mismatch = support & (raw != p2)
    assert int(mismatch.sum()) == 2994
    r2, c2 = np.nonzero(roi2)
    rp, cp = (r2-939)//2, (c2-249)//2
    assert ((rp >= 0) & (rp < rr) & (cp >= 0) & (cp < cc)).all()
    roi_weights = np.bincount(rp*cc+cp, minlength=rr*cc).reshape(SHAPE_RAW).astype(np.int32)
    assert int(roi_weights.sum()) == 187690
    path = output / 'geometry.npz'
    arrays = {'sensor_raw': raw, 'mapper_gt': mapper_gt, 'support': support,
              'p2_anyocc': p2, 'roi_weights': roi_weights, 'raster_mismatch': mismatch,
              'p2_data': data2, 'p2_roi': roi2}
    np.savez_compressed(str(path), **arrays)
    manifest = {'contract': 'MX072_R4_NEW_ROOM_CENTER_CELL_V1',
                'source_sdf_sha256': file_sha(sdf), 'generator_sha256': file_sha(script),
                'counts_boxes_evaluation_occupied_free_roi': counts,
                'p2_seed': list(seed), 'bounds_slam': list(bounds),
                'full_support_cells': 52461, 'raster_mismatch_cells': 2994,
                'roi_area_m2': 469.225, 'geometry_file_sha256': file_sha(path),
                'array_sha256': {key: array_sha(a) for key, a in arrays.items()},
                'frozen_files_sha256': {name: file_sha(frozen / name) for name in
                   ['new_room_structural_gt_v2.npz', 'new_room_connected_free_v2.npy']}}
    atomic_json(output / 'geometry_manifest.json', manifest)
    return manifest

def load(path):
    import json
    path = Path(path)
    manifest = json.loads((path / 'geometry_manifest.json').read_text())
    assert file_sha(path / 'geometry.npz') == manifest['geometry_file_sha256']
    with np.load(str(path / 'geometry.npz'), allow_pickle=False) as f:
        arrays = {key: f[key] for key in f.files}
    for key, value in arrays.items():
        assert array_sha(value) == manifest['array_sha256'][key]
        value.flags.writeable = False
    return arrays, manifest

def coverage(obs_map, geo):
    known = obs_map[500:703, 500:763] != 0.5
    return float(np.sum(geo['roi_weights'][known])) / 187690

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--repo', required=True)
    p.add_argument('--output', required=True)
    a = p.parse_args()
    print(prepare(a.repo, a.output))
