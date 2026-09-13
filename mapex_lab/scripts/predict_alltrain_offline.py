#!/usr/bin/env python3
"""Generate MapEx whole-training predictions from a completed run (no ROS).

This script supports both MapEx ``decisions.csv`` logs and Nearest-Frontier
``policy_decisions.csv`` logs. Original decisions/metadata stay intact. An
atomically written manifest is published only after all predictions succeed.
"""
import argparse
import contextlib
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with open(str(path), 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _decision_log(run):
    """Return (path, id_field) for a supported recorded-run decision log."""
    mapex_path = run / 'decisions.csv'
    nf_path = run / 'policy_decisions.csv'
    if mapex_path.is_file():
        return mapex_path, 'decision_id'
    if nf_path.is_file():
        return nf_path, 'policy_decision_id'
    raise FileNotFoundError(
        'Run has neither decisions.csv nor policy_decisions.csv: {}'.format(run)
    )


def load_observed(path):
    with np.load(str(path), allow_pickle=False) as bundle:
        raw = np.asarray(bundle['data'])
        resolution = float(bundle['resolution'])
        if raw.ndim != 2 or not math.isclose(resolution, .1, abs_tol=1e-6):
            raise ValueError('Expected a 2-D raw OccupancyGrid at 0.10 m/cell')
        if 'origin_yaw' in bundle and abs(float(bundle['origin_yaw'])) > 1e-6:
            raise ValueError('Rotated grids are not supported by this evaluator')
        geometry = dict(resolution=resolution, source_height=raw.shape[0],
                        source_width=raw.shape[1], origin_x=float(bundle['origin_x']),
                        origin_y=float(bundle['origin_y']))
    observed = np.ones(raw.shape, dtype=np.float32)
    observed[raw == 0] = 0.
    observed[raw < 0] = .5
    return observed, geometry


def generate(run, checkpoint, predict):
    """predict(observed) returns the padded all-training occupancy prediction."""
    run = Path(run).resolve()
    decisions_path, id_field = _decision_log(run)
    source_hash = sha256(decisions_path)
    with decisions_path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError('No decisions to process')

    try:
        ids = [int(row[id_field]) for row in rows]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('Decision log has invalid {} values'.format(id_field)) from exc
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate decision IDs')

    output = run / 'evaluation' / 'alltrain'
    # Never overwrite existing evaluated predictions or an interrupted attempt.
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for row in rows:
        decision_id = int(row[id_field])
        raw_rel = row.get('raw_map', '').strip()
        if not raw_rel:
            raise ValueError('Decision {} lacks raw_map'.format(decision_id))
        raw_path = run / raw_rel
        observed, geometry = load_observed(raw_path)
        prediction = np.asarray(predict(observed), dtype=np.float32)
        h, w = observed.shape
        if (prediction.ndim != 2 or prediction.shape[0] < h
                or prediction.shape[1] < w or not np.isfinite(prediction).all()):
            raise ValueError('Invalid prediction shape or non-finite values')
        name = 'decision_{:06d}_alltrain.npz'.format(decision_id)
        target = output / name
        np.savez_compressed(str(target), data=np.clip(prediction, 0., 1.),
                            member='alltrain', pad_top=(prediction.shape[0]-h)//2,
                            pad_left=(prediction.shape[1]-w)//2, **geometry)
        records.append(dict(decision_id=decision_id,
                            alltrain_map=str(target.relative_to(run)),
                            raw_map=raw_rel, raw_sha256=sha256(raw_path)))
    if sha256(decisions_path) != source_hash:
        raise RuntimeError('{} changed during inference; manifest not published'.format(
            decisions_path.name))
    payload = dict(prediction_source='alltrain',
                   decision_log=decisions_path.name,
                   decision_id_field=id_field,
                   decisions_sha256=source_hash,
                   checkpoint=dict(path=str(checkpoint), sha256=sha256(checkpoint)),
                   decisions=records)
    temporary = output / 'manifest.tmp'
    temporary.write_text(json.dumps(payload, indent=2) + '\n')
    temporary.replace(output / 'manifest.json')
    return output / 'manifest.json'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir')
    parser.add_argument('--mapex-root', default=os.environ.get('MAPEX_ROOT', str(Path.home()/'MapEx')))
    parser.add_argument('--checkpoint', default='pretrained_models/weights/big_lama')
    parser.add_argument('--device', default='cuda:0')
    args = parser.parse_args()
    root = Path(args.mapex_root).expanduser().resolve()
    checkpoint = Path(args.checkpoint).expanduser()
    if not checkpoint.is_absolute():
        checkpoint = root / checkpoint
    if checkpoint.is_dir():
        checkpoint = checkpoint / 'models' / 'best.ckpt'
    if not checkpoint.is_file() or checkpoint.parent.name != 'models':
        parser.error('Supply a MapEx all-training checkpoint under <model>/models/')
    model_dir = checkpoint.parent.parent
    if not (model_dir/'config.yaml').is_file():
        parser.error('Missing model config.yaml')
    sys.path.insert(0, str(root/'lama'))
    sys.path.insert(0, str(root/'scripts'))
    with contextlib.redirect_stdout(sys.stderr):
        import torch
        from lama_pred_utils import load_lama_model, get_lama_transform, convert_obsimg_to_model_input
        model = load_lama_model(str(model_dir), checkpoint_name=checkpoint.name, device=args.device)
        model.eval()
        transform = get_lama_transform('default_map_eval', (512, 512))

    def predict(observed):
        with contextlib.redirect_stdout(sys.stderr), torch.no_grad():
            batch, _ = convert_obsimg_to_model_input(
                np.stack([observed]*3, axis=2), transform, args.device)
            return model(batch)['inpainted'][0, 0].detach().float().cpu().numpy()

    print(generate(args.run_dir, checkpoint, predict))


if __name__ == '__main__':
    main()
