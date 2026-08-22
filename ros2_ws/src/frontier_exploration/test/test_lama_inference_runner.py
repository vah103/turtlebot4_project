from pathlib import Path

import pytest

from frontier_exploration.lama_inference_runner import (
    build_predict_command,
    collect_predictions,
    discover_frame_pairs,
    stage_lama_inputs,
)


def _prepared_eval(tmp_path: Path, frames: tuple[str, ...] = ('000001', '000010')) -> Path:
    eval_dir = tmp_path / 'lama_eval_20'
    input_dir = eval_dir / 'model_input'
    mask_dir = eval_dir / 'model_mask'
    input_dir.mkdir(parents=True)
    mask_dir.mkdir(parents=True)
    for frame in frames:
        (input_dir / f'{frame}.png').write_bytes(b'input-' + frame.encode())
        (mask_dir / f'{frame}.png').write_bytes(b'mask-' + frame.encode())
    return eval_dir


def test_discover_frame_pairs_is_numeric_and_chronological(tmp_path: Path) -> None:
    eval_dir = _prepared_eval(tmp_path, ('000010', '000002', '000001'))

    pairs = discover_frame_pairs(eval_dir)

    assert [pair.frame for pair in pairs] == ['000001', '000002', '000010']


def test_discover_frame_pairs_requires_matching_mask(tmp_path: Path) -> None:
    eval_dir = _prepared_eval(tmp_path, ('000001',))
    (eval_dir / 'model_mask' / '000001.png').unlink()

    with pytest.raises(FileNotFoundError, match='Missing mask for frame 000001'):
        discover_frame_pairs(eval_dir)


def test_stage_lama_inputs_uses_upstream_mask_naming(tmp_path: Path) -> None:
    eval_dir = _prepared_eval(tmp_path)
    pairs = discover_frame_pairs(eval_dir)
    staging_dir = stage_lama_inputs(pairs, tmp_path / 'staging')

    assert sorted(path.name for path in staging_dir.glob('*.png')) == [
        '000001.png',
        '000001_mask001.png',
        '000010.png',
        '000010_mask001.png',
    ]
    assert (staging_dir / '000001.png').read_bytes() == b'input-000001'
    assert (staging_dir / '000001_mask001.png').read_bytes() == b'mask-000001'


def test_build_predict_command_matches_upstream_hydra_arguments(tmp_path: Path) -> None:
    lama_root = tmp_path / 'lama'
    command = build_predict_command(
        'python3',
        lama_root,
        lama_root / 'big-lama',
        tmp_path / 'staging',
        tmp_path / 'raw-output',
        'best.ckpt',
    )

    assert command == [
        'python3',
        str(lama_root / 'bin' / 'predict.py'),
        f'model.path={lama_root / "big-lama"}',
        f'indir={tmp_path / "staging"}',
        f'outdir={tmp_path / "raw-output"}',
        'model.checkpoint=best.ckpt',
    ]


def test_collect_predictions_normalizes_mask_output_names(tmp_path: Path) -> None:
    eval_dir = _prepared_eval(tmp_path)
    pairs = discover_frame_pairs(eval_dir)
    raw_output = tmp_path / 'raw-output'
    raw_output.mkdir()
    for pair in pairs:
        (raw_output / f'{pair.frame}_mask001.png').write_bytes(
            b'prediction-' + pair.frame.encode()
        )

    prediction_dir = tmp_path / 'prediction'
    outputs = collect_predictions(pairs, raw_output, prediction_dir)

    assert [path.name for path in outputs] == ['000001.png', '000010.png']
    assert (prediction_dir / '000010.png').read_bytes() == b'prediction-000010'
