"""Run upstream LaMa inference on a prepared snapshot evaluation subset.

The dataset preprocessor stores images and masks in separate directories. Upstream
LaMa expects each image and its mask in one directory with names like
``image.png`` and ``image_mask001.png``. This module stages that convention,
invokes the pinned/local LaMa ``bin/predict.py`` script once for the batch, and
normalizes outputs to ``prediction/<frame>.png``.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from frontier_exploration.project_paths import find_project_root


@dataclass(frozen=True)
class FramePair:
    frame: str
    input_path: Path
    mask_path: Path


def discover_frame_pairs(eval_dir: Path) -> list[FramePair]:
    """Return chronological input/mask pairs from one prepared LaMa eval dir."""
    eval_dir = eval_dir.expanduser().resolve()
    input_dir = eval_dir / 'model_input'
    mask_dir = eval_dir / 'model_mask'
    if not input_dir.is_dir():
        raise FileNotFoundError(f'Missing LaMa input directory: {input_dir}')
    if not mask_dir.is_dir():
        raise FileNotFoundError(f'Missing LaMa mask directory: {mask_dir}')

    input_paths = sorted(input_dir.glob('*.png'))
    if not input_paths:
        raise FileNotFoundError(f'No PNG inputs found in {input_dir}')

    pairs: list[FramePair] = []
    for input_path in input_paths:
        frame = input_path.stem
        if not frame.isdigit():
            raise ValueError(f'Expected numeric frame PNG, got: {input_path.name}')
        mask_path = mask_dir / input_path.name
        if not mask_path.is_file():
            raise FileNotFoundError(
                f'Missing mask for frame {frame}: expected {mask_path}'
            )
        pairs.append(FramePair(frame, input_path, mask_path))

    extra_masks = {
        path.stem for path in mask_dir.glob('*.png')
    } - {pair.frame for pair in pairs}
    if extra_masks:
        raise ValueError(
            'Mask directory contains frames without inputs: '
            + ', '.join(sorted(extra_masks))
        )

    pairs.sort(key=lambda pair: int(pair.frame))
    return pairs


def _clean_pngs(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for path in directory.glob('*.png'):
        path.unlink()


def stage_lama_inputs(pairs: list[FramePair], staging_dir: Path) -> Path:
    """Create upstream-LaMa image/mask filename pairs in one directory."""
    if not pairs:
        raise ValueError('At least one frame pair is required')
    staging_dir = staging_dir.expanduser().resolve()
    _clean_pngs(staging_dir)

    for pair in pairs:
        shutil.copy2(pair.input_path, staging_dir / f'{pair.frame}.png')
        shutil.copy2(pair.mask_path, staging_dir / f'{pair.frame}_mask001.png')
    return staging_dir


def resolve_lama_root(value: Path | None, *, start: Path | None = None) -> Path:
    """Resolve the local LaMa runtime workspace containing ``bin/predict.py``."""
    if value is not None:
        root = value.expanduser().resolve()
    else:
        project_root = find_project_root(start=start)
        candidates = [
            project_root / 'third_party' / 'lama',
            Path.home() / 'lama',
            project_root / 'third_party' / 'lama_upstream',
        ]
        root = next(
            (
                candidate
                for candidate in candidates
                if (candidate / 'bin' / 'predict.py').is_file()
            ),
            candidates[0],
        )

    predict_script = root / 'bin' / 'predict.py'
    if not predict_script.is_file():
        raise FileNotFoundError(
            'Cannot find upstream LaMa bin/predict.py. '
            f'Checked LaMa root: {root}. '
            'Use --lama-root to point at the working LaMa checkout.'
        )
    return root


def resolve_model_path(lama_root: Path, value: Path | None) -> Path:
    """Resolve and validate a LaMa checkpoint directory."""
    if value is None:
        candidates = [lama_root / 'big-lama', lama_root / 'big_lama']
        model_path = next((path for path in candidates if path.is_dir()), candidates[0])
    else:
        expanded = value.expanduser()
        model_path = expanded if expanded.is_absolute() else lama_root / expanded
        model_path = model_path.resolve()

    if not model_path.is_dir():
        raise FileNotFoundError(
            f'LaMa model directory not found: {model_path}. '
            'Use --model-path to point at the downloaded big-lama checkpoint.'
        )
    if not (model_path / 'config.yaml').is_file():
        raise FileNotFoundError(f'Missing LaMa model config: {model_path / "config.yaml"}')
    if not (model_path / 'models').is_dir():
        raise FileNotFoundError(f'Missing LaMa model weights directory: {model_path / "models"}')
    return model_path


def build_predict_command(
    python_executable: str,
    lama_root: Path,
    model_path: Path,
    staging_dir: Path,
    raw_output_dir: Path,
    checkpoint: str | None = None,
) -> list[str]:
    """Build the upstream ``bin/predict.py`` command as argv tokens."""
    command = [
        python_executable,
        str(lama_root / 'bin' / 'predict.py'),
        f'model.path={model_path}',
        f'indir={staging_dir}',
        f'outdir={raw_output_dir}',
    ]
    if checkpoint:
        command.append(f'model.checkpoint={checkpoint}')
    return command


def collect_predictions(
    pairs: list[FramePair],
    raw_output_dir: Path,
    prediction_dir: Path,
) -> list[Path]:
    """Normalize upstream mask-based output names to ``<frame>.png``."""
    _clean_pngs(prediction_dir)
    outputs: list[Path] = []
    for pair in pairs:
        candidates = [
            raw_output_dir / f'{pair.frame}_mask001.png',
            raw_output_dir / f'{pair.frame}.png',
        ]
        source = next((path for path in candidates if path.is_file()), None)
        if source is None:
            raise FileNotFoundError(
                f'LaMa produced no prediction for frame {pair.frame}; '
                f'expected one of: {", ".join(str(path) for path in candidates)}'
            )
        destination = prediction_dir / f'{pair.frame}.png'
        shutil.copy2(source, destination)
        outputs.append(destination)
    return outputs


def run_lama_inference(
    eval_dir: Path,
    *,
    lama_root: Path | None = None,
    model_path: Path | None = None,
    python_executable: str = sys.executable,
    checkpoint: str | None = None,
    keep_workdir: bool = False,
    dry_run: bool = False,
) -> dict:
    """Stage a prepared subset, invoke upstream LaMa, and save predictions."""
    eval_dir = eval_dir.expanduser().resolve()
    pairs = discover_frame_pairs(eval_dir)
    resolved_lama_root = resolve_lama_root(lama_root, start=eval_dir)
    resolved_model_path = resolve_model_path(resolved_lama_root, model_path)

    staging_dir = eval_dir / '.lama_staging'
    raw_output_dir = eval_dir / '.lama_raw_output'
    prediction_dir = eval_dir / 'prediction'
    stage_lama_inputs(pairs, staging_dir)
    _clean_pngs(raw_output_dir)

    command = build_predict_command(
        python_executable,
        resolved_lama_root,
        resolved_model_path,
        staging_dir,
        raw_output_dir,
        checkpoint,
    )

    if dry_run:
        return {
            'dry_run': True,
            'frames': len(pairs),
            'command': command,
            'lama_root': str(resolved_lama_root),
            'model_path': str(resolved_model_path),
            'staging_dir': str(staging_dir),
            'prediction_dir': str(prediction_dir),
        }

    env = os.environ.copy()
    existing_pythonpath = env.get('PYTHONPATH', '')
    env['PYTHONPATH'] = (
        str(resolved_lama_root)
        if not existing_pythonpath
        else str(resolved_lama_root) + os.pathsep + existing_pythonpath
    )
    env.setdefault('TORCH_HOME', str(resolved_lama_root))

    started = time.perf_counter()
    try:
        subprocess.run(
            command,
            cwd=resolved_lama_root,
            env=env,
            check=True,
        )
        outputs = collect_predictions(pairs, raw_output_dir, prediction_dir)
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(
            f'LaMa inference failed with exit code {exc.returncode}. '
            'The upstream error above usually identifies a missing Python '
            'dependency, incompatible environment, or checkpoint problem.'
        ) from exc
    finally:
        if not keep_workdir:
            shutil.rmtree(staging_dir, ignore_errors=True)
            shutil.rmtree(raw_output_dir, ignore_errors=True)

    elapsed = time.perf_counter() - started
    summary = {
        'dry_run': False,
        'frames': len(outputs),
        'elapsed_sec': elapsed,
        'mean_sec_per_frame': elapsed / len(outputs),
        'lama_root': str(resolved_lama_root),
        'model_path': str(resolved_model_path),
        'checkpoint_override': checkpoint,
        'python_executable': python_executable,
        'prediction_dir': str(prediction_dir),
        'frame_ids': [pair.frame for pair in pairs],
    }
    with (eval_dir / 'inference.json').open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    return summary


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description='Run upstream big-LaMa inference on a prepared snapshot subset.'
    )
    parser.add_argument(
        'eval_dir',
        type=Path,
        help='Prepared directory containing model_input/ and model_mask/.',
    )
    parser.add_argument(
        '--lama-root',
        type=Path,
        default=None,
        help=(
            'LaMa checkout/runtime root (auto-detect project third_party/lama, '
            '~/lama, then pinned upstream checkout).'
        ),
    )
    parser.add_argument(
        '--model-path',
        type=Path,
        default=None,
        help='Checkpoint directory (default: <lama-root>/big-lama).',
    )
    parser.add_argument(
        '--python',
        dest='python_executable',
        default=sys.executable,
        help='Python executable used for upstream LaMa (default: current Python).',
    )
    parser.add_argument(
        '--checkpoint',
        default=None,
        help='Optional model.checkpoint override, e.g. best.ckpt.',
    )
    parser.add_argument(
        '--keep-workdir',
        action='store_true',
        help='Keep .lama_staging and .lama_raw_output for debugging.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Validate paths and stage inputs without running the neural network.',
    )
    parsed = parser.parse_args(args)

    summary = run_lama_inference(
        parsed.eval_dir,
        lama_root=parsed.lama_root,
        model_path=parsed.model_path,
        python_executable=parsed.python_executable,
        checkpoint=parsed.checkpoint,
        keep_workdir=parsed.keep_workdir,
        dry_run=parsed.dry_run,
    )

    if summary['dry_run']:
        print(f'Validated {summary["frames"]} LaMa frame(s).')
        print('Command:')
        print('  ' + ' '.join(summary['command']))
        return

    print(f'LaMa inference complete: {summary["frames"]} frame(s).')
    print(f'Elapsed: {summary["elapsed_sec"]:.2f} s')
    print(f'Mean: {summary["mean_sec_per_frame"]:.2f} s/frame')
    print(f'Predictions: {summary["prediction_dir"]}')


if __name__ == '__main__':
    main()
