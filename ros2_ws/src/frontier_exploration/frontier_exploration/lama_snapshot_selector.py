"""Select representative LaMa snapshots across one exploration run.

The preprocessor writes one record per frame to ``lama_dataset_manifest.jsonl``.
This module chooses a fixed-size chronological subset that spans the observed
``known_fraction`` range, while always retaining the first and final frame.
Selected input/mask PNGs are copied into a compact evaluation directory so LaMa
inference can run on the subset directly.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path


def load_dataset_manifest(run_dir: Path) -> list[dict]:
    """Load and validate the preprocessed LaMa manifest in chronological order."""
    run_dir = run_dir.expanduser().resolve()
    manifest_path = run_dir / 'lama_dataset_manifest.jsonl'
    if not manifest_path.is_file():
        raise FileNotFoundError(
            f'Missing {manifest_path}; run prepare_lama_dataset first'
        )

    records: list[dict] = []
    with manifest_path.open('r', encoding='utf-8') as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            record = json.loads(line)
            frame = str(record.get('frame', ''))
            if not frame.isdigit():
                raise ValueError(
                    f'Invalid frame at {manifest_path}:{line_number}: {frame!r}'
                )
            known_fraction = float(record.get('known_fraction', math.nan))
            if not math.isfinite(known_fraction):
                raise ValueError(
                    'Invalid known_fraction at '
                    f'{manifest_path}:{line_number}: {known_fraction!r}'
                )
            input_rel = record.get('input')
            mask_rel = record.get('mask')
            if not isinstance(input_rel, str) or not isinstance(mask_rel, str):
                raise ValueError(
                    f'Missing input/mask path at {manifest_path}:{line_number}'
                )
            records.append(
                {
                    **record,
                    'frame': frame,
                    'known_fraction': known_fraction,
                }
            )

    if not records:
        raise ValueError(f'No records found in {manifest_path}')

    records.sort(key=lambda record: int(record['frame']))
    return records


def select_records_by_known_fraction(
    records: list[dict],
    count: int = 20,
) -> list[dict]:
    """Choose ``count`` unique chronological frames spanning known_fraction.

    The first and final frame are always kept. Intermediate targets are evenly
    spaced between their known fractions. For each target we choose the nearest
    available future frame while reserving enough later frames to complete the
    requested subset. This preserves chronology even if SLAM known_fraction has
    small non-monotonic fluctuations.
    """
    if count < 2:
        raise ValueError('count must be at least 2 to span start and end frames')
    if len(records) < count:
        raise ValueError(
            f'Cannot select {count} snapshots from only {len(records)} frames'
        )

    start_fraction = float(records[0]['known_fraction'])
    end_fraction = float(records[-1]['known_fraction'])
    targets = [
        start_fraction
        + (end_fraction - start_fraction) * slot / (count - 1)
        for slot in range(count)
    ]

    selected_indices = [0]
    previous_index = 0
    total = len(records)

    for slot in range(1, count - 1):
        min_index = previous_index + 1
        # Leave exactly enough later indices for all remaining slots, including
        # the final frame which is fixed to records[-1].
        max_index = total - (count - slot)
        target = targets[slot]
        best_index = min(
            range(min_index, max_index + 1),
            key=lambda index: (
                abs(float(records[index]['known_fraction']) - target),
                index,
            ),
        )
        selected_indices.append(best_index)
        previous_index = best_index

    selected_indices.append(total - 1)

    selected: list[dict] = []
    for slot, (index, target) in enumerate(zip(selected_indices, targets), start=1):
        record = dict(records[index])
        actual = float(record['known_fraction'])
        record['selection_slot'] = slot
        record['target_known_fraction'] = target
        record['known_fraction_error'] = actual - target
        selected.append(record)
    return selected


def _clean_pngs(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for path in directory.glob('*.png'):
        path.unlink()


def materialize_selection(
    run_dir: Path,
    selected: list[dict],
    output_subdir: str = 'lama_eval_20',
) -> Path:
    """Copy selected model input/mask images and write selection metadata."""
    run_dir = run_dir.expanduser().resolve()
    output_dir = run_dir / output_subdir
    input_dir = output_dir / 'model_input'
    mask_dir = output_dir / 'model_mask'
    _clean_pngs(input_dir)
    _clean_pngs(mask_dir)

    manifest_records: list[dict] = []
    for record in selected:
        source_input = run_dir / str(record['input'])
        source_mask = run_dir / str(record['mask'])
        if not source_input.is_file():
            raise FileNotFoundError(f'Missing preprocessed input: {source_input}')
        if not source_mask.is_file():
            raise FileNotFoundError(f'Missing preprocessed mask: {source_mask}')

        frame = str(record['frame'])
        destination_input = input_dir / f'{frame}.png'
        destination_mask = mask_dir / f'{frame}.png'
        shutil.copy2(source_input, destination_input)
        shutil.copy2(source_mask, destination_mask)

        manifest_records.append(
            {
                **record,
                'selected_input': str(destination_input.relative_to(output_dir)),
                'selected_mask': str(destination_mask.relative_to(output_dir)),
            }
        )

    manifest_path = output_dir / 'selection_manifest.jsonl'
    with manifest_path.open('w', encoding='utf-8') as stream:
        for record in manifest_records:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')

    summary = {
        'method': 'chronological nearest-to-even-known-fraction targets',
        'source_manifest': 'lama_dataset_manifest.jsonl',
        'count': len(manifest_records),
        'first_frame': manifest_records[0]['frame'],
        'last_frame': manifest_records[-1]['frame'],
        'first_known_fraction': manifest_records[0]['known_fraction'],
        'last_known_fraction': manifest_records[-1]['known_fraction'],
        'input_dir': 'model_input',
        'mask_dir': 'model_mask',
        'manifest': manifest_path.name,
        'frames': [record['frame'] for record in manifest_records],
    }
    with (output_dir / 'selection.json').open('w', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write('\n')

    return output_dir


def select_snapshots(
    run_dir: Path,
    count: int = 20,
    output_subdir: str | None = None,
) -> Path:
    """Load, select, and materialize a representative LaMa evaluation subset."""
    records = load_dataset_manifest(run_dir)
    selected = select_records_by_known_fraction(records, count=count)
    if output_subdir is None:
        output_subdir = f'lama_eval_{count}'
    return materialize_selection(run_dir, selected, output_subdir=output_subdir)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Select a chronological LaMa evaluation subset spanning the observed '
            'known_fraction range.'
        )
    )
    parser.add_argument(
        'run_dir',
        type=Path,
        help='Preprocessed snapshot run containing lama_dataset_manifest.jsonl.',
    )
    parser.add_argument(
        '--count',
        type=int,
        default=20,
        help='Number of snapshots to select (default: 20).',
    )
    parser.add_argument(
        '--output-subdir',
        default=None,
        help='Output folder inside the run (default: lama_eval_<count>).',
    )
    parsed = parser.parse_args(args)

    output_dir = select_snapshots(
        parsed.run_dir,
        count=parsed.count,
        output_subdir=parsed.output_subdir,
    )
    summary = json.loads((output_dir / 'selection.json').read_text(encoding='utf-8'))
    print(f'Selected {summary["count"]} snapshots into {output_dir}')
    for record in load_dataset_manifest(parsed.run_dir):
        if record['frame'] in set(summary['frames']):
            print(
                f'  frame={record["frame"]} '
                f'known_fraction={record["known_fraction"]:.4f}'
            )


if __name__ == '__main__':
    main()
