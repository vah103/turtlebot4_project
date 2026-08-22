"""Select representative raw snapshots before LaMa preprocessing.

Each recorded snapshot already has lightweight JSON metadata containing the
number of known fixed-canvas cells. This module uses those metadata files to
choose a fixed-size chronological subset spanning the observed
``known_fraction`` range, while always retaining the first and final frame.

Only the selected raw occupancy snapshots are copied into a compact evaluation
directory. ``prepare_lama_dataset`` can then preprocess that directory, so a
20-frame evaluation does not waste time generating PNGs for every frame in the
full exploration run.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path


def _load_canvas_cell_count(run_dir: Path) -> int:
    run_path = run_dir / 'run.json'
    if not run_path.is_file():
        raise FileNotFoundError(f'Missing recorder metadata: {run_path}')

    payload = json.loads(run_path.read_text(encoding='utf-8'))
    fixed = payload.get('fixed_canvas')
    if not isinstance(fixed, dict):
        raise ValueError(f'run.json has no fixed_canvas object: {run_path}')

    width = int(fixed['width'])
    height = int(fixed['height'])
    if width <= 0 or height <= 0:
        raise ValueError('fixed_canvas width/height must be positive')
    return width * height


def load_raw_snapshot_records(run_dir: Path) -> list[dict]:
    """Load chronological raw snapshot records using lightweight metadata."""
    run_dir = run_dir.expanduser().resolve()
    total_cells = _load_canvas_cell_count(run_dir)

    metadata_paths = sorted(run_dir.glob('*_metadata.json'))
    if not metadata_paths:
        raise FileNotFoundError(f'No *_metadata.json snapshots found in {run_dir}')

    records: list[dict] = []
    for metadata_path in metadata_paths:
        frame = metadata_path.name.removesuffix('_metadata.json')
        if not frame.isdigit():
            continue

        raw_path = run_dir / f'{frame}_occupancy.bin'
        if not raw_path.is_file():
            raise FileNotFoundError(f'Missing raw occupancy snapshot: {raw_path}')

        payload = json.loads(metadata_path.read_text(encoding='utf-8'))
        projection = payload.get('projection')
        if not isinstance(projection, dict):
            raise ValueError(f'Missing projection object in {metadata_path}')

        known_cells = projection.get('known_cells_copied')
        if known_cells is None:
            raise ValueError(
                f'Missing projection.known_cells_copied in {metadata_path}'
            )
        known_cells = int(known_cells)
        if known_cells < 0 or known_cells > total_cells:
            raise ValueError(
                f'Invalid known cell count in {metadata_path}: {known_cells}'
            )

        pgm_path = run_dir / f'{frame}_map.pgm'
        records.append(
            {
                'frame': frame,
                'raw': raw_path.name,
                'metadata': metadata_path.name,
                'pgm': pgm_path.name if pgm_path.is_file() else None,
                'known_cells': known_cells,
                'total_cells': total_cells,
                'known_fraction': known_cells / total_cells,
            }
        )

    if not records:
        raise ValueError(f'No valid numeric snapshot metadata found in {run_dir}')

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


def materialize_raw_selection(
    run_dir: Path,
    selected: list[dict],
    output_subdir: str = 'lama_eval_20',
) -> Path:
    """Copy only selected raw snapshots into a preprocess-ready run directory."""
    run_dir = run_dir.expanduser().resolve()
    output_dir = run_dir / output_subdir

    if output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True)

    run_json = run_dir / 'run.json'
    shutil.copy2(run_json, output_dir / 'run.json')

    manifest_records: list[dict] = []
    for record in selected:
        frame = str(record['frame'])
        source_raw = run_dir / str(record['raw'])
        source_metadata = run_dir / str(record['metadata'])
        destination_raw = output_dir / source_raw.name
        destination_metadata = output_dir / source_metadata.name

        shutil.copy2(source_raw, destination_raw)
        shutil.copy2(source_metadata, destination_metadata)

        source_pgm = None
        if record.get('pgm'):
            candidate = run_dir / str(record['pgm'])
            if candidate.is_file():
                source_pgm = candidate
                shutil.copy2(candidate, output_dir / candidate.name)

        manifest_records.append(
            {
                **record,
                'selected_raw': destination_raw.name,
                'selected_metadata': destination_metadata.name,
                'selected_pgm': source_pgm.name if source_pgm else None,
            }
        )

    manifest_path = output_dir / 'selection_manifest.jsonl'
    with manifest_path.open('w', encoding='utf-8') as stream:
        for record in manifest_records:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')

    summary = {
        'method': 'raw metadata selection before LaMa preprocessing',
        'source_run': str(run_dir),
        'count': len(manifest_records),
        'first_frame': manifest_records[0]['frame'],
        'last_frame': manifest_records[-1]['frame'],
        'first_known_fraction': manifest_records[0]['known_fraction'],
        'last_known_fraction': manifest_records[-1]['known_fraction'],
        'manifest': manifest_path.name,
        'frames': [record['frame'] for record in manifest_records],
        'next_step': (
            'run prepare_lama_dataset on this directory; only selected raw '
            'snapshots will be preprocessed'
        ),
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
    """Select raw snapshots first and materialize a compact evaluation run."""
    records = load_raw_snapshot_records(run_dir)
    selected = select_records_by_known_fraction(records, count=count)
    if output_subdir is None:
        output_subdir = f'lama_eval_{count}'
    return materialize_raw_selection(
        run_dir,
        selected,
        output_subdir=output_subdir,
    )


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Select raw snapshots by known_fraction before LaMa preprocessing.'
        )
    )
    parser.add_argument(
        'run_dir',
        type=Path,
        help='Raw snapshot run containing run.json and *_metadata.json files.',
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
    selected_frames = set(summary['frames'])
    print(f'Selected {summary["count"]} raw snapshots into {output_dir}')
    for record in load_raw_snapshot_records(parsed.run_dir):
        if record['frame'] in selected_frames:
            print(
                f'  frame={record["frame"]} '
                f'known_fraction={record["known_fraction"]:.4f}'
            )
    print('Next: run prepare_lama_dataset on the selected output directory.')


if __name__ == '__main__':
    main()
