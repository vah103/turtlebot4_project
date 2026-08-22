import json
from pathlib import Path

import pytest

from frontier_exploration.lama_snapshot_selector import (
    load_dataset_manifest,
    materialize_selection,
    select_records_by_known_fraction,
)


def _records(count: int) -> list[dict]:
    denominator = max(count - 1, 1)
    return [
        {
            'frame': f'{index:06d}',
            'input': f'model_input/{index:06d}.png',
            'mask': f'model_mask/{index:06d}.png',
            'known_fraction': index / denominator,
        }
        for index in range(count)
    ]


def test_select_even_known_fraction_targets_keeps_endpoints() -> None:
    selected = select_records_by_known_fraction(_records(101), count=5)

    assert [record['frame'] for record in selected] == [
        '000000',
        '000025',
        '000050',
        '000075',
        '000100',
    ]
    assert selected[0]['selection_slot'] == 1
    assert selected[-1]['selection_slot'] == 5


def test_select_twenty_is_unique_and_chronological_with_plateaus() -> None:
    records = _records(60)
    # Simulate several stretches where map coverage barely changes.
    for index in range(10, 20):
        records[index]['known_fraction'] = records[10]['known_fraction']
    for index in range(35, 43):
        records[index]['known_fraction'] = records[35]['known_fraction']

    selected = select_records_by_known_fraction(records, count=20)
    indices = [int(record['frame']) for record in selected]

    assert len(selected) == 20
    assert len(set(indices)) == 20
    assert indices == sorted(indices)
    assert indices[0] == 0
    assert indices[-1] == 59


def test_select_rejects_more_snapshots_than_available() -> None:
    with pytest.raises(ValueError, match='Cannot select 20 snapshots'):
        select_records_by_known_fraction(_records(19), count=20)


def test_materialize_selection_copies_input_mask_and_writes_manifest(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / 'run'
    input_dir = run_dir / 'model_input'
    mask_dir = run_dir / 'model_mask'
    input_dir.mkdir(parents=True)
    mask_dir.mkdir(parents=True)

    records = _records(4)
    manifest = run_dir / 'lama_dataset_manifest.jsonl'
    with manifest.open('w', encoding='utf-8') as stream:
        for record in records:
            stream.write(json.dumps(record) + '\n')
        
    for record in records:
        (run_dir / record['input']).write_bytes(b'input-' + record['frame'].encode())
        (run_dir / record['mask']).write_bytes(b'mask-' + record['frame'].encode())

    loaded = load_dataset_manifest(run_dir)
    selected = select_records_by_known_fraction(loaded, count=3)
    output_dir = materialize_selection(run_dir, selected, 'lama_eval_3')

    summary = json.loads((output_dir / 'selection.json').read_text())
    assert summary['count'] == 3
    assert summary['frames'][0] == '000000'
    assert summary['frames'][-1] == '000003'
    assert len(list((output_dir / 'model_input').glob('*.png'))) == 3
    assert len(list((output_dir / 'model_mask').glob('*.png'))) == 3
    assert len((output_dir / 'selection_manifest.jsonl').read_text().splitlines()) == 3
