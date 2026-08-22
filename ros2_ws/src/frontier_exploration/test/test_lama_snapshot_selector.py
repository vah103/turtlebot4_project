import json
from pathlib import Path

import pytest

from frontier_exploration.lama_snapshot_selector import (
    load_raw_snapshot_records,
    materialize_raw_selection,
    select_records_by_known_fraction,
)


def _records(count: int) -> list[dict]:
    denominator = max(count - 1, 1)
    return [
        {
            'frame': f'{index:06d}',
            'raw': f'{index:06d}_occupancy.bin',
            'metadata': f'{index:06d}_metadata.json',
            'pgm': f'{index:06d}_map.pgm',
            'known_fraction': index / denominator,
        }
        for index in range(count)
    ]


def _write_raw_run(run_dir: Path, known_cells: list[int], total_cells: int = 100) -> None:
    run_dir.mkdir(parents=True)
    (run_dir / 'run.json').write_text(
        json.dumps(
            {
                'fixed_canvas': {
                    'width': total_cells,
                    'height': 1,
                    'resolution': 0.05,
                    'origin': {'x': 0.0, 'y': 0.0},
                }
            }
        ),
        encoding='utf-8',
    )

    for index, count in enumerate(known_cells):
        frame = f'{index:06d}'
        (run_dir / f'{frame}_occupancy.bin').write_bytes(bytes([255]) * total_cells)
        (run_dir / f'{frame}_map.pgm').write_bytes(b'pgm')
        (run_dir / f'{frame}_metadata.json').write_text(
            json.dumps(
                {
                    'sequence': index,
                    'projection': {'known_cells_copied': count},
                }
            ),
            encoding='utf-8',
        )


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


def test_load_raw_records_uses_metadata_without_preprocessed_pngs(
    tmp_path: Path,
) -> None:
    run_dir = tmp_path / 'run'
    _write_raw_run(run_dir, [0, 25, 50, 100])

    records = load_raw_snapshot_records(run_dir)

    assert [record['frame'] for record in records] == [
        '000000',
        '000001',
        '000002',
        '000003',
    ]
    assert [record['known_fraction'] for record in records] == [
        0.0,
        0.25,
        0.5,
        1.0,
    ]
    assert not (run_dir / 'model_input').exists()


def test_materialize_raw_selection_is_preprocess_ready(tmp_path: Path) -> None:
    run_dir = tmp_path / 'run'
    _write_raw_run(run_dir, [0, 20, 40, 60, 80, 100])

    records = load_raw_snapshot_records(run_dir)
    selected = select_records_by_known_fraction(records, count=3)
    output_dir = materialize_raw_selection(run_dir, selected, 'lama_eval_3')

    summary = json.loads((output_dir / 'selection.json').read_text())
    assert summary['count'] == 3
    assert summary['frames'][0] == '000000'
    assert summary['frames'][-1] == '000005'
    assert (output_dir / 'run.json').is_file()
    assert len(list(output_dir.glob('*_occupancy.bin'))) == 3
    assert len(list(output_dir.glob('*_metadata.json'))) == 3
    assert len(list(output_dir.glob('*_map.pgm'))) == 3
    assert len((output_dir / 'selection_manifest.jsonl').read_text().splitlines()) == 3
    assert not (output_dir / 'model_input').exists()
