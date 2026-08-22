import csv
from pathlib import Path

from frontier_exploration.lama_analysis_report import (
    analyze_evaluation,
    choose_qualitative_frames,
    enrich_progress,
    pearson_defined,
)


def _rows() -> list[dict]:
    known = [0.02, 0.05, 0.10, 0.15, 0.20]
    miou = [0.10, 0.25, 0.50, 0.65, None]
    rows = []
    for index, (known_fraction, score) in enumerate(zip(known, miou)):
        rows.append(
            {
                'frame': f'{index:06d}',
                'known_fraction': known_fraction,
                'eval_pixels': 100 if score is not None else 0,
                'miou': score,
                'occupied_iou': score,
                'occupied_f1': score,
            }
        )
    return rows


def test_enrich_progress_uses_final_known_fraction() -> None:
    rows, final_known = enrich_progress(_rows())

    assert final_known == 0.20
    assert rows[0]['progress_relative_to_final'] == 0.10
    assert rows[3]['progress_relative_to_final'] == 0.75
    assert rows[-1]['progress_relative_to_final'] == 1.0


def test_choose_qualitative_frames_spans_progress() -> None:
    rows, _ = enrich_progress(_rows())
    selected = choose_qualitative_frames(rows)

    assert [row['frame'] for row in selected] == [
        '000000',
        '000001',
        '000002',
        '000003',
    ]


def test_pearson_defined_reports_positive_trend() -> None:
    rows, _ = enrich_progress(_rows())
    correlation = pearson_defined(rows, 'progress_relative_to_final', 'miou')

    assert correlation is not None
    assert correlation > 0.95


def test_analyze_evaluation_writes_report_artifacts(tmp_path: Path) -> None:
    eval_dir = tmp_path / 'lama_eval_20'
    eval_dir.mkdir()
    csv_path = eval_dir / 'evaluation.csv'
    fields = [
        'frame',
        'known_fraction',
        'eval_pixels',
        'miou',
        'occupied_iou',
        'occupied_f1',
    ]
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in _rows():
            writer.writerow(row)

    summary = analyze_evaluation(eval_dir)

    assert summary['final_known_fraction'] == 0.20
    assert (eval_dir / 'evaluation_progress.csv').is_file()
    assert (eval_dir / 'evaluation_metrics.svg').is_file()
    assert (eval_dir / 'evaluation_report.html').is_file()
    report = (eval_dir / 'evaluation_report.html').read_text(encoding='utf-8')
    assert 'LaMa Hospital evaluation' in report
    assert 'model_input/000002.png' in report
