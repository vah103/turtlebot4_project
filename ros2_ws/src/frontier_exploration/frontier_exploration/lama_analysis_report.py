"""Create lightweight plots and a qualitative report from LaMa evaluation CSV."""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
from pathlib import Path


METRIC_KEYS = ('miou', 'occupied_iou', 'occupied_f1')
DEFAULT_PROGRESS_TARGETS = (0.15, 0.40, 0.60, 0.85)


def _parse_float(value: str | None) -> float | None:
    if value is None or value == '':
        return None
    return float(value)


def load_evaluation_rows(eval_dir: Path) -> list[dict]:
    path = eval_dir / 'evaluation.csv'
    if not path.is_file():
        raise FileNotFoundError(f'Missing evaluation CSV: {path}')

    rows: list[dict] = []
    with path.open('r', encoding='utf-8', newline='') as stream:
        reader = csv.DictReader(stream)
        for raw in reader:
            row = dict(raw)
            row['known_fraction'] = _parse_float(raw.get('known_fraction'))
            row['eval_pixels'] = int(raw.get('eval_pixels') or 0)
            for key in METRIC_KEYS:
                row[key] = _parse_float(raw.get(key))
            rows.append(row)
    if not rows:
        raise ValueError(f'No evaluation rows in {path}')
    return rows


def enrich_progress(rows: list[dict]) -> tuple[list[dict], float]:
    final_known = next(
        (
            float(row['known_fraction'])
            for row in reversed(rows)
            if row.get('known_fraction') is not None
        ),
        0.0,
    )
    if final_known <= 0.0:
        raise ValueError('Final known_fraction must be positive')

    enriched: list[dict] = []
    for row in rows:
        copy = dict(row)
        known = copy.get('known_fraction')
        copy['progress_relative_to_final'] = (
            None if known is None else float(known) / final_known
        )
        enriched.append(copy)
    return enriched, final_known


def pearson_defined(rows: list[dict], x_key: str, y_key: str) -> float | None:
    pairs = [
        (float(row[x_key]), float(row[y_key]))
        for row in rows
        if row.get(x_key) is not None and row.get(y_key) is not None
    ]
    if len(pairs) < 2:
        return None
    xs = [pair[0] for pair in pairs]
    ys = [pair[1] for pair in pairs]
    x_mean = sum(xs) / len(xs)
    y_mean = sum(ys) / len(ys)
    numerator = sum((x - x_mean) * (y - y_mean) for x, y in pairs)
    x_var = sum((x - x_mean) ** 2 for x in xs)
    y_var = sum((y - y_mean) ** 2 for y in ys)
    denominator = math.sqrt(x_var * y_var)
    return None if denominator == 0.0 else numerator / denominator


def choose_qualitative_frames(
    rows: list[dict],
    targets: tuple[float, ...] = DEFAULT_PROGRESS_TARGETS,
) -> list[dict]:
    candidates = [
        row
        for row in rows
        if row.get('progress_relative_to_final') is not None
        and int(row.get('eval_pixels', 0)) > 0
    ]
    if not candidates:
        return []

    selected: list[dict] = []
    used: set[str] = set()
    for target in targets:
        available = [row for row in candidates if str(row['frame']) not in used]
        if not available:
            break
        best = min(
            available,
            key=lambda row: (
                abs(float(row['progress_relative_to_final']) - target),
                int(row['frame']),
            ),
        )
        selected.append(best)
        used.add(str(best['frame']))
    selected.sort(key=lambda row: int(row['frame']))
    return selected


def write_progress_csv(eval_dir: Path, rows: list[dict]) -> Path:
    output = eval_dir / 'evaluation_progress.csv'
    original_fields = list(rows[0].keys())
    fields = [
        field for field in original_fields if field != 'progress_relative_to_final'
    ]
    insert_at = fields.index('known_fraction') + 1 if 'known_fraction' in fields else 1
    fields.insert(insert_at, 'progress_relative_to_final')

    with output.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            rendered = {}
            for field in fields:
                value = row.get(field)
                if isinstance(value, float):
                    rendered[field] = f'{value:.8f}'
                elif value is None:
                    rendered[field] = ''
                else:
                    rendered[field] = value
            writer.writerow(rendered)
    return output


def _polyline_points(
    rows: list[dict],
    metric: str,
    width: int,
    height: int,
    margin: int,
) -> str:
    points: list[str] = []
    for row in rows:
        x_value = row.get('progress_relative_to_final')
        y_value = row.get(metric)
        if x_value is None or y_value is None:
            continue
        x = margin + float(x_value) * (width - 2 * margin)
        y = height - margin - float(y_value) * (height - 2 * margin)
        points.append(f'{x:.1f},{y:.1f}')
    return ' '.join(points)


def write_metrics_svg(eval_dir: Path, rows: list[dict]) -> Path:
    output = eval_dir / 'evaluation_metrics.svg'
    width, height, margin = 900, 520, 70
    plot_width = width - 2 * margin
    plot_height = height - 2 * margin

    grid = []
    for index in range(6):
        fraction = index / 5
        x = margin + fraction * plot_width
        y = height - margin - fraction * plot_height
        grid.append(
            f'<line x1="{x:.1f}" y1="{margin}" x2="{x:.1f}" y2="{height-margin}" '
            'stroke="#ddd" stroke-width="1"/>'
        )
        grid.append(
            f'<line x1="{margin}" y1="{y:.1f}" x2="{width-margin}" y2="{y:.1f}" '
            'stroke="#ddd" stroke-width="1"/>'
        )
        grid.append(
            f'<text x="{x:.1f}" y="{height-margin+24}" text-anchor="middle" '
            f'font-size="13">{int(fraction*100)}%</text>'
        )
        grid.append(
            f'<text x="{margin-12}" y="{y+4:.1f}" text-anchor="end" '
            f'font-size="13">{int(fraction*100)}%</text>'
        )

    series = [
        ('mIoU', 'miou', '#111', ''),
        ('Occupied IoU', 'occupied_iou', '#555', '8 5'),
        ('Occupied F1', 'occupied_f1', '#999', '2 4'),
    ]
    lines = []
    legend = []
    for idx, (label, key, stroke, dash) in enumerate(series):
        points = _polyline_points(rows, key, width, height, margin)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ''
        lines.append(
            f'<polyline points="{points}" fill="none" stroke="{stroke}" '
            f'stroke-width="3"{dash_attr}/>'
        )
        ly = 28 + idx * 22
        legend.append(
            f'<line x1="650" y1="{ly}" x2="690" y2="{ly}" stroke="{stroke}" '
            f'stroke-width="3"{dash_attr}/><text x="700" y="{ly+5}" '
            f'font-size="14">{html.escape(label)}</text>'
        )

    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/>
<text x="{width/2}" y="28" text-anchor="middle" font-size="20" font-weight="bold">LaMa quality vs exploration progress</text>
{''.join(grid)}
<line x1="{margin}" y1="{height-margin}" x2="{width-margin}" y2="{height-margin}" stroke="#222" stroke-width="2"/>
<line x1="{margin}" y1="{margin}" x2="{margin}" y2="{height-margin}" stroke="#222" stroke-width="2"/>
{''.join(lines)}
{''.join(legend)}
<text x="{width/2}" y="{height-12}" text-anchor="middle" font-size="15">Exploration progress relative to final SLAM map</text>
<text x="18" y="{height/2}" text-anchor="middle" font-size="15" transform="rotate(-90 18 {height/2})">Metric</text>
</svg>\n'''
    output.write_text(svg, encoding='utf-8')
    return output


def _pct(value: float | None) -> str:
    return 'n/a' if value is None else f'{100.0 * value:.1f}%'


def write_html_report(
    eval_dir: Path,
    rows: list[dict],
    selected: list[dict],
    final_frame: str,
    correlation: float | None,
) -> Path:
    output = eval_dir / 'evaluation_report.html'
    cards = []
    for row in selected:
        frame = str(row['frame'])
        cards.append(f'''
<section class="frame">
<h2>Frame {html.escape(frame)} — progress {_pct(row.get('progress_relative_to_final'))}</h2>
<p>known={_pct(row.get('known_fraction'))} · mIoU={_pct(row.get('miou'))} · occupied IoU={_pct(row.get('occupied_iou'))} · occupied F1={_pct(row.get('occupied_f1'))}</p>
<div class="images">
<figure><img src="model_input/{frame}.png"><figcaption>Observed input</figcaption></figure>
<figure><img src="model_mask/{frame}.png"><figcaption>Unknown mask</figcaption></figure>
<figure><img src="prediction/{frame}.png"><figcaption>LaMa prediction</figcaption></figure>
<figure><img src="model_input/{html.escape(final_frame)}.png"><figcaption>Final SLAM pseudo-GT</figcaption></figure>
</div>
</section>''')

    correlation_text = 'n/a' if correlation is None else f'{correlation:.3f}'
    document = f'''<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>LaMa evaluation report</title>
<style>
body {{ font-family: sans-serif; max-width: 1500px; margin: 30px auto; padding: 0 20px; color: #222; }}
img {{ width: 100%; border: 1px solid #bbb; image-rendering: auto; }}
.images {{ display: grid; grid-template-columns: repeat(4, 1fr); gap: 14px; }}
figure {{ margin: 0; }} figcaption {{ text-align: center; margin-top: 6px; }}
.frame {{ margin: 42px 0; border-top: 1px solid #ccc; padding-top: 20px; }}
.summary {{ background: #f5f5f5; padding: 16px 20px; }}
@media (max-width: 900px) {{ .images {{ grid-template-columns: repeat(2, 1fr); }} }}
</style>
</head>
<body>
<h1>LaMa Hospital evaluation</h1>
<div class="summary">
<p>Final pseudo-GT frame: <b>{html.escape(final_frame)}</b></p>
<p>Pearson correlation(progress, mIoU): <b>{correlation_text}</b></p>
<p>Plot: <a href="evaluation_metrics.svg">evaluation_metrics.svg</a> · Data: <a href="evaluation_progress.csv">evaluation_progress.csv</a></p>
</div>
<img src="evaluation_metrics.svg" alt="Evaluation plot">
{''.join(cards)}
</body>
</html>\n'''
    output.write_text(document, encoding='utf-8')
    return output


def analyze_evaluation(eval_dir: Path) -> dict:
    eval_dir = eval_dir.expanduser().resolve()
    rows = load_evaluation_rows(eval_dir)
    rows, final_known = enrich_progress(rows)
    selected = choose_qualitative_frames(rows)
    final_frame = str(rows[-1]['frame'])
    correlation = pearson_defined(rows, 'progress_relative_to_final', 'miou')

    progress_csv = write_progress_csv(eval_dir, rows)
    plot_svg = write_metrics_svg(eval_dir, rows)
    report_html = write_html_report(
        eval_dir, rows, selected, final_frame, correlation
    )
    summary = {
        'final_known_fraction': final_known,
        'pearson_progress_vs_miou': correlation,
        'qualitative_frames': [str(row['frame']) for row in selected],
        'progress_csv': progress_csv.name,
        'plot_svg': plot_svg.name,
        'report_html': report_html.name,
    }
    (eval_dir / 'analysis_summary.json').write_text(
        json.dumps(summary, indent=2) + '\n', encoding='utf-8'
    )
    return summary


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description='Create LaMa progress plot and qualitative HTML report.'
    )
    parser.add_argument('eval_dir', type=Path)
    parsed = parser.parse_args(args)

    summary = analyze_evaluation(parsed.eval_dir)
    print(f'Final known fraction: {100.0 * summary["final_known_fraction"]:.2f}%')
    correlation = summary['pearson_progress_vs_miou']
    print('Pearson(progress, mIoU): ' + ('n/a' if correlation is None else f'{correlation:.3f}'))
    print('Qualitative frames: ' + ', '.join(summary['qualitative_frames']))
    print(f'Plot:   {parsed.eval_dir / summary["plot_svg"]}')
    print(f'Report: {parsed.eval_dir / summary["report_html"]}')


if __name__ == '__main__':
    main()
