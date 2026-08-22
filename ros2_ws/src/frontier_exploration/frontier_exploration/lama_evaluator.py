"""Evaluate LaMa map completion on pixels unknown at snapshot time.

The default evaluation uses the final selected SLAM snapshot as a pseudo-ground
truth. A prediction is scored only where the snapshot was unknown *and* the
pseudo-ground-truth map is known. This avoids inflating metrics with pixels that
LaMa simply copied from the observed map, and avoids treating regions still
unknown in the final SLAM map as ground truth.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path

import numpy as np


PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


@dataclass(frozen=True)
class Confusion:
    tp: int
    fp: int
    fn: int
    tn: int

    @property
    def total(self) -> int:
        return self.tp + self.fp + self.fn + self.tn

    def __add__(self, other: 'Confusion') -> 'Confusion':
        return Confusion(
            self.tp + other.tp,
            self.fp + other.fp,
            self.fn + other.fn,
            self.tn + other.tn,
        )


def _paeth_predictor(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def _read_png_chunks(path: Path) -> tuple[dict, bytes]:
    payload = path.read_bytes()
    if not payload.startswith(PNG_SIGNATURE):
        raise ValueError(f'Not a PNG file: {path}')

    offset = len(PNG_SIGNATURE)
    header: dict | None = None
    idat_parts: list[bytes] = []
    while offset + 12 <= len(payload):
        length = struct.unpack('>I', payload[offset:offset + 4])[0]
        chunk_type = payload[offset + 4:offset + 8]
        chunk_data = payload[offset + 8:offset + 8 + length]
        offset += 12 + length

        if chunk_type == b'IHDR':
            if length != 13:
                raise ValueError(f'Invalid IHDR length in {path}')
            (
                width,
                height,
                bit_depth,
                color_type,
                compression,
                filter_method,
                interlace,
            ) = struct.unpack('>IIBBBBB', chunk_data)
            header = {
                'width': width,
                'height': height,
                'bit_depth': bit_depth,
                'color_type': color_type,
                'compression': compression,
                'filter_method': filter_method,
                'interlace': interlace,
            }
        elif chunk_type == b'IDAT':
            idat_parts.append(chunk_data)
        elif chunk_type == b'IEND':
            break

    if header is None or not idat_parts:
        raise ValueError(f'PNG is missing IHDR/IDAT data: {path}')
    return header, b''.join(idat_parts)


def read_png_grayscale(path: Path) -> np.ndarray:
    """Decode a common 8-bit non-interlaced PNG to uint8 grayscale.

    The project-generated inputs/masks are grayscale PNGs, while upstream LaMa
    normally writes RGB/RGBA PNGs. Supporting these color types keeps evaluation
    independent of Pillow/OpenCV in the ROS Python environment.
    """
    path = path.expanduser().resolve()
    header, compressed = _read_png_chunks(path)
    if header['bit_depth'] != 8:
        raise ValueError(f'Only 8-bit PNGs are supported: {path}')
    if header['compression'] != 0 or header['filter_method'] != 0:
        raise ValueError(f'Unsupported PNG compression/filter method: {path}')
    if header['interlace'] != 0:
        raise ValueError(f'Interlaced PNG is not supported: {path}')

    channels_by_type = {0: 1, 2: 3, 4: 2, 6: 4}
    color_type = int(header['color_type'])
    if color_type not in channels_by_type:
        raise ValueError(
            f'Unsupported PNG color type {color_type} in {path}; '
            'expected grayscale, RGB, grayscale+alpha, or RGBA'
        )

    width = int(header['width'])
    height = int(header['height'])
    channels = channels_by_type[color_type]
    stride = width * channels
    raw = zlib.decompress(compressed)
    expected = height * (stride + 1)
    if len(raw) != expected:
        raise ValueError(
            f'Unexpected decompressed PNG size in {path}: '
            f'{len(raw)} bytes, expected {expected}'
        )

    rows = np.empty((height, stride), dtype=np.uint8)
    source_offset = 0
    previous = bytearray(stride)
    for y in range(height):
        filter_type = raw[source_offset]
        source_offset += 1
        encoded = raw[source_offset:source_offset + stride]
        source_offset += stride
        reconstructed = bytearray(stride)

        if filter_type == 0:
            reconstructed[:] = encoded
        elif filter_type == 1:  # Sub
            for x, value in enumerate(encoded):
                left = reconstructed[x - channels] if x >= channels else 0
                reconstructed[x] = (value + left) & 0xFF
        elif filter_type == 2:  # Up
            for x, value in enumerate(encoded):
                reconstructed[x] = (value + previous[x]) & 0xFF
        elif filter_type == 3:  # Average
            for x, value in enumerate(encoded):
                left = reconstructed[x - channels] if x >= channels else 0
                up = previous[x]
                reconstructed[x] = (value + ((left + up) // 2)) & 0xFF
        elif filter_type == 4:  # Paeth
            for x, value in enumerate(encoded):
                left = reconstructed[x - channels] if x >= channels else 0
                up = previous[x]
                upper_left = previous[x - channels] if x >= channels else 0
                reconstructed[x] = (
                    value + _paeth_predictor(left, up, upper_left)
                ) & 0xFF
        else:
            raise ValueError(f'Unsupported PNG filter {filter_type} in {path}')

        rows[y] = np.frombuffer(reconstructed, dtype=np.uint8)
        previous = reconstructed

    pixels = rows.reshape(height, width, channels)
    if color_type in (0, 4):
        return pixels[:, :, 0].copy()

    rgb = pixels[:, :, :3].astype(np.float32)
    gray = np.rint(
        rgb[:, :, 0] * 0.299
        + rgb[:, :, 1] * 0.587
        + rgb[:, :, 2] * 0.114
    )
    return np.clip(gray, 0, 255).astype(np.uint8)


def _safe_div(numerator: int | float, denominator: int | float) -> float | None:
    if denominator == 0:
        return None
    return float(numerator) / float(denominator)


def metrics_from_confusion(confusion: Confusion) -> dict:
    tp, fp, fn, tn = confusion.tp, confusion.fp, confusion.fn, confusion.tn
    occupied_iou = _safe_div(tp, tp + fp + fn)
    free_iou = _safe_div(tn, tn + fp + fn)
    valid_ious = [value for value in (occupied_iou, free_iou) if value is not None]
    miou = sum(valid_ious) / len(valid_ious) if valid_ious else None

    occupied_precision = _safe_div(tp, tp + fp)
    occupied_recall = _safe_div(tp, tp + fn)
    occupied_f1 = _f1(occupied_precision, occupied_recall)

    free_precision = _safe_div(tn, tn + fn)
    free_recall = _safe_div(tn, tn + fp)
    free_f1 = _f1(free_precision, free_recall)

    return {
        'accuracy': _safe_div(tp + tn, confusion.total),
        'occupied_iou': occupied_iou,
        'free_iou': free_iou,
        'miou': miou,
        'occupied_precision': occupied_precision,
        'occupied_recall': occupied_recall,
        'occupied_f1': occupied_f1,
        'free_precision': free_precision,
        'free_recall': free_recall,
        'free_f1': free_f1,
    }


def _f1(precision: float | None, recall: float | None) -> float | None:
    if precision is None or recall is None or precision + recall == 0.0:
        return None
    return 2.0 * precision * recall / (precision + recall)


def compute_confusion(
    prediction: np.ndarray,
    ground_truth: np.ndarray,
    evaluation_mask: np.ndarray,
    threshold: int = 127,
) -> Confusion:
    if prediction.shape != ground_truth.shape or prediction.shape != evaluation_mask.shape:
        raise ValueError('Prediction, ground truth, and evaluation mask shapes must match')
    if not 0 <= threshold <= 255:
        raise ValueError('threshold must be in [0, 255]')

    valid = evaluation_mask.astype(bool)
    predicted_occupied = prediction <= threshold
    gt_occupied = ground_truth <= threshold

    return Confusion(
        tp=int(np.count_nonzero(valid & predicted_occupied & gt_occupied)),
        fp=int(np.count_nonzero(valid & predicted_occupied & ~gt_occupied)),
        fn=int(np.count_nonzero(valid & ~predicted_occupied & gt_occupied)),
        tn=int(np.count_nonzero(valid & ~predicted_occupied & ~gt_occupied)),
    )


def _load_known_fractions(eval_dir: Path) -> dict[str, float]:
    candidates = [
        eval_dir / 'selection_manifest.jsonl',
        eval_dir / 'lama_dataset_manifest.jsonl',
    ]
    for path in candidates:
        if not path.is_file():
            continue
        result: dict[str, float] = {}
        for line in path.read_text(encoding='utf-8').splitlines():
            if not line.strip():
                continue
            record = json.loads(line)
            if 'frame' in record and 'known_fraction' in record:
                result[str(record['frame'])] = float(record['known_fraction'])
        if result:
            return result
    return {}


def _discover_frames(eval_dir: Path) -> list[str]:
    input_dir = eval_dir / 'model_input'
    mask_dir = eval_dir / 'model_mask'
    prediction_dir = eval_dir / 'prediction'
    for directory in (input_dir, mask_dir, prediction_dir):
        if not directory.is_dir():
            raise FileNotFoundError(f'Missing evaluation directory: {directory}')

    frames = sorted(
        (path.stem for path in input_dir.glob('*.png') if path.stem.isdigit()),
        key=int,
    )
    if not frames:
        raise FileNotFoundError(f'No model_input PNGs found in {input_dir}')
    for frame in frames:
        for directory in (mask_dir, prediction_dir):
            path = directory / f'{frame}.png'
            if not path.is_file():
                raise FileNotFoundError(f'Missing frame {frame}: {path}')
    return frames


def _default_gt_frame(eval_dir: Path, frames: list[str]) -> str:
    selection_path = eval_dir / 'selection.json'
    if selection_path.is_file():
        payload = json.loads(selection_path.read_text(encoding='utf-8'))
        last_frame = str(payload.get('last_frame', ''))
        if last_frame in frames:
            return last_frame
    return frames[-1]


def _mean_defined(rows: list[dict], key: str) -> float | None:
    values = [float(row[key]) for row in rows if row.get(key) is not None]
    if not values:
        return None
    return sum(values) / len(values)


def _csv_value(value: object) -> object:
    if value is None:
        return ''
    if isinstance(value, float):
        return f'{value:.8f}'
    return value


def evaluate_lama_predictions(
    eval_dir: Path,
    *,
    gt_frame: str | None = None,
    threshold: int = 127,
) -> dict:
    """Evaluate all predictions against a final-frame pseudo-ground truth."""
    eval_dir = eval_dir.expanduser().resolve()
    frames = _discover_frames(eval_dir)
    if gt_frame is None:
        gt_frame = _default_gt_frame(eval_dir, frames)
    gt_frame = str(gt_frame)
    if gt_frame not in frames:
        raise ValueError(f'Ground-truth frame {gt_frame} is not in the evaluation subset')

    input_dir = eval_dir / 'model_input'
    mask_dir = eval_dir / 'model_mask'
    prediction_dir = eval_dir / 'prediction'
    known_fractions = _load_known_fractions(eval_dir)

    gt_image = read_png_grayscale(input_dir / f'{gt_frame}.png')
    gt_unknown = read_png_grayscale(mask_dir / f'{gt_frame}.png') >= 128
    gt_known = ~gt_unknown

    rows: list[dict] = []
    aggregate_confusion = Confusion(0, 0, 0, 0)
    for frame in frames:
        prediction = read_png_grayscale(prediction_dir / f'{frame}.png')
        snapshot_unknown = read_png_grayscale(mask_dir / f'{frame}.png') >= 128
        if prediction.shape != gt_image.shape or snapshot_unknown.shape != gt_image.shape:
            raise ValueError(
                f'Image shape mismatch for frame {frame}: prediction={prediction.shape}, '
                f'mask={snapshot_unknown.shape}, gt={gt_image.shape}'
            )

        evaluation_mask = snapshot_unknown & gt_known
        confusion = compute_confusion(
            prediction,
            gt_image,
            evaluation_mask,
            threshold=threshold,
        )
        metrics = metrics_from_confusion(confusion)
        unknown_pixels = int(np.count_nonzero(snapshot_unknown))
        eval_pixels = int(np.count_nonzero(evaluation_mask))
        unresolved_in_gt = int(np.count_nonzero(snapshot_unknown & gt_unknown))

        row = {
            'frame': frame,
            'known_fraction': known_fractions.get(frame),
            'unknown_pixels': unknown_pixels,
            'eval_pixels': eval_pixels,
            'unresolved_in_pseudo_gt_pixels': unresolved_in_gt,
            'labeled_fraction_of_unknown': _safe_div(eval_pixels, unknown_pixels),
            'tp_occupied': confusion.tp,
            'fp_occupied': confusion.fp,
            'fn_occupied': confusion.fn,
            'tn_free': confusion.tn,
            **metrics,
        }
        rows.append(row)
        aggregate_confusion = aggregate_confusion + confusion

    evaluable_rows = [row for row in rows if int(row['eval_pixels']) > 0]
    micro_metrics = metrics_from_confusion(aggregate_confusion)
    metric_keys = list(micro_metrics.keys())
    macro_metrics = {
        key: _mean_defined(evaluable_rows, key)
        for key in metric_keys
    }

    csv_path = eval_dir / 'evaluation.csv'
    fieldnames = list(rows[0].keys())
    with csv_path.open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(value) for key, value in row.items()})

    summary = {
        'evaluation_type': 'unknown-at-snapshot vs final-SLAM pseudo-ground-truth',
        'eval_dir': str(eval_dir),
        'pseudo_gt_frame': gt_frame,
        'threshold_grayscale': threshold,
        'class_rule': f'occupied <= {threshold}; free > {threshold}',
        'evaluation_region': (
            'snapshot unknown pixels intersected with pixels known in pseudo-GT'
        ),
        'frames_total': len(rows),
        'frames_evaluable': len(evaluable_rows),
        'frames_without_labeled_unknown_pixels': [
            row['frame'] for row in rows if int(row['eval_pixels']) == 0
        ],
        'aggregate_pixels': aggregate_confusion.total,
        'aggregate_confusion': {
            'tp_occupied': aggregate_confusion.tp,
            'fp_occupied': aggregate_confusion.fp,
            'fn_occupied': aggregate_confusion.fn,
            'tn_free': aggregate_confusion.tn,
        },
        'micro_metrics': micro_metrics,
        'macro_frame_metrics': macro_metrics,
        'csv': csv_path.name,
        'caveat': (
            'The final SLAM map is a pseudo-ground-truth, not simulator ground truth. '
            'Pixels still unknown in that final map are excluded. The final frame '
            'normally has zero evaluable pixels because it is being used as its own GT.'
        ),
    }
    summary_path = eval_dir / 'evaluation_summary.json'
    summary_path.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    return summary


def _format_metric(value: float | None) -> str:
    return 'n/a' if value is None else f'{100.0 * value:.2f}%'


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Evaluate LaMa predictions only on pixels unknown at snapshot time, '
            'using the final selected SLAM map as pseudo-ground-truth.'
        )
    )
    parser.add_argument(
        'eval_dir',
        type=Path,
        help='Directory containing model_input/, model_mask/, and prediction/.',
    )
    parser.add_argument(
        '--gt-frame',
        default=None,
        help='Pseudo-GT frame ID (default: final selected frame).',
    )
    parser.add_argument(
        '--threshold',
        type=int,
        default=127,
        help='Grayscale occupied/free split; occupied <= threshold (default: 127).',
    )
    parsed = parser.parse_args(args)

    summary = evaluate_lama_predictions(
        parsed.eval_dir,
        gt_frame=parsed.gt_frame,
        threshold=parsed.threshold,
    )
    metrics = summary['micro_metrics']
    print(
        f'Evaluated {summary["frames_evaluable"]}/{summary["frames_total"]} '
        f'frame(s) against pseudo-GT frame {summary["pseudo_gt_frame"]}.'
    )
    print(f'Evaluation pixels: {summary["aggregate_pixels"]}')
    print(f'Occupied IoU: {_format_metric(metrics["occupied_iou"])}')
    print(f'Free IoU:     {_format_metric(metrics["free_iou"])}')
    print(f'mIoU:         {_format_metric(metrics["miou"])}')
    print(f'Occ precision:{_format_metric(metrics["occupied_precision"])}')
    print(f'Occ recall:   {_format_metric(metrics["occupied_recall"])}')
    print(f'Occ F1:       {_format_metric(metrics["occupied_f1"])}')
    print(f'CSV:          {Path(summary["eval_dir"]) / summary["csv"]}')
    print(
        'Summary:      '
        + str(Path(summary['eval_dir']) / 'evaluation_summary.json')
    )


if __name__ == '__main__':
    main()
