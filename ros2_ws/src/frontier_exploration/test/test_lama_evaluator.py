import json
import struct
import zlib
from pathlib import Path

import numpy as np
import pytest

from frontier_exploration.lama_dataset_preprocessor import encode_grayscale_png
from frontier_exploration.lama_evaluator import (
    Confusion,
    compute_confusion,
    evaluate_lama_predictions,
    metrics_from_confusion,
    read_png_grayscale,
)


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    body = kind + payload
    return (
        struct.pack('>I', len(payload))
        + body
        + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)
    )


def _encode_rgb_png(array: np.ndarray) -> bytes:
    height, width, channels = array.shape
    assert channels == 3
    scanlines = bytearray()
    for row in array.astype(np.uint8):
        scanlines.append(0)
        scanlines.extend(row.tobytes())
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    return b''.join(
        [
            b'\x89PNG\r\n\x1a\n',
            _png_chunk(b'IHDR', ihdr),
            _png_chunk(b'IDAT', zlib.compress(bytes(scanlines))),
            _png_chunk(b'IEND', b''),
        ]
    )


def _write_gray(path: Path, array: np.ndarray) -> None:
    height, width = array.shape
    path.write_bytes(
        encode_grayscale_png(width, height, array.astype(np.uint8).tobytes())
    )


def test_read_png_grayscale_supports_rgb_prediction(tmp_path: Path) -> None:
    rgb = np.array(
        [
            [[0, 0, 0], [255, 255, 255]],
            [[100, 100, 100], [200, 200, 200]],
        ],
        dtype=np.uint8,
    )
    path = tmp_path / 'prediction.png'
    path.write_bytes(_encode_rgb_png(rgb))

    decoded = read_png_grayscale(path)

    assert decoded.tolist() == [[0, 255], [100, 200]]


def test_compute_confusion_and_metrics() -> None:
    ground_truth = np.array([[0, 0, 255, 255]], dtype=np.uint8)
    prediction = np.array([[0, 255, 0, 255]], dtype=np.uint8)
    valid = np.ones_like(ground_truth, dtype=bool)

    confusion = compute_confusion(prediction, ground_truth, valid)
    metrics = metrics_from_confusion(confusion)

    assert confusion == Confusion(tp=1, fp=1, fn=1, tn=1)
    assert metrics['occupied_iou'] == pytest.approx(1 / 3)
    assert metrics['free_iou'] == pytest.approx(1 / 3)
    assert metrics['miou'] == pytest.approx(1 / 3)
    assert metrics['occupied_precision'] == pytest.approx(0.5)
    assert metrics['occupied_recall'] == pytest.approx(0.5)
    assert metrics['occupied_f1'] == pytest.approx(0.5)


def test_evaluate_uses_only_snapshot_unknown_and_final_known(tmp_path: Path) -> None:
    eval_dir = tmp_path / 'lama_eval_3'
    input_dir = eval_dir / 'model_input'
    mask_dir = eval_dir / 'model_mask'
    prediction_dir = eval_dir / 'prediction'
    input_dir.mkdir(parents=True)
    mask_dir.mkdir()
    prediction_dir.mkdir()

    # Final pseudo-GT: [occupied, free, occupied, unknown].
    final_image = np.array([[0, 255, 0, 127]], dtype=np.uint8)
    final_mask = np.array([[0, 0, 0, 255]], dtype=np.uint8)

    frame1_input = np.array([[0, 127, 127, 127]], dtype=np.uint8)
    frame1_mask = np.array([[0, 255, 255, 255]], dtype=np.uint8)
    frame1_prediction = np.array([[0, 255, 255, 0]], dtype=np.uint8)

    frame2_input = np.array([[0, 255, 127, 127]], dtype=np.uint8)
    frame2_mask = np.array([[0, 0, 255, 255]], dtype=np.uint8)
    frame2_prediction = np.array([[0, 255, 0, 255]], dtype=np.uint8)

    for frame, image, mask, prediction in (
        ('000001', frame1_input, frame1_mask, frame1_prediction),
        ('000002', frame2_input, frame2_mask, frame2_prediction),
        ('000003', final_image, final_mask, final_image),
    ):
        _write_gray(input_dir / f'{frame}.png', image)
        _write_gray(mask_dir / f'{frame}.png', mask)
        _write_gray(prediction_dir / f'{frame}.png', prediction)

    (eval_dir / 'selection.json').write_text(
        json.dumps(
            {
                'last_frame': '000003',
                'frames': ['000001', '000002', '000003'],
            }
        ),
        encoding='utf-8',
    )
    with (eval_dir / 'selection_manifest.jsonl').open('w', encoding='utf-8') as stream:
        for frame, fraction in (
            ('000001', 0.25),
            ('000002', 0.50),
            ('000003', 0.75),
        ):
            stream.write(
                json.dumps({'frame': frame, 'known_fraction': fraction}) + '\n'
            )

    summary = evaluate_lama_predictions(eval_dir)

    assert summary['pseudo_gt_frame'] == '000003'
    assert summary['frames_total'] == 3
    assert summary['frames_evaluable'] == 2
    assert summary['frames_without_labeled_unknown_pixels'] == ['000003']
    # Frame 1 evaluates pixels 1 and 2; pixel 3 is still unknown in pseudo-GT.
    # Frame 2 evaluates only pixel 2. Aggregate: TP=1, FP=0, FN=1, TN=1.
    assert summary['aggregate_confusion'] == {
        'tp_occupied': 1,
        'fp_occupied': 0,
        'fn_occupied': 1,
        'tn_free': 1,
    }
    assert summary['aggregate_pixels'] == 3
    assert summary['micro_metrics']['occupied_iou'] == pytest.approx(0.5)
    assert summary['micro_metrics']['free_iou'] == pytest.approx(0.5)
    assert (eval_dir / 'evaluation.csv').is_file()
    assert (eval_dir / 'evaluation_summary.json').is_file()
