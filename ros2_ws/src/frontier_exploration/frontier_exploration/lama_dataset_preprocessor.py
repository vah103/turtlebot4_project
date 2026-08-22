"""Prepare fixed-canvas SLAM snapshots for LaMa inference.

The recorder stores exact signed-int8 OccupancyGrid snapshots at the SLAM
resolution. This module converts those raw grids into deterministic PNG pairs:

- model_input/<frame>.png : grayscale occupancy map (free=255, occupied=0,
  unknown=127)
- model_mask/<frame>.png  : LaMa mask (unknown=255, known=0)

For MapEx compatibility the default target resolution is 0.10 m/pixel. The
result is padded with unknown cells on +X/+Y to a size divisible by 8, without
changing metric scale.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class DatasetGeometry:
    """Geometry of the generated LaMa images."""

    source_width: int
    source_height: int
    source_resolution: float
    scaled_width: int
    scaled_height: int
    target_resolution: float
    output_width: int
    output_height: int
    pad_right: int
    pad_top: int


def _ceil_divisible(value: int, multiple: int) -> int:
    if value <= 0:
        raise ValueError('value must be positive')
    if multiple <= 0:
        raise ValueError('multiple must be positive')
    return ((value + multiple - 1) // multiple) * multiple


def plan_dataset_geometry(
    source_width: int,
    source_height: int,
    source_resolution: float,
    target_resolution: float = 0.10,
    pad_multiple: int = 8,
) -> DatasetGeometry:
    """Plan metric-preserving resampling and unknown padding."""
    if source_width <= 0 or source_height <= 0:
        raise ValueError('source dimensions must be positive')
    if source_resolution <= 0.0 or target_resolution <= 0.0:
        raise ValueError('resolutions must be positive')
    if target_resolution + 1e-12 < source_resolution:
        raise ValueError(
            'target_resolution must be >= source_resolution; this preprocessor '
            'is intended for occupancy-grid downsampling, not upsampling'
        )

    metric_width = source_width * source_resolution
    metric_height = source_height * source_resolution
    scaled_width = max(1, int(math.ceil(metric_width / target_resolution - 1e-12)))
    scaled_height = max(1, int(math.ceil(metric_height / target_resolution - 1e-12)))
    output_width = _ceil_divisible(scaled_width, pad_multiple)
    output_height = _ceil_divisible(scaled_height, pad_multiple)

    return DatasetGeometry(
        source_width=source_width,
        source_height=source_height,
        source_resolution=source_resolution,
        scaled_width=scaled_width,
        scaled_height=scaled_height,
        target_resolution=target_resolution,
        output_width=output_width,
        output_height=output_height,
        pad_right=output_width - scaled_width,
        # ROS rows grow toward +Y. Padding extra rows therefore adds metric
        # extent at +Y, which appears at the top of the top-down PNG.
        pad_top=output_height - scaled_height,
    )


def decode_signed_int8(payload: bytes, expected_cells: int) -> list[int]:
    """Decode recorder *_occupancy.bin bytes back to signed int8 values."""
    if len(payload) != expected_cells:
        raise ValueError(
            f'Occupancy payload has {len(payload)} bytes; expected {expected_cells}'
        )
    return [value if value < 128 else value - 256 for value in payload]


def downsample_occupancy(
    data: list[int],
    source_width: int,
    source_height: int,
    source_resolution: float,
    target_resolution: float,
) -> tuple[list[int], int, int]:
    """Downsample a ROS row-major occupancy grid while preserving obstacles.

    Each target cell covers its metric source footprint. If every source cell
    in that footprint is unknown, the target remains unknown. Otherwise the
    maximum known occupancy value is retained. This avoids erasing thin wall
    cells while never inventing a known value from an all-unknown region.
    """
    geometry = plan_dataset_geometry(
        source_width,
        source_height,
        source_resolution,
        target_resolution,
        pad_multiple=1,
    )
    if len(data) != source_width * source_height:
        raise ValueError('Occupancy data length does not match source dimensions')

    ratio = target_resolution / source_resolution
    output = [-1] * (geometry.scaled_width * geometry.scaled_height)

    for target_y in range(geometry.scaled_height):
        source_y0 = int(math.floor(target_y * ratio + 1e-12))
        source_y1 = int(math.ceil((target_y + 1) * ratio - 1e-12))
        source_y0 = min(max(source_y0, 0), source_height)
        source_y1 = min(max(source_y1, source_y0 + 1), source_height)
        if source_y0 >= source_height:
            continue

        for target_x in range(geometry.scaled_width):
            source_x0 = int(math.floor(target_x * ratio + 1e-12))
            source_x1 = int(math.ceil((target_x + 1) * ratio - 1e-12))
            source_x0 = min(max(source_x0, 0), source_width)
            source_x1 = min(max(source_x1, source_x0 + 1), source_width)
            if source_x0 >= source_width:
                continue

            known_values: list[int] = []
            for source_y in range(source_y0, source_y1):
                row_offset = source_y * source_width
                for source_x in range(source_x0, source_x1):
                    value = int(data[row_offset + source_x])
                    if value >= 0:
                        known_values.append(max(0, min(100, value)))

            if known_values:
                output[target_y * geometry.scaled_width + target_x] = max(
                    known_values
                )

    return output, geometry.scaled_width, geometry.scaled_height


def pad_occupancy_positive_axes(
    data: list[int],
    width: int,
    height: int,
    output_width: int,
    output_height: int,
) -> list[int]:
    """Pad an occupancy grid with unknown cells on +X and +Y only."""
    if len(data) != width * height:
        raise ValueError('Occupancy data length does not match dimensions')
    if output_width < width or output_height < height:
        raise ValueError('Output dimensions must not be smaller than input')

    padded = [-1] * (output_width * output_height)
    for row in range(height):
        src_offset = row * width
        dst_offset = row * output_width
        padded[dst_offset:dst_offset + width] = data[src_offset:src_offset + width]
    return padded


def occupancy_to_top_down_pixels(
    data: list[int],
    width: int,
    height: int,
) -> bytes:
    """Convert ROS occupancy values to top-down grayscale image pixels."""
    if len(data) != width * height:
        raise ValueError('Occupancy data length does not match dimensions')

    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        offset = row * width
        for raw_value in data[offset:offset + width]:
            value = int(raw_value)
            if value < 0:
                pixels.append(127)
            else:
                occupancy = max(0, min(100, value))
                pixels.append(round(255 * (100 - occupancy) / 100))
    return bytes(pixels)


def unknown_mask_to_top_down_pixels(
    data: list[int],
    width: int,
    height: int,
) -> bytes:
    """Return a LaMa mask: unknown=255, known=0, in top-down image order."""
    if len(data) != width * height:
        raise ValueError('Occupancy data length does not match dimensions')

    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        offset = row * width
        for value in data[offset:offset + width]:
            pixels.append(255 if int(value) < 0 else 0)
    return bytes(pixels)


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    body = chunk_type + payload
    return (
        struct.pack('>I', len(payload))
        + body
        + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)
    )


def encode_grayscale_png(width: int, height: int, pixels: bytes) -> bytes:
    """Encode an 8-bit grayscale PNG using only the Python standard library."""
    if width <= 0 or height <= 0 or len(pixels) != width * height:
        raise ValueError('Pixel data length does not match PNG dimensions')

    scanlines = bytearray()
    for row in range(height):
        scanlines.append(0)  # PNG filter type 0
        start = row * width
        scanlines.extend(pixels[start:start + width])

    signature = b'\x89PNG\r\n\x1a\n'
    ihdr = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return b''.join(
        [
            signature,
            _png_chunk(b'IHDR', ihdr),
            _png_chunk(b'IDAT', zlib.compress(bytes(scanlines), level=6)),
            _png_chunk(b'IEND', b''),
        ]
    )


def _write_bytes_atomic(path: Path, payload: bytes) -> None:
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('wb') as stream:
        stream.write(payload)
    os.replace(temporary, path)


def _write_json_atomic(path: Path, payload: dict) -> None:
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    os.replace(temporary, path)


def _load_run_geometry(run_dir: Path) -> tuple[int, int, float, float, float]:
    run_path = run_dir / 'run.json'
    if not run_path.is_file():
        raise FileNotFoundError(f'Missing recorder metadata: {run_path}')
    with run_path.open('r', encoding='utf-8') as stream:
        payload = json.load(stream)

    fixed = payload.get('fixed_canvas')
    if not isinstance(fixed, dict):
        raise ValueError(f'run.json has no fixed_canvas object: {run_path}')
    origin = fixed.get('origin', {})
    return (
        int(fixed['width']),
        int(fixed['height']),
        float(fixed['resolution']),
        float(origin['x']),
        float(origin['y']),
    )


def _clean_generated_pngs(directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for path in directory.glob('*.png'):
        path.unlink()


def prepare_dataset(
    run_dir: Path,
    target_resolution: float = 0.10,
    pad_multiple: int = 8,
) -> dict:
    """Generate model_input/model_mask PNG pairs for every recorded frame."""
    run_dir = run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        raise NotADirectoryError(f'Run directory not found: {run_dir}')

    width, height, source_resolution, origin_x, origin_y = _load_run_geometry(
        run_dir
    )
    geometry = plan_dataset_geometry(
        width,
        height,
        source_resolution,
        target_resolution,
        pad_multiple,
    )

    raw_paths = sorted(run_dir.glob('*_occupancy.bin'))
    if not raw_paths:
        raise FileNotFoundError(f'No *_occupancy.bin snapshots found in {run_dir}')

    input_dir = run_dir / 'model_input'
    mask_dir = run_dir / 'model_mask'
    _clean_generated_pngs(input_dir)
    _clean_generated_pngs(mask_dir)

    manifest_records: list[dict] = []
    expected_cells = width * height

    for raw_path in raw_paths:
        stem = raw_path.name.removesuffix('_occupancy.bin')
        if not stem.isdigit():
            continue

        raw_data = decode_signed_int8(raw_path.read_bytes(), expected_cells)
        scaled, scaled_width, scaled_height = downsample_occupancy(
            raw_data,
            width,
            height,
            source_resolution,
            target_resolution,
        )
        padded = pad_occupancy_positive_axes(
            scaled,
            scaled_width,
            scaled_height,
            geometry.output_width,
            geometry.output_height,
        )

        input_pixels = occupancy_to_top_down_pixels(
            padded,
            geometry.output_width,
            geometry.output_height,
        )
        mask_pixels = unknown_mask_to_top_down_pixels(
            padded,
            geometry.output_width,
            geometry.output_height,
        )

        input_path = input_dir / f'{stem}.png'
        mask_path = mask_dir / f'{stem}.png'
        _write_bytes_atomic(
            input_path,
            encode_grayscale_png(
                geometry.output_width,
                geometry.output_height,
                input_pixels,
            ),
        )
        _write_bytes_atomic(
            mask_path,
            encode_grayscale_png(
                geometry.output_width,
                geometry.output_height,
                mask_pixels,
            ),
        )

        known_cells = sum(1 for value in padded if value >= 0)
        manifest_records.append(
            {
                'frame': stem,
                'source_raw': raw_path.name,
                'input': str(input_path.relative_to(run_dir)),
                'mask': str(mask_path.relative_to(run_dir)),
                'known_fraction': known_cells / len(padded),
            }
        )

    if not manifest_records:
        raise ValueError(f'No valid numeric snapshot files found in {run_dir}')

    manifest_path = run_dir / 'lama_dataset_manifest.jsonl'
    with manifest_path.open('w', encoding='utf-8') as stream:
        for record in manifest_records:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')

    metadata = {
        'source': {
            'run_dir': str(run_dir),
            'width_cells': width,
            'height_cells': height,
            'resolution_m_per_cell': source_resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
        },
        'model': {
            'target_resolution_m_per_pixel': target_resolution,
            'scaled_width_pixels': geometry.scaled_width,
            'scaled_height_pixels': geometry.scaled_height,
            'pad_multiple': pad_multiple,
            'output_width_pixels': geometry.output_width,
            'output_height_pixels': geometry.output_height,
            'pad_right_pixels': geometry.pad_right,
            'pad_top_pixels': geometry.pad_top,
            'input_encoding': 'grayscale: free=255, occupied=0, unknown=127',
            'mask_encoding': 'grayscale: unknown=255, known=0',
            'padding': 'unknown cells on +X/+Y; metric origin unchanged',
            'downsampling': (
                'metric source footprint; all-unknown remains unknown; otherwise '
                'maximum known occupancy is retained to preserve obstacles'
            ),
        },
        'frames': len(manifest_records),
        'manifest': manifest_path.name,
        'input_dir': input_dir.name,
        'mask_dir': mask_dir.name,
    }
    _write_json_atomic(run_dir / 'preprocess.json', metadata)
    return metadata


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Convert fixed-canvas TurtleBot4 snapshot runs into LaMa model_input '
            'and model_mask PNG datasets.'
        )
    )
    parser.add_argument(
        'run_dir',
        type=Path,
        help='Snapshot run directory containing run.json and *_occupancy.bin files.',
    )
    parser.add_argument(
        '--target-resolution',
        type=float,
        default=0.10,
        help='Output metric resolution in m/pixel (default: 0.10).',
    )
    parser.add_argument(
        '--pad-multiple',
        type=int,
        default=8,
        help='Pad image dimensions to this multiple using unknown cells (default: 8).',
    )
    parsed = parser.parse_args(args)

    metadata = prepare_dataset(
        parsed.run_dir,
        target_resolution=parsed.target_resolution,
        pad_multiple=parsed.pad_multiple,
    )
    model = metadata['model']
    print(f"Prepared {metadata['frames']} LaMa frame(s).")
    print(
        'Model image size: '
        f"{model['output_width_pixels']}x{model['output_height_pixels']} px"
    )
    print(
        'Metric resolution: '
        f"{model['target_resolution_m_per_pixel']:.3f} m/pixel"
    )
    print(f"Input: {metadata['source']['run_dir']}/model_input")
    print(f"Mask:  {metadata['source']['run_dir']}/model_mask")


if __name__ == '__main__':
    main()
