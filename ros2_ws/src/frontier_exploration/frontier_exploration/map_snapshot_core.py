"""ROS-independent helpers for recording occupancy-grid snapshots."""

import json
import math
import os
from dataclasses import dataclass
from math import hypot, isclose
from pathlib import Path


@dataclass(frozen=True)
class FixedCanvasSpec:
    """Geometry shared by every snapshot in one experiment."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError('Fixed canvas width and height must be positive')
        if self.resolution <= 0.0:
            raise ValueError('Fixed canvas resolution must be positive')

    @property
    def cell_count(self) -> int:
        return self.width * self.height

    def as_dict(self) -> dict:
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin': {
                'x': self.origin_x,
                'y': self.origin_y,
            },
        }


def unknown_canvas(spec: FixedCanvasSpec) -> list[int]:
    """Return an all-unknown fixed occupancy canvas."""
    return [-1] * spec.cell_count


def project_occupancy_to_fixed_canvas(
    data: list[int],
    source_width: int,
    source_height: int,
    source_resolution: float,
    source_origin_x: float,
    source_origin_y: float,
    canvas: FixedCanvasSpec,
) -> tuple[list[int], dict]:
    """Project one axis-aligned ROS OccupancyGrid onto a fixed canvas.

    SLAM Toolbox may publish an OccupancyGrid whose origin is not exactly on the
    same metric cell lattice as a canvas chosen before the run. Both grids use
    the same resolution and axes, so each source cell is assigned to the fixed
    cell containing its centre (nearest-cell resampling). This keeps every saved
    snapshot on one fixed geometry without requiring an artificial exact-origin
    alignment.

    Unknown source cells do not overwrite anything; every output snapshot is
    rebuilt from an all-unknown canvas.
    """
    if (
        source_width <= 0
        or source_height <= 0
        or len(data) != source_width * source_height
    ):
        raise ValueError('Source OccupancyGrid dimensions do not match its data')

    if not isclose(
        float(source_resolution),
        canvas.resolution,
        rel_tol=1e-6,
        abs_tol=1e-9,
    ):
        raise ValueError(
            'Source map resolution does not match fixed canvas resolution: '
            f'{source_resolution} != {canvas.resolution}'
        )

    x_offset_float = (
        float(source_origin_x) - canvas.origin_x
    ) / canvas.resolution
    y_offset_float = (
        float(source_origin_y) - canvas.origin_y
    ) / canvas.resolution

    # Mapping by source-cell centres is equivalent, for equal resolutions, to
    # rounding the origin offset to the nearest fixed cell. Use floor(x + 0.5)
    # instead of Python round() so half-cell ties are deterministic.
    x_offset = math.floor(x_offset_float + 0.5)
    y_offset = math.floor(y_offset_float + 0.5)

    output = unknown_canvas(canvas)
    known_copied = 0
    known_outside_canvas = 0

    for source_y in range(source_height):
        canvas_y = source_y + y_offset
        source_row = source_y * source_width
        for source_x in range(source_width):
            value = int(data[source_row + source_x])
            if value < 0:
                continue

            canvas_x = source_x + x_offset
            if (
                canvas_x < 0
                or canvas_x >= canvas.width
                or canvas_y < 0
                or canvas_y >= canvas.height
            ):
                known_outside_canvas += 1
                continue

            output[canvas_y * canvas.width + canvas_x] = value
            known_copied += 1

    return output, {
        'source_offset_cells': {
            'x': x_offset,
            'y': y_offset,
        },
        'source_offset_cells_float': {
            'x': x_offset_float,
            'y': y_offset_float,
        },
        'subcell_alignment_error_cells': {
            'x': x_offset_float - x_offset,
            'y': y_offset_float - y_offset,
        },
        'projection_method': 'nearest_cell_center',
        'known_cells_copied': known_copied,
        'known_cells_outside_canvas': known_outside_canvas,
    }


def occupancy_known_mask(data: list[int]) -> bytes:
    """Encode only known/unknown state for lightweight snapshot deduplication."""
    return bytes(1 if int(value) >= 0 else 0 for value in data)


def known_mask_change_count(previous: bytes, current: bytes) -> int:
    """Count cells whose known/unknown state changed between two fixed maps."""
    if len(previous) != len(current):
        raise ValueError('Known-mask dimensions do not match')
    return sum(first != second for first, second in zip(previous, current))


class SnapshotPolicy:
    """Trigger snapshots after enough movement or elapsed time."""

    def __init__(
        self,
        distance_interval_m: float,
        time_interval_sec: float,
        min_interval_sec: float,
        max_odom_step_m: float = 0.0,
    ) -> None:
        self.distance_interval_m = max(0.0, float(distance_interval_m))
        self.time_interval_sec = max(0.0, float(time_interval_sec))
        self.min_interval_sec = max(0.0, float(min_interval_sec))
        self.max_odom_step_m = max(0.0, float(max_odom_step_m))
        self.last_time_sec: float | None = None
        self.distance_since_capture_m = 0.0
        self.last_observed_x: float | None = None
        self.last_observed_y: float | None = None

    def observe_motion(self, x: float, y: float) -> None:
        """Accumulate odometry path length, including turns and loops."""
        if self.last_observed_x is not None and self.last_observed_y is not None:
            step_m = hypot(
                x - self.last_observed_x,
                y - self.last_observed_y,
            )
            if self.max_odom_step_m <= 0.0 or step_m <= self.max_odom_step_m:
                self.distance_since_capture_m += step_m
        self.last_observed_x = x
        self.last_observed_y = y

    def should_capture(self, now_sec: float) -> bool:
        if self.last_time_sec is None:
            return True

        elapsed = max(0.0, now_sec - self.last_time_sec)
        if elapsed < self.min_interval_sec:
            return False

        distance_due = (
            self.distance_interval_m > 0.0
            and self.distance_since_capture_m >= self.distance_interval_m
        )
        time_due = (
            self.time_interval_sec > 0.0
            and elapsed >= self.time_interval_sec
        )
        return distance_due or time_due

    def record_capture(self, now_sec: float) -> None:
        self.last_time_sec = now_sec
        self.distance_since_capture_m = 0.0


def create_unique_run_dir(output_dir: Path, run_name: str) -> Path:
    """Create a run directory without overwriting an earlier experiment."""
    suffix = 0
    while True:
        name = run_name if suffix == 0 else f'{run_name}_{suffix:03d}'
        candidate = output_dir / name
        try:
            candidate.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            suffix += 1
            continue
        return candidate


def nearest_timestamp_index(
    target_sec: float,
    candidate_sec: list[float],
    tolerance_sec: float,
) -> tuple[int, float] | None:
    """Return the nearest timestamp index and absolute delta within tolerance."""
    if not candidate_sec:
        return None
    index = min(
        range(len(candidate_sec)),
        key=lambda item: abs(candidate_sec[item] - target_sec),
    )
    delta = abs(candidate_sec[index] - target_sec)
    if delta > max(0.0, tolerance_sec):
        return None
    return index, delta


def occupancy_to_pgm(
    data: list[int],
    width: int,
    height: int,
) -> bytes:
    """Encode a ROS OccupancyGrid as a top-down 8-bit PGM image.

    Unknown cells become 127, free cells 255, occupied cells 0, and
    intermediate occupancy values are mapped linearly.
    """
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('OccupancyGrid dimensions do not match its data')

    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        offset = row * width
        for value in data[offset:offset + width]:
            if value < 0:
                pixels.append(127)
            else:
                occupancy = max(0, min(100, int(value)))
                pixels.append(round(255 * (100 - occupancy) / 100))

    header = f'P5\n{width} {height}\n255\n'.encode('ascii')
    return header + bytes(pixels)


def occupancy_to_signed_bytes(data: list[int]) -> bytes:
    """Preserve exact signed int8 occupancy values in a binary file."""
    return bytes(int(value) & 0xFF for value in data)


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


def write_snapshot_files(
    run_dir: Path,
    sequence: int,
    pgm: bytes,
    raw: bytes,
    metadata: dict,
) -> None:
    """Write the PGM, raw grid, metadata, and one manifest record."""
    prefix = f'{sequence:06d}'
    _write_bytes_atomic(run_dir / f'{prefix}_map.pgm', pgm)
    _write_bytes_atomic(run_dir / f'{prefix}_occupancy.bin', raw)
    _write_json_atomic(run_dir / f'{prefix}_metadata.json', metadata)
    with (run_dir / 'manifest.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(metadata, ensure_ascii=False) + '\n')


def write_json_atomic(path: Path, payload: dict) -> None:
    """Public wrapper used for run-level metadata."""
    _write_json_atomic(path, payload)
