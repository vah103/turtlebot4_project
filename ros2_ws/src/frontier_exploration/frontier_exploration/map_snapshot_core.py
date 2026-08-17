"""ROS-independent helpers for recording occupancy-grid snapshots."""

import json
import os
from math import hypot
from pathlib import Path


class SnapshotPolicy:
    """Trigger snapshots after enough movement or elapsed time."""

    def __init__(
        self,
        distance_interval_m: float,
        time_interval_sec: float,
        min_interval_sec: float,
    ) -> None:
        self.distance_interval_m = max(0.0, float(distance_interval_m))
        self.time_interval_sec = max(0.0, float(time_interval_sec))
        self.min_interval_sec = max(0.0, float(min_interval_sec))
        self.last_time_sec: float | None = None
        self.distance_since_capture_m = 0.0
        self.last_observed_x: float | None = None
        self.last_observed_y: float | None = None

    def observe_motion(self, x: float, y: float) -> None:
        """Accumulate odometry path length, including turns and loops."""
        if self.last_observed_x is not None and self.last_observed_y is not None:
            self.distance_since_capture_m += hypot(
                x - self.last_observed_x,
                y - self.last_observed_y,
            )
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
