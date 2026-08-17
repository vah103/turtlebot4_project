"""ROS-independent helpers for recording occupancy-grid snapshots."""

from math import hypot


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
        self.last_x: float | None = None
        self.last_y: float | None = None

    def should_capture(self, now_sec: float, x: float, y: float) -> bool:
        if self.last_time_sec is None:
            return True

        elapsed = max(0.0, now_sec - self.last_time_sec)
        if elapsed < self.min_interval_sec:
            return False

        moved = hypot(x - self.last_x, y - self.last_y)
        distance_due = (
            self.distance_interval_m > 0.0
            and moved >= self.distance_interval_m
        )
        time_due = (
            self.time_interval_sec > 0.0
            and elapsed >= self.time_interval_sec
        )
        return distance_due or time_due

    def record_capture(self, now_sec: float, x: float, y: float) -> None:
        self.last_time_sec = now_sec
        self.last_x = x
        self.last_y = y


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
