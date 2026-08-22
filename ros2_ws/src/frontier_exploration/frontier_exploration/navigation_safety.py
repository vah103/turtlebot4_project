"""ROS-independent helpers for frontier clearance and trapped-robot detection."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, hypot
from typing import Sequence


def circle_is_clear(
    data: Sequence[int],
    width: int,
    height: int,
    center_index: int,
    resolution_m: float,
    radius_m: float,
    occupancy_threshold: int,
    *,
    unknown_is_blocked: bool,
    outside_is_blocked: bool,
) -> bool:
    """Return whether every sampled costmap cell inside a circular area is safe."""
    if width <= 0 or height <= 0 or len(data) < width * height:
        return False
    if not (0 <= center_index < width * height):
        return False

    resolution = max(float(resolution_m), 1e-9)
    radius = max(0.0, float(radius_m))
    threshold = int(occupancy_threshold)
    center_x = center_index % width
    center_y = center_index // width
    radius_cells = max(0, int(ceil(radius / resolution)))

    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if hypot(dx, dy) * resolution > radius + 1e-9:
                continue
            cell_x = center_x + dx
            cell_y = center_y + dy
            if not (0 <= cell_x < width and 0 <= cell_y < height):
                if outside_is_blocked:
                    return False
                continue

            value = int(data[cell_y * width + cell_x])
            if value < 0:
                if unknown_is_blocked:
                    return False
                continue
            if value >= threshold:
                return False

    return True


@dataclass
class TrappedFailureTracker:
    """Detect repeated navigation failures without meaningful translational escape."""

    radius_m: float = 0.20
    failure_limit: int = 3
    anchor_xy: tuple[float, float] | None = None
    failure_count: int = 0
    trapped: bool = False

    def reset(self) -> None:
        self.anchor_xy = None
        self.failure_count = 0
        self.trapped = False

    def record_failure(self, x: float, y: float) -> tuple[int, bool]:
        radius = max(0.0, float(self.radius_m))
        limit = max(1, int(self.failure_limit))
        current = (float(x), float(y))

        if (
            self.anchor_xy is None
            or hypot(current[0] - self.anchor_xy[0], current[1] - self.anchor_xy[1])
            > radius
        ):
            self.anchor_xy = current
            self.failure_count = 1
            self.trapped = self.failure_count >= limit
            return self.failure_count, self.trapped

        self.failure_count += 1
        self.trapped = self.failure_count >= limit
        return self.failure_count, self.trapped
