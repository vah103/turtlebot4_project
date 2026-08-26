#!/usr/bin/env python3
"""Hospital runtime wrapper for the pinned MapEx nearest policy.

The policy itself remains in ``mapex_nearest_ros.py`` and is pinned to the
published castacks/MapEx nearest semantics.  This wrapper adds only Hospital
runtime diagnostics and explicit startup-failure semantics so a run with no
usable frontier is never mislabeled as successful exploration completion.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np
import rclpy
from std_msgs.msg import String

from mapex_nearest_ros import (
    COMPLETION_STARTUP_GRACE_S,
    MIN_FRONTIER_DISTANCE_M,
    REGION_SIZE_THRESHOLD,
    MapExNearestROS,
)


FAILURE_TOPIC = "/exploration_failed"


def region_sizes_8(frontier: np.ndarray) -> list[int]:
    """Return 8-connected frontier component sizes for diagnostics only."""
    h, w = frontier.shape
    visited = np.zeros((h, w), dtype=bool)
    sizes: list[int] = []
    neighbours = (
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1),             (0, 1),
        (1, -1),  (1, 0),   (1, 1),
    )
    for row, col in np.argwhere(frontier):
        row = int(row)
        col = int(col)
        if visited[row, col]:
            continue
        queue = deque([(row, col)])
        visited[row, col] = True
        size = 0
        while queue:
            rr, cc = queue.popleft()
            size += 1
            for dr, dc in neighbours:
                nr, nc = rr + dr, cc + dc
                if (
                    0 <= nr < h
                    and 0 <= nc < w
                    and frontier[nr, nc]
                    and not visited[nr, nc]
                ):
                    visited[nr, nc] = True
                    queue.append((nr, nc))
        sizes.append(size)
    return sizes


class HospitalMapExNearestROS(MapExNearestROS):
    """Pinned MapEx nearest policy plus Hospital-only audit/failure handling."""

    def __init__(self) -> None:
        super().__init__()
        self.failure_pub = self.create_publisher(String, FAILURE_TOPIC, 10)
        self._last_diag_signature: tuple | None = None
        self._last_diag: dict[str, int | float | None] = {}
        self._startup_exhausted_since: float | None = None
        self._startup_failure_sent = False
        self.get_logger().info(
            "Hospital audit wrapper active: startup with no usable MapEx frontier "
            "is recorded as policy failure, not exploration completion"
        )

    def _diagnostics(self, msg, robot_xy) -> dict[str, int | float | None]:
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        frontier = self._frontier_mask(grid)
        sizes = region_sizes_8(frontier)
        centers = self._region_representatives(frontier)
        robot_row, robot_col = self._world_to_grid_float(
            robot_xy[0], robot_xy[1], msg
        )
        res = float(msg.info.resolution)
        distances = [
            math.hypot(row - robot_row, col - robot_col) * res
            for row, col in centers
        ]
        return {
            "free_cells": int((grid == 0).sum()),
            "unknown_cells": int((grid < 0).sum()),
            "occupied_or_probabilistic_cells": int((grid > 0).sum()),
            "frontier_cells": int(frontier.sum()),
            "region_count": len(sizes),
            "max_region_size": max(sizes) if sizes else 0,
            "large_region_count": sum(
                size > REGION_SIZE_THRESHOLD for size in sizes
            ),
            "center_count": len(centers),
            "centers_at_least_1m": sum(
                distance >= MIN_FRONTIER_DISTANCE_M for distance in distances
            ),
            "min_center_distance_m": min(distances) if distances else None,
            "max_center_distance_m": max(distances) if distances else None,
        }

    def _compute_candidates(self, msg, robot_xy):
        candidates = super()._compute_candidates(msg, robot_xy)
        diag = self._diagnostics(msg, robot_xy)
        diag["usable_candidate_count"] = len(candidates)
        signature = tuple(diag.items())
        if signature != self._last_diag_signature:
            self._last_diag_signature = signature
            self._last_diag = diag
            self.get_logger().warning(
                "MapEx frontier audit: "
                f"free={diag['free_cells']}, unknown={diag['unknown_cells']}, "
                f"frontier_cells={diag['frontier_cells']}, "
                f"regions={diag['region_count']}, "
                f"max_region={diag['max_region_size']}, "
                f"large_regions(>10)={diag['large_region_count']}, "
                f"centers>=1m={diag['centers_at_least_1m']}, "
                f"usable_candidates={diag['usable_candidate_count']}, "
                f"min_center_dist={diag['min_center_distance_m']}"
            )
        return candidates

    def _startup_failure_reason(self) -> str:
        diag = self._last_diag
        if not diag:
            return "mapex_nearest_startup_no_diagnostics"
        if int(diag.get("frontier_cells", 0) or 0) == 0:
            return "mapex_nearest_startup_no_frontier_cells"
        if int(diag.get("large_region_count", 0) or 0) == 0:
            return "mapex_nearest_startup_no_region_larger_than_10_cells"
        if int(diag.get("centers_at_least_1m", 0) or 0) == 0:
            return "mapex_nearest_startup_all_frontier_centers_within_1m"
        if int(diag.get("usable_candidate_count", 0) or 0) == 0:
            return "mapex_nearest_startup_all_candidates_suppressed"
        return "mapex_nearest_startup_no_nav2_reachable_frontier"

    def _observe_exhausted(self) -> None:
        # In the published MapEx loop, absence of a usable frontier before the
        # mission ever starts is a mission failure, not successful completion.
        if not self.started:
            now = self._now_sec()
            if self._startup_exhausted_since is None:
                self._startup_exhausted_since = now
                return
            if now - self._startup_exhausted_since < COMPLETION_STARTUP_GRACE_S:
                return
            if not self._startup_failure_sent:
                self._startup_failure_sent = True
                reason = self._startup_failure_reason()
                msg = String()
                msg.data = reason
                self.failure_pub.publish(msg)
                self.get_logger().error(
                    "MapEx-nearest startup policy failure: " + reason
                )
                self.complete = True
                self.shutdown_timer = self.create_timer(1.0, self._shutdown)
            return

        self._startup_exhausted_since = None
        super()._observe_exhausted()

    def _on_plan_result(self, future, candidate, frontier_xy) -> None:
        # A planner-valid frontier means the mission has genuinely started; the
        # base class will set ``started`` when it publishes the selected path.
        super()._on_plan_result(future, candidate, frontier_xy)
        if self.started:
            self._startup_exhausted_since = None


def main() -> int:
    rclpy.init()
    node = HospitalMapExNearestROS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
