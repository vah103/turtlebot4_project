#!/usr/bin/env python3
"""Hospital runtime wrapper for the pinned MapEx nearest policy.

The policy itself remains in ``mapex_nearest_ros.py`` and is pinned to the
published castacks/MapEx nearest semantics. This wrapper adds only Hospital
runtime diagnostics, visualization, and explicit startup-failure semantics so
a run with no usable frontier is never mislabeled as successful exploration
completion.

Important semantic correction:
MapEx's ``cur_pose_dist_threshold_m`` is used when checking whether an already
locked frontier remains valid. It is NOT a global minimum-distance filter for
newly detected frontier candidates. The Hospital adapter therefore keeps all
new region centers (>10 cells) in the nearest ranking, regardless of whether
they are initially within 1 m of the robot.

Visualization is diagnostic only and does not alter frontier detection,
ranking, planning validation, or navigation execution. The node publishes
``/frontier_markers`` as a MarkerArray compatible with the repository's
``hospital_exploration.rviz`` configuration. The base adapter already publishes
``/frontier_selected_path``; the Hospital stack already publishes ``/map_cloud``.
"""

from __future__ import annotations

from collections import deque
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from mapex_nearest_ros import (
    COMPLETION_STARTUP_GRACE_S,
    MIN_FRONTIER_DISTANCE_M,
    REGION_SIZE_THRESHOLD,
    FrontierCandidate,
    MapExNearestROS,
)


FAILURE_TOPIC = "/exploration_failed"
MARKER_TOPIC = "/frontier_markers"


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
    """Pinned MapEx nearest policy plus Hospital-only audit/visualization."""

    def __init__(self) -> None:
        super().__init__()
        self.failure_pub = self.create_publisher(String, FAILURE_TOPIC, 10)

        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, MARKER_TOPIC, marker_qos
        )

        self._last_diag_signature: tuple | None = None
        self._last_diag: dict[str, int | float | None] = {}
        self._startup_exhausted_since: float | None = None
        self._startup_failure_sent = False
        self._viz_checking_candidate: FrontierCandidate | None = None

        self.get_logger().info(
            "Hospital audit wrapper active: startup with no usable MapEx frontier "
            "is recorded as policy failure, not exploration completion"
        )
        self.get_logger().info(
            "MapEx semantic correction active: the 1.0 m cur_pose threshold is "
            "diagnostic/locked-frontier validity only; new candidates are NOT "
            "filtered by distance"
        )
        self.get_logger().info(
            "RViz visualization enabled by default: /map_cloud, "
            "/frontier_selected_path, /frontier_markers"
        )

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _cell_point(self, row: int, col: int, msg, z: float) -> Point:
        x, y = self._cell_to_world(row, col, msg)
        return self._point(x, y, z)

    def _publish_markers(
        self,
        msg,
        *,
        candidates: list[FrontierCandidate] | None = None,
    ) -> None:
        """Publish MapEx frontier diagnostics without changing policy state."""
        if msg is None:
            return

        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        frontier = self._frontier_mask(grid)
        centers = self._region_representatives(frontier)
        shown_candidates = (
            list(candidates)
            if candidates is not None
            else list(self.current_candidates)
        )

        frame_id = msg.header.frame_id or "map"
        resolution = max(float(msg.info.resolution), 0.01)
        stamp = self.get_clock().now().to_msg()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL

        frontier_cells = Marker()
        frontier_cells.header.frame_id = frame_id
        frontier_cells.header.stamp = stamp
        frontier_cells.ns = "mapex_frontier_cells"
        frontier_cells.id = 0
        frontier_cells.type = Marker.POINTS
        frontier_cells.action = Marker.ADD
        frontier_cells.pose.orientation.w = 1.0
        frontier_cells.scale.x = max(resolution * 0.8, 0.03)
        frontier_cells.scale.y = max(resolution * 0.8, 0.03)
        frontier_cells.color.r = 0.0
        frontier_cells.color.g = 0.8
        frontier_cells.color.b = 1.0
        frontier_cells.color.a = 0.9
        frontier_cells.points = [
            self._cell_point(int(row), int(col), msg, 0.05)
            for row, col in np.argwhere(frontier)
        ]

        region_centers = Marker()
        region_centers.header.frame_id = frame_id
        region_centers.header.stamp = stamp
        region_centers.ns = "mapex_region_centers"
        region_centers.id = 1
        region_centers.type = Marker.SPHERE_LIST
        region_centers.action = Marker.ADD
        region_centers.pose.orientation.w = 1.0
        center_scale = max(resolution * 3.0, 0.15)
        region_centers.scale.x = center_scale
        region_centers.scale.y = center_scale
        region_centers.scale.z = center_scale
        region_centers.color.r = 1.0
        region_centers.color.g = 0.5
        region_centers.color.b = 0.0
        region_centers.color.a = 0.95
        region_centers.points = [
            self._cell_point(row, col, msg, 0.10) for row, col in centers
        ]

        usable_candidates = Marker()
        usable_candidates.header.frame_id = frame_id
        usable_candidates.header.stamp = stamp
        usable_candidates.ns = "mapex_usable_candidates"
        usable_candidates.id = 2
        usable_candidates.type = Marker.SPHERE_LIST
        usable_candidates.action = Marker.ADD
        usable_candidates.pose.orientation.w = 1.0
        candidate_scale = max(resolution * 3.8, 0.19)
        usable_candidates.scale.x = candidate_scale
        usable_candidates.scale.y = candidate_scale
        usable_candidates.scale.z = candidate_scale
        usable_candidates.color.r = 0.2
        usable_candidates.color.g = 1.0
        usable_candidates.color.b = 0.2
        usable_candidates.color.a = 1.0
        usable_candidates.points = [
            self._cell_point(item.row, item.col, msg, 0.14)
            for item in shown_candidates
        ]

        checking = Marker()
        checking.header.frame_id = frame_id
        checking.header.stamp = stamp
        checking.ns = "mapex_planner_candidate_checking"
        checking.id = 3
        checking.type = Marker.SPHERE
        checking.action = Marker.ADD
        checking.pose.orientation.w = 1.0
        checking_scale = max(resolution * 4.5, 0.23)
        checking.scale.x = checking_scale
        checking.scale.y = checking_scale
        checking.scale.z = checking_scale
        checking.color.r = 1.0
        checking.color.g = 1.0
        checking.color.b = 0.1
        checking.color.a = 1.0
        if self._viz_checking_candidate is not None:
            point = self._cell_point(
                self._viz_checking_candidate.row,
                self._viz_checking_candidate.col,
                msg,
                0.18,
            )
            checking.pose.position = point
        else:
            checking.action = Marker.DELETE

        selected = Marker()
        selected.header.frame_id = frame_id
        selected.header.stamp = stamp
        selected.ns = "mapex_selected_frontier"
        selected.id = 4
        selected.type = Marker.SPHERE
        selected.action = Marker.ADD
        selected.pose.orientation.w = 1.0
        selected_scale = max(resolution * 5.5, 0.28)
        selected.scale.x = selected_scale
        selected.scale.y = selected_scale
        selected.scale.z = selected_scale
        selected.color.r = 1.0
        selected.color.g = 0.1
        selected.color.b = 0.1
        selected.color.a = 1.0
        if self.active_frontier_xy is not None:
            selected.pose.position = self._point(
                self.active_frontier_xy[0],
                self.active_frontier_xy[1],
                0.22,
            )
        else:
            selected.action = Marker.DELETE

        marker_array = MarkerArray()
        marker_array.markers = [
            delete_all,
            frontier_cells,
            region_centers,
            usable_candidates,
            checking,
            selected,
        ]
        self.marker_pub.publish(marker_array)

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
            # This is informational only. In official MapEx the 1 m threshold
            # validates an already locked frontier; it does not remove a newly
            # detected candidate before ranking.
            "centers_beyond_locked_validity_distance": sum(
                distance >= MIN_FRONTIER_DISTANCE_M for distance in distances
            ),
            "min_center_distance_m": min(distances) if distances else None,
            "max_center_distance_m": max(distances) if distances else None,
        }

    def _compute_candidates(self, msg, robot_xy):
        """Rank all MapEx region centers; do not apply 1 m as a new-goal filter."""
        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        centers = self._region_representatives(self._frontier_mask(grid))
        robot_row, robot_col = self._world_to_grid_float(
            robot_xy[0], robot_xy[1], msg
        )
        res = float(msg.info.resolution)
        candidates: list[FrontierCandidate] = []
        for row, col in centers:
            distance_cells = math.hypot(row - robot_row, col - robot_col)
            distance_m = distance_cells * res
            x, y = self._cell_to_world(row, col, msg)
            if self._is_execution_suppressed(x, y):
                continue
            candidates.append(
                FrontierCandidate(row, col, distance_cells, distance_m)
            )

        # Official MapEx nearest score: Euclidean distance, ascending.
        candidates.sort(key=lambda item: item.distance_cells)

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
                f"centers={diag['center_count']}, "
                f"centers>=1m(lock-validity-only)="
                f"{diag['centers_beyond_locked_validity_distance']}, "
                f"usable_candidates={diag['usable_candidate_count']}, "
                f"min_center_dist={diag['min_center_distance_m']}"
            )

        self._publish_markers(msg, candidates=candidates)
        return candidates

    def _plan_next_candidate(self) -> None:
        if (
            self.decision_map is not None
            and self.planning_index < len(self.current_candidates)
        ):
            self._viz_checking_candidate = self.current_candidates[
                self.planning_index
            ]
        else:
            self._viz_checking_candidate = None

        super()._plan_next_candidate()
        self._publish_markers(self.decision_map or self.latest_map)

    def _startup_failure_reason(self) -> str:
        diag = self._last_diag
        if not diag:
            return "mapex_nearest_startup_no_diagnostics"
        if int(diag.get("frontier_cells", 0) or 0) == 0:
            return "mapex_nearest_startup_no_frontier_cells"
        if int(diag.get("large_region_count", 0) or 0) == 0:
            return "mapex_nearest_startup_no_region_larger_than_10_cells"
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
        self._viz_checking_candidate = None
        super()._on_plan_result(future, candidate, frontier_xy)
        if self.started:
            self._startup_exhausted_since = None
        self._publish_markers(self.decision_map or self.latest_map)

    def _on_goal_success(self, msg) -> None:
        super()._on_goal_success(msg)
        self._viz_checking_candidate = None
        self._publish_markers(self.latest_map)

    def _on_goal_failure(self, msg) -> None:
        super()._on_goal_failure(msg)
        self._viz_checking_candidate = None
        self._publish_markers(self.latest_map)


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
