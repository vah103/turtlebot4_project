#!/usr/bin/env python3
"""Auxiliary nearest-frontier runner for cross-environment smoke tests.

This runner reuses the canonical ``nf_basic.py`` execution stack unchanged and
only adds one geometry fallback for auxiliary worlds such as ``new_room``:

- compute the canonical region representative (frontier cell nearest the region
  arithmetic mean), exactly as ``nf_basic.py`` does;
- if that representative is at least 0.5 m from the robot, keep it unchanged;
- if it is closer than 0.5 m, select the nearest *actual frontier cell in the
  same region* whose robot distance is at least 0.5 m;
- if the region has no such cell, keep the canonical representative, allowing
  the canonical min-distance guard to block it as before.

This is deliberately AUXILIARY/DEBUG behavior. It is not the official Hospital
Nearest Frontier policy and must not be used for hospital_v2 benchmark runs.
"""

import math

import rclpy

from nf_basic import MIN_DISTANCE_THRESHOLD, NearestEuclideanFrontier


class AuxiliaryNearestEuclideanFrontier(NearestEuclideanFrontier):
    """Nearest Frontier with a same-region fallback for near representatives."""

    def __init__(self):
        super().__init__()
        self._last_fallback = None
        self.get_logger().warning(
            "AUXILIARY MODE: near-representative fallback enabled; "
            "do not use this runner for hospital_v2 benchmark runs."
        )

    def representative(self, region):
        """Return canonical representative unless it is inside the 0.5 m guard."""
        row, col = super().representative(region)

        robot = self.robot_position()
        if robot is None:
            return row, col

        robot_x, robot_y = robot
        canonical_x, canonical_y = self.cell_to_world(row, col)
        canonical_distance = math.hypot(
            canonical_x - robot_x,
            canonical_y - robot_y,
        )

        if canonical_distance >= MIN_DISTANCE_THRESHOLD:
            self._last_fallback = None
            return row, col

        eligible_cells = []
        for fallback_row, fallback_col in region:
            x, y = self.cell_to_world(fallback_row, fallback_col)
            distance = math.hypot(x - robot_x, y - robot_y)
            if distance >= MIN_DISTANCE_THRESHOLD:
                eligible_cells.append(
                    (distance, fallback_row, fallback_col, x, y)
                )

        if not eligible_cells:
            return row, col

        distance, fallback_row, fallback_col, x, y = min(
            eligible_cells,
            key=lambda item: (item[0], item[1], item[2]),
        )

        fallback_key = (
            row,
            col,
            fallback_row,
            fallback_col,
        )
        if fallback_key != self._last_fallback:
            self.get_logger().warning(
                "Aux representative fallback: canonical "
                f"({canonical_x:.2f}, {canonical_y:.2f}) at "
                f"{canonical_distance:.2f} m is inside the "
                f"{MIN_DISTANCE_THRESHOLD:.2f} m guard; using same-region "
                f"frontier ({x:.2f}, {y:.2f}) at {distance:.2f} m."
            )
            self._last_fallback = fallback_key

        return fallback_row, fallback_col


def main(args=None):
    rclpy.init(args=args)
    node = AuxiliaryNearestEuclideanFrontier()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
