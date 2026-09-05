#!/usr/bin/env python3
"""Segmented correction alternative to local_scan.py.

This frontend reuses the same bounded local ICP implementation as local_scan.py,
but deliberately removes the run-long ``output_anchor_warp`` behavior. Each
local segment owns only a small correction. When the robot travels/turns far
enough, or the correction approaches a safety bound, that correction is faded
smoothly back to identity and a fresh segment starts from zero.

SLAM Toolbox still owns the connected global map, scan history, loop closure,
and pose-graph optimization. Segments here are only short-lived correction
regions; they are not a second global SLAM graph.
"""

import math
from typing import Optional

import numpy as np
import rclpy
from std_msgs.msg import Int32

import local_scan


def _scaled_warp(warp: np.ndarray, scale: float) -> np.ndarray:
    """Interpolate a small SE(2) correction from identity (0) to warp (1)."""
    scale = max(0.0, min(1.0, float(scale)))
    return local_scan._pose_matrix(
        float(warp[0, 2]) * scale,
        float(warp[1, 2]) * scale,
        local_scan._matrix_yaw(warp) * scale,
    )


def _clamp_warp(warp: np.ndarray, max_translation: float, max_rotation: float) -> np.ndarray:
    """Bound the correction that is ever exposed to SLAM Toolbox."""
    translation = np.asarray(warp[:2, 2], dtype=np.float64).copy()
    translation_norm = float(np.linalg.norm(translation))
    if translation_norm > max_translation:
        translation *= max_translation / translation_norm

    yaw = local_scan._matrix_yaw(warp)
    yaw = max(-max_rotation, min(max_rotation, yaw))
    return local_scan._pose_matrix(float(translation[0]), float(translation[1]), yaw)


class SubmapScanFrontend(local_scan.LocalScanWindow):
    """Local ICP whose correction lifetime is limited to one local segment."""

    def __init__(self) -> None:
        super().__init__()

        self.declare_parameter("segment_distance_m", 4.5)
        self.declare_parameter("segment_turn_rad", math.radians(120.0))
        self.declare_parameter("segment_max_translation_correction_m", 0.30)
        self.declare_parameter("segment_max_rotation_correction_rad", math.radians(3.0))
        self.declare_parameter("handoff_scans", 20)
        self.declare_parameter("min_segment_scans", 30)

        self.segment_distance = float(self.get_parameter("segment_distance_m").value)
        self.segment_turn = float(self.get_parameter("segment_turn_rad").value)
        self.segment_max_translation = float(
            self.get_parameter("segment_max_translation_correction_m").value
        )
        self.segment_max_rotation = float(
            self.get_parameter("segment_max_rotation_correction_rad").value
        )
        self.handoff_scans = max(1, int(self.get_parameter("handoff_scans").value))
        self.min_segment_scans = max(1, int(self.get_parameter("min_segment_scans").value))

        if self.segment_max_translation <= 0.0 or self.segment_max_rotation <= 0.0:
            raise ValueError("segment correction bounds must be positive")

        self.segment_pub = self.create_publisher(Int32, "submap_icp/segment_id", 10)

        self.segment_id = 0
        self.segment_scan_count = 0
        self.segment_travel_m = 0.0
        self.segment_turn_accum = 0.0
        self.segment_close_count = 0
        self.icp_attempt_count = 0

        self.handoff_start_warp: Optional[np.ndarray] = None
        self.handoff_step = 0
        self.handoff_reason = ""

        # Deliberately unused in this variant: nothing is accumulated across
        # segments. It remains only because LocalScanWindow owns the base state.
        self.output_anchor_warp = np.eye(3, dtype=np.float64)

        self.get_logger().info(
            "SUBMAP MODE: run-long output_anchor_warp disabled; "
            f"segment_distance={self.segment_distance:.1f} m, "
            f"segment_turn={math.degrees(self.segment_turn):.1f} deg, "
            f"correction_bound={self.segment_max_translation:.2f} m/"
            f"{math.degrees(self.segment_max_rotation):.1f} deg, "
            f"handoff={self.handoff_scans} scans"
        )

    def _start_segment(self, raw_pose: np.ndarray, sampled: np.ndarray) -> None:
        self.segment_id += 1
        self.segment_scan_count = 0
        self.segment_travel_m = 0.0
        self.segment_turn_accum = 0.0

        self.raw_origin_pose = raw_pose.copy()
        self.last_raw_pose = raw_pose.copy()
        self.last_corrected_pose = np.eye(3, dtype=np.float64)
        self.registered_scans.clear()
        self.registered_scans.append(sampled.copy())

        msg = Int32()
        msg.data = self.segment_id
        self.segment_pub.publish(msg)
        self.get_logger().info(f"Started local segment {self.segment_id}")

    def _begin_handoff(self, warp: np.ndarray, reason: str) -> None:
        self.segment_close_count += 1
        self.handoff_start_warp = warp.copy()
        self.handoff_step = 0
        self.handoff_reason = reason

        translation = float(np.linalg.norm(warp[:2, 2]))
        rotation = math.degrees(local_scan._matrix_yaw(warp))
        self.get_logger().warning(
            f"Closing segment {self.segment_id}: {reason}; "
            f"handoff={translation:.3f} m/{rotation:.2f} deg -> identity "
            f"over {self.handoff_scans} scans"
        )

    def _handle_handoff(
        self, scan, raw_pose: np.ndarray, sampled: np.ndarray
    ) -> None:
        assert self.handoff_start_warp is not None

        self.handoff_step += 1
        scale = max(0.0, 1.0 - self.handoff_step / float(self.handoff_scans))
        warp = _scaled_warp(self.handoff_start_warp, scale)
        self._publish_corrected_scan(scan, warp)
        self._publish_diagnostics(scan, False, math.inf, warp)

        if self.handoff_step >= self.handoff_scans:
            previous_reason = self.handoff_reason
            self.handoff_start_warp = None
            self.handoff_step = 0
            self.handoff_reason = ""
            self._start_segment(raw_pose, sampled)
            self.get_logger().info(
                f"Segment handoff complete ({previous_reason}); local correction reset to 0"
            )

    def _scan_callback(self, scan) -> None:
        self.scan_count += 1
        raw_pose = self._lookup_raw_pose(scan)
        if raw_pose is None:
            return

        points = self._scan_points(scan)
        if points.shape[0] < self.min_correspondences:
            self.invalid_scan_drop_count += 1
            if self.raw_origin_pose is None and self.handoff_start_warp is None:
                self.scan_pub.publish(scan)
            return
        sampled = points[:: self.scan_stride]

        if self.handoff_start_warp is not None:
            self._handle_handoff(scan, raw_pose, sampled)
            return

        if self.raw_origin_pose is None:
            self._start_segment(raw_pose, sampled)
            self.scan_pub.publish(scan)
            self._publish_diagnostics(scan, False, math.inf, np.eye(3, dtype=np.float64))
            return

        assert self.last_raw_pose is not None
        assert self.last_corrected_pose is not None

        odom_increment = np.linalg.inv(self.last_raw_pose) @ raw_pose
        self.segment_travel_m += float(np.linalg.norm(odom_increment[:2, 2]))
        self.segment_turn_accum += abs(local_scan._matrix_yaw(odom_increment))
        self.segment_scan_count += 1

        predicted_pose = self.last_corrected_pose @ odom_increment
        target = self._local_map()
        self.icp_attempt_count += 1
        corrected_pose, accepted, rmse, match_count = self._icp(
            sampled, predicted_pose, target
        )
        if accepted:
            self.accepted_count += 1

        raw_relative_pose = np.linalg.inv(self.raw_origin_pose) @ raw_pose
        local_warp = np.linalg.inv(raw_relative_pose) @ corrected_pose
        local_translation = float(np.linalg.norm(local_warp[:2, 2]))
        local_rotation = abs(local_scan._matrix_yaw(local_warp))

        correction_limit = (
            local_translation >= self.segment_max_translation
            or local_rotation >= self.segment_max_rotation
        )
        distance_limit = (
            self.segment_scan_count >= self.min_segment_scans
            and self.segment_travel_m >= self.segment_distance
        )
        turn_limit = (
            self.segment_scan_count >= self.min_segment_scans
            and self.segment_turn_accum >= self.segment_turn
        )

        reason: Optional[str] = None
        if correction_limit:
            reason = (
                f"correction limit {local_translation:.3f} m/"
                f"{math.degrees(local_scan._matrix_yaw(local_warp)):.2f} deg"
            )
        elif distance_limit:
            reason = f"travel limit {self.segment_travel_m:.2f} m"
        elif turn_limit:
            reason = f"turn limit {math.degrees(self.segment_turn_accum):.1f} deg"

        if reason is not None:
            safe_warp = _clamp_warp(
                local_warp,
                self.segment_max_translation,
                self.segment_max_rotation,
            )
            self._publish_corrected_scan(scan, safe_warp)
            self._publish_diagnostics(scan, accepted, rmse, safe_warp)
            self._begin_handoff(safe_warp, reason)
            return

        self.registered_scans.append(local_scan._transform_points(corrected_pose, sampled))
        self.last_raw_pose = raw_pose
        self.last_corrected_pose = corrected_pose

        self._publish_corrected_scan(scan, local_warp)
        self._publish_diagnostics(scan, accepted, rmse, local_warp)

        if self.scan_count % 50 == 0:
            acceptance_rate = self.accepted_count / max(1, self.icp_attempt_count)
            self.get_logger().info(
                "Submap ICP: "
                f"segment={self.segment_id} scans={self.segment_scan_count} "
                f"accepted={accepted} matches={match_count} rmse={rmse:.3f} m "
                f"rate={acceptance_rate:.2f} "
                f"travel={self.segment_travel_m:.2f} m "
                f"turn={math.degrees(self.segment_turn_accum):.1f} deg "
                f"local_correction={local_translation:.3f} m/"
                f"{math.degrees(local_scan._matrix_yaw(local_warp)):.2f} deg "
                f"closed_segments={self.segment_close_count} "
                f"tf_drops={self.tf_failure_count} "
                f"invalid_drops={self.invalid_scan_drop_count}"
            )


def main() -> None:
    rclpy.init()
    node = SubmapScanFrontend()
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
