#!/usr/bin/env python3
"""Rolling local-window scan registration frontend for SLAM Toolbox.

Debug-only frontend. It never publishes velocity commands. The node uses the
raw odom->laser TF as a motion prediction, aligns each new scan to a rolling
window of recent registered scans with bounded 2-D ICP, then republishes the
small accepted correction as a corrected LaserScan. Global loop closure and
pose-graph optimization remain SLAM Toolbox responsibilities.

To keep this a *local* stabilizer rather than a second global SLAM system, the
rolling ICP state is re-anchored if the correction accumulated since the current
anchor grows beyond a conservative translation/yaw bound. Re-anchoring keeps
the correction already visible at the output instead of snapping back to a raw
scan. If TF is temporarily unavailable, that scan is dropped rather than mixing
an uncorrected raw scan into the corrected output stream.
"""

from collections import deque
import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener


def _pose_matrix(x: float, y: float, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]], dtype=np.float64)


def _matrix_yaw(transform: np.ndarray) -> float:
    return math.atan2(transform[1, 0], transform[0, 0])


def _transform_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    if points.size == 0:
        return points.copy()
    return points @ transform[:2, :2].T + transform[:2, 2]


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    if points.size == 0 or voxel_size <= 0.0:
        return points
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, first_indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first_indices)]


class LocalScanWindow(Node):
    """Bounded scan-to-local-window registration in front of SLAM Toolbox."""

    def __init__(self) -> None:
        super().__init__("slam_local_scan_window")

        self.declare_parameter("input_scan_topic", "/scan")
        self.declare_parameter("output_scan_topic", "/scan_local_window")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("window_scans", 20)
        self.declare_parameter("scan_stride", 3)
        self.declare_parameter("voxel_size_m", 0.07)
        self.declare_parameter("max_correspondence_distance_m", 0.25)
        self.declare_parameter("max_iterations", 6)
        self.declare_parameter("min_correspondences", 25)
        self.declare_parameter("max_rmse_m", 0.12)
        self.declare_parameter("max_translation_correction_m", 0.20)
        self.declare_parameter("max_rotation_correction_rad", math.radians(8.0))
        self.declare_parameter("max_iteration_translation_m", 0.08)
        self.declare_parameter("max_iteration_rotation_rad", math.radians(3.0))
        self.declare_parameter("max_total_translation_correction_m", 0.50)
        self.declare_parameter("max_total_rotation_correction_rad", math.radians(15.0))
        self.declare_parameter("tf_timeout_s", 0.10)

        self.input_scan_topic = str(self.get_parameter("input_scan_topic").value)
        self.output_scan_topic = str(self.get_parameter("output_scan_topic").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.window_scans = max(2, int(self.get_parameter("window_scans").value))
        self.scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self.voxel_size = float(self.get_parameter("voxel_size_m").value)
        self.max_correspondence = float(
            self.get_parameter("max_correspondence_distance_m").value
        )
        self.max_iterations = max(1, int(self.get_parameter("max_iterations").value))
        self.min_correspondences = max(
            3, int(self.get_parameter("min_correspondences").value)
        )
        self.max_rmse = float(self.get_parameter("max_rmse_m").value)
        self.max_translation_correction = float(
            self.get_parameter("max_translation_correction_m").value
        )
        self.max_rotation_correction = float(
            self.get_parameter("max_rotation_correction_rad").value
        )
        self.max_iteration_translation = float(
            self.get_parameter("max_iteration_translation_m").value
        )
        self.max_iteration_rotation = float(
            self.get_parameter("max_iteration_rotation_rad").value
        )
        self.max_total_translation_correction = float(
            self.get_parameter("max_total_translation_correction_m").value
        )
        self.max_total_rotation_correction = float(
            self.get_parameter("max_total_rotation_correction_rad").value
        )
        self.tf_timeout = float(self.get_parameter("tf_timeout_s").value)

        if self.input_scan_topic == self.output_scan_topic:
            raise ValueError("input_scan_topic and output_scan_topic must differ")
        if self.max_correspondence <= 0.0:
            raise ValueError("max_correspondence_distance_m must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_pub = self.create_publisher(
            LaserScan, self.output_scan_topic, qos_profile_sensor_data
        )
        self.accepted_pub = self.create_publisher(Bool, "local_window_icp/accepted", 10)
        self.rmse_pub = self.create_publisher(Float32, "local_window_icp/rmse_m", 10)
        self.correction_pub = self.create_publisher(
            Vector3Stamped, "local_window_icp/correction", 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.input_scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        self.registered_scans = deque(maxlen=self.window_scans)
        self.raw_origin_pose: Optional[np.ndarray] = None
        self.last_raw_pose: Optional[np.ndarray] = None
        self.last_corrected_pose: Optional[np.ndarray] = None

        # Correction already exposed to SLAM before the current local window.
        # The local ICP starts from identity after every re-anchor, while this
        # handoff transform preserves output continuity across that re-anchor.
        self.output_anchor_warp = np.eye(3, dtype=np.float64)

        self.scan_count = 0
        self.accepted_count = 0
        self.tf_failure_count = 0
        self.invalid_scan_drop_count = 0
        self.reset_count = 0  # Kept for log compatibility; now counts re-anchors.

        self.get_logger().info(
            "Local-window SLAM frontend active: "
            f"{self.input_scan_topic} -> {self.output_scan_topic}, "
            f"window={self.window_scans} scans, stride={self.scan_stride}, "
            "continuous_reanchor=True"
        )

    def _lookup_raw_pose(self, scan: LaserScan) -> Optional[np.ndarray]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                scan.header.frame_id,
                Time.from_msg(scan.header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self.tf_failure_count += 1
            if self.tf_failure_count <= 3 or self.tf_failure_count % 50 == 0:
                self.get_logger().warning(
                    "TF unavailable for local-window frontend; dropping scan to "
                    "preserve corrected-scan continuity "
                    f"({exc})"
                )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        return _pose_matrix(translation.x, translation.y, yaw)

    def _scan_points(self, scan: LaserScan) -> np.ndarray:
        ranges = np.asarray(scan.ranges, dtype=np.float64)
        if ranges.size == 0:
            return np.empty((0, 2), dtype=np.float64)

        angles = (
            float(scan.angle_min)
            + np.arange(ranges.size, dtype=np.float64) * float(scan.angle_increment)
        )
        valid = np.isfinite(ranges)
        valid &= ranges >= float(scan.range_min)
        valid &= ranges <= float(scan.range_max)
        if not np.any(valid):
            return np.empty((0, 2), dtype=np.float64)

        selected_ranges = ranges[valid]
        selected_angles = angles[valid]
        return np.column_stack(
            (
                selected_ranges * np.cos(selected_angles),
                selected_ranges * np.sin(selected_angles),
            )
        )

    def _reset_window(
        self,
        raw_pose: np.ndarray,
        sampled: np.ndarray,
        output_anchor_warp: Optional[np.ndarray] = None,
    ) -> None:
        if output_anchor_warp is not None:
            self.output_anchor_warp = output_anchor_warp.copy()
        self.raw_origin_pose = raw_pose.copy()
        self.last_raw_pose = raw_pose.copy()
        self.last_corrected_pose = np.eye(3, dtype=np.float64)
        self.registered_scans.clear()
        self.registered_scans.append(sampled.copy())

    def _local_map(self) -> np.ndarray:
        if not self.registered_scans:
            return np.empty((0, 2), dtype=np.float64)
        merged = np.vstack(tuple(self.registered_scans))
        return _voxel_downsample(merged, self.voxel_size)

    def _build_hash(self, target: np.ndarray) -> Dict[Tuple[int, int], List[int]]:
        inv_cell = 1.0 / self.max_correspondence
        buckets: Dict[Tuple[int, int], List[int]] = {}
        keys = np.floor(target * inv_cell).astype(np.int64)
        for index, key in enumerate(keys):
            bucket_key = (int(key[0]), int(key[1]))
            buckets.setdefault(bucket_key, []).append(index)
        return buckets

    def _correspondences(
        self,
        source: np.ndarray,
        target: np.ndarray,
        buckets: Dict[Tuple[int, int], List[int]],
    ) -> Tuple[np.ndarray, np.ndarray]:
        if source.size == 0 or target.size == 0:
            empty = np.empty((0, 2), dtype=np.float64)
            return empty, empty

        inv_cell = 1.0 / self.max_correspondence
        max_d2 = self.max_correspondence * self.max_correspondence
        source_matches: List[np.ndarray] = []
        target_matches: List[np.ndarray] = []

        for point in source:
            key = np.floor(point * inv_cell).astype(np.int64)
            best_index = -1
            best_d2 = max_d2
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    candidates = buckets.get((int(key[0]) + dx, int(key[1]) + dy))
                    if not candidates:
                        continue
                    candidate_points = target[candidates]
                    deltas = candidate_points - point
                    distances = np.einsum("ij,ij->i", deltas, deltas)
                    local_index = int(np.argmin(distances))
                    distance = float(distances[local_index])
                    if distance < best_d2:
                        best_d2 = distance
                        best_index = candidates[local_index]

            if best_index >= 0:
                source_matches.append(point)
                target_matches.append(target[best_index])

        if not source_matches:
            empty = np.empty((0, 2), dtype=np.float64)
            return empty, empty

        return (
            np.asarray(source_matches, dtype=np.float64),
            np.asarray(target_matches, dtype=np.float64),
        )

    def _rigid_fit(self, source: np.ndarray, target: np.ndarray) -> np.ndarray:
        source_center = np.mean(source, axis=0)
        target_center = np.mean(target, axis=0)
        source_zero = source - source_center
        target_zero = target - target_center

        covariance = source_zero.T @ target_zero
        u, _, vt = np.linalg.svd(covariance)
        rotation = vt.T @ u.T
        if np.linalg.det(rotation) < 0.0:
            vt[-1, :] *= -1.0
            rotation = vt.T @ u.T

        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        yaw = max(-self.max_iteration_rotation, min(self.max_iteration_rotation, yaw))
        rotation = np.array(
            [
                [math.cos(yaw), -math.sin(yaw)],
                [math.sin(yaw), math.cos(yaw)],
            ],
            dtype=np.float64,
        )

        translation = target_center - rotation @ source_center
        translation_norm = float(np.linalg.norm(translation))
        if translation_norm > self.max_iteration_translation:
            translation *= self.max_iteration_translation / translation_norm

        delta = np.eye(3, dtype=np.float64)
        delta[:2, :2] = rotation
        delta[:2, 2] = translation
        return delta

    def _icp(
        self,
        current_points: np.ndarray,
        predicted_pose: np.ndarray,
        target: np.ndarray,
    ) -> Tuple[np.ndarray, bool, float, int]:
        if (
            current_points.shape[0] < self.min_correspondences
            or target.shape[0] < self.min_correspondences
        ):
            return predicted_pose, False, math.inf, 0

        buckets = self._build_hash(target)
        estimate = predicted_pose.copy()

        for _ in range(self.max_iterations):
            transformed = _transform_points(estimate, current_points)
            source_matches, target_matches = self._correspondences(
                transformed, target, buckets
            )
            if source_matches.shape[0] < self.min_correspondences:
                return predicted_pose, False, math.inf, source_matches.shape[0]

            delta = self._rigid_fit(source_matches, target_matches)
            estimate = delta @ estimate
            if (
                float(np.linalg.norm(delta[:2, 2])) < 0.002
                and abs(_matrix_yaw(delta)) < math.radians(0.15)
            ):
                break

        transformed = _transform_points(estimate, current_points)
        source_matches, target_matches = self._correspondences(
            transformed, target, buckets
        )
        match_count = source_matches.shape[0]
        if match_count < self.min_correspondences:
            return predicted_pose, False, math.inf, match_count

        residual = source_matches - target_matches
        rmse = float(math.sqrt(np.mean(np.einsum("ij,ij->i", residual, residual))))

        total_delta = estimate @ np.linalg.inv(predicted_pose)
        translation_correction = float(np.linalg.norm(total_delta[:2, 2]))
        rotation_correction = abs(_matrix_yaw(total_delta))
        accepted = (
            rmse <= self.max_rmse
            and translation_correction <= self.max_translation_correction
            and rotation_correction <= self.max_rotation_correction
        )
        if not accepted:
            return predicted_pose, False, rmse, match_count
        return estimate, True, rmse, match_count

    def _publish_corrected_scan(self, scan: LaserScan, warp: np.ndarray) -> None:
        points = self._scan_points(scan)
        if points.size == 0 or len(scan.ranges) == 0 or scan.angle_increment == 0.0:
            return

        corrected_points = _transform_points(warp, points)
        angles = np.arctan2(corrected_points[:, 1], corrected_points[:, 0])
        ranges = np.linalg.norm(corrected_points, axis=1)
        output_ranges = np.full(len(scan.ranges), np.inf, dtype=np.float64)
        indices = np.rint(
            (angles - float(scan.angle_min)) / float(scan.angle_increment)
        ).astype(np.int64)
        valid = (indices >= 0) & (indices < len(scan.ranges))
        valid &= ranges >= float(scan.range_min)
        valid &= ranges <= float(scan.range_max)

        for index, distance in zip(indices[valid], ranges[valid]):
            if distance < output_ranges[index]:
                output_ranges[index] = distance

        output = LaserScan()
        output.header = scan.header
        output.angle_min = scan.angle_min
        output.angle_max = scan.angle_max
        output.angle_increment = scan.angle_increment
        output.time_increment = scan.time_increment
        output.scan_time = scan.scan_time
        output.range_min = scan.range_min
        output.range_max = scan.range_max
        output.ranges = output_ranges.astype(np.float32).tolist()
        output.intensities = []
        self.scan_pub.publish(output)

    def _publish_diagnostics(
        self, scan: LaserScan, accepted: bool, rmse: float, warp: np.ndarray
    ) -> None:
        accepted_msg = Bool()
        accepted_msg.data = accepted
        self.accepted_pub.publish(accepted_msg)

        rmse_msg = Float32()
        rmse_msg.data = float(rmse)
        self.rmse_pub.publish(rmse_msg)

        correction = Vector3Stamped()
        correction.header = scan.header
        correction.vector.x = float(warp[0, 2])
        correction.vector.y = float(warp[1, 2])
        correction.vector.z = float(_matrix_yaw(warp))
        self.correction_pub.publish(correction)

    def _scan_callback(self, scan: LaserScan) -> None:
        self.scan_count += 1
        raw_pose = self._lookup_raw_pose(scan)
        if raw_pose is None:
            # Do not insert an uncorrected scan in the middle of a corrected
            # stream. A single missing scan is safer than a discontinuity.
            return

        points = self._scan_points(scan)
        if points.shape[0] < self.min_correspondences:
            self.invalid_scan_drop_count += 1
            if self.raw_origin_pose is None:
                # Before the frontend has an anchor there is no corrected
                # stream to preserve, so passing the raw scan is harmless.
                self.scan_pub.publish(scan)
            return
        sampled = points[:: self.scan_stride]

        if self.raw_origin_pose is None:
            self.output_anchor_warp = np.eye(3, dtype=np.float64)
            self._reset_window(raw_pose, sampled)
            self.scan_pub.publish(scan)
            self._publish_diagnostics(
                scan, False, math.inf, np.eye(3, dtype=np.float64)
            )
            return

        assert self.last_raw_pose is not None
        assert self.last_corrected_pose is not None
        assert self.raw_origin_pose is not None

        odom_increment = np.linalg.inv(self.last_raw_pose) @ raw_pose
        predicted_pose = self.last_corrected_pose @ odom_increment
        target = self._local_map()
        corrected_pose, accepted, rmse, match_count = self._icp(
            sampled, predicted_pose, target
        )
        if accepted:
            self.accepted_count += 1

        raw_relative_pose = np.linalg.inv(self.raw_origin_pose) @ raw_pose
        local_warp = np.linalg.inv(raw_relative_pose) @ corrected_pose
        local_translation = float(np.linalg.norm(local_warp[:2, 2]))
        local_rotation = abs(_matrix_yaw(local_warp))

        # Apply the correction accumulated in previous windows first, then the
        # small correction produced by the current local window. At re-anchor
        # this exact output transform becomes the new handoff transform, so the
        # scan stream does not jump back to identity/raw.
        output_warp = self.output_anchor_warp @ local_warp
        output_translation = float(np.linalg.norm(output_warp[:2, 2]))
        output_rotation = _matrix_yaw(output_warp)

        if (
            local_translation > self.max_total_translation_correction
            or local_rotation > self.max_total_rotation_correction
        ):
            self.reset_count += 1
            self.get_logger().warning(
                "Local-window correction exceeded safety bound; re-anchoring "
                "without raw-scan jump "
                f"(local={local_translation:.3f} m/"
                f"{math.degrees(_matrix_yaw(local_warp)):.2f} deg, "
                f"output={output_translation:.3f} m/"
                f"{math.degrees(output_rotation):.2f} deg, "
                f"reanchors={self.reset_count})"
            )

            # Preserve the transform already visible to SLAM, but reset only
            # the *local* ICP coordinate system. The triggering scan is still
            # published with output_warp, so there is no identity/raw spike.
            self._reset_window(raw_pose, sampled, output_anchor_warp=output_warp)
            self._publish_corrected_scan(scan, output_warp)
            self._publish_diagnostics(scan, accepted, rmse, output_warp)
            return

        self.registered_scans.append(_transform_points(corrected_pose, sampled))
        self.last_raw_pose = raw_pose
        self.last_corrected_pose = corrected_pose

        self._publish_corrected_scan(scan, output_warp)
        self._publish_diagnostics(scan, accepted, rmse, output_warp)

        if self.scan_count % 50 == 0:
            acceptance_rate = self.accepted_count / max(1, self.scan_count - 1)
            self.get_logger().info(
                "Local-window ICP: "
                f"accepted={accepted} matches={match_count} rmse={rmse:.3f} m "
                f"rate={acceptance_rate:.2f} reanchors={self.reset_count} "
                f"local_correction={local_translation:.3f} m/"
                f"{math.degrees(_matrix_yaw(local_warp)):.2f} deg "
                f"output_correction={output_translation:.3f} m/"
                f"{math.degrees(output_rotation):.2f} deg "
                f"tf_drops={self.tf_failure_count} "
                f"invalid_drops={self.invalid_scan_drop_count}"
            )


def main() -> None:
    rclpy.init()
    node = LocalScanWindow()
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
