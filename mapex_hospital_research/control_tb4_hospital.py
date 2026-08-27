#!/usr/bin/env python3
"""Hospital-safe wrapper around control_tb4.py.

Keeps the frontier-generation and nearest-frontier selection logic from
control_tb4.py, but fixes three integration issues for SLAM Toolbox + Nav2:
1. robot pose is read in the `map` frame via TF (map -> base_link);
2. a transient out-of-map pose is not interpreted as exploration completion;
3. shutdown is performed safely without calling rclpy.shutdown() twice.
"""

import logging
import time

import rclpy
from tf2_ros import Buffer, TransformException, TransformListener

import control_tb4 as base

logger = logging.getLogger(__name__)


class HospitalNavigationControl(base.navigationControl):
    def __init__(self):
        # State used by the overridden exploration loop.
        self.tf_buffer = None
        self.tf_listener = None
        self.empty_frontier_streak = 0
        self.empty_frontier_required = 5
        self.exploration_finished = False

        super().__init__()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        logger.info("[INFO] Hospital fix active: robot pose uses TF map -> base_link")

    def odom_callback(self, msg):
        """Keep odometry only as auxiliary data; do not use it as a map-frame pose."""
        self.odom_data = msg
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def robot_pose_in_map(self):
        if self.tf_buffer is None:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
            )
        except TransformException as exc:
            logger.debug(f"Waiting for map -> base_link TF: {exc}")
            return None

        t = transform.transform.translation
        q = transform.transform.rotation
        yaw = base.euler_from_quaternion(q.x, q.y, q.z, q.w)
        return float(t.x), float(t.y), float(yaw)

    def finish_exploration(self):
        if self.exploration_finished:
            return

        self.exploration_finished = True
        logger.info("[INFO] EXPLORATION COMPLETED")

        if self.start_time is not None:
            self.end_time = time.perf_counter()
            self.total_exploration_time = self.end_time - self.start_time
            logger.info(
                f"[INFO] Total Exploration Time: {self.total_exploration_time:.2f} seconds"
            )

        elapsed_time = self.goal_timer.stop(success=True)
        if elapsed_time is not None:
            logger.info(f"[INFO] Exploration time recorded: {elapsed_time:.2f} seconds")

        # Stop rclpy.spin(). main() checks rclpy.ok() before attempting shutdown again.
        if rclpy.ok():
            rclpy.shutdown()

    def exploration_loop(self):
        if self.exploration_finished:
            return

        if not hasattr(self, 'map_data') or not hasattr(self, 'scan_data'):
            logger.debug("Waiting for map and scan data")
            return

        pose = self.robot_pose_in_map()
        if pose is None:
            return

        self.x, self.y, self.yaw = pose

        if self.start_time is None:
            self.start_time = time.perf_counter()
            self.goal_timer.start()
            logger.info("[INFO] Exploration started.")

        column = int((self.x - self.originX) / self.resolution)
        row = int((self.y - self.originY) / self.resolution)

        # Do not let an expanding/repositioned SLAM OccupancyGrid be interpreted
        # as "no frontier". Just skip this sweep and try again on the next map.
        if not (0 <= row < self.height and 0 <= column < self.width):
            logger.warning(
                "Robot map-frame pose is temporarily outside OccupancyGrid bounds: "
                f"x={self.x:.2f}, y={self.y:.2f}, row={row}, col={column}, "
                f"map={self.height}x{self.width}"
            )
            self.empty_frontier_streak = 0
            return

        robot_position = (self.x, self.y)
        groups = base.exploration(
            self.data,
            self.width,
            self.height,
            self.resolution,
            column,
            row,
            self.originX,
            self.originY,
        )

        # base.exploration() should now be in bounds, but guard against any
        # unexpected invalid return so it can never mean "mission complete".
        if groups is None:
            self.empty_frontier_streak = 0
            return

        self.groups = groups
        self.centroids = [g[2] for g in groups if g[2] is not None]

        if len(groups) == 0:
            # If Nav2 is still driving to the current frontier, wait for that
            # goal to finish before deciding that exploration is exhausted.
            if self.current_goal is not None:
                self.empty_frontier_streak = 0
                logger.debug("No frontier in this sweep; current Nav2 goal is still active")
                return

            self.empty_frontier_streak += 1
            logger.info(
                f"[INFO] No frontier candidates: "
                f"{self.empty_frontier_streak}/{self.empty_frontier_required}"
            )
            if self.empty_frontier_streak >= self.empty_frontier_required:
                self.finish_exploration()
            return

        self.empty_frontier_streak = 0

        best_centroid = self.select_best_frontier(groups, robot_position)
        if best_centroid is None:
            logger.info("[INFO] No suitable frontier found")
            return

        goal_x = best_centroid[1] * self.resolution + self.originX
        goal_y = best_centroid[0] * self.resolution + self.originY

        if self.current_goal is None:
            self.send_goal(goal_x, goal_y)
            logger.info(f"[INFO] NEW TARGET ASSIGNED at ({goal_x}, {goal_y})")
            self.kesif = False
            self.current_goal = (goal_x, goal_y)
            self.create_goal_marker(goal_x, goal_y)
            self.publish_goal_markers()
        else:
            logger.debug("[INFO] Continuing with current goal")


def main(args=None):
    rclpy.init(args=args)
    node = HospitalNavigationControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
