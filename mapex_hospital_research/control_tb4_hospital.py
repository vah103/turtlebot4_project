#!/usr/bin/env python3
"""Hospital-safe wrapper around control_tb4.py.

Keeps the frontier-generation and nearest-frontier selection logic from
control_tb4.py, but fixes Hospital integration issues for SLAM Toolbox + Nav2:
1. robot pose is read in the `map` frame via TF (map -> base_link);
2. a transient out-of-map pose is not interpreted as exploration completion;
3. shutdown is performed safely without calling rclpy.shutdown() twice;
4. failed/rejected Nav2 goals are temporarily blacklisted so the controller
   does not immediately select the same unreachable frontier again.
"""

import logging
import math
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

        # Failed-goal blacklist is stored in map-frame metric coordinates.
        # Group IDs are intentionally not used because frontier grouping can
        # change whenever the SLAM map changes.
        self.failed_goal_blacklist = []
        self.failed_goal_radius_m = 0.75
        self.failed_goal_cooldown_s = 60.0

        super().__init__()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        logger.info("[INFO] Hospital fix active: robot pose uses TF map -> base_link")
        logger.info(
            "[INFO] Failed frontier goals are blacklisted for "
            f"{self.failed_goal_cooldown_s:.0f}s within "
            f"{self.failed_goal_radius_m:.2f}m"
        )

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

    def _now_sim_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _prune_failed_goals(self):
        now_s = self._now_sim_s()
        self.failed_goal_blacklist = [
            item for item in self.failed_goal_blacklist if item[2] > now_s
        ]

    def _blacklist_goal(self, goal, reason):
        if goal is None:
            return

        goal_x, goal_y = float(goal[0]), float(goal[1])
        now_s = self._now_sim_s()
        expires_s = now_s + self.failed_goal_cooldown_s

        # Refresh an existing nearby blacklist entry instead of adding duplicates.
        refreshed = False
        for idx, (x, y, _expiry) in enumerate(self.failed_goal_blacklist):
            if math.hypot(goal_x - x, goal_y - y) <= self.failed_goal_radius_m:
                self.failed_goal_blacklist[idx] = (goal_x, goal_y, expires_s)
                refreshed = True
                break
        if not refreshed:
            self.failed_goal_blacklist.append((goal_x, goal_y, expires_s))

        logger.warning(
            f"[INFO] Blacklisting failed frontier ({goal_x:.2f}, {goal_y:.2f}) "
            f"for {self.failed_goal_cooldown_s:.0f}s: {reason}"
        )

    def _is_blacklisted(self, x, y):
        self._prune_failed_goals()
        for failed_x, failed_y, _expiry in self.failed_goal_blacklist:
            if math.hypot(x - failed_x, y - failed_y) <= self.failed_goal_radius_m:
                return True
        return False

    def select_best_frontier(self, groups, robot_position):
        """Use the original nearest-frontier rule after removing failed goals."""
        filtered_groups = []
        for group in groups:
            group_id, _group_cells, centroid = group
            if centroid is None:
                continue

            centroid_x = centroid[1] * self.resolution + self.originX
            centroid_y = centroid[0] * self.resolution + self.originY
            if self._is_blacklisted(centroid_x, centroid_y):
                logger.info(
                    f"[INFO] Skipping blacklisted frontier {group_id} at "
                    f"({centroid_x:.2f}, {centroid_y:.2f})"
                )
                continue
            filtered_groups.append(group)

        return super().select_best_frontier(filtered_groups, robot_position)

    def goal_response_callback(self, future):
        """Blacklist a goal if Nav2 rejects it before navigation starts."""
        attempted_goal = self.current_goal
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.info('Goal rejected by Nav2')
                self._blacklist_goal(attempted_goal, 'Nav2 rejected goal')
                self.kesif = True
                self.current_goal = None
                return

            logger.info('Goal accepted by Nav2')
            self.goal_handle = goal_handle
            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self.get_result_callback)
        except Exception as exc:
            logger.error(f'Exception in goal_response_callback: {exc}')
            self._blacklist_goal(attempted_goal, 'goal-response exception')
            self.kesif = True
            self.current_goal = None

    def get_result_callback(self, future):
        """Blacklist an unreachable Nav2 goal before selecting the next frontier."""
        attempted_goal = self.current_goal
        try:
            status = future.result().status
            if status == base.GoalStatus.STATUS_SUCCEEDED:
                logger.info('Goal succeeded!')
            else:
                logger.info(f'Goal failed with status: {status}')
                # If feedback_callback already considered the goal reached and
                # cleared current_goal, attempted_goal is None and we deliberately
                # do not blacklist the resulting cancellation.
                self._blacklist_goal(attempted_goal, f'Nav2 status {status}')

            self.kesif = True
            self.current_goal = None
        except Exception as exc:
            logger.error(f'Exception in get_result_callback: {exc}')
            self._blacklist_goal(attempted_goal, 'result exception')
            self.kesif = True
            self.current_goal = None

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
            logger.info("[INFO] No suitable non-blacklisted frontier found")
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
