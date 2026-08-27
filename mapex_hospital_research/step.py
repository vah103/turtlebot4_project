"""Interactive one-goal-at-a-time debug runner for control_tb4.

Run one nearest-frontier goal, print only the navigation signals useful for
locating bugs, then wait for ENTER before allowing the next goal.

This wrapper intentionally leaves control_tb4.py unchanged.
"""

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist

from control_tb4 import navigationControl, logger


_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class StepNavigationControl(navigationControl):
    """Run exactly one Nav2 goal per user confirmation."""

    def __init__(self):
        self.waiting_for_continue = False
        self.goal_index = 0
        self.goal_start_time = None
        self.goal_start_pose = None
        self.goal_start_xy = None
        self.goal_last_odom_xy = None
        self.goal_odom_distance = 0.0
        self.goal_min_scan = float("inf")
        self.goal_path_length = None
        self.goal_frontier_count = None
        self.goal_region_id = None
        self.goal_region_size = None
        self.goal_euclidean_distance = None
        self.last_cmd_v = None
        self.last_cmd_w = None
        self.last_odom_v = None
        self.last_odom_w = None

        super().__init__()

        self.subscription_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.timer_step_debug = self.create_timer(1.0, self.print_live_debug)
        logger.info("[STEP] One-goal debug mode active")
        logger.info("[STEP] Nav2 must finish the goal itself; no early 0.5 m cancel")

    def exploration_loop(self):
        if self.waiting_for_continue:
            return
        super().exploration_loop()

    def send_goal(self, x, y):
        self.goal_index += 1
        self.goal_start_time = time.perf_counter()
        self.goal_start_pose = (self.x, self.y, self.yaw)
        self.goal_start_xy = (self.x, self.y)
        self.goal_last_odom_xy = (self.x, self.y)
        self.goal_odom_distance = 0.0
        self.goal_min_scan = float("inf")
        self.goal_path_length = None
        self.goal_frontier_count = len(getattr(self, 'groups', []))
        self.goal_region_id = None
        self.goal_region_size = None
        self.goal_euclidean_distance = math.hypot(x - self.x, y - self.y)

        # Recover the selected frontier-region metadata for the debug summary.
        best_error = float("inf")
        for group_id, group_cells, centroid in getattr(self, 'groups', []):
            if centroid is None:
                continue
            cx = centroid[1] * self.resolution + self.originX
            cy = centroid[0] * self.resolution + self.originY
            error = math.hypot(cx - x, cy - y)
            if error < best_error:
                best_error = error
                self.goal_region_id = group_id
                self.goal_region_size = len(group_cells)

        logger.info(
            "[STEP] GOAL %d selected | target=(%.2f, %.2f) | "
            "distance=%.2f m | frontier_groups=%s | region=%s (%s cells)",
            self.goal_index,
            x,
            y,
            self.goal_euclidean_distance,
            self.goal_frontier_count,
            self.goal_region_id,
            self.goal_region_size,
        )
        super().send_goal(x, y)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                logger.warning("[STEP] Goal rejected by Nav2")
                self._finish_step(GoalStatus.STATUS_UNKNOWN, rejected=True)
                return

            logger.info("[STEP] Goal accepted by Nav2")
            self.goal_handle = goal_handle
            self.result_future = goal_handle.get_result_async()
            self.result_future.add_done_callback(self.get_result_callback)
        except Exception as exc:
            logger.error("[STEP] goal_response_callback exception: %s", exc)
            self._finish_step(GoalStatus.STATUS_UNKNOWN)

    def get_result_callback(self, future):
        try:
            status = future.result().status
            self._finish_step(status)
        except Exception as exc:
            logger.error("[STEP] get_result_callback exception: %s", exc)
            self._finish_step(GoalStatus.STATUS_UNKNOWN)

    def feedback_callback(self, feedback_msg):
        # Important for debugging: do not cancel at the old 0.5 m custom
        # tolerance. Wait for Nav2's real terminal result instead.
        return

    def plan_callback(self, msg):
        super().plan_callback(msg)
        if self.current_goal is None or len(msg.poses) < 2:
            return
        length = 0.0
        for first, second in zip(msg.poses[:-1], msg.poses[1:]):
            dx = second.pose.position.x - first.pose.position.x
            dy = second.pose.position.y - first.pose.position.y
            length += math.hypot(dx, dy)
        self.goal_path_length = length

    def scan_callback(self, msg):
        super().scan_callback(msg)
        if self.current_goal is None:
            return
        valid = [
            value for value in msg.ranges
            if math.isfinite(value) and msg.range_min <= value <= msg.range_max
        ]
        if valid:
            self.goal_min_scan = min(self.goal_min_scan, min(valid))

    def odom_callback(self, msg):
        super().odom_callback(msg)
        self.last_odom_v = msg.twist.twist.linear.x
        self.last_odom_w = msg.twist.twist.angular.z

        if self.current_goal is None or self.goal_last_odom_xy is None:
            return
        current = (self.x, self.y)
        self.goal_odom_distance += math.hypot(
            current[0] - self.goal_last_odom_xy[0],
            current[1] - self.goal_last_odom_xy[1],
        )
        self.goal_last_odom_xy = current

    def cmd_vel_callback(self, msg):
        self.last_cmd_v = msg.linear.x
        self.last_cmd_w = msg.angular.z

    def print_live_debug(self):
        if self.current_goal is None or self.waiting_for_continue:
            return
        elapsed = time.perf_counter() - self.goal_start_time
        remaining = math.hypot(
            self.current_goal[0] - self.x,
            self.current_goal[1] - self.y,
        )
        min_scan = (
            f"{self.goal_min_scan:.2f}"
            if math.isfinite(self.goal_min_scan)
            else "n/a"
        )
        cmd = (
            f"({self.last_cmd_v:.2f}, {self.last_cmd_w:.2f})"
            if self.last_cmd_v is not None
            else "n/a"
        )
        odom_vel = (
            f"({self.last_odom_v:.2f}, {self.last_odom_w:.2f})"
            if self.last_odom_v is not None
            else "n/a"
        )
        logger.info(
            "[MOVE] t=%.1fs pose=(%.2f, %.2f) remaining=%.2fm "
            "cmd(v,w)=%s odom(v,w)=%s min_scan=%sm",
            elapsed,
            self.x,
            self.y,
            remaining,
            cmd,
            odom_vel,
            min_scan,
        )

    def _finish_step(self, status, rejected=False):
        # Set pause before releasing current_goal so the 1 Hz exploration timer
        # cannot immediately dispatch another frontier.
        self.waiting_for_continue = True

        elapsed = (
            time.perf_counter() - self.goal_start_time
            if self.goal_start_time is not None
            else 0.0
        )
        final_pose = (self.x, self.y, self.yaw)
        final_error = (
            math.hypot(self.current_goal[0] - self.x, self.current_goal[1] - self.y)
            if self.current_goal is not None
            else float("nan")
        )
        min_scan = (
            f"{self.goal_min_scan:.2f} m"
            if math.isfinite(self.goal_min_scan)
            else "n/a"
        )
        path_length = (
            f"{self.goal_path_length:.2f} m"
            if self.goal_path_length is not None
            else "n/a"
        )
        status_name = "REJECTED" if rejected else _STATUS_NAMES.get(status, str(status))

        logger.info("\n========== STEP %d RESULT ==========", self.goal_index)
        logger.info("Nav2 result      : %s", status_name)
        logger.info(
            "Start pose       : (%.2f, %.2f, %.2f)", *self.goal_start_pose
        )
        if self.current_goal is not None:
            logger.info(
                "Goal             : (%.2f, %.2f) | initial distance %.2f m",
                self.current_goal[0],
                self.current_goal[1],
                self.goal_euclidean_distance,
            )
        logger.info(
            "Frontier         : %s groups | selected region %s (%s cells)",
            self.goal_frontier_count,
            self.goal_region_id,
            self.goal_region_size,
        )
        logger.info("Latest path      : %s", path_length)
        logger.info("Travel time      : %.2f s", elapsed)
        logger.info("Odom distance    : %.2f m", self.goal_odom_distance)
        logger.info(
            "Final pose       : (%.2f, %.2f, %.2f)", *final_pose
        )
        logger.info("Final goal error : %.2f m", final_error)
        logger.info("Minimum scan     : %s", min_scan)
        if self.last_cmd_v is not None:
            logger.info(
                "Last cmd(v,w)    : (%.2f, %.2f)", self.last_cmd_v, self.last_cmd_w
            )
        if self.last_odom_v is not None:
            logger.info(
                "Last odom(v,w)   : (%.2f, %.2f)", self.last_odom_v, self.last_odom_w
            )
        logger.info("Map resolution   : %.3f m/cell", self.resolution)
        logger.info("====================================")

        self.kesif = True
        self.current_goal = None
        self.goal_handle = None

        threading.Thread(target=self._wait_for_enter, daemon=True).start()

    def _wait_for_enter(self):
        try:
            input("\nPress ENTER to run the next frontier goal... ")
        except EOFError:
            logger.warning("[STEP] No interactive terminal; remaining paused")
            return
        self.waiting_for_continue = False
        logger.info("[STEP] Continuing to next frontier goal")


def main(args=None):
    rclpy.init(args=args)
    node = StepNavigationControl()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
