#!/usr/bin/env python3
"""Nearest-frontier exploration with path-guided recovery and robust completion.

Frontier policy:
- free cell: occupancy == 0
- unknown cell: occupancy < 0
- frontier: free cell adjacent to unknown in the 8-neighbourhood
- frontier regions: 8-connected
- keep region only when size > 10 cells
- representative: actual frontier cell nearest the arithmetic mean
- ranking: Euclidean robot -> representative
- representatives closer than 0.5 m are not eligible

Navigation recovery:
- the selected frontier remains the main exploration goal until reached
- a lightweight Nav2 BT returns planner/controller failure quickly
- on main-goal failure, choose a temporary subgoal on the last global path
- after reaching the subgoal, retry the exact same main frontier

Completion policy:
- NEVER declare complete while a main/subgoal is active
- NEVER declare complete merely because remaining representatives are < 0.5 m
- require zero frontier regions (>10 cells) on 5 distinct map updates
- terminal evidence must span at least 10 s of navigation idle time
- do not declare complete during the first 20 s after node start
- once complete, publish a latched completion/status message exactly once and
  permanently stop issuing new navigation goals
"""

from collections import deque
import json
import math
import os

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MIN_REGION_SIZE = 10
MIN_DISTANCE_THRESHOLD = 0.5
MAP_FRAME = "map"
ROBOT_FRAME = "base_link"

SUBGOAL_MAX_DISTANCE_M = 0.70
SUBGOAL_MIN_DISTANCE_M = 0.25
SUBGOAL_FRACTION_OF_REMAINING = 0.50
SUBGOAL_MAIN_GOAL_CLEARANCE_M = 0.15

# A /plan is usable for path-guided recovery only when it actually reaches
# the exact main frontier, rather than stopping early because of planner/goal tolerance.
MAIN_PLAN_ENDPOINT_TOLERANCE_M = 0.10

COMPLETION_REQUIRED_SWEEPS = 5
COMPLETION_SWEEP_INTERVAL_S = 2.0
COMPLETION_IDLE_S = 10.0
COMPLETION_STARTUP_GRACE_S = 20.0

BT_XML_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "behavior_trees",
    "navigate_to_pose_subgoal.xml",
)


class NearestEuclideanFrontier(Node):
    def __init__(self):
        super().__init__("nearest_euclidean_frontier")

        self.map_msg = None
        self.map_generation = 0

        self.goal_active = False
        self.current_goal_mode = None

        # The chosen frontier remains fixed during path-guided recovery.
        self.main_goal = None
        self.latest_main_plan = None
        self.recovery_count = 0

        # Robust completion state.
        now_s = self.now_s()
        self.start_time_s = now_s
        self.last_navigation_activity_s = now_s
        self.completed = False
        self.completion_reason = None
        self.completion_streak = 0
        self.completion_first_evidence_s = None
        self.completion_last_sweep_s = -math.inf
        self.completion_last_map_generation = -1
        self.last_status_state = None

        self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.create_subscription(Path, "/plan", self.plan_callback, 10)

        self.goals_pub = self.create_publisher(
            MarkerArray,
            "/frontier/goals_markers",
            10,
        )
        self.path_pub = self.create_publisher(
            Path,
            "/frontier_selected_path",
            10,
        )

        terminal_qos = QoSProfile(depth=1)
        terminal_qos.reliability = ReliabilityPolicy.RELIABLE
        terminal_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.completion_pub = self.create_publisher(
            Bool,
            "/frontier_exploration_complete",
            terminal_qos,
        )
        self.status_pub = self.create_publisher(
            String,
            "/frontier_exploration_status",
            terminal_qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(1.0, self.exploration_step)

        if not os.path.isfile(BT_XML_PATH):
            self.get_logger().warn(
                f"Fast-fail recovery BT not found at {BT_XML_PATH}; "
                "NavigateToPose goals will use the Nav2 default BT."
            )

        initial_complete = Bool()
        initial_complete.data = False
        self.completion_pub.publish(initial_complete)
        self.publish_status("EXPLORING", reason="startup")

        self.get_logger().info(
            "Nearest Euclidean Frontier started "
            f"(min distance={MIN_DISTANCE_THRESHOLD:.2f} m, "
            f"path-subgoal recovery={SUBGOAL_MAX_DISTANCE_M:.2f} m max, "
            f"completion={COMPLETION_REQUIRED_SWEEPS} stable map sweeps)"
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_generation += 1

    def plan_callback(self, msg: Path):
        self.path_pub.publish(msg)

        # A subgoal plan must not overwrite the last path to the main frontier.
        # For main-goal recovery, also verify that the path has enough poses and
        # that its final pose actually reaches the exact frontier position.
        if (
            self.goal_active
            and self.current_goal_mode == "main"
            and self.main_goal is not None
        ):
            pose_count = len(msg.poses)
            main_x, main_y = self.main_goal

            if pose_count < 2:
                self.latest_main_plan = None
                self.get_logger().warn(
                    f"Main /plan diagnostic: poses={pose_count}; "
                    "not usable for path-guided recovery."
                )
                return

            endpoint = msg.poses[-1].pose.position
            endpoint_to_frontier_m = math.hypot(
                endpoint.x - main_x,
                endpoint.y - main_y,
            )
            endpoint_ok = (
                endpoint_to_frontier_m <= MAIN_PLAN_ENDPOINT_TOLERANCE_M
            )

            diagnostic = (
                f"Main /plan diagnostic: poses={pose_count}, "
                f"endpoint=({endpoint.x:.2f}, {endpoint.y:.2f}), "
                f"frontier=({main_x:.2f}, {main_y:.2f}), "
                f"endpoint_to_frontier={endpoint_to_frontier_m:.3f} m, "
                f"limit={MAIN_PLAN_ENDPOINT_TOLERANCE_M:.2f} m"
            )

            if endpoint_ok:
                self.latest_main_plan = msg
                self.get_logger().info(diagnostic + " -> usable")
            else:
                self.latest_main_plan = None
                self.get_logger().warn(
                    diagnostic + " -> rejected for subgoal recovery"
                )

    def publish_status(self, state: str, reason: str, **extra):
        payload = {
            "state": state,
            "reason": reason,
            "time_s": self.now_s(),
            "goal_active": bool(self.goal_active),
            "goal_mode": self.current_goal_mode,
            "main_goal": (
                None
                if self.main_goal is None
                else {"x": self.main_goal[0], "y": self.main_goal[1]}
            ),
            "map_generation": self.map_generation,
            **extra,
        }

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.status_pub.publish(msg)
        self.last_status_state = state

    def reset_completion_verification(self, reason: str):
        had_evidence = self.completion_streak > 0
        self.completion_reason = None
        self.completion_streak = 0
        self.completion_first_evidence_s = None
        self.completion_last_sweep_s = -math.inf
        self.completion_last_map_generation = -1

        if had_evidence:
            self.get_logger().info(
                f"Completion verification reset: {reason}"
            )
        if not self.completed and self.last_status_state != "EXPLORING":
            self.publish_status("EXPLORING", reason=reason)

    def observe_no_frontier_terminal_state(self):
        """Accumulate conservative completion evidence on distinct map updates."""
        if self.completed:
            return

        now_s = self.now_s()

        # Do not count the same OccupancyGrid repeatedly.
        if self.map_generation == self.completion_last_map_generation:
            return

        # Keep terminal sweeps spaced in time even if /map updates very quickly.
        if now_s - self.completion_last_sweep_s < COMPLETION_SWEEP_INTERVAL_S:
            return

        self.completion_last_map_generation = self.map_generation
        self.completion_last_sweep_s = now_s

        if self.completion_reason != "no_frontier_region_gt_10":
            self.completion_reason = "no_frontier_region_gt_10"
            self.completion_streak = 1
            self.completion_first_evidence_s = now_s
        else:
            self.completion_streak += 1

        idle_s = now_s - self.last_navigation_activity_s
        age_s = now_s - self.start_time_s

        self.publish_status(
            "VERIFYING_COMPLETE",
            reason=self.completion_reason,
            stable_sweeps=self.completion_streak,
            required_sweeps=COMPLETION_REQUIRED_SWEEPS,
            idle_s=round(idle_s, 3),
            required_idle_s=COMPLETION_IDLE_S,
            age_s=round(age_s, 3),
            startup_grace_s=COMPLETION_STARTUP_GRACE_S,
        )

        self.get_logger().info(
            "Completion verification: "
            f"{self.completion_streak}/{COMPLETION_REQUIRED_SWEEPS} "
            f"stable no-frontier sweeps, idle={idle_s:.1f}s, age={age_s:.1f}s"
        )

        if (
            self.completion_streak >= COMPLETION_REQUIRED_SWEEPS
            and idle_s >= COMPLETION_IDLE_S
            and age_s >= COMPLETION_STARTUP_GRACE_S
        ):
            self.mark_exploration_complete()

    def mark_exploration_complete(self):
        if self.completed:
            return

        self.completed = True
        self.clear_goal_markers()

        empty_path = Path()
        empty_path.header.frame_id = MAP_FRAME
        empty_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(empty_path)

        complete = Bool()
        complete.data = True
        self.completion_pub.publish(complete)

        self.publish_status(
            "COMPLETE",
            reason="no_frontier_region_gt_10",
            stable_sweeps=self.completion_streak,
            required_sweeps=COMPLETION_REQUIRED_SWEEPS,
            idle_s=round(self.now_s() - self.last_navigation_activity_s, 3),
        )

        self.get_logger().warn(
            "================ EXPLORATION COMPLETE ================\n"
            "Reason: no frontier region > 10 cells remained stably.\n"
            f"Evidence: {self.completion_streak} distinct-map sweeps, "
            f"idle >= {COMPLETION_IDLE_S:.0f} s.\n"
            "No further navigation goals will be issued.\n"
            "======================================================"
        )

    @staticmethod
    def frontier_mask(grid: np.ndarray) -> np.ndarray:
        height, width = grid.shape
        mask = np.zeros((height, width), dtype=bool)

        for row in range(height):
            for col in range(width):
                if grid[row, col] != 0:
                    continue

                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue

                        nr = row + dr
                        nc = col + dc
                        if (
                            0 <= nr < height
                            and 0 <= nc < width
                            and grid[nr, nc] < 0
                        ):
                            mask[row, col] = True
                            break
                    if mask[row, col]:
                        break

        return mask

    @staticmethod
    def frontier_regions(mask: np.ndarray):
        height, width = mask.shape
        visited = np.zeros_like(mask, dtype=bool)
        regions = []

        for row in range(height):
            for col in range(width):
                if not mask[row, col] or visited[row, col]:
                    continue

                queue = deque([(row, col)])
                visited[row, col] = True
                region = []

                while queue:
                    r, c = queue.popleft()
                    region.append((r, c))

                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue

                            nr = r + dr
                            nc = c + dc
                            if (
                                0 <= nr < height
                                and 0 <= nc < width
                                and mask[nr, nc]
                                and not visited[nr, nc]
                            ):
                                visited[nr, nc] = True
                                queue.append((nr, nc))

                if len(region) > MIN_REGION_SIZE:
                    regions.append(region)

        return regions

    @staticmethod
    def representative(region):
        mean_row = sum(cell[0] for cell in region) / len(region)
        mean_col = sum(cell[1] for cell in region) / len(region)

        return min(
            region,
            key=lambda cell: (
                (cell[0] - mean_row) ** 2 + (cell[1] - mean_col) ** 2
            ),
        )

    def cell_to_world(self, row: int, col: int):
        info = self.map_msg.info
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (row + 0.5) * info.resolution
        return x, y

    def robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                ROBOT_FRAME,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(f"Waiting for TF map -> base_link: {exc}")
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def publish_goal_markers(self, candidates, selected):
        """Green = eligible candidates, small red = selected. No text labels."""
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        now = self.get_clock().now().to_msg()

        for index, candidate in enumerate(candidates):
            _distance, x, y, _row, _col, _region_size = candidate

            marker = Marker()
            marker.header.frame_id = MAP_FRAME
            marker.header.stamp = now
            marker.ns = "frontier_candidates"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.10
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.18
            marker.scale.y = 0.18
            marker.scale.z = 0.18
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.1
            marker.color.a = 0.95
            markers.markers.append(marker)

        _distance, x, y, _row, _col, _region_size = selected

        chosen = Marker()
        chosen.header.frame_id = MAP_FRAME
        chosen.header.stamp = now
        chosen.ns = "selected_frontier"
        chosen.id = 0
        chosen.type = Marker.SPHERE
        chosen.action = Marker.ADD
        chosen.pose.position.x = x
        chosen.pose.position.y = y
        chosen.pose.position.z = 0.12
        chosen.pose.orientation.w = 1.0
        chosen.scale.x = 0.24
        chosen.scale.y = 0.24
        chosen.scale.z = 0.24
        chosen.color.r = 1.0
        chosen.color.g = 0.1
        chosen.color.b = 0.1
        chosen.color.a = 1.0
        markers.markers.append(chosen)

        self.goals_pub.publish(markers)

    def clear_goal_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self.goals_pub.publish(markers)

    def exploration_step(self):
        if self.completed:
            return

        if self.map_msg is None or self.goal_active:
            return

        # Never declare completion while a selected frontier is still unresolved.
        # A failed selected frontier remains the main goal rather than being
        # blacklisted or replaced by another frontier.
        if self.main_goal is not None:
            self.reset_completion_verification("main_frontier_still_pending")
            x, y = self.main_goal
            self.get_logger().info(
                f"Retrying main frontier after recovery: x={x:.2f}, y={y:.2f}"
            )
            self.send_navigation_goal(x, y, mode="main")
            return

        robot = self.robot_position()
        if robot is None:
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        grid = np.asarray(
            self.map_msg.data,
            dtype=np.int16,
        ).reshape(height, width)

        mask = self.frontier_mask(grid)
        regions = self.frontier_regions(mask)

        # Conservative completion: only zero large frontier regions counts.
        # A transient empty map must survive the stable verification window.
        if not regions:
            self.clear_goal_markers()
            self.observe_no_frontier_terminal_state()
            return

        # Any real large frontier region immediately invalidates completion
        # evidence, even if its representative is inside the 0.5 m filter.
        self.reset_completion_verification("frontier_region_present")

        robot_x, robot_y = robot
        candidates = []

        for region in regions:
            row, col = self.representative(region)
            x, y = self.cell_to_world(row, col)
            distance = math.hypot(x - robot_x, y - robot_y)

            if distance < MIN_DISTANCE_THRESHOLD:
                continue

            candidates.append((distance, x, y, row, col, len(region)))

        if not candidates:
            self.clear_goal_markers()
            self.publish_status(
                "BLOCKED_BY_MIN_DISTANCE",
                reason="frontier_regions_exist_but_all_representatives_too_close",
                frontier_regions=len(regions),
                min_distance_m=MIN_DISTANCE_THRESHOLD,
            )
            self.get_logger().warn(
                "Frontier regions still exist, but every representative is "
                f"closer than {MIN_DISTANCE_THRESHOLD:.2f} m. "
                "NOT declaring exploration complete."
            )
            return

        selected = min(candidates, key=lambda item: item[0])
        distance, x, y, _row, _col, region_size = selected

        self.publish_goal_markers(candidates, selected)

        self.get_logger().info(
            f"Selected nearest frontier: x={x:.2f}, y={y:.2f}, "
            f"distance={distance:.2f} m, region={region_size} cells"
        )

        self.main_goal = (x, y)
        self.latest_main_plan = None
        self.recovery_count = 0
        self.send_navigation_goal(x, y, mode="main")

    def send_navigation_goal(self, x: float, y: float, mode: str):
        if self.completed:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().warn("NavigateToPose action server is not ready")
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0

        if os.path.isfile(BT_XML_PATH):
            goal.behavior_tree = BT_XML_PATH

        if mode == "main":
            # Require a fresh /plan for each main-goal attempt.
            self.latest_main_plan = None

        self.goal_active = True
        self.current_goal_mode = mode
        self.last_navigation_activity_s = self.now_s()
        self.reset_completion_verification(f"{mode}_goal_started")

        self.publish_status(
            "NAVIGATING",
            reason=f"{mode}_goal",
            target_x=round(x, 4),
            target_y=round(y, 4),
            recovery_count=self.recovery_count,
        )

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done_future, sent_mode=mode: self.goal_response_callback(
                done_future,
                sent_mode,
            )
        )

    def goal_response_callback(self, future, mode: str):
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"{mode.capitalize()} goal request failed: {exc}"
            )
            self.goal_active = False
            self.current_goal_mode = None
            self.last_navigation_activity_s = self.now_s()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(f"{mode.capitalize()} goal rejected")
            self.goal_active = False
            self.current_goal_mode = None
            self.last_navigation_activity_s = self.now_s()
            return

        self.get_logger().info(f"{mode.capitalize()} goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, sent_mode=mode: self.goal_result_callback(
                done_future,
                sent_mode,
            )
        )

    def goal_result_callback(self, future, mode: str):
        try:
            wrapped_result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"{mode.capitalize()} goal result failed: {exc}"
            )
            self.goal_active = False
            self.current_goal_mode = None
            self.last_navigation_activity_s = self.now_s()
            return

        status = wrapped_result.status
        result = wrapped_result.result

        error_code = getattr(result, "error_code", None)
        error_msg = getattr(result, "error_msg", "")

        self.goal_active = False
        self.current_goal_mode = None
        self.last_navigation_activity_s = self.now_s()

        if status == GoalStatus.STATUS_SUCCEEDED:
            if mode == "subgoal":
                self.get_logger().info(
                    "Recovery subgoal reached; retrying the same main frontier."
                )
                if self.main_goal is not None:
                    x, y = self.main_goal
                    self.send_navigation_goal(x, y, mode="main")
                return

            self.get_logger().info("Goal reached")
            self.main_goal = None
            self.latest_main_plan = None
            self.recovery_count = 0
            self.publish_status("EXPLORING", reason="main_goal_reached")
            return

        detail = f"status={status}"
        if error_code is not None:
            detail += f", error_code={error_code}"
        if error_msg:
            detail += f", error_msg={error_msg}"

        self.get_logger().warn(f"{mode.capitalize()} goal failed ({detail})")

        if mode == "main":
            self.try_path_guided_subgoal()
        else:
            # Do not blacklist or switch frontier. The timer retries the same
            # main frontier from the current pose.
            self.get_logger().warn(
                "Recovery subgoal also failed; keeping the same main frontier."
            )

    def try_path_guided_subgoal(self):
        if self.main_goal is None or self.completed:
            return

        plan = self.latest_main_plan
        if plan is None or len(plan.poses) < 2:
            self.get_logger().warn(
                "Main goal failed but no usable main-goal /plan is available; "
                "the same main frontier will be retried."
            )
            return

        subgoal = self.select_subgoal_on_path(plan)
        if subgoal is None:
            self.get_logger().warn(
                "Could not choose a useful intermediate point on the main path; "
                "the same main frontier will be retried."
            )
            return

        x, y, along_path_m, remaining_path_m = subgoal
        self.recovery_count += 1

        self.get_logger().warn(
            f"Path-guided recovery #{self.recovery_count}: "
            f"temporary subgoal x={x:.2f}, y={y:.2f}, "
            f"{along_path_m:.2f} m ahead on main path "
            f"(remaining main path={remaining_path_m:.2f} m)"
        )
        self.send_navigation_goal(x, y, mode="subgoal")

    def select_subgoal_on_path(self, plan: Path):
        robot = self.robot_position()
        if robot is None:
            return None

        robot_x, robot_y = robot
        poses = plan.poses
        if len(poses) < 2:
            return None

        closest_index = min(
            range(len(poses)),
            key=lambda index: (
                (poses[index].pose.position.x - robot_x) ** 2
                + (poses[index].pose.position.y - robot_y) ** 2
            ),
        )

        cumulative = [0.0]
        for index in range(closest_index + 1, len(poses)):
            previous = poses[index - 1].pose.position
            current = poses[index].pose.position
            cumulative.append(
                cumulative[-1]
                + math.hypot(
                    current.x - previous.x,
                    current.y - previous.y,
                )
            )

        remaining_path_m = cumulative[-1]
        if remaining_path_m <= SUBGOAL_MIN_DISTANCE_M:
            return None

        desired_distance_m = min(
            SUBGOAL_MAX_DISTANCE_M,
            max(
                SUBGOAL_MIN_DISTANCE_M,
                remaining_path_m * SUBGOAL_FRACTION_OF_REMAINING,
            ),
        )

        # Keep the temporary goal distinct from the main frontier whenever the
        # remaining path is long enough.
        if remaining_path_m > SUBGOAL_MAIN_GOAL_CLEARANCE_M:
            desired_distance_m = min(
                desired_distance_m,
                remaining_path_m - SUBGOAL_MAIN_GOAL_CLEARANCE_M,
            )

        if desired_distance_m <= 0.0:
            return None

        selected_offset = len(cumulative) - 1
        for offset, distance_m in enumerate(cumulative):
            if distance_m >= desired_distance_m:
                selected_offset = offset
                break

        selected_pose = poses[closest_index + selected_offset].pose.position
        actual_distance_m = cumulative[selected_offset]

        # If discretization leaves the selected pose effectively on top of the
        # robot, there is no useful recovery motion to command.
        if math.hypot(
            selected_pose.x - robot_x,
            selected_pose.y - robot_y,
        ) < 0.10:
            return None

        return (
            selected_pose.x,
            selected_pose.y,
            actual_distance_m,
            remaining_path_m,
        )


def main(args=None):
    rclpy.init(args=args)
    node = NearestEuclideanFrontier()

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
