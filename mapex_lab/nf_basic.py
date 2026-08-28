#!/usr/bin/env python3
"""Minimal nearest-frontier exploration using Euclidean distance.

Pipeline:
    /map -> frontier cells -> 8-connected frontier regions
    -> one representative per region -> reject representatives < 0.5 m
    -> nearest remaining representative by Euclidean distance
    -> Nav2 NavigateToPose

Navigation recovery:
- Frontier ranking is unchanged.
- The selected frontier remains the main exploration goal until it is reached.
- A lightweight Behavior Tree returns controller/planner failure directly instead
  of running the stock recovery loop many times.
- If the main goal aborts and a valid recent global path exists, choose a temporary
  subgoal on that path near the robot, reach it, then retry the same main frontier
  so Nav2 replans from the new pose.

RViz visualization uses the Hospital RViz config's existing displays:
- /frontier/goals_markers : eligible frontier representatives + selected goal
- /frontier_selected_path : current Nav2 global path

No LaMa, information gain, path-length ranking, recorder, or MapEx prediction.
"""

from collections import deque
import math
import os

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
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

BT_XML_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "behavior_trees",
    "navigate_to_pose_subgoal.xml",
)


class NearestEuclideanFrontier(Node):
    def __init__(self):
        super().__init__("nearest_euclidean_frontier")

        self.map_msg = None
        self.goal_active = False
        self.current_goal_mode = None

        # The selected frontier stays fixed through temporary subgoal recovery.
        self.main_goal = None
        self.latest_main_plan = None
        self.recovery_count = 0

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.timer = self.create_timer(1.0, self.exploration_step)

        if not os.path.isfile(BT_XML_PATH):
            self.get_logger().warn(
                f"Fast-fail recovery BT not found at {BT_XML_PATH}; "
                "NavigateToPose goals will use the Nav2 default BT."
            )

        self.get_logger().info(
            f"Nearest Euclidean Frontier started "
            f"(min distance={MIN_DISTANCE_THRESHOLD:.2f} m, "
            f"path-subgoal recovery={SUBGOAL_MAX_DISTANCE_M:.2f} m max)"
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def plan_callback(self, msg: Path):
        self.path_pub.publish(msg)

        # Subgoal plans must not overwrite the last path toward the main frontier.
        if self.goal_active and self.current_goal_mode == "main" and msg.poses:
            self.latest_main_plan = msg

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
                        if 0 <= nr < height and 0 <= nc < width and grid[nr, nc] < 0:
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
            key=lambda cell: (cell[0] - mean_row) ** 2 + (cell[1] - mean_col) ** 2,
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
        if self.map_msg is None or self.goal_active:
            return

        # A failed selected frontier remains the main goal. Retry it rather than
        # re-running frontier ranking or choosing a different frontier.
        if self.main_goal is not None:
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
        grid = np.asarray(self.map_msg.data, dtype=np.int16).reshape(height, width)

        mask = self.frontier_mask(grid)
        regions = self.frontier_regions(mask)

        if not regions:
            self.clear_goal_markers()
            self.get_logger().info("No frontier left. Exploration complete.")
            return

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
            self.get_logger().info(
                f"No frontier at least {MIN_DISTANCE_THRESHOLD:.2f} m from robot."
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

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done_future, sent_mode=mode: self.goal_response_callback(
                done_future,
                sent_mode,
            )
        )

    def goal_response_callback(self, future, mode: str):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(f"{mode.capitalize()} goal rejected")
            self.goal_active = False
            self.current_goal_mode = None
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
        wrapped_result = future.result()
        status = wrapped_result.status
        result = wrapped_result.result

        error_code = getattr(result, "error_code", None)
        error_msg = getattr(result, "error_msg", "")

        self.goal_active = False
        self.current_goal_mode = None

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
            # Do not blacklist or switch frontier. The timer will retry the same
            # main goal, obtain a fresh global path, and recovery can be attempted
            # again from the current pose.
            self.get_logger().warn(
                "Recovery subgoal also failed; keeping the same main frontier."
            )

    def try_path_guided_subgoal(self):
        if self.main_goal is None:
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
                + math.hypot(current.x - previous.x, current.y - previous.y)
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
        # remaining path is long enough to do so.
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
