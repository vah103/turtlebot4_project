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
- representatives at least 1.0 m away are preferred; if every current
  representative is closer than 1.0 m, the near-frontier set is allowed as a
  fallback so exploration does not stall solely because of the distance guard

Navigation recovery:
- the selected frontier remains the main exploration goal until reached
- a lightweight Nav2 BT returns planner/controller failure quickly
- on ordinary main-goal execution failure, choose a temporary subgoal on the last
  global path and then retry the exact same main frontier
- Nav2 105 (FAILED_TO_MAKE_PROGRESS) gets at most one path-guided recovery attempt;
  if no useful subgoal exists, the recovery subgoal fails, or 105 happens again,
  suppress that frontier locally for a short cooldown and continue elsewhere
- Nav2 206 (GOAL_OCCUPIED) and 208 (NO_VALID_PATH) remain planner-blocking failures
  and are handled by the existing planner suppression/revalidation logic

Completion policy:
- NEVER declare complete while a main/subgoal is active
- NEVER declare complete merely because remaining representatives are < 1.0 m
- case 1: require zero frontier regions (>10 cells) on 5 distinct map updates
- case 2: if frontier regions remain but every eligible representative is currently
  suppressed after planner-blocking 206/208 failures, revalidate the whole set with
  ComputePathToPose; require 5 consecutive exhausted planner sweeps at least 2 s apart
- a temporary 105 execution cooldown is NOT terminal evidence and blocks terminal
  planner revalidation until the cooldown expires
- any planner-reachable frontier resets completion verification and resumes exploration
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
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


MIN_REGION_SIZE = 10
MIN_DISTANCE_THRESHOLD = 1.0
MAP_FRAME = "map"
ROBOT_FRAME = "base_link"

SUBGOAL_MAX_DISTANCE_M = 0.70
SUBGOAL_MIN_DISTANCE_M = 0.25
SUBGOAL_FRACTION_OF_REMAINING = 0.50
SUBGOAL_MAIN_GOAL_CLEARANCE_M = 0.15

# A /plan is usable for path-guided recovery only when it actually reaches
# the exact main frontier, rather than stopping early because of planner/goal tolerance.
MAIN_PLAN_ENDPOINT_TOLERANCE_M = 0.10

# FollowPath error 105 is an execution/local-navigation failure. Give it one
# path-guided recovery chance, then move on temporarily rather than retry forever.
FAILED_TO_MAKE_PROGRESS_ERROR_CODE = 105
EXECUTION_FAILURE_LIMIT = 2
EXECUTION_BLOCKED_SKIP_RADIUS_M = 0.30
EXECUTION_BLOCKED_COOLDOWN_S = 30.0

# NavigateToPose propagates planner failures from ComputePathToPose. GOAL_OCCUPIED
# (206) and NO_VALID_PATH (208) are planner-blocking failures.
GOAL_OCCUPIED_ERROR_CODE = 206
NO_VALID_PATH_ERROR_CODE = 208
PLANNER_BLOCKING_ERROR_CODES = {
    GOAL_OCCUPIED_ERROR_CODE,
    NO_VALID_PATH_ERROR_CODE,
}
PLANNER_BLOCKED_SKIP_RADIUS_M = 0.10

COMPLETION_REQUIRED_SWEEPS = 5
COMPLETION_SWEEP_INTERVAL_S = 2.0
COMPLETION_IDLE_S = 10.0
COMPLETION_STARTUP_GRACE_S = 20.0

BT_XML_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "behavior_trees",
    "subgoal_bt.xml",
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

        # 105-specific bounded recovery state for the current main frontier.
        self.execution_failure_count = 0
        self.recovery_trigger_error_code = None
        # (x, y, expiry_time_s)
        self.execution_blocked_goals = []

        # Main frontier positions that most recently returned planner-blocking
        # GOAL_OCCUPIED (206) or NO_VALID_PATH (208).
        self.planner_blocked_goals = []

        # Robust completion state shared by both terminal conditions.
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

        # Planner-only terminal revalidation state.
        self.revalidation_active = False
        self.revalidation_candidates = []
        self.revalidation_index = 0
        self.revalidation_signature = None
        self.revalidation_last_start_s = -math.inf

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
        self.planner_client = ActionClient(
            self,
            ComputePathToPose,
            "compute_path_to_pose",
        )
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
            f"(preferred min distance={MIN_DISTANCE_THRESHOLD:.2f} m, "
            "near fallback=all-frontiers-close, "
            f"path-subgoal recovery={SUBGOAL_MAX_DISTANCE_M:.2f} m max, "
            f"105 limit={EXECUTION_FAILURE_LIMIT}, "
            f"105 cooldown={EXECUTION_BLOCKED_COOLDOWN_S:.0f} s, "
            f"completion={COMPLETION_REQUIRED_SWEEPS} stable sweeps)"
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.map_generation += 1

    def plan_callback(self, msg: Path):
        self.path_pub.publish(msg)

        # A subgoal plan must not overwrite the last path to the main frontier.
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

        if self.map_generation == self.completion_last_map_generation:
            return
        if now_s - self.completion_last_sweep_s < COMPLETION_SWEEP_INTERVAL_S:
            return

        self.completion_last_map_generation = self.map_generation
        self.completion_last_sweep_s = now_s
        self.revalidation_signature = None

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

        reason = self.completion_reason or "no_frontier_region_gt_10"
        self.completed = True
        self.revalidation_active = False
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
            reason=reason,
            stable_sweeps=self.completion_streak,
            required_sweeps=COMPLETION_REQUIRED_SWEEPS,
            idle_s=round(self.now_s() - self.last_navigation_activity_s, 3),
        )

        if reason == "no_planner_reachable_frontier":
            reason_text = (
                "frontier regions remain, but no eligible frontier was planner-"
                "reachable across repeated ComputePathToPose sweeps"
            )
        else:
            reason_text = "no frontier region > 10 cells remained stably"

        self.get_logger().warn(
            "================ EXPLORATION COMPLETE ================\n"
            f"Reason: {reason_text}.\n"
            f"Evidence: {self.completion_streak} stable sweeps, "
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

    # ----- temporary 105 execution suppression -----

    def prune_execution_blocked_goals(self):
        now_s = self.now_s()
        self.execution_blocked_goals = [
            (x, y, expiry_s)
            for x, y, expiry_s in self.execution_blocked_goals
            if expiry_s > now_s
        ]

    def is_execution_blocked_suppressed(self, x: float, y: float) -> bool:
        self.prune_execution_blocked_goals()
        return any(
            math.hypot(x - blocked_x, y - blocked_y)
            <= EXECUTION_BLOCKED_SKIP_RADIUS_M
            for blocked_x, blocked_y, _expiry_s in self.execution_blocked_goals
        )

    def execution_blocked_remaining_s(self, x: float, y: float):
        self.prune_execution_blocked_goals()
        remaining = [
            expiry_s - self.now_s()
            for blocked_x, blocked_y, expiry_s in self.execution_blocked_goals
            if math.hypot(x - blocked_x, y - blocked_y)
            <= EXECUTION_BLOCKED_SKIP_RADIUS_M
        ]
        return max(remaining) if remaining else 0.0

    def candidates_have_execution_cooldown(self, candidates) -> bool:
        self.prune_execution_blocked_goals()
        return any(
            self.is_execution_blocked_suppressed(candidate[1], candidate[2])
            for candidate in candidates
        )

    def suppress_main_goal_execution_failed(
        self,
        error_code: int,
        reason: str,
    ):
        if self.main_goal is None:
            return

        x, y = self.main_goal
        now_s = self.now_s()
        expiry_s = now_s + EXECUTION_BLOCKED_COOLDOWN_S

        # Refresh one suppression record around this frontier rather than stacking it.
        self.execution_blocked_goals = [
            (blocked_x, blocked_y, blocked_expiry_s)
            for blocked_x, blocked_y, blocked_expiry_s
            in self.execution_blocked_goals
            if math.hypot(x - blocked_x, y - blocked_y)
            > EXECUTION_BLOCKED_SKIP_RADIUS_M
        ]
        self.execution_blocked_goals.append((x, y, expiry_s))

        self.get_logger().warn(
            "Main frontier temporarily suppressed after "
            f"FAILED_TO_MAKE_PROGRESS ({error_code}): x={x:.2f}, y={y:.2f}; "
            f"reason={reason}; radius={EXECUTION_BLOCKED_SKIP_RADIUS_M:.2f} m, "
            f"cooldown={EXECUTION_BLOCKED_COOLDOWN_S:.0f} s. "
            "Exploration will continue with another frontier."
        )

        failure_count = self.execution_failure_count
        self.main_goal = None
        self.latest_main_plan = None
        self.recovery_count = 0
        self.execution_failure_count = 0
        self.recovery_trigger_error_code = None
        self.revalidation_signature = None

        self.publish_status(
            "EXPLORING",
            reason="main_goal_execution_cooldown",
            abandoned_x=round(x, 4),
            abandoned_y=round(y, 4),
            error_code=error_code,
            execution_failure_count=failure_count,
            execution_failure_limit=EXECUTION_FAILURE_LIMIT,
            skip_radius_m=EXECUTION_BLOCKED_SKIP_RADIUS_M,
            cooldown_s=EXECUTION_BLOCKED_COOLDOWN_S,
            cooldown_reason=reason,
        )

    # ----- planner-blocked suppression -----

    def is_planner_blocked_suppressed(self, x: float, y: float) -> bool:
        # Keep the existing public method as the shared selection gate used by
        # both Nearest and MapEx. A temporary 105 cooldown therefore affects both
        # methods without duplicating execution logic in mapex.py.
        if self.is_execution_blocked_suppressed(x, y):
            return True

        return any(
            math.hypot(x - rejected_x, y - rejected_y)
            <= PLANNER_BLOCKED_SKIP_RADIUS_M
            for rejected_x, rejected_y in self.planner_blocked_goals
        )

    def clear_planner_blocked_suppression_near(self, x: float, y: float):
        self.planner_blocked_goals = [
            (rejected_x, rejected_y)
            for rejected_x, rejected_y in self.planner_blocked_goals
            if math.hypot(x - rejected_x, y - rejected_y)
            > PLANNER_BLOCKED_SKIP_RADIUS_M
        ]

    def abandon_main_goal_planner_blocked(self, error_code: int):
        if self.main_goal is None:
            return

        x, y = self.main_goal

        # Check planner suppression only; a temporary 105 cooldown for a nearby
        # point must not prevent recording a real 206/208 planner failure.
        already_planner_blocked = any(
            math.hypot(x - rejected_x, y - rejected_y)
            <= PLANNER_BLOCKED_SKIP_RADIUS_M
            for rejected_x, rejected_y in self.planner_blocked_goals
        )
        if not already_planner_blocked:
            self.planner_blocked_goals.append((x, y))

        if error_code == GOAL_OCCUPIED_ERROR_CODE:
            error_name = "GOAL_OCCUPIED"
            status_reason = "main_goal_abandoned_goal_occupied"
        else:
            error_name = "NO_VALID_PATH"
            status_reason = "main_goal_abandoned_no_valid_path"

        self.get_logger().warn(
            f"Main frontier abandoned after {error_name} ({error_code}): "
            f"x={x:.2f}, y={y:.2f}; candidates within "
            f"{PLANNER_BLOCKED_SKIP_RADIUS_M:.2f} m are skipped during normal "
            "selection but will be checked again by terminal planner revalidation."
        )

        self.main_goal = None
        self.latest_main_plan = None
        self.recovery_count = 0
        self.execution_failure_count = 0
        self.recovery_trigger_error_code = None
        self.publish_status(
            "EXPLORING",
            reason=status_reason,
            abandoned_x=round(x, 4),
            abandoned_y=round(y, 4),
            error_code=error_code,
            skip_radius_m=PLANNER_BLOCKED_SKIP_RADIUS_M,
            abandoned_count=len(self.planner_blocked_goals),
        )

    @staticmethod
    def candidate_signature(candidates):
        return tuple(
            sorted(
                (
                    round(candidate[1], 2),
                    round(candidate[2], 2),
                    int(candidate[3]),
                    int(candidate[4]),
                    int(candidate[5]),
                )
                for candidate in candidates
            )
        )

    def maybe_start_planner_revalidation(self, candidates):
        if self.completed or self.goal_active or self.revalidation_active:
            return

        # A 105 cooldown is deliberately temporary execution evidence, not proof
        # that the frontier is planner-unreachable. Do not let MapEx/Nearest's
        # existing "all suppressed" branch immediately revive the same goal via
        # ComputePathToPose. Wait for the cooldown to expire.
        if self.candidates_have_execution_cooldown(candidates):
            remaining_s = max(
                (
                    self.execution_blocked_remaining_s(candidate[1], candidate[2])
                    for candidate in candidates
                ),
                default=0.0,
            )
            self.reset_completion_verification(
                "execution_cooldown_active"
            )
            self.publish_status(
                "WAITING_EXECUTION_COOLDOWN",
                reason="failed_to_make_progress_frontier_temporarily_suppressed",
                candidate_count=len(candidates),
                remaining_cooldown_s=round(max(0.0, remaining_s), 3),
            )
            return

        now_s = self.now_s()
        if now_s - self.revalidation_last_start_s < COMPLETION_SWEEP_INTERVAL_S:
            return

        if not self.planner_client.server_is_ready():
            self.get_logger().warn(
                "ComputePathToPose action server is not ready for terminal "
                "planner revalidation."
            )
            return

        signature = self.candidate_signature(candidates)
        if signature != self.revalidation_signature:
            self.reset_completion_verification(
                "planner_revalidation_candidate_set_changed"
            )
            self.revalidation_signature = signature

        self.revalidation_active = True
        self.revalidation_candidates = sorted(candidates, key=lambda item: item[0])
        self.revalidation_index = 0
        self.revalidation_last_start_s = now_s

        self.publish_status(
            "VERIFYING_COMPLETE",
            reason="planner_revalidation_sweep",
            candidate_count=len(self.revalidation_candidates),
            stable_sweeps=self.completion_streak,
            required_sweeps=COMPLETION_REQUIRED_SWEEPS,
        )
        self.get_logger().warn(
            "No normally selectable frontier remains; starting planner "
            f"revalidation sweep over {len(self.revalidation_candidates)} candidates."
        )
        self.send_next_planner_revalidation_goal()

    def send_next_planner_revalidation_goal(self):
        if not self.revalidation_active or self.completed:
            return

        if self.revalidation_index >= len(self.revalidation_candidates):
            self.finish_exhausted_planner_revalidation_sweep()
            return

        candidate = self.revalidation_candidates[self.revalidation_index]
        _distance, x, y, _row, _col, _region_size = candidate

        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = MAP_FRAME
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = x
        goal.goal.pose.position.y = y
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = "GridBased"
        goal.use_start = False

        future = self.planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done_future, checked_candidate=candidate: (
                self.planner_revalidation_goal_response(
                    done_future,
                    checked_candidate,
                )
            )
        )

    def planner_revalidation_goal_response(self, future, candidate):
        if not self.revalidation_active or self.completed:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Planner revalidation request failed: {exc}"
            )
            self.revalidation_index += 1
            self.send_next_planner_revalidation_goal()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Planner revalidation goal rejected")
            self.revalidation_index += 1
            self.send_next_planner_revalidation_goal()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, checked_candidate=candidate: (
                self.planner_revalidation_result_callback(
                    done_future,
                    checked_candidate,
                )
            )
        )

    def planner_revalidation_result_callback(self, future, candidate):
        if not self.revalidation_active or self.completed:
            return

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
            path = getattr(result, "path", None)
            path_nonempty = path is not None and len(path.poses) > 0
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Planner revalidation result failed: {exc}"
            )
            status = None
            path_nonempty = False

        _distance, x, y, _row, _col, _region_size = candidate
        reachable = status == GoalStatus.STATUS_SUCCEEDED and path_nonempty

        if reachable:
            self.get_logger().warn(
                "Planner revalidation found a reachable frontier again: "
                f"x={x:.2f}, y={y:.2f}. Resuming exploration."
            )
            self.clear_planner_blocked_suppression_near(x, y)
            self.revalidation_active = False
            self.revalidation_candidates = []
            self.revalidation_index = 0
            self.revalidation_signature = None
            self.reset_completion_verification(
                "planner_revalidation_found_reachable_frontier"
            )
            return

        self.revalidation_index += 1
        self.send_next_planner_revalidation_goal()

    def finish_exhausted_planner_revalidation_sweep(self):
        if not self.revalidation_active or self.completed:
            return

        self.revalidation_active = False
        candidate_count = len(self.revalidation_candidates)
        self.revalidation_candidates = []
        self.revalidation_index = 0

        now_s = self.now_s()
        self.completion_last_sweep_s = now_s
        if self.completion_reason != "no_planner_reachable_frontier":
            self.completion_reason = "no_planner_reachable_frontier"
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
            candidate_count=candidate_count,
            idle_s=round(idle_s, 3),
            required_idle_s=COMPLETION_IDLE_S,
            age_s=round(age_s, 3),
            startup_grace_s=COMPLETION_STARTUP_GRACE_S,
        )

        self.get_logger().warn(
            "Planner-reachability completion verification: "
            f"{self.completion_streak}/{COMPLETION_REQUIRED_SWEEPS} exhausted "
            f"sweeps across {candidate_count} candidates, idle={idle_s:.1f}s, "
            f"age={age_s:.1f}s"
        )

        if (
            self.completion_streak >= COMPLETION_REQUIRED_SWEEPS
            and idle_s >= COMPLETION_IDLE_S
            and age_s >= COMPLETION_STARTUP_GRACE_S
        ):
            self.mark_exploration_complete()

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

        if self.map_msg is None or self.goal_active or self.revalidation_active:
            return

        # Most execution failures keep the selected frontier pending so path-guided
        # recovery can retry it. 105 may clear main_goal when its bounded recovery
        # budget is exhausted; 206/208 clear it immediately as planner-blocked.
        if self.main_goal is not None:
            self.reset_completion_verification("main_frontier_still_pending")
            self.revalidation_signature = None
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

        if not regions:
            self.clear_goal_markers()
            self.revalidation_signature = None
            self.observe_no_frontier_terminal_state()
            return

        robot_x, robot_y = robot
        raw_candidates = []

        for region in regions:
            row, col = self.representative(region)
            x, y = self.cell_to_world(row, col)
            distance = math.hypot(x - robot_x, y - robot_y)
            raw_candidates.append((distance, x, y, row, col, len(region)))

        distant_candidates = [
            candidate
            for candidate in raw_candidates
            if candidate[0] >= MIN_DISTANCE_THRESHOLD
        ]
        near_frontier_fallback = not distant_candidates
        all_candidates = (
            distant_candidates if distant_candidates else raw_candidates
        )

        if near_frontier_fallback:
            self.get_logger().info(
                "All current frontier representatives are closer than "
                f"{MIN_DISTANCE_THRESHOLD:.2f} m; enabling near-frontier fallback."
            )

        candidates = []
        suppressed_planner_blocked = 0
        for candidate in all_candidates:
            _distance, x, y, _row, _col, _region_size = candidate
            if self.is_planner_blocked_suppressed(x, y):
                suppressed_planner_blocked += 1
                continue
            candidates.append(candidate)

        if not candidates:
            self.clear_goal_markers()

            # This call refuses terminal revalidation while any candidate is
            # under a temporary 105 execution cooldown.
            if (
                all_candidates
                and suppressed_planner_blocked == len(all_candidates)
            ):
                self.maybe_start_planner_revalidation(all_candidates)
                return

            self.revalidation_signature = None
            self.reset_completion_verification("frontier_region_present")
            self.publish_status(
                "EXPLORING",
                reason="frontier_candidates_present_but_not_selectable",
                frontier_regions=len(regions),
                candidate_count=len(all_candidates),
                preferred_min_distance_m=MIN_DISTANCE_THRESHOLD,
                near_frontier_fallback=near_frontier_fallback,
                suppressed_planner_blocked=suppressed_planner_blocked,
            )
            return

        self.revalidation_signature = None
        self.reset_completion_verification("planner_candidate_available")

        selected = min(candidates, key=lambda item: item[0])
        distance, x, y, _row, _col, region_size = selected

        self.publish_goal_markers(candidates, selected)

        self.get_logger().info(
            f"Selected nearest frontier: x={x:.2f}, y={y:.2f}, "
            f"distance={distance:.2f} m, region={region_size} cells, "
            f"near_fallback={near_frontier_fallback}"
        )

        self.main_goal = (x, y)
        self.latest_main_plan = None
        self.recovery_count = 0
        self.execution_failure_count = 0
        self.recovery_trigger_error_code = None
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
            execution_failure_count=self.execution_failure_count,
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
                self.recovery_trigger_error_code = None
                if self.main_goal is not None:
                    x, y = self.main_goal
                    self.send_navigation_goal(x, y, mode="main")
                return

            self.get_logger().info("Goal reached")
            self.main_goal = None
            self.latest_main_plan = None
            self.recovery_count = 0
            self.execution_failure_count = 0
            self.recovery_trigger_error_code = None
            self.revalidation_signature = None
            self.publish_status("EXPLORING", reason="main_goal_reached")
            return

        detail = f"status={status}"
        if error_code is not None:
            detail += f", error_code={error_code}"
        if error_msg:
            detail += f", error_msg={error_msg}"

        self.get_logger().warn(f"{mode.capitalize()} goal failed ({detail})")

        # Planner failures keep their existing semantics.
        if mode == "main" and error_code in PLANNER_BLOCKING_ERROR_CODES:
            self.abandon_main_goal_planner_blocked(error_code)
            return

        # Bounded handling for FAILED_TO_MAKE_PROGRESS.
        if (
            mode == "main"
            and error_code == FAILED_TO_MAKE_PROGRESS_ERROR_CODE
        ):
            self.execution_failure_count += 1

            if self.execution_failure_count >= EXECUTION_FAILURE_LIMIT:
                self.suppress_main_goal_execution_failed(
                    error_code,
                    reason="repeated_failed_to_make_progress",
                )
                return

            recovery_started = self.try_path_guided_subgoal(
                trigger_error_code=error_code,
            )
            if not recovery_started:
                self.suppress_main_goal_execution_failed(
                    error_code,
                    reason="no_useful_path_guided_subgoal",
                )
            return

        if mode == "main":
            self.try_path_guided_subgoal(trigger_error_code=error_code)
            return

        # If a subgoal launched specifically to recover from 105 also fails,
        # that recovery attempt is exhausted: cool down the main frontier now.
        if (
            mode == "subgoal"
            and self.recovery_trigger_error_code
            == FAILED_TO_MAKE_PROGRESS_ERROR_CODE
        ):
            self.suppress_main_goal_execution_failed(
                FAILED_TO_MAKE_PROGRESS_ERROR_CODE,
                reason="failed_to_make_progress_recovery_subgoal_failed",
            )
            return

        # Preserve old behavior for recovery subgoals triggered by other errors.
        self.recovery_trigger_error_code = None
        self.get_logger().warn(
            "Recovery subgoal also failed; keeping the same main frontier."
        )

    def try_path_guided_subgoal(self, trigger_error_code=None) -> bool:
        if self.main_goal is None or self.completed:
            return False

        plan = self.latest_main_plan
        if plan is None or len(plan.poses) < 2:
            self.get_logger().warn(
                "Main goal failed but no usable main-goal /plan is available."
            )
            return False

        subgoal = self.select_subgoal_on_path(plan)
        if subgoal is None:
            self.get_logger().warn(
                "Could not choose a useful intermediate point on the main path."
            )
            return False

        x, y, along_path_m, remaining_path_m = subgoal
        self.recovery_count += 1
        self.recovery_trigger_error_code = trigger_error_code

        self.get_logger().warn(
            f"Path-guided recovery #{self.recovery_count}: "
            f"temporary subgoal x={x:.2f}, y={y:.2f}, "
            f"{along_path_m:.2f} m ahead on main path "
            f"(remaining main path={remaining_path_m:.2f} m, "
            f"trigger_error={trigger_error_code})"
        )
        self.send_navigation_goal(x, y, mode="subgoal")
        return True

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
