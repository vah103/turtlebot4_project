#!/usr/bin/env python3
"""Hospital research adaptation for MapEx-nearest.

Hospital preserves the pinned MapEx frontier generation and Euclidean ranking,
but adapts the execution layer to a live TurtleBot4/Nav2 stack:

- the original MapEx post-ranking <1 m rejection is disabled because it deadlocks
  at Hospital startup;
- every ranked candidate is validated with Nav2 ComputePathToPose;
- planner action status must be SUCCEEDED as well as returning a non-empty path;
- a stable non-empty frontier set is periodically revalidated instead of treating
  one transient no-path result as permanent evidence;
- execution failures are suppressed only for a bounded cooldown, not forever;
- frontier goals have no policy-defined yaw.  The planner request preserves the
  robot's current yaw as a neutral seed, while the Hospital Nav2 goal checker is
  configured to ignore final yaw.

These execution adaptations must be shared by Nearest, full MapEx and any
proposed Hospital method used in the comparison.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import ComputePathToPose
from std_msgs.msg import String

from mapex_nearest_ros_research import ResearchHospitalMapExNearestROS


PLANNER_REVALIDATE_PERIOD_S = 2.0
EXECUTION_FAILURE_COOLDOWN_S = 30.0
EXECUTION_FAILURE_RADIUS_M = 0.25


class HospitalAdaptedNearestROS(ResearchHospitalMapExNearestROS):
    """Research Nearest runtime with Hospital-specific execution adaptations."""

    def __init__(self, run_id: str) -> None:
        self._last_planner_revalidation_sec: float | None = None
        self._execution_failure_cooldowns: list[tuple[float, float, float]] = []
        super().__init__(run_id)
        self.get_logger().warning(
            "HOSPITAL ADAPTATION ACTIVE: MapEx cur_pose_dist_threshold_m=1.0 is "
            "NOT enforced; planner status is checked explicitly; exhausted "
            "frontiers are periodically revalidated; execution failures use a "
            "bounded cooldown; frontier goal yaw is policy-agnostic."
        )

    @staticmethod
    def _nav2_audit_from_rows(rows: list[dict]) -> dict[str, int]:
        """Summarize Nav2 planner outcomes for one frozen policy decision."""
        path_success = sum(bool(item.get("selected")) for item in rows)
        no_path = sum(item.get("status") == "nav2_no_path" for item in rows)
        rejected = sum(item.get("status") == "nav2_rejected" for item in rows)
        errors = sum(
            item.get("status")
            in {
                "nav2_request_error",
                "nav2_result_error",
                "nav2_action_failed",
            }
            for item in rows
        )
        return {
            "nav2_path_success_count": int(path_success),
            "nav2_no_path_count": int(no_path),
            "nav2_rejected_count": int(rejected),
            "nav2_error_count": int(errors),
            "nav2_checked_count": int(path_success + no_path + rejected + errors),
        }

    def _refresh_nav2_audit_fields(self) -> None:
        if self._current_policy is None:
            return
        self._current_policy.update(
            self._nav2_audit_from_rows(self._current_policy.get("candidates", []))
        )

    def _is_execution_suppressed(self, x: float, y: float) -> bool:
        """Suppress a failed execution region only until its cooldown expires."""
        now = self._now_sec()
        self._execution_failure_cooldowns = [
            item for item in self._execution_failure_cooldowns if item[2] > now
        ]
        return any(
            math.hypot(x - fx, y - fy) <= EXECUTION_FAILURE_RADIUS_M
            for fx, fy, _expires in self._execution_failure_cooldowns
        )

    def _tick(self) -> None:
        """Revalidate stable non-empty exhausted frontiers at a fixed cadence."""
        if self.complete or self.active or self.planning or self.latest_map is None:
            return
        if not self.planner.server_is_ready():
            if not self.planner_wait_notice:
                self.get_logger().info("Waiting for Nav2 ComputePathToPose action server")
                self.planner_wait_notice = True
            return
        self.planner_wait_notice = False

        msg = self.latest_map
        frame = msg.header.frame_id or "map"
        robot_xy = self._robot_xy(frame)
        if robot_xy is None:
            return

        candidates = self._compute_candidates(msg, robot_xy)
        signature = self._signature(msg, candidates)
        self._publish_candidate_count(len(candidates))
        same_exhausted = self.exhausted_signature == signature
        now = self._now_sec()

        if same_exhausted and not candidates:
            # There is nothing for the planner to revalidate. Candidate generation
            # itself is rerun every tick, so any map change that creates a frontier
            # immediately breaks this condition.
            self._observe_exhausted()
            return

        revalidating_exhaustion = False
        if same_exhausted and candidates:
            if (
                self._last_planner_revalidation_sec is not None
                and now - self._last_planner_revalidation_sec
                < PLANNER_REVALIDATE_PERIOD_S
            ):
                return
            revalidating_exhaustion = True
            self._last_planner_revalidation_sec = now
            self.get_logger().info(
                "Revalidating unchanged ranked frontier set with Nav2; previous "
                "no-path evidence is treated as transient"
            )
        elif not same_exhausted:
            self._last_planner_revalidation_sec = None

        self.decision_map = msg
        self.current_candidates = candidates
        self.current_candidate_signature = signature
        self.planning_index = 0

        if not candidates:
            self.exhausted_signature = signature
            self._observe_exhausted()
            return

        if not revalidating_exhaustion:
            self._reset_idle()
            self.first_ready_sec = None
        self._plan_next_candidate()

    def _plan_next_candidate(self) -> None:
        """Try ranked candidates without the original MapEx 1 m rejection."""
        self._begin_policy_decision_if_needed()

        if self.decision_map is None:
            self.planning = False
            self._viz_checking_candidate = None
            return

        if self._current_policy is not None:
            self._current_policy["hospital_1m_rule_enforced"] = False
            self._current_policy["mapex_original_cur_pose_dist_threshold_m"] = 1.0

        if self.planning_index >= len(self.current_candidates):
            self.planning = False
            self._viz_checking_candidate = None
            self.exhausted_signature = self.current_candidate_signature
            self._last_planner_revalidation_sec = self._now_sec()

            # Exact terminal state for this planner sweep. Completion is only
            # allowed after repeated sweeps of an unchanged non-empty set.
            if self._current_policy is not None:
                self._refresh_nav2_audit_fields()
                self._current_policy["outcome"] = (
                    "no_nav2_reachable_ranked_candidate"
                )
                self._current_policy["terminal_reason"] = (
                    "all_ranked_candidates_failed_nav2_path_validation"
                )
                self._current_policy["planner_revalidation_period_s"] = (
                    PLANNER_REVALIDATE_PERIOD_S
                )
                self._write_policy_files()

            self.get_logger().warning(
                "All current ranked MapEx frontier centers failed this Nav2 path "
                "validation sweep; saved terminal state and will revalidate before "
                "completion"
            )
            self._observe_exhausted()
            self._publish_markers(self.decision_map or self.latest_map)
            return

        candidate = self.current_candidates[self.planning_index]
        self._viz_checking_candidate = candidate

        if self._current_policy is not None:
            record = self._candidate_record(candidate)
            if record is not None:
                record["status"] = (
                    "checking_nav2_below_1m_allowed"
                    if bool(record.get("below_1m"))
                    else "checking_nav2"
                )
            self._planner_started_perf[(candidate.row, candidate.col)] = time.perf_counter()
            self._write_policy_files()

        self.planning_index += 1
        msg = self.decision_map
        frame = msg.header.frame_id or "map"
        x, y = self._cell_to_world(candidate.row, candidate.col, msg)

        if candidate.distance_m < 1.0:
            self.get_logger().warning(
                "Hospital adaptation allowing ranked frontier below 1 m: "
                f"rank={self.planning_index}, distance={candidate.distance_m:.3f} m"
            )

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame
        goal_pose.header.stamp.sec = 0
        goal_pose.header.stamp.nanosec = 0
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        # MapEx frontier policy contains no yaw objective. Preserve current robot
        # yaw as a neutral seed rather than silently demanding yaw=0. Hospital
        # Nav2 additionally ignores final yaw in its goal checker/GoalAngleCritic.
        yaw = self._robot_yaw(frame)
        if yaw is None:
            yaw = 0.0
        goal_pose.pose.orientation.z = math.sin(0.5 * yaw)
        goal_pose.pose.orientation.w = math.cos(0.5 * yaw)

        goal = ComputePathToPose.Goal()
        goal.goal = goal_pose
        goal.planner_id = ""
        goal.use_start = False

        self.planning = True
        future = self.planner.send_goal_async(goal)
        future.add_done_callback(
            lambda done, cand=candidate, xy=(x, y): self._on_plan_goal_response(
                done, cand, xy
            )
        )
        self._publish_markers(self.decision_map or self.latest_map)

    def _on_plan_result(self, future, candidate, frontier_xy) -> None:
        """Accept a candidate only on SUCCEEDED action status + non-empty path."""
        self.planning = False
        record = self._candidate_record(candidate)
        path = None
        wrapped_status: int | None = None
        started = self._planner_started_perf.pop((candidate.row, candidate.col), None)

        try:
            wrapped = future.result()
            wrapped_status = int(wrapped.status)
            if record is not None:
                record["nav2_action_status"] = wrapped_status
                if started is not None:
                    record["planner_check_ms"] = (
                        time.perf_counter() - started
                    ) * 1000.0

            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                if record is not None:
                    record["status"] = "nav2_action_failed"
                self.get_logger().warning(
                    "ComputePathToPose action did not succeed: "
                    f"status={wrapped_status}; trying next ranked frontier"
                )
            else:
                path = wrapped.result.path
                if path is None or not path.poses:
                    if record is not None:
                        record["status"] = "nav2_no_path"
                else:
                    length = 0.0
                    for first, second in zip(path.poses, path.poses[1:]):
                        length += math.hypot(
                            second.pose.position.x - first.pose.position.x,
                            second.pose.position.y - first.pose.position.y,
                        )
                    if record is not None:
                        record["status"] = "selected"
                        record["selected"] = True
                        record["selected_path_length_m"] = length
                    if self._current_policy is not None:
                        self._current_policy["selected_rank"] = (
                            "" if record is None else record.get("policy_rank", "")
                        )
                        self._current_policy["selected_x"] = frontier_xy[0]
                        self._current_policy["selected_y"] = frontier_xy[1]
                        self._current_policy["selected_distance_m"] = candidate.distance_m
                        self._current_policy["selected_path_length_m"] = length
                        self._current_policy["outcome"] = "selected_for_navigation"
        except Exception as exc:  # noqa: BLE001
            if record is not None:
                record["status"] = "nav2_result_error"
                if started is not None and record.get("planner_check_ms", "") == "":
                    record["planner_check_ms"] = (
                        time.perf_counter() - started
                    ) * 1000.0
            self.get_logger().warning(f"ComputePathToPose result failed: {exc}")

        self._write_policy_files()

        valid_path = (
            wrapped_status == GoalStatus.STATUS_SUCCEEDED
            and path is not None
            and bool(path.poses)
        )
        if not valid_path:
            self._viz_checking_candidate = None
            self._plan_next_candidate()
            return

        self.exhausted_signature = None
        self._last_planner_revalidation_sec = None
        self._reset_idle()
        self.started = True
        self.active = True
        self.active_frontier_xy = frontier_xy
        self._viz_checking_candidate = None

        # The planner path is reachability evidence. Manager pairs it with the
        # exact frontier center and executes the exact x/y goal.
        self.path_pub.publish(path)
        point = PointStamped()
        point.header.frame_id = (
            self.decision_map.header.frame_id if self.decision_map is not None else "map"
        ) or "map"
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = frontier_xy[0]
        point.point.y = frontier_xy[1]
        self.frontier_pub.publish(point)

        if self._current_policy is not None:
            msg = String()
            msg.data = self._current_policy["policy_decision_id"]
            self.decision_id_pub.publish(msg)

        self.get_logger().warning(
            "Selected MapEx nearest frontier after successful Nav2 validation: "
            f"row={candidate.row}, col={candidate.col}, "
            f"euclidean_distance={candidate.distance_m:.2f} m"
        )
        self._publish_markers(self.decision_map or self.latest_map)

    def _on_goal_success(self, msg) -> None:
        super()._on_goal_success(msg)
        # A successful move changes the local execution context, so old temporary
        # failures should not bias subsequent frontier selection.
        self._execution_failure_cooldowns.clear()
        self.failed_execution_xy.clear()
        self._last_planner_revalidation_sec = None

    def _on_goal_failure(self, msg) -> None:
        failed_xy = self.active_frontier_xy
        super()._on_goal_failure(msg)

        # Base/reference adapter stores failures until a later success. Hospital
        # online execution instead uses a bounded cooldown so a controller failure
        # cannot permanently convert a planner-reachable frontier into completion.
        self.failed_execution_xy.clear()
        if failed_xy is not None:
            expires = self._now_sec() + EXECUTION_FAILURE_COOLDOWN_S
            self._execution_failure_cooldowns.append(
                (float(failed_xy[0]), float(failed_xy[1]), expires)
            )
            self.get_logger().warning(
                "Planner-valid frontier execution failed; suppressing only for "
                f"{EXECUTION_FAILURE_COOLDOWN_S:.0f}s within "
                f"{EXECUTION_FAILURE_RADIUS_M:.2f}m, then re-enabling it"
            )
        self.exhausted_signature = None
        self._last_planner_revalidation_sec = None

    def _startup_failure_reason(self) -> str:
        """Distance <1 m is not a failure condition in Hospital adaptation."""
        diag = self._last_diag
        if not diag:
            return "mapex_nearest_startup_no_diagnostics"
        if int(diag.get("frontier_cells", 0) or 0) == 0:
            return "mapex_nearest_startup_no_frontier_cells"
        if int(diag.get("large_region_count", 0) or 0) == 0:
            return "mapex_nearest_startup_no_region_larger_than_10_cells"
        if int(diag.get("ranked_candidate_count", 0) or 0) == 0:
            return "mapex_nearest_startup_all_candidates_suppressed"
        return "mapex_nearest_startup_no_nav2_reachable_frontier"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = HospitalAdaptedNearestROS(args.run_id)
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
