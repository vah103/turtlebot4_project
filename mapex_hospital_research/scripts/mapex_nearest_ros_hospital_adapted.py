#!/usr/bin/env python3
"""Hospital research adaptation for MapEx-nearest.

The pinned MapEx implementation uses ``cur_pose_dist_threshold_m=1.0`` as a
post-ranking validity check. Hospital pilot ``nearest_pilot_005`` showed a
startup deadlock: the only large frontier region had one representative at
0.469 m, so the original rule rejected the only candidate before Nav2 was even
queried.

For Hospital benchmark runs we therefore preserve MapEx frontier generation and
Euclidean ranking, but DO NOT reject a ranked candidate only because it is
closer than 1 m. Every unsuppressed ranked candidate is passed to Nav2
ComputePathToPose in score order. This adaptation must be shared by Nearest and
full MapEx Hospital runs.

The original distance is still logged as ``below_1m`` for audit/replay.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

from mapex_nearest_ros_research import ResearchHospitalMapExNearestROS


class HospitalAdaptedNearestROS(ResearchHospitalMapExNearestROS):
    """Research Nearest runtime with Hospital-specific 1 m bypass."""

    def __init__(self, run_id: str) -> None:
        super().__init__(run_id)
        self.get_logger().warning(
            "HOSPITAL ADAPTATION ACTIVE: MapEx cur_pose_dist_threshold_m=1.0 is "
            "NOT enforced. Ranked frontiers below 1 m are sent to Nav2 normally; "
            "their below_1m status remains logged."
        )

    @staticmethod
    def _nav2_audit_from_rows(rows: list[dict]) -> dict[str, int]:
        """Summarize Nav2 planner outcomes for one frozen policy decision."""
        path_success = sum(bool(item.get("selected")) for item in rows)
        no_path = sum(item.get("status") == "nav2_no_path" for item in rows)
        rejected = sum(item.get("status") == "nav2_rejected" for item in rows)
        errors = sum(
            item.get("status") in {"nav2_request_error", "nav2_result_error"}
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

            # This is the exact terminal state for a non-empty candidate set:
            # every ranked candidate has already been submitted to Nav2 and none
            # produced a usable path.  Persist it BEFORE entering the completion
            # window, regardless of whatever transient outcome was written last.
            if self._current_policy is not None:
                self._refresh_nav2_audit_fields()
                self._current_policy["outcome"] = (
                    "no_nav2_reachable_ranked_candidate"
                )
                self._current_policy["terminal_reason"] = (
                    "all_ranked_candidates_failed_nav2_path_validation"
                )
                self._write_policy_files()

            self.get_logger().warning(
                "All current ranked MapEx frontier centers failed Nav2 path "
                "validation; terminal policy state saved before completion window"
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
        goal_pose.pose.orientation.w = 1.0

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
