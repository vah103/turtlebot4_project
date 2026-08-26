#!/usr/bin/env python3
"""Final Hospital Nearest research wrapper used by benchmark launches.

This layer does not change frontier generation, Euclidean ranking, the Hospital
below-1m adaptation, or Nav2 execution.  It only closes reproducibility gaps
needed before official runs:

- emit the exploration start timestamp *before* first candidate computation;
- save an exact frozen policy state even when the ranked candidate set is empty;
- assign a stable candidate_id within each policy decision;
- publish the selected candidate_id so decisions.csv can join candidates.csv
  without relying on coordinates/rank inference.
"""

from __future__ import annotations

import argparse
import csv
import json

import rclpy
from std_msgs.msg import String

from mapex_nearest_ros_hospital_adapted import HospitalAdaptedNearestROS


START_TOPIC = "/frontier_exploration_start"
SELECTED_CANDIDATE_TOPIC = "/frontier_policy_selected_candidate"


class OfficialHospitalNearestROS(HospitalAdaptedNearestROS):
    def __init__(self, run_id: str) -> None:
        super().__init__(run_id)
        self.start_pub = self.create_publisher(String, START_TOPIC, 10)
        self.selected_candidate_pub = self.create_publisher(
            String, SELECTED_CANDIDATE_TOPIC, 10
        )
        self._start_published = False
        self._last_exact_empty_signature: tuple | None = None
        self.get_logger().info(
            "Official-run logging wrapper active: pre-compute t=0, exact empty "
            "candidate decisions, stable candidate IDs"
        )

    def _publish_start_once(self, msg) -> None:
        if self._start_published:
            return
        self._start_published = True
        payload = {
            "event": "first_policy_decision_before_compute",
            "sim_time_s": float(self._now_sec()),
            "map_stamp_s": float(self._map_stamp_s(msg)),
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self.start_pub.publish(out)
        self.get_logger().info(
            f"Exploration benchmark clock started before first policy compute at "
            f"sim_time={payload['sim_time_s']:.6f}"
        )

    @staticmethod
    def _ensure_candidate_ids(rows: list[dict]) -> None:
        for item in rows:
            raw_rank = int(item.get("raw_rank") or 0)
            item["candidate_id"] = f"candidate_{raw_rank:04d}"

    def _compute_candidates(self, msg, robot_xy):
        self._publish_start_once(msg)
        candidates = super()._compute_candidates(msg, robot_xy)
        self._ensure_candidate_ids(self._last_candidate_audit)

        # Base _tick() returns before _plan_next_candidate() when candidates is
        # empty. Save that exact termination-relevant state here, once per stable
        # empty signature, so offline replay has the actual map + pose + audit.
        if not candidates:
            signature = self._signature(msg, candidates)
            if (
                self.exhausted_signature != signature
                and self._last_exact_empty_signature != signature
            ):
                self.decision_map = msg
                self.current_candidates = []
                self.current_candidate_signature = signature
                self.planning_index = 0
                self._begin_policy_decision_if_needed()
                if self._current_policy is not None:
                    self._current_policy["outcome"] = "exhausted_no_ranked_candidate"
                    self._current_policy["hospital_1m_rule_enforced"] = False
                    self._current_policy[
                        "mapex_original_cur_pose_dist_threshold_m"
                    ] = 1.0
                    self._write_policy_files()
                self._last_exact_empty_signature = signature
                self.get_logger().warning(
                    "Saved exact exhausted policy state with zero ranked candidates"
                )
        else:
            self._last_exact_empty_signature = None
        return candidates

    def _write_policy_files(self) -> None:
        """Write policy tables with explicit candidate IDs and Hospital semantics."""
        if self._current_policy is None:
            return

        decision_id = self._current_policy["policy_decision_id"]
        decision_dir = self.policy_dir / decision_id
        decision_dir.mkdir(parents=True, exist_ok=True)

        candidates = self._current_policy.get("candidates", [])
        self._ensure_candidate_ids(candidates)
        self._current_policy["below_1m_count"] = sum(
            bool(item.get("below_1m")) and not bool(item.get("execution_suppressed"))
            for item in candidates
        )
        self._current_policy.setdefault("hospital_1m_rule_enforced", False)
        self._current_policy.setdefault(
            "mapex_original_cur_pose_dist_threshold_m", 1.0
        )

        selected = next((item for item in candidates if item.get("selected")), None)
        self._current_policy["selected_candidate_id"] = (
            "" if selected is None else selected.get("candidate_id", "")
        )

        (decision_dir / "decision.json").write_text(
            json.dumps(self._current_policy, indent=2) + "\n", encoding="utf-8"
        )

        candidate_fields = [
            "policy_decision_id",
            "candidate_id",
            "raw_rank",
            "policy_rank",
            "row",
            "col",
            "x",
            "y",
            "distance_cells",
            "distance_m",
            "below_1m",
            "execution_suppressed",
            "status",
            "planner_check_ms",
            "selected",
            "selected_path_length_m",
            "execution_result",
        ]
        with (decision_dir / "candidates.csv").open(
            "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=candidate_fields)
            writer.writeheader()
            for item in candidates:
                row = {"policy_decision_id": decision_id, **item}
                writer.writerow({key: row.get(key, "") for key in candidate_fields})

        summary_fields = [
            "policy_decision_id",
            "sim_time_s",
            "map_stamp_s",
            "robot_x",
            "robot_y",
            "robot_yaw",
            "candidate_compute_ms",
            "candidate_count",
            "below_1m_count",
            "hospital_1m_rule_enforced",
            "selected_candidate_id",
            "selected_rank",
            "selected_x",
            "selected_y",
            "selected_distance_m",
            "selected_path_length_m",
            "outcome",
        ]
        with self.policy_summary_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=summary_fields)
            writer.writeheader()
            for item in self._policy_history:
                rows = item.get("candidates", [])
                self._ensure_candidate_ids(rows)
                item["below_1m_count"] = sum(
                    bool(row.get("below_1m"))
                    and not bool(row.get("execution_suppressed"))
                    for row in rows
                )
                item.setdefault("hospital_1m_rule_enforced", False)
                chosen = next((row for row in rows if row.get("selected")), None)
                item["selected_candidate_id"] = (
                    "" if chosen is None else chosen.get("candidate_id", "")
                )
                writer.writerow({key: item.get(key, "") for key in summary_fields})

        with self.global_candidates_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=candidate_fields)
            writer.writeheader()
            for decision in self._policy_history:
                rows = decision.get("candidates", [])
                self._ensure_candidate_ids(rows)
                for item in rows:
                    row = {
                        "policy_decision_id": decision["policy_decision_id"],
                        **item,
                    }
                    writer.writerow(
                        {key: row.get(key, "") for key in candidate_fields}
                    )

    def _on_plan_result(self, future, candidate, frontier_xy) -> None:
        super()._on_plan_result(future, candidate, frontier_xy)
        if self._current_policy is None:
            return
        record = self._candidate_record(candidate)
        if record is None or not record.get("selected"):
            return
        self._ensure_candidate_ids([record])
        payload = {
            "policy_decision_id": self._current_policy["policy_decision_id"],
            "candidate_id": record["candidate_id"],
            "policy_rank": record.get("policy_rank", ""),
            "row": int(candidate.row),
            "col": int(candidate.col),
        }
        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self.selected_candidate_pub.publish(out)
        self._write_policy_files()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = OfficialHospitalNearestROS(args.run_id)
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
