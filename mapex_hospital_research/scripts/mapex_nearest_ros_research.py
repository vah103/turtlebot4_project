#!/usr/bin/env python3
"""Research runtime for the Hospital MapEx-nearest baseline.

This extends ``HospitalMapExNearestROS`` without changing its frontier policy.
The active 1 m semantics remain the official MapEx order used by the Hospital
wrapper: rank every frontier center first, reject the currently selected
candidate if it is <1 m from the robot, then try the next ranked candidate.

This layer adds reproducibility data only:
- exact frozen OccupancyGrid used for each policy decision;
- fixed-canvas copy of that exact map;
- map-frame robot pose used for ranking;
- every frontier candidate, rank, 1 m rejection and Nav2 path status;
- planner timing and selected path length;
- execution success/failure for the selected frontier.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import TransformException

from mapex_nearest_ros import MIN_FRONTIER_DISTANCE_M, FrontierCandidate
from mapex_nearest_ros_hospital import HospitalMapExNearestROS
from research_recorder import (
    CANVAS_HEIGHT,
    CANVAS_ID,
    CANVAS_ORIGIN_X,
    CANVAS_ORIGIN_Y,
    CANVAS_RESOLUTION,
    CANVAS_WIDTH,
)


WORKSPACE = Path(__file__).resolve().parents[1]
DECISION_ID_TOPIC = "/frontier_policy_decision_id"


class ResearchHospitalMapExNearestROS(HospitalMapExNearestROS):
    def __init__(self, run_id: str) -> None:
        super().__init__()
        self.run_id = run_id
        self.run_dir = WORKSPACE / "experiments" / "nearest" / run_id
        if not self.run_dir.exists():
            raise RuntimeError(
                f"Recorder run directory does not exist: {self.run_dir}. "
                "The recorder must start before the policy node."
            )

        self.policy_dir = self.run_dir / "decisions"
        self.policy_dir.mkdir(parents=True, exist_ok=True)
        self.policy_summary_path = self.run_dir / "policy_decisions.csv"
        self.global_candidates_path = self.run_dir / "candidates.csv"
        self.decision_id_pub = self.create_publisher(String, DECISION_ID_TOPIC, 10)

        self._policy_counter = 0
        self._decision_map_token: int | None = None
        self._current_policy: dict | None = None
        self._policy_history: list[dict] = []
        self._last_candidate_audit: list[dict] = []
        self._last_ranking_robot_xy: tuple[float, float] | None = None
        self._last_candidate_compute_ms: float | None = None
        self._planner_started_perf: dict[tuple[int, int], float] = {}

        self.get_logger().info(
            "Exact research logging active: frozen decision map, map-frame pose, "
            "all ranked candidates, 1 m rejections and Nav2 path outcomes"
        )

    @staticmethod
    def _map_stamp_s(msg) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9

    def _robot_yaw(self, frame: str) -> float | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                frame,
                str(self.get_parameter("robot_frame").value),
                Time(),
            )
        except TransformException:
            return None
        q = tf.transform.rotation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _map_on_fixed_canvas(self, msg) -> np.ndarray:
        if not math.isclose(
            float(msg.info.resolution), CANVAS_RESOLUTION, rel_tol=1e-6, abs_tol=1e-9
        ):
            raise RuntimeError(
                f"Decision map resolution changed: {msg.info.resolution} != "
                f"{CANVAS_RESOLUTION}"
            )
        if abs(self._origin_yaw(msg)) > 1e-5:
            raise RuntimeError("Decision map origin is not axis-aligned")

        source = np.asarray(msg.data, dtype=np.int8).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        canvas = np.full((CANVAS_HEIGHT, CANVAS_WIDTH), -1, dtype=np.int8)
        xoff = math.floor(
            (float(msg.info.origin.position.x) - CANVAS_ORIGIN_X)
            / CANVAS_RESOLUTION
            + 0.5
        )
        yoff = math.floor(
            (float(msg.info.origin.position.y) - CANVAS_ORIGIN_Y)
            / CANVAS_RESOLUTION
            + 0.5
        )
        src_x0 = max(0, -xoff)
        src_y0 = max(0, -yoff)
        dst_x0 = max(0, xoff)
        dst_y0 = max(0, yoff)
        copy_w = min(source.shape[1] - src_x0, CANVAS_WIDTH - dst_x0)
        copy_h = min(source.shape[0] - src_y0, CANVAS_HEIGHT - dst_y0)
        if copy_w > 0 and copy_h > 0:
            canvas[
                dst_y0 : dst_y0 + copy_h,
                dst_x0 : dst_x0 + copy_w,
            ] = source[
                src_y0 : src_y0 + copy_h,
                src_x0 : src_x0 + copy_w,
            ]
        return canvas

    def _compute_candidates(self, msg, robot_xy):
        started = time.perf_counter()
        candidates = super()._compute_candidates(msg, robot_xy)
        self._last_candidate_compute_ms = (time.perf_counter() - started) * 1000.0
        self._last_ranking_robot_xy = (float(robot_xy[0]), float(robot_xy[1]))

        grid = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        centers = self._region_representatives(self._frontier_mask(grid))
        robot_row, robot_col = self._world_to_grid_float(
            robot_xy[0], robot_xy[1], msg
        )
        res = float(msg.info.resolution)
        rows: list[dict] = []
        for row, col in centers:
            distance_cells = math.hypot(row - robot_row, col - robot_col)
            distance_m = distance_cells * res
            x, y = self._cell_to_world(row, col, msg)
            suppressed = self._is_execution_suppressed(x, y)
            rows.append(
                {
                    "row": int(row),
                    "col": int(col),
                    "x": float(x),
                    "y": float(y),
                    "distance_cells": float(distance_cells),
                    "distance_m": float(distance_m),
                    "below_1m": bool(distance_m < MIN_FRONTIER_DISTANCE_M),
                    "execution_suppressed": bool(suppressed),
                    "raw_rank": 0,
                    "policy_rank": "",
                    "status": "execution_suppressed" if suppressed else "ranked",
                    "planner_check_ms": "",
                    "selected": False,
                    "selected_path_length_m": "",
                    "execution_result": "",
                }
            )
        rows.sort(key=lambda item: item["distance_cells"])
        policy_rank = 0
        for raw_rank, item in enumerate(rows, start=1):
            item["raw_rank"] = raw_rank
            if not item["execution_suppressed"]:
                policy_rank += 1
                item["policy_rank"] = policy_rank
        self._last_candidate_audit = rows
        return candidates

    def _candidate_record(self, candidate: FrontierCandidate) -> dict | None:
        if self._current_policy is None:
            return None
        for item in self._current_policy["candidates"]:
            if item["row"] == candidate.row and item["col"] == candidate.col:
                return item
        return None

    def _write_policy_files(self) -> None:
        if self._current_policy is None:
            return
        decision_id = self._current_policy["policy_decision_id"]
        decision_dir = self.policy_dir / decision_id
        decision_dir.mkdir(parents=True, exist_ok=True)
        (decision_dir / "decision.json").write_text(
            json.dumps(self._current_policy, indent=2) + "\n", encoding="utf-8"
        )

        candidate_fields = [
            "policy_decision_id",
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
            for item in self._current_policy["candidates"]:
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
            "valid_ge_1m_count",
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
                writer.writerow({key: item.get(key, "") for key in summary_fields})

        with self.global_candidates_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=candidate_fields)
            writer.writeheader()
            for decision in self._policy_history:
                for item in decision.get("candidates", []):
                    row = {"policy_decision_id": decision["policy_decision_id"], **item}
                    writer.writerow({key: row.get(key, "") for key in candidate_fields})

    def _begin_policy_decision_if_needed(self) -> None:
        if self.decision_map is None:
            return
        token = id(self.decision_map)
        if self._decision_map_token == token:
            return

        self._decision_map_token = token
        self._policy_counter += 1
        decision_id = f"policy_decision_{self._policy_counter:06d}"
        frame = self.decision_map.header.frame_id or "map"
        robot_xy = self._last_ranking_robot_xy or (float("nan"), float("nan"))
        robot_yaw = self._robot_yaw(frame)
        decision_dir = self.policy_dir / decision_id
        decision_dir.mkdir(parents=True, exist_ok=True)

        raw = np.asarray(self.decision_map.data, dtype=np.int8).reshape(
            int(self.decision_map.info.height), int(self.decision_map.info.width)
        )
        np.savez_compressed(
            decision_dir / "observed_map_raw.npz",
            data=raw,
            resolution=np.float64(self.decision_map.info.resolution),
            width=np.int32(self.decision_map.info.width),
            height=np.int32(self.decision_map.info.height),
            origin_x=np.float64(self.decision_map.info.origin.position.x),
            origin_y=np.float64(self.decision_map.info.origin.position.y),
            origin_yaw=np.float64(self._origin_yaw(self.decision_map)),
            frame_id=np.asarray(frame),
            source_stamp_s=np.float64(self._map_stamp_s(self.decision_map)),
        )
        canvas = self._map_on_fixed_canvas(self.decision_map)
        np.savez_compressed(
            decision_dir / "observed_map_canvas.npz",
            data=canvas,
            resolution=np.float64(CANVAS_RESOLUTION),
            width=np.int32(CANVAS_WIDTH),
            height=np.int32(CANVAS_HEIGHT),
            origin_x=np.float64(CANVAS_ORIGIN_X),
            origin_y=np.float64(CANVAS_ORIGIN_Y),
            canvas_id=np.asarray(CANVAS_ID),
        )

        self._current_policy = {
            "policy_decision_id": decision_id,
            "sim_time_s": self._now_sec(),
            "map_stamp_s": self._map_stamp_s(self.decision_map),
            "map_frame": frame,
            "robot_x": robot_xy[0],
            "robot_y": robot_xy[1],
            "robot_yaw": robot_yaw,
            "candidate_compute_ms": self._last_candidate_compute_ms,
            "candidate_count": len(self.current_candidates),
            "valid_ge_1m_count": sum(
                1
                for item in self.current_candidates
                if item.distance_m >= MIN_FRONTIER_DISTANCE_M
            ),
            "selected_rank": "",
            "selected_x": "",
            "selected_y": "",
            "selected_distance_m": "",
            "selected_path_length_m": "",
            "outcome": "planning",
            "raw_map_file": f"decisions/{decision_id}/observed_map_raw.npz",
            "canvas_map_file": f"decisions/{decision_id}/observed_map_canvas.npz",
            "candidates": [dict(item) for item in self._last_candidate_audit],
        }
        self._policy_history.append(self._current_policy)
        self._write_policy_files()
        self.get_logger().info(
            f"Saved exact frozen policy state: {decision_id}, "
            f"candidates={len(self.current_candidates)}"
        )

    def _plan_next_candidate(self) -> None:
        self._begin_policy_decision_if_needed()

        if self._current_policy is not None:
            idx = self.planning_index
            while idx < len(self.current_candidates):
                candidate = self.current_candidates[idx]
                if candidate.distance_m >= MIN_FRONTIER_DISTANCE_M:
                    break
                record = self._candidate_record(candidate)
                if record is not None:
                    record["status"] = "rejected_lt_1m"
                idx += 1

            if idx < len(self.current_candidates):
                candidate = self.current_candidates[idx]
                record = self._candidate_record(candidate)
                if record is not None and record["status"] not in {
                    "nav2_no_path",
                    "nav2_rejected",
                    "nav2_request_error",
                    "nav2_result_error",
                }:
                    record["status"] = "checking_nav2"
                self._planner_started_perf[(candidate.row, candidate.col)] = time.perf_counter()
            else:
                if self._current_policy.get("outcome") == "planning":
                    self._current_policy["outcome"] = "no_candidate_after_validity_or_path_checks"
            self._write_policy_files()

        # HospitalMapExNearestROS performs the official order here:
        # rank all -> reject <1 m -> next candidate -> Nav2 path validation.
        super()._plan_next_candidate()

    def _on_plan_goal_response(self, future, candidate, frontier_xy) -> None:
        record = self._candidate_record(candidate)
        try:
            handle = future.result()
            if not handle.accepted and record is not None:
                record["status"] = "nav2_rejected"
                started = self._planner_started_perf.pop((candidate.row, candidate.col), None)
                if started is not None:
                    record["planner_check_ms"] = (time.perf_counter() - started) * 1000.0
                self._write_policy_files()
        except Exception:
            if record is not None:
                record["status"] = "nav2_request_error"
                started = self._planner_started_perf.pop((candidate.row, candidate.col), None)
                if started is not None:
                    record["planner_check_ms"] = (time.perf_counter() - started) * 1000.0
                self._write_policy_files()
        super()._on_plan_goal_response(future, candidate, frontier_xy)

    def _on_plan_result(self, future, candidate, frontier_xy) -> None:
        record = self._candidate_record(candidate)
        path = None
        try:
            wrapped = future.result()
            path = wrapped.result.path
            started = self._planner_started_perf.pop((candidate.row, candidate.col), None)
            if record is not None and started is not None:
                record["planner_check_ms"] = (time.perf_counter() - started) * 1000.0
            if path is None or not path.poses:
                if record is not None:
                    record["status"] = "nav2_no_path"
            else:
                if record is not None:
                    record["status"] = "selected"
                    record["selected"] = True
                    length = 0.0
                    for first, second in zip(path.poses, path.poses[1:]):
                        length += math.hypot(
                            second.pose.position.x - first.pose.position.x,
                            second.pose.position.y - first.pose.position.y,
                        )
                    record["selected_path_length_m"] = length
                if self._current_policy is not None:
                    self._current_policy["selected_rank"] = (
                        "" if record is None else record.get("policy_rank", "")
                    )
                    self._current_policy["selected_x"] = frontier_xy[0]
                    self._current_policy["selected_y"] = frontier_xy[1]
                    self._current_policy["selected_distance_m"] = candidate.distance_m
                    self._current_policy["selected_path_length_m"] = (
                        "" if record is None else record.get("selected_path_length_m", "")
                    )
                    self._current_policy["outcome"] = "selected_for_navigation"
        except Exception:
            if record is not None:
                record["status"] = "nav2_result_error"
                started = self._planner_started_perf.pop((candidate.row, candidate.col), None)
                if started is not None:
                    record["planner_check_ms"] = (time.perf_counter() - started) * 1000.0
        self._write_policy_files()

        valid_path = path is not None and bool(path.poses)
        super()._on_plan_result(future, candidate, frontier_xy)
        if valid_path and self._current_policy is not None:
            msg = String()
            msg.data = self._current_policy["policy_decision_id"]
            self.decision_id_pub.publish(msg)

    def _on_goal_success(self, msg) -> None:
        if self._current_policy is not None:
            self._current_policy["outcome"] = "navigation_succeeded"
            for item in self._current_policy["candidates"]:
                if item.get("selected"):
                    item["execution_result"] = "SUCCEEDED"
            self._write_policy_files()
        super()._on_goal_success(msg)
        self._decision_map_token = None

    def _on_goal_failure(self, msg) -> None:
        if self._current_policy is not None:
            self._current_policy["outcome"] = "navigation_failed"
            for item in self._current_policy["candidates"]:
                if item.get("selected"):
                    item["execution_result"] = "FAILED"
            self._write_policy_files()
        super()._on_goal_failure(msg)
        self._decision_map_token = None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = ResearchHospitalMapExNearestROS(args.run_id)
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
