#!/usr/bin/env python3
"""Stage-2 full nearest-frontier baseline runner.

Runs the existing ``mapex_nearest_simple.py`` policy unchanged, but subclasses it
only to emit structured policy events. A second ROS node records the Stage-2
baseline data needed for later comparison with full MapEx:

- coverage / known fraction versus time and odometry distance
- total elapsed time and odometry distance
- frontier goals attempted / succeeded / failed and success rate
- nearest-policy computation time for every decision
- termination reason
- periodic and final raw OccupancyGrid snapshots for offline IoU / TU

Heavy run data is written under:
  mapex_hospital_research/experiments/nearest/<run_id>/

The canonical ROI mask is intentionally gitignored. If the local frozen mask is
available, exact Hospital coverage is recorded. If it is missing, coverage is
NaN but maps are still retained so coverage can be recomputed after restoring
that frozen mask.

Hospital v2 runs the policy/SLAM grid at 0.10 m/cell while retaining the frozen
0.05 m evaluation canvas. Runtime cells are expanded to the canonical canvas by
nearest-neighbour area preservation before coverage is computed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from mapex_nearest_simple import FrontierCandidate, MapExNearestSimple


CANVAS_RESOLUTION_M = 0.05
CANVAS_WIDTH = 1504
CANVAS_HEIGHT = 2123
CANVAS_ORIGIN_X_M = -25.6
CANVAS_ORIGIN_Y_M = -60.1
ROI_DENOMINATOR = 215435
METRICS_PERIOD_S = 1.0
MAP_SNAPSHOT_PERIOD_S = 10.0
TERMINAL_SWEEPS = 5
TERMINAL_IDLE_S = 10.0
TRAJECTORY_PERIOD_S = 0.20
EVENT_TOPIC = "/nearest_frontier/events"


class InstrumentedNearest(MapExNearestSimple):
    """Existing nearest policy plus non-invasive structured event publishing."""

    def __init__(self) -> None:
        super().__init__()
        self.event_pub = self.create_publisher(String, EVENT_TOPIC, 20)
        self.full_decision_id = 0
        self.full_goal_open = False
        self.full_goal_id = 0

    def _emit(self, event: str, **fields: Any) -> None:
        payload = {
            "event": event,
            "sim_time_s": self._now_s(),
            **fields,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.event_pub.publish(msg)

    @staticmethod
    def _candidate_signature(candidates: list[FrontierCandidate]) -> str:
        return ";".join(f"{c.row}:{c.col}" for c in candidates)

    def _rank_candidates(
        self, msg: OccupancyGrid, robot_x: float, robot_y: float
    ) -> list[FrontierCandidate]:
        decision_start_sim_s = self._now_s()
        t0 = time.perf_counter()
        candidates = super()._rank_candidates(msg, robot_x, robot_y)
        computation_ms = (time.perf_counter() - t0) * 1000.0

        self.full_decision_id += 1
        signature = self._candidate_signature(candidates)
        self._emit(
            "decision",
            decision_id=self.full_decision_id,
            decision_start_sim_s=decision_start_sim_s,
            candidate_count=len(candidates),
            candidate_signature=signature,
            computation_ms=computation_ms,
        )

        if not candidates:
            self._emit(
                "terminal_state",
                decision_id=self.full_decision_id,
                reason="no_frontier_region_gt_10",
                candidate_signature=signature,
            )
        return candidates

    def _plan_current_candidate(self) -> None:
        if (
            self.current_candidates
            and self.current_mode != "exact"
            and self.current_candidate_index >= len(self.current_candidates)
        ):
            self._emit(
                "terminal_state",
                decision_id=self.full_decision_id,
                reason="no_nav2_reachable_ranked_candidate",
                candidate_signature=self._candidate_signature(self.current_candidates),
            )
        super()._plan_current_candidate()

    def _on_plan_result(self, future) -> None:
        try:
            wrapped = future.result()
            path_ok = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and bool(wrapped.result.path.poses)
            )
        except Exception:
            path_ok = False

        if path_ok and self.current_candidate is not None and self.latest_map is not None:
            candidate = self.current_candidate
            fx, fy = self._grid_to_world(candidate.row, candidate.col, self.latest_map)
            ex, ey = self.current_execution_xy or (fx, fy)
            self.full_goal_id += 1
            self.full_goal_open = True
            self._emit(
                "goal_attempt",
                goal_id=self.full_goal_id,
                decision_id=self.full_decision_id,
                mode=self.current_mode,
                frontier_row=candidate.row,
                frontier_col=candidate.col,
                frontier_x=fx,
                frontier_y=fy,
                goal_x=ex,
                goal_y=ey,
                euclidean_distance_m=(
                    candidate.distance_cells * float(self.latest_map.info.resolution)
                ),
            )

        super()._on_plan_result(future)

    def _finish_goal(self, result: str, status: int | None = None) -> None:
        if not self.full_goal_open:
            return
        self._emit(
            "goal_result",
            goal_id=self.full_goal_id,
            decision_id=self.full_decision_id,
            result=result,
            action_status=status,
        )
        self.full_goal_open = False

    def _on_follow_result(self, future) -> None:
        try:
            wrapped = future.result()
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                self._finish_goal("succeeded", int(wrapped.status))
        except Exception:
            pass
        super()._on_follow_result(future)

    def _mark_execution_failure_and_continue(self) -> None:
        self._finish_goal("failed", None)
        super()._mark_execution_failure_and_continue()


class BaselineRecorder(Node):
    def __init__(self, run_id: str) -> None:
        super().__init__("nearest_baseline_recorder")
        self.run_id = run_id
        self.research_dir = Path(__file__).resolve().parents[1]
        self.run_dir = self.research_dir / "experiments" / "nearest" / run_id
        if self.run_dir.exists():
            raise RuntimeError(f"Run already exists: {self.run_dir}")
        self.maps_dir = self.run_dir / "maps"
        self.maps_dir.mkdir(parents=True, exist_ok=False)

        self.roi_path = (
            self.research_dir
            / "ground_truth"
            / "hospital"
            / "generated"
            / "hospital_connected_free_v1.npy"
        )
        self.roi_mask: np.ndarray | None = None
        if self.roi_path.exists():
            mask = np.load(self.roi_path)
            if mask.shape != (CANVAS_HEIGHT, CANVAS_WIDTH):
                raise RuntimeError(
                    f"ROI mask shape {mask.shape} != {(CANVAS_HEIGHT, CANVAS_WIDTH)}"
                )
            self.roi_mask = mask.astype(bool)
            self.get_logger().info(f"Loaded frozen ROI mask: {self.roi_path}")
        else:
            self.get_logger().warning(
                "Frozen ROI .npy is missing; exact coverage will be NaN. "
                "Raw/fixed maps are still saved for offline recomputation."
            )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_subscription(String, EVENT_TOPIC, self._on_event, 50)

        self.metrics_file = (self.run_dir / "metrics.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.trajectory_file = (self.run_dir / "trajectory.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.decisions_file = (self.run_dir / "decisions.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.events_file = (self.run_dir / "events.csv").open(
            "w", newline="", encoding="utf-8"
        )

        self.metrics_writer = csv.writer(self.metrics_file)
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.decisions_writer = csv.writer(self.decisions_file)
        self.events_writer = csv.writer(self.events_file)

        self.metrics_writer.writerow(
            [
                "time_s",
                "distance_m",
                "known_fraction",
                "coverage",
                "goals_attempted",
                "goals_succeeded",
                "goals_failed",
                "success_rate",
            ]
        )
        self.trajectory_writer.writerow(["time_s", "x", "y", "distance_m"])
        self.decisions_writer.writerow(
            [
                "decision_id",
                "time_s",
                "candidate_count",
                "candidate_signature",
                "computation_ms",
            ]
        )
        self.events_writer.writerow(["time_s", "event", "payload_json"])

        self.latest_map: OccupancyGrid | None = None
        self.current_odom_xy: tuple[float, float] | None = None
        self.last_distance_xy: tuple[float, float] | None = None
        self.total_distance_m = 0.0
        self.last_trajectory_write_s = -1e9

        self.t0_sim_s: float | None = None
        self.last_snapshot_time_s = -1e9
        self.goals_attempted = 0
        self.goals_succeeded = 0
        self.goals_failed = 0
        self.computation_ms: list[float] = []
        self.last_known_fraction = math.nan
        self.last_coverage = math.nan

        self.terminal_reason: str | None = None
        self.terminal_signature: str | None = None
        self.terminal_streak = 0
        self.terminal_first_sim_s: float | None = None
        self.finalized = False

        self.create_timer(METRICS_PERIOD_S, self._write_metrics)
        self.get_logger().warning(
            f"NEAREST BASELINE RECORDING: run_id={run_id}, output={self.run_dir}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _elapsed(self, sim_s: float | None = None) -> float:
        if self.t0_sim_s is None:
            return 0.0
        if sim_s is None:
            sim_s = self._now_s()
        return max(0.0, sim_s - self.t0_sim_s)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _on_odom(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self.current_odom_xy = (x, y)
        if self.t0_sim_s is None:
            return

        if self.last_distance_xy is None:
            self.last_distance_xy = (x, y)
        else:
            dx = x - self.last_distance_xy[0]
            dy = y - self.last_distance_xy[1]
            step = math.hypot(dx, dy)
            if step < 2.0:
                self.total_distance_m += step
            self.last_distance_xy = (x, y)

        elapsed = self._elapsed()
        if elapsed - self.last_trajectory_write_s >= TRAJECTORY_PERIOD_S:
            self.trajectory_writer.writerow(
                [f"{elapsed:.6f}", f"{x:.6f}", f"{y:.6f}", f"{self.total_distance_m:.6f}"]
            )
            self.trajectory_file.flush()
            self.last_trajectory_write_s = elapsed

    def _reset_terminal_window(self) -> None:
        self.terminal_reason = None
        self.terminal_signature = None
        self.terminal_streak = 0
        self.terminal_first_sim_s = None

    def _on_event(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning("Ignoring malformed nearest event JSON")
            return

        event = str(payload.get("event", "unknown"))
        sim_s = float(payload.get("sim_time_s", self._now_s()))

        if self.t0_sim_s is None and event == "decision":
            self.t0_sim_s = float(payload.get("decision_start_sim_s", sim_s))
            self.last_distance_xy = self.current_odom_xy
            self.get_logger().warning(
                f"Benchmark t=0 at first policy decision: {self.t0_sim_s:.3f} s sim time"
            )

        if self.t0_sim_s is None:
            return

        elapsed = self._elapsed(sim_s)
        self.events_writer.writerow(
            [f"{elapsed:.6f}", event, json.dumps(payload, separators=(",", ":"))]
        )
        self.events_file.flush()

        if event == "decision":
            decision_id = int(payload.get("decision_id", 0))
            count = int(payload.get("candidate_count", 0))
            signature = str(payload.get("candidate_signature", ""))
            comp_ms = float(payload.get("computation_ms", math.nan))
            if math.isfinite(comp_ms):
                self.computation_ms.append(comp_ms)
            self.decisions_writer.writerow(
                [decision_id, f"{elapsed:.6f}", count, signature, f"{comp_ms:.6f}"]
            )
            self.decisions_file.flush()
            if (
                self.terminal_signature is not None
                and signature != self.terminal_signature
            ):
                self._reset_terminal_window()

        elif event == "goal_attempt":
            self.goals_attempted += 1
            self._reset_terminal_window()

        elif event == "goal_result":
            result = str(payload.get("result", "failed"))
            if result == "succeeded":
                self.goals_succeeded += 1
            else:
                self.goals_failed += 1

        elif event == "terminal_state":
            reason = str(payload.get("reason", "unknown_terminal_state"))
            signature = str(payload.get("candidate_signature", ""))
            if reason == self.terminal_reason and signature == self.terminal_signature:
                self.terminal_streak += 1
            else:
                self.terminal_reason = reason
                self.terminal_signature = signature
                self.terminal_streak = 1
                self.terminal_first_sim_s = sim_s

            idle_s = (
                0.0
                if self.terminal_first_sim_s is None
                else sim_s - self.terminal_first_sim_s
            )
            if self.terminal_streak >= TERMINAL_SWEEPS and idle_s >= TERMINAL_IDLE_S:
                self.get_logger().warning(
                    f"Stable exploration completion: {reason}, "
                    f"sweeps={self.terminal_streak}, idle={idle_s:.1f}s"
                )
                self.finalize(reason)
                if rclpy.ok():
                    rclpy.shutdown()

    @staticmethod
    def _origin_yaw(msg: OccupancyGrid) -> float:
        q = msg.info.origin.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _fixed_canvas(self, msg: OccupancyGrid) -> np.ndarray | None:
        if abs(self._origin_yaw(msg)) > 1e-4:
            self.get_logger().error("Rotated OccupancyGrid origin is unsupported for fixed-canvas paste")
            return None

        source_resolution = float(msg.info.resolution)
        ratio_f = source_resolution / CANVAS_RESOLUTION_M
        ratio = int(round(ratio_f))
        if ratio < 1 or abs(ratio_f - ratio) > 1e-6:
            self.get_logger().error(
                "Map resolution %.6f m is not an integer multiple of canonical %.3f m canvas",
                source_resolution,
                CANVAS_RESOLUTION_M,
            )
            return None

        raw = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        if ratio > 1:
            raw = np.repeat(np.repeat(raw, ratio, axis=0), ratio, axis=1)

        canvas = np.full((CANVAS_HEIGHT, CANVAS_WIDTH), -1, dtype=np.int16)
        col0 = int(round((float(msg.info.origin.position.x) - CANVAS_ORIGIN_X_M) / CANVAS_RESOLUTION_M))
        row0 = int(round((float(msg.info.origin.position.y) - CANVAS_ORIGIN_Y_M) / CANVAS_RESOLUTION_M))

        src_r0 = max(0, -row0)
        src_c0 = max(0, -col0)
        dst_r0 = max(0, row0)
        dst_c0 = max(0, col0)
        rows = min(raw.shape[0] - src_r0, CANVAS_HEIGHT - dst_r0)
        cols = min(raw.shape[1] - src_c0, CANVAS_WIDTH - dst_c0)
        if rows <= 0 or cols <= 0:
            return canvas
        canvas[dst_r0 : dst_r0 + rows, dst_c0 : dst_c0 + cols] = raw[
            src_r0 : src_r0 + rows, src_c0 : src_c0 + cols
        ]
        return canvas

    def _compute_map_metrics(self) -> tuple[float, float]:
        if self.latest_map is None:
            return math.nan, math.nan
        canvas = self._fixed_canvas(self.latest_map)
        if canvas is None:
            return math.nan, math.nan
        known = canvas >= 0
        known_fraction = float(np.count_nonzero(known)) / float(canvas.size)
        coverage = math.nan
        if self.roi_mask is not None:
            coverage = float(np.count_nonzero(known & self.roi_mask)) / float(ROI_DENOMINATOR)
        return known_fraction, coverage

    def _save_snapshot(self, name: str) -> None:
        if self.latest_map is None:
            return
        msg = self.latest_map
        raw = np.asarray(msg.data, dtype=np.int16).reshape(
            int(msg.info.height), int(msg.info.width)
        )
        canvas = self._fixed_canvas(msg)
        out = self.maps_dir / f"{name}.npz"
        kwargs: dict[str, Any] = {
            "raw": raw.astype(np.int16),
            "resolution": float(msg.info.resolution),
            "origin_x": float(msg.info.origin.position.x),
            "origin_y": float(msg.info.origin.position.y),
            "origin_yaw": self._origin_yaw(msg),
            "frame_id": msg.header.frame_id,
        }
        if canvas is not None:
            kwargs["fixed_canvas"] = canvas.astype(np.int16)
        np.savez_compressed(out, **kwargs)

    def _write_metrics(self) -> None:
        if self.finalized or self.t0_sim_s is None or self.latest_map is None:
            return
        elapsed = self._elapsed()
        known_fraction, coverage = self._compute_map_metrics()
        self.last_known_fraction = known_fraction
        self.last_coverage = coverage
        success_rate = (
            float(self.goals_succeeded) / float(self.goals_attempted)
            if self.goals_attempted
            else 0.0
        )
        self.metrics_writer.writerow(
            [
                f"{elapsed:.6f}",
                f"{self.total_distance_m:.6f}",
                f"{known_fraction:.9f}",
                "nan" if not math.isfinite(coverage) else f"{coverage:.9f}",
                self.goals_attempted,
                self.goals_succeeded,
                self.goals_failed,
                f"{success_rate:.9f}",
            ]
        )
        self.metrics_file.flush()

        if elapsed - self.last_snapshot_time_s >= MAP_SNAPSHOT_PERIOD_S:
            self._save_snapshot(f"map_{int(round(elapsed)):06d}s")
            self.last_snapshot_time_s = elapsed

    def finalize(self, reason: str) -> None:
        if self.finalized:
            return
        self.finalized = True
        if self.t0_sim_s is not None:
            known_fraction, coverage = self._compute_map_metrics()
            self.last_known_fraction = known_fraction
            self.last_coverage = coverage
            self._save_snapshot("map_final")

        elapsed = self._elapsed()
        success_rate = (
            float(self.goals_succeeded) / float(self.goals_attempted)
            if self.goals_attempted
            else 0.0
        )
        comp = np.asarray(self.computation_ms, dtype=np.float64)
        summary = {
            "run_id": self.run_id,
            "method": "nearest",
            "termination_reason": reason,
            "total_time_s": elapsed,
            "total_distance_m": self.total_distance_m,
            "goals_attempted": self.goals_attempted,
            "goals_succeeded": self.goals_succeeded,
            "goals_failed": self.goals_failed,
            "success_rate": success_rate,
            "decision_count": int(comp.size),
            "computation_ms_mean": float(np.mean(comp)) if comp.size else None,
            "computation_ms_std": float(np.std(comp)) if comp.size else None,
            "final_known_fraction": (
                self.last_known_fraction if math.isfinite(self.last_known_fraction) else None
            ),
            "final_coverage": self.last_coverage if math.isfinite(self.last_coverage) else None,
            "coverage_available": self.roi_mask is not None,
            "roi_id": "hospital_connected_free_v1",
            "roi_denominator_cells": ROI_DENOMINATOR,
            "fixed_canvas_id": "hospital_canvas_v1",
            "fixed_canvas_resolution_m": CANVAS_RESOLUTION_M,
            "runtime_map_resolution_m": (
                float(self.latest_map.info.resolution)
                if self.latest_map is not None
                else None
            ),
            "map_snapshot_period_s": MAP_SNAPSHOT_PERIOD_S,
        }
        (self.run_dir / "summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )

        for f in (
            self.metrics_file,
            self.trajectory_file,
            self.decisions_file,
            self.events_file,
        ):
            try:
                f.flush()
                f.close()
            except Exception:
                pass
        self.get_logger().warning(f"Baseline summary saved: {self.run_dir / 'summary.json'}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", required=True)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    explorer: InstrumentedNearest | None = None
    recorder: BaselineRecorder | None = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        explorer = InstrumentedNearest()
        recorder = BaselineRecorder(args.run_id)
        executor.add_node(explorer)
        executor.add_node(recorder)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if recorder is not None:
            recorder.finalize("manual_shutdown" if recorder.terminal_reason is None else recorder.terminal_reason)
        if explorer is not None:
            explorer.destroy_node()
        if recorder is not None:
            recorder.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
