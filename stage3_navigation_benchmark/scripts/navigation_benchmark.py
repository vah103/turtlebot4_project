#!/usr/bin/env python3
"""Run repeatable TurtleBot4 Nav2 goals and save benchmark metrics to CSV."""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

import rclpy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


def as_float(value: Any, field: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field} must be numeric, got {value!r}") from exc


def load_config(path: Path) -> tuple[str, dict[str, Any], list[dict[str, Any]]]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict):
        raise ValueError("The YAML root must be a mapping.")

    frame_id = str(config.get("frame_id", "map"))
    benchmark = config.get("benchmark", {})
    goals = config.get("goals")

    if not isinstance(benchmark, dict):
        raise ValueError("'benchmark' must be a mapping.")
    if not isinstance(goals, list) or not goals:
        raise ValueError("'goals' must be a non-empty list.")

    validated: list[dict[str, Any]] = []
    for index, goal in enumerate(goals, start=1):
        if not isinstance(goal, dict):
            raise ValueError(f"Goal {index} must be a mapping.")
        validated.append(
            {
                "name": str(goal.get("name", f"goal_{index}")),
                "x": as_float(goal.get("x"), f"goals[{index}].x"),
                "y": as_float(goal.get("y"), f"goals[{index}].y"),
                "yaw": as_float(goal.get("yaw", 0.0), f"goals[{index}].yaw"),
            }
        )

    return frame_id, benchmark, validated


class NavigationBenchmark(Node):
    def __init__(self, action_name: str, odom_topic: str) -> None:
        super().__init__("navigation_benchmark_runner")
        self._client = ActionClient(self, NavigateToPose, action_name)
        self._odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )

        self._tracking = False
        self._last_odom_xy: tuple[float, float] | None = None
        self.path_length_m = 0.0
        self.recovery_count = 0
        self.final_distance_remaining_m = math.nan
        self.nav2_navigation_time_s = math.nan

    def wait_for_server(self, timeout_sec: float = 30.0) -> bool:
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def _odom_callback(self, message: Odometry) -> None:
        if not self._tracking:
            return

        position = message.pose.pose.position
        current_xy = (float(position.x), float(position.y))
        if self._last_odom_xy is not None:
            self.path_length_m += math.hypot(
                current_xy[0] - self._last_odom_xy[0],
                current_xy[1] - self._last_odom_xy[1],
            )
        self._last_odom_xy = current_xy

    def _feedback_callback(self, feedback_message: Any) -> None:
        feedback = feedback_message.feedback
        self.recovery_count = max(
            self.recovery_count,
            int(getattr(feedback, "number_of_recoveries", 0)),
        )
        self.final_distance_remaining_m = float(
            getattr(feedback, "distance_remaining", math.nan)
        )

        navigation_time = getattr(feedback, "navigation_time", None)
        if navigation_time is not None:
            self.nav2_navigation_time_s = (
                float(navigation_time.sec)
                + float(navigation_time.nanosec) / 1_000_000_000.0
            )

    def _reset_metrics(self) -> None:
        self._last_odom_xy = None
        self.path_length_m = 0.0
        self.recovery_count = 0
        self.final_distance_remaining_m = math.nan
        self.nav2_navigation_time_s = math.nan

    def run_goal(
        self,
        frame_id: str,
        goal: dict[str, Any],
        timeout_sec: float,
    ) -> dict[str, Any]:
        self._reset_metrics()

        message = NavigateToPose.Goal()
        message.pose.header.frame_id = frame_id
        message.pose.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = goal["x"]
        message.pose.pose.position.y = goal["y"]
        message.pose.pose.orientation.z = math.sin(goal["yaw"] / 2.0)
        message.pose.pose.orientation.w = math.cos(goal["yaw"] / 2.0)

        started_monotonic = time.monotonic()
        started_utc = datetime.now(timezone.utc).isoformat()
        send_future = self._client.send_goal_async(
            message,
            feedback_callback=self._feedback_callback,
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            return {
                "started_utc": started_utc,
                "status": "REJECTED",
                "status_code": GoalStatus.STATUS_UNKNOWN,
                "travel_time_s": time.monotonic() - started_monotonic,
                "nav2_navigation_time_s": math.nan,
                "path_length_odom_m": 0.0,
                "recovery_count": 0,
                "final_distance_remaining_m": math.nan,
                "error_code": "",
                "failure_reason": "Goal was rejected by the action server.",
            }

        self._tracking = True
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout_sec
        timed_out = False

        while rclpy.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() >= deadline:
                timed_out = True
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                break

        self._tracking = False
        elapsed = time.monotonic() - started_monotonic

        if timed_out:
            return {
                "started_utc": started_utc,
                "status": "TIMEOUT",
                "status_code": GoalStatus.STATUS_CANCELING,
                "travel_time_s": elapsed,
                "nav2_navigation_time_s": self.nav2_navigation_time_s,
                "path_length_odom_m": self.path_length_m,
                "recovery_count": self.recovery_count,
                "final_distance_remaining_m": self.final_distance_remaining_m,
                "error_code": "",
                "failure_reason": f"Exceeded {timeout_sec:.1f} seconds.",
            }

        wrapped_result = result_future.result()
        status_code = int(wrapped_result.status)
        result = wrapped_result.result
        error_code = getattr(result, "error_code", "")
        error_message = str(getattr(result, "error_msg", "") or "")

        return {
            "started_utc": started_utc,
            "status": STATUS_NAMES.get(status_code, f"STATUS_{status_code}"),
            "status_code": status_code,
            "travel_time_s": elapsed,
            "nav2_navigation_time_s": self.nav2_navigation_time_s,
            "path_length_odom_m": self.path_length_m,
            "recovery_count": self.recovery_count,
            "final_distance_remaining_m": self.final_distance_remaining_m,
            "error_code": error_code,
            "failure_reason": error_message,
        }


def format_number(value: Any) -> Any:
    if isinstance(value, float):
        if math.isnan(value):
            return ""
        return f"{value:.3f}"
    return value


def default_output_path() -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"benchmark_results_{stamp}.csv")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run repeatable Nav2 goals and save per-goal metrics.",
    )
    parser.add_argument("--goals", required=True, type=Path, help="Goal YAML file.")
    parser.add_argument("--output", type=Path, default=None, help="Output CSV path.")
    parser.add_argument("--rounds", type=int, default=None, help="Override YAML rounds.")
    parser.add_argument(
        "--goal-timeout-sec",
        type=float,
        default=None,
        help="Override the per-goal timeout.",
    )
    parser.add_argument(
        "--settle-time-sec",
        type=float,
        default=None,
        help="Override the pause after each goal.",
    )
    parser.add_argument(
        "--action-name",
        default="/bot1/navigate_to_pose",
        help="NavigateToPose action name.",
    )
    parser.add_argument("--odom-topic", default="/bot1/odom", help="Odometry topic.")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Send all goals without an ENTER confirmation before each goal.",
    )
    parser.add_argument(
        "--stop-on-failure",
        action="store_true",
        help="Stop the benchmark after the first non-succeeded goal.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print the route without connecting to ROS.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    try:
        frame_id, benchmark, goals = load_config(args.goals)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f"Configuration error: {exc}", file=sys.stderr)
        return 2

    rounds = args.rounds or int(benchmark.get("rounds", 1))
    goal_timeout_sec = (
        args.goal_timeout_sec
        if args.goal_timeout_sec is not None
        else float(benchmark.get("goal_timeout_sec", 180.0))
    )
    settle_time_sec = (
        args.settle_time_sec
        if args.settle_time_sec is not None
        else float(benchmark.get("settle_time_sec", 2.0))
    )

    if rounds < 1:
        print("Configuration error: rounds must be at least 1.", file=sys.stderr)
        return 2
    if goal_timeout_sec <= 0 or settle_time_sec < 0:
        print("Configuration error: invalid timeout or settle time.", file=sys.stderr)
        return 2

    print(f"Frame: {frame_id}")
    print(f"Rounds: {rounds}")
    print(f"Goals per round: {len(goals)}")
    for index, goal in enumerate(goals, start=1):
        print(
            f"  {index}. {goal['name']}: "
            f"x={goal['x']:.3f}, y={goal['y']:.3f}, yaw={goal['yaw']:.3f}"
        )

    if args.dry_run:
        print("Dry run passed; no goals were sent.")
        return 0

    output_path = args.output or default_output_path()
    output_path = output_path.expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "run_id",
        "started_utc",
        "round",
        "goal_index",
        "goal_name",
        "goal_x",
        "goal_y",
        "goal_yaw",
        "status",
        "status_code",
        "travel_time_s",
        "nav2_navigation_time_s",
        "path_length_odom_m",
        "recovery_count",
        "final_distance_remaining_m",
        "error_code",
        "failure_reason",
    ]
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")

    rclpy.init()
    node = NavigationBenchmark(args.action_name, args.odom_topic)

    try:
        print(f"Waiting for action server: {args.action_name}")
        if not node.wait_for_server(timeout_sec=30.0):
            print("Action server was not available after 30 seconds.", file=sys.stderr)
            return 3

        print(f"Writing results to: {output_path}")
        with output_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            stream.flush()

            for round_number in range(1, rounds + 1):
                print(f"\n=== Round {round_number}/{rounds} ===")
                for goal_index, goal in enumerate(goals, start=1):
                    if not args.auto:
                        input(
                            f"Press ENTER to send {goal['name']} "
                            f"({goal_index}/{len(goals)}), or Ctrl+C to stop: "
                        )

                    print(
                        f"Sending {goal['name']}: "
                        f"x={goal['x']:.3f}, y={goal['y']:.3f}, "
                        f"yaw={goal['yaw']:.3f}"
                    )
                    metrics = node.run_goal(frame_id, goal, goal_timeout_sec)
                    row = {
                        "run_id": run_id,
                        "round": round_number,
                        "goal_index": goal_index,
                        "goal_name": goal["name"],
                        "goal_x": goal["x"],
                        "goal_y": goal["y"],
                        "goal_yaw": goal["yaw"],
                        **metrics,
                    }
                    writer.writerow({key: format_number(value) for key, value in row.items()})
                    stream.flush()

                    print(
                        f"Result: {metrics['status']} | "
                        f"time={metrics['travel_time_s']:.2f}s | "
                        f"path={metrics['path_length_odom_m']:.2f}m | "
                        f"recoveries={metrics['recovery_count']}"
                    )

                    if (
                        args.stop_on_failure
                        and metrics["status"] != "SUCCEEDED"
                    ):
                        print("Stopping after a non-succeeded goal.")
                        return 4

                    if settle_time_sec > 0:
                        time.sleep(settle_time_sec)

        print("\nBenchmark complete.")
        return 0
    except KeyboardInterrupt:
        print("\nBenchmark interrupted; completed CSV rows were preserved.")
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
