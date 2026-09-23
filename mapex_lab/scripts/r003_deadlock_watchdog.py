#!/usr/bin/env python3
"""Objective ROS watchdog for R003 unattended paper500 runs."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


MIN_PROGRESS_M = 0.05
NO_PROGRESS_WINDOW_S = 180.0
STARTUP_GRACE_S = 120.0
ACTIVE_STATUSES = {1, 2, 3}
DEADLOCK_EXIT_CODE = 42


class DeadlockDetector:
    def __init__(
        self,
        *,
        min_progress_m: float = MIN_PROGRESS_M,
        window_s: float = NO_PROGRESS_WINDOW_S,
        startup_grace_s: float = STARTUP_GRACE_S,
        started_at: float = 0.0,
    ) -> None:
        self.min_progress_m = float(min_progress_m)
        self.window_s = float(window_s)
        self.startup_grace_s = float(startup_grace_s)
        self.started_at = float(started_at)
        self.action_active = False
        self.last_xy: tuple[float, float] | None = None
        self.progress_m = 0.0
        self.window_started_at: float | None = None

    def set_action_active(self, active: bool, now: float) -> None:
        active = bool(active)
        if active != self.action_active:
            self.progress_m = 0.0
            self.window_started_at = float(now) if active else None
        self.action_active = active

    def observe_odom(self, x: float, y: float, now: float) -> None:
        xy = (float(x), float(y))
        if self.last_xy is not None and self.action_active:
            self.progress_m += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            if self.progress_m >= self.min_progress_m:
                self.progress_m = 0.0
                self.window_started_at = float(now)
        self.last_xy = xy

    def check(self, now: float) -> dict | None:
        now = float(now)
        if now - self.started_at < self.startup_grace_s:
            return None
        if not self.action_active or self.last_xy is None or self.window_started_at is None:
            return None
        stalled_s = now - self.window_started_at
        if stalled_s < self.window_s:
            return None
        return {
            "reason": "technical_deadlock_no_odom_progress_with_active_action",
            "stalled_s": stalled_s,
            "progress_m": self.progress_m,
            "min_progress_m": self.min_progress_m,
            "window_s": self.window_s,
            "startup_grace_s": self.startup_grace_s,
            "last_xy": list(self.last_xy),
        }


def atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--action-status-topic", default="/navigate_to_pose/_action/status")
    args = parser.parse_args()

    import rclpy
    from action_msgs.msg import GoalStatusArray
    from nav_msgs.msg import Odometry

    rclpy.init(args=None)
    node = rclpy.create_node("r003_paper500_deadlock_watchdog")
    started = time.monotonic()
    detector = DeadlockDetector(started_at=started)
    verdict: dict | None = None

    def odom_callback(msg: Odometry) -> None:
        detector.observe_odom(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            time.monotonic(),
        )

    def status_callback(msg: GoalStatusArray) -> None:
        active = any(item.status in ACTIVE_STATUSES for item in msg.status_list)
        detector.set_action_active(active, time.monotonic())

    node.create_subscription(Odometry, args.odom_topic, odom_callback, 20)
    node.create_subscription(GoalStatusArray, args.action_status_topic, status_callback, 20)
    try:
        while rclpy.ok() and verdict is None:
            rclpy.spin_once(node, timeout_sec=1.0)
            verdict = detector.check(time.monotonic())
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if verdict is None:
        return 0
    verdict.update({"run_id": args.run_id, "detected_at_unix_s": time.time()})
    atomic_json(args.evidence.resolve(), verdict)
    return DEADLOCK_EXIT_CODE


if __name__ == "__main__":
    sys.exit(main())

