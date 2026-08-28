#!/usr/bin/env python3
"""Planner-only validation for the Hospital nearest-frontier baseline.

This script reuses the exact frontier generation from ../control_tb4.py, orders
frontier centroids by Euclidean distance, and asks Nav2 ComputePathToPose whether
each candidate is reachable from an explicit start pose.

It intentionally does not run Gazebo, SLAM, lidar, odometry, controller_server,
or behavior_server.  Use it together with hospital_planner_only.launch.py and a
saved *partial* Hospital occupancy map.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


PROJECT_DIR = Path(__file__).resolve().parents[1]
# control_tb4.py loads autonomous_exploration/config/params.yaml relative to cwd.
os.chdir(PROJECT_DIR)
sys.path.insert(0, str(PROJECT_DIR))
import control_tb4 as base  # noqa: E402


class PlannerOnlyNearestTest(Node):
    def __init__(self) -> None:
        super().__init__('nearest_frontier_planner_test')
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_msg: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.compute_path = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose'
        )

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def wait_for_map(self, timeout_sec: float) -> OccupancyGrid:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and self.map_msg is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.map_msg is None:
            raise RuntimeError('Timed out waiting for /map from map_server')
        return self.map_msg

    @staticmethod
    def _pose(x: float, y: float, stamp) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def check_path(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
        timeout_sec: float,
    ) -> dict:
        if not self.compute_path.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError('compute_path_to_pose action server is not available')

        stamp = self.get_clock().now().to_msg()
        request = ComputePathToPose.Goal()
        request.start = self._pose(start_x, start_y, stamp)
        request.goal = self._pose(goal_x, goal_y, stamp)
        request.planner_id = 'GridBased'
        request.use_start = True

        send_future = self.compute_path.send_goal_async(request)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None:
            return {
                'reachable': False,
                'status': -1,
                'path_length_m': math.nan,
                'error_code': -1,
                'error_msg': 'timeout waiting for goal response',
            }
        if not goal_handle.accepted:
            return {
                'reachable': False,
                'status': -1,
                'path_length_m': math.nan,
                'error_code': -1,
                'error_msg': 'planner rejected action goal',
            }

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        wrapped = result_future.result()
        if wrapped is None:
            return {
                'reachable': False,
                'status': -1,
                'path_length_m': math.nan,
                'error_code': -1,
                'error_msg': 'timeout waiting for planner result',
            }

        result = wrapped.result
        path = result.path
        poses = path.poses
        length_m = 0.0
        for previous, current in zip(poses, poses[1:]):
            dx = current.pose.position.x - previous.pose.position.x
            dy = current.pose.position.y - previous.pose.position.y
            length_m += math.hypot(dx, dy)

        reachable = (
            wrapped.status == GoalStatus.STATUS_SUCCEEDED and len(poses) > 0
        )
        return {
            'reachable': reachable,
            'status': int(wrapped.status),
            'path_length_m': float(length_m) if poses else math.nan,
            'error_code': int(getattr(result, 'error_code', 0)),
            'error_msg': str(getattr(result, 'error_msg', '')),
        }


def build_candidates(
    map_msg: OccupancyGrid,
    start_x: float,
    start_y: float,
    min_distance_m: float,
) -> list[dict]:
    width = int(map_msg.info.width)
    height = int(map_msg.info.height)
    resolution = float(map_msg.info.resolution)
    origin_x = float(map_msg.info.origin.position.x)
    origin_y = float(map_msg.info.origin.position.y)

    column = int((start_x - origin_x) / resolution)
    row = int((start_y - origin_y) / resolution)
    if not (0 <= row < height and 0 <= column < width):
        raise RuntimeError(
            f'Start pose ({start_x:.2f}, {start_y:.2f}) is outside map bounds'
        )

    # Reuse the production nearest-frontier implementation exactly for frontier
    # detection/grouping. Pass a copy because exploration() mutates its array.
    groups = base.exploration(
        list(map_msg.data),
        width,
        height,
        resolution,
        column,
        row,
        origin_x,
        origin_y,
    )

    original = np.asarray(map_msg.data, dtype=np.int16).reshape(height, width)
    candidates: list[dict] = []
    for group_id, group_cells, centroid in groups:
        if centroid is None:
            continue
        centroid_row, centroid_col = centroid
        goal_x = centroid_col * resolution + origin_x
        goal_y = centroid_row * resolution + origin_y
        distance_m = math.hypot(goal_x - start_x, goal_y - start_y)
        if distance_m < min_distance_m:
            continue

        candidates.append({
            'group_id': int(group_id),
            'group_size': int(len(group_cells)),
            'row': int(centroid_row),
            'col': int(centroid_col),
            'x': float(goal_x),
            'y': float(goal_y),
            'distance_m': float(distance_m),
            # This exposes the centroid problem directly: a group centroid can
            # land on unknown/occupied even though its constituent cells are
            # valid frontier cells.
            'raw_occupancy': int(original[centroid_row, centroid_col]),
        })

    candidates.sort(key=lambda item: item['distance_m'])
    return candidates


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--start-x', type=float, required=True)
    parser.add_argument('--start-y', type=float, required=True)
    parser.add_argument('--min-distance', type=float, default=0.5)
    parser.add_argument('--max-candidates', type=int, default=30)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--csv', default='')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    # All test configuration is passed through argparse, not ROS parameters.
    rclpy.init(args=[])
    node = PlannerOnlyNearestTest()
    rows: list[dict] = []

    try:
        map_msg = node.wait_for_map(args.timeout)
        candidates = build_candidates(
            map_msg,
            args.start_x,
            args.start_y,
            args.min_distance,
        )
        if args.max_candidates > 0:
            candidates = candidates[: args.max_candidates]

        print(
            f'Found {len(candidates)} nearest frontier candidates to test. '
            'Nav2 planner only; no Gazebo/SLAM/lidar.'
        )
        print('rank group      x      y   dist  occ  reachable  path_m  status')

        selected_rank = None
        for rank, candidate in enumerate(candidates, start=1):
            outcome = node.check_path(
                args.start_x,
                args.start_y,
                candidate['x'],
                candidate['y'],
                args.timeout,
            )
            row = {**candidate, **outcome, 'rank': rank, 'selected': False}
            if selected_rank is None and outcome['reachable']:
                selected_rank = rank
                row['selected'] = True
            rows.append(row)

            path_text = (
                f"{outcome['path_length_m']:.2f}"
                if math.isfinite(outcome['path_length_m'])
                else '-'
            )
            print(
                f"{rank:>4} {candidate['group_id']:>5} "
                f"{candidate['x']:>6.2f} {candidate['y']:>6.2f} "
                f"{candidate['distance_m']:>6.2f} "
                f"{candidate['raw_occupancy']:>4} "
                f"{str(outcome['reachable']):>9} "
                f"{path_text:>7} {outcome['status']:>7}"
            )

        if selected_rank is None:
            print('RESULT: no tested frontier is reachable by Nav2.')
        else:
            chosen = rows[selected_rank - 1]
            print(
                'RESULT: nearest reachable frontier = '
                f"rank {selected_rank}, group {chosen['group_id']}, "
                f"({chosen['x']:.2f}, {chosen['y']:.2f})."
            )

        if args.csv:
            output = Path(args.csv).expanduser().resolve()
            output.parent.mkdir(parents=True, exist_ok=True)
            fieldnames = [
                'rank', 'group_id', 'group_size', 'row', 'col', 'x', 'y',
                'distance_m', 'raw_occupancy', 'reachable', 'path_length_m',
                'status', 'error_code', 'error_msg', 'selected',
            ]
            with output.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(rows)
            print(f'CSV: {output}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
