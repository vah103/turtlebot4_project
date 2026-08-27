#!/usr/bin/env python3
"""Replay a saved Hospital snapshot run through Nav2 planner-only validation.

For each fixed-canvas snapshot with a saved map-frame robot pose:
1. publish the saved OccupancyGrid on /map;
2. publish map->base_link at that snapshot robot pose;
3. reuse control_tb4.py frontier generation;
4. test the nearest N frontier centroids with Nav2 ComputePathToPose;
5. record whether the nearest centroid is reachable and whether a fallback among
   the tested nearest candidates is reachable.

No Gazebo, SLAM, lidar, odometry, controller_server, or behavior_server is used.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(SCRIPT_DIR))

from test_nearest_frontier_nav import (  # noqa: E402
    PlannerOnlyNearestTest,
    build_candidates,
)


class SnapshotReplayNode(PlannerOnlyNearestTest):
    def __init__(self) -> None:
        super().__init__()
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

    def publish_snapshot(
        self,
        map_msg: OccupancyGrid,
        robot_pose: dict,
        settle_sec: float,
    ) -> None:
        deadline = time.monotonic() + max(0.1, settle_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            stamp = self.get_clock().now().to_msg()
            map_msg.header.stamp = stamp
            self.map_pub.publish(map_msg)

            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'base_link'
            transform.header.stamp = stamp
            position = robot_pose['position']
            orientation = robot_pose['orientation']
            transform.transform.translation.x = float(position['x'])
            transform.transform.translation.y = float(position['y'])
            transform.transform.translation.z = float(position.get('z', 0.0))
            transform.transform.rotation.x = float(orientation.get('x', 0.0))
            transform.transform.rotation.y = float(orientation.get('y', 0.0))
            transform.transform.rotation.z = float(orientation.get('z', 0.0))
            transform.transform.rotation.w = float(orientation.get('w', 1.0))
            self.tf_broadcaster.sendTransform(transform)

            rclpy.spin_once(self, timeout_sec=0.05)


def _load_metadata(path: Path) -> dict:
    with path.open('r', encoding='utf-8') as stream:
        return json.load(stream)


def _load_map(run_dir: Path, metadata: dict) -> OccupancyGrid:
    map_meta = metadata['map']
    width = int(map_meta['width'])
    height = int(map_meta['height'])
    resolution = float(map_meta['resolution'])

    raw_name = metadata.get('files', {}).get('raw')
    if not raw_name:
        sequence = int(metadata['sequence'])
        raw_name = f'{sequence:06d}_occupancy.bin'
    raw_path = run_dir / raw_name

    data = np.fromfile(raw_path, dtype=np.int8)
    expected = width * height
    if data.size != expected:
        raise RuntimeError(
            f'{raw_path}: expected {expected} cells, found {data.size}'
        )

    msg = OccupancyGrid()
    msg.header.frame_id = str(map_meta.get('frame_id', 'map')) or 'map'
    msg.info.width = width
    msg.info.height = height
    msg.info.resolution = resolution

    origin = map_meta['origin']
    position = origin.get('position', origin)
    orientation = origin.get('orientation', {})
    msg.info.origin.position.x = float(position.get('x', 0.0))
    msg.info.origin.position.y = float(position.get('y', 0.0))
    msg.info.origin.position.z = float(position.get('z', 0.0))
    msg.info.origin.orientation.x = float(orientation.get('x', 0.0))
    msg.info.origin.orientation.y = float(orientation.get('y', 0.0))
    msg.info.origin.orientation.z = float(orientation.get('z', 0.0))
    msg.info.origin.orientation.w = float(orientation.get('w', 1.0))
    msg.data = [int(value) for value in data]
    return msg


def _metadata_files(run_dir: Path) -> list[Path]:
    files = sorted(run_dir.glob('*_metadata.json'))
    if files:
        return files

    manifest = run_dir / 'manifest.jsonl'
    if not manifest.is_file():
        return []

    # Metadata files are the canonical input. A manifest without those files is
    # not sufficient because the raw file names still need to exist locally.
    return []


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--run-dir',
        default=str(PROJECT_DIR.parent / 'data/lama_runs/hospital_flat_lama_01_001'),
    )
    parser.add_argument('--top-k', type=int, default=3)
    parser.add_argument('--min-distance', type=float, default=0.5)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--settle-sec', type=float, default=0.8)
    parser.add_argument('--stride', type=int, default=1)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--start-sequence', type=int, default=0)
    parser.add_argument('--end-sequence', type=int, default=-1)
    parser.add_argument('--output-dir', default='')
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    run_dir = Path(args.run_dir).expanduser().resolve()
    if not run_dir.is_dir():
        raise SystemExit(f'Run directory not found: {run_dir}')

    metadata_files = _metadata_files(run_dir)
    if not metadata_files:
        raise SystemExit(f'No *_metadata.json snapshots found in {run_dir}')

    stride = max(1, int(args.stride))
    selected_files: list[Path] = []
    for path in metadata_files:
        try:
            sequence = int(path.name.split('_', 1)[0])
        except ValueError:
            continue
        if sequence < args.start_sequence:
            continue
        if args.end_sequence >= 0 and sequence > args.end_sequence:
            continue
        selected_files.append(path)
    selected_files = selected_files[::stride]
    if args.limit > 0:
        selected_files = selected_files[: args.limit]

    if args.output_dir:
        output_dir = Path(args.output_dir).expanduser().resolve()
    else:
        output_dir = run_dir / 'planner_only_nearest_test'
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / 'snapshot_results.csv'
    summary_path = output_dir / 'summary.json'

    rclpy.init(args=[])
    node = SnapshotReplayNode()
    rows: list[dict] = []

    skipped_no_pose = 0
    skipped_no_frontier = 0
    nearest_reachable = 0
    fallback_reachable = 0
    no_reachable = 0
    consecutive_nearest_fail = 0
    max_consecutive_nearest_fail = 0

    try:
        if not node.compute_path.wait_for_server(timeout_sec=args.timeout):
            raise RuntimeError(
                'compute_path_to_pose is unavailable. Start '
                'hospital_planner_replay.launch.py first.'
            )

        total = len(selected_files)
        print(f'Testing {total} saved Hospital snapshots; top-k={args.top_k}.')

        for index, meta_path in enumerate(selected_files, start=1):
            metadata = _load_metadata(meta_path)
            sequence = int(metadata.get('sequence', -1))
            robot_pose = metadata.get('map_robot_pose')
            if not robot_pose:
                skipped_no_pose += 1
                print(f'[{index}/{total}] seq={sequence}: SKIP no map_robot_pose')
                continue

            map_msg = _load_map(run_dir, metadata)
            start_x = float(robot_pose['position']['x'])
            start_y = float(robot_pose['position']['y'])

            node.publish_snapshot(map_msg, robot_pose, args.settle_sec)

            try:
                candidates = build_candidates(
                    map_msg,
                    start_x,
                    start_y,
                    args.min_distance,
                )
            except Exception as exc:
                rows.append({
                    'sequence': sequence,
                    'start_x': start_x,
                    'start_y': start_y,
                    'candidate_count': 0,
                    'nearest_x': math.nan,
                    'nearest_y': math.nan,
                    'nearest_distance_m': math.nan,
                    'nearest_raw_occupancy': '',
                    'nearest_reachable': False,
                    'first_reachable_rank': '',
                    'tested_candidates': 0,
                    'status': 'frontier_error',
                    'detail': str(exc),
                })
                print(f'[{index}/{total}] seq={sequence}: FRONTIER ERROR {exc}')
                continue

            if not candidates:
                skipped_no_frontier += 1
                rows.append({
                    'sequence': sequence,
                    'start_x': start_x,
                    'start_y': start_y,
                    'candidate_count': 0,
                    'nearest_x': math.nan,
                    'nearest_y': math.nan,
                    'nearest_distance_m': math.nan,
                    'nearest_raw_occupancy': '',
                    'nearest_reachable': False,
                    'first_reachable_rank': '',
                    'tested_candidates': 0,
                    'status': 'no_frontier',
                    'detail': '',
                })
                print(f'[{index}/{total}] seq={sequence}: no frontier candidates')
                continue

            test_candidates = candidates[: max(1, int(args.top_k))]
            outcomes: list[dict] = []
            for candidate in test_candidates:
                outcome = node.check_path(
                    start_x,
                    start_y,
                    candidate['x'],
                    candidate['y'],
                    args.timeout,
                )
                outcomes.append(outcome)

            nearest_ok = bool(outcomes[0]['reachable'])
            first_rank = next(
                (
                    rank
                    for rank, outcome in enumerate(outcomes, start=1)
                    if outcome['reachable']
                ),
                None,
            )

            if nearest_ok:
                nearest_reachable += 1
                consecutive_nearest_fail = 0
                status = 'nearest_reachable'
            else:
                consecutive_nearest_fail += 1
                max_consecutive_nearest_fail = max(
                    max_consecutive_nearest_fail,
                    consecutive_nearest_fail,
                )
                if first_rank is not None:
                    fallback_reachable += 1
                    status = 'fallback_reachable'
                else:
                    no_reachable += 1
                    status = 'no_reachable_top_k'

            nearest = candidates[0]
            detail_parts = []
            for rank, (candidate, outcome) in enumerate(
                zip(test_candidates, outcomes), start=1
            ):
                detail_parts.append(
                    f"r{rank}:g{candidate['group_id']}@"
                    f"({candidate['x']:.2f},{candidate['y']:.2f})="
                    f"{'ok' if outcome['reachable'] else 'fail'}"
                )

            rows.append({
                'sequence': sequence,
                'start_x': start_x,
                'start_y': start_y,
                'candidate_count': len(candidates),
                'nearest_x': nearest['x'],
                'nearest_y': nearest['y'],
                'nearest_distance_m': nearest['distance_m'],
                'nearest_raw_occupancy': nearest['raw_occupancy'],
                'nearest_reachable': nearest_ok,
                'first_reachable_rank': first_rank if first_rank is not None else '',
                'tested_candidates': len(test_candidates),
                'status': status,
                'detail': '; '.join(detail_parts),
            })

            print(
                f'[{index}/{total}] seq={sequence}: {status}; '
                f"nearest=({nearest['x']:.2f},{nearest['y']:.2f}) "
                f"occ={nearest['raw_occupancy']} "
                f"first_reachable_rank={first_rank}"
            )

        fieldnames = [
            'sequence', 'start_x', 'start_y', 'candidate_count',
            'nearest_x', 'nearest_y', 'nearest_distance_m',
            'nearest_raw_occupancy', 'nearest_reachable',
            'first_reachable_rank', 'tested_candidates', 'status', 'detail',
        ]
        with csv_path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        evaluated = sum(
            row['status']
            in {'nearest_reachable', 'fallback_reachable', 'no_reachable_top_k'}
            for row in rows
        )
        summary = {
            'run_dir': str(run_dir),
            'requested_snapshots': len(selected_files),
            'evaluated_snapshots': evaluated,
            'skipped_no_map_robot_pose': skipped_no_pose,
            'snapshots_with_no_frontier': skipped_no_frontier,
            'top_k': max(1, int(args.top_k)),
            'min_distance_m': float(args.min_distance),
            'nearest_reachable_count': nearest_reachable,
            'nearest_reachable_rate': (
                nearest_reachable / evaluated if evaluated else None
            ),
            'nearest_failed_but_fallback_reachable_count': fallback_reachable,
            'no_reachable_in_top_k_count': no_reachable,
            'max_consecutive_nearest_failures': max_consecutive_nearest_fail,
            'csv': str(csv_path),
        }
        with summary_path.open('w', encoding='utf-8') as stream:
            json.dump(summary, stream, ensure_ascii=False, indent=2)
            stream.write('\n')

        print('\nSUMMARY')
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        print(f'CSV: {csv_path}')
        print(f'Summary: {summary_path}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
