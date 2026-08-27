#!/usr/bin/env python3
"""Stage-3 MapEx metric recorder for Hospital closed-loop experiments.

This node is intentionally separate from ``exploration_recorder.py``.
It records the two Stage-3 map-quality metrics that the generic recorder does not:

- occupied IoU = |predicted_occupied ∩ GT_occupied| / |union|
- TU = A* success rate to a fixed random set of valid-space goals, where the
  predicted-map path must also be collision-free in structural ground truth.

The TU definition matches the Hospital MapEx benchmark already kept in this repo.
Planning is evaluated on a 0.10 m grid (2x reduction from the canonical 0.05 m
Hospital canvas), with 4-connected A* and unknown cells treated as occupied.

Expected ROS inputs
-------------------
- prediction topic: nav_msgs/OccupancyGrid, default ``/mapex/predicted_map``
- exploration event topic: std_msgs/String JSON, default ``/exploration/events``
  The benchmark clock starts at the first ``event=decision`` message, matching
  ``exploration_recorder.py``.

Required local ground-truth files
---------------------------------
Pass two aligned NumPy masks with the canonical Hospital canvas shape:

- ``--gt-occupied``: boolean / 0-1 mask, True means structural obstacle
- ``--gt-valid``: boolean / 0-1 mask, True means valid connected free space

Large generated GT arrays should remain local and are not committed.

Output
------
``mapex_hospital_research/results/stage3_metrics/<method>/<run_id>/``

- ``stage3_metrics.csv``
- ``metadata.json``

Typical future launch command
-----------------------------
/usr/bin/python3 stage3_metrics_recorder.py \
  --method mapex --run-id mapex_001 \
  --gt-occupied /path/to/hospital_structural_occupied_v1.npy \
  --gt-valid /path/to/hospital_connected_free_v1.npy \
  --ros-args -p use_sim_time:=true
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import re
from pathlib import Path
from typing import Iterable

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


CANVAS_RESOLUTION_M = 0.05
CANVAS_WIDTH = 1504
CANVAS_HEIGHT = 2123
CANVAS_ORIGIN_X_M = -25.6
CANVAS_ORIGIN_Y_M = -60.1
TU_RESOLUTION_M = 0.10
_SAFE_NAME = re.compile(r"^[A-Za-z0-9_.-]+$")


def _load_bool_mask(path: Path, label: str) -> np.ndarray:
    if not path.exists():
        raise FileNotFoundError(f"{label} mask does not exist: {path}")

    loaded = np.load(path)
    if isinstance(loaded, np.lib.npyio.NpzFile):
        keys = list(loaded.files)
        if len(keys) != 1:
            raise RuntimeError(
                f"{label} NPZ must contain exactly one array; found keys={keys}"
            )
        arr = loaded[keys[0]]
        loaded.close()
    else:
        arr = loaded

    arr = np.asarray(arr)
    if arr.shape != (CANVAS_HEIGHT, CANVAS_WIDTH):
        raise RuntimeError(
            f"{label} mask shape {arr.shape} != "
            f"{(CANVAS_HEIGHT, CANVAS_WIDTH)}"
        )
    if arr.dtype == np.bool_:
        return arr.copy()

    unique = np.unique(arr)
    if len(unique) > 4 or np.any(unique < 0):
        raise RuntimeError(
            f"{label} must be an explicit boolean/0-1 mask; values={unique[:8]}"
        )
    return arr > 0


def _reduce_any_2(mask: np.ndarray) -> np.ndarray:
    """2x2 OR reduction: 0.05 m -> 0.10 m."""
    hpad = (-mask.shape[0]) % 2
    wpad = (-mask.shape[1]) % 2
    padded = np.pad(mask, ((0, hpad), (0, wpad)), constant_values=False)
    return padded.reshape(
        padded.shape[0] // 2,
        2,
        padded.shape[1] // 2,
        2,
    ).any(axis=(1, 3))


def _origin_yaw(msg: OccupancyGrid) -> float:
    q = msg.info.origin.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _occupancygrid_to_canvas(
    msg: OccupancyGrid,
    *,
    occupied_threshold: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (occupied, known) on canonical 0.05 m Hospital canvas.

    Integer-multiple resolutions such as 0.10 m are expanded by nearest-neighbor
    blocks before pasting into the fixed canvas.
    """
    if abs(_origin_yaw(msg)) > 1e-4:
        raise RuntimeError("Rotated OccupancyGrid origins are not supported")

    resolution = float(msg.info.resolution)
    ratio_f = resolution / CANVAS_RESOLUTION_M
    ratio = int(round(ratio_f))
    if ratio < 1 or abs(ratio_f - ratio) > 1e-6:
        raise RuntimeError(
            f"Prediction resolution {resolution} m is not an integer multiple of "
            f"{CANVAS_RESOLUTION_M} m"
        )

    raw = np.asarray(msg.data, dtype=np.int16).reshape(
        int(msg.info.height), int(msg.info.width)
    )
    if ratio > 1:
        raw = np.repeat(np.repeat(raw, ratio, axis=0), ratio, axis=1)

    occupied_src = raw >= occupied_threshold
    known_src = raw >= 0

    occupied = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH), dtype=bool)
    known = np.zeros((CANVAS_HEIGHT, CANVAS_WIDTH), dtype=bool)

    col0 = int(
        round(
            (float(msg.info.origin.position.x) - CANVAS_ORIGIN_X_M)
            / CANVAS_RESOLUTION_M
        )
    )
    row0 = int(
        round(
            (float(msg.info.origin.position.y) - CANVAS_ORIGIN_Y_M)
            / CANVAS_RESOLUTION_M
        )
    )

    src_r0 = max(0, -row0)
    src_c0 = max(0, -col0)
    dst_r0 = max(0, row0)
    dst_c0 = max(0, col0)
    rows = min(raw.shape[0] - src_r0, CANVAS_HEIGHT - dst_r0)
    cols = min(raw.shape[1] - src_c0, CANVAS_WIDTH - dst_c0)
    if rows <= 0 or cols <= 0:
        return occupied, known

    occupied[dst_r0 : dst_r0 + rows, dst_c0 : dst_c0 + cols] = occupied_src[
        src_r0 : src_r0 + rows, src_c0 : src_c0 + cols
    ]
    known[dst_r0 : dst_r0 + rows, dst_c0 : dst_c0 + cols] = known_src[
        src_r0 : src_r0 + rows, src_c0 : src_c0 + cols
    ]
    return occupied, known


def _metric_xy_to_reduced_rc(x: float, y: float) -> tuple[int, int]:
    col_raw = int(round((x - CANVAS_ORIGIN_X_M) / CANVAS_RESOLUTION_M))
    row_raw = int(round((y - CANVAS_ORIGIN_Y_M) / CANVAS_RESOLUTION_M))
    return row_raw // 2, col_raw // 2


def _astar_path(
    blocked: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> np.ndarray | None:
    """4-connected A* fallback used when pyastar2d is unavailable."""
    h, w = blocked.shape
    sr, sc = start
    gr, gc = goal
    if not (0 <= sr < h and 0 <= sc < w and 0 <= gr < h and 0 <= gc < w):
        return None
    if blocked[sr, sc] or blocked[gr, gc]:
        return None

    start_i = sr * w + sc
    goal_i = gr * w + gc
    open_heap: list[tuple[float, float, int]] = []
    heapq.heappush(open_heap, (abs(sr - gr) + abs(sc - gc), 0.0, start_i))
    g_score = {start_i: 0.0}
    parent: dict[int, int] = {}
    closed: set[int] = set()

    while open_heap:
        _f, g, idx = heapq.heappop(open_heap)
        if idx in closed:
            continue
        if idx == goal_i:
            rev = [idx]
            while rev[-1] != start_i:
                rev.append(parent[rev[-1]])
            rev.reverse()
            rows = np.fromiter((v // w for v in rev), dtype=np.int32)
            cols = np.fromiter((v % w for v in rev), dtype=np.int32)
            return np.stack([rows, cols], axis=1)

        closed.add(idx)
        r, c = divmod(idx, w)
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if nr < 0 or nr >= h or nc < 0 or nc >= w or blocked[nr, nc]:
                continue
            nidx = nr * w + nc
            if nidx in closed:
                continue
            ng = g + 1.0
            if ng >= g_score.get(nidx, float("inf")):
                continue
            g_score[nidx] = ng
            parent[nidx] = idx
            heuristic = abs(nr - gr) + abs(nc - gc)
            heapq.heappush(open_heap, (ng + heuristic, ng, nidx))
    return None


def _plan_path(
    blocked: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
) -> np.ndarray | None:
    try:
        import pyastar2d  # type: ignore

        cost = np.ones(blocked.shape, dtype=np.float32)
        cost[blocked] = np.inf
        try:
            path = pyastar2d.astar_path(
                cost,
                np.asarray(start, dtype=np.int32),
                np.asarray(goal, dtype=np.int32),
                allow_diagonal=False,
            )
        except Exception:
            return None
        return None if path is None else np.asarray(path, dtype=np.int32)
    except ImportError:
        return _astar_path(blocked, start, goal)


class Stage3MetricsRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stage3_metrics_recorder")

        if not _SAFE_NAME.fullmatch(args.method):
            raise ValueError(f"Unsafe method name: {args.method!r}")
        if not _SAFE_NAME.fullmatch(args.run_id):
            raise ValueError(f"Unsafe run_id: {args.run_id!r}")

        self.args = args
        self.gt_occupied_raw = _load_bool_mask(args.gt_occupied, "GT occupied")
        self.gt_valid_raw = _load_bool_mask(args.gt_valid, "GT valid")
        if np.any(self.gt_occupied_raw & self.gt_valid_raw):
            raise RuntimeError("GT occupied and GT valid masks overlap")

        self.gt_occupied = _reduce_any_2(self.gt_occupied_raw)
        self.gt_valid = _reduce_any_2(self.gt_valid_raw)

        valid_coords = np.argwhere(self.gt_valid)
        if len(valid_coords) == 0:
            raise RuntimeError("GT valid-space mask is empty")
        rng = np.random.default_rng(args.seed)
        count = min(args.tu_goals, len(valid_coords))
        inds = rng.choice(len(valid_coords), size=count, replace=False)
        self.tu_goals = valid_coords[inds].astype(np.int32)

        self.start_rc = _metric_xy_to_reduced_rc(args.start_x, args.start_y)
        sr, sc = self.start_rc
        if not (
            0 <= sr < self.gt_valid.shape[0]
            and 0 <= sc < self.gt_valid.shape[1]
            and self.gt_valid[sr, sc]
        ):
            coords = np.argwhere(self.gt_valid)
            d2 = np.sum((coords - np.asarray(self.start_rc)) ** 2, axis=1)
            nearest = coords[int(np.argmin(d2))]
            self.start_rc = (int(nearest[0]), int(nearest[1]))
            self.get_logger().warning(
                f"Configured TU start is outside valid space; using nearest "
                f"valid cell {self.start_rc}"
            )

        self.output_dir = (
            args.output_root.resolve() / args.method / args.run_id
        )
        if self.output_dir.exists():
            raise RuntimeError(f"Stage-3 output already exists: {self.output_dir}")
        self.output_dir.mkdir(parents=True, exist_ok=False)

        self.csv_file = (self.output_dir / "stage3_metrics.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "time_s",
                "occupied_iou",
                "tu_success_rate",
                "tu_succeeded",
                "tu_total",
                "prediction_known_fraction",
            ]
        )
        self.csv_file.flush()

        metadata = {
            "method": args.method,
            "run_id": args.run_id,
            "prediction_topic": args.prediction_topic,
            "event_topic": args.event_topic,
            "gt_occupied": str(args.gt_occupied.resolve()),
            "gt_valid": str(args.gt_valid.resolve()),
            "canonical_resolution_m": CANVAS_RESOLUTION_M,
            "tu_resolution_m": TU_RESOLUTION_M,
            "occupied_threshold": args.occupied_threshold,
            "unknown_as_occupied_for_tu": True,
            "tu_goals": int(count),
            "seed": int(args.seed),
            "tu_start_rc": list(self.start_rc),
            "metric_definition": {
                "occupied_iou": "intersection-over-union of predicted occupied cells and structural GT occupied cells",
                "tu": "4-connected A* success over fixed valid-space goals; selected predicted-map path must also be collision-free in structural GT",
            },
        }
        (self.output_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(
            OccupancyGrid,
            args.prediction_topic,
            self._on_prediction,
            qos,
        )
        self.create_subscription(String, args.event_topic, self._on_event, 100)

        self.latest_prediction: OccupancyGrid | None = None
        self.prediction_seq = 0
        self.last_evaluated_seq = -1
        self.t0_sim_s: float | None = None
        self.create_timer(args.eval_period_s, self._evaluate_latest)

        self.get_logger().warning(
            f"STAGE-3 METRICS: method={args.method}, run={args.run_id}, "
            f"prediction={args.prediction_topic}, output={self.output_dir}"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_event(self, msg: String) -> None:
        if self.t0_sim_s is not None:
            return
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict) or payload.get("event") != "decision":
            return
        sim_s = float(payload.get("sim_time_s", self._now_s()))
        self.t0_sim_s = float(payload.get("decision_start_sim_s", sim_s))
        self.get_logger().warning(
            f"Stage-3 benchmark t=0 at first policy decision: {self.t0_sim_s:.3f}"
        )

    def _on_prediction(self, msg: OccupancyGrid) -> None:
        self.latest_prediction = msg
        self.prediction_seq += 1
        if self.t0_sim_s is None and self.args.start_on_first_prediction:
            self.t0_sim_s = self._now_s()
            self.get_logger().warning(
                "No decision event received; benchmark clock started on first prediction"
            )

    def _occupied_iou(self, pred_occ: np.ndarray) -> float:
        intersection = int(np.count_nonzero(pred_occ & self.gt_occupied_raw))
        union = int(np.count_nonzero(pred_occ | self.gt_occupied_raw))
        return float(intersection / union) if union else 0.0

    def _tu(self, pred_occ_raw: np.ndarray, known_raw: np.ndarray) -> tuple[float, int, int]:
        pred_occ = _reduce_any_2(pred_occ_raw)
        pred_known = _reduce_any_2(known_raw)
        # MapEx benchmark uses unknown_as_occ=true for planning.
        blocked = pred_occ | ~pred_known

        succeeded = 0
        total = int(len(self.tu_goals))
        for goal_arr in self.tu_goals:
            goal = (int(goal_arr[0]), int(goal_arr[1]))
            path = _plan_path(blocked, self.start_rc, goal)
            if path is None or len(path) == 0:
                continue
            rows = path[:, 0]
            cols = path[:, 1]
            in_bounds = (
                (rows >= 0)
                & (rows < self.gt_occupied.shape[0])
                & (cols >= 0)
                & (cols < self.gt_occupied.shape[1])
            )
            if not bool(np.all(in_bounds)):
                continue
            if np.any(self.gt_occupied[rows, cols]):
                continue
            succeeded += 1

        return (
            float(succeeded / total) if total else float("nan"),
            succeeded,
            total,
        )

    def _evaluate_latest(self) -> None:
        if self.latest_prediction is None or self.t0_sim_s is None:
            return
        if self.last_evaluated_seq == self.prediction_seq:
            return

        try:
            pred_occ, known = _occupancygrid_to_canvas(
                self.latest_prediction,
                occupied_threshold=self.args.occupied_threshold,
            )
            iou = self._occupied_iou(pred_occ)
            tu, tu_ok, tu_total = self._tu(pred_occ, known)
        except Exception as exc:
            self.get_logger().error(f"Stage-3 metric evaluation failed: {exc}")
            return

        elapsed = max(0.0, self._now_s() - self.t0_sim_s)
        known_fraction = float(np.count_nonzero(known) / known.size)
        self.writer.writerow(
            [
                f"{elapsed:.6f}",
                f"{iou:.8f}",
                f"{tu:.8f}",
                tu_ok,
                tu_total,
                f"{known_fraction:.8f}",
            ]
        )
        self.csv_file.flush()
        self.last_evaluated_seq = self.prediction_seq
        self.get_logger().info(
            f"Stage3 t={elapsed:.1f}s IoU={iou:.4f} TU={tu:.4f} "
            f"({tu_ok}/{tu_total})"
        )

    def destroy_node(self) -> bool:
        try:
            self.csv_file.flush()
            self.csv_file.close()
        finally:
            return super().destroy_node()


def parse_args() -> tuple[argparse.Namespace, list[str]]:
    research_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Record Hospital Stage-3 IoU and TU")
    parser.add_argument("--method", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--gt-occupied", type=Path, required=True)
    parser.add_argument("--gt-valid", type=Path, required=True)
    parser.add_argument("--prediction-topic", default="/mapex/predicted_map")
    parser.add_argument("--event-topic", default="/exploration/events")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=research_dir / "results" / "stage3_metrics",
    )
    parser.add_argument("--tu-goals", type=int, default=100)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--eval-period-s", type=float, default=10.0)
    parser.add_argument("--occupied-threshold", type=int, default=50)
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--start-on-first-prediction", action="store_true")
    args, ros_args = parser.parse_known_args()

    if args.tu_goals <= 0:
        parser.error("--tu-goals must be > 0")
    if args.eval_period_s <= 0:
        parser.error("--eval-period-s must be > 0")
    if not 0 <= args.occupied_threshold <= 100:
        parser.error("--occupied-threshold must be in [0, 100]")
    return args, ros_args


def main() -> None:
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node: Stage3MetricsRecorder | None = None
    try:
        node = Stage3MetricsRecorder(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
