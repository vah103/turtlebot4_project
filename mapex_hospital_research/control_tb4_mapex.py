#!/usr/bin/env python3
"""
TurtleBot4 MapEx closed-loop exploration policy.

This file is the Stage-3 "pure MapEx" counterpart of control_tb4.py:
ROS2 map/pose -> MapEx frontier candidates -> LaMa ensemble -> mean/variance
-> probabilistic visibility -> information gain -> I / Euclidean distance
-> best frontier -> Nav2 -> repeat after the waypoint is no longer active.

Primary method source:
  Ho et al., "MapEx: Indoor Structure Exploration with Probabilistic
  Information Gain from Global Map Predictions", arXiv:2409.15590,
  Sec. IV and Algorithm 1.

Reference implementation:
  https://github.com/castacks/MapEx
  pinned source SHA: 53636bd1c79153acc3c74a532837d78c926bae5e

Paper / official settings reproduced here:
  - observed map labels: free=0, unknown=0.5, occupied=1
  - map predictor resolution: 0.1 m / pixel
  - LaMa ensemble size: 3
  - frontier: free cell adjacent to unknown in 8-neighbourhood
  - frontier clusters: 8-connected, keep regions with size > 10
  - representative: actual frontier cell closest to region arithmetic mean
  - probabilistic raycast: 360 deg, 20 m, 250 rays
  - accumulated predicted occupancy stopping threshold epsilon = 0.8
  - information gain: sum of ensemble variance inside visible AND unknown cells
  - frontier score: information_gain / Euclidean_distance
  - choose frontier with maximum score

The paper states that occupancy is accumulated along each ray until epsilon is
reached. That paper definition is implemented literally here.

Runtime dependency:
The official MapEx LaMa submodule and pretrained ensemble weights are not copied
into this repository. Set MAPEX_ROOT (or ROS parameter "mapex_root") to a local
clone of castacks/MapEx containing:
  lama/
  pretrained_models/weights/lama_ensemble/train_1/models/best.ckpt
  pretrained_models/weights/lama_ensemble/train_2/models/best.ckpt
  pretrained_models/weights/lama_ensemble/train_3/models/best.ckpt

This file intentionally does NOT include Stage-3 metrics/recording/oracle code.
It is only the exploration policy.
"""

from __future__ import annotations

import logging
import math
import os
from pathlib import Path
import sys
import time
from typing import Iterable

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("control_tb4_mapex")


MAPEX_RESOLUTION_M = 0.10
ENSEMBLE_SIZE = 3
FRONTIER_REGION_SIZE_THRESHOLD = 10
PROB_RAYCAST_EPSILON = 0.8
PRED_VIS_RANGE_M = 20.0
PRED_VIS_NUM_RAYS = 250
MODEL_DIVISOR = 16


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _bresenham(start: tuple[int, int], end: tuple[int, int]) -> Iterable[tuple[int, int]]:
    """Yield integer (row, col) cells on a line, including both endpoints."""
    r0, c0 = start
    r1, c1 = end
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc

    while True:
        yield r0, c0
        if r0 == r1 and c0 == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r0 += sr
        if e2 < dr:
            err += dr
            c0 += sc


def ros_occupancy_to_mapex(data: np.ndarray) -> np.ndarray:
    """ROS OccupancyGrid -> MapEx labels: 0 free, 0.5 unknown, 1 occupied."""
    out = np.full(data.shape, 0.5, dtype=np.float32)
    out[data == 0] = 0.0
    out[data >= 50] = 1.0
    return out


def downsample_to_mapex_resolution(
    obs_map: np.ndarray,
    source_resolution_m: float,
    target_resolution_m: float = MAPEX_RESOLUTION_M,
) -> np.ndarray:
    """Convert the SLAM grid to the paper's 0.1 m/pixel representation."""
    ratio = target_resolution_m / source_resolution_m
    factor = int(round(ratio))
    if factor < 1 or not math.isclose(ratio, factor, rel_tol=0.0, abs_tol=1e-6):
        raise RuntimeError(
            f"MapEx requires an integer downsample to {target_resolution_m:.3f} m/pixel; "
            f"received SLAM resolution {source_resolution_m:.6f} m/pixel."
        )
    if factor == 1:
        return obs_map.copy()

    h, w = obs_map.shape
    pad_h = (-h) % factor
    pad_w = (-w) % factor
    padded = np.pad(
        obs_map,
        ((0, pad_h), (0, pad_w)),
        mode="constant",
        constant_values=0.5,
    )
    hh, ww = padded.shape
    blocks = padded.reshape(hh // factor, factor, ww // factor, factor)

    has_occ = np.any(blocks == 1.0, axis=(1, 3))
    has_unknown = np.any(blocks == 0.5, axis=(1, 3))
    coarse = np.zeros((hh // factor, ww // factor), dtype=np.float32)
    coarse[has_unknown] = 0.5
    coarse[has_occ] = 1.0
    return coarse


def extract_frontier_regions(
    obs_map: np.ndarray,
    min_region_size: int = FRONTIER_REGION_SIZE_THRESHOLD,
) -> tuple[list[np.ndarray], np.ndarray]:
    """
    Official MapEx frontier semantics.

    Frontier cell = known free cell with at least one unknown 8-neighbour.
    Frontier regions are 8-connected. Regions are kept only when size > 10.
    Representative = actual frontier cell closest to arithmetic mean.
    """
    unknown = obs_map == 0.5
    free = obs_map == 0.0
    h, w = obs_map.shape

    adjacent_unknown = np.zeros_like(unknown, dtype=bool)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            src_r0 = max(0, -dr)
            src_r1 = min(h, h - dr)
            src_c0 = max(0, -dc)
            src_c1 = min(w, w - dc)
            dst_r0 = src_r0 + dr
            dst_r1 = src_r1 + dr
            dst_c0 = src_c0 + dc
            dst_c1 = src_c1 + dc
            adjacent_unknown[dst_r0:dst_r1, dst_c0:dst_c1] |= unknown[
                src_r0:src_r1, src_c0:src_c1
            ]

    frontier_mask = free & adjacent_unknown
    visited = np.zeros_like(frontier_mask, dtype=bool)
    regions: list[np.ndarray] = []
    representatives: list[np.ndarray] = []

    for r, c in np.argwhere(frontier_mask):
        if visited[r, c]:
            continue
        stack = [(int(r), int(c))]
        visited[r, c] = True
        cells: list[tuple[int, int]] = []

        while stack:
            cr, cc = stack.pop()
            cells.append((cr, cc))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = cr + dr, cc + dc
                    if (
                        0 <= nr < h
                        and 0 <= nc < w
                        and frontier_mask[nr, nc]
                        and not visited[nr, nc]
                    ):
                        visited[nr, nc] = True
                        stack.append((nr, nc))

        if len(cells) <= min_region_size:
            continue

        region = np.asarray(cells, dtype=np.int32)
        center = np.mean(region, axis=0)
        representative = region[np.argmin(np.linalg.norm(region - center, axis=1))]
        regions.append(region)
        representatives.append(representative)

    return representatives, frontier_mask


def pad_to_divisor(
    obs_map: np.ndarray,
    divisor: int = MODEL_DIVISOR,
) -> tuple[np.ndarray, int, int]:
    """Match MapEx default_map_eval: center-pad to a multiple of 16, no resize."""
    h, w = obs_map.shape
    target_h = int(math.ceil(h / divisor) * divisor)
    target_w = int(math.ceil(w / divisor) * divisor)
    dh = target_h - h
    dw = target_w - w
    top = dh // 2
    bottom = dh - top
    left = dw // 2
    right = dw - left
    padded = np.pad(
        obs_map,
        ((top, bottom), (left, right)),
        mode="constant",
        constant_values=0.0,
    )
    return padded.astype(np.float32, copy=False), top, left


def probabilistic_visibility_mask(
    mean_map: np.ndarray,
    observed_map: np.ndarray,
    viewpoint: tuple[int, int],
    ray_range_cells: int,
    num_rays: int = PRED_VIS_NUM_RAYS,
    epsilon: float = PROB_RAYCAST_EPSILON,
) -> np.ndarray:
    """
    MapEx Sec. IV-C probabilistic sensor visibility.

    Each ray accumulates occupancy from the mean predicted map and stops when
    accumulated occupancy reaches epsilon. The ordered endpoints form the
    sensor boundary; flood fill gives coverage. Already-observed cells are
    removed afterwards.
    """
    h, w = mean_map.shape
    vr, vc = viewpoint
    if not (0 <= vr < h and 0 <= vc < w):
        return np.zeros_like(mean_map, dtype=bool)

    endpoints: list[tuple[int, int]] = []
    for angle in np.linspace(0.0, 2.0 * math.pi, num_rays):
        end_r = int(round(vr + ray_range_cells * math.sin(angle)))
        end_c = int(round(vc + ray_range_cells * math.cos(angle)))

        delta = 0.0
        last_valid = (vr, vc)
        for r, c in _bresenham((vr, vc), (end_r, end_c)):
            if r < 0 or r >= h or c < 0 or c >= w:
                break
            last_valid = (r, c)
            delta += float(np.clip(mean_map[r, c], 0.0, 1.0))
            if delta >= epsilon:
                break
        endpoints.append(last_valid)

    if len(endpoints) < 3:
        return np.zeros_like(mean_map, dtype=bool)

    boundary = np.zeros((h, w), dtype=np.uint8)
    polygon = np.asarray([(c, r) for r, c in endpoints], dtype=np.int32)
    cv2.polylines(boundary, [polygon], isClosed=True, color=1, thickness=1)

    seed_r, seed_c = vr, vc
    if boundary[seed_r, seed_c] != 0:
        found = False
        for radius in range(1, 4):
            for rr in range(max(0, vr - radius), min(h, vr + radius + 1)):
                for cc in range(max(0, vc - radius), min(w, vc + radius + 1)):
                    if boundary[rr, cc] == 0:
                        seed_r, seed_c = rr, cc
                        found = True
                        break
                if found:
                    break
            if found:
                break
        if not found:
            return np.zeros_like(mean_map, dtype=bool)

    flood_canvas = boundary.copy()
    flood_mask = np.zeros((h + 2, w + 2), dtype=np.uint8)
    cv2.floodFill(
        flood_canvas,
        flood_mask,
        seedPoint=(int(seed_c), int(seed_r)),
        newVal=2,
        flags=4,
    )
    sensor_coverage = flood_canvas == 2
    return sensor_coverage & (observed_map == 0.5)


class LamaEnsemble:
    """Minimal loader/inference wrapper for the three MapEx LaMa Gi models."""

    def __init__(self, mapex_root: Path, device: str) -> None:
        self.mapex_root = mapex_root.expanduser().resolve()
        self.device = device
        lama_root = self.mapex_root / "lama"
        if not lama_root.is_dir():
            raise RuntimeError(
                f"MapEx LaMa submodule not found: {lama_root}. "
                "Clone castacks/MapEx with --recurse-submodules."
            )
        if str(lama_root) not in sys.path:
            sys.path.insert(0, str(lama_root))

        try:
            import torch
            import yaml
            from omegaconf import OmegaConf
            from saicinpainting.training.trainers import load_checkpoint
        except Exception as exc:
            raise RuntimeError(
                "Cannot import the official MapEx/LaMa runtime. "
                "Activate the MapEx lama environment before launching this node."
            ) from exc

        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                f"Requested MapEx device '{device}', but CUDA is unavailable."
            )

        self.torch = torch
        self.yaml = yaml
        self.OmegaConf = OmegaConf
        self.load_checkpoint = load_checkpoint

        ensemble_dir = (
            self.mapex_root
            / "pretrained_models"
            / "weights"
            / "lama_ensemble"
        )
        if not ensemble_dir.is_dir():
            raise RuntimeError(
                f"MapEx ensemble weights not found: {ensemble_dir}. "
                "Download the official pretrained weights described in the MapEx README."
            )

        model_dirs = sorted(p for p in ensemble_dir.iterdir() if p.is_dir())
        if len(model_dirs) < ENSEMBLE_SIZE:
            raise RuntimeError(
                f"MapEx requires {ENSEMBLE_SIZE} ensemble models; "
                f"found {len(model_dirs)} under {ensemble_dir}."
            )

        self.models = [self._load_model(path) for path in model_dirs[:ENSEMBLE_SIZE]]
        logger.info(
            "Loaded MapEx ensemble: %s",
            ", ".join(path.name for path in model_dirs[:ENSEMBLE_SIZE]),
        )

    def _load_model(self, model_path: Path):
        config_path = model_path / "config.yaml"
        checkpoint_path = model_path / "models" / "best.ckpt"
        if not config_path.is_file() or not checkpoint_path.is_file():
            raise RuntimeError(
                f"Incomplete MapEx model directory: {model_path} "
                "(need config.yaml and models/best.ckpt)."
            )

        with config_path.open("r", encoding="utf-8") as f:
            train_config = self.OmegaConf.create(self.yaml.safe_load(f))
        train_config.training_model.predict_only = True
        train_config.visualizer.kind = "noop"

        model = self.load_checkpoint(
            train_config,
            str(checkpoint_path),
            strict=False,
            map_location=self.device,
        ).to(self.device)
        model.freeze()
        model.eval()
        return model

    def predict_mean_variance(self, observed_map: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Eq. 2-3: Pi,t = Gi(Ot), then pixel-wise variance over 3 predictions."""
        torch = self.torch
        image_np = np.stack([observed_map, observed_map, observed_map], axis=0)
        image = torch.from_numpy(image_np).unsqueeze(0).float().to(self.device)
        mask_np = np.isclose(observed_map, 0.5).astype(np.float32)[None, None, :, :]
        mask = torch.from_numpy(mask_np).to(self.device)

        predictions = []
        with torch.no_grad():
            for model in self.models:
                batch = {"image": image.clone(), "mask": mask.clone()}
                output = model(batch)
                predictions.append(output["inpainted"][0, 0].detach())

        prediction_stack = torch.stack(predictions, dim=0)
        variance = torch.var(prediction_stack, dim=0)
        mean = torch.mean(prediction_stack, dim=0)

        return mean.float().cpu().numpy(), variance.float().cpu().numpy()


class MapExNavigation(Node):
    def __init__(self) -> None:
        super().__init__("mapex_exploration")

        default_root = os.environ.get("MAPEX_ROOT", str(Path.home() / "MapEx"))
        self.declare_parameter("mapex_root", default_root)
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("mapex_resolution_m", MAPEX_RESOLUTION_M)
        self.declare_parameter("sensor_range_m", PRED_VIS_RANGE_M)
        self.declare_parameter("num_rays", PRED_VIS_NUM_RAYS)
        self.declare_parameter("epsilon", PROB_RAYCAST_EPSILON)

        self.mapex_root = Path(
            self.get_parameter("mapex_root").get_parameter_value().string_value
        )
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.target_resolution = (
            self.get_parameter("mapex_resolution_m").get_parameter_value().double_value
        )
        self.sensor_range_m = (
            self.get_parameter("sensor_range_m").get_parameter_value().double_value
        )
        self.num_rays = self.get_parameter("num_rays").get_parameter_value().integer_value
        self.epsilon = self.get_parameter("epsilon").get_parameter_value().double_value

        if not math.isclose(self.target_resolution, MAPEX_RESOLUTION_M, abs_tol=1e-9):
            self.get_logger().warning(
                "Paper-faithful MapEx uses 0.1 m/pixel; current parameter is %.3f m/pixel.",
                self.target_resolution,
            )

        self.ensemble = LamaEnsemble(self.mapex_root, self.device)

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(Path, "/plan", self.plan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Waiting for Nav2 NavigateToPose...")
        self.nav_client.wait_for_server()

        self.frontier_marker_pub = self.create_publisher(
            MarkerArray, "mapex/frontier_markers", 10
        )
        self.selected_marker_pub = self.create_publisher(
            Marker, "mapex/selected_frontier", 10
        )
        self.selected_path_pub = self.create_publisher(
            Path, "/frontier_selected_path", 10
        )

        self.latest_map: OccupancyGrid | None = None
        self.current_goal: tuple[float, float] | None = None
        self.current_goal_handle = None
        self.last_selected_grid: tuple[int, int] | None = None
        self.decision_id = 0

        self.create_timer(1.0, self.exploration_loop)
        self.get_logger().info("MapEx exploration mode active.")

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def plan_callback(self, msg: Path) -> None:
        self.selected_path_pub.publish(msg)

    def _robot_pose_map(self) -> tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as exc:
            self.get_logger().debug("Waiting for map->base_link TF: %s", exc)
            return None

        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        yaw = quaternion_to_yaw(tf.transform.rotation)
        return x, y, yaw

    def _prepare_observed_map(self) -> tuple[np.ndarray, float, float]:
        assert self.latest_map is not None
        msg = self.latest_map
        raw = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )
        labels = ros_occupancy_to_mapex(raw)
        coarse = downsample_to_mapex_resolution(
            labels,
            float(msg.info.resolution),
            self.target_resolution,
        )
        return (
            coarse,
            float(msg.info.origin.position.x),
            float(msg.info.origin.position.y),
        )

    def _world_to_grid(
        self,
        x: float,
        y: float,
        origin_x: float,
        origin_y: float,
    ) -> tuple[int, int]:
        col = int(math.floor((x - origin_x) / self.target_resolution))
        row = int(math.floor((y - origin_y) / self.target_resolution))
        return row, col

    def _grid_to_world(
        self,
        row: int,
        col: int,
        origin_x: float,
        origin_y: float,
    ) -> tuple[float, float]:
        x = origin_x + (col + 0.5) * self.target_resolution
        y = origin_y + (row + 0.5) * self.target_resolution
        return x, y

    def _score_frontiers(
        self,
        observed: np.ndarray,
        representatives: list[np.ndarray],
        robot_grid: tuple[int, int],
    ) -> list[tuple[float, float, float, tuple[int, int]]]:
        padded_obs, pad_top, pad_left = pad_to_divisor(observed)
        mean_map, variance_map = self.ensemble.predict_mean_variance(padded_obs)

        if mean_map.shape != padded_obs.shape or variance_map.shape != padded_obs.shape:
            raise RuntimeError(
                "LaMa output shape mismatch: "
                f"obs={padded_obs.shape}, mean={mean_map.shape}, var={variance_map.shape}"
            )

        ray_range_cells = int(round(self.sensor_range_m / self.target_resolution))
        robot_rc = np.asarray(robot_grid, dtype=np.float64)
        scored = []

        for representative in representatives:
            r = int(representative[0])
            c = int(representative[1])
            pr = r + pad_top
            pc = c + pad_left

            visibility = probabilistic_visibility_mask(
                mean_map,
                padded_obs,
                (pr, pc),
                ray_range_cells=ray_range_cells,
                num_rays=self.num_rays,
                epsilon=self.epsilon,
            )
            information_gain = float(np.sum(variance_map[visibility]))

            distance_cells = float(
                np.linalg.norm(np.asarray([r, c], dtype=np.float64) - robot_rc)
            )
            score = information_gain / max(distance_cells, 1e-6)
            distance_m = distance_cells * self.target_resolution
            scored.append((score, information_gain, distance_m, (r, c)))

        scored.sort(key=lambda item: item[0], reverse=True)
        return scored

    def exploration_loop(self) -> None:
        # Algorithm 1 only selects a new waypoint when one is not available.
        if self.current_goal is not None or self.latest_map is None:
            return

        robot_pose = self._robot_pose_map()
        if robot_pose is None:
            return
        robot_x, robot_y, robot_yaw = robot_pose

        try:
            observed, origin_x, origin_y = self._prepare_observed_map()
        except Exception as exc:
            self.get_logger().error("MapEx map preparation failed: %s", exc)
            return

        robot_grid = self._world_to_grid(robot_x, robot_y, origin_x, origin_y)
        rr, rc = robot_grid
        if not (0 <= rr < observed.shape[0] and 0 <= rc < observed.shape[1]):
            self.get_logger().warning(
                "Robot is outside MapEx occupancy grid: grid=(%d,%d), shape=%s",
                rr,
                rc,
                observed.shape,
            )
            return

        representatives, _frontier_mask = extract_frontier_regions(observed)
        self._publish_frontier_markers(representatives, origin_x, origin_y)

        if not representatives:
            self.get_logger().info("No MapEx frontier region > 10 cells.")
            return

        self.decision_id += 1
        t0 = time.perf_counter()
        try:
            scored = self._score_frontiers(observed, representatives, robot_grid)
        except Exception as exc:
            self.get_logger().exception("MapEx scoring failed: %s", exc)
            return

        if not scored:
            self.get_logger().info("No scoreable MapEx frontier.")
            return

        best_score, best_ig, best_distance_m, best_grid = scored[0]
        goal_x, goal_y = self._grid_to_world(
            best_grid[0], best_grid[1], origin_x, origin_y
        )
        elapsed = time.perf_counter() - t0

        self.get_logger().info(
            "MapEx decision %d: frontier=(%d,%d), IG=%.6f, distance=%.2f m, "
            "score=%.6f, candidates=%d, compute=%.2f s",
            self.decision_id,
            best_grid[0],
            best_grid[1],
            best_ig,
            best_distance_m,
            best_score,
            len(scored),
            elapsed,
        )

        self.last_selected_grid = best_grid
        self.current_goal = (goal_x, goal_y)
        self._publish_selected_marker(goal_x, goal_y, best_score)
        self._send_goal(goal_x, goal_y, robot_yaw)

    def _send_goal(self, x: float, y: float, current_yaw: float) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0

        # Frontier is position-only; Hospital Nav2 must ignore final-yaw objective.
        goal.pose.pose.orientation = yaw_to_quaternion(current_yaw)

        future = self.nav_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            self.get_logger().error("NavigateToPose send failed: %s", exc)
            self.current_goal = None
            return

        if not handle.accepted:
            self.get_logger().warning("MapEx frontier goal rejected by Nav2.")
            self.current_goal = None
            return

        self.current_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("MapEx frontier goal succeeded.")
            else:
                self.get_logger().warning(
                    "MapEx frontier goal ended with Nav2 status %d.", status
                )
        except Exception as exc:
            self.get_logger().error("NavigateToPose result failed: %s", exc)
        finally:
            self.current_goal_handle = None
            self.current_goal = None

    def _feedback_callback(self, _feedback_msg) -> None:
        return

    def _publish_frontier_markers(
        self,
        representatives: list[np.ndarray],
        origin_x: float,
        origin_y: float,
    ) -> None:
        msg = MarkerArray()

        delete = Marker()
        delete.header.frame_id = "map"
        delete.header.stamp = self.get_clock().now().to_msg()
        delete.action = Marker.DELETEALL
        msg.markers.append(delete)

        for idx, rc in enumerate(representatives):
            x, y = self._grid_to_world(int(rc[0]), int(rc[1]), origin_x, origin_y)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "mapex_frontiers"
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.a = 1.0
            marker.color.r = 1.0
            msg.markers.append(marker)

        self.frontier_marker_pub.publish(msg)

    def _publish_selected_marker(self, x: float, y: float, score: float) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "mapex_selected"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.g = 1.0
        marker.text = f"score={score:.6f}"
        self.selected_marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MapExNavigation()
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
