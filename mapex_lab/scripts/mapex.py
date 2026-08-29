#!/usr/bin/env python3
"""Paper-faithful MapEx policy on the shared nf_basic ROS/Nav2 execution layer.

This node intentionally reuses ``NearestEuclideanFrontier`` for everything that
is not MapEx's frontier utility policy:

- ROS2 /map + TF plumbing
- exact frontier x/y NavigateToPose goals
- /plan visualization and path-guided recovery
- NO_VALID_PATH suppression
- repeated ComputePathToPose terminal revalidation
- completion/status/marker behavior

Only the policy decision is replaced:

    observed map
      -> official MapEx frontier geometry
      -> 3-member LaMa ensemble
      -> mean + pixel-wise variance
      -> probabilistic visibility from every frontier
      -> IG = sum(variance over visible AND currently-unknown cells)
      -> score = IG / Euclidean distance
      -> highest-score selectable frontier

Method source:
    Ho et al., "MapEx: Indoor Structure Exploration with Probabilistic
    Information Gain from Global Map Predictions", ICRA 2025 / arXiv:2409.15590.

Pinned reference implementation:
    castacks/MapEx @ 53636bd1c79153acc3c74a532837d78c926bae5e

Hospital adaptation shared with Nearest:
    The original MapEx <1 m waypoint rejection is NOT applied. Hospital keeps
    close frontiers and lets the common ROS/Nav2 execution layer determine
    reachability. No separate 0.5 m guard from nf_basic is applied here.

Important implementation choice:
    Sec. IV-C of the paper states that occupancy is accumulated along each ray
    until epsilon is reached. This file implements that definition literally:
    ``delta`` is initialized once per ray, not once per pixel.

Model loading:
    Preferred: official MapEx layout under MAPEX_ROOT (default ~/MapEx):
      pretrained_models/weights/lama_ensemble/<member>/config.yaml
      pretrained_models/weights/lama_ensemble/<member>/models/best.ckpt

    Optional: MAPEX_MODEL_FILES may contain three comma-separated torch files.
    This works only when each file serializes a complete torch.nn.Module (or a
    dict containing one under ``model``). A bare state_dict cannot be restored
    without its architecture/config and is rejected explicitly.

The LaMa input is the Hospital global occupancy map at 0.10 m/cell, center-
padded to a multiple of 16 without resizing so world geometry is preserved.
"""

from __future__ import annotations

from collections import deque
import math
import os
from pathlib import Path
import sys
import time
from typing import Iterable

import cv2
import numpy as np
import rclpy

from nf_basic import NearestEuclideanFrontier


MAPEX_REFERENCE_COMMIT = "53636bd1c79153acc3c74a532837d78c926bae5e"
MAPEX_RESOLUTION_M = 0.10
ENSEMBLE_SIZE = 3
PRED_VIS_RANGE_M = 20.0
PRED_VIS_NUM_RAYS = 250
PROB_RAYCAST_EPSILON = 0.8
MODEL_DIVISOR = 16


def _bresenham(
    start: tuple[int, int],
    end: tuple[int, int],
) -> Iterable[tuple[int, int]]:
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


def ros_occupancy_to_mapex(grid: np.ndarray) -> np.ndarray:
    """ROS OccupancyGrid labels -> MapEx labels: free=0, unknown=0.5, occupied=1."""
    observed = np.ones(grid.shape, dtype=np.float32)
    observed[grid == 0] = 0.0
    observed[grid < 0] = 0.5
    observed[grid > 0] = 1.0
    return observed


def pad_to_divisor(
    observed_map: np.ndarray,
    divisor: int = MODEL_DIVISOR,
) -> tuple[np.ndarray, int, int]:
    """Center-pad without resize and return (padded, top_offset, left_offset)."""
    height, width = observed_map.shape
    target_h = int(math.ceil(height / divisor) * divisor)
    target_w = int(math.ceil(width / divisor) * divisor)
    dh = target_h - height
    dw = target_w - width
    top = dh // 2
    bottom = dh - top
    left = dw // 2
    right = dw - left

    padded = np.pad(
        observed_map,
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
    """MapEx Sec. IV-C probabilistic visibility mask.

    For every hypothetical ray, predicted occupancy from the ensemble mean is
    accumulated along the ray. The ray stops when accumulated occupancy reaches
    epsilon or the map/range boundary. Ordered ray endpoints form the sensor
    boundary; flood fill gives the coverage region; already-observed cells are
    then removed, leaving the currently-unknown visibility mask used by Eq. 4.
    """
    height, width = mean_map.shape
    vr, vc = viewpoint
    if not (0 <= vr < height and 0 <= vc < width):
        return np.zeros_like(mean_map, dtype=bool)

    endpoints: list[tuple[int, int]] = []
    for angle in np.linspace(
        0.0,
        2.0 * math.pi,
        num_rays,
        endpoint=False,
    ):
        end_r = int(round(vr + ray_range_cells * math.sin(angle)))
        end_c = int(round(vc + ray_range_cells * math.cos(angle)))

        # Paper definition: Delta starts once per ray and accumulates pixel
        # occupancy until it reaches epsilon.
        delta = 0.0
        last_valid = (vr, vc)
        first_cell = True
        for row, col in _bresenham((vr, vc), (end_r, end_c)):
            if row < 0 or row >= height or col < 0 or col >= width:
                break

            last_valid = (row, col)

            # Do not let the frontier/viewpoint cell itself terminate a ray.
            # It is a known free frontier cell under the shared MapEx geometry.
            if first_cell:
                first_cell = False
                continue

            delta += float(np.clip(mean_map[row, col], 0.0, 1.0))
            if delta >= epsilon:
                break

        endpoints.append(last_valid)

    if len(endpoints) < 3:
        return np.zeros_like(mean_map, dtype=bool)

    boundary = np.zeros((height, width), dtype=np.uint8)
    polygon = np.asarray([(col, row) for row, col in endpoints], dtype=np.int32)
    cv2.polylines(boundary, [polygon], isClosed=True, color=1, thickness=1)

    seed_r, seed_c = vr, vc
    if boundary[seed_r, seed_c] != 0:
        found = False
        for radius in range(1, 4):
            for row in range(
                max(0, vr - radius),
                min(height, vr + radius + 1),
            ):
                for col in range(
                    max(0, vc - radius),
                    min(width, vc + radius + 1),
                ):
                    if boundary[row, col] == 0:
                        seed_r, seed_c = row, col
                        found = True
                        break
                if found:
                    break
            if found:
                break

        if not found:
            return np.zeros_like(mean_map, dtype=bool)

    flood_canvas = boundary.copy()
    flood_mask = np.zeros((height + 2, width + 2), dtype=np.uint8)
    cv2.floodFill(
        flood_canvas,
        flood_mask,
        seedPoint=(int(seed_c), int(seed_r)),
        newVal=2,
        flags=4,
    )

    sensor_coverage = flood_canvas == 2
    return sensor_coverage & np.isclose(observed_map, 0.5)


class LamaEnsemble:
    """Loader/inference wrapper for the three independent MapEx LaMa Gi models."""

    def __init__(
        self,
        mapex_root: Path,
        device: str,
        model_files: list[Path] | None = None,
    ) -> None:
        self.mapex_root = mapex_root.expanduser().resolve()
        self.device = device

        try:
            import torch
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "PyTorch is required for MapEx LaMa inference."
            ) from exc

        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                f"Requested MapEx device '{device}', but CUDA is unavailable."
            )

        self.torch = torch

        if model_files:
            if len(model_files) != ENSEMBLE_SIZE:
                raise RuntimeError(
                    f"MapEx requires exactly {ENSEMBLE_SIZE} model files; "
                    f"received {len(model_files)}."
                )
            self.models = [self._load_serialized_model(path) for path in model_files]
            self.source_names = [str(path) for path in model_files]
        else:
            self.models, self.source_names = self._load_official_model_dirs()

    def _load_serialized_model(self, path: Path):
        path = path.expanduser().resolve()
        if not path.is_file():
            raise RuntimeError(f"MapEx model file not found: {path}")

        torch = self.torch
        try:
            loaded = torch.load(
                str(path),
                map_location=self.device,
                weights_only=False,
            )
        except TypeError:
            loaded = torch.load(str(path), map_location=self.device)

        model = loaded
        if isinstance(loaded, dict):
            candidate = loaded.get("model")
            if isinstance(candidate, torch.nn.Module):
                model = candidate

        if not isinstance(model, torch.nn.Module):
            raise RuntimeError(
                f"{path} does not contain a complete torch.nn.Module. "
                "A bare state_dict is insufficient; use the official MapEx "
                "model directory (config.yaml + models/best.ckpt) or provide "
                "a serialized complete model."
            )

        model = model.to(self.device)
        model.eval()
        for parameter in model.parameters():
            parameter.requires_grad_(False)
        return model

    def _load_official_model_dirs(self):
        lama_root = self.mapex_root / "lama"
        if not lama_root.is_dir():
            raise RuntimeError(
                f"MapEx LaMa submodule not found: {lama_root}. "
                "Set MAPEX_ROOT to a castacks/MapEx clone with submodules."
            )

        if str(lama_root) not in sys.path:
            sys.path.insert(0, str(lama_root))

        try:
            import yaml
            from omegaconf import OmegaConf
            from saicinpainting.training.trainers import load_checkpoint
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "Cannot import the official MapEx/LaMa runtime. Activate the "
                "LaMa environment used by the MapEx repository."
            ) from exc

        ensemble_dir = Path(
            os.environ.get(
                "MAPEX_ENSEMBLE_DIR",
                str(
                    self.mapex_root
                    / "pretrained_models"
                    / "weights"
                    / "lama_ensemble"
                ),
            )
        ).expanduser().resolve()

        if not ensemble_dir.is_dir():
            raise RuntimeError(
                f"MapEx ensemble directory not found: {ensemble_dir}."
            )

        model_dirs = sorted(path for path in ensemble_dir.iterdir() if path.is_dir())
        if len(model_dirs) < ENSEMBLE_SIZE:
            raise RuntimeError(
                f"MapEx requires {ENSEMBLE_SIZE} ensemble members; found "
                f"{len(model_dirs)} under {ensemble_dir}."
            )

        models = []
        source_names = []
        for model_dir in model_dirs[:ENSEMBLE_SIZE]:
            config_path = model_dir / "config.yaml"
            checkpoint_path = model_dir / "models" / "best.ckpt"
            if not config_path.is_file() or not checkpoint_path.is_file():
                raise RuntimeError(
                    f"Incomplete MapEx model directory: {model_dir}; need "
                    "config.yaml and models/best.ckpt."
                )

            with config_path.open("r", encoding="utf-8") as stream:
                train_config = OmegaConf.create(yaml.safe_load(stream))
            train_config.training_model.predict_only = True
            train_config.visualizer.kind = "noop"

            model = load_checkpoint(
                train_config,
                str(checkpoint_path),
                strict=False,
                map_location=self.device,
            ).to(self.device)
            if hasattr(model, "freeze"):
                model.freeze()
            model.eval()
            models.append(model)
            source_names.append(str(model_dir))

        return models, source_names

    def predict_mean_variance(
        self,
        observed_map: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Eq. 2-3: Pi,t = Gi(Ot), followed by pixel-wise ensemble variance."""
        torch = self.torch

        image_np = np.stack(
            [observed_map, observed_map, observed_map],
            axis=0,
        )
        image = torch.from_numpy(image_np).unsqueeze(0).float().to(self.device)
        mask_np = np.isclose(observed_map, 0.5).astype(np.float32)[
            None,
            None,
            :,
            :,
        ]
        mask = torch.from_numpy(mask_np).to(self.device)

        predictions = []
        with torch.no_grad():
            for model in self.models:
                batch = {
                    "image": image.clone(),
                    "mask": mask.clone(),
                }
                output = model(batch)

                if isinstance(output, dict) and "inpainted" in output:
                    prediction = output["inpainted"][0, 0]
                elif torch.is_tensor(output):
                    tensor = output
                    if tensor.ndim == 4:
                        prediction = tensor[0, 0]
                    elif tensor.ndim == 3:
                        prediction = tensor[0]
                    else:
                        raise RuntimeError(
                            f"Unsupported LaMa tensor output shape: {tuple(tensor.shape)}"
                        )
                else:
                    raise RuntimeError(
                        "Unsupported LaMa model output; expected dict['inpainted'] "
                        "or a tensor."
                    )

                predictions.append(prediction.detach())

        prediction_stack = torch.stack(predictions, dim=0)
        mean = torch.mean(prediction_stack, dim=0)
        variance = torch.var(prediction_stack, dim=0)

        mean_np = np.nan_to_num(
            mean.float().cpu().numpy(),
            nan=0.5,
            posinf=1.0,
            neginf=0.0,
        )
        variance_np = np.nan_to_num(
            variance.float().cpu().numpy(),
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        mean_np = np.clip(mean_np, 0.0, 1.0).astype(np.float32, copy=False)
        variance_np = np.maximum(variance_np, 0.0).astype(np.float32, copy=False)

        # Known occupancy cells are observations, not uncertain predictions.
        known = ~np.isclose(observed_map, 0.5)
        mean_np[known] = observed_map[known]
        variance_np[known] = 0.0
        return mean_np, variance_np


class MapExExplorer(NearestEuclideanFrontier):
    """MapEx frontier scoring with nf_basic navigation/recovery/completion."""

    def __init__(self) -> None:
        # The parent creates all shared ROS/Nav2 execution state and its timer.
        # Python method dispatch makes that timer call this class's
        # exploration_step(), while every navigation callback remains inherited.
        super().__init__()

        default_root = os.environ.get("MAPEX_ROOT", str(Path.home() / "MapEx"))
        default_device = os.environ.get("MAPEX_DEVICE", "cuda:0")
        default_model_files = os.environ.get("MAPEX_MODEL_FILES", "")

        self.declare_parameter("mapex_root", default_root)
        self.declare_parameter("mapex_device", default_device)
        self.declare_parameter("mapex_model_files", default_model_files)
        self.declare_parameter("mapex_resolution_m", MAPEX_RESOLUTION_M)
        self.declare_parameter("mapex_sensor_range_m", PRED_VIS_RANGE_M)
        self.declare_parameter("mapex_num_rays", PRED_VIS_NUM_RAYS)
        self.declare_parameter("mapex_epsilon", PROB_RAYCAST_EPSILON)

        self.mapex_root = Path(str(self.get_parameter("mapex_root").value))
        self.mapex_device = str(self.get_parameter("mapex_device").value)
        self.mapex_resolution_m = float(
            self.get_parameter("mapex_resolution_m").value
        )
        self.mapex_sensor_range_m = float(
            self.get_parameter("mapex_sensor_range_m").value
        )
        self.mapex_num_rays = int(self.get_parameter("mapex_num_rays").value)
        self.mapex_epsilon = float(self.get_parameter("mapex_epsilon").value)

        model_file_text = str(self.get_parameter("mapex_model_files").value).strip()
        model_files = None
        if model_file_text:
            model_files = [
                Path(item.strip())
                for item in model_file_text.split(",")
                if item.strip()
            ]

        if not math.isclose(
            self.mapex_resolution_m,
            MAPEX_RESOLUTION_M,
            rel_tol=0.0,
            abs_tol=1e-9,
        ):
            raise RuntimeError(
                "Paper-faithful MapEx requires 0.10 m/cell policy grid; "
                f"configured {self.mapex_resolution_m:.6f} m/cell."
            )

        self.get_logger().info(
            "Loading MapEx LaMa ensemble; this may take a while..."
        )
        self.ensemble = LamaEnsemble(
            self.mapex_root,
            self.mapex_device,
            model_files=model_files,
        )

        self.mapex_decision_id = 0
        self.last_mean_map: np.ndarray | None = None
        self.last_variance_map: np.ndarray | None = None
        self.last_candidate_metrics: list[dict] = []

        self.get_logger().info(
            "MapEx policy ready: commit=%s, ensemble=%d, grid=%.2f m, "
            "visibility=%.1f m/%d rays, epsilon=%.2f, shared execution=nf_basic",
            MAPEX_REFERENCE_COMMIT,
            ENSEMBLE_SIZE,
            self.mapex_resolution_m,
            self.mapex_sensor_range_m,
            self.mapex_num_rays,
            self.mapex_epsilon,
        )
        for index, source in enumerate(self.ensemble.source_names, start=1):
            self.get_logger().info("MapEx G%d: %s", index, source)

    def _score_mapex_candidates(
        self,
        grid: np.ndarray,
        regions,
        robot_x: float,
        robot_y: float,
    ) -> tuple[list[tuple], list[dict]]:
        observed = ros_occupancy_to_mapex(grid)
        padded_obs, pad_top, pad_left = pad_to_divisor(observed)

        prediction_start = time.perf_counter()
        mean_map, variance_map = self.ensemble.predict_mean_variance(padded_obs)
        prediction_s = time.perf_counter() - prediction_start

        if mean_map.shape != padded_obs.shape or variance_map.shape != padded_obs.shape:
            raise RuntimeError(
                "LaMa output shape mismatch: "
                f"obs={padded_obs.shape}, mean={mean_map.shape}, "
                f"variance={variance_map.shape}"
            )

        self.last_mean_map = mean_map
        self.last_variance_map = variance_map

        ray_range_cells = int(
            round(self.mapex_sensor_range_m / self.mapex_resolution_m)
        )
        candidates = []
        metrics = []

        scoring_start = time.perf_counter()
        for candidate_id, region in enumerate(regions):
            row, col = self.representative(region)
            x, y = self.cell_to_world(row, col)
            distance_m = math.hypot(x - robot_x, y - robot_y)

            padded_row = row + pad_top
            padded_col = col + pad_left
            visibility = probabilistic_visibility_mask(
                mean_map,
                padded_obs,
                (padded_row, padded_col),
                ray_range_cells=ray_range_cells,
                num_rays=self.mapex_num_rays,
                epsilon=self.mapex_epsilon,
            )
            information_gain = float(np.sum(variance_map[visibility]))
            score = information_gain / max(distance_m, 1e-6)

            # nf_basic expects a six-field tuple whose first field is minimized.
            # Store -score there so every inherited sorting/revalidation helper
            # remains structurally compatible while MapEx maximizes score.
            candidate = (
                -score,
                x,
                y,
                row,
                col,
                len(region),
            )
            candidates.append(candidate)
            metrics.append(
                {
                    "candidate_id": candidate_id,
                    "row": int(row),
                    "col": int(col),
                    "x": float(x),
                    "y": float(y),
                    "region_size": int(len(region)),
                    "distance_m": float(distance_m),
                    "information_gain": information_gain,
                    "score": float(score),
                    "visible_unknown_cells": int(np.count_nonzero(visibility)),
                }
            )

        scoring_s = time.perf_counter() - scoring_start
        for metric in metrics:
            metric["ensemble_prediction_s"] = prediction_s
            metric["all_frontier_scoring_s"] = scoring_s

        return candidates, metrics

    def exploration_step(self):
        """MapEx policy decision; all goal execution after selection is inherited."""
        if self.completed:
            return

        if self.map_msg is None or self.goal_active or self.revalidation_active:
            return

        # Preserve the parent's selected-frontier locking and recovery semantics.
        if self.main_goal is not None:
            self.reset_completion_verification("main_frontier_still_pending")
            self.revalidation_signature = None
            x, y = self.main_goal
            self.get_logger().info(
                f"Retrying MapEx main frontier after recovery: x={x:.2f}, y={y:.2f}"
            )
            self.send_navigation_goal(x, y, mode="main")
            return

        robot = self.robot_position()
        if robot is None:
            return

        resolution = float(self.map_msg.info.resolution)
        if not math.isclose(
            resolution,
            self.mapex_resolution_m,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_resolution_mismatch")
            self.publish_status(
                "MAPEX_ERROR",
                reason="policy_grid_resolution_mismatch",
                map_resolution_m=resolution,
                required_resolution_m=self.mapex_resolution_m,
            )
            self.get_logger().error(
                "MapEx requires the hospital_v2 0.10 m/cell /map; got %.6f m/cell. "
                "Not issuing a goal.",
                resolution,
            )
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        grid = np.asarray(
            self.map_msg.data,
            dtype=np.int16,
        ).reshape(height, width)

        # Frontier geometry is deliberately inherited from nf_basic because that
        # implementation already matches the pinned MapEx rule used by Nearest:
        # free==0, unknown<0, 8-neighbour frontier, 8-connected region, size>10,
        # representative = actual frontier cell nearest arithmetic mean.
        mask = self.frontier_mask(grid)
        regions = self.frontier_regions(mask)

        if not regions:
            self.clear_goal_markers()
            self.revalidation_signature = None
            self.observe_no_frontier_terminal_state()
            return

        robot_x, robot_y = robot
        self.mapex_decision_id += 1
        decision_start = time.perf_counter()

        try:
            all_candidates, metrics = self._score_mapex_candidates(
                grid,
                regions,
                robot_x,
                robot_y,
            )
        except Exception as exc:  # noqa: BLE001
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_scoring_failed")
            self.publish_status(
                "MAPEX_ERROR",
                reason="prediction_visibility_or_scoring_failed",
                decision_id=self.mapex_decision_id,
                error=str(exc),
            )
            self.get_logger().exception(
                "MapEx decision %d failed: %s",
                self.mapex_decision_id,
                exc,
            )
            return

        self.last_candidate_metrics = metrics
        if not all_candidates:
            # Regions were non-empty, so this means an internal scoring failure,
            # not exploration completion.
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_no_scoreable_candidate")
            self.publish_status(
                "MAPEX_ERROR",
                reason="frontier_regions_exist_but_no_scoreable_candidate",
                decision_id=self.mapex_decision_id,
                frontier_regions=len(regions),
            )
            return

        metric_by_grid = {
            (metric["row"], metric["col"]): metric for metric in metrics
        }

        candidates = []
        suppressed_no_path = 0
        for candidate in all_candidates:
            _negative_score, x, y, _row, _col, _region_size = candidate
            if self.is_no_valid_path_suppressed(x, y):
                suppressed_no_path += 1
                continue
            candidates.append(candidate)

        if not candidates:
            self.clear_goal_markers()
            if suppressed_no_path == len(all_candidates):
                self.maybe_start_planner_revalidation(all_candidates)
                return

            self.revalidation_signature = None
            self.reset_completion_verification("mapex_frontier_region_present")
            return

        self.revalidation_signature = None
        self.reset_completion_verification("planner_candidate_available")

        selected = min(candidates, key=lambda item: item[0])
        negative_score, x, y, row, col, region_size = selected
        selected_metric = metric_by_grid[(row, col)]

        self.publish_goal_markers(candidates, selected)

        decision_s = time.perf_counter() - decision_start
        self.get_logger().info(
            "MapEx decision %d: selected frontier x=%.2f, y=%.2f, "
            "IG=%.6f, distance=%.2f m, score=%.6f, region=%d, "
            "candidates=%d, compute=%.2f s",
            self.mapex_decision_id,
            x,
            y,
            selected_metric["information_gain"],
            selected_metric["distance_m"],
            -negative_score,
            region_size,
            len(candidates),
            decision_s,
        )

        self.publish_status(
            "MAPEX_SELECTED",
            reason="highest_information_gain_over_euclidean_distance",
            decision_id=self.mapex_decision_id,
            candidate_count=len(candidates),
            suppressed_no_path=suppressed_no_path,
            selected_row=int(row),
            selected_col=int(col),
            target_x=round(x, 4),
            target_y=round(y, 4),
            information_gain=selected_metric["information_gain"],
            distance_m=selected_metric["distance_m"],
            score=-negative_score,
            computation_s=round(decision_s, 4),
        )

        self.main_goal = (x, y)
        self.latest_main_plan = None
        self.recovery_count = 0
        self.send_navigation_goal(x, y, mode="main")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapExExplorer()
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
