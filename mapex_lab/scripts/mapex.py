#!/usr/bin/env python3
"""MapEx exploration policy on the shared ``nf_basic.py`` execution layer.

The file keeps ROS2/Nav2 execution, exact-frontier goals, recovery, planner-
blocking suppression, terminal revalidation, completion, markers and status in
``NearestEuclideanFrontier``.  Only the exploration policy is replaced by the
MapEx pipeline from Ho et al., ICRA 2025 / arXiv:2409.15590:

    observed map
      -> MapEx frontier representatives
      -> three LaMa ensemble predictions
      -> ensemble mean + variance
      -> probabilistic visibility at each frontier
      -> IG = sum(variance on predicted-visible AND currently-unknown cells)
      -> score = IG / Euclidean distance
      -> choose the highest-score selectable frontier

Pinned reference implementation:
    castacks/MapEx @ 53636bd1c79153acc3c74a532837d78c926bae5e

Reproduction choices:
- ``config/mapex.yaml`` is the source of truth for policy parameters.
- LaMa preprocessing uses the official MapEx ``lama_pred_utils`` helpers and
  ``default_map_eval`` transform rather than a locally recreated tensor path.
- Visibility follows the official boundary construction: probabilistic ray hit
  points -> Polygon -> buffer(1) -> Bresenham boundary -> 4-neighbour flood fill.
- The occupancy accumulator is intentionally initialized ONCE PER RAY, matching
  Sec. IV-C of the paper.  This avoids reproducing the upstream helper bug that
  resets the accumulator inside the per-pixel loop.
- Hospital does not apply the original implementation's <1 m waypoint rejection
  or the Nearest-only <0.5 m guard; Nav2 reachability handles close frontiers.
"""

from __future__ import annotations

from collections import deque
import math
import os
from pathlib import Path
import sys
import time
from typing import Iterable

import numpy as np
import rclpy

from nf_basic import MIN_REGION_SIZE, NearestEuclideanFrontier


MAPEX_REFERENCE_COMMIT = "53636bd1c79153acc3c74a532837d78c926bae5e"
ENSEMBLE_SIZE = 3
DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "mapex.yaml"
)


class _ParentStartupLoggerProxy:
    """Hide the one Nearest-specific startup line while parent logic initializes."""

    def __init__(self, logger):
        self._logger = logger

    def info(self, message, *args, **kwargs):
        if isinstance(message, str) and message.startswith(
            "Nearest Euclidean Frontier started"
        ):
            return None
        return self._logger.info(message, *args, **kwargs)

    def __getattr__(self, name):
        return getattr(self._logger, name)


def _load_yaml(path: Path) -> dict:
    try:
        import yaml
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("PyYAML is required to read config/mapex.yaml") from exc

    path = path.expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f"MapEx config not found: {path}")

    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise RuntimeError(f"Invalid MapEx YAML root in {path}")
    return config


def _required(mapping: dict, key: str, section: str):
    if key not in mapping:
        raise RuntimeError(f"Missing MapEx config key: {section}.{key}")
    return mapping[key]


def load_mapex_policy_config(path: Path) -> dict:
    """Load and validate the policy contract used by this implementation."""
    config = _load_yaml(path)
    if config.get("method") != "mapex":
        raise RuntimeError("config/mapex.yaml must declare method: mapex")

    prediction = _required(config, "prediction", "root")
    frontier = _required(config, "frontier", "root")
    visibility = _required(config, "visibility", "root")
    information_gain = _required(config, "information_gain", "root")
    selection = _required(config, "selection", "root")
    execution = _required(config, "execution", "root")

    if int(_required(prediction, "ensemble_members", "prediction")) != ENSEMBLE_SIZE:
        raise RuntimeError("MapEx requires exactly three LaMa ensemble members")
    if str(_required(prediction, "model", "prediction")) != "lama_ensemble":
        raise RuntimeError("MapEx prediction.model must be lama_ensemble")
    if str(_required(prediction, "transform_variant", "prediction")) != "default_map_eval":
        raise RuntimeError("MapEx must use the official default_map_eval transform")

    target_resolution = float(
        _required(prediction, "target_resolution_m", "prediction")
    )
    runtime_resolution = float(
        _required(frontier, "runtime_resolution_m", "frontier")
    )
    if not math.isclose(target_resolution, 0.10, abs_tol=1e-9):
        raise RuntimeError("Paper-faithful MapEx prediction grid must be 0.10 m/cell")
    if not math.isclose(runtime_resolution, target_resolution, abs_tol=1e-9):
        raise RuntimeError("MapEx prediction/frontier resolutions must match")

    region_threshold = int(
        _required(
            frontier,
            "min_frontier_size_cells_strictly_greater_than",
            "frontier",
        )
    )
    if region_threshold != MIN_REGION_SIZE:
        raise RuntimeError(
            "MapEx frontier threshold must match shared nf_basic semantics: "
            f"> {MIN_REGION_SIZE} cells"
        )
    if frontier.get("reject_below_distance_m") is not None:
        raise RuntimeError(
            "Hospital MapEx adaptation must keep close frontiers; "
            "frontier.reject_below_distance_m must be null"
        )

    if str(_required(visibility, "method", "visibility")) != "mapex_probabilistic_raycast":
        raise RuntimeError("MapEx visibility.method must be mapex_probabilistic_raycast")
    if str(_required(visibility, "boundary_method", "visibility")) != "buffered_polygon_flood_fill":
        raise RuntimeError("MapEx visibility boundary must use buffered polygon flood fill")

    if str(_required(information_gain, "source", "information_gain")) != "ensemble_variance":
        raise RuntimeError("MapEx IG source must be ensemble_variance")
    if str(_required(information_gain, "domain", "information_gain")) != "predicted_visible_and_currently_unknown":
        raise RuntimeError(
            "MapEx IG domain must be predicted_visible_and_currently_unknown"
        )

    if str(_required(selection, "score", "selection")) != "IG_over_distance":
        raise RuntimeError("MapEx selection score must be IG_over_distance")
    if str(_required(selection, "distance_metric", "selection")) != "euclidean":
        raise RuntimeError("MapEx distance metric must be Euclidean")
    if str(_required(selection, "objective", "selection")) != "maximize":
        raise RuntimeError("MapEx selection objective must be maximize")

    if str(_required(execution, "shared_controller", "execution")) != "nf_basic.py":
        raise RuntimeError("MapEx must share execution with nf_basic.py")
    if not bool(_required(execution, "exact_frontier_xy", "execution")):
        raise RuntimeError("Hospital MapEx must execute exact frontier x/y")

    return config


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
    """ROS OccupancyGrid labels -> MapEx labels: free=0, unknown=.5, occupied=1."""
    observed = np.ones(grid.shape, dtype=np.float32)
    observed[grid == 0] = 0.0
    observed[grid < 0] = 0.5
    observed[grid > 0] = 1.0
    return observed


def _paper_probabilistic_hit_points(
    mean_map: np.ndarray,
    viewpoint: tuple[int, int],
    ray_range_cells: int,
    num_rays: int,
    epsilon: float,
) -> np.ndarray:
    """Return ordered ray endpoints with Delta accumulated once along each ray."""
    height, width = mean_map.shape
    vr, vc = viewpoint
    hit_points: list[tuple[int, int]] = []

    # Official implementation uses np.linspace with the endpoint included.
    for angle in np.linspace(0.0, 2.0 * math.pi, num_rays):
        end_r = int(vr + ray_range_cells * math.cos(angle))
        end_c = int(vc + ray_range_cells * math.sin(angle))

        delta = 0.0
        last_valid = (vr, vc)
        for row, col in _bresenham((vr, vc), (end_r, end_c)):
            if row < 0 or row >= height or col < 0 or col >= width:
                break
            last_valid = (row, col)
            delta += float(np.clip(mean_map[row, col], 0.0, 1.0))
            if delta >= epsilon:
                break
        hit_points.append(last_valid)

    return np.asarray(hit_points, dtype=np.int64)


def _init_buffered_boundary(
    shape: tuple[int, int],
    boundary_points: np.ndarray,
) -> np.ndarray:
    """Match MapEx init_flood_fill: unknown=.5, Bresenham boundary=0."""
    boundary_grid = np.ones(shape, dtype=np.float32) * 0.5
    if len(boundary_points) == 0:
        return boundary_grid

    closed = np.vstack((boundary_points, boundary_points[0]))
    previous = closed[0]
    for point in closed[1:]:
        start = (int(previous[0]), int(previous[1]))
        end = (int(point[0]), int(point[1]))
        for row, col in _bresenham(start, end):
            if 0 <= row < shape[0] and 0 <= col < shape[1]:
                boundary_grid[row, col] = 0.0
        previous = point
    return boundary_grid


def _flood_fill_simple(
    seed: tuple[int, int],
    occupancy_map: np.ndarray,
) -> np.ndarray:
    """Four-neighbour flood fill matching MapEx simple_mask_utils."""
    flooded = occupancy_map.copy()
    height, width = flooded.shape
    row, col = seed
    if not (0 <= row < height and 0 <= col < width):
        return flooded

    fringe = deque([(row, col)])
    while fringe:
        row, col = fringe.pop()
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr = row + dr
            nc = col + dc
            if 0 <= nr < height and 0 <= nc < width and flooded[nr, nc] == 0.5:
                flooded[nr, nc] = 0.0
                fringe.append((nr, nc))
    return flooded


def probabilistic_visibility_mask(
    mean_map: np.ndarray,
    observed_map: np.ndarray,
    viewpoint: tuple[int, int],
    ray_range_cells: int,
    num_rays: int,
    epsilon: float,
) -> np.ndarray:
    """MapEx probabilistic visibility with official buffered-boundary flood fill."""
    try:
        from shapely.geometry import MultiPolygon, Point, Polygon
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(
            "Shapely is required for the official MapEx visibility boundary"
        ) from exc

    height, width = mean_map.shape
    vr, vc = viewpoint
    if not (0 <= vr < height and 0 <= vc < width):
        return np.zeros_like(mean_map, dtype=bool)

    hit_points = _paper_probabilistic_hit_points(
        mean_map,
        viewpoint,
        ray_range_cells,
        num_rays,
        epsilon,
    )
    if len(hit_points) < 3 or len(np.unique(hit_points, axis=0)) < 3:
        return np.zeros_like(mean_map, dtype=bool)

    # simple_mask_utils.get_vis_mask constructs Polygon(hit_points).buffer(1).
    polygon = Polygon(hit_points)
    if polygon.is_empty:
        return np.zeros_like(mean_map, dtype=bool)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    expanded = polygon.buffer(1)

    if isinstance(expanded, MultiPolygon):
        viewpoint_point = Point(vr, vc)
        containing = [
            geometry
            for geometry in expanded.geoms
            if geometry.contains(viewpoint_point) or geometry.touches(viewpoint_point)
        ]
        expanded = max(
            containing if containing else list(expanded.geoms),
            key=lambda geometry: geometry.area,
        )
    if expanded.is_empty or not hasattr(expanded, "exterior"):
        return np.zeros_like(mean_map, dtype=bool)

    expanded_boundary = np.asarray(expanded.exterior.coords).astype(np.int64)
    initialized = _init_buffered_boundary(mean_map.shape, expanded_boundary)
    boundary_indices = np.argwhere(initialized == 0.0)

    seed = (vr, vc)
    if initialized[seed[0], seed[1]] == 0.0:
        interior = Polygon(hit_points).representative_point()
        seed = (int(interior.x), int(interior.y))

    if (
        not (0 <= seed[0] < height and 0 <= seed[1] < width)
        or initialized[seed[0], seed[1]] == 0.0
    ):
        # Robust fallback for a degenerate discretized boundary.
        candidates = np.argwhere(initialized == 0.5)
        if len(candidates) == 0:
            return np.zeros_like(mean_map, dtype=bool)
        seed = tuple(
            candidates[
                np.argmin(
                    np.sum(
                        (candidates - np.asarray([vr, vc], dtype=np.int64)) ** 2,
                        axis=1,
                    )
                )
            ]
        )

    flooded = _flood_fill_simple((int(seed[0]), int(seed[1])), initialized)
    if len(boundary_indices):
        flooded[boundary_indices[:, 0], boundary_indices[:, 1]] = 0.5

    predicted_visible = flooded == 0.0
    currently_unknown = np.isclose(observed_map, 0.5)
    return predicted_visible & currently_unknown


class LamaEnsemble:
    """Official MapEx LaMa preprocessing + three Gi ensemble members."""

    def __init__(
        self,
        mapex_root: Path,
        device: str,
        prediction_config: dict,
    ) -> None:
        self.mapex_root = mapex_root.expanduser().resolve()
        self.device = device
        self.prediction_config = prediction_config

        lama_root = self.mapex_root / "lama"
        scripts_root = self.mapex_root / "scripts"
        if not lama_root.exists():
            raise RuntimeError(
                f"MapEx LaMa submodule not found: {lama_root}. "
                "Clone castacks/MapEx with submodules initialized."
            )
        if not scripts_root.is_dir():
            raise RuntimeError(f"MapEx scripts directory not found: {scripts_root}")

        for import_path in (str(lama_root), str(scripts_root)):
            if import_path not in sys.path:
                sys.path.insert(0, import_path)

        try:
            import torch
            from lama_pred_utils import (
                convert_obsimg_to_model_input,
                get_lama_transform,
                load_lama_model,
            )
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "Cannot import official MapEx/LaMa preprocessing runtime. "
                "Activate the MapEx LaMa environment before launching this node."
            ) from exc

        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                f"Requested MapEx device '{device}', but CUDA is unavailable."
            )

        self.torch = torch
        self.convert_obsimg_to_model_input = convert_obsimg_to_model_input
        transform_variant = str(prediction_config["transform_variant"])
        # default_map_eval ignores out_size, but the official API expects it.
        self.map_transform = get_lama_transform(transform_variant, (512, 512))
        self.load_lama_model = load_lama_model

        model_specs = self._resolve_model_specs()
        self.models = []
        self.source_names = []
        for model_dir, checkpoint_name in model_specs:
            model = self.load_lama_model(
                str(model_dir),
                checkpoint_name=checkpoint_name,
                device=self.device,
            )
            model.eval()
            self.models.append(model)
            self.source_names.append(
                str(model_dir / "models" / checkpoint_name)
            )

    def _resolve_model_specs(self) -> list[tuple[Path, str]]:
        checkpoint_values = [
            self.prediction_config.get("checkpoint_g1"),
            self.prediction_config.get("checkpoint_g2"),
            self.prediction_config.get("checkpoint_g3"),
        ]

        if any(value is not None for value in checkpoint_values):
            if not all(value is not None for value in checkpoint_values):
                raise RuntimeError(
                    "Set all checkpoint_g1/g2/g3 values or leave all three null"
                )
            return [
                self._resolve_checkpoint_spec(str(value))
                for value in checkpoint_values
            ]

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
                f"MapEx ensemble directory not found: {ensemble_dir}"
            )

        member_dirs = sorted(path for path in ensemble_dir.iterdir() if path.is_dir())
        if len(member_dirs) != ENSEMBLE_SIZE:
            raise RuntimeError(
                f"Expected exactly {ENSEMBLE_SIZE} MapEx ensemble directories under "
                f"{ensemble_dir}; found {len(member_dirs)}"
            )
        return [(member_dir, "best.ckpt") for member_dir in member_dirs]

    def _resolve_checkpoint_spec(self, value: str) -> tuple[Path, str]:
        path = Path(value).expanduser()
        if not path.is_absolute():
            path = self.mapex_root / path
        path = path.resolve()

        if path.is_dir():
            model_dir = path
            checkpoint_name = "best.ckpt"
        elif path.is_file():
            if path.parent.name != "models":
                raise RuntimeError(
                    f"Configured MapEx checkpoint must live in <model>/models/: {path}"
                )
            model_dir = path.parent.parent
            checkpoint_name = path.name
        else:
            raise RuntimeError(f"Configured MapEx checkpoint not found: {path}")

        if not (model_dir / "config.yaml").is_file():
            raise RuntimeError(f"LaMa config.yaml missing under {model_dir}")
        if not (model_dir / "models" / checkpoint_name).is_file():
            raise RuntimeError(
                f"LaMa checkpoint missing: {model_dir / 'models' / checkpoint_name}"
            )
        return model_dir, checkpoint_name

    def predict_mean_variance(
        self,
        observed_map: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, int, int]:
        """Run official preprocessing, then compute MapEx ensemble mean/variance."""
        torch = self.torch
        observed_3channel = np.stack(
            [observed_map, observed_map, observed_map],
            axis=2,
        )
        input_batch, _mask = self.convert_obsimg_to_model_input(
            observed_3channel,
            self.map_transform,
            self.device,
        )

        padded_observed = (
            input_batch["image"][0, 0]
            .detach()
            .float()
            .cpu()
            .numpy()
            .astype(np.float32, copy=False)
        )
        output_h, output_w = padded_observed.shape
        input_h, input_w = observed_map.shape
        if output_h < input_h or output_w < input_w:
            raise RuntimeError(
                "Official default_map_eval unexpectedly shrank the occupancy map"
            )
        pad_top = (output_h - input_h) // 2
        pad_left = (output_w - input_w) // 2

        predictions = []
        with torch.no_grad():
            for model in self.models:
                # Keep the official batch semantics, but clone tensors so one model
                # cannot leave fields that influence the next ensemble member.
                batch = {
                    key: (value.clone() if torch.is_tensor(value) else value)
                    for key, value in input_batch.items()
                }
                output = model(batch)
                if not isinstance(output, dict) or "inpainted" not in output:
                    raise RuntimeError(
                        "Official LaMa model output does not contain 'inpainted'"
                    )
                predictions.append(output["inpainted"][0, 0].detach())

        prediction_stack = torch.stack(predictions, dim=0)
        mean_map = torch.mean(prediction_stack, dim=0).float().cpu().numpy()
        variance_map = torch.var(prediction_stack, dim=0).float().cpu().numpy()

        mean_map = np.nan_to_num(mean_map, nan=0.5, posinf=1.0, neginf=0.0)
        variance_map = np.nan_to_num(
            variance_map,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        mean_map = np.clip(mean_map, 0.0, 1.0).astype(np.float32, copy=False)
        variance_map = np.maximum(variance_map, 0.0).astype(
            np.float32,
            copy=False,
        )

        # Inpainting output should already preserve known cells. Enforce the same
        # semantics explicitly so numerical noise cannot create known-cell IG.
        known = ~np.isclose(padded_observed, 0.5)
        mean_map[known] = padded_observed[known]
        variance_map[known] = 0.0

        return mean_map, variance_map, padded_observed, pad_top, pad_left


class MapExExplorer(NearestEuclideanFrontier):
    """MapEx policy with shared nf_basic navigation/recovery/completion."""

    def get_logger(self):
        logger = super().get_logger()
        if getattr(self, "_suppress_parent_nearest_startup_log", False):
            return _ParentStartupLoggerProxy(logger)
        return logger

    def __init__(self) -> None:
        self._suppress_parent_nearest_startup_log = True
        super().__init__()
        self._suppress_parent_nearest_startup_log = False

        self.declare_parameter("mapex_config", str(DEFAULT_CONFIG_PATH))
        self.declare_parameter(
            "mapex_root",
            os.environ.get("MAPEX_ROOT", str(Path.home() / "MapEx")),
        )
        self.declare_parameter(
            "mapex_device",
            os.environ.get("MAPEX_DEVICE", "cuda:0"),
        )

        self.config_path = Path(str(self.get_parameter("mapex_config").value))
        self.config = load_mapex_policy_config(self.config_path)
        self.mapex_root = Path(str(self.get_parameter("mapex_root").value))
        self.mapex_device = str(self.get_parameter("mapex_device").value)

        prediction = self.config["prediction"]
        visibility = self.config["visibility"]
        self.mapex_resolution_m = float(prediction["target_resolution_m"])
        self.mapex_sensor_range_m = float(visibility["ray_length_m"])
        self.mapex_num_rays = int(visibility["num_rays"])
        self.mapex_epsilon = float(visibility["occupancy_threshold"])

        self.get_logger().info("Loading official MapEx LaMa ensemble...")
        self.ensemble = LamaEnsemble(
            self.mapex_root,
            self.mapex_device,
            prediction,
        )

        self.mapex_decision_id = 0
        self.last_mean_map: np.ndarray | None = None
        self.last_variance_map: np.ndarray | None = None
        self.last_candidate_metrics: list[dict] = []

        self.get_logger().info(
            "MapEx Explorer started "
            f"(config={self.config_path.resolve()}, "
            f"commit={MAPEX_REFERENCE_COMMIT}, "
            f"grid={self.mapex_resolution_m:.2f} m, "
            f"visibility={self.mapex_sensor_range_m:.1f} m/"
            f"{self.mapex_num_rays} rays, epsilon={self.mapex_epsilon:.2f}, "
            "shared execution=nf_basic)"
        )
        for index, source in enumerate(self.ensemble.source_names, start=1):
            self.get_logger().info(f"MapEx G{index}: {source}")

    def _score_mapex_candidates(
        self,
        grid: np.ndarray,
        regions,
        robot_x: float,
        robot_y: float,
    ) -> tuple[list[tuple], list[dict]]:
        observed = ros_occupancy_to_mapex(grid)

        prediction_start = time.perf_counter()
        (
            mean_map,
            variance_map,
            padded_observed,
            pad_top,
            pad_left,
        ) = self.ensemble.predict_mean_variance(observed)
        prediction_s = time.perf_counter() - prediction_start

        if (
            mean_map.shape != padded_observed.shape
            or variance_map.shape != padded_observed.shape
        ):
            raise RuntimeError(
                f"LaMa shape mismatch: obs={padded_observed.shape}, "
                f"mean={mean_map.shape}, variance={variance_map.shape}"
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

            visibility = probabilistic_visibility_mask(
                mean_map,
                padded_observed,
                (row + pad_top, col + pad_left),
                ray_range_cells,
                self.mapex_num_rays,
                self.mapex_epsilon,
            )
            information_gain = float(np.sum(variance_map[visibility]))
            score = information_gain / max(distance_m, 1e-6)

            # Parent helpers minimize tuple[0].  -score preserves that interface
            # while MapEx itself maximizes IG / distance.
            candidates.append((-score, x, y, row, col, len(region)))
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
        """Run one MapEx decision; goal execution remains inherited from nf_basic."""
        if self.completed:
            return
        if self.map_msg is None or self.goal_active or self.revalidation_active:
            return

        # Preserve the exact same selected-frontier lock/recovery semantics.
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
                f"MapEx config requires /map at {self.mapex_resolution_m:.2f} m/cell; "
                f"got {resolution:.6f} m/cell. Not issuing a goal."
            )
            return

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        grid = np.asarray(self.map_msg.data, dtype=np.int16).reshape(height, width)

        # These inherited routines match the pinned MapEx frontier implementation:
        # free==0, unknown<0, 8-neighbour frontier, 8-connected clusters, size>10,
        # representative = actual frontier cell nearest the arithmetic mean.
        regions = self.frontier_regions(self.frontier_mask(grid))
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
            self.get_logger().error(
                f"MapEx decision {self.mapex_decision_id} failed: {exc}"
            )
            return

        self.last_candidate_metrics = metrics
        if not all_candidates:
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_no_scoreable_candidate")
            self.publish_status(
                "MAPEX_ERROR",
                reason="frontier_regions_exist_but_no_scoreable_candidate",
                decision_id=self.mapex_decision_id,
                frontier_regions=len(regions),
            )
            return

        metric_by_grid = {(m["row"], m["col"]): m for m in metrics}
        candidates = []
        suppressed_planner_blocked = 0
        for candidate in all_candidates:
            _negative_score, x, y, _row, _col, _region_size = candidate
            if self.is_planner_blocked_suppressed(x, y):
                suppressed_planner_blocked += 1
            else:
                candidates.append(candidate)

        if not candidates:
            self.clear_goal_markers()
            if suppressed_planner_blocked == len(all_candidates):
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
            f"MapEx decision {self.mapex_decision_id}: selected x={x:.2f}, y={y:.2f}, "
            f"IG={selected_metric['information_gain']:.6f}, "
            f"distance={selected_metric['distance_m']:.2f} m, "
            f"score={-negative_score:.6f}, region={region_size}, "
            f"candidates={len(candidates)}, compute={decision_s:.2f} s"
        )
        self.publish_status(
            "MAPEX_SELECTED",
            reason="highest_information_gain_over_euclidean_distance",
            decision_id=self.mapex_decision_id,
            candidate_count=len(candidates),
            suppressed_planner_blocked=suppressed_planner_blocked,
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


def _mapex_init_args(args):
    """Force a MapEx-specific ROS node name without changing nf_basic defaults."""
    effective_args = list(sys.argv if args is None else args)
    effective_args.extend(["--ros-args", "-r", "__node:=mapex_explorer"])
    return effective_args


def main(args=None):
    rclpy.init(args=_mapex_init_args(args))
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
