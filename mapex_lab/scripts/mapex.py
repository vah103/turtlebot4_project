#!/usr/bin/env python3
"""MapEx exploration policy on the shared ``nf_basic.py`` execution layer.

The file keeps ROS2/Nav2 execution, exact-frontier goals, recovery, planner-
blocking suppression, terminal revalidation, completion, markers and status in
``NearestEuclideanFrontier``. Only the exploration policy is implemented here,
with the code structure intentionally following the MapEx block diagram:

    observed map
      -> three LaMa ensemble predictions
      -> ensemble mean + variance
      -> MapEx frontier extraction
      -> probabilistic visibility at each frontier
      -> IG = sum(variance on predicted-visible AND currently-unknown cells)
      -> Euclidean distance
      -> score = IG / distance
      -> choose argmax(score)

Pinned reference implementation:
    castacks/MapEx @ 53636bd1c79153acc3c74a532837d78c926bae5e

Reproduction choices:
- ``config/mapex.yaml`` is the source of truth for policy parameters.
- LaMa preprocessing uses the official MapEx ``lama_pred_utils`` helpers and
  ``default_map_eval`` transform rather than a locally recreated tensor path.
- Visibility follows the official boundary construction: probabilistic ray hit
  points -> Polygon -> buffer(1) -> Bresenham boundary -> 4-neighbour flood fill.
- The occupancy accumulator is initialized ONCE PER RAY, matching Sec. IV-C of
  the paper rather than reproducing the upstream accumulator-reset helper bug.
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
            self.source_names.append(str(model_dir / "models" / checkpoint_name))

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
            raise RuntimeError(f"MapEx ensemble directory not found: {ensemble_dir}")

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

    def predict_maps(self, observed_map: np.ndarray):
        """Run official preprocessing and return P1/P2/P3 plus padded O_t."""
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
        if prediction_stack.shape[0] != ENSEMBLE_SIZE:
            raise RuntimeError(
                f"Expected {ENSEMBLE_SIZE} LaMa predictions; "
                f"got shape {tuple(prediction_stack.shape)}"
            )
        return prediction_stack, padded_observed, pad_top, pad_left

    def compute_mean_map(self, predictions, padded_observed: np.ndarray) -> np.ndarray:
        """P_bar_t = mean(P1_t, P2_t, P3_t), preserving known observed cells."""
        mean_map = (
            self.torch.mean(predictions, dim=0)
            .float()
            .cpu()
            .numpy()
        )
        mean_map = np.nan_to_num(mean_map, nan=0.5, posinf=1.0, neginf=0.0)
        mean_map = np.clip(mean_map, 0.0, 1.0).astype(np.float32, copy=False)

        known = ~np.isclose(padded_observed, 0.5)
        mean_map[known] = padded_observed[known]
        return mean_map

    def compute_variance_map(self, predictions, padded_observed: np.ndarray) -> np.ndarray:
        """V_t = variance(P1_t, P2_t, P3_t), preserving current semantics."""
        variance_map = (
            self.torch.var(predictions, dim=0)
            .float()
            .cpu()
            .numpy()
        )
        variance_map = np.nan_to_num(
            variance_map,
            nan=0.0,
            posinf=0.0,
            neginf=0.0,
        )
        variance_map = np.maximum(variance_map, 0.0).astype(
            np.float32,
            copy=False,
        )

        known = ~np.isclose(padded_observed, 0.5)
        variance_map[known] = 0.0
        return variance_map


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
        self.ray_range_cells = int(
            round(self.mapex_sensor_range_m / self.mapex_resolution_m)
        )

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

    def get_current_map(self) -> np.ndarray:
        """Return the current ROS OccupancyGrid as a 2-D numpy array."""
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        return np.asarray(self.map_msg.data, dtype=np.int16).reshape(height, width)

    def get_robot_pose(self):
        """Block-diagram name for the inherited map-frame robot pose lookup."""
        return self.robot_position()

    def predict_maps(self, observed_map: np.ndarray):
        """P1_t, P2_t, P3_t = G1/G2/G3(O_t)."""
        return self.ensemble.predict_maps(observed_map)

    def compute_mean_map(
        self,
        predictions,
        padded_observed: np.ndarray,
    ) -> np.ndarray:
        """Compute the ensemble mean predicted map P_bar_t."""
        return self.ensemble.compute_mean_map(predictions, padded_observed)

    def compute_variance_map(
        self,
        predictions,
        padded_observed: np.ndarray,
    ) -> np.ndarray:
        """Compute the ensemble variance map V_t."""
        return self.ensemble.compute_variance_map(predictions, padded_observed)

    def detect_frontier_cells(self, grid: np.ndarray) -> np.ndarray:
        """Detect free cells adjacent to unknown using the MapEx 8-neighbour rule."""
        return self.frontier_mask(grid)

    @staticmethod
    def connected_components(frontier_mask: np.ndarray) -> list[list[tuple[int, int]]]:
        """Return all 8-connected frontier components without size filtering."""
        height, width = frontier_mask.shape
        visited = np.zeros_like(frontier_mask, dtype=bool)
        regions: list[list[tuple[int, int]]] = []

        for row in range(height):
            for col in range(width):
                if not frontier_mask[row, col] or visited[row, col]:
                    continue

                queue = deque([(row, col)])
                visited[row, col] = True
                region: list[tuple[int, int]] = []

                while queue:
                    r, c = queue.popleft()
                    region.append((r, c))

                    for dr in (-1, 0, 1):
                        for dc in (-1, 0, 1):
                            if dr == 0 and dc == 0:
                                continue
                            nr = r + dr
                            nc = c + dc
                            if (
                                0 <= nr < height
                                and 0 <= nc < width
                                and frontier_mask[nr, nc]
                                and not visited[nr, nc]
                            ):
                                visited[nr, nc] = True
                                queue.append((nr, nc))

                regions.append(region)

        return regions

    @staticmethod
    def filter_small_clusters(
        regions: list[list[tuple[int, int]]],
    ) -> list[list[tuple[int, int]]]:
        """Keep MapEx frontier regions whose size is strictly greater than 10."""
        return [region for region in regions if len(region) > MIN_REGION_SIZE]

    def compute_frontier_centroids(
        self,
        regions: list[list[tuple[int, int]]],
    ) -> list[dict]:
        """Create F_t using the frontier cell nearest each region arithmetic mean."""
        frontiers = []
        for candidate_id, region in enumerate(regions):
            row, col = self.representative(region)
            x, y = self.cell_to_world(row, col)
            frontiers.append(
                {
                    "candidate_id": candidate_id,
                    "row": int(row),
                    "col": int(col),
                    "x": float(x),
                    "y": float(y),
                    "region_size": int(len(region)),
                }
            )
        return frontiers

    def cast_ray(
        self,
        mean_map: np.ndarray,
        viewpoint: tuple[int, int],
        angle: float,
    ) -> tuple[int, int]:
        """Cast one probabilistic ray and return its endpoint."""
        height, width = mean_map.shape
        vr, vc = viewpoint
        end_r = int(vr + self.ray_range_cells * math.cos(angle))
        end_c = int(vc + self.ray_range_cells * math.sin(angle))

        delta = 0.0
        last_valid = (vr, vc)
        for row, col in _bresenham((vr, vc), (end_r, end_c)):
            if row < 0 or row >= height or col < 0 or col >= width:
                break
            last_valid = (row, col)
            delta += float(np.clip(mean_map[row, col], 0.0, 1.0))
            if delta >= self.mapex_epsilon:
                break
        return last_valid

    def collect_ray_endpoints(
        self,
        mean_map: np.ndarray,
        viewpoint: tuple[int, int],
    ) -> np.ndarray:
        """Cast the configured 360-degree ray set from one frontier."""
        hit_points = [
            self.cast_ray(mean_map, viewpoint, angle)
            for angle in np.linspace(
                0.0,
                2.0 * math.pi,
                self.mapex_num_rays,
            )
        ]
        return np.asarray(hit_points, dtype=np.int64)

    @staticmethod
    def build_visibility_mask(
        hit_points: np.ndarray,
        viewpoint: tuple[int, int],
        shape: tuple[int, int],
    ) -> np.ndarray:
        """Build the predicted sensor-coverage mask from ordered ray endpoints."""
        try:
            from shapely.geometry import MultiPolygon, Point, Polygon
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                "Shapely is required for the official MapEx visibility boundary"
            ) from exc

        vr, vc = viewpoint
        height, width = shape
        if not (0 <= vr < height and 0 <= vc < width):
            return np.zeros(shape, dtype=bool)
        if len(hit_points) < 3 or len(np.unique(hit_points, axis=0)) < 3:
            return np.zeros(shape, dtype=bool)

        polygon = Polygon(hit_points)
        if polygon.is_empty:
            return np.zeros(shape, dtype=bool)
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
        expanded = polygon.buffer(1)

        if isinstance(expanded, MultiPolygon):
            viewpoint_point = Point(vr, vc)
            containing = [
                geometry
                for geometry in expanded.geoms
                if geometry.contains(viewpoint_point)
                or geometry.touches(viewpoint_point)
            ]
            expanded = max(
                containing if containing else list(expanded.geoms),
                key=lambda geometry: geometry.area,
            )
        if expanded.is_empty or not hasattr(expanded, "exterior"):
            return np.zeros(shape, dtype=bool)

        expanded_boundary = np.asarray(expanded.exterior.coords).astype(np.int64)
        initialized = _init_buffered_boundary(shape, expanded_boundary)
        boundary_indices = np.argwhere(initialized == 0.0)

        seed = (vr, vc)
        if initialized[seed[0], seed[1]] == 0.0:
            interior = Polygon(hit_points).representative_point()
            seed = (int(interior.x), int(interior.y))

        if (
            not (0 <= seed[0] < height and 0 <= seed[1] < width)
            or initialized[seed[0], seed[1]] == 0.0
        ):
            candidates = np.argwhere(initialized == 0.5)
            if len(candidates) == 0:
                return np.zeros(shape, dtype=bool)
            seed = tuple(
                candidates[
                    np.argmin(
                        np.sum(
                            (
                                candidates
                                - np.asarray([vr, vc], dtype=np.int64)
                            )
                            ** 2,
                            axis=1,
                        )
                    )
                ]
            )

        flooded = _flood_fill_simple((int(seed[0]), int(seed[1])), initialized)
        if len(boundary_indices):
            flooded[boundary_indices[:, 0], boundary_indices[:, 1]] = 0.5
        return flooded == 0.0

    def compute_visibility(
        self,
        frontier: dict,
        mean_map: np.ndarray,
        padded_observed: np.ndarray,
        pad_top: int,
        pad_left: int,
    ) -> np.ndarray:
        """Return nu(f) = predicted-visible AND currently-unknown cells."""
        viewpoint = (
            int(frontier["row"]) + pad_top,
            int(frontier["col"]) + pad_left,
        )
        hit_points = self.collect_ray_endpoints(mean_map, viewpoint)
        sensor_coverage_mask = self.build_visibility_mask(
            hit_points,
            viewpoint,
            mean_map.shape,
        )
        currently_unknown = np.isclose(padded_observed, 0.5)
        return sensor_coverage_mask & currently_unknown

    @staticmethod
    def compute_information_gain(
        variance_map: np.ndarray,
        visibility_mask: np.ndarray,
    ) -> float:
        """I(f) = sum V_t(x,y) over cells in nu(f)."""
        return float(np.sum(variance_map[visibility_mask]))

    @staticmethod
    def compute_distance(robot_pose, frontier: dict) -> float:
        """d(f) = Euclidean distance from current robot pose to frontier."""
        robot_x, robot_y = robot_pose
        return math.hypot(
            float(frontier["x"]) - robot_x,
            float(frontier["y"]) - robot_y,
        )

    @staticmethod
    def compute_score(information_gain: float, distance_m: float) -> float:
        """S(f) = I(f) / d(f)."""
        return information_gain / max(distance_m, 1e-6)

    def evaluate_frontiers(
        self,
        frontiers: list[dict],
        mean_map: np.ndarray,
        variance_map: np.ndarray,
        padded_observed: np.ndarray,
        robot_pose,
        pad_top: int,
        pad_left: int,
        prediction_s: float,
    ) -> tuple[list[dict], float]:
        """Evaluate every f in F_t: visibility -> IG -> distance -> score."""
        evaluations = []
        scoring_start = time.perf_counter()

        for frontier in frontiers:
            visibility = self.compute_visibility(
                frontier,
                mean_map,
                padded_observed,
                pad_top,
                pad_left,
            )
            information_gain = self.compute_information_gain(
                variance_map,
                visibility,
            )
            distance_m = self.compute_distance(robot_pose, frontier)
            score = self.compute_score(information_gain, distance_m)

            evaluations.append(
                {
                    **frontier,
                    "distance_m": float(distance_m),
                    "information_gain": float(information_gain),
                    "score": float(score),
                    "visible_unknown_cells": int(np.count_nonzero(visibility)),
                }
            )

        scoring_s = time.perf_counter() - scoring_start
        for evaluation in evaluations:
            evaluation["ensemble_prediction_s"] = prediction_s
            evaluation["all_frontier_scoring_s"] = scoring_s
        return evaluations, scoring_s

    @staticmethod
    def select_best_frontier(evaluations: list[dict]) -> dict | None:
        """Psi = argmax_f S(f)."""
        if not evaluations:
            return None
        return max(evaluations, key=lambda candidate: candidate["score"])

    @staticmethod
    def to_execution_candidate(evaluation: dict) -> tuple:
        """Adapt MapEx dict data to the tuple interface expected by nf_basic."""
        return (
            -float(evaluation["score"]),
            float(evaluation["x"]),
            float(evaluation["y"]),
            int(evaluation["row"]),
            int(evaluation["col"]),
            int(evaluation["region_size"]),
        )

    def exploration_step(self):
        """Run one MapEx decision; execution remains inherited from nf_basic."""
        if self.completed:
            return
        if self.map_msg is None or self.goal_active or self.revalidation_active:
            return

        if self.main_goal is not None:
            self.reset_completion_verification("main_frontier_still_pending")
            self.revalidation_signature = None
            x, y = self.main_goal
            self.get_logger().info(
                f"Retrying MapEx main frontier after recovery: x={x:.2f}, y={y:.2f}"
            )
            self.send_navigation_goal(x, y, mode="main")
            return

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
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

        grid = self.get_current_map()
        observed_map = ros_occupancy_to_mapex(grid)
        self.mapex_decision_id += 1
        decision_start = time.perf_counter()

        try:
            prediction_start = time.perf_counter()
            (
                predictions,
                padded_observed,
                pad_top,
                pad_left,
            ) = self.predict_maps(observed_map)
            mean_map = self.compute_mean_map(predictions, padded_observed)
            variance_map = self.compute_variance_map(
                predictions,
                padded_observed,
            )
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

            frontier_cells = self.detect_frontier_cells(grid)
            regions = self.connected_components(frontier_cells)
            regions = self.filter_small_clusters(regions)

            if not regions:
                self.last_candidate_metrics = []
                self.clear_goal_markers()
                self.revalidation_signature = None
                self.observe_no_frontier_terminal_state()
                return

            frontiers = self.compute_frontier_centroids(regions)
            evaluations, _scoring_s = self.evaluate_frontiers(
                frontiers=frontiers,
                mean_map=mean_map,
                variance_map=variance_map,
                padded_observed=padded_observed,
                robot_pose=robot_pose,
                pad_top=pad_top,
                pad_left=pad_left,
                prediction_s=prediction_s,
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

        self.last_candidate_metrics = evaluations
        if not evaluations:
            self.clear_goal_markers()
            self.reset_completion_verification("mapex_no_scoreable_candidate")
            self.publish_status(
                "MAPEX_ERROR",
                reason="frontier_regions_exist_but_no_scoreable_candidate",
                decision_id=self.mapex_decision_id,
                frontier_regions=len(regions),
            )
            return

        selectable_evaluations = []
        suppressed_planner_blocked = 0
        for evaluation in evaluations:
            if self.is_planner_blocked_suppressed(
                evaluation["x"],
                evaluation["y"],
            ):
                suppressed_planner_blocked += 1
            else:
                selectable_evaluations.append(evaluation)

        all_execution_candidates = [
            self.to_execution_candidate(evaluation)
            for evaluation in evaluations
        ]

        if not selectable_evaluations:
            self.clear_goal_markers()
            if suppressed_planner_blocked == len(evaluations):
                self.maybe_start_planner_revalidation(all_execution_candidates)
                return
            self.revalidation_signature = None
            self.reset_completion_verification("mapex_frontier_region_present")
            return

        self.revalidation_signature = None
        self.reset_completion_verification("planner_candidate_available")

        selected_metric = self.select_best_frontier(selectable_evaluations)
        if selected_metric is None:
            return

        selectable_execution_candidates = [
            self.to_execution_candidate(evaluation)
            for evaluation in selectable_evaluations
        ]
        selected_execution_candidate = self.to_execution_candidate(selected_metric)
        self.publish_goal_markers(
            selectable_execution_candidates,
            selected_execution_candidate,
        )

        x = float(selected_metric["x"])
        y = float(selected_metric["y"])
        region_size = int(selected_metric["region_size"])
        decision_s = time.perf_counter() - decision_start

        self.get_logger().info(
            f"MapEx decision {self.mapex_decision_id}: selected x={x:.2f}, y={y:.2f}, "
            f"IG={selected_metric['information_gain']:.6f}, "
            f"distance={selected_metric['distance_m']:.2f} m, "
            f"score={selected_metric['score']:.6f}, region={region_size}, "
            f"candidates={len(selectable_evaluations)}, compute={decision_s:.2f} s"
        )
        self.publish_status(
            "MAPEX_SELECTED",
            reason="highest_information_gain_over_euclidean_distance",
            decision_id=self.mapex_decision_id,
            candidate_count=len(selectable_evaluations),
            suppressed_planner_blocked=suppressed_planner_blocked,
            selected_row=int(selected_metric["row"]),
            selected_col=int(selected_metric["col"]),
            target_x=round(x, 4),
            target_y=round(y, 4),
            information_gain=selected_metric["information_gain"],
            distance_m=selected_metric["distance_m"],
            score=selected_metric["score"],
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
