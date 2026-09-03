#!/usr/bin/env python3
"""Generate structural ground truth + connected-free ROI for new_room.sdf.

The artifact is rasterized directly from collision boxes in the
``mini_hospital_structure`` model. The fixed raster frame intentionally matches
``nf_run.py`` / ``mapex_run.py`` (hospital_canvas_v1) so recorded prediction
maps can be compared without another reprojection convention.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import deque
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

CANVAS_ID = "hospital_canvas_v1"
CANVAS_RES = 0.05
CANVAS_W = 1504
CANVAS_H = 2123
CANVAS_X = -25.6
CANVAS_Y = -60.1

GT_ID = "new_room_structural_gt_v1"
ROI_ID = "new_room_connected_free_v1"
STRUCTURE_MODEL = "mini_hospital_structure"
DEFAULT_Z_SLICE_M = 0.20
DEFAULT_SPAWN_X = 0.0
DEFAULT_SPAWN_Y = 0.0


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _pose6(element: ET.Element) -> tuple[float, float, float, float, float, float]:
    text = element.findtext("pose", default="0 0 0 0 0 0")
    values = [float(value) for value in text.split()]
    if len(values) < 6:
        values.extend([0.0] * (6 - len(values)))
    if len(values) != 6:
        raise ValueError(f"invalid SDF pose: {text!r}")
    return tuple(values)  # type: ignore[return-value]


def _compose_pose(
    parent: tuple[float, float, float, float, float, float],
    child: tuple[float, float, float, float, float, float],
) -> tuple[float, float, float, float, float, float]:
    px, py, pz, proll, ppitch, pyaw = parent
    cx, cy, cz, croll, cpitch, cyaw = child
    if any(abs(value) > 1e-9 for value in (proll, ppitch, croll, cpitch)):
        raise ValueError(
            "new_room GT generator currently supports planar collision poses only"
        )
    c = math.cos(pyaw)
    s = math.sin(pyaw)
    return (
        px + c * cx - s * cy,
        py + s * cx + c * cy,
        pz + cz,
        0.0,
        0.0,
        pyaw + cyaw,
    )


def _box_corners(
    x: float,
    y: float,
    yaw: float,
    sx: float,
    sy: float,
) -> np.ndarray:
    local = np.asarray(
        [
            [-sx / 2.0, -sy / 2.0],
            [-sx / 2.0, sy / 2.0],
            [sx / 2.0, -sy / 2.0],
            [sx / 2.0, sy / 2.0],
        ],
        dtype=np.float64,
    )
    c = math.cos(yaw)
    s = math.sin(yaw)
    rotation = np.asarray([[c, -s], [s, c]], dtype=np.float64)
    return local @ rotation.T + np.asarray([x, y], dtype=np.float64)


def _load_collision_boxes(
    sdf_path: Path,
    model_name: str,
    z_slice_m: float,
) -> list[dict]:
    root = ET.parse(sdf_path).getroot()
    model = next(
        (item for item in root.findall(".//model") if item.get("name") == model_name),
        None,
    )
    if model is None:
        raise ValueError(f"model {model_name!r} not found in {sdf_path}")

    model_pose = _pose6(model)
    boxes = []
    for link in model.findall("link"):
        link_pose = _compose_pose(model_pose, _pose6(link))
        for collision in link.findall("collision"):
            box = collision.find("geometry/box")
            if box is None:
                continue
            size_text = box.findtext("size")
            if not size_text:
                continue
            sx, sy, sz = [float(value) for value in size_text.split()]
            pose = _compose_pose(link_pose, _pose6(collision))
            x, y, z, _roll, _pitch, yaw = pose
            if not (z - sz / 2.0 - 1e-9 <= z_slice_m <= z + sz / 2.0 + 1e-9):
                continue
            corners = _box_corners(x, y, yaw, sx, sy)
            boxes.append(
                {
                    "name": collision.get("name", ""),
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "sx": sx,
                    "sy": sy,
                    "sz": sz,
                    "corners": corners,
                }
            )
    if not boxes:
        raise ValueError(
            f"no box collisions from {model_name!r} intersect z={z_slice_m:.3f} m"
        )
    return boxes


def _cell_center_x(cols: np.ndarray) -> np.ndarray:
    return CANVAS_X + (cols.astype(np.float64) + 0.5) * CANVAS_RES


def _cell_center_y(rows: np.ndarray) -> np.ndarray:
    return CANVAS_Y + (rows.astype(np.float64) + 0.5) * CANVAS_RES


def _rasterize_boxes(boxes: list[dict]) -> np.ndarray:
    occupied = np.zeros((CANVAS_H, CANVAS_W), dtype=bool)
    eps = 1e-10

    for box in boxes:
        corners = box["corners"]
        xmin, ymin = np.min(corners, axis=0)
        xmax, ymax = np.max(corners, axis=0)

        c0 = max(0, int(math.floor((xmin - CANVAS_X) / CANVAS_RES)) - 1)
        c1 = min(CANVAS_W - 1, int(math.floor((xmax - CANVAS_X) / CANVAS_RES)) + 1)
        r0 = max(0, int(math.floor((ymin - CANVAS_Y) / CANVAS_RES)) - 1)
        r1 = min(CANVAS_H - 1, int(math.floor((ymax - CANVAS_Y) / CANVAS_RES)) + 1)
        if c1 < c0 or r1 < r0:
            continue

        cols = np.arange(c0, c1 + 1, dtype=np.int32)
        rows = np.arange(r0, r1 + 1, dtype=np.int32)
        xx, yy = np.meshgrid(_cell_center_x(cols), _cell_center_y(rows))

        dx = xx - float(box["x"])
        dy = yy - float(box["y"])
        c = math.cos(float(box["yaw"]))
        s = math.sin(float(box["yaw"]))
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        inside = (
            (np.abs(local_x) <= float(box["sx"]) / 2.0 + eps)
            & (np.abs(local_y) <= float(box["sy"]) / 2.0 + eps)
        )
        occupied[r0 : r1 + 1, c0 : c1 + 1] |= inside

    return occupied


def _building_mask(boxes: list[dict]) -> tuple[np.ndarray, tuple[float, float, float, float]]:
    all_corners = np.concatenate([box["corners"] for box in boxes], axis=0)
    xmin = float(np.min(all_corners[:, 0]))
    xmax = float(np.max(all_corners[:, 0]))
    ymin = float(np.min(all_corners[:, 1]))
    ymax = float(np.max(all_corners[:, 1]))

    cols = np.arange(CANVAS_W, dtype=np.int32)
    rows = np.arange(CANVAS_H, dtype=np.int32)
    xs = _cell_center_x(cols)
    ys = _cell_center_y(rows)
    col_mask = (xs >= xmin) & (xs <= xmax)
    row_mask = (ys >= ymin) & (ys <= ymax)
    mask = row_mask[:, None] & col_mask[None, :]
    return mask, (xmin, xmax, ymin, ymax)


def _world_to_cell(x: float, y: float) -> tuple[int, int]:
    col = int(math.floor((x - CANVAS_X) / CANVAS_RES))
    row = int(math.floor((y - CANVAS_Y) / CANVAS_RES))
    return row, col


def _connected_free(
    free: np.ndarray,
    spawn_x: float,
    spawn_y: float,
) -> tuple[np.ndarray, tuple[int, int]]:
    start = _world_to_cell(spawn_x, spawn_y)
    row, col = start
    if not (0 <= row < free.shape[0] and 0 <= col < free.shape[1]):
        raise ValueError(f"spawn ({spawn_x}, {spawn_y}) is outside the fixed canvas")
    if not free[start]:
        raise ValueError(
            f"spawn ({spawn_x}, {spawn_y}) is not free in generated ground truth"
        )

    connected = np.zeros(free.shape, dtype=bool)
    connected[start] = True
    queue = deque([start])
    h, w = free.shape
    while queue:
        row, col = queue.popleft()
        for nr, nc in (
            (row - 1, col),
            (row + 1, col),
            (row, col - 1),
            (row, col + 1),
        ):
            if (
                0 <= nr < h
                and 0 <= nc < w
                and free[nr, nc]
                and not connected[nr, nc]
            ):
                connected[nr, nc] = True
                queue.append((nr, nc))
    return connected, start


def _write_preview(path: Path, data: np.ndarray) -> None:
    # PGM: occupied=black, free=white, outside/unscored=gray.
    image = np.full(data.shape, 205, dtype=np.uint8)
    image[data == 0] = 254
    image[data > 50] = 0
    with path.open("wb") as stream:
        stream.write(f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii"))
        stream.write(np.flipud(image).tobytes())


def generate(
    sdf_path: Path,
    output_dir: Path,
    z_slice_m: float,
    spawn_x: float,
    spawn_y: float,
) -> dict:
    boxes = _load_collision_boxes(sdf_path, STRUCTURE_MODEL, z_slice_m)
    occupied = _rasterize_boxes(boxes)
    evaluation_mask, bounds = _building_mask(boxes)
    occupied &= evaluation_mask

    data = np.full((CANVAS_H, CANVAS_W), -1, dtype=np.int16)
    data[evaluation_mask] = 0
    data[occupied] = 100

    free = evaluation_mask & ~occupied
    connected, start_cell = _connected_free(free, spawn_x, spawn_y)

    output_dir.mkdir(parents=True, exist_ok=True)
    gt_path = output_dir / f"{GT_ID}.npz"
    roi_path = output_dir / f"{ROI_ID}.npy"
    preview_path = output_dir / f"{GT_ID}.pgm"
    summary_path = output_dir / f"{GT_ID}_summary.json"

    source_sha = _sha256(sdf_path)
    np.savez_compressed(
        gt_path,
        data=data,
        evaluation_mask=evaluation_mask,
        resolution=np.float64(CANVAS_RES),
        width=np.int64(CANVAS_W),
        height=np.int64(CANVAS_H),
        origin_x=np.float64(CANVAS_X),
        origin_y=np.float64(CANVAS_Y),
        canvas_id=np.asarray(CANVAS_ID),
        ground_truth_id=np.asarray(GT_ID),
        source_model=np.asarray(STRUCTURE_MODEL),
        source_sdf=np.asarray(str(sdf_path)),
        source_sdf_sha256=np.asarray(source_sha),
        z_slice_m=np.float64(z_slice_m),
    )
    np.save(roi_path, connected)
    _write_preview(preview_path, data)

    xmin, xmax, ymin, ymax = bounds
    summary = {
        "ground_truth_id": GT_ID,
        "roi_id": ROI_ID,
        "canvas_id": CANVAS_ID,
        "source_sdf": str(sdf_path),
        "source_sdf_sha256": source_sha,
        "source_model": STRUCTURE_MODEL,
        "z_slice_m": z_slice_m,
        "spawn_world_xy": [spawn_x, spawn_y],
        "spawn_cell_row_col": [int(start_cell[0]), int(start_cell[1])],
        "collision_boxes_rasterized": len(boxes),
        "building_bounds_m": {
            "x_min": xmin,
            "x_max": xmax,
            "y_min": ymin,
            "y_max": ymax,
            "width": xmax - xmin,
            "height": ymax - ymin,
        },
        "canvas": {
            "resolution_m": CANVAS_RES,
            "width_cells": CANVAS_W,
            "height_cells": CANVAS_H,
            "origin_x": CANVAS_X,
            "origin_y": CANVAS_Y,
        },
        "evaluation_mask_cells": int(np.count_nonzero(evaluation_mask)),
        "occupied_cells": int(np.count_nonzero(occupied)),
        "free_cells": int(np.count_nonzero(free)),
        "connected_free_roi_cells": int(np.count_nonzero(connected)),
        "outputs": {
            "ground_truth": str(gt_path),
            "connected_free_roi": str(roi_path),
            "preview_pgm": str(preview_path),
        },
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--sdf",
        type=Path,
        default=root / "map" / "new_room.sdf",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=root / "ground_truth" / "new_room" / "generated",
    )
    parser.add_argument("--z-slice-m", type=float, default=DEFAULT_Z_SLICE_M)
    parser.add_argument("--spawn-x", type=float, default=DEFAULT_SPAWN_X)
    parser.add_argument("--spawn-y", type=float, default=DEFAULT_SPAWN_Y)
    args = parser.parse_args()

    summary = generate(
        sdf_path=args.sdf.expanduser().resolve(),
        output_dir=args.output_dir.expanduser().resolve(),
        z_slice_m=args.z_slice_m,
        spawn_x=args.spawn_x,
        spawn_y=args.spawn_y,
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
