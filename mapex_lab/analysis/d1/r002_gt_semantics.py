#!/usr/bin/env python3
"""R002 frozen V2 evaluator for the one-run mpx_001 feasibility check.

Implements the USER-approved R002 references without changing Gate P2:
A: existing structural-solid Gate-P2 semantics (reused from d1_gate_p.py)
B: occupancy-surface boundary diagnostic with unknown-only prediction provenance
C0: point-connected remaining-free diagnostic on the canonical 0.05 m canvas

First implementation is intentionally restricted to mpx_001.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Iterable

import numpy as np
from scipy import ndimage
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

import d1_gate_p as gate_p


PREDICTION_THRESHOLD = 0.5
GT_OCC_THRESHOLD = gate_p.GT_OCC_THRESHOLD
PRIMARY_BOUNDARY_TOLERANCE_M = 0.10
SENSITIVITY_BOUNDARY_TOLERANCE_M = 0.05
EXACT_BOUNDARY_TOLERANCE_M = 1e-12
SEED_FALLBACK_RADIUS_M = 0.10
ALLOWED_RUN = "mpx_001"
LAST_N = 10
STAGE_TARGETS = {
    "early": 1.0 / 6.0,
    "mid": 1.0 / 2.0,
    "late": 5.0 / 6.0,
}
FOUR_CONNECTIVITY = np.asarray(
    [[0, 1, 0], [1, 1, 1], [0, 1, 0]],
    dtype=bool,
)
EIGHT_CONNECTIVITY = np.ones((3, 3), dtype=bool)


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        raise ValueError(f"cannot write empty CSV: {path}")
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _finite_mean(values: Iterable[float]) -> float:
    arr = np.asarray([float(v) for v in values], dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    return float(np.mean(arr)) if arr.size else math.nan


def _finite_std(values: Iterable[float]) -> float:
    arr = np.asarray([float(v) for v in values], dtype=np.float64)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return math.nan
    if arr.size == 1:
        return 0.0
    return float(np.std(arr, ddof=1))


def _safe_div(numer: int | float, denom: int | float) -> float:
    return float(numer / denom) if denom else math.nan


def _stage(progress: float) -> str:
    if progress < 1.0 / 3.0:
        return "early"
    if progress < 2.0 / 3.0:
        return "mid"
    return "late"


def _resolve_run_path(run_dir: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else run_dir / path


def _canonical_projection(
    observed: gate_p.RawGrid,
    gt: gate_p.StructuralGT,
) -> dict:
    """Project one exact runtime OccupancyGrid onto the Gate-P canonical canvas.

    This deliberately calls Gate P's frozen structural context and verifies that
    U_t exactly reproduces its unknown/evaluation-mask footprint.
    """
    context = gate_p._structural_context(observed, gt)
    ratio = int(context["ratio"])

    expanded = np.repeat(
        np.repeat(observed.data, ratio, axis=0),
        ratio,
        axis=1,
    )

    support = np.zeros(gt.shape, dtype=bool)
    canonical_observed = np.full(gt.shape, -1, dtype=np.int16)

    src_slice = context["src_slice"]
    gt_slice = context["gt_slice"]
    if expanded[src_slice].size:
        support[gt_slice] = True
        canonical_observed[gt_slice] = expanded[src_slice]

    unknown = support & (canonical_observed < 0)
    observed_free = support & (canonical_observed >= 0) & (
        canonical_observed <= GT_OCC_THRESHOLD
    )
    observed_occupied = support & (canonical_observed > GT_OCC_THRESHOLD)
    universe = unknown & gt.evaluation_mask

    gate_p_universe = np.zeros(gt.shape, dtype=bool)
    if context["valid"].size:
        gate_p_universe[gt_slice] = context["valid"]

    if not np.array_equal(universe, gate_p_universe):
        raise AssertionError(
            "DecisionMapSupport/U_t projection does not reproduce Gate-P structural footprint"
        )

    return {
        "context": context,
        "ratio": ratio,
        "support": support,
        "canonical_observed": canonical_observed,
        "unknown": unknown,
        "observed_free": observed_free,
        "observed_occupied": observed_occupied,
        "universe": universe,
    }


def _canonical_prediction(
    runtime_prediction: np.ndarray,
    projection: dict,
    gt: gate_p.StructuralGT,
) -> np.ndarray:
    ratio = int(projection["ratio"])
    expanded = np.repeat(
        np.repeat(runtime_prediction, ratio, axis=0),
        ratio,
        axis=1,
    )
    result = np.full(gt.shape, np.nan, dtype=np.float32)
    src_slice = projection["context"]["src_slice"]
    gt_slice = projection["context"]["gt_slice"]
    if expanded[src_slice].size:
        result[gt_slice] = expanded[src_slice]
    return result


def _adjacent8(mask: np.ndarray) -> np.ndarray:
    return ndimage.binary_dilation(mask, structure=EIGHT_CONNECTIVITY)


def _occupied_boundary(occupied: np.ndarray, free: np.ndarray) -> np.ndarray:
    return occupied & _adjacent8(free)


def _coords(mask: np.ndarray) -> np.ndarray:
    return np.argwhere(mask).astype(np.int32, copy=False)


def _boundary_matching(
    gt_coords: np.ndarray,
    pred_coords: np.ndarray,
    resolution_m: float,
    tolerance_m: float,
) -> dict:
    """Deterministic maximum-cardinality, minimum-distance one-to-one matching.

    Eligible graph is decomposed into connected bipartite components. Each
    component is solved with an assignment containing explicit unmatched
    dummies. GT/pred coordinates are sorted lexicographically before assignment;
    a tiny rank perturbation is used only to make exact equal-distance optima
    deterministic in lexicographic pair order.
    """
    gt_coords = np.asarray(gt_coords, dtype=np.int32).reshape((-1, 2))
    pred_coords = np.asarray(pred_coords, dtype=np.int32).reshape((-1, 2))

    n_gt = int(gt_coords.shape[0])
    n_pred = int(pred_coords.shape[0])
    if n_gt == 0 or n_pred == 0:
        matched = 0
        precision = 0.0 if n_pred else math.nan
        recall = 0.0 if n_gt else math.nan
        f1 = math.nan if n_gt == 0 and n_pred == 0 else 0.0
        return {
            "matched": matched,
            "gt_count": n_gt,
            "pred_count": n_pred,
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "total_distance_m": 0.0,
            "pairs": [],
        }

    gt_order = np.lexsort((gt_coords[:, 1], gt_coords[:, 0]))
    pred_order = np.lexsort((pred_coords[:, 1], pred_coords[:, 0]))
    gt_sorted = gt_coords[gt_order]
    pred_sorted = pred_coords[pred_order]

    gt_xy = gt_sorted[:, [1, 0]].astype(np.float64) * float(resolution_m)
    pred_xy = pred_sorted[:, [1, 0]].astype(np.float64) * float(resolution_m)

    tree = cKDTree(pred_xy)
    neighborhoods = tree.query_ball_point(
        gt_xy,
        r=float(tolerance_m) + 1e-12,
    )

    edges: list[tuple[int, int, float]] = []
    gt_adj: dict[int, list[int]] = defaultdict(list)
    pred_adj: dict[int, list[int]] = defaultdict(list)

    for gi, candidates in enumerate(neighborhoods):
        for pj in sorted(candidates):
            distance = float(np.linalg.norm(gt_xy[gi] - pred_xy[pj]))
            if distance <= float(tolerance_m) + 1e-12:
                edges.append((gi, pj, distance))
                gt_adj[gi].append(pj)
                pred_adj[pj].append(gi)

    if not edges:
        return {
            "matched": 0,
            "gt_count": n_gt,
            "pred_count": n_pred,
            "precision": 0.0,
            "recall": 0.0,
            "f1": 0.0,
            "total_distance_m": 0.0,
            "pairs": [],
        }

    edge_distance = {(gi, pj): d for gi, pj, d in edges}
    unvisited_gt = set(gt_adj)
    components: list[tuple[list[int], list[int]]] = []

    while unvisited_gt:
        start = min(unvisited_gt)
        stack: list[tuple[str, int]] = [("g", start)]
        comp_g: set[int] = set()
        comp_p: set[int] = set()
        while stack:
            side, idx = stack.pop()
            if side == "g":
                if idx in comp_g:
                    continue
                comp_g.add(idx)
                unvisited_gt.discard(idx)
                for pj in gt_adj.get(idx, []):
                    if pj not in comp_p:
                        stack.append(("p", pj))
            else:
                if idx in comp_p:
                    continue
                comp_p.add(idx)
                for gi in pred_adj.get(idx, []):
                    if gi not in comp_g:
                        stack.append(("g", gi))
        components.append((sorted(comp_g), sorted(comp_p)))

    matched_pairs: list[tuple[int, int, float]] = []

    for comp_g, comp_p in components:
        g = len(comp_g)
        p = len(comp_p)
        size = g + p
        unmatched_penalty = 10.0
        forbidden = 1e9
        cost = np.full((size, size), forbidden, dtype=np.float64)

        g_local = {
            global_i: local_i
            for local_i, global_i in enumerate(comp_g)
        }
        p_local = {
            global_i: local_i
            for local_i, global_i in enumerate(comp_p)
        }

        eligible_pairs = sorted(
            (
                (gi, pj, edge_distance[(gi, pj)])
                for gi in comp_g
                for pj in gt_adj[gi]
                if pj in p_local
            ),
            key=lambda item: (
                int(gt_sorted[item[0], 0]),
                int(gt_sorted[item[0], 1]),
                int(pred_sorted[item[1], 0]),
                int(pred_sorted[item[1], 1]),
            ),
        )
        pair_count = max(len(eligible_pairs), 1)
        tie_unit = 1e-14 / float(pair_count * pair_count + 1)
        for rank, (gi, pj, distance) in enumerate(eligible_pairs):
            cost[g_local[gi], p_local[pj]] = distance + tie_unit * rank

        cost[:g, p:] = unmatched_penalty
        cost[g:, :p] = unmatched_penalty
        cost[g:, p:] = 0.0

        row_ind, col_ind = linear_sum_assignment(cost)
        for rr, cc in zip(row_ind.tolist(), col_ind.tolist()):
            if rr < g and cc < p:
                gi = comp_g[rr]
                pj = comp_p[cc]
                key = (gi, pj)
                if key not in edge_distance:
                    raise AssertionError(
                        "assignment selected an ineligible boundary pair"
                    )
                matched_pairs.append((gi, pj, edge_distance[key]))

    matched_pairs.sort(
        key=lambda item: (
            int(gt_sorted[item[0], 0]),
            int(gt_sorted[item[0], 1]),
            int(pred_sorted[item[1], 0]),
            int(pred_sorted[item[1], 1]),
        )
    )

    matched = len(matched_pairs)
    precision = matched / n_pred
    recall = matched / n_gt
    f1 = (
        2.0 * precision * recall / (precision + recall)
        if (precision + recall) > 0.0
        else 0.0
    )
    pairs = [
        {
            "gt_row": int(gt_sorted[gi, 0]),
            "gt_col": int(gt_sorted[gi, 1]),
            "pred_row": int(pred_sorted[pj, 0]),
            "pred_col": int(pred_sorted[pj, 1]),
            "distance_m": float(distance),
        }
        for gi, pj, distance in matched_pairs
    ]
    return {
        "matched": matched,
        "gt_count": n_gt,
        "pred_count": n_pred,
        "precision": float(precision),
        "recall": float(recall),
        "f1": float(f1),
        "total_distance_m": float(
            sum(item[2] for item in matched_pairs)
        ),
        "pairs": pairs,
    }


def _project_robot_cell(
    robot_x: float,
    robot_y: float,
    gt: gate_p.StructuralGT,
) -> tuple[int, int]:
    col = int(math.floor((robot_x - gt.origin_x) / gt.resolution))
    row = int(math.floor((robot_y - gt.origin_y) / gt.resolution))
    return row, col


def _cell_center(
    row: int,
    col: int,
    gt: gate_p.StructuralGT,
) -> tuple[float, float]:
    return (
        gt.origin_x + (col + 0.5) * gt.resolution,
        gt.origin_y + (row + 0.5) * gt.resolution,
    )


def _select_seed(
    robot_x: float,
    robot_y: float,
    observed_free: np.ndarray,
    topology_domain: np.ndarray,
    gt: gate_p.StructuralGT,
) -> tuple[tuple[int, int] | None, str]:
    row, col = _project_robot_cell(robot_x, robot_y, gt)
    h, w = gt.shape
    if (
        0 <= row < h
        and 0 <= col < w
        and topology_domain[row, col]
        and observed_free[row, col]
    ):
        return (row, col), "exact"

    radius_cells = int(
        math.ceil(SEED_FALLBACK_RADIUS_M / gt.resolution)
    )
    candidates: list[tuple[float, int, int]] = []
    for rr in range(
        max(0, row - radius_cells - 1),
        min(h, row + radius_cells + 2),
    ):
        for cc in range(
            max(0, col - radius_cells - 1),
            min(w, col + radius_cells + 2),
        ):
            if not topology_domain[rr, cc] or not observed_free[rr, cc]:
                continue
            cx, cy = _cell_center(rr, cc, gt)
            distance = math.hypot(cx - robot_x, cy - robot_y)
            if distance <= SEED_FALLBACK_RADIUS_M + 1e-12:
                candidates.append((distance, rr, cc))

    if not candidates:
        return None, "invalid"

    candidates.sort(key=lambda item: (item[0], item[1], item[2]))
    _, rr, cc = candidates[0]
    return (rr, cc), "fallback"


def _classification_positive_metrics(
    gt_positive: np.ndarray,
    pred_positive: np.ndarray,
) -> dict:
    gt_positive = np.asarray(gt_positive, dtype=bool)
    pred_positive = np.asarray(pred_positive, dtype=bool)
    if gt_positive.shape != pred_positive.shape:
        raise ValueError("C0 positive masks must have identical shapes")

    gt_n = int(np.count_nonzero(gt_positive))
    pred_n = int(np.count_nonzero(pred_positive))
    tp = int(np.count_nonzero(gt_positive & pred_positive))
    union = int(np.count_nonzero(gt_positive | pred_positive))

    if gt_n == 0 and pred_n == 0:
        precision = recall = iou = math.nan
    elif gt_n == 0:
        precision = 0.0
        recall = math.nan
        iou = 0.0
    elif pred_n == 0:
        precision = math.nan
        recall = 0.0
        iou = 0.0
    else:
        precision = tp / pred_n
        recall = tp / gt_n
        iou = tp / union if union else math.nan

    return {
        "gt_positive_count": gt_n,
        "pred_positive_count": pred_n,
        "true_positive_count": tp,
        "precision": float(precision),
        "recall": float(recall),
        "iou": float(iou),
    }


def _pre_registered_visual_indices(total: int) -> dict[str, int]:
    if total <= 0:
        return {}
    result = {}
    for name, target in STAGE_TARGETS.items():
        best = min(
            range(1, total + 1),
            key=lambda index: (
                abs(index / total - target),
                index,
            ),
        )
        result[name] = best
    return result


def _plot_overlay(
    output_path: Path,
    stage_name: str,
    decision_id: int,
    gt: gate_p.StructuralGT,
    projection: dict,
    pred_canvas: np.ndarray,
    b_gt: np.ndarray,
    b_pred: np.ndarray,
    c0_gt: np.ndarray,
    c0_pred: np.ndarray | None,
    seed: tuple[int, int] | None,
) -> None:
    import matplotlib.pyplot as plt

    support = projection["support"]
    if np.any(support):
        rows, cols = np.nonzero(support)
        r0 = max(0, int(rows.min()) - 4)
        r1 = min(gt.shape[0], int(rows.max()) + 5)
        c0 = max(0, int(cols.min()) - 4)
        c1 = min(gt.shape[1], int(cols.max()) + 5)
    else:
        r0, r1, c0, c1 = 0, gt.shape[0], 0, gt.shape[1]

    observed = projection["canonical_observed"][r0:r1, c0:c1]
    pred = pred_canvas[r0:r1, c0:c1]

    fig, axes = plt.subplots(
        1,
        4,
        figsize=(20, 5),
        constrained_layout=True,
    )

    obs_img = np.full(observed.shape, 0.5, dtype=np.float32)
    obs_img[
        (observed >= 0) & (observed <= GT_OCC_THRESHOLD)
    ] = 1.0
    obs_img[observed > GT_OCC_THRESHOLD] = 0.0
    axes[0].imshow(
        obs_img,
        origin="lower",
        vmin=0.0,
        vmax=1.0,
        cmap="gray",
    )
    axes[0].set_title("Observed canonical")

    axes[1].imshow(
        pred,
        origin="lower",
        vmin=0.0,
        vmax=1.0,
        cmap="gray_r",
    )
    axes[1].set_title("Ensemble mean")

    b = np.zeros(observed.shape + (3,), dtype=np.float32)
    b[:] = 1.0
    bgt = b_gt[r0:r1, c0:c1]
    bpr = b_pred[r0:r1, c0:c1]
    b[bgt] = np.asarray([0.0, 0.0, 0.0])
    b[bpr] = np.asarray([0.5, 0.5, 0.5])
    both = bgt & bpr
    b[both] = np.asarray([0.2, 0.2, 0.2])
    axes[2].imshow(b, origin="lower")
    axes[2].set_title("B: GT surface / pred boundary")

    c = np.ones(observed.shape + (3,), dtype=np.float32)
    cgt = c0_gt[r0:r1, c0:c1]
    c[cgt] = np.asarray([0.15, 0.15, 0.15])
    if c0_pred is not None:
        cpr = c0_pred[r0:r1, c0:c1]
        c[cpr] = np.asarray([0.6, 0.6, 0.6])
        c[cgt & cpr] = np.asarray([0.3, 0.3, 0.3])
    axes[3].imshow(c, origin="lower")
    if (
        seed is not None
        and r0 <= seed[0] < r1
        and c0 <= seed[1] < c1
    ):
        axes[3].plot(
            seed[1] - c0,
            seed[0] - r0,
            marker="x",
            markersize=8,
        )
    axes[3].set_title("C0: GT / predicted point-connected")

    for ax in axes:
        ax.set_xticks([])
        ax.set_yticks([])

    fig.suptitle(
        f"R002 mpx_001 {stage_name} decision {decision_id}"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def _summary_block(rows: list[dict]) -> dict:
    metric_keys = [
        "a_accuracy",
        "a_free_precision",
        "a_free_recall",
        "a_free_iou",
        "a_occupied_precision",
        "a_occupied_recall",
        "a_occupied_iou",
        "a_macro_iou",
        "a_mae",
        "a_interior_solid_area_m2",
        "a_false_free_interior_fraction",
        "b_precision_0p10",
        "b_recall_0p10",
        "b_f1_0p10",
        "b_precision_0p05",
        "b_recall_0p05",
        "b_f1_0p05",
        "b_precision_exact",
        "b_recall_exact",
        "b_f1_exact",
        "c0_gt_area_m2",
        "c0_pred_area_m2",
        "c0_signed_area_error_m2",
        "c0_abs_area_error_m2",
        "c0_free_precision",
        "c0_free_recall",
        "c0_free_iou",
        "a_false_free_interior_excluded_from_c0_fraction",
    ]
    result = {
        "decision_count": len(rows),
        "topology_invalid_count": int(
            sum(int(r["c0_topology_invalid"]) for r in rows)
        ),
        "u_cell_count_sum": int(
            sum(int(r["u_cell_count"]) for r in rows)
        ),
        "u_area_m2_sum": float(
            sum(float(r["u_area_m2"]) for r in rows)
        ),
        "b_gt_support_sum": int(
            sum(int(r["b_gt_count"]) for r in rows)
        ),
        "b_pred_support_sum": int(
            sum(int(r["b_pred_count"]) for r in rows)
        ),
    }
    for key in metric_keys:
        result[f"{key}_mean"] = _finite_mean(
            r[key] for r in rows
        )
        result[f"{key}_std"] = _finite_std(
            r[key] for r in rows
        )
    return result


def analyze_run(
    run_dir: Path,
    gt: gate_p.StructuralGT,
    connected_free: np.ndarray,
    output_dir: Path,
    no_figures: bool,
) -> tuple[list[dict], dict]:
    if run_dir.name != ALLOWED_RUN:
        raise ValueError(
            f"R002 first implementation is frozen to {ALLOWED_RUN}; "
            f"got {run_dir.name}"
        )
    if connected_free.shape != gt.shape:
        raise ValueError(
            "connected-free mask shape does not match structural GT"
        )

    decisions = sorted(
        _read_csv(run_dir / "decisions.csv"),
        key=lambda row: int(row["decision_id"]),
    )
    policies = _read_csv(run_dir / "policy_decisions.csv")
    policy_by_id = {
        int(row["policy_decision_id"]): row
        for row in policies
    }
    policy_by_mapex_id = {
        int(row["mapex_policy_decision_id"]): row
        for row in policies
        if row.get("mapex_policy_decision_id", "").strip()
    }

    metadata = json.loads(
        (run_dir / "metadata.json").read_text(encoding="utf-8")
    )
    observed_grids = [
        gate_p.load_raw_grid(
            _resolve_run_path(run_dir, row["raw_map"])
        )
        for row in decisions
    ]
    provenance_warnings = gate_p.validate_structural_gt_provenance(
        run_name=run_dir.name,
        metadata=metadata,
        structural_gt=gt,
        decision_grids=observed_grids,
    )
    for warning in provenance_warnings:
        print(f"[WARN] {run_dir.name}: {warning}")

    structural_occupied = gt.data > GT_OCC_THRESHOLD
    gt_surface = structural_occupied & _adjacent8(
        connected_free
    )
    # Frozen proposal states interior_solid remains occupied in A and is
    # outside B's surface-target set. Therefore this derived diagnostic is
    # the structural occupied complement of GT_surface.
    interior_solid = structural_occupied & ~gt_surface
    cell_area = gt.resolution * gt.resolution

    rows: list[dict] = []
    visual_indices = _pre_registered_visual_indices(
        len(decisions)
    )
    visual_stage_by_index = {
        index: stage
        for stage, index in visual_indices.items()
    }

    for decision_index, (decision, observed) in enumerate(
        zip(decisions, observed_grids),
        start=1,
    ):
        decision_id = int(decision["decision_id"])
        mapex_id = int(
            decision.get("mapex_policy_decision_id")
            or decision_id
        )
        policy = (
            policy_by_mapex_id.get(mapex_id)
            or policy_by_id.get(mapex_id)
        )
        if policy is None:
            raise KeyError(
                f"no policy_decisions row for MapEx decision {mapex_id}"
            )

        projection = _canonical_projection(observed, gt)
        universe = projection["universe"]
        support = projection["support"]
        topology_domain = gt.evaluation_mask & support

        prediction = gate_p.load_runtime_prediction(
            _resolve_run_path(run_dir, decision["mean_map"]),
            expected_grid=observed,
            expected_member="mean",
            expected_environment=(
                str(metadata.get("environment", "")) or None
            ),
        )
        pred_canvas = _canonical_prediction(
            prediction,
            projection,
            gt,
        )
        if not np.all(np.isfinite(pred_canvas[universe])):
            raise ValueError(
                f"decision {decision_id}: non-finite prediction in U_t"
            )

        pred_free = (
            universe
            & (pred_canvas < PREDICTION_THRESHOLD)
        )

        # Reference A: reuse exact Gate-P2 vector semantics and hard-check
        # parity with the canonical full-canvas representation.
        context = projection["context"]
        a_values_gate = gate_p._structural_prediction_values(
            prediction,
            context,
        )
        a_values_canvas = pred_canvas[universe]
        if not np.array_equal(
            a_values_gate,
            a_values_canvas,
        ):
            if not np.allclose(
                a_values_gate,
                a_values_canvas,
                rtol=0.0,
                atol=0.0,
            ):
                raise AssertionError(
                    f"decision {decision_id}: "
                    "A prediction vector parity failed"
                )
        a_truth_gate = context["truth_occupied"]
        a_truth_canvas = structural_occupied[universe]
        if not np.array_equal(
            a_truth_gate,
            a_truth_canvas,
        ):
            raise AssertionError(
                f"decision {decision_id}: A truth vector parity failed"
            )
        a_metrics = gate_p.classification_metrics(
            a_values_gate,
            a_truth_gate,
        )

        false_free = (
            universe
            & structural_occupied
            & (pred_canvas < PREDICTION_THRESHOLD)
        )
        false_free_count = int(
            np.count_nonzero(false_free)
        )
        false_free_interior = (
            false_free & interior_solid
        )
        false_free_interior_count = int(
            np.count_nonzero(false_free_interior)
        )

        # Reference B.
        hybrid_free = (
            projection["observed_free"]
            | (
                projection["unknown"]
                & (pred_canvas < PREDICTION_THRESHOLD)
            )
        ) & support
        hybrid_occupied = (
            projection["observed_occupied"]
            | (
                projection["unknown"]
                & (pred_canvas >= PREDICTION_THRESHOLD)
            )
        ) & support
        predicted_occupied_boundary = _occupied_boundary(
            hybrid_occupied,
            hybrid_free,
        )
        b_gt = gt_surface & universe
        b_pred = predicted_occupied_boundary & universe

        b_gt_coords = _coords(b_gt)
        b_pred_coords = _coords(b_pred)
        b_010 = _boundary_matching(
            b_gt_coords,
            b_pred_coords,
            gt.resolution,
            PRIMARY_BOUNDARY_TOLERANCE_M,
        )
        b_005 = _boundary_matching(
            b_gt_coords,
            b_pred_coords,
            gt.resolution,
            SENSITIVITY_BOUNDARY_TOLERANCE_M,
        )
        b_exact = _boundary_matching(
            b_gt_coords,
            b_pred_coords,
            gt.resolution,
            EXACT_BOUNDARY_TOLERANCE_M,
        )

        # Reference C0.
        robot_x = float(policy["robot_x"])
        robot_y = float(policy["robot_y"])
        seed, seed_kind = _select_seed(
            robot_x=robot_x,
            robot_y=robot_y,
            observed_free=projection["observed_free"],
            topology_domain=topology_domain,
            gt=gt,
        )

        c0_gt = universe & connected_free
        topology_invalid = seed is None
        c0_pred: np.ndarray | None
        pred_connected: np.ndarray | None
        if topology_invalid:
            c0_pred = None
            pred_connected = None
            c0_metrics = {
                "gt_positive_count": int(
                    np.count_nonzero(c0_gt)
                ),
                "pred_positive_count": 0,
                "true_positive_count": 0,
                "precision": math.nan,
                "recall": math.nan,
                "iou": math.nan,
            }
            c0_pred_area = math.nan
            c0_signed_error = math.nan
            c0_abs_error = math.nan
        else:
            topology_free = topology_domain & (
                projection["observed_free"]
                | (
                    projection["unknown"]
                    & (pred_canvas < PREDICTION_THRESHOLD)
                )
            )
            labels, _ = ndimage.label(
                topology_free,
                structure=FOUR_CONNECTIVITY,
            )
            label_id = int(labels[seed])
            if label_id <= 0:
                raise AssertionError(
                    f"decision {decision_id}: "
                    "valid observed-free seed is not in free topology"
                )
            pred_connected = labels == label_id
            c0_pred = (
                universe
                & (pred_canvas < PREDICTION_THRESHOLD)
                & pred_connected
            )
            c0_metrics = _classification_positive_metrics(
                c0_gt,
                c0_pred,
            )
            c0_pred_area = float(
                c0_metrics["pred_positive_count"]
                * cell_area
            )
            c0_signed_error = (
                c0_pred_area
                - float(
                    c0_metrics["gt_positive_count"]
                    * cell_area
                )
            )
            c0_abs_error = abs(c0_signed_error)

        c0_gt_area = float(
            int(np.count_nonzero(c0_gt))
            * cell_area
        )

        if topology_invalid:
            excluded_count = math.nan
            excluded_fraction = math.nan
        else:
            assert pred_connected is not None
            excluded_mask = (
                false_free_interior
                & ~pred_connected
            )
            excluded_count = int(
                np.count_nonzero(excluded_mask)
            )
            excluded_fraction = _safe_div(
                excluded_count,
                false_free_interior_count,
            )

        progress = decision_index / max(
            len(decisions),
            1,
        )
        row = {
            "run_id": run_dir.name,
            "decision_id": decision_id,
            "decision_index": decision_index,
            "total_decisions": len(decisions),
            "progress_fraction": float(progress),
            "stage": _stage(progress),
            "is_last_n": int(
                (len(decisions) - decision_index)
                < LAST_N
            ),
            "robot_x": robot_x,
            "robot_y": robot_y,
            "support_cell_count": int(
                np.count_nonzero(support)
            ),
            "u_cell_count": int(
                np.count_nonzero(universe)
            ),
            "u_area_m2": float(
                np.count_nonzero(universe)
                * cell_area
            ),
            "origin_rounding_residual_x_m": float(
                context[
                    "origin_rounding_residual_x_m"
                ]
            ),
            "origin_rounding_residual_y_m": float(
                context[
                    "origin_rounding_residual_y_m"
                ]
            ),
            "decision_map_support_gate_p_parity": 1,
            "a_evaluated_cell_count": int(
                a_metrics["n"]
            ),
            "a_tp_occ": int(a_metrics["tp_occ"]),
            "a_tn_free": int(
                a_metrics["tn_free"]
            ),
            "a_fp_occ": int(
                a_metrics["fp_occ"]
            ),
            "a_fn_occ": int(
                a_metrics["fn_occ"]
            ),
            "a_accuracy": float(
                a_metrics["accuracy"]
            ),
            "a_free_precision": float(
                a_metrics["free_precision"]
            ),
            "a_free_recall": float(
                a_metrics["free_recall"]
            ),
            "a_free_iou": float(
                a_metrics["free_iou"]
            ),
            "a_occupied_precision": float(
                a_metrics["occupied_precision"]
            ),
            "a_occupied_recall": float(
                a_metrics["occupied_recall"]
            ),
            "a_occupied_iou": float(
                a_metrics["occupied_iou"]
            ),
            "a_macro_iou": float(
                a_metrics["macro_iou"]
            ),
            "a_mae": float(a_metrics["mae"]),
            "a_false_free_count": false_free_count,
            "a_false_free_area_m2": float(
                false_free_count
                * cell_area
            ),
            "a_interior_solid_cell_count": int(
                np.count_nonzero(
                    interior_solid & universe
                )
            ),
            "a_interior_solid_area_m2": float(
                np.count_nonzero(
                    interior_solid & universe
                )
                * cell_area
            ),
            "a_false_free_interior_count": (
                false_free_interior_count
            ),
            "a_false_free_interior_area_m2": float(
                false_free_interior_count
                * cell_area
            ),
            "a_false_free_interior_fraction": (
                _safe_div(
                    false_free_interior_count,
                    false_free_count,
                )
            ),
            "b_gt_count": int(
                b_gt_coords.shape[0]
            ),
            "b_pred_count": int(
                b_pred_coords.shape[0]
            ),
            "b_matched_0p10": int(
                b_010["matched"]
            ),
            "b_precision_0p10": float(
                b_010["precision"]
            ),
            "b_recall_0p10": float(
                b_010["recall"]
            ),
            "b_f1_0p10": float(
                b_010["f1"]
            ),
            "b_matched_0p05": int(
                b_005["matched"]
            ),
            "b_precision_0p05": float(
                b_005["precision"]
            ),
            "b_recall_0p05": float(
                b_005["recall"]
            ),
            "b_f1_0p05": float(
                b_005["f1"]
            ),
            "b_matched_exact": int(
                b_exact["matched"]
            ),
            "b_precision_exact": float(
                b_exact["precision"]
            ),
            "b_recall_exact": float(
                b_exact["recall"]
            ),
            "b_f1_exact": float(
                b_exact["f1"]
            ),
            "c0_topology_invalid": int(
                topology_invalid
            ),
            "c0_seed_kind": seed_kind,
            "c0_seed_row": (
                ""
                if seed is None
                else int(seed[0])
            ),
            "c0_seed_col": (
                ""
                if seed is None
                else int(seed[1])
            ),
            "c0_gt_positive_count": int(
                c0_metrics["gt_positive_count"]
            ),
            "c0_pred_positive_count": (
                ""
                if topology_invalid
                else int(
                    c0_metrics[
                        "pred_positive_count"
                    ]
                )
            ),
            "c0_gt_area_m2": c0_gt_area,
            "c0_pred_area_m2": c0_pred_area,
            "c0_signed_area_error_m2": (
                c0_signed_error
            ),
            "c0_abs_area_error_m2": (
                c0_abs_error
            ),
            "c0_free_precision": float(
                c0_metrics["precision"]
            ),
            "c0_free_recall": float(
                c0_metrics["recall"]
            ),
            "c0_free_iou": float(
                c0_metrics["iou"]
            ),
            "a_false_free_interior_excluded_from_c0_count": (
                excluded_count
            ),
            "a_false_free_interior_excluded_from_c0_fraction": (
                excluded_fraction
            ),
        }
        rows.append(row)

        stage_name = visual_stage_by_index.get(
            decision_index
        )
        if stage_name and not no_figures:
            _plot_overlay(
                output_path=(
                    output_dir
                    / "overlays"
                    / (
                        f"{stage_name}_decision_"
                        f"{decision_id:06d}.png"
                    )
                ),
                stage_name=stage_name,
                decision_id=decision_id,
                gt=gt,
                projection=projection,
                pred_canvas=pred_canvas,
                b_gt=b_gt,
                b_pred=b_pred,
                c0_gt=c0_gt,
                c0_pred=c0_pred,
                seed=seed,
            )

        c0_iou = row["c0_free_iou"]
        c0_iou_text = (
            f"{c0_iou:.4f}"
            if math.isfinite(c0_iou)
            else "nan"
        )
        print(
            f"[R002] decision {decision_id:03d}: "
            f"A freeIoU={row['a_free_iou']:.4f}, "
            f"B F1@0.10={row['b_f1_0p10']:.4f}, "
            f"C0 IoU={c0_iou_text}, "
            f"topology_invalid="
            f"{row['c0_topology_invalid']}"
        )

    summary = {
        "schema_version": (
            "r002_v2_first_implementation"
        ),
        "run_id": run_dir.name,
        "prediction": {
            "member": "mean",
            "threshold": PREDICTION_THRESHOLD,
        },
        "structural_gt": {
            "ground_truth_id": (
                gt.ground_truth_id
            ),
            "canvas_id": gt.canvas_id,
            "resolution_m": gt.resolution,
        },
        "connected_free": {
            "cell_count": int(
                np.count_nonzero(
                    connected_free
                )
            ),
            "connectivity": 4,
        },
        "reference_b": {
            "primary_tolerance_m": (
                PRIMARY_BOUNDARY_TOLERANCE_M
            ),
            "sensitivity_tolerance_m": (
                SENSITIVITY_BOUNDARY_TOLERANCE_M
            ),
            "matching": (
                "maximum-cardinality one-to-one, "
                "then minimum total Euclidean distance, "
                "deterministic lexicographic tie handling"
            ),
        },
        "reference_c0": {
            "topology_connectivity": 4,
            "seed_fallback_radius_m": (
                SEED_FALLBACK_RADIUS_M
            ),
            "topology_invalid_count": int(
                sum(
                    r["c0_topology_invalid"]
                    for r in rows
                )
            ),
        },
        "visual_selection": {
            stage: {
                "decision_index": index,
                "decision_id": int(
                    rows[index - 1]["decision_id"]
                ),
                "progress_fraction": float(
                    rows[index - 1][
                        "progress_fraction"
                    ]
                ),
            }
            for stage, index
            in visual_indices.items()
        },
        "overall": _summary_block(rows),
        "early": _summary_block(
            [
                r for r in rows
                if r["stage"] == "early"
            ]
        ),
        "mid": _summary_block(
            [
                r for r in rows
                if r["stage"] == "mid"
            ]
        ),
        "late": _summary_block(
            [
                r for r in rows
                if r["stage"] == "late"
            ]
        ),
        "last10": _summary_block(
            [
                r for r in rows
                if int(r["is_last_n"]) == 1
            ]
        ),
        "provenance_warnings": (
            provenance_warnings
        ),
    }
    return rows, summary


def parse_args() -> argparse.Namespace:
    script_path = Path(__file__).resolve()
    mapex_lab_root = script_path.parents[2]
    parser = argparse.ArgumentParser(
        description=(
            "R002 frozen V2 one-run evaluator"
        )
    )
    parser.add_argument(
        "--run",
        default=ALLOWED_RUN,
        help=(
            "Frozen first implementation only "
            f"permits {ALLOWED_RUN}."
        ),
    )
    parser.add_argument(
        "--experiments-root",
        type=Path,
        default=(
            mapex_lab_root
            / "experiments"
            / "mapex"
        ),
    )
    parser.add_argument(
        "--ground-truth",
        type=Path,
        default=(
            mapex_lab_root
            / "ground_truth"
            / "new_room"
            / "generated"
            / "new_room_structural_gt_v2.npz"
        ),
    )
    parser.add_argument(
        "--connected-free",
        type=Path,
        default=(
            mapex_lab_root
            / "ground_truth"
            / "new_room"
            / "generated"
            / "new_room_connected_free_v2.npy"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            script_path.parent
            / "results"
            / "r002_mpx_001"
        ),
    )
    parser.add_argument(
        "--no-figures",
        action="store_true",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.run != ALLOWED_RUN:
        raise ValueError(
            "Frozen R002 first execution is limited "
            f"to {ALLOWED_RUN}; refusing {args.run}"
        )

    experiments_root = (
        args.experiments_root
        .expanduser()
        .resolve()
    )
    run_dir = experiments_root / args.run
    output_dir = (
        args.output_dir
        .expanduser()
        .resolve()
    )
    gt = gate_p.load_structural_gt(
        args.ground_truth
        .expanduser()
        .resolve()
    )
    connected_free = np.asarray(
        np.load(
            args.connected_free
            .expanduser()
            .resolve(),
            allow_pickle=False,
        ),
        dtype=bool,
    )

    rows, summary = analyze_run(
        run_dir=run_dir,
        gt=gt,
        connected_free=connected_free,
        output_dir=output_dir,
        no_figures=args.no_figures,
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True,
    )
    decision_csv = (
        output_dir / "r002_decisions.csv"
    )
    summary_json = (
        output_dir / "r002_summary.json"
    )
    _write_csv(decision_csv, rows)
    summary_json.write_text(
        json.dumps(
            summary,
            indent=2,
            sort_keys=True,
            allow_nan=True,
        )
        + "\n",
        encoding="utf-8",
    )

    print(f"[R002] wrote {decision_csv}")
    print(f"[R002] wrote {summary_json}")
    print(
        "[R002] overall: "
        f"A freeIoU="
        f"{summary['overall']['a_free_iou_mean']:.4f}, "
        f"B F1@0.10="
        f"{summary['overall']['b_f1_0p10_mean']:.4f}, "
        f"C0 freeIoU="
        f"{summary['overall']['c0_free_iou_mean']:.4f}, "
        f"C0 signed area error="
        f"{summary['overall']['c0_signed_area_error_m2_mean']:.4f} m^2, "
        f"topology-invalid="
        f"{summary['reference_c0']['topology_invalid_count']}"
    )
    print(
        "[R002] last10: "
        f"A freeIoU="
        f"{summary['last10']['a_free_iou_mean']:.4f}, "
        f"B F1@0.10="
        f"{summary['last10']['b_f1_0p10_mean']:.4f}, "
        f"C0 freeIoU="
        f"{summary['last10']['c0_free_iou_mean']:.4f}, "
        f"C0 signed area error="
        f"{summary['last10']['c0_signed_area_error_m2_mean']:.4f} m^2"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
