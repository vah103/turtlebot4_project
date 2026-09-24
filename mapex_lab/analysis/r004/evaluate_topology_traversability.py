#!/usr/bin/env python3
"""Frozen R004 V2 topology/traversability diagnostic."""
from __future__ import annotations

import argparse
import csv
import hashlib
import heapq
import json
import math
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.ndimage import binary_erosion, distance_transform_edt

try:
    from mapex_lab.analysis.r004 import evaluate_prediction_vs_final_observed as base
except ModuleNotFoundError:
    import evaluate_prediction_vs_final_observed as base

ACCEPTED_BASE_SHA = "3ba38c2e309673b698f9882499cedd402916c189"
UNDERLYING_BASE_SHA = "550fa35a082041972133700f9688130bc73fd6f3"
RADIUS_M = 0.189
RESOLUTION_M = 0.05
BINS = ("[0.00,0.25)", "[0.25,0.50)", "[0.50,0.75)", "[0.75,1.00]")
NEIGHBORS = ((-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1))
CANONICAL_FOOTPRINT = (
    (0.189, 0.000), (0.134, -0.134), (0.000, -0.189), (-0.134, -0.134),
    (-0.189, 0.000), (-0.134, 0.134), (0.000, 0.189), (0.134, 0.134),
)
METRICS = (
    "navigable_missed_free_fraction", "reachable_future_free_retention",
    "lost_reachable_future_free_fraction", "reachable_pair_connectivity_retention",
    "largest_predicted_piece_fraction", "prediction_component_count",
    "free_support_coverage", "unsupported_future_free_fraction",
    "missed_free_clearance_cells_mean", "missed_free_clearance_meters_mean",
)


def div(a, b):
    return float(a / b) if b else math.nan


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def atomic_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def write_csv(path: Path, rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    fieldnames = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        if fieldnames:
            writer.writeheader()
            writer.writerows(rows)


def strict_occupied(probability):
    return np.asarray(probability) > 0.5


def footprint_radius(footprint):
    return max(max(abs(x), abs(y)) for x, y in footprint)


def validate_resolution(resolution):
    if not math.isclose(float(resolution), RESOLUTION_M, rel_tol=0.0, abs_tol=1e-12):
        raise ValueError(f"canonical resolution {resolution} != {RESOLUTION_M}")


def completed_masks(observed, prediction, final, support):
    known = observed >= 0
    future = (observed < 0) & (final >= 0)
    scoreable = future & support
    domain = known | scoreable
    final_free = final == 0
    reference_occupied = (known & (observed > 0)) | (scoreable & (final > 0))
    prediction_occupied = (known & (observed > 0)) | (scoreable & strict_occupied(prediction))
    missed_free = scoreable & final_free & strict_occupied(prediction)
    return known, future, scoreable, domain, final_free, reference_occupied, prediction_occupied, missed_free


def pair_retention(component_sizes, population):
    if population < 2:
        return math.nan
    return div(sum(size * (size - 1) for size in component_sizes), population * (population - 1))


def largest_piece_fraction(component_sizes, population):
    return div(max(component_sizes) if component_sizes else 0, population)


def read_trajectory(path: Path):
    rows = base.read_csv(path)
    out = []
    for row in rows:
        out.append((float(row["time_s"]), float(row["x"]), float(row["y"]), float(row["yaw"])))
    if not out or any(out[i][0] >= out[i + 1][0] for i in range(len(out) - 1)):
        raise ValueError(f"invalid/non-increasing trajectory: {path}")
    return out


def align_source(time_s: float, trajectory, atol=1e-9):
    times = np.asarray([row[0] for row in trajectory], dtype=float)
    exact = np.flatnonzero(np.isclose(times, time_s, rtol=0.0, atol=atol))
    if exact.size:
        t, x, y, yaw = trajectory[int(exact[0])]
        return {"available": True, "mode": "exact", "x": x, "y": y, "yaw": yaw,
                "lower_time_s": t, "upper_time_s": t, "bracket_gap_s": 0.0}
    hi = int(np.searchsorted(times, time_s, side="right"))
    if hi == 0 or hi == len(trajectory):
        return {"available": False, "mode": "outside_trajectory_range", "x": math.nan,
                "y": math.nan, "yaw": math.nan, "lower_time_s": math.nan,
                "upper_time_s": math.nan, "bracket_gap_s": math.nan}
    lo = hi - 1
    t0, x0, y0, yaw0 = trajectory[lo]
    t1, x1, y1, yaw1 = trajectory[hi]
    alpha = (time_s - t0) / (t1 - t0)
    return {"available": True, "mode": "interpolated", "x": x0 + alpha * (x1 - x0),
            "y": y0 + alpha * (y1 - y0), "yaw": yaw0 + alpha * (yaw1 - yaw0),
            "lower_time_s": t0, "upper_time_s": t1, "bracket_gap_s": t1 - t0}


def world_to_cell(x, y, origin_x, origin_y, resolution):
    return (int(math.floor((y - origin_y) / resolution)),
            int(math.floor((x - origin_x) / resolution)))


def collision_stencil(radius=RADIUS_M, resolution=RESOLUTION_M):
    limit = int(math.ceil(radius / resolution + 0.5))
    stencil = np.zeros((2 * limit + 1, 2 * limit + 1), dtype=bool)
    for dr in range(-limit, limit + 1):
        for dc in range(-limit, limit + 1):
            dx = max(abs(dc) * resolution - resolution / 2.0, 0.0)
            dy = max(abs(dr) * resolution - resolution / 2.0, 0.0)
            stencil[dr + limit, dc + limit] = math.hypot(dx, dy) <= radius + 1e-12
    return stencil


def cspace(free_mask, domain_mask, stencil):
    allowed = np.asarray(free_mask, bool) & np.asarray(domain_mask, bool)
    return binary_erosion(allowed, structure=stencil, border_value=0)


def crop_domain(domain, *arrays):
    rr, cc = np.nonzero(domain)
    if not rr.size:
        raise ValueError("empty decision domain")
    r0, r1, c0, c1 = int(rr.min()), int(rr.max()) + 1, int(cc.min()), int(cc.max()) + 1
    return (r0, c0), tuple(np.asarray(a)[r0:r1, c0:c1] for a in (domain,) + arrays)


def valid_move(mask, row, col, dr, dc):
    nr, nc = row + dr, col + dc
    h, w = mask.shape
    if not (0 <= nr < h and 0 <= nc < w and mask[nr, nc]):
        return False
    if dr and dc and not (mask[row + dr, col] and mask[row, col + dc]):
        return False
    return True


def reachable(mask, source):
    out = np.zeros(mask.shape, dtype=bool)
    if source is None or not mask[source]:
        return out
    queue = deque([source]); out[source] = True
    while queue:
        row, col = queue.popleft()
        for dr, dc in NEIGHBORS:
            nr, nc = row + dr, col + dc
            if valid_move(mask, row, col, dr, dc) and not out[nr, nc]:
                out[nr, nc] = True; queue.append((nr, nc))
    return out


def components(mask):
    labels = np.zeros(mask.shape, dtype=np.int32)
    records = []
    label = 0
    for row, col in zip(*np.nonzero(mask)):
        if labels[row, col]:
            continue
        label += 1; labels[row, col] = label; queue = deque([(row, col)]); cells = []
        while queue:
            r, c = queue.popleft(); cells.append((r, c))
            for dr, dc in NEIGHBORS:
                nr, nc = r + dr, c + dc
                if valid_move(mask, r, c, dr, dc) and labels[nr, nc] == 0:
                    labels[nr, nc] = label; queue.append((nr, nc))
        records.append({"label": label, "cells": cells})
    return labels, records


def shortest_path(mask, start, goal):
    if start == goal:
        return [start]
    dist = {start: 0.0}; parent = {}; queue = [(0.0, 0, start[0], start[1])]; order = 0
    done = set()
    while queue:
        cost, _, row, col = heapq.heappop(queue); node = (row, col)
        if node in done:
            continue
        done.add(node)
        if node == goal:
            break
        for dr, dc in NEIGHBORS:
            if not valid_move(mask, row, col, dr, dc):
                continue
            nxt = (row + dr, col + dc); new = cost + (math.sqrt(2.0) if dr and dc else 1.0)
            if new + 1e-12 < dist.get(nxt, math.inf):
                dist[nxt] = new; parent[nxt] = node; order += 1
                heapq.heappush(queue, (new, order, nxt[0], nxt[1]))
    if goal not in dist:
        return []
    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    return list(reversed(path))


def representative(cells, clearance):
    best = max(clearance[r, c] for r, c in cells)
    return min((r, c) for r, c in cells if math.isclose(clearance[r, c], best, abs_tol=1e-12))


def _collect_key(value, key):
    found = []
    if isinstance(value, dict):
        for name, item in value.items():
            if name == key:
                found.append(item)
            found.extend(_collect_key(item, key))
    elif isinstance(value, list):
        for item in value:
            found.extend(_collect_key(item, key))
    return found


def parse_footprint(value):
    value = yaml.safe_load(value) if isinstance(value, str) else value
    return tuple((float(x), float(y)) for x, y in value)


def preflight(data_root: Path):
    runs = []
    canonical_meta = None
    for run_id in base.RUNS:
        run = data_root / "mapex_lab/experiments/mapex" / run_id
        missing = [name for name in ("decisions.csv", "trajectory.csv", "snapshots.csv", "runtime_nav2_merged.yaml")
                   if not (run / name).is_file()]
        if missing:
            raise ValueError(f"{run_id}: missing inputs {missing}")
        config = yaml.safe_load((run / "runtime_nav2_merged.yaml").read_text(encoding="utf-8"))
        footprints = [parse_footprint(v) for v in _collect_key(config, "footprint")]
        if not footprints or any(fp != CANONICAL_FOOTPRINT for fp in footprints):
            raise ValueError(f"{run_id}: footprint provenance mismatch")
        if not math.isclose(footprint_radius(CANONICAL_FOOTPRINT), RADIUS_M, abs_tol=1e-12):
            raise ValueError(f"{run_id}: frozen radius derivation mismatch")
        decisions = base.read_csv(run / "decisions.csv")
        if not decisions:
            raise ValueError(f"{run_id}: no decisions")
        _, meta, _ = base.load_canvas(run, decisions[0]["canvas_map"])
        validate_resolution(meta[1])
        canonical_meta = canonical_meta or meta
        if meta != canonical_meta:
            raise ValueError(f"{run_id}: canonical canvas mismatch")
        read_trajectory(run / "trajectory.csv")
        base.final_snapshot(run)
        for row in decisions:
            base.prediction_canvas(run, row, meta[0])
        runs.append({"run_id": run_id, "run_path": str(run.resolve()), "decision_count": len(decisions),
                     "footprint_entries": len(footprints), "footprint_valid": True,
                     "frozen_radius_m": RADIUS_M, "canonical_resolution_m": meta[1]})
    return {"schema": "r004_topology_preflight_v1", "data_root": str(data_root.resolve()),
            "runs": runs, "canonical_canvas": {"shape": list(canonical_meta[0]),
            "resolution": canonical_meta[1], "origin_x": canonical_meta[2],
            "origin_y": canonical_meta[3], "canvas_id": canonical_meta[4]},
            "footprint": [list(v) for v in CANONICAL_FOOTPRINT], "radius_m": RADIUS_M,
            "inflation_radius_used_as_hard_radius": False}


def input_fingerprints(run: Path, decisions):
    paths = {run / name for name in ("decisions.csv", "trajectory.csv", "snapshots.csv", "runtime_nav2_merged.yaml")}
    for row in decisions:
        for key in ("raw_map", "canvas_map", "mean_map"):
            paths.add(base.artifact_path(run, row[key]))
    return {str(path.relative_to(run)): sha256(path) for path in sorted(paths)}


def component_inventory(labels, source_set, offset):
    records = []
    total = int(source_set.sum())
    for label in np.unique(labels[source_set & (labels > 0)]):
        cells = list(zip(*np.nonzero(source_set & (labels == label))))
        records.append({"label": int(label), "size": len(cells), "fraction": div(len(cells), total),
                        "minimum_cell": [min(cells)[0] + offset[0], min(cells)[1] + offset[1]],
                        "local_cells": cells})
    records.sort(key=lambda item: (-item["size"], item["minimum_cell"]))
    return records


def render_topology(path, domain, ref_trav, pred_trav, missed, source, offset, barrier=None):
    image = np.ones((*domain.shape, 3), dtype=np.float32)
    image[domain] = (0.82, 0.82, 0.82)
    image[ref_trav] = (0.30, 0.70, 0.35)
    image[ref_trav & ~pred_trav] = (0.95, 0.45, 0.05)
    image[missed] = (0.90, 0.05, 0.10)
    if barrier:
        for row, col in barrier["path"]:
            image[row, col] = (0.15, 0.35, 0.95) if pred_trav[row, col] else (0.75, 0.05, 0.80)
        for row, col in barrier["representatives"]:
            image[row, col] = (0.05, 0.05, 0.05)
    if source is not None and 0 <= source[0] < domain.shape[0] and 0 <= source[1] < domain.shape[1]:
        image[source] = (0.05, 0.05, 0.05)
    path.parent.mkdir(parents=True, exist_ok=True)
    plt.imsave(path, image)


def analyze_run(data_root: Path, run_id: str, output: Path):
    run = data_root / "mapex_lab/experiments/mapex" / run_id
    final, final_file, fallback = base.final_snapshot(run)
    decisions = base.read_csv(run / "decisions.csv")
    _, meta, _ = base.load_canvas(run, decisions[0]["canvas_map"])
    shape, resolution, origin_x, origin_y, canvas_id = meta
    validate_resolution(resolution)
    trajectory = read_trajectory(run / "trajectory.csv")
    stencil = collision_stencil(RADIUS_M, resolution)
    if not np.any(final == 1):
        raise ValueError(f"{run_id}: FinalObserved has no occupied cells")
    occupied_distance = distance_transform_edt(final != 1)
    rows, sources, fragments, overlay_rows, clearance_rows = [], [], [], [], []
    for index, decision in enumerate(decisions, 1):
        obs, prediction, support = base.prediction_canvas(run, decision, shape)
        known, future, scoreable, domain, final_free, ref_occupied, pred_occupied, missed = completed_masks(
            obs, prediction, final, support)
        offset, cropped = crop_domain(domain, ref_occupied, pred_occupied, missed, scoreable,
                                      final_free, occupied_distance)
        d, ro, po, miss, e, ffree, clearance = cropped
        ref_trav = cspace(d & ~ro, d, stencil); pred_trav = cspace(d & ~po, d, stencil)
        progress = base.progress(index, len(decisions)); decision_id = int(decision["decision_id"])
        source = align_source(float(decision["time_s"]), trajectory)
        source_global = None; source_local = None; source_reason = source["mode"]
        if source["available"]:
            source_global = world_to_cell(source["x"], source["y"], origin_x, origin_y, resolution)
            source_local = (source_global[0] - offset[0], source_global[1] - offset[1])
            if not (0 <= source_local[0] < d.shape[0] and 0 <= source_local[1] < d.shape[1]):
                source_reason = "source_outside_decision_domain"; source["available"] = False
            elif not ref_trav[source_local]:
                source_reason = "source_not_reference_traversable"; source["available"] = False
        missed_count = int(miss.sum()); navigable_missed = int(np.sum(miss & ref_trav))
        miss_clear = clearance[miss]
        if miss_clear.size:
            values, counts = np.unique(miss_clear, return_counts=True)
            for value, count in zip(values, counts):
                at_value = miss & np.isclose(clearance, value, rtol=0.0, atol=1e-12)
                clearance_rows.append({"run_id": run_id, "decision_id": decision_id,
                                       "clearance_cells": float(value),
                                       "clearance_meters": float(value * resolution),
                                       "count": int(count),
                                       "reference_navigable_count": int(np.sum(at_value & ref_trav))})
        record = {"run_id": run_id, "decision_id": decision_id, "decision_index": index,
                  "decision_count": len(decisions), "decision_progress": progress,
                  "progress_bin": base.pbin(progress), "domain_count": int(d.sum()),
                  "known_count": int(known.sum()), "future_observed_count": int(future.sum()),
                  "scoreable_count": int(scoreable.sum()), "supported_future_free_count": int(np.sum(scoreable & final_free)),
                  "unsupported_future_free_count": int(np.sum(future & final_free & ~support)),
                  "free_support_coverage": div(np.sum(scoreable & final_free), np.sum(future & final_free)),
                  "unsupported_future_free_fraction": div(np.sum(future & final_free & ~support), np.sum(future & final_free)),
                  "missed_free_count": missed_count, "navigable_missed_free_count": navigable_missed,
                  "navigable_missed_free_fraction": div(navigable_missed, missed_count),
                  "missed_free_clearance_cells_mean": float(np.mean(miss_clear)) if miss_clear.size else math.nan,
                  "missed_free_clearance_cells_median": float(np.median(miss_clear)) if miss_clear.size else math.nan,
                  "missed_free_clearance_cells_p90": float(np.percentile(miss_clear, 90)) if miss_clear.size else math.nan,
                  "missed_free_clearance_meters_mean": float(np.mean(miss_clear) * resolution) if miss_clear.size else math.nan,
                  "source_available": int(source["available"]), "source_reason": source_reason,
                  "source_prediction_traversable": 0, "reference_source_component_size": 0,
                  "reachable_future_free_count": 0, "prediction_reachable_future_free_count": 0,
                  "reachable_future_free_retention": math.nan, "lost_reachable_future_free_fraction": math.nan,
                  "pair_denominator_n": 0, "prediction_component_count": math.nan,
                  "reachable_pair_connectivity_retention": math.nan,
                  "largest_predicted_piece_fraction": math.nan, "fragmented": 0}
        inventory = []
        barrier = None
        if source["available"]:
            ref_reach = reachable(ref_trav, source_local)
            pred_reach = reachable(pred_trav, source_local) if pred_trav[source_local] else np.zeros(d.shape, bool)
            r_ref = e & ffree & ref_trav & ref_reach
            n_future = int(r_ref.sum()); kept_future = int(np.sum(r_ref & pred_reach))
            s_ref = ref_reach; n = int(s_ref.sum())
            labels, _ = components(pred_trav)
            inventory = component_inventory(labels, s_ref, offset)
            sizes = [item["size"] for item in inventory]
            pair = pair_retention(sizes, n)
            record.update(source_prediction_traversable=int(pred_trav[source_local]),
                          reference_source_component_size=n, reachable_future_free_count=n_future,
                          prediction_reachable_future_free_count=kept_future,
                          reachable_future_free_retention=div(kept_future, n_future),
                          lost_reachable_future_free_fraction=(1.0 - div(kept_future, n_future)) if n_future else math.nan,
                          pair_denominator_n=n, prediction_component_count=len(inventory),
                          reachable_pair_connectivity_retention=pair,
                          largest_predicted_piece_fraction=largest_piece_fraction(sizes, n),
                          fragmented=int(len(inventory) >= 2))
            if len(inventory) >= 2 and progress >= 0.75:
                ref_clearance = distance_transform_edt(ref_trav)
                reps = [representative(item["local_cells"], ref_clearance) for item in inventory[:2]]
                path = shortest_path(ref_trav, reps[0], reps[1])
                barrier = {"representatives": reps, "path": path,
                           "obstructed_path_cells": int(sum(not pred_trav[cell] for cell in path))}
                name = f"{run_id}_decision_{decision_id:03d}_false_barrier.png"
                render_topology(output / "overlays" / name, d, ref_trav, pred_trav, miss,
                                source_local, offset, barrier)
                overlay_rows.append({"run_id": run_id, "decision_id": decision_id,
                                     "kind": "false_barrier", "file": f"overlays/{name}",
                                     "obstructed_path_cells": barrier["obstructed_path_cells"]})
        if index == len(decisions):
            name = f"{run_id}_decision_{decision_id:03d}_late_topology.png"
            render_topology(output / "overlays" / name, d, ref_trav, pred_trav, miss,
                            source_local if source["available"] else None, offset)
            overlay_rows.append({"run_id": run_id, "decision_id": decision_id,
                                 "kind": "late_topology", "file": f"overlays/{name}",
                                 "obstructed_path_cells": ""})
        sources.append({"run_id": run_id, "decision_id": decision_id, "decision_time_s": float(decision["time_s"]),
                        **source, "source_row": source_global[0] if source_global else "",
                        "source_col": source_global[1] if source_global else "", "availability_reason": source_reason})
        fragments.append({"run_id": run_id, "decision_id": decision_id,
                          "pair_denominator_n": record["pair_denominator_n"],
                          "component_count": record["prediction_component_count"],
                          "components": [{k: v for k, v in item.items() if k != "local_cells"} for item in inventory]})
        rows.append(record)
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / "topology_diagnostic_decisions.csv", rows)
    write_csv(output / "source_pose_alignment.csv", sources)
    write_csv(output / "overlay_manifest.csv", overlay_rows)
    write_csv(output / "missed_free_clearance_distribution.csv", clearance_rows)
    atomic_json(output / "fragmentation_inventory.json", fragments)
    result = {"schema": "r004_topology_run_result_v1", "run_id": run_id,
              "run_path": str(run.resolve()), "decision_count": len(rows),
              "source_available_count": sum(r["source_available"] for r in rows),
              "source_unavailable_count": sum(not r["source_available"] for r in rows),
              "late_fragmentation_count": sum(r["fragmented"] and r["decision_progress"] >= 0.75 for r in rows),
              "false_barrier_overlay_count": sum(r["kind"] == "false_barrier" for r in overlay_rows),
              "final_snapshot": final_file, "fallback_final_snapshot": fallback,
              "canvas_id": canvas_id, "resolution_m": resolution,
              "input_fingerprints": input_fingerprints(run, decisions)}
    atomic_json(output / "run_result.json", result)
    return result


def run_bin_rows(rows):
    per_run = []
    for run_id in base.RUNS:
        for progress_bin in BINS:
            chosen = [row for row in rows if row["run_id"] == run_id and row["progress_bin"] == progress_bin]
            item = {"aggregation": "run_median", "run_id": run_id, "progress_bin": progress_bin,
                    "decision_count": len(chosen)}
            for metric in METRICS:
                values = base.finite([row[metric] for row in chosen])
                item[metric] = float(np.median(values)) if values.size else math.nan
            per_run.append(item)
    macro = []
    for progress_bin in BINS:
        chosen = [row for row in per_run if row["progress_bin"] == progress_bin]
        item = {"aggregation": "run_macro", "run_id": "", "progress_bin": progress_bin,
                "decision_count": sum(row["decision_count"] for row in chosen)}
        for metric in METRICS:
            for key, value in base.summarize([row[metric] for row in chosen]).items():
                item[f"{metric}_{key}"] = value
        macro.append(item)
    return per_run, macro


def plot_macro(macro, output, filename, series, title, ylabel):
    x = np.arange(len(BINS)); fig, ax = plt.subplots(figsize=(8, 5))
    for metric, label, style in series:
        ax.errorbar(x, [row[f"{metric}_mean"] for row in macro],
                    yerr=[row[f"{metric}_std"] for row in macro], fmt=style, capsize=3, label=label)
    ax.set_xticks(x, BINS, rotation=15); ax.set(xlabel="Decision-progress bin", ylabel=ylabel, title=title)
    ax.grid(alpha=0.25); ax.legend(); fig.tight_layout(); fig.savefig(output / filename, dpi=160); plt.close(fig)


def aggregate(output_root: Path):
    rows, sources, fragments, overlays, clearance_rows = [], [], [], [], []
    run_results = []
    for run_id in base.RUNS:
        directory = output_root / "runs" / run_id
        manifest = json.loads((directory / "completion_manifest.json").read_text())
        if manifest.get("status") != "complete" or manifest.get("run_id") != run_id:
            raise ValueError(f"{run_id}: invalid completion manifest")
        rows.extend(base.read_csv(directory / "topology_diagnostic_decisions.csv"))
        sources.extend(base.read_csv(directory / "source_pose_alignment.csv"))
        fragments.extend(json.loads((directory / "fragmentation_inventory.json").read_text()))
        clearance_rows.extend(base.read_csv(directory / "missed_free_clearance_distribution.csv"))
        for item in base.read_csv(directory / "overlay_manifest.csv"):
            item["file"] = f"runs/{run_id}/{item['file']}"; overlays.append(item)
        run_results.append(json.loads((directory / "run_result.json").read_text()))
    numeric = set(METRICS) | {"decision_progress", "source_available", "fragmented"}
    for row in rows:
        for key in numeric:
            if key in row and row[key] != "": row[key] = float(row[key])
    per_run, macro = run_bin_rows(rows)
    write_csv(output_root / "topology_diagnostic_decisions.csv", rows)
    write_csv(output_root / "source_pose_alignment_inventory.csv", sources)
    write_csv(output_root / "topology_overlay_manifest.csv", overlays)
    write_csv(output_root / "missed_free_clearance_distribution.csv", clearance_rows)
    combined_bins = per_run + macro
    write_csv(output_root / "topology_diagnostic_run_bins.csv", combined_bins)
    atomic_json(output_root / "fragmentation_inventory.json", fragments)
    figures = output_root / "figures"; figures.mkdir(exist_ok=True)
    plot_macro(macro, figures, "navigable_missed_free_vs_progress.png",
               (("navigable_missed_free_fraction", "Navigable MissedFree", "o-"),),
               "Reference-navigable MissedFree", "Run-macro fraction mean ± std")
    plot_macro(macro, figures, "reachable_future_free_retention_vs_progress.png",
               (("reachable_future_free_retention", "Retention", "o-"),
                ("lost_reachable_future_free_fraction", "Lost fraction", "s--")),
               "Reachable future-free retention", "Run-macro fraction mean ± std")
    plot_macro(macro, figures, "reachable_pair_connectivity_retention_vs_progress.png",
               (("reachable_pair_connectivity_retention", "Pair retention", "o-"),),
               "Reachable-pair connectivity retention", "Run-macro retention mean ± std")
    plot_macro(macro, figures, "largest_piece_and_component_count_vs_progress.png",
               (("largest_predicted_piece_fraction", "Largest piece", "o-"),
                ("prediction_component_count", "Component count", "s--")),
               "Predicted fragmentation", "Run-macro value mean ± std")
    plot_macro(macro, figures, "support_context_vs_progress.png",
               (("free_support_coverage", "Free support", "o-"),
                ("unsupported_future_free_fraction", "Unsupported future free", "s--")),
               "Future-free support context", "Run-macro fraction mean ± std")
    clearance = np.asarray([float(row["clearance_cells"]) for row in clearance_rows], dtype=float)
    weights = np.asarray([int(row["count"]) for row in clearance_rows], dtype=int)
    fig, ax = plt.subplots(figsize=(7, 4)); ax.hist(clearance, bins=20, weights=weights)
    ax.set(xlabel="MissedFree clearance (cells)", ylabel="Cells", title="MissedFree clearance distribution")
    fig.tight_layout(); fig.savefig(figures / "missed_free_clearance_distribution.png", dpi=160); plt.close(fig)
    summary = {"schema": "r004_topology_traversability_v1", "accepted_base_sha": ACCEPTED_BASE_SHA,
               "included_runs": list(base.RUNS), "excluded_runs": [], "run_count": len(base.RUNS),
               "decision_count": len(rows), "source_available_count": sum(int(float(r["source_available"])) for r in rows),
               "source_unavailable_count": sum(not int(float(r["source_available"])) for r in rows),
               "late_fragmentation_count": sum(int(float(r["fragmented"])) and float(r["decision_progress"]) >= 0.75 for r in rows),
               "false_barrier_overlay_count": sum(r["kind"] == "false_barrier" for r in overlays),
               "run_macro_progress_bins": macro, "scientific_interpretation": None}
    atomic_json(output_root / "topology_diagnostic_summary.json", summary)
    provenance = {"schema": "r004_topology_provenance_v1", "accepted_base_sha": ACCEPTED_BASE_SHA,
                  "underlying_base_sha": UNDERLYING_BASE_SHA, "runs": list(base.RUNS),
                  "data_root": run_results[0]["run_path"].split("/mapex_lab/experiments/mapex/")[0],
                  "threshold": "occupied iff p > 0.5", "radius_m": RADIUS_M,
                  "collision": "closed disk against full closed cell squares; touching collides",
                  "outside_domain": "collision", "graph": "8-neighbour N,NE,E,SE,S,SW,W,NW; no corner cutting",
                  "source": "trajectory exact-time or strict bracket interpolation; floor containing-cell; no snapping/extrapolation",
                  "pair_denominator": "all cells in reference source-reachable component; prediction-occupied retained in N",
                  "aggregation": "decision -> run/bin median -> equal-run mean/sample std/median/count"}
    atomic_json(output_root / "topology_provenance.json", provenance)
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("data_root", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--run-id", choices=base.RUNS)
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--aggregate", action="store_true")
    args = parser.parse_args()
    if args.preflight:
        print(json.dumps(preflight(args.data_root), indent=2)); return
    if args.aggregate:
        print(json.dumps(aggregate(args.output), indent=2)); return
    if not args.run_id:
        parser.error("--run-id is required unless --preflight or --aggregate is used")
    print(json.dumps(analyze_run(args.data_root, args.run_id, args.output), indent=2))


if __name__ == "__main__":
    main()
