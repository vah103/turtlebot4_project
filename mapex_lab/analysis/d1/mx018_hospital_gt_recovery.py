#!/usr/bin/env python3
"""Frozen MX017 V2 semantic identity and MX018 recovery helpers."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import struct
from pathlib import Path

import cv2
import numpy as np

from mapex_lab.scripts import evaluate_mapex_run as evaluator
from mapex_lab.scripts import hospital_ground_truth_core as core
from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo

SCHEMA = "mapex_structural_gt_semantic_manifest_v1"
TARGET_SHA = "7db2d4a2b5866e6854baa5b3e02ca3dd22ce205fcfcac5ec431e872c4724b9e9"
WORLD_SHA = "59d482029ad56dabf7ce9bb68ece16e476fd263c0ec57f237dff1f893880aeb0"
WORLD_BLOB = "70b01b01f4b908a9ca8acfc16d10772558180ee9"
MESH_SHA = "233b75caa7eb94807427505d21e15522f3f5b15ce30f3b6a6f51b58eb901a523"
MESH_BLOB = "de2c21d5a28ba1c68c266d6730868d04d3fb2730"
GENERATOR_BLOB = "f0b4c09ab1d6cbdf68ed519e30eb2bf340e74961"
CORE_BLOB = "f50aa99a0e6bedbab2aa07274fdd07383f871acd"
RESULT_TO_ACTION = {
    "ORIGINAL_EXACT_ARTIFACT_RECOVERED": "RESUME_MX016_EXACT_G2_PATH",
    "EXACT_BINARY_RECONSTRUCTION": "RESUME_MX016_EXACT_G2_PATH",
    "SEMANTIC_EQUIVALENCE_PROVEN_BINARY_MISMATCH": "PACKAGE_SEMANTIC_RECOVERY_EVIDENCE_FOR_QA",
    "SEMANTIC_MISMATCH_PROVEN": "GENERATE_GT_V2_AFTER_SEPARATE_ARTIFACT_GATE",
    "SEMANTIC_RECONSTRUCTION_SUPPORTED_NOT_PROVEN": "GENERATE_GT_V2_AFTER_SEPARATE_ARTIFACT_GATE",
    "SEMANTIC_RECONSTRUCTION_NOT_SUPPORTED": "GENERATE_GT_V2_AFTER_SEPARATE_ARTIFACT_GATE",
    "RECONSTRUCTION_NONDETERMINISTIC": "GENERATE_GT_V2_AFTER_SEPARATE_ARTIFACT_GATE",
    "SOURCE_PROVENANCE_MISMATCH": "STOP_AND_ESCALATE_PROVENANCE",
    "RECONSTRUCTION_ENVIRONMENT_BLOCKED": "STOP_AND_ESCALATE_EXECUTION_ENVIRONMENT",
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def f64be_hex(value) -> str:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("semantic float must be finite")
    return struct.pack(">d", number).hex()


def scalar(bundle, key):
    value = np.asarray(bundle[key])
    if value.size != 1:
        raise ValueError(f"{key} must be scalar")
    return value.reshape(()).item()


def canonical_manifest(gt_path: Path, *, gt_id: str, evaluation_domain_rule: str) -> tuple[dict, bytes, str]:
    with np.load(gt_path, allow_pickle=False) as bundle:
        data = np.ascontiguousarray(np.asarray(bundle["data"], dtype="<i2"))
        mask = np.ascontiguousarray(np.asarray(bundle["evaluation_mask"], dtype=np.uint8))
        if not set(np.unique(data)).issubset({-1, 0, 100}):
            raise ValueError("invalid data label inventory")
        if not set(np.unique(mask)).issubset({0, 1}):
            raise ValueError("invalid evaluation_mask inventory")
        height, width = data.shape
        if mask.shape != data.shape or (height, width) != (2123, 1504):
            raise ValueError("invalid canonical Hospital shape")
        manifest = {
            "schema": SCHEMA,
            "arrays": {
                "data": {"dtype": "<i2", "shape": [height, width], "sha256": sha256_bytes(data.tobytes(order="C"))},
                "evaluation_mask": {"dtype": "uint8", "shape": [height, width], "sha256": sha256_bytes(mask.tobytes(order="C"))},
            },
            "geometry": {
                "canvas_id": str(scalar(bundle, "canvas_id")), "ground_truth_id": gt_id,
                "width": width, "height": height,
                "resolution_f64be_hex": f64be_hex(scalar(bundle, "resolution")),
                "origin_x_f64be_hex": f64be_hex(scalar(bundle, "origin_x")),
                "origin_y_f64be_hex": f64be_hex(scalar(bundle, "origin_y")),
                "hospital_scale_f64be_hex": f64be_hex(scalar(bundle, "hospital_scale")),
                "spawn_world_x_f64be_hex": f64be_hex(scalar(bundle, "spawn_world_x")),
                "spawn_world_y_f64be_hex": f64be_hex(scalar(bundle, "spawn_world_y")),
                "spawn_world_yaw_f64be_hex": f64be_hex(scalar(bundle, "spawn_world_yaw")),
                "world_to_slam_rule": "R(-spawn_world_yaw) @ (p_world - p_spawn)",
            },
            "source": {
                "source_world_sha256": str(scalar(bundle, "source_world_sha256")),
                "source_world_git_blob": WORLD_BLOB,
                "source_wall_mesh_sha256": str(scalar(bundle, "source_wall_mesh_sha256")),
                "source_wall_mesh_git_blob": MESH_BLOB,
                "generator_git_blob": GENERATOR_BLOB, "core_git_blob": CORE_BLOB,
            },
            "rasterization": {
                "wall_slice_z_m_f64be_hex": f64be_hex(.30), "wall_thickness_cells": 2,
                "close_raster_cracks_dilation_cells": 1, "include_flat_elevator_blockers": True,
                "metric_to_cell_rule": "np.rint", "evaluation_domain_rule": evaluation_domain_rule,
                "free_component_connectivity": 8,
            },
        }
    canonical = json.dumps(manifest, sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False).encode("utf-8")
    return manifest, canonical, sha256_bytes(canonical)


def occupied_iou_fingerprint(run_dir: Path, gt_path: Path) -> dict:
    saved = {int(row["decision_id"]): float(row["occupied_iou"]) for row in csv.DictReader((run_dir / "evaluation.csv").open())}
    decisions = list(csv.DictReader((run_dir / "decisions.csv").open()))
    mask, occupied, _ = evaluator._load_ground_truth(gt_path)
    comparisons = []
    for row in decisions:
        decision_id = int(row["decision_id"])
        canvas = evaluator._prediction_to_canvas(run_dir / row["mean_map"])
        value = evaluator.occupied_iou(canvas, occupied, mask)
        delta = abs(value - saved[decision_id])
        comparisons.append({"decision_id": decision_id, "recomputed": value, "saved_9dp": saved[decision_id], "absolute_difference": delta, "pass": delta <= 5e-10})
    return {
        "comparison": "occupied_iou_56_decisions", "tolerance_absolute": 5e-10,
        "decision_count": len(comparisons), "all_pass": len(comparisons) == 56 and all(x["pass"] for x in comparisons),
        "max_absolute_difference": max(x["absolute_difference"] for x in comparisons),
        "mismatches": [x for x in comparisons if not x["pass"]], "comparisons": comparisons,
        "tu_corroboration": {"status": "NOT_RUN", "reason": "legacy goal-source audit is secondary and unnecessary for structural fingerprint decision"},
    }


def generate_v2(output_dir: Path) -> dict:
    """Generate frozen geometry-derived GT-v2; reads no hpx decision/outcome data."""
    output_dir.mkdir(parents=True, exist_ok=True)
    spec = core.load_spec()
    roi, obstacle, bbox_mask, audit, _metric_to_rc, mesh_path = core.build_geometry(spec)
    occupied = (obstacle > 0) & bbox_mask
    evaluation_mask = roi | occupied
    data = np.full(roi.shape, -1, dtype=np.int16); data[roi] = 0; data[occupied] = 100
    roi_path = output_dir / "hospital_connected_free_v2.npy"
    gt_path = output_dir / "hospital_structural_gt_v2.npz"
    preview_path = output_dir / "hospital_structural_gt_v2_preview.png"
    np.save(roi_path, roi.astype(np.uint8), allow_pickle=False)
    canvas = spec["canvas"]
    np.savez_compressed(
        gt_path, data=data, evaluation_mask=evaluation_mask,
        resolution=np.float64(spec["resolution_m"]), width=np.int64(canvas["width_cells"]), height=np.int64(canvas["height_cells"]),
        origin_x=np.float64(canvas["origin_x_m"]), origin_y=np.float64(canvas["origin_y_m"]),
        canvas_id=np.asarray("hospital_canvas_v1"), ground_truth_id=np.asarray("hospital_structural_gt_v2"), roi_id=np.asarray("hospital_connected_free_v2"),
        source_world=np.asarray("mapex_lab/map/hospital_aws_flat.sdf"), source_world_sha256=np.asarray(core.sha256_file(core.WORLD_PATH)),
        source_wall_mesh=np.asarray("mapex_lab/map/models/aws_robomaker_hospital_floor_01_walls/meshes/aws_robomaker_hospital_floor_01_walls_collision.dae"),
        source_wall_mesh_sha256=np.asarray(core.sha256_file(mesh_path)), hospital_scale=np.float64(1.0),
        spawn_world_x=np.float64(core.SPAWN_X), spawn_world_y=np.float64(core.SPAWN_Y), spawn_world_yaw=np.float64(core.SPAWN_YAW),
    )
    preview=np.zeros(roi.shape,dtype=np.uint8); preview[roi]=200; preview[occupied]=255; cv2.imwrite(str(preview_path),np.flipud(preview))
    manifest, canonical, digest = canonical_manifest(gt_path, gt_id="hospital_structural_gt_v2", evaluation_domain_rule="frozen_hospital_bounds_then_start_connected_free_plus_structural_occupied_v2")
    (output_dir / "gt_v2_semantic_manifest.canonical.json").write_bytes(canonical)
    summary = {
        "ground_truth_id": "hospital_structural_gt_v2", "connected_free_id": "hospital_connected_free_v2",
        "gt_file_sha256": sha256_file(gt_path), "connected_free_sha256": sha256_file(roi_path),
        "semantic_digest": digest, "semantic_manifest_sha256": sha256_file(output_dir / "gt_v2_semantic_manifest.canonical.json"),
        "shape": list(data.shape), "labels": [int(x) for x in np.unique(data)], "evaluation_mask_cells": int(evaluation_mask.sum()),
        "connected_free_cells": int(roi.sum()), "occupied_cells": int(occupied.sum()), "audit": audit,
        "decision_30_used_for_gt_construction": False, "transfer_scoring_performed": False,
    }
    (output_dir / "hospital_structural_gt_v2_summary.json").write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n")
    (output_dir / "gt_v2_semantic_manifest.json").write_text(json.dumps({"manifest":manifest,"semantic_digest":digest},indent=2,sort_keys=True)+"\n")
    return summary


def validate_v2_alignment(run_dir: Path, gt_path: Path) -> dict:
    """Frozen geometry-only hpx alignment checks; performs no transfer scoring."""
    with np.load(gt_path,allow_pickle=False) as gt:
        data=np.asarray(gt["data"]); mask=np.asarray(gt["evaluation_mask"],bool)
        resolution=float(scalar(gt,"resolution")); ox=float(scalar(gt,"origin_x")); oy=float(scalar(gt,"origin_y"))
    start=(1202,512); structural=(data==0)&mask
    safe=topo.cspace(structural,mask,topo.collision_stencil(radius=.189,resolution=resolution))
    universe=topo.reachable(safe,start)
    decisions=list(csv.DictReader((run_dir/"decisions.csv").open()))
    rows=[]; reference_resolution=None
    for row in decisions:
        with np.load(run_dir/row["raw_map"],allow_pickle=False) as raw:
            rr=float(scalar(raw,"resolution")); yaw=float(scalar(raw,"origin_yaw")); rx=float(scalar(raw,"origin_x")); ry=float(scalar(raw,"origin_y"))
        if reference_resolution is None: reference_resolution=rr
        ratio_f=rr/resolution; ratio=int(round(ratio_f)); row0=round((ry-oy)/resolution); col0=round((rx-ox)/resolution)
        residual_x=rx-(ox+col0*resolution); residual_y=ry-(oy+row0*resolution)
        checks={
            "runtime_resolution_consistent":math.isclose(rr,reference_resolution,rel_tol=0,abs_tol=1e-6),
            "ratio_integer_two":ratio==2 and math.isclose(ratio_f,ratio,rel_tol=0,abs_tol=1e-6),
            "axis_aligned":abs(yaw)<=1e-6,
            "origin_residual":abs(residual_x)<=.025+1e-6 and abs(residual_y)<=.025+1e-6,
        }
        rows.append({"decision_id":int(row["decision_id"]),"runtime_resolution":rr,"ratio":ratio,"origin_yaw":yaw,
                     "residual_x_m":residual_x,"residual_y_m":residual_y,"checks":checks,"pass":all(checks.values())})
    result={
        "ground_truth_id":"hospital_structural_gt_v2","source_world_sha256":WORLD_SHA,
        "start_cell":[1202,512],"start_structural_free":bool(structural[start]),"start_footprint_safe":bool(safe[start]),
        "universe_cells":int(universe.sum()),"decision_count":len(rows),"all_decisions_pass":len(rows)==56 and all(x["pass"] for x in rows),
        "transfer_scoring_performed":False,"decisions":rows,
    }
    return result


def recovery_result(result: str, candidate_sha: str | None, digest: str | None) -> dict:
    if result not in RESULT_TO_ACTION:
        raise ValueError("non-frozen recovery_result")
    exact = result in {"ORIGINAL_EXACT_ARTIFACT_RECOVERED", "EXACT_BINARY_RECONSTRUCTION"}
    blocked = result in {"SOURCE_PROVENANCE_MISMATCH", "RECONSTRUCTION_ENVIRONMENT_BLOCKED"}
    return {
        "task": "MX017", "execution_task": "MX018", "run_id": "hpx_001", "recovery_result": result,
        "next_action": RESULT_TO_ACTION[result], "recorded_target_gt_sha256": TARGET_SHA,
        "candidate_gt_sha256": candidate_sha, "semantic_manifest_schema": SCHEMA,
        "candidate_semantic_digest": digest, "direct_original_semantic_reference_available": False,
        "mx015_g2_literal_status": "PASS_EXACT_SHA" if exact else ("NOT_EVALUABLE" if blocked else "FAIL_EXACT_SHA"),
        "decision_30_used_for_gt_construction": False,
    }
