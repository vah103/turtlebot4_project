#!/usr/bin/env python3
"""MX016 thin Hospital adapter and truth-side evaluator for accepted MX015 V2."""
from __future__ import annotations

import csv
import json
import math
from hashlib import sha256
from pathlib import Path

import numpy as np

from mapex_lab.analysis.d1.extract_shared_phase0_evidence import load_runtime, runtime_fields
from mapex_lab.analysis.d1.mx013_online_stop import (
    OnlineDecision, RecognizerParams, compute_topology_feature, replay_recognizer, sha256_file,
)
from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo

TAU_R = 0.03
K = 1
RUN_ID = "hpx_001"
DECISION_COUNT = 56
GT_ID = "hospital_structural_gt_v1"
GT_SHA256 = "7db2d4a2b5866e6854baa5b3e02ca3dd22ce205fcfcac5ec431e872c4724b9e9"
WORLD_SHA256 = "59d482029ad56dabf7ce9bb68ece16e476fd263c0ec57f237dff1f893880aeb0"
ROI_ID = "hospital_connected_free_v1"
ROI_DENOMINATOR = 215435
ROI_SHA256 = "05d45b7aba66cb6dbb71e0406005f4a3e21875901af3ae72164b17f1d3add8d1"
GT_SHAPE = (2123, 1504)
GT_ORIGIN = (-25.6, -60.1)
START_CELL = (1202, 512)
ONLINE_ARTIFACT_FIELDS = ("raw_map", "g1_map", "g2_map", "g3_map", "mean_map", "variance_map")
TRUTH_COLUMNS = {
    "oracle_stop_4", "oracle_remaining_fraction_h", "n_gt_h", "n_remaining_h",
    "a_true_remaining_h_m2", "persistent_suffix_qualifies", "hospital_oracle4_decision",
}


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def clean(value):
    if isinstance(value, (bool, np.bool_)):
        return "true" if value else "false"
    if value is None:
        return ""
    if isinstance(value, (float, np.floating)):
        return "" if math.isnan(float(value)) else format(float(value), ".17g")
    return value


def write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: clean(row.get(key)) for key in columns})


def atomic_json(path: Path, value: dict) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    tmp.replace(path)


def _safe_path(run_dir: Path, relative: str) -> Path:
    path = (run_dir / relative).resolve()
    if run_dir.resolve() not in path.parents:
        raise ValueError("artifact path escapes hpx_001")
    return path


def phase_a_schema() -> list[str]:
    base = list(OnlineDecision.__dataclass_fields__)
    return base + [
        "tau_r", "k", "r_condition", "topology_condition", "u_condition", "qualify",
        "streak_before", "streak_after", "reset_reason", "stop_consider", "first_fire",
    ]


def assert_phase_a_schema(columns: list[str]) -> None:
    overlap = set(columns) & TRUTH_COLUMNS
    if overlap:
        raise AssertionError(f"truth columns in Phase A: {sorted(overlap)}")


def build_online_decisions(run_dir: Path) -> tuple[list[OnlineDecision], dict[str, str]]:
    """Use only current-decision online artifacts and accepted MX013 primitives."""
    decisions = read_csv(run_dir / "decisions.csv")
    if len(decisions) != DECISION_COUNT:
        raise ValueError("hpx_001 must contain exactly 56 decisions")
    ids = [int(row["decision_id"]) for row in decisions]
    if ids != list(range(1, DECISION_COUNT + 1)):
        raise ValueError("hpx_001 decision identity/order mismatch")
    trajectory = topo.read_trajectory(run_dir / "trajectory.csv")
    hashes = {
        "decisions.csv": sha256_file(run_dir / "decisions.csv"),
        "trajectory.csv": sha256_file(run_dir / "trajectory.csv"),
        "metadata.json": sha256_file(run_dir / "metadata.json"),
    }
    result: list[OnlineDecision] = []
    for index, decision in enumerate(decisions, 1):
        for field in ONLINE_ARTIFACT_FIELDS:
            path = _safe_path(run_dir, decision[field])
            if not path.is_file():
                raise FileNotFoundError(path)
            hashes[decision[field]] = sha256_file(path)
        raw, members, variance, meta, _ = load_runtime(run_dir, decision)
        pose = topo.align_source(float(decision["time_s"]), trajectory)
        source = None
        if pose["available"]:
            source = topo.world_to_cell(pose["x"], pose["y"], meta["origin_x"], meta["origin_y"], meta["resolution"])
        fields = runtime_fields(raw, members, variance, meta["resolution"], source, topo)
        if not pose["available"]:
            fields["d1_reason"] = pose["mode"]
        row = dict(decision)
        row.update(
            run_id=RUN_ID, decision_index=index, decision_time_s=float(decision["time_s"]),
            RemainingFraction=fields["RemainingFraction"],
            d1_source_available=fields["d1_source_available"], d1_reason=fields["d1_reason"],
            d1_source_x=pose["x"], d1_source_y=pose["y"],
            d1_source_row=source[0] if source else math.nan, d1_source_col=source[1] if source else math.nan,
            d1_source_mode=pose["mode"], d1_source_lower_time_s=pose["lower_time_s"],
            d1_source_upper_time_s=pose["upper_time_s"],
            raw_map_sha256=hashes[decision["raw_map"]], mean_map_sha256=hashes[decision["mean_map"]],
            g1_map_sha256=hashes[decision["g1_map"]], g2_map_sha256=hashes[decision["g2_map"]],
            g3_map_sha256=hashes[decision["g3_map"]],
        )
        feature = compute_topology_feature(row, run_dir)
        r_hat = float(fields["RemainingFraction"])
        result.append(OnlineDecision(
            RUN_ID, int(decision["decision_id"]), index, float(decision["time_s"]),
            (index - 1) / (DECISION_COUNT - 1), r_hat,
            bool(fields["d1_runtime_region_evaluable"]) and math.isfinite(r_hat), str(fields["d1_reason"] or ""),
            **feature, u_p95=float(fields["U_p95"]), u_mean=float(fields["U_mean"]),
            u_disagreement=float(fields["U_disagreement"]), low_u_25=None,
            raw_map_sha256=hashes[decision["raw_map"]],
            prediction_member_sha256s="|".join(hashes[decision[f"g{i}_map"]] for i in (1, 2, 3)),
            mean_map_sha256=hashes[decision["mean_map"]], source_mode=str(pose["mode"]),
            source_lower_time_s=float(pose["lower_time_s"]), source_upper_time_s=float(pose["upper_time_s"]),
        ))
    return result, dict(sorted(hashes.items()))


def run_phase_a(run_dir: Path, output: Path, recognizer_path: Path, adapter_path: Path) -> dict:
    output.mkdir(parents=True, exist_ok=True)
    schema = phase_a_schema()
    assert_phase_a_schema(schema)
    decisions, hashes = build_online_decisions(run_dir)
    trace = replay_recognizer(decisions, RecognizerParams(TAU_R, K, use_topology=True, use_u_veto=False))
    trace_path = output / "hospital_phase_a_trace.csv"
    write_csv(trace_path, trace, schema)
    fires = [row for row in trace if row["first_fire"]]
    stop = int(fires[0]["decision_index"]) if fires else None
    stop_time = float(fires[0]["decision_time_s"]) if fires else None
    manifest = {
        "task": "MX016", "run_id": RUN_ID, "phase": "A_SEALED", "sealed": True,
        "recognizer_revision": "mapex-mx013-online-stop-replay-v1@20edfd1527c63ade813203af04f4a773313f65c9",
        "recognizer_blob": "8c4d993be1218d5273c4f6c3b57483dae78d5e0b",
        "recognizer_sha256": sha256_file(recognizer_path), "adapter_sha256": sha256_file(adapter_path),
        "configuration": {"tau_r": TAU_R, "k": K, "topology_gate_enabled": True, "u_gating": False},
        "candidate_stop_decision": stop, "candidate_stop_time_s": stop_time,
        "final_decision_time_s": float(trace[-1]["decision_time_s"]),
        "candidate_stop": "NO_STOP" if stop is None else f"STOP_CONSIDER:{stop}",
        "decision_count": len(trace), "online_input_hashes": hashes,
        "phase_a_trace_sha256": sha256_file(trace_path),
        "truth_firewall": {"truth_loaded": False, "future_map_loaded": False, "evaluation_csv_loaded": False},
    }
    atomic_json(output / "hospital_phase_a_manifest.json", manifest)
    return manifest


def _gt_scalar(z, key):
    return np.asarray(z[key]).reshape(()).item()


def _check_gt_internal(z) -> tuple[bool, list[str]]:
    reasons = []
    required = {"data", "evaluation_mask", "resolution", "origin_x", "origin_y", "source_world_sha256"}
    missing = required - set(z.files)
    if missing:
        reasons.append("missing_keys:" + ",".join(sorted(missing)))
        return False, reasons
    data = np.asarray(z["data"]); mask = np.asarray(z["evaluation_mask"])
    if data.shape != GT_SHAPE or mask.shape != GT_SHAPE or mask.dtype != np.bool_:
        reasons.append("invalid_shape_or_mask")
    if not set(np.unique(data)).issubset({-1, 0, 100}):
        reasons.append("invalid_structural_classes")
    vals = [float(_gt_scalar(z, key)) for key in ("resolution", "origin_x", "origin_y")]
    if not all(math.isfinite(x) for x in vals) or vals[0] <= 0:
        reasons.append("invalid_geometry_scalars")
    if not math.isclose(vals[0], .05, rel_tol=0, abs_tol=1e-6) or not math.isclose(vals[1], GT_ORIGIN[0], abs_tol=1e-9) or not math.isclose(vals[2], GT_ORIGIN[1], abs_tol=1e-9):
        reasons.append("canvas_geometry_mismatch")
    return not reasons, reasons


def _project_known(raw_path: Path, z, universe: np.ndarray) -> dict:
    with np.load(raw_path, allow_pickle=False) as raw_z:
        raw = np.asarray(raw_z["data"]); rr = float(_gt_scalar(raw_z, "resolution"))
        ox = float(_gt_scalar(raw_z, "origin_x")); oy = float(_gt_scalar(raw_z, "origin_y"))
        yaw = float(_gt_scalar(raw_z, "origin_yaw"))
    gr = float(_gt_scalar(z, "resolution")); gx = float(_gt_scalar(z, "origin_x")); gy = float(_gt_scalar(z, "origin_y"))
    ratio_f = rr / gr; ratio = int(round(ratio_f))
    row_f = (oy - gy) / gr; col_f = (ox - gx) / gr
    row0 = round(row_f); col0 = round(col_f)
    residual_x = ox - (gx + col0 * gr); residual_y = oy - (gy + row0 * gr)
    reasons = []
    if abs(yaw) > 1e-6: reasons.append("ORIGIN_YAW_NOT_AXIS_ALIGNED")
    if ratio < 1 or not math.isclose(ratio_f, ratio, rel_tol=0, abs_tol=1e-6) or ratio != 2: reasons.append("RUNTIME_GT_RATIO_INVALID")
    if abs(residual_x) > .025 + 1e-6 or abs(residual_y) > .025 + 1e-6: reasons.append("HOSPITAL_ORIGIN_ALIGNMENT_RESIDUAL_EXCEEDED")
    expanded = np.repeat(np.repeat(raw >= 0, ratio, axis=0), ratio, axis=1) if not reasons else np.zeros((0, 0), bool)
    r0=max(0,row0); c0=max(0,col0); r1=min(universe.shape[0],row0+expanded.shape[0]); c1=min(universe.shape[1],col0+expanded.shape[1])
    if r1 <= r0 or c1 <= c0: reasons.append("NO_POSITIVE_GT_OVERLAP")
    known = np.zeros(universe.shape, bool)
    if not reasons:
        known[r0:r1,c0:c1] = expanded[r0-row0:r1-row0,c0-col0:c1-col0]
    return {"evaluable": not reasons, "reason": "|".join(reasons), "known": known,
            "runtime_resolution": rr, "origin_yaw": yaw, "ratio": ratio,
            "residual_x_m": residual_x, "residual_y_m": residual_y}


def invalid_result(manifest: dict, validity: dict) -> dict:
    stop = manifest["candidate_stop_decision"]
    stop_time = manifest.get("candidate_stop_time_s")
    final_time = manifest.get("final_decision_time_s")
    return {
        "task": "MX016", "run_id": RUN_ID, "candidate_stop_decision": stop,
        "candidate_stop": "NO_STOP" if stop is None else f"STOP_CONSIDER:{stop}",
        "candidate_stop_time_s": stop_time,
        "hospital_oracle4_status": "HOSPITAL_ORACLE_INVALID_FOR_TRANSFER_EVAL",
        "hospital_oracle4_decision": None, "timing_status": None, "delay_decisions": None,
        "delay_seconds": None, "decisions_saved": 0 if stop is None else DECISION_COUNT-stop,
        "time_saved_s": 0.0 if stop is None else float(final_time)-float(stop_time),
        "saved_progress": 0.0 if stop is None else 1-stop/DECISION_COUNT,
        "true_remaining_at_stop": None, "early_stop_consequence": None,
        "retrospective_topology": {"evaluable": False, "reason": "ORACLE_INVALID_TRUTH_UNAVAILABLE"},
        "transfer_label": "TRANSFER_SANITY_INCONCLUSIVE_ORACLE_INVALID",
        "phase_a_trace_sha256": manifest["phase_a_trace_sha256"],
        "oracle_invalid_reasons": validity["fatal_failures"],
        "guardrail": "N1_NOT_CONFIRMATION_NOT_DEPLOYMENT; MX013_REMAINS_NO_STABLE_ONLINE_RECOGNIZER_CANDIDATE",
    }


def run_phase_b(run_dir: Path, output: Path, gt_path: Path, roi_path: Path | None = None) -> dict:
    manifest_path = output / "hospital_phase_a_manifest.json"; trace_path = output / "hospital_phase_a_trace.csv"
    if not manifest_path.is_file() or not trace_path.is_file():
        raise RuntimeError("sealed Phase A required before Phase B")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if not manifest.get("sealed") or manifest.get("phase") != "A_SEALED" or sha256_file(trace_path) != manifest.get("phase_a_trace_sha256"):
        raise RuntimeError("Phase A seal/hash invalid")
    metadata = json.loads((run_dir / "metadata.json").read_text()); evaluation = json.loads((run_dir / "evaluation.json").read_text())
    decisions = read_csv(run_dir / "decisions.csv")
    checks = {f"G{i}": {"status": "FAIL", "reasons": []} for i in range(1, 9)}
    g1_ok = metadata.get("run_id") == RUN_ID and metadata.get("environment") == "hospital" and len(decisions) == DECISION_COUNT and [int(x["decision_id"]) for x in decisions] == list(range(1,57))
    checks["G1"] = {"status": "PASS" if g1_ok else "FAIL", "reasons": [] if g1_ok else ["run_identity_or_decision_inventory"]}
    gt_exists = gt_path.is_file()
    gt_sha = sha256_file(gt_path) if gt_exists else None
    g2_ok = gt_exists and evaluation.get("ground_truth_sha256") == GT_SHA256 and gt_sha == GT_SHA256 and metadata.get("structural_ground_truth_id") == GT_ID
    checks["G2"] = {"status": "PASS" if g2_ok else "FAIL", "reasons": [] if g2_ok else ["ORACLE_INVALID_GT_IDENTITY"]}
    oracle_rows: list[dict] = []
    if g2_ok:
        with np.load(gt_path, allow_pickle=False) as z:
            world = str(_gt_scalar(z, "source_world_sha256")) if "source_world_sha256" in z.files else None
            recorded_world = metadata.get("config_sha256", {}).get("hospital_world")
            g3_ok = world == WORLD_SHA256 and recorded_world == WORLD_SHA256
            checks["G3"] = {"status": "PASS" if g3_ok else "FAIL", "reasons": [] if g3_ok else ["ORACLE_INVALID_WORLD_VERSION"]}
            g4_ok, g4_reasons = _check_gt_internal(z)
            checks["G4"] = {"status": "PASS" if g4_ok else "FAIL", "reasons": g4_reasons}
            raw_res=[]; numeric_reasons=[]
            for row in decisions:
                with np.load(_safe_path(run_dir,row["raw_map"]),allow_pickle=False) as rz:
                    raw_res.append(float(_gt_scalar(rz,"resolution")))
            r0=raw_res[0]
            if not all(np.isclose(x,r0,rtol=0,atol=1e-6) for x in raw_res): numeric_reasons.append("runtime_resolution_inconsistent")
            if not math.isclose(float(metadata["runtime_map_resolution_m"]),r0,rel_tol=0,abs_tol=1e-6): numeric_reasons.append("metadata_runtime_resolution_mismatch")
            if not math.isclose(float(metadata["fixed_canvas_resolution_m"]),float(_gt_scalar(z,"resolution")),rel_tol=0,abs_tol=1e-6): numeric_reasons.append("canvas_resolution_mismatch")
            checks["G5"]={"status":"PASS" if not numeric_reasons else "FAIL","reasons":numeric_reasons}
            universe=np.zeros(GT_SHAPE,bool); g6_reasons=[]
            if g4_ok:
                structural=(np.asarray(z["data"])==0)&np.asarray(z["evaluation_mask"],bool)
                safe=topo.cspace(structural,np.asarray(z["evaluation_mask"],bool),topo.collision_stencil(radius=.189,resolution=float(_gt_scalar(z,"resolution"))))
                if not structural[START_CELL]: g6_reasons.append("start_not_structural_free")
                if not safe[START_CELL]: g6_reasons.append("start_not_footprint_safe")
                universe=topo.reachable(safe,START_CELL)
                if not universe.any(): g6_reasons.append("empty_hospital_universe")
            else: g6_reasons.append("gt_internal_contract_unavailable")
            checks["G6"]={"status":"PASS" if not g6_reasons else "FAIL","reasons":g6_reasons}
            projection_reasons=[]; n_gt=int(universe.sum())
            if not g6_reasons:
                for index,row in enumerate(decisions,1):
                    projection=_project_known(_safe_path(run_dir,row["raw_map"]),z,universe)
                    n_remaining=int((universe & ~projection["known"]).sum()) if projection["evaluable"] else None
                    fraction=n_remaining/n_gt if n_remaining is not None else None
                    if not projection["evaluable"]: projection_reasons.append(f"decision_{index}:{projection['reason']}")
                    oracle_rows.append({"decision_id":int(row["decision_id"]),"decision_index":index,"decision_time_s":float(row["time_s"]),
                        "truth_evaluable":projection["evaluable"],"truth_reason":projection["reason"],"n_gt_h":n_gt,
                        "n_remaining_h":n_remaining,"a_true_remaining_h_m2":None if n_remaining is None else n_remaining*.05**2,
                        "oracle_remaining_fraction_h":fraction,"qualifies_4":bool(fraction is not None and fraction<=.04),
                        "persistent_suffix_qualifies":False,"origin_residual_x_m":projection["residual_x_m"],"origin_residual_y_m":projection["residual_y_m"]})
            checks["G7"]={"status":"PASS" if oracle_rows and not projection_reasons else "FAIL","reasons":projection_reasons or ([] if oracle_rows else ["projection_unavailable"])}
    else:
        for key, reason in (("G3","GT_UNAVAILABLE"),("G4","GT_UNAVAILABLE"),("G5","GT_UNAVAILABLE"),("G6","GT_UNAVAILABLE"),("G7","GT_UNAVAILABLE")):
            checks[key] = {"status":"FAIL","reasons":[reason]}
    roi_match = bool(roi_path and roi_path.is_file() and sha256_file(roi_path)==ROI_SHA256 and int(np.load(roi_path,allow_pickle=False).sum())==ROI_DENOMINATOR)
    checks["G8"]={"status":"PASS","fatal":False,"audit_label":"LEGACY_ROI_V1_MATCH" if roi_match else "LEGACY_ROI_V1_GEOMETRY_DIFF_WARNING",
                  "recorded_id":metadata.get("evaluation_roi_id"),"recorded_denominator":metadata.get("evaluation_roi_denominator"),"frozen_sha256":ROI_SHA256,
                  "readme_warning":"ROI v1 predates elevator-blocker geometry change; audit is non-fatal and ROI is excluded from Oracle formula."}
    fatal=[key for key in ("G1","G2","G3","G4","G5","G6","G7") if checks[key]["status"]!="PASS"]
    validity={"task":"MX016","run_id":RUN_ID,"oracle_gate":"HOSPITAL_ORACLE_VALID" if not fatal else "HOSPITAL_ORACLE_INVALID_FOR_TRANSFER_EVAL",
              "checks":checks,"fatal_failures":fatal,"gt_path":str(gt_path),"gt_sha256":gt_sha,"expected_gt_sha256":GT_SHA256,
              "world_sha256_recorded":metadata.get("config_sha256",{}).get("hospital_world"),"legacy_roi_is_nonfatal":True}
    atomic_json(output/"hospital_oracle_validity.json",validity)
    oracle_columns=["decision_id","decision_index","decision_time_s","truth_evaluable","truth_reason","n_gt_h","n_remaining_h","a_true_remaining_h_m2","oracle_remaining_fraction_h","qualifies_4","persistent_suffix_qualifies","origin_residual_x_m","origin_residual_y_m"]
    write_csv(output/"hospital_oracle_trace.csv",oracle_rows,oracle_columns)
    if fatal:
        result=invalid_result(manifest,validity); atomic_json(output/"hospital_transfer_result.json",result); return result
    suffix=True
    for row in reversed(oracle_rows):
        suffix=suffix and row["truth_evaluable"] and row["qualifies_4"]; row["persistent_suffix_qualifies"]=suffix
    write_csv(output/"hospital_oracle_trace.csv",oracle_rows,oracle_columns)
    found=next((r for r in oracle_rows if r["persistent_suffix_qualifies"]),None)
    stop=manifest["candidate_stop_decision"]
    if found is None:
        result=invalid_result(manifest,validity); result.update(hospital_oracle4_status="HOSPITAL_ORACLE4_NO_ACCEPTABLE_STOP",transfer_label="TRANSFER_SANITY_FAILURE_N1")
    else:
        fire=oracle_rows[stop-1] if stop else None; delay=None if stop is None else stop-found["decision_index"]
        timing="NO_STOP" if stop is None else ("EARLY" if delay<0 else ("ON_TARGET" if delay==0 else "LATE"))
        severe=bool(fire and fire["oracle_remaining_fraction_h"]>.10)
        saved=0 if stop is None else DECISION_COUNT-stop
        label=("TRANSFER_SANITY_FAILURE_N1" if stop is None or delay < -3 or delay > 3 or severe or saved<=0 else
               "TRANSFER_SANITY_NEAR_MISS_N1" if delay<0 else "TRANSFER_SANITY_SUPPORTIVE_N1")
        final_time=float(oracle_rows[-1]["decision_time_s"]); fire_time=float(fire["decision_time_s"]) if fire else None
        result={"task":"MX016","run_id":RUN_ID,"candidate_stop_decision":stop,"candidate_stop":"NO_STOP" if stop is None else f"STOP_CONSIDER:{stop}",
            "hospital_oracle4_status":"HOSPITAL_ORACLE_VALID","hospital_oracle4_decision":found["decision_index"],"timing_status":timing,"delay_decisions":delay,
            "delay_seconds":None if fire is None else fire_time-float(found["decision_time_s"]),"decisions_saved":saved,
            "time_saved_s":0 if fire is None else final_time-fire_time,"saved_progress":0 if fire is None else 1-stop/DECISION_COUNT,
            "true_remaining_at_stop":None if fire is None else {"cells":fire["n_remaining_h"],"area_m2":fire["a_true_remaining_h_m2"],"fraction":fire["oracle_remaining_fraction_h"],
                "budget4_area_m2":.04*fire["n_gt_h"]*.05**2,"excess_fraction":max(0,fire["oracle_remaining_fraction_h"]-.04),"severe_false_stop_10":severe},
            "early_stop_consequence":None,"retrospective_topology":{"evaluable":False,"reason":"NOT_COMPUTED_EXISTING_ACCEPTED_HOSPITAL_FUTURE_REFERENCE_UNAVAILABLE"},
            "transfer_label":label,"phase_a_trace_sha256":manifest["phase_a_trace_sha256"],
            "guardrail":"N1_NOT_CONFIRMATION_NOT_DEPLOYMENT; MX013_REMAINS_NO_STABLE_ONLINE_RECOGNIZER_CANDIDATE"}
    atomic_json(output/"hospital_transfer_result.json",result); return result


def write_artifact_manifest(output: Path) -> dict:
    artifacts=[]
    for path in sorted(output.iterdir()):
        if path.is_file() and path.name != "artifact_manifest.json":
            artifacts.append({"file":path.name,"bytes":path.stat().st_size,"sha256":sha256_file(path)})
    value={"task":"MX016","artifacts":artifacts}; atomic_json(output/"artifact_manifest.json",value); return value
