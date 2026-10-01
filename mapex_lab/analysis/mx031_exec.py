#!/usr/bin/env python3
"""MX031 deterministic offline replay for Analyst04.

Implements exact accepted Method V3:
c0130eb8bdbdba2a2b144c2d62e16e1bea00ef46

New Room development only. No model inference, simulation, Hospital scoring,
post-hoc retuning, P/U recognizer thresholds, composite score, or deployment.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from collections import Counter, defaultdict

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo


SCRIPT = Path(__file__).resolve()
ANALYSIS = SCRIPT.parent
MAPEX_LAB = ANALYSIS.parent
REPO = MAPEX_LAB.parent
RESULTS = ANALYSIS / "mx031_exec_results"
FIGURES = RESULTS / "figures"
EXPERIMENTS = MAPEX_LAB / "experiments" / "mapex"

METHOD = "c0130eb8bdbdba2a2b144c2d62e16e1bea00ef46"
METHOD_QA = "2d4590e0474be9c004704dcac21c8c4d34c35ec1"
FROZEN_BASE = "4899ee95c85640965befeaf98c20f156c0d3181a"
MX026_RESULT = "e4a09c423107f708610f18931c9ae947a020e34c"
MX027_RESULT = "5ae879c95607fb62accef6ad49c31a36be9dffac"
W032_RESULT = "4f674a81add0c14eb15cc0ab95754130c5923ead"
MX025_COMPANY_ACCEPT = "c9b0b3fad47839a4e0ed6c29cb4b17d50965b78e"

R_PATH = ANALYSIS / "mx026_exec_results" / "MX026_PUR_PER_DECISION.csv"
R_MANIFEST_PATH = ANALYSIS / "mx026_exec_results" / "MX026_PUR_ARTIFACT_MANIFEST.json"
R_SOURCE_PATH = ANALYSIS / "mx025_exec_results" / "MX025_R_MAP_SOURCE_PARITY.csv"
COV_PATH = ANALYSIS / "mx027_exec_results" / "MX027_IG_COVERAGE_PER_DECISION.csv"
COV_SOURCE_PATH = ANALYSIS / "mx027_exec_results" / "MX027_IG_COVERAGE_SOURCE_PARITY.csv"
ORACLE_PATH = ANALYSIS / "d1" / "results" / "oracle_stop_retro_v1" / "oracle_decisions.csv"
ORACLE_MANIFEST_PATH = ANALYSIS / "d1" / "results" / "oracle_stop_retro_v1" / "artifact_manifest.json"
TOPO_PATH = ANALYSIS / "r004" / "evaluate_topology_traversability.py"
SHARED_PATH = ANALYSIS / "d1" / "extract_shared_phase0_evidence.py"

EXPECTED_BLOBS = {
    R_PATH: "8e73f4441b1c852f90d801c20f993216bbd1e78a",
    R_MANIFEST_PATH: "193be0531b9d5f43f57bd3d76e41527063e0a9d2",
    R_SOURCE_PATH: "0a3af2267f37805e7fc7e1964d89f68d3d49013a",
    COV_PATH: "dab247d44578d728c44ffc2245ee306d7375d9cb",
    COV_SOURCE_PATH: "629560dba455a91c95f2f636d4d532016fd5a4c1",
    ORACLE_PATH: "9ae8f0479cf37a2b6aa9426a158f120efdc655cf",
    TOPO_PATH: "ff031cb826908aafd3bc039ca67a499ed092cbc5",
    SHARED_PATH: "c48de3322c4dcaefa63b07f8469cdfec2d740f1e",
}
ORACLE_SHA256 = "2ca31f97ff4abfeb305667539dd0616af0caabfc5b11743c9bd8347e2f949cf8"

RUNS = tuple(f"mpx_{i:03d}" for i in range(1, 11))
ORACLE4 = {
    "mpx_001": 21, "mpx_002": 18, "mpx_003": 21, "mpx_004": 16,
    "mpx_005": 16, "mpx_006": 20, "mpx_007": 18, "mpx_008": 19,
    "mpx_009": 18, "mpx_010": 19,
}
FAMILY_ORDER = {"F0": 0, "F1": 1, "F2": 2, "F3": 3}
COMPLEXITY = {"F0": 0, "F1": 1, "F2": 1, "F3": 2}
HYPERPARAMS = {"F0": 2, "F1": 3, "F2": 2, "F3": 3}
PRIMARY_TAUS = tuple(float(i) for i in range(1, 16))
SENS_A_TAUS = tuple(0.5 * i for i in range(1, 31))
SENS_B_TAUS = tuple(0.5 * i for i in range(1, 41))
KS = (1, 2, 3, 4)
CS = (0, 1, 2)


def run_cmd(*args: str) -> str:
    return subprocess.check_output(args, cwd=REPO, text=True).strip()


def git_blob(path: Path) -> str:
    return run_cmd("git", "hash-object", str(path))


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def index_rows(rows: list[dict[str, str]], label: str) -> dict[tuple[str, int], dict[str, str]]:
    out = {}
    for row in rows:
        key = (row["run_id"], int(row["decision_id"]))
        if key in out:
            raise RuntimeError(f"{label}: duplicate key {key}")
        out[key] = row
    return out


def is_blank(v) -> bool:
    return v is None or str(v).strip() == ""


def finite(v) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def fval(v, default=math.nan) -> float:
    return float(v) if finite(v) else default


def ival(v, default=None):
    try:
        if is_blank(v):
            return default
        x = float(v)
        if not math.isfinite(x) or int(x) != x:
            return default
        return int(x)
    except (TypeError, ValueError):
        return default


def bval(v) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes"}


def med(vals) -> float:
    a = np.asarray([float(x) for x in vals if finite(x)], dtype=np.float64)
    return float(np.median(a)) if a.size else math.nan


def mean(vals) -> float:
    a = np.asarray([float(x) for x in vals if finite(x)], dtype=np.float64)
    return float(np.mean(a)) if a.size else math.nan


def clean(v):
    if v is None:
        return ""
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.bool_):
        return int(bool(v))
    if isinstance(v, (np.floating, float)):
        x = float(v)
        return x if math.isfinite(x) else ""
    if isinstance(v, bool):
        return int(v)
    return v


def write_csv(path: Path, rows: list[dict]):
    keys, seen = [], set()
    for row in rows:
        for k in row:
            if k not in seen:
                seen.add(k)
                keys.append(k)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for row in rows:
            w.writerow({k: clean(row.get(k, "")) for k in keys})


def json_clean(v):
    if isinstance(v, dict):
        return {str(k): json_clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [json_clean(x) for x in v]
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, np.bool_):
        return bool(v)
    if isinstance(v, np.floating):
        v = float(v)
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    return v


def write_json(path: Path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_clean(obj), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def scalar(z, key):
    a = np.asarray(z[key])
    return a.reshape(()).item() if a.size == 1 else a


def local_path(run_dir: Path, rel: str) -> Path:
    p = (run_dir / rel).resolve()
    if run_dir.resolve() not in p.parents:
        raise RuntimeError(f"artifact escapes run dir: {rel}")
    return p


def source_guard():
    for path, expected in EXPECTED_BLOBS.items():
        actual = git_blob(path)
        if actual != expected:
            raise RuntimeError(f"accepted blob mismatch {path}: {actual} != {expected}")
    if sha256_file(ORACLE_PATH) != ORACLE_SHA256:
        raise RuntimeError("W032 oracle_decisions sha256 mismatch")
    oracle_manifest = json.loads(ORACLE_MANIFEST_PATH.read_text(encoding="utf-8"))
    if oracle_manifest["artifacts"]["oracle_decisions.csv"] != ORACLE_SHA256:
        raise RuntimeError("W032 artifact manifest oracle sha256 mismatch")
    rmanifest = json.loads(R_MANIFEST_PATH.read_text(encoding="utf-8"))
    if rmanifest.get("accepted_input_revisions", {}).get("R") != MX025_COMPANY_ACCEPT:
        raise RuntimeError("MX026 manifest accepted R revision mismatch")

    allowed = {
        ".github/workflows/mx031-preflight.yml",
        ".github/workflows/mx031-oneoff.yml",
        "mapex_lab/analysis/mx031_exec.py",
        str(R_PATH.relative_to(REPO)),
        str(R_MANIFEST_PATH.relative_to(REPO)),
        str(R_SOURCE_PATH.relative_to(REPO)),
        str(COV_PATH.relative_to(REPO)),
        str(COV_SOURCE_PATH.relative_to(REPO)),
    }
    changed = set(run_cmd("git", "diff", "--name-only", FROZEN_BASE, "HEAD").splitlines())
    bad = sorted(
        p for p in changed
        if p not in allowed and not p.startswith("mapex_lab/analysis/mx031_exec_results/")
    )
    if bad:
        raise RuntimeError(f"unexpected drift from frozen base: {bad}")
    return sorted(changed)


def load_prediction(path: Path, raw_meta: dict, expected_member: str):
    with np.load(path, allow_pickle=False) as z:
        required = {
            "data", "source_height", "source_width", "pad_top", "pad_left",
            "resolution", "origin_x", "origin_y", "source_map_stamp_s", "member",
        }
        if not required.issubset(z.files):
            raise RuntimeError(f"{path}: missing prediction metadata")
        a = np.asarray(z["data"], dtype=np.float64)
        h = int(scalar(z, "source_height"))
        w = int(scalar(z, "source_width"))
        top = int(scalar(z, "pad_top"))
        left = int(scalar(z, "pad_left"))
        if (h, w) != raw_meta["shape"] or top < 0 or left < 0:
            raise RuntimeError(f"{path}: crop/source shape mismatch")
        if top + h > a.shape[0] or left + w > a.shape[1]:
            raise RuntimeError(f"{path}: invalid crop")
        if str(scalar(z, "member")) != expected_member:
            raise RuntimeError(f"{path}: wrong member")
        for field in ("resolution", "origin_x", "origin_y"):
            if not math.isclose(float(scalar(z, field)), float(raw_meta[field]), rel_tol=0.0, abs_tol=1e-12):
                raise RuntimeError(f"{path}: {field} mismatch")
        if not math.isclose(float(scalar(z, "source_map_stamp_s")), float(raw_meta["source_stamp_s"]), rel_tol=0.0, abs_tol=1e-12):
            raise RuntimeError(f"{path}: source stamp mismatch")
        crop = a[top:top+h, left:left+w]
        if crop.shape != raw_meta["shape"] or not np.all(np.isfinite(crop)):
            raise RuntimeError(f"{path}: invalid finite runtime crop")
        if expected_member != "variance" and (np.any(crop < 0) or np.any(crop > 1)):
            raise RuntimeError(f"{path}: probability outside [0,1]")
        return crop, {
            "source_height": h, "source_width": w, "pad_top": top, "pad_left": left,
            "padded_shape": list(a.shape),
        }


def load_runtime_for_topology(run_dir: Path, decision: dict, rsrc: dict):
    raw_path = local_path(run_dir, decision["raw_map"])
    with np.load(raw_path, allow_pickle=False) as z:
        raw = np.asarray(z["data"])
        meta = {
            "resolution": float(scalar(z, "resolution")),
            "origin_x": float(scalar(z, "origin_x")),
            "origin_y": float(scalar(z, "origin_y")),
            "origin_yaw": float(scalar(z, "origin_yaw")),
            "source_stamp_s": float(scalar(z, "source_stamp_s")),
            "frame_id": str(scalar(z, "frame_id")),
            "height": int(scalar(z, "height")),
            "width": int(scalar(z, "width")),
        }
    meta["shape"] = tuple(raw.shape)
    if raw.ndim != 2 or meta["shape"] != (meta["height"], meta["width"]):
        raise RuntimeError("raw map shape metadata mismatch")
    if meta["resolution"] <= 0 or meta["origin_yaw"] != 0.0 or meta["frame_id"] != "map":
        raise RuntimeError("raw grid geometry/frame mismatch")
    if not np.all(np.isfinite(raw)):
        raise RuntimeError("raw map contains nonfinite values")

    identities = {
        "raw": decision["raw_map"],
        "g1": decision["g1_map"],
        "g2": decision["g2_map"],
        "g3": decision["g3_map"],
        "mean": decision["mean_map"],
    }
    expected_names = {
        "raw": f"decision_{int(decision['decision_id']):06d}_raw.npz",
        "g1": f"decision_{int(decision['decision_id']):06d}_g1.npz",
        "g2": f"decision_{int(decision['decision_id']):06d}_g2.npz",
        "g3": f"decision_{int(decision['decision_id']):06d}_g3.npz",
        "mean": f"decision_{int(decision['decision_id']):06d}_mean.npz",
    }
    if any(Path(identities[k]).name != expected_names[k] for k in expected_names):
        raise RuntimeError("PROVENANCE_MISMATCH: decision artifact filename identity")

    # Exact accepted map-only R source identity for raw/G1/G2/G3.
    parity_identity_fields = {
        "raw": ("raw_identity", "raw_sha256"),
        "g1": ("g1_identity", "g1_sha256"),
        "g2": ("g2_identity", "g2_sha256"),
        "g3": ("g3_identity", "g3_sha256"),
    }
    for k, (identity_field, hash_field) in parity_identity_fields.items():
        if identities[k] != rsrc[identity_field]:
            raise RuntimeError(f"PROVENANCE_MISMATCH: {k} identity")
        if sha256_file(local_path(run_dir, identities[k])) != rsrc[hash_field]:
            raise RuntimeError(f"PROVENANCE_MISMATCH: {k} sha256")
    if ival(rsrc["alignment_pass"], 0) != 1 or not is_blank(rsrc["reason"]):
        raise RuntimeError("PROVENANCE_MISMATCH: accepted R source alignment")
    if not math.isclose(meta["resolution"], float(rsrc["resolution"]), rel_tol=0.0, abs_tol=1e-12):
        raise RuntimeError("PROVENANCE_MISMATCH: raw resolution")
    if not math.isclose(meta["origin_x"], float(rsrc["origin_x"]), rel_tol=0.0, abs_tol=1e-12):
        raise RuntimeError("PROVENANCE_MISMATCH: origin_x")
    if not math.isclose(meta["origin_y"], float(rsrc["origin_y"]), rel_tol=0.0, abs_tol=1e-12):
        raise RuntimeError("PROVENANCE_MISMATCH: origin_y")
    if not math.isclose(meta["source_stamp_s"], float(rsrc["source_stamp_s"]), rel_tol=0.0, abs_tol=1e-12):
        raise RuntimeError("PROVENANCE_MISMATCH: source stamp")

    g1, c1 = load_prediction(local_path(run_dir, identities["g1"]), meta, "G1")
    g2, c2 = load_prediction(local_path(run_dir, identities["g2"]), meta, "G2")
    g3, c3 = load_prediction(local_path(run_dir, identities["g3"]), meta, "G3")
    mean_map, cm = load_prediction(local_path(run_dir, identities["mean"]), meta, "mean")

    accepted_crop = {
        "source_height": int(float(rsrc["source_height"])),
        "source_width": int(float(rsrc["source_width"])),
        "pad_top": int(float(rsrc["pad_top"])),
        "pad_left": int(float(rsrc["pad_left"])),
    }
    for crop in (c1, c2, c3, cm):
        for k, v in accepted_crop.items():
            if int(crop[k]) != int(v):
                raise RuntimeError(f"PROVENANCE_MISMATCH: crop {k}")

    # Every runtime cell in the validated source crop is backed by the mean artifact.
    support = np.ones(raw.shape, dtype=bool)
    return raw, mean_map, support, meta, cm


def topo_valid_for_decision(run_dir: Path, decision: dict, rrow: dict, rsrc: dict, trajectory):
    if str(rrow["R_map_evaluable"]).strip() != "1" or not finite(rrow["R_MapRemainingFraction"]):
        return False, "R_MAP_UNAVAILABLE", 0, "", "", "", ""

    try:
        raw, mean_map, support, meta, crop = load_runtime_for_topology(run_dir, decision, rsrc)
    except Exception as exc:
        msg = str(exc)
        if "PROVENANCE_MISMATCH" in msg:
            raise
        raise RuntimeError(f"PROVENANCE_MISMATCH: runtime input validation: {msg}") from exc

    pose = topo.align_source(float(decision["time_s"]), trajectory)
    if not pose["available"]:
        return False, str(pose["mode"]), 0, pose["mode"], "", "", json.dumps(crop, sort_keys=True)

    source = topo.world_to_cell(
        pose["x"], pose["y"], meta["origin_x"], meta["origin_y"], meta["resolution"]
    )
    r, c = source
    if not (0 <= r < raw.shape[0] and 0 <= c < raw.shape[1]):
        return False, "SOURCE_OUTSIDE_RUNTIME_GRID", 0, pose["mode"], r, c, json.dumps(crop, sort_keys=True)

    known = raw >= 0
    supported_unknown = (raw < 0) & support
    online_domain = known | supported_unknown
    online_free = (raw == 0) | (supported_unknown & (mean_map <= 0.5))

    if not online_domain[source]:
        return False, "SOURCE_OUTSIDE_ONLINE_DOMAIN", 0, pose["mode"], r, c, json.dumps(crop, sort_keys=True)

    stencil = topo.collision_stencil(radius=0.189, resolution=meta["resolution"])
    pred_trav = topo.cspace(online_free, online_domain, stencil)
    if not pred_trav[source]:
        return False, "SOURCE_NOT_PREDICTED_TRAVERSABLE", 0, pose["mode"], r, c, json.dumps(crop, sort_keys=True)

    reach = topo.reachable(pred_trav, source)
    n = int(np.count_nonzero(reach))
    if n <= 0:
        return False, "EMPTY_SOURCE_REACHABLE_COMPONENT", 0, pose["mode"], r, c, json.dumps(crop, sort_keys=True)
    return True, "", n, pose["mode"], r, c, json.dumps(crop, sort_keys=True)


def frontier_state(per: dict, src: dict):
    policy_raw = str(src.get("policy_raw", "")).strip()
    outcome = str(src.get("outcome", "")).strip()
    candidate_total = ival(src.get("candidate_total"))
    lookup = ival(src.get("candidate_lookup_attempted"))
    src_reason = str(src.get("IG_reason", "")).strip()
    per_eval = ival(per.get("IG_evaluable"))
    per_reason = str(per.get("IG_reason", "")).strip()

    state_a = (
        policy_raw == "" and outcome == "no_selection" and candidate_total == 0
        and lookup == 0 and src_reason == "NO_RUNTIME_SELECTED_FRONTIER"
        and per_eval == 0 and per_reason == "NO_RUNTIME_SELECTED_FRONTIER"
    )
    if state_a:
        return "A_NO_RUNTIME_SELECTED_FRONTIER", True

    valid_policy = False
    if policy_raw != "":
        try:
            int(policy_raw)
            valid_policy = True
        except ValueError:
            valid_policy = False

    state_b = (
        valid_policy and lookup == 1 and src_reason == ""
        and per_eval == 1 and per_reason == ""
    )
    if state_b:
        return "B_SELECTED_FRONTIER_AVAILABLE", False

    state_c = (
        valid_policy and lookup == 1 and outcome == "no_normally_selectable_candidate"
        and candidate_total == 1
        and not is_blank(src.get("candidate_table_path"))
        and not is_blank(src.get("candidate_table_sha256"))
        and src_reason == "SELECTED_FRONTIER_CARDINALITY_NE_1"
        and per_eval == 0 and per_reason == "SELECTED_FRONTIER_CARDINALITY_NE_1"
    )
    if state_c:
        return "C_FRONTIER_INDETERMINATE", False

    return "D_FRONTIER_INTEGRITY_FAIL", False


def build_candidate_grid():
    rows = []
    for grid_name, taus in (
        ("PRIMARY", PRIMARY_TAUS),
        ("SENSITIVITY_A", SENS_A_TAUS),
        ("SENSITIVITY_B", SENS_B_TAUS),
    ):
        for family in ("F0", "F1", "F2", "F3"):
            c_values = CS if family in {"F1", "F3"} else (None,)
            for tau_pct in taus:
                for K in KS:
                    for c in c_values:
                        rows.append({
                            "grid": grid_name,
                            "candidate_id": f"{grid_name}:{family}:tau{tau_pct:.1f}:K{K}:c{c if c is not None else 'NA'}",
                            "family": family,
                            "tau_R_pct": tau_pct,
                            "tau_R_fraction": tau_pct / 100.0,
                            "K": K,
                            "c": c if c is not None else "",
                            "optional_confirmation_complexity": COMPLEXITY[family],
                            "hyperparameter_count": HYPERPARAMS[family],
                            "primary_boundary_nonselectable": int(
                                grid_name == "PRIMARY" and tau_pct in {1.0, 15.0}
                            ),
                        })
    return rows


def qualify(row: dict, family: str, tau_frac: float, c):
    rcond = bool(row["R_available"]) and float(row["R_map"]) <= tau_frac
    base = rcond and bool(row["TopoValid"])
    if family == "F0":
        return rcond, base
    if family == "F1":
        return rcond, base and bool(row[f"LowGain_c{int(c)}"])
    if family == "F2":
        return rcond, base and bool(row["NoFrontierTerminal"])
    if family == "F3":
        return rcond, base and (
            bool(row[f"LowGain_c{int(c)}"]) or bool(row["NoFrontierTerminal"])
        )
    raise ValueError(family)


def replay_rule(run_rows: list[dict], family: str, tau_pct: float, K: int, c=None,
                remove_topology=False):
    streak = 0
    first = None
    trace = []
    for row in run_rows:
        rcond = bool(row["R_available"]) and float(row["R_map"]) <= tau_pct / 100.0
        base = rcond and (True if remove_topology else bool(row["TopoValid"]))
        if family == "F0":
            q = base
        elif family == "F1":
            q = base and bool(row[f"LowGain_c{int(c)}"])
        elif family == "F2":
            q = base and bool(row["NoFrontierTerminal"])
        elif family == "F3":
            q = base and (
                bool(row[f"LowGain_c{int(c)}"]) or bool(row["NoFrontierTerminal"])
            )
        else:
            raise ValueError(family)
        streak = streak + 1 if q else 0
        fire = first is None and streak >= K
        if fire:
            first = row
        trace.append((rcond, q, streak, fire))
    return first, trace


def eval_run(run_rows: list[dict], family: str, tau_pct: float, K: int, c,
             oracle_index: dict, remove_topology=False):
    run_id = run_rows[0]["run_id"]
    stop, _ = replay_rule(run_rows, family, tau_pct, K, c, remove_topology=remove_topology)
    N = int(run_rows[0]["decision_count"])
    io = ORACLE4[run_id]
    if stop is None:
        return {
            "run_id": run_id, "StopStatus": "NO_STOP", "CandidateStop": "",
            "OracleStop_4": io, "DelayDecisions": math.nan, "PrematureStop": False,
            "PrematureByDecisions": 0, "SevereFalseStop10": False,
            "OracleRemainingFraction_GT_at_stop": math.nan,
            "SavedDecisions": 0, "PositiveSaving": False, "SavedProgress": 0.0,
            "SavedTime_s": math.nan,
        }
    ic = int(stop["decision_index"])
    delay = ic - io
    status = "PREMATURE" if delay < 0 else ("ON_TARGET" if delay == 0 else "LATE")
    o = oracle_index[(run_id, int(stop["decision_id"]))]
    if not bval(o["oracle_truth_evaluable"]) or not finite(o["OracleRemainingFraction_GT"]):
        raise RuntimeError(f"{run_id} d{stop['decision_id']}: oracle truth unavailable")
    oracle_remaining = float(o["OracleRemainingFraction_GT"])
    saved_decisions = N - ic
    final_time = float(run_rows[-1]["decision_time_s"])
    stop_time = float(stop["decision_time_s"])
    saved_time = final_time - stop_time if finite(final_time) and finite(stop_time) else math.nan
    return {
        "run_id": run_id, "StopStatus": status, "CandidateStop": int(stop["decision_id"]),
        "OracleStop_4": io, "DelayDecisions": delay, "PrematureStop": delay < 0,
        "PrematureByDecisions": max(0, -delay),
        "SevereFalseStop10": oracle_remaining > 0.10,
        "OracleRemainingFraction_GT_at_stop": oracle_remaining,
        "SavedDecisions": saved_decisions,
        "PositiveSaving": saved_decisions > 0,
        "SavedProgress": 1.0 - float(stop["normalized_progress"]),
        "SavedTime_s": saved_time,
    }


def aggregate_candidate(candidate: dict, runs: list[str], base_by_run, oracle_index, grid_name: str):
    outcomes = [
        eval_run(
            base_by_run[r], candidate["family"], float(candidate["tau_R_pct"]),
            int(candidate["K"]), None if is_blank(candidate["c"]) else int(candidate["c"]),
            oracle_index,
        )
        for r in runs
    ]
    n = len(runs)
    fired = sum(not is_blank(o["CandidateStop"]) for o in outcomes)
    positive = sum(bool(o["PositiveSaving"]) for o in outcomes)
    premature = sum(bool(o["PrematureStop"]) for o in outcomes)
    severe = sum(bool(o["SevereFalseStop10"]) for o in outcomes)
    nonprem_delays = [
        float(o["DelayDecisions"]) for o in outcomes
        if o["StopStatus"] in {"ON_TARGET", "LATE"} and finite(o["DelayDecisions"])
    ]
    median_delay = med(nonprem_delays)
    mean_delay = mean(nonprem_delays)
    stop_cov = fired / n
    pos_cov = positive / n
    mean_saved_progress = mean([o["SavedProgress"] for o in outcomes])

    reasons = []
    if premature != 0:
        reasons.append("PREMATURE_STOP_GT0")
    if severe != 0:
        reasons.append("SEVERE_FALSE_STOP10_GT0")
    if fired < math.ceil(0.80 * n):
        reasons.append("FIRED_RUN_COUNT_LT_CEIL_80PCT_N")
    if positive < math.ceil(0.70 * n):
        reasons.append("POSITIVE_SAVING_RUN_COUNT_LT_CEIL_70PCT_N")
    if not finite(median_delay):
        reasons.append("NO_FIRED_NONPREMATURE_DELAY_SUPPORT")
    elif median_delay > 3:
        reasons.append("MEDIAN_NONPREMATURE_DELAY_GT3")
    if grid_name == "PRIMARY" and float(candidate["tau_R_pct"]) in {1.0, 15.0}:
        reasons.append("PRIMARY_TAU_BOUNDARY_NONSEL")
    admissible = len(reasons) == 0

    return {
        **candidate,
        "truth_valid_run_count": n,
        "fired_run_count": fired,
        "positive_saving_run_count": positive,
        "StopCoverage": stop_cov,
        "PositiveSavingCoverage": pos_cov,
        "premature_stop_count": premature,
        "SevereFalseStop10_count": severe,
        "median_nonpremature_DelayDecisions": median_delay,
        "mean_nonpremature_DelayDecisions": mean_delay,
        "nonpremature_delay_support": len(nonprem_delays),
        "MeanSavedProgress": mean_saved_progress,
        "admissible": int(admissible),
        "reject_reason": "|".join(reasons),
        "_outcomes": outcomes,
    }


def selector_key(row: dict):
    c = int(row["c"]) if not is_blank(row["c"]) else -1
    return (
        float(row["median_nonpremature_DelayDecisions"]),
        int(row["optional_confirmation_complexity"]),
        float(row["mean_nonpremature_DelayDecisions"]),
        -float(row["PositiveSavingCoverage"]),
        -float(row["StopCoverage"]),
        -float(row["MeanSavedProgress"]),
        float(row["tau_R_pct"]),
        -int(row["K"]),
        c,
        FAMILY_ORDER[row["family"]],
    )


def select(rows: list[dict]):
    admissible = [r for r in rows if int(r["admissible"]) == 1]
    if not admissible:
        return None, []
    ranked = sorted(admissible, key=selector_key)
    return ranked[0], ranked


def public_summary(row: dict) -> dict:
    return {k: v for k, v in row.items() if not k.startswith("_")}


def make_base_replay(ridx, cidx, csrcidx, rsrcidx, oracle_index):
    base_rows = []
    missingness = Counter()
    state_counts = Counter()
    topo_reason_counts = Counter()
    hard_failures = []

    expected_keys = set()
    for run_id in RUNS:
        run_dir = EXPERIMENTS / run_id
        decisions = sorted(read_csv(run_dir / "decisions.csv"), key=lambda r: int(r["decision_id"]))
        trajectory = topo.read_trajectory(run_dir / "trajectory.csv")
        total = len(decisions)
        for pos, decision in enumerate(decisions, 1):
            key = (run_id, int(decision["decision_id"]))
            expected_keys.add(key)
            if key not in ridx or key not in cidx or key not in csrcidx or key not in rsrcidx or key not in oracle_index:
                raise RuntimeError(f"missing accepted replay key {key}")
            rr, cp, cs, rs = ridx[key], cidx[key], csrcidx[key], rsrcidx[key]

            if int(rr["decision_index"]) != pos or int(rr["decision_count"]) != total:
                raise RuntimeError(f"{key}: MX026 decision identity mismatch")
            if int(cp["decision_index"]) != pos or int(cp["decision_count"]) != total:
                raise RuntimeError(f"{key}: MX027 decision identity mismatch")
            if not math.isclose(float(rr["normalized_progress"]), float(cp["normalized_progress"]), rel_tol=0.0, abs_tol=1e-15):
                raise RuntimeError(f"{key}: accepted frame progress mismatch")

            r_available = str(rr["R_map_evaluable"]).strip() == "1" and finite(rr["R_MapRemainingFraction"])
            r_reason = str(rr["R_map_reason"]).strip()
            r_map = float(rr["R_MapRemainingFraction"]) if r_available else math.nan
            if not r_available:
                missingness[("R", r_reason or "R_UNAVAILABLE")] += 1

            coverage_source_reason = str(cs.get("coverage_source_reason", "")).strip()
            resolution = fval(cs.get("raw_resolution"))
            if coverage_source_reason or not finite(resolution) or resolution <= 0:
                hard_failures.append((key, "COVERAGE_SOURCE_INTEGRITY"))
                raise RuntimeError(f"{key}: hard coverage source integrity failure")
            if is_blank(cs.get("raw_map")) or is_blank(cs.get("raw_map_sha256")):
                raise RuntimeError(f"{key}: missing raw-map source provenance")

            delta_reason = str(cp.get("DeltaKnownArea_reason", "")).strip()
            rate_reason = str(cp.get("KnownAreaRate_reason", "")).strip()
            coverage_reason = str(cp.get("Coverage_reason", "")).strip()
            coverage_ok = (
                str(cp.get("Coverage_evaluable", "")).strip() == "1"
                and coverage_reason == "" and delta_reason == "" and rate_reason == ""
                and finite(cp.get("DeltaKnownArea_m2"))
                and finite(cp.get("KnownAreaRate_m2_s"))
            )
            if not coverage_ok:
                reason = "|".join(x for x in (coverage_reason, delta_reason, rate_reason) if x) or "COVERAGE_UNAVAILABLE"
                missingness[("Coverage", reason)] += 1

            low_gain = {}
            low_reasons = {}
            for c in CS:
                if not coverage_ok:
                    low_gain[c] = False
                    low_reasons[c] = delta_reason or rate_reason or coverage_reason or "COVERAGE_UNAVAILABLE"
                else:
                    d = float(cp["DeltaKnownArea_m2"])
                    rate = float(cp["KnownAreaRate_m2_s"])
                    low_gain[c] = bool(0.0 <= d <= c * resolution * resolution and rate >= 0.0)
                    low_reasons[c] = "" if low_gain[c] else (
                        "NEGATIVE_DELTA_OR_RATE" if (d < 0 or rate < 0) else "GAIN_ABOVE_CELL_BUDGET"
                    )

            fs, nofront = frontier_state(cp, cs)
            state_counts[fs] += 1
            if fs.startswith("D_"):
                raise RuntimeError(f"{key}: FRONTIER_INTEGRITY_FAIL")
            if fs.startswith("C_"):
                missingness[("Frontier", "FRONTIER_INDETERMINATE")] += 1
            elif fs.startswith("A_"):
                missingness[("Frontier", "NO_RUNTIME_SELECTED_FRONTIER")] += 1

            try:
                tv, treason, reach_n, source_mode, source_row, source_col, crop_json = topo_valid_for_decision(
                    run_dir, decision, rr, rs, trajectory
                )
            except Exception as exc:
                raise RuntimeError(f"{key}: {exc}") from exc
            topo_reason_counts[treason or "VALID"] += 1
            if not tv:
                missingness[("TopoValid", treason or "INVALID")] += 1

            orow = oracle_index[key]
            if not bval(orow["oracle_truth_evaluable"]) or not finite(orow["OracleRemainingFraction_GT"]):
                raise RuntimeError(f"{key}: W032 oracle truth invalid")

            row = {
                "run_id": run_id,
                "decision_id": int(decision["decision_id"]),
                "decision_index": pos,
                "decision_count": total,
                "normalized_progress": float(rr["normalized_progress"]),
                "decision_time_s": float(decision["time_s"]),
                "R_available": int(r_available),
                "R_reason": r_reason,
                "R_map": r_map,
                "TopoValid": int(tv),
                "TopoValid_reason": treason,
                "Topo_source_mode": source_mode,
                "Topo_source_row": source_row,
                "Topo_source_col": source_col,
                "Topo_source_reachable_count": reach_n,
                "Topo_crop_metadata": crop_json,
                "Coverage_evaluable": int(str(cp.get("Coverage_evaluable", "")).strip() == "1"),
                "Coverage_reason": coverage_reason,
                "KnownArea_m2": fval(cp.get("KnownArea_m2")),
                "DeltaKnownArea_m2": fval(cp.get("DeltaKnownArea_m2")),
                "DeltaKnownArea_reason": delta_reason,
                "KnownAreaRate_m2_s": fval(cp.get("KnownAreaRate_m2_s")),
                "KnownAreaRate_reason": rate_reason,
                "raw_resolution": resolution,
                "LowGain_c0": int(low_gain[0]),
                "LowGain_c0_reason": low_reasons[0],
                "LowGain_c1": int(low_gain[1]),
                "LowGain_c1_reason": low_reasons[1],
                "LowGain_c2": int(low_gain[2]),
                "LowGain_c2_reason": low_reasons[2],
                "FrontierState": fs,
                "NoFrontierTerminal": int(nofront),
                "IG_evaluable": cp.get("IG_evaluable", ""),
                "IG_reason": cp.get("IG_reason", ""),
                "IG_selected": fval(cp.get("IG_selected")),
                "IG_visible_unknown_cells": fval(cp.get("IG_visible_unknown_cells")),
                "OracleStop_4": ORACLE4[run_id],
                "OracleRemainingFraction_GT": float(orow["OracleRemainingFraction_GT"]),
            }
            for tau in PRIMARY_TAUS:
                row[f"RCondition_tau_{int(tau):02d}pct"] = int(r_available and r_map <= tau / 100.0)
            base_rows.append(row)

    if len(base_rows) != 365 or len(expected_keys) != 365:
        raise RuntimeError(f"expected 365 decisions, got {len(base_rows)}")
    for name, idx in (
        ("MX026", ridx), ("MX027", cidx), ("MX027_SOURCE", csrcidx),
        ("MX025_SOURCE", rsrcidx), ("W032", oracle_index),
    ):
        if set(idx) != expected_keys:
            extra = len(set(idx) - expected_keys)
            missing = len(expected_keys - set(idx))
            raise RuntimeError(f"{name} membership mismatch missing={missing} extra={extra}")
    return base_rows, missingness, state_counts, topo_reason_counts


def add_selected_trace(base_rows, selected):
    by_run = defaultdict(list)
    for r in base_rows:
        by_run[r["run_id"]].append(r)
    for rr in by_run.values():
        rr.sort(key=lambda x: x["decision_index"])

    if selected is None:
        for row in base_rows:
            row.update({
                "FullFit_family": "", "FullFit_tau_R_pct": "", "FullFit_K": "", "FullFit_c": "",
                "FullFit_RCondition": "", "FullFit_Qualify": "", "FullFit_streak": "",
                "FullFit_first_fire_here": "", "FullFit_reset_reason": "NO_FULL_FIT_RULE",
            })
        return

    family = selected["family"]
    tau = float(selected["tau_R_pct"])
    K = int(selected["K"])
    c = None if is_blank(selected["c"]) else int(selected["c"])
    for run_id, rr in by_run.items():
        _, trace = replay_rule(rr, family, tau, K, c)
        for row, (rcond, q, streak, fire) in zip(rr, trace):
            if q:
                reset = ""
            elif not row["R_available"]:
                reset = "R_UNAVAILABLE"
            elif not rcond:
                reset = "R_ABOVE_TAU"
            elif not row["TopoValid"]:
                reset = "TOPO_INVALID:" + str(row["TopoValid_reason"])
            elif family in {"F1", "F3"} and not row[f"LowGain_c{c}"] and not (
                family == "F3" and row["NoFrontierTerminal"]
            ):
                reset = "LOW_GAIN_CONFIRMATION_FALSE"
            elif family in {"F2", "F3"} and not row["NoFrontierTerminal"] and not (
                family == "F3" and row[f"LowGain_c{c}"]
            ):
                reset = "NO_FRONTIER_CONFIRMATION_FALSE"
            else:
                reset = "QUALIFY_FALSE"
            row.update({
                "FullFit_family": family,
                "FullFit_tau_R_pct": tau,
                "FullFit_K": K,
                "FullFit_c": c if c is not None else "",
                "FullFit_RCondition": int(rcond),
                "FullFit_Qualify": int(q),
                "FullFit_streak": streak,
                "FullFit_first_fire_here": int(fire),
                "FullFit_reset_reason": reset,
            })


def sensitivity_fit(grid_name, candidates, base_by_run, oracle_index):
    rows = [
        aggregate_candidate(c, list(RUNS), base_by_run, oracle_index, grid_name)
        for c in candidates if c["grid"] == grid_name
    ]
    chosen, ranked = select(rows)
    if chosen is None:
        return {
            "grid": grid_name, "status": "NO_ADMISSIBLE_MX031_RULE",
            "candidate_count": len(rows), "admissible_count": 0,
        }
    out = public_summary(chosen)
    out.update({
        "status": "ADMISSIBLE",
        "candidate_count": len(rows),
        "admissible_count": len(ranked),
        "selector_rank": 1,
    })
    return out


def main():
    execution_delta = source_guard()
    RESULTS.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)

    ridx = index_rows(read_csv(R_PATH), "MX026")
    cidx = index_rows(read_csv(COV_PATH), "MX027")
    csrcidx = index_rows(read_csv(COV_SOURCE_PATH), "MX027_SOURCE")
    rsrcidx = index_rows(read_csv(R_SOURCE_PATH), "MX025_SOURCE")
    oracle_index = index_rows(read_csv(ORACLE_PATH), "W032")

    base_rows, missingness, state_counts, topo_reason_counts = make_base_replay(
        ridx, cidx, csrcidx, rsrcidx, oracle_index
    )
    base_by_run = {
        r: sorted([x for x in base_rows if x["run_id"] == r], key=lambda x: x["decision_index"])
        for r in RUNS
    }

    grid_rows = build_candidate_grid()
    primary_candidates = [r for r in grid_rows if r["grid"] == "PRIMARY"]

    # Outer LORO: every candidate scored on the nine training runs.
    training_rows = []
    loro_rows = []
    fold_selected = []
    for heldout in RUNS:
        training_runs = [r for r in RUNS if r != heldout]
        scored = [
            aggregate_candidate(c, training_runs, base_by_run, oracle_index, "PRIMARY")
            for c in primary_candidates
        ]
        selected, ranked = select(scored)
        rank_map = {r["candidate_id"]: i + 1 for i, r in enumerate(ranked)}
        for s in scored:
            z = public_summary(s)
            z["heldout_run"] = heldout
            z["training_runs"] = ";".join(training_runs)
            z["selector_rank_among_admissible"] = rank_map.get(s["candidate_id"], "")
            z["selected_training_rule"] = int(selected is not None and s["candidate_id"] == selected["candidate_id"])
            training_rows.append(z)

        if selected is None:
            row = {
                "heldout_run": heldout,
                "fold_status": "NO_ADMISSIBLE_MX031_RULE",
                "selected_family": "", "selected_tau_R_pct": "", "selected_K": "", "selected_c": "",
                "heldout_stop_status": "NO_RULE", "heldout_CandidateStop": "",
                "heldout_OracleStop_4": ORACLE4[heldout], "heldout_DelayDecisions": math.nan,
                "heldout_PrematureStop": False, "heldout_PrematureByDecisions": 0,
                "heldout_SevereFalseStop10": False, "heldout_SavedDecisions": 0,
                "heldout_PositiveSaving": False, "heldout_SavedProgress": 0.0,
                "heldout_SavedTime_s": math.nan,
            }
            fold_selected.append(None)
        else:
            c = None if is_blank(selected["c"]) else int(selected["c"])
            out = eval_run(
                base_by_run[heldout], selected["family"], float(selected["tau_R_pct"]),
                int(selected["K"]), c, oracle_index
            )
            row = {
                "heldout_run": heldout,
                "fold_status": "ADMISSIBLE",
                "selected_family": selected["family"],
                "selected_tau_R_pct": float(selected["tau_R_pct"]),
                "selected_K": int(selected["K"]),
                "selected_c": c if c is not None else "",
                "training_median_nonpremature_DelayDecisions": selected["median_nonpremature_DelayDecisions"],
                "training_mean_nonpremature_DelayDecisions": selected["mean_nonpremature_DelayDecisions"],
                "training_StopCoverage": selected["StopCoverage"],
                "training_PositiveSavingCoverage": selected["PositiveSavingCoverage"],
                "heldout_stop_status": out["StopStatus"],
                "heldout_CandidateStop": out["CandidateStop"],
                "heldout_OracleStop_4": out["OracleStop_4"],
                "heldout_DelayDecisions": out["DelayDecisions"],
                "heldout_PrematureStop": out["PrematureStop"],
                "heldout_PrematureByDecisions": out["PrematureByDecisions"],
                "heldout_SevereFalseStop10": out["SevereFalseStop10"],
                "heldout_OracleRemainingFraction_GT_at_stop": out["OracleRemainingFraction_GT_at_stop"],
                "heldout_SavedDecisions": out["SavedDecisions"],
                "heldout_PositiveSaving": out["PositiveSaving"],
                "heldout_SavedProgress": out["SavedProgress"],
                "heldout_SavedTime_s": out["SavedTime_s"],
            }
            fold_selected.append(selected)
        loro_rows.append(row)

    # Full primary fit: keep all 480 candidate summaries and selector trace.
    full_scored = [
        aggregate_candidate(c, list(RUNS), base_by_run, oracle_index, "PRIMARY")
        for c in primary_candidates
    ]
    full_selected, full_ranked = select(full_scored)
    full_rank_map = {r["candidate_id"]: i + 1 for i, r in enumerate(full_ranked)}
    full_rows = []
    for s in full_scored:
        z = public_summary(s)
        z["selector_rank_among_admissible"] = full_rank_map.get(s["candidate_id"], "")
        z["selected_full_fit_rule"] = int(full_selected is not None and s["candidate_id"] == full_selected["candidate_id"])
        full_rows.append(z)

    # Sensitivity full-development selectors; boundaries are allowed to be selected
    # so S8 can explicitly fail if the optimum is on a sensitivity boundary.
    sens_a = sensitivity_fit("SENSITIVITY_A", grid_rows, base_by_run, oracle_index)
    sens_b = sensitivity_fit("SENSITIVITY_B", grid_rows, base_by_run, oracle_index)
    sens_rows = [sens_a, sens_b]

    add_selected_trace(base_rows, full_selected)

    # Stability gates.
    stability = []
    solved = sum(r["fold_status"] == "ADMISSIBLE" for r in loro_rows)
    s1 = solved >= 8
    stability.append({"gate": "S1", "status": "PASS" if s1 else "FAIL", "value": solved, "requirement": "admissible outer folds >= 8/10"})

    held_prem = sum(bool(r["heldout_PrematureStop"]) for r in loro_rows)
    held_severe = sum(bool(r["heldout_SevereFalseStop10"]) for r in loro_rows)
    s2 = held_prem == 0 and held_severe == 0
    stability.append({"gate": "S2", "status": "PASS" if s2 else "FAIL", "value": f"premature={held_prem};severe={held_severe}", "requirement": "premature=0 and SevereFalseStop10=0"})

    held_fired = sum(not is_blank(r["heldout_CandidateStop"]) for r in loro_rows)
    held_positive = sum(bool(r["heldout_PositiveSaving"]) for r in loro_rows)
    held_delays = [
        float(r["heldout_DelayDecisions"]) for r in loro_rows
        if r["heldout_stop_status"] in {"ON_TARGET", "LATE"} and finite(r["heldout_DelayDecisions"])
    ]
    held_med_delay = med(held_delays)
    s3 = held_fired >= 8 and held_positive >= 7 and finite(held_med_delay) and held_med_delay <= 3
    stability.append({"gate": "S3", "status": "PASS" if s3 else "FAIL", "value": f"fired={held_fired};positive={held_positive};median_delay={held_med_delay}", "requirement": "fired>=8; positive>=7; median nonpremature delay<=3"})

    if full_selected is None:
        for gate, requirement in (
            ("S4", "family match >=7/10"),
            ("S5", "tau match/range stability"),
            ("S6", "K match/range stability"),
            ("S7", "coverage c stability when applicable"),
            ("S8", "Sensitivity A/B stability"),
        ):
            stability.append({"gate": gate, "status": "NA", "value": "NO_FULL_FIT_RULE", "requirement": requirement})
        overall = "NO_STABLE_MULTI_SIGNAL_STOP_CANDIDATE"
    else:
        fam = full_selected["family"]
        tau_star = float(full_selected["tau_R_pct"])
        k_star = int(full_selected["K"])
        c_star = None if is_blank(full_selected["c"]) else int(full_selected["c"])
        solved_rows = [r for r in loro_rows if r["fold_status"] == "ADMISSIBLE"]

        fam_matches = sum(r["fold_status"] == "ADMISSIBLE" and r["selected_family"] == fam for r in loro_rows)
        s4 = fam_matches >= 7
        stability.append({"gate": "S4", "status": "PASS" if s4 else "FAIL", "value": fam_matches, "requirement": f"same family as full fit >=7/10"})

        tau_matches = sum(
            r["fold_status"] == "ADMISSIBLE" and abs(float(r["selected_tau_R_pct"]) - tau_star) <= 2.0
            for r in loro_rows
        )
        taus = [float(r["selected_tau_R_pct"]) for r in solved_rows]
        tau_range = max(taus) - min(taus) if taus else math.nan
        s5 = tau_matches >= 7 and finite(tau_range) and tau_range <= 6.0
        stability.append({"gate": "S5", "status": "PASS" if s5 else "FAIL", "value": f"within2pp={tau_matches};range={tau_range}", "requirement": ">=7/10 within ±2pp; admissible-fold tau range<=6pp"})

        k_matches = sum(r["fold_status"] == "ADMISSIBLE" and int(r["selected_K"]) == k_star for r in loro_rows)
        kvals = [int(r["selected_K"]) for r in solved_rows]
        k_range = max(kvals) - min(kvals) if kvals else math.nan
        s6 = k_matches >= 7 and finite(k_range) and k_range <= 1
        stability.append({"gate": "S6", "status": "PASS" if s6 else "FAIL", "value": f"exactK={k_matches};range={k_range}", "requirement": ">=7/10 exact K; admissible-fold K range<=1"})

        if fam in {"F1", "F3"}:
            exact_fc = sum(
                r["fold_status"] == "ADMISSIBLE"
                and r["selected_family"] == fam
                and int(r["selected_c"]) == c_star
                for r in loro_rows
            )
            cov_c = [
                int(r["selected_c"]) for r in solved_rows
                if r["selected_family"] in {"F1", "F3"} and not is_blank(r["selected_c"])
            ]
            crange = max(cov_c) - min(cov_c) if cov_c else math.nan
            s7 = exact_fc >= 7 and finite(crange) and crange <= 1
            stability.append({"gate": "S7", "status": "PASS" if s7 else "FAIL", "value": f"exact_family_c={exact_fc};coverage_c_range={crange}", "requirement": "exact family+c >=7/10 and coverage-family c range<=1"})
        else:
            s7 = True
            stability.append({"gate": "S7", "status": "NA", "value": "NOT_APPLICABLE", "requirement": "only F1/F3"})

        def sens_ok(sens, grid_name):
            if sens.get("status") != "ADMISSIBLE":
                return False, "NO_ADMISSIBLE_RULE"
            same = (
                sens["family"] == fam and int(sens["K"]) == k_star
                and ((fam not in {"F1", "F3"}) or int(sens["c"]) == c_star)
            )
            t = float(sens["tau_R_pct"])
            if grid_name == "A":
                close = abs(t - tau_star) <= 1.0
                interior = 0.5 < t < 15.0
            else:
                close = abs(t - tau_star) <= 2.0
                interior = 0.5 < t < 20.0
            return same and close and interior, f"family={sens.get('family')};tau={t};K={sens.get('K')};c={sens.get('c')};same={same};close={close};interior={interior}"

        a_ok, a_detail = sens_ok(sens_a, "A")
        b_ok, b_detail = sens_ok(sens_b, "B")
        s8 = a_ok and b_ok
        stability.append({"gate": "S8", "status": "PASS" if s8 else "FAIL", "value": f"A[{a_detail}] B[{b_detail}]", "requirement": "A/B same family,K,c; tau close and strictly interior"})

        applicable_pass = all(
            row["status"] == "PASS" or (row["gate"] == "S7" and row["status"] == "NA")
            for row in stability
        )
        overall = "MX031_NEW_ROOM_DEVELOPMENT_CANDIDATE" if applicable_pass else "NO_STABLE_MULTI_SIGNAL_STOP_CANDIDATE"

    # Ablations using exact full-fit parameters if a full-fit candidate exists.
    ablation_rows = []
    if full_selected is not None:
        family = full_selected["family"]
        tau = float(full_selected["tau_R_pct"])
        K = int(full_selected["K"])
        c = None if is_blank(full_selected["c"]) else int(full_selected["c"])
        defs = [
            ("A0", family, tau, K, c, False, "selected architecture"),
            ("A1", family, tau, K, c, True, "remove topology veto"),
            ("A2", family, tau, 1, c, False, "remove persistence"),
            ("A3", "F0", tau, K, None, False, "R-only backbone"),
            ("A4", "F0", tau, K, None, False, "confirmation-off counterpart"),
        ]
        for aid, afam, atau, ak, ac, remove_topo, desc in defs:
            outs = [eval_run(base_by_run[r], afam, atau, ak, ac, oracle_index, remove_topology=remove_topo) for r in RUNS]
            for o in outs:
                ablation_rows.append({
                    "row_type": "RUN", "ablation": aid, "description": desc,
                    "family": afam, "tau_R_pct": atau, "K": ak, "c": ac if ac is not None else "",
                    "remove_topology_veto": int(remove_topo), **o,
                })
            fired = sum(not is_blank(o["CandidateStop"]) for o in outs)
            positive = sum(bool(o["PositiveSaving"]) for o in outs)
            prem = sum(bool(o["PrematureStop"]) for o in outs)
            severe = sum(bool(o["SevereFalseStop10"]) for o in outs)
            delays = [o["DelayDecisions"] for o in outs if o["StopStatus"] in {"ON_TARGET", "LATE"}]
            ablation_rows.append({
                "row_type": "AGGREGATE", "ablation": aid, "description": desc,
                "family": afam, "tau_R_pct": atau, "K": ak, "c": ac if ac is not None else "",
                "remove_topology_veto": int(remove_topo),
                "fired_run_count": fired, "StopCoverage": fired / 10.0,
                "positive_saving_run_count": positive, "PositiveSavingCoverage": positive / 10.0,
                "premature_stop_count": prem, "SevereFalseStop10_count": severe,
                "median_nonpremature_DelayDecisions": med(delays),
                "mean_nonpremature_DelayDecisions": mean(delays),
                "MeanSavedProgress": mean([o["SavedProgress"] for o in outs]),
            })

    # Stop context after decisions are frozen. P/U/topology fields are evaluator-only.
    stop_context = []
    for lr in loro_rows:
        held = lr["heldout_run"]
        did = ival(lr["heldout_CandidateStop"])
        context = {
            "scope": "OUTER_HELDOUT", "run_id": held, "decision_id": did if did is not None else "",
            "rule_status": lr["fold_status"], "stop_status": lr["heldout_stop_status"],
            "recognizer_context_separation": "P/U/retrospective topology joined only after fixed stop",
        }
        if did is not None:
            key = (held, did)
            p = ridx[key]
            o = oracle_index[key]
            b = next(x for x in base_by_run[held] if x["decision_id"] == did)
            context.update({
                "P_structural_evaluable": p["P_structural_evaluable"],
                "P_STRUCT_FreeRecall": fval(p["P_STRUCT_FreeRecall"]),
                "P_STRUCT_OccupiedPrecision": fval(p["P_STRUCT_OccupiedPrecision"]),
                "P_STRUCT_MissedFreeRate": fval(p["P_STRUCT_MissedFreeRate"]),
                "P_STRUCT_FalseOpenRate": fval(p["P_STRUCT_FalseOpenRate"]),
                "P_later_observed_evaluable": p["P_later_observed_evaluable"],
                "P_LATER_FreeRecall": fval(p["P_LATER_FreeRecall"]),
                "P_LATER_OccupiedPrecision": fval(p["P_LATER_OccupiedPrecision"]),
                "U_evaluable": p["U_evaluable"],
                "U_MAPEX_U_mean": fval(p["U_MAPEX_U_mean"]),
                "U_MAPEX_U_p95": fval(p["U_MAPEX_U_p95"]),
                "U_MAPEX_U_disagreement": fval(p["U_MAPEX_U_disagreement"]),
                "TopoValid_MX031": b["TopoValid"],
                "TopoValid_MX031_reason": b["TopoValid_reason"],
                "R004_source_prediction_traversable_RETROSPECTIVE": o.get("r004_topology_source_prediction_traversable", ""),
                "R004_fragmented_RETROSPECTIVE": o.get("r004_topology_fragmented", ""),
                "R004_lost_reachable_future_free_fraction_RETROSPECTIVE": fval(o.get("r004_topology_lost_reachable_future_free_fraction")),
            })
        stop_context.append(context)

    if full_selected is not None:
        family = full_selected["family"]
        tau = float(full_selected["tau_R_pct"])
        K = int(full_selected["K"])
        c = None if is_blank(full_selected["c"]) else int(full_selected["c"])
        for run_id in RUNS:
            out = eval_run(base_by_run[run_id], family, tau, K, c, oracle_index)
            did = ival(out["CandidateStop"])
            context = {
                "scope": "FULL_FIT", "run_id": run_id, "decision_id": did if did is not None else "",
                "rule_status": "ADMISSIBLE_FULL_FIT", "stop_status": out["StopStatus"],
                "recognizer_context_separation": "P/U/retrospective topology joined only after fixed stop",
            }
            if did is not None:
                key = (run_id, did)
                p = ridx[key]
                o = oracle_index[key]
                b = next(x for x in base_by_run[run_id] if x["decision_id"] == did)
                context.update({
                    "P_structural_evaluable": p["P_structural_evaluable"],
                    "P_STRUCT_FreeRecall": fval(p["P_STRUCT_FreeRecall"]),
                    "P_STRUCT_OccupiedPrecision": fval(p["P_STRUCT_OccupiedPrecision"]),
                    "P_STRUCT_MissedFreeRate": fval(p["P_STRUCT_MissedFreeRate"]),
                    "P_STRUCT_FalseOpenRate": fval(p["P_STRUCT_FalseOpenRate"]),
                    "P_later_observed_evaluable": p["P_later_observed_evaluable"],
                    "P_LATER_FreeRecall": fval(p["P_LATER_FreeRecall"]),
                    "P_LATER_OccupiedPrecision": fval(p["P_LATER_OccupiedPrecision"]),
                    "U_evaluable": p["U_evaluable"],
                    "U_MAPEX_U_mean": fval(p["U_MAPEX_U_mean"]),
                    "U_MAPEX_U_p95": fval(p["U_MAPEX_U_p95"]),
                    "U_MAPEX_U_disagreement": fval(p["U_MAPEX_U_disagreement"]),
                    "TopoValid_MX031": b["TopoValid"],
                    "TopoValid_MX031_reason": b["TopoValid_reason"],
                    "R004_source_prediction_traversable_RETROSPECTIVE": o.get("r004_topology_source_prediction_traversable", ""),
                    "R004_fragmented_RETROSPECTIVE": o.get("r004_topology_fragmented", ""),
                    "R004_lost_reachable_future_free_fraction_RETROSPECTIVE": fval(o.get("r004_topology_lost_reachable_future_free_fraction")),
                })
            stop_context.append(context)

    # Missingness inventory.
    missing_rows = []
    for (family, reason), count in sorted(missingness.items()):
        missing_rows.append({"signal_family": family, "reason": reason, "decision_count": count})
    for state, count in sorted(state_counts.items()):
        missing_rows.append({"signal_family": "FrontierState", "reason": state, "decision_count": count})
    for reason, count in sorted(topo_reason_counts.items()):
        missing_rows.append({"signal_family": "TopoValid_reason", "reason": reason, "decision_count": count})

    # Figures: use full-fit internal candidate if it exists. This is descriptive even if stability fails.
    for run_id in RUNS:
        rr = base_by_run[run_id]
        x = np.asarray([r["normalized_progress"] for r in rr], dtype=float)
        fig, axes = plt.subplots(5, 1, figsize=(12, 16), sharex=True)

        axes[0].plot(x, [r["R_map"] for r in rr], label="R_map")
        if full_selected is not None:
            axes[0].axhline(float(full_selected["tau_R_pct"]) / 100.0, linestyle="--", label="full-fit tau_R")
        axes[0].set_ylabel("R_map"); axes[0].legend()

        axes[1].plot(x, [r["DeltaKnownArea_m2"] for r in rr], label="DeltaKnownArea_m2")
        if full_selected is not None and full_selected["family"] in {"F1", "F3"}:
            c = int(full_selected["c"])
            axes[1].plot(x, [c * r["raw_resolution"] ** 2 for r in rr], linestyle="--", label=f"c={c} cell-area budget")
        axes[1].set_ylabel("coverage gain m2"); axes[1].legend()

        state_code = {
            "A_NO_RUNTIME_SELECTED_FRONTIER": 0,
            "B_SELECTED_FRONTIER_AVAILABLE": 1,
            "C_FRONTIER_INDETERMINATE": 2,
            "D_FRONTIER_INTEGRITY_FAIL": 3,
        }
        axes[2].step(x, [state_code[r["FrontierState"]] for r in rr], where="mid", label="FrontierState A/B/C/D")
        axes[2].set_yticks([0,1,2,3], ["A","B","C","D"]); axes[2].set_ylabel("frontier"); axes[2].legend()

        axes[3].step(x, [r["TopoValid"] for r in rr], where="mid", label="TopoValid_MX031")
        axes[3].set_ylim(-0.1,1.1); axes[3].set_ylabel("TopoValid"); axes[3].legend()

        if full_selected is not None:
            axes[4].step(x, [r["FullFit_Qualify"] for r in rr], where="mid", label="full-fit Qualify")
            fire = [r for r in rr if r["FullFit_first_fire_here"] == 1]
            if fire:
                axes[4].axvline(fire[0]["normalized_progress"], linestyle=":", label="full-fit first fire")
        axes[4].set_ylim(-0.1,1.1); axes[4].set_ylabel("qualify"); axes[4].set_xlabel("normalized progress"); axes[4].legend()

        oracle_row = next(r for r in rr if int(r["decision_id"]) == ORACLE4[run_id])
        for ax in axes:
            ax.axvline(oracle_row["normalized_progress"], linestyle="-.", linewidth=0.9, label="_nolegend_")
            ax.grid(True, alpha=0.2)
        fig.suptitle(f"MX031 {run_id} — Oracle-4 evaluator marker only")
        fig.tight_layout()
        fig.savefig(FIGURES / f"MX031_{run_id}_REPLAY.svg", format="svg")
        plt.close(fig)

    # Save outputs.
    write_csv(RESULTS / "MX031_CANDIDATE_GRID.csv", grid_rows)
    write_csv(RESULTS / "MX031_PER_DECISION_REPLAY.csv", base_rows)
    write_csv(RESULTS / "MX031_TRAINING_CANDIDATE_SUMMARY.csv", training_rows)
    write_csv(RESULTS / "MX031_LORO_SELECTED_RULES.csv", loro_rows)
    write_csv(RESULTS / "MX031_FULL_FIT_SELECTION.csv", full_rows)
    write_csv(RESULTS / "MX031_SENSITIVITY_SELECTION.csv", sens_rows)
    write_csv(RESULTS / "MX031_STABILITY_GATES.csv", stability)
    write_csv(RESULTS / "MX031_ABLATIONS.csv", ablation_rows)
    write_csv(RESULTS / "MX031_STOP_CONTEXT.csv", stop_context)
    write_csv(RESULTS / "MX031_MISSINGNESS_INVENTORY.csv", missing_rows)

    dictionary = {
        "schema": "mx031_metric_dictionary_v1",
        "method": METHOD,
        "result_status": "COMPLETE_PENDING_IR1_RESULT_QA",
        "R_map": "Read only from accepted MX026 R_MapRemainingFraction; no recomputation.",
        "LowGain_c": "Coverage-evaluable, finite Delta/Rate, 0<=Delta<=c*resolution^2, Rate>=0, c in {0,1,2}.",
        "FrontierState": {
            "A": "NO_RUNTIME_SELECTED_FRONTIER; terminal confirmation true",
            "B": "SELECTED_FRONTIER_AVAILABLE",
            "C": "FRONTIER_INDETERMINATE; ordinary fail-closed",
            "D": "FRONTIER_INTEGRITY_FAIL; hard replay integrity failure",
        },
        "TopoValid_MX031": "Runtime-only mean-prediction C-space using exact R004 primitives, radius 0.189 m, 8-neighbour no-corner-cut source reachability, with accepted map-only R provenance compatibility.",
        "families": {
            "F0": "R + TopoValid + persistence",
            "F1": "F0 + LowGain_c",
            "F2": "F0 + NoFrontierTerminal",
            "F3": "F0 + (LowGain_c OR NoFrontierTerminal)",
        },
        "primary_grid": {"tau_R_pct": list(PRIMARY_TAUS), "K": list(KS), "c": list(CS), "boundary_tau_nonselectable": [1.0, 15.0]},
        "sensitivity_A_tau_pct": list(SENS_A_TAUS),
        "sensitivity_B_tau_pct": list(SENS_B_TAUS),
        "admissibility": {
            "premature_stop_count": 0,
            "SevereFalseStop10_count": 0,
            "fired_run_count": ">= ceil(0.80*n)",
            "positive_saving_run_count": ">= ceil(0.70*n)",
            "median_nonpremature_DelayDecisions": "<=3",
            "integrity_failure": 0,
            "primary_tau": "interior only",
        },
        "selector_order": [
            "min median nonpremature delay", "min optional complexity",
            "min mean nonpremature delay", "max positive-saving coverage",
            "max stop coverage", "max mean saved progress", "lower tau_R",
            "larger K", "lower c for F1/F3", "family F0<F1<F2<F3",
        ],
        "SevereFalseStop10": "CandidateStop exists AND W032 OracleRemainingFraction_GT(CandidateStop)>0.10",
        "oracle_role": "evaluator truth only; never recognizer input",
        "P_U_role": "evaluator-only context after stops fixed; no effect on selection",
        "Hospital": "not scored; no retuning",
    }
    write_json(RESULTS / "MX031_METRIC_DICTIONARY.json", dictionary)

    # Concise report. Naming full-fit parameters is allowed only if S1-S8 all pass.
    gates_text = ", ".join(f"{r['gate']}={r['status']}" for r in stability)
    if overall == "MX031_NEW_ROOM_DEVELOPMENT_CANDIDATE":
        promoted = (
            f"Stable New Room development candidate exists: family {full_selected['family']}, "
            f"tau_R={float(full_selected['tau_R_pct']):.1f}%, K={int(full_selected['K'])}"
            + (f", c={int(full_selected['c'])}" if not is_blank(full_selected["c"]) else "")
            + "."
        )
    else:
        promoted = "No stable New Room multi-signal STOP candidate satisfies the frozen V3 gates. No retuning is performed."

    report = f"""# MX031 Analyst04 deterministic result candidate

Status: COMPLETE_PENDING_IR1_RESULT_QA

Frozen method: {METHOD}
Method QA ACCEPT: {METHOD_QA}

{promoted}

## Integrity

- New Room mpx_001..mpx_010: 365/365 accepted decisions.
- MX026 R replay blob: {EXPECTED_BLOBS[R_PATH]}.
- MX027 per-decision blob: {EXPECTED_BLOBS[COV_PATH]}.
- MX027 source-parity blob: {EXPECTED_BLOBS[COV_SOURCE_PATH]}.
- W032 oracle_decisions blob: {EXPECTED_BLOBS[ORACLE_PATH]}.
- W032 oracle_decisions SHA-256: {ORACLE_SHA256}.
- R004 topology implementation blob: {EXPECTED_BLOBS[TOPO_PATH]}.
- Shared online-source implementation blob: {EXPECTED_BLOBS[SHARED_PATH]}.
- Frontier state counts: {dict(state_counts)}.
- TopoValid reasons: {dict(topo_reason_counts)}.
- No Hospital data were scored.

## Outer LORO

- Solved folds: {solved}/10.
- Held-out fires: {held_fired}/10.
- Held-out positive-saving runs: {held_positive}/10.
- Held-out premature stops: {held_prem}.
- Held-out SevereFalseStop10: {held_severe}.
- Held-out median non-premature delay: {held_med_delay if finite(held_med_delay) else 'NA'} decisions.

## Stability

{gates_text}

Final frozen development status: {overall}

P/U and retrospective topology fields in MX031_STOP_CONTEXT.csv are joined only after stop decisions are fixed. They are evaluator context, not recognizer inputs.

Engineer remains blocked. No model/simulation rerun, Hospital retuning, weighted/composite score, P/U threshold promotion, online STOP implementation, or deployment is authorized.
"""
    (RESULTS / "MX031_ANALYST_REPORT.md").write_text(report, encoding="utf-8")

    provenance = {
        "schema": "mx031_execution_provenance_v1",
        "status": "COMPLETE_PENDING_IR1_RESULT_QA",
        "executor_role": "Data & Evidence Analyst successor 04",
        "method_commit": METHOD,
        "method_qa_commit": METHOD_QA,
        "technical_frozen_base": FROZEN_BASE,
        "staged_exact_inputs": {
            str(p.relative_to(REPO)): EXPECTED_BLOBS[p]
            for p in (R_PATH, R_MANIFEST_PATH, R_SOURCE_PATH, COV_PATH, COV_SOURCE_PATH)
        },
        "frozen_in_base": {
            str(ORACLE_PATH.relative_to(REPO)): EXPECTED_BLOBS[ORACLE_PATH],
            str(TOPO_PATH.relative_to(REPO)): EXPECTED_BLOBS[TOPO_PATH],
            str(SHARED_PATH.relative_to(REPO)): EXPECTED_BLOBS[SHARED_PATH],
        },
        "oracle_sha256": ORACLE_SHA256,
        "execution_delta_from_frozen_base": execution_delta,
        "cohort": {"runs": list(RUNS), "decision_n": 365, "oracle4": ORACLE4},
        "candidate_counts": {
            "primary": sum(r["grid"] == "PRIMARY" for r in grid_rows),
            "sensitivity_A": sum(r["grid"] == "SENSITIVITY_A" for r in grid_rows),
            "sensitivity_B": sum(r["grid"] == "SENSITIVITY_B" for r in grid_rows),
        },
        "guards": {
            "hospital_scored": False,
            "model_inference": False,
            "simulation_rerun": False,
            "threshold_retune": False,
            "P_U_recognizer_threshold": False,
            "weighted_composite": False,
            "engineer_implementation": False,
            "robot_stop": False,
            "deployment": False,
        },
        "runtime": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "matplotlib": matplotlib.__version__,
            "github_run_id": os.environ.get("GITHUB_RUN_ID", ""),
            "github_sha_at_start": os.environ.get("GITHUB_SHA", ""),
        },
    }
    write_json(RESULTS / "MX031_EXECUTION_PROVENANCE.json", provenance)

    artifacts = []
    for p in sorted(RESULTS.rglob("*")):
        if p.is_file() and p.name != "MX031_ARTIFACT_MANIFEST.json":
            artifacts.append({
                "path": str(p.relative_to(RESULTS)),
                "sha256": sha256_file(p),
                "size": p.stat().st_size,
            })
    manifest = {
        "schema": "mx031_artifact_manifest_v1",
        "status": "COMPLETE_PENDING_IR1_RESULT_QA",
        "method_commit": METHOD,
        "technical_frozen_base": FROZEN_BASE,
        "development_status": overall,
        "decision_rows": len(base_rows),
        "candidate_grid_rows": len(grid_rows),
        "training_candidate_rows": len(training_rows),
        "loro_rows": len(loro_rows),
        "full_fit_rows": len(full_rows),
        "sensitivity_rows": len(sens_rows),
        "stability_rows": len(stability),
        "artifact_count_excluding_manifest": len(artifacts),
        "artifacts": artifacts,
    }
    write_json(RESULTS / "MX031_ARTIFACT_MANIFEST.json", manifest)

    print(json.dumps({
        "status": "COMPLETE_PENDING_IR1_RESULT_QA",
        "development_status": overall,
        "decision_rows": len(base_rows),
        "candidate_grid_rows": len(grid_rows),
        "training_candidate_rows": len(training_rows),
        "primary_candidate_count": len(primary_candidates),
        "solved_folds": solved,
        "heldout_fires": held_fired,
        "heldout_positive": held_positive,
        "heldout_premature": held_prem,
        "heldout_severe": held_severe,
        "heldout_median_nonprem_delay": held_med_delay if finite(held_med_delay) else None,
        "frontier_state_counts": dict(state_counts),
        "topo_reason_counts": dict(topo_reason_counts),
        "full_fit_exists": full_selected is not None,
        "stability": {r["gate"]: r["status"] for r in stability},
    }, sort_keys=True))


if __name__ == "__main__":
    main()
