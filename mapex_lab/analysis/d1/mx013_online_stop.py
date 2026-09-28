#!/usr/bin/env python3
"""Frozen MX012 V2 online STOP_CONSIDER recognizer and truth-side evaluator."""
from __future__ import annotations

from dataclasses import dataclass, fields
from hashlib import sha256
from pathlib import Path
from typing import Optional
import math

import numpy as np

from mapex_lab.analysis.r004 import evaluate_topology_traversability as topo

FORBIDDEN = {
    "OracleStop_4", "OracleRemainingFraction_GT", "structural_gt",
    "final_map", "future_map", "fragmented",
    "reachable_pair_connectivity_retention", "reachable_future_free_retention",
    "lost_reachable_future_free_fraction",
}


@dataclass(frozen=True)
class OnlineDecision:
    run_id: str
    decision_id: int
    decision_index: int
    decision_time_s: float
    decision_progress: float
    r_hat: float
    r_evaluable: bool
    r_missing_reason: str
    topo_valid: bool
    topo_reason: str
    source_prediction_traversable: bool
    prediction_component_count: int
    source_component_area_m2: float
    total_prediction_traversable_area_m2: float
    source_component_fraction: float
    u_p95: float = math.nan
    u_mean: float = math.nan
    u_disagreement: float = math.nan
    low_u_25: Optional[bool] = None
    raw_map_sha256: str = ""
    prediction_member_sha256s: str = ""
    mean_map_sha256: str = ""
    source_mode: str = ""
    source_lower_time_s: float = math.nan
    source_upper_time_s: float = math.nan


@dataclass(frozen=True)
class RecognizerParams:
    tau_r: float
    k: int
    use_topology: bool = True
    use_u_veto: bool = False


def assert_truth_free_api() -> None:
    names = {f.name for f in fields(OnlineDecision)}
    overlap = names & FORBIDDEN
    if overlap:
        raise AssertionError(f"truth/future fields entered recognizer API: {sorted(overlap)}")


def replay_recognizer(decisions: list[OnlineDecision], params: RecognizerParams) -> list[dict]:
    """Run the one-shot frozen state machine. This function has no truth input."""
    assert_truth_free_api()
    if params.k not in {1, 2, 3, 4}:
        raise ValueError("K outside frozen set")
    streak = 0
    first_fire_seen = False
    out = []
    for d in decisions:
        before = streak
        r_condition = bool(d.r_evaluable and math.isfinite(d.r_hat) and d.r_hat <= params.tau_r)
        topology_condition = d.topo_valid if params.use_topology else True
        u_condition = True
        if params.use_u_veto:
            u_condition = d.low_u_25 is True
        qualify = r_condition and topology_condition and u_condition
        reset_reason = ""
        if qualify:
            streak += 1
        else:
            streak = 0
            if not d.r_evaluable or not math.isfinite(d.r_hat):
                reset_reason = f"R_MISSING:{d.r_missing_reason or 'UNKNOWN'}"
            elif not r_condition:
                reset_reason = "R_ABOVE_THRESHOLD"
            elif params.use_topology and not d.topo_valid:
                reset_reason = f"TOPO_INVALID:{d.topo_reason or 'UNKNOWN'}"
            elif params.use_u_veto and d.low_u_25 is not True:
                reset_reason = "U_MISSING_OR_VETO"
        first_fire = bool(not first_fire_seen and streak >= params.k)
        if first_fire:
            first_fire_seen = True
        out.append({
            **d.__dict__, "tau_r": params.tau_r, "k": params.k,
            "r_condition": r_condition, "topology_condition": topology_condition,
            "u_condition": u_condition, "qualify": qualify,
            "streak_before": before, "streak_after": streak,
            "reset_reason": reset_reason, "stop_consider": first_fire,
            "first_fire": first_fire,
        })
    return out


def sha256_file(path: Path) -> str:
    h = sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _scalar(z, name):
    return np.asarray(z[name]).item()


def online_masks(observed: np.ndarray, mean: np.ndarray, support: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    known = observed >= 0
    supported_unknown = (observed < 0) & support.astype(bool)
    domain = known | supported_unknown
    online_free = (observed == 0) | (supported_unknown & (mean <= 0.5))
    return domain, online_free


def compute_topology_feature(row: dict, run_dir: Path) -> dict:
    """Compute MX012 S23 TopoValid once per decision for reuse across sweeps."""
    raw_path = run_dir / str(row["raw_map"])
    mean_path = run_dir / str(row["mean_map"])
    if sha256_file(raw_path) != str(row["raw_map_sha256"]):
        raise RuntimeError(f"PROVENANCE_MISMATCH raw map {row['run_id']}:{row['decision_id']}")
    if sha256_file(mean_path) != str(row["mean_map_sha256"]):
        raise RuntimeError(f"PROVENANCE_MISMATCH mean map {row['run_id']}:{row['decision_id']}")
    for member in ("g1", "g2", "g3"):
        p = run_dir / str(row[f"{member}_map"])
        if sha256_file(p) != str(row[f"{member}_map_sha256"]):
            raise RuntimeError(f"PROVENANCE_MISMATCH {member} {row['run_id']}:{row['decision_id']}")

    with np.load(raw_path) as rz, np.load(mean_path) as mz:
        observed = np.asarray(rz["data"])
        h, w = observed.shape
        if (h, w) != (int(_scalar(mz, "source_height")), int(_scalar(mz, "source_width"))):
            raise RuntimeError("PROVENANCE_MISMATCH runtime/mean source shape")
        if not math.isclose(float(_scalar(rz, "resolution")), float(_scalar(mz, "resolution")), abs_tol=1e-12):
            raise RuntimeError("PROVENANCE_MISMATCH runtime/mean resolution")
        for key in ("origin_x", "origin_y"):
            if not math.isclose(float(_scalar(rz, key)), float(_scalar(mz, key)), abs_tol=1e-9):
                raise RuntimeError(f"PROVENANCE_MISMATCH runtime/mean {key}")
        if not math.isclose(float(_scalar(rz, "source_stamp_s")), float(_scalar(mz, "source_map_stamp_s")), abs_tol=1e-9):
            raise RuntimeError("PROVENANCE_MISMATCH runtime/mean timestamp")
        pt, pl = int(_scalar(mz, "pad_top")), int(_scalar(mz, "pad_left"))
        mean = np.asarray(mz["data"], dtype=float)[pt:pt+h, pl:pl+w]
        if mean.shape != observed.shape or not np.all(np.isfinite(mean)):
            raise RuntimeError("PROVENANCE_MISMATCH invalid mean crop")

        r_hat = float(row["RemainingFraction"]) if str(row["RemainingFraction"]) not in ("", "nan") else math.nan
        source_available = str(row["d1_source_available"]).lower() == "true"
        base = dict(topo_valid=False, topo_reason="", source_prediction_traversable=False,
                    prediction_component_count=0, source_component_area_m2=0.0,
                    total_prediction_traversable_area_m2=0.0, source_component_fraction=math.nan)
        if not math.isfinite(r_hat):
            return {**base, "topo_reason": "R_NOT_FINITE"}
        if not source_available:
            return {**base, "topo_reason": str(row.get("d1_reason") or "SOURCE_UNAVAILABLE")}
        source = topo.world_to_cell(float(row["d1_source_x"]), float(row["d1_source_y"]),
                                    float(_scalar(rz, "origin_x")), float(_scalar(rz, "origin_y")),
                                    float(_scalar(rz, "resolution")))
        if source != (int(float(row["d1_source_row"])), int(float(row["d1_source_col"]))):
            raise RuntimeError("PROVENANCE_MISMATCH source cell")
        support = np.ones(observed.shape, dtype=bool)
        domain, online_free = online_masks(observed, mean, support)
        pred_trav = topo.cspace(online_free, domain,
                                topo.collision_stencil(radius=0.189, resolution=float(_scalar(rz, "resolution"))))
        if not (0 <= source[0] < h and 0 <= source[1] < w):
            return {**base, "topo_reason": "SOURCE_OUTSIDE_ONLINE_GRID"}
        if not domain[source]:
            return {**base, "topo_reason": "SOURCE_OUTSIDE_ONLINE_DOMAIN"}
        if not pred_trav[source]:
            return {**base, "topo_reason": "SOURCE_NOT_PREDICTION_TRAVERSABLE"}
        reach = topo.reachable(pred_trav, source)
        _, comps = topo.components(pred_trav)
        source_n, total_n = int(reach.sum()), int(pred_trav.sum())
        resolution = float(_scalar(rz, "resolution"))
        if source_n == 0:
            return {**base, "topo_reason": "EMPTY_SOURCE_COMPONENT"}
        return dict(topo_valid=True, topo_reason="OK", source_prediction_traversable=True,
                    prediction_component_count=len(comps), source_component_area_m2=source_n*resolution**2,
                    total_prediction_traversable_area_m2=total_n*resolution**2,
                    source_component_fraction=source_n/total_n if total_n else math.nan)


def evaluate_run(trace: list[dict], truth: list[dict]) -> dict:
    """Truth-side join and frozen S24 accounting; never called by recognizer."""
    if len(trace) != len(truth):
        raise ValueError("trace/truth length mismatch")
    oracle_rows = [x for x in truth if x["oracle_stop_4"]]
    if len(oracle_rows) != 1:
        raise ValueError("truth must contain exactly one OracleStop_4")
    i_o = int(oracle_rows[0]["decision_index"])
    fires = [x for x in trace if x["first_fire"]]
    n = len(trace)
    if not fires:
        return dict(candidate_stop=None, oracle_stop=i_o, stop_status="NO_STOP", delay_decisions=None,
                    saved_decisions=0, positive_saving=False, saved_progress=0.0,
                    time_saved_s=None, delay_seconds=None, severe_false_stop_10=False)
    fire = fires[0]
    i_c = int(fire["decision_index"])
    truth_row = truth[i_c - 1]
    delay = i_c - i_o
    status = "PREMATURE" if delay < 0 else ("ON_TARGET" if delay == 0 else "LATE")
    return dict(candidate_stop=i_c, oracle_stop=i_o, stop_status=status, delay_decisions=delay,
                saved_decisions=n-i_c, positive_saving=(n-i_c)>0,
                saved_progress=1.0-float(fire["decision_progress"]),
                time_saved_s=float(trace[-1]["decision_time_s"])-float(fire["decision_time_s"]),
                delay_seconds=float(fire["decision_time_s"])-float(trace[i_o-1]["decision_time_s"]),
                severe_false_stop_10=float(truth_row["oracle_remaining_fraction_gt"]) > 0.10)
