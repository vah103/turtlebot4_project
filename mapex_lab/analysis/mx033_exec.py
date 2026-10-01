#!/usr/bin/env python3
"""MX034 executes the exact QA-accepted MX033 Method V2.

Bounded deterministic replay only. No model inference, simulation, Hospital,
MX028, engineering, deployment, or robot STOP.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import statistics
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path

SCRIPT = Path(__file__).resolve()
ANALYSIS = SCRIPT.parent
REPO = ANALYSIS.parent.parent
RESULTS = ANALYSIS / "mx033_exec_results"
INPUTS = ANALYSIS / "mx033_inputs"

METHOD = "86e17f061b3acee14b84d27da2f95b43c007e677"
METHOD_BLOB = "83e1d2038d7bd81618eb47fc65b6e4fa644fbfc6"
MX032_PACK = "9aefb2bbe0be8685c8e779f3def677b0b5054e82"
MX031_REV = "d90498e1a3ead2a0c170a3fee7c02a1bc9526cca"

PRIMARY = ANALYSIS / "mx031_exec_results" / "MX031_PER_DECISION_REPLAY.csv"
LORO = ANALYSIS / "mx031_exec_results" / "MX031_LORO_SELECTED_RULES.csv"
FULLSEL = ANALYSIS / "mx031_exec_results" / "MX031_FULL_FIT_SELECTION.csv"
MX026 = ANALYSIS / "mx026_exec_results" / "MX026_PUR_PER_DECISION.csv"
MX027 = ANALYSIS / "mx027_exec_results" / "MX027_IG_COVERAGE_PER_DECISION.csv"
MX027SRC = ANALYSIS / "mx027_exec_results" / "MX027_IG_COVERAGE_SOURCE_PARITY.csv"
MX025SRC = ANALYSIS / "mx025_exec_results" / "MX025_R_MAP_SOURCE_PARITY.csv"
ORACLE = ANALYSIS / "d1" / "results" / "oracle_stop_retro_v1" / "oracle_decisions.csv"
CASEBOOK = INPUTS / "MX032_RUN_EXCEPTION_CASEBOOK.csv"

EXPECTED_BLOBS = {
    PRIMARY: "daf11687221080790e20227f2bf7b7406476298e",
    LORO: "cc0d4230a997e50b00f5c776537d59af0815769f",
    MX026: "8e73f4441b1c852f90d801c20f993216bbd1e78a",
    MX027: "dab247d44578d728c44ffc2245ee306d7375d9cb",
    MX027SRC: "629560dba455a91c95f2f636d4d532016fd5a4c1",
    MX025SRC: "0a3af2267f37805e7fc7e1964d89f68d3d49013a",
    ORACLE: "9ae8f0479cf37a2b6aa9426a158f120efdc655cf",
    CASEBOOK: "f465036fbb08de7f790af8943d56d8ad59f1dcb0",
}
RUNS = tuple(f"mpx_{i:03d}" for i in range(1, 11))
TAU_IGS = (0.00, 0.01, 0.05, 0.10)
CS = (0, 1, 2)
Q_GRID = tuple((t, c) for t in TAU_IGS for c in CS)

FROZEN_BACKBONE = {
    "mpx_001": (0.06, 1),
    "mpx_002": (0.10, 2),
    "mpx_003": (0.05, 1),
    "mpx_004": (0.08, 2),
    "mpx_005": (0.05, 1),
    "mpx_006": (0.05, 1),
    "mpx_007": (0.05, 1),
    "mpx_008": (0.05, 1),
    "mpx_009": (0.08, 2),
    "mpx_010": (0.05, 1),
}
ACCEPTED_BASE_STOPS = {
    "mpx_001": 20, "mpx_002": 15, "mpx_003": 22, "mpx_004": 19,
    "mpx_005": 19, "mpx_006": 23, "mpx_007": 19, "mpx_008": 20,
    "mpx_009": 23, "mpx_010": 23,
}
ACCEPTED_FULL_FIRES = {
    "mpx_001": 23, "mpx_002": 18, "mpx_003": 22, "mpx_004": 16,
    "mpx_005": 19, "mpx_006": 23, "mpx_007": 19, "mpx_008": 20,
    "mpx_009": 19, "mpx_010": 23,
}


def git_blob(path: Path) -> str:
    return subprocess.check_output(
        ["git", "hash-object", str(path)], cwd=REPO, text=True
    ).strip()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def index(rows, run_field="run_id", decision_field="decision_id"):
    out = {}
    for r in rows:
        k = (r[run_field], int(r[decision_field]))
        if k in out:
            raise RuntimeError(f"duplicate key {k}")
        out[k] = r
    return out


def blank(v) -> bool:
    return v is None or str(v).strip() == ""


def finite(v) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def f(v, default=math.nan) -> float:
    return float(v) if finite(v) else default


def i(v, default=None):
    if blank(v):
        return default
    try:
        x = float(v)
        if not math.isfinite(x) or int(x) != x:
            return default
        return int(x)
    except (ValueError, TypeError):
        return default


def b(v) -> bool:
    return str(v).strip().lower() in {"1", "true", "yes"}


def med(vals):
    a = [float(x) for x in vals if finite(x)]
    return statistics.median(a) if a else math.nan


def avg(vals):
    a = [float(x) for x in vals if finite(x)]
    return sum(a) / len(a) if a else math.nan


def same_num(a, z, tol=1e-12):
    if not finite(a) and not finite(z):
        return True
    return finite(a) and finite(z) and math.isclose(float(a), float(z), rel_tol=0.0, abs_tol=tol)


def clean(v):
    if v is None:
        return ""
    if isinstance(v, float):
        return v if math.isfinite(v) else ""
    if isinstance(v, bool):
        return int(v)
    return v


def write_csv(path: Path, rows: list[dict]):
    keys, seen = [], set()
    for r in rows:
        for k in r:
            if k not in seen:
                seen.add(k)
                keys.append(k)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fobj:
        w = csv.DictWriter(fobj, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: clean(r.get(k, "")) for k in keys})


def json_clean(v):
    if isinstance(v, dict):
        return {str(k): json_clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [json_clean(x) for x in v]
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    return v


def write_json(path: Path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(json_clean(obj), indent=2, sort_keys=True) + "\n", encoding="utf-8")


def source_and_value_parity(primary, mx026, mx027, mx027src, oracle):
    failures = []
    pidx = index(primary)
    i26 = index(mx026)
    i27 = index(mx027)
    i27s = index(mx027src)
    io = index(oracle)
    keys = set(pidx)
    if len(keys) != 365:
        failures.append(f"PRIMARY_KEY_COUNT={len(keys)}")
    for name, idx_ in (("MX026", i26), ("MX027", i27), ("MX027SRC", i27s), ("ORACLE", io)):
        if set(idx_) != keys:
            failures.append(f"{name}_KEY_MEMBERSHIP_MISMATCH")

    for k in sorted(keys):
        p, r26, r27, rs, o = pidx[k], i26[k], i27[k], i27s[k], io[k]
        comparisons = [
            ("R_map", p["R_map"], r26["R_MapRemainingFraction"]),
            ("R_available", p["R_available"], r26["R_map_evaluable"]),
            ("IG_selected", p["IG_selected"], r27["IG_selected"]),
            ("IG_evaluable", p["IG_evaluable"], r27["IG_evaluable"]),
            ("DeltaKnownArea_m2", p["DeltaKnownArea_m2"], r27["DeltaKnownArea_m2"]),
            ("KnownAreaRate_m2_s", p["KnownAreaRate_m2_s"], r27["KnownAreaRate_m2_s"]),
            ("raw_resolution", p["raw_resolution"], rs["raw_resolution"]),
            ("OracleRemainingFraction_GT", p["OracleRemainingFraction_GT"], o["OracleRemainingFraction_GT"]),
        ]
        for field, a, z in comparisons:
            if not same_num(a, z):
                failures.append(f"{k}:{field}")
        strings = [
            ("R_reason", p["R_reason"], r26["R_map_reason"]),
            ("IG_reason", p["IG_reason"], r27["IG_reason"]),
            ("Coverage_reason", p["Coverage_reason"], r27["Coverage_reason"]),
            ("DeltaKnownArea_reason", p["DeltaKnownArea_reason"], r27["DeltaKnownArea_reason"]),
            ("KnownAreaRate_reason", p["KnownAreaRate_reason"], r27["KnownAreaRate_reason"]),
        ]
        for field, a, z in strings:
            if str(a).strip() != str(z).strip():
                failures.append(f"{k}:{field}")
        if i(p["hard_replay_integrity_failure"], 1) != 0:
            failures.append(f"{k}:hard_replay_integrity_failure")
    return failures


def prepare_rows(primary):
    by_run = defaultdict(list)
    for r0 in primary:
        r = dict(r0)
        run = r["run_id"]
        did = int(r["decision_id"])
        r["_decision_id"] = did
        r["_decision_index"] = int(r["decision_index"])
        r["_decision_count"] = int(r["decision_count"])
        r["_progress"] = float(r["normalized_progress"])
        r["_time"] = float(r["decision_time_s"])
        r["_R_available"] = b(r["R_available"]) and finite(r["R_map"])
        r["_R"] = f(r["R_map"])
        r["_topo"] = b(r["TopoValid"])
        state = r["FrontierState"]
        r["_state"] = state

        if state == "A_NO_RUNTIME_SELECTED_FRONTIER":
            if not blank(r["IG_selected"]) and str(r["IG_selected"]).strip().lower() != "nan":
                raise RuntimeError(f"{run} d{did}: State A IG must be NA")
            r["_ig_peak"] = math.nan
            r["_ig_rel"] = math.nan
        elif state == "B_SELECTED_FRONTIER_AVAILABLE":
            if not b(r["IG_evaluable"]) or not finite(r["IG_selected"]) or float(r["IG_selected"]) < 0:
                raise RuntimeError(f"{run} d{did}: invalid State-B IG")
        elif state == "C_FRONTIER_INDETERMINATE":
            if not blank(r["IG_selected"]) and str(r["IG_selected"]).strip().lower() != "nan":
                raise RuntimeError(f"{run} d{did}: State C IG must be NA")
            r["_ig_peak"] = math.nan
            r["_ig_rel"] = math.nan
        else:
            raise RuntimeError(f"{run} d{did}: State D/unknown frontier state {state}")
        by_run[run].append(r)

    if set(by_run) != set(RUNS):
        raise RuntimeError("run membership mismatch")

    for run in RUNS:
        rr = sorted(by_run[run], key=lambda x: x["_decision_index"])
        ids = [x["_decision_index"] for x in rr]
        if ids != list(range(1, len(rr) + 1)):
            raise RuntimeError(f"{run}: decision_index not contiguous")
        peak = None
        for r in rr:
            if r["_state"] == "B_SELECTED_FRONTIER_AVAILABLE":
                ig = float(r["IG_selected"])
                peak = ig if peak is None else max(peak, ig)
                r["_ig_peak"] = peak
                if peak > 0:
                    r["_ig_rel"] = ig / peak
                elif ig == 0:
                    r["_ig_rel"] = 0.0
                else:
                    raise RuntimeError(f"{run} d{r['_decision_id']}: impossible IG peak")
        by_run[run] = rr
    return by_run


def frontier_clear(row, tau_ig):
    st = row["_state"]
    if st == "A_NO_RUNTIME_SELECTED_FRONTIER":
        return True, "STATE_A_SEMANTIC_NO_FRONTIER"
    if st == "B_SELECTED_FRONTIER_AVAILABLE":
        return row["_ig_rel"] <= tau_ig, (
            "STATE_B_RELATIVE_IG_CLEAR" if row["_ig_rel"] <= tau_ig else "STATE_B_RELATIVE_IG_ACTIVE"
        )
    if st == "C_FRONTIER_INDETERMINATE":
        return False, "STATE_C_FRONTIER_INDETERMINATE"
    raise RuntimeError("frontier integrity failure")


def coverage_clear(row, c):
    if not b(row["Coverage_evaluable"]):
        return False, row["Coverage_reason"].strip() or "COVERAGE_UNAVAILABLE"
    reasons = [
        row["Coverage_reason"].strip(),
        row["DeltaKnownArea_reason"].strip(),
        row["KnownAreaRate_reason"].strip(),
    ]
    if any(reasons):
        if "FIRST_DECISION_NO_PREDECESSOR" in reasons:
            return False, "FIRST_DECISION_NO_PREDECESSOR"
        return False, "|".join(x for x in reasons if x)
    if not finite(row["DeltaKnownArea_m2"]) or not finite(row["KnownAreaRate_m2_s"]):
        raise RuntimeError(f"{row['run_id']} d{row['_decision_id']}: malformed coverage finite state")
    delta = float(row["DeltaKnownArea_m2"])
    rate = float(row["KnownAreaRate_m2_s"])
    if delta < 0 or rate < 0:
        return False, "SIGNED_COVERAGE_REGRESSION_VETO"
    res = float(row["raw_resolution"])
    if not math.isfinite(res) or res <= 0:
        raise RuntimeError(f"{row['run_id']} d{row['_decision_id']}: invalid resolution")
    budget = c * res * res
    return delta <= budget, ("COVERAGE_CLEAR" if delta <= budget else "POSITIVE_GAIN_ABOVE_CELL_BUDGET")


def replay(rr, tau_r, K, tau_ig=None, c=None, base=False):
    streak = 0
    stop = None
    trace = []
    for row in rr:
        rsmall = row["_R_available"] and row["_R"] <= tau_r
        topo_clear = row["_topo"]
        if base:
            fclear = None
            cclear = None
            qualify = rsmall and topo_clear
            freason = ""
            creason = ""
        else:
            fclear, freason = frontier_clear(row, tau_ig)
            cclear, creason = coverage_clear(row, c)
            qualify = rsmall and topo_clear and fclear and cclear
        streak = streak + 1 if qualify else 0
        fire = stop is None and streak >= K
        if fire:
            stop = row
        trace.append({
            "row": row,
            "RSmall": rsmall,
            "TopoClear": topo_clear,
            "FrontierClear": fclear,
            "FrontierClear_reason": freason,
            "CoverageClear": cclear,
            "CoverageClear_reason": creason,
            "Qualify": qualify,
            "streak": streak,
            "fire": fire,
        })
    return stop, trace


def score(stop, rr):
    oracle = int(rr[0]["OracleStop_4"])
    n = len(rr)
    if stop is None:
        return {
            "StopStatus": "NO_STOP", "CandidateStop": "", "OracleStop_4": oracle,
            "DelayDecisions": math.nan, "PrematureStop": False, "PrematureByDecisions": 0,
            "SevereFalseStop10": False, "SavedDecisions": 0, "PositiveSaving": False,
            "SavedProgress": 0.0, "SavedTime_s": math.nan,
            "OracleRemainingFraction_GT_at_stop": math.nan,
        }
    ic = stop["_decision_index"]
    delay = ic - oracle
    status = "PREMATURE" if delay < 0 else ("ON_TARGET" if delay == 0 else "LATE")
    oracle_remaining = float(stop["OracleRemainingFraction_GT"])
    saved = n - ic
    return {
        "StopStatus": status, "CandidateStop": stop["_decision_id"], "OracleStop_4": oracle,
        "DelayDecisions": delay, "PrematureStop": delay < 0,
        "PrematureByDecisions": max(0, -delay),
        "SevereFalseStop10": oracle_remaining > 0.10,
        "SavedDecisions": saved, "PositiveSaving": saved > 0,
        "SavedProgress": 1.0 - stop["_progress"],
        "SavedTime_s": rr[-1]["_time"] - stop["_time"],
        "OracleRemainingFraction_GT_at_stop": oracle_remaining,
    }


def aggregate(outcomes, n):
    fired = sum(not blank(o["CandidateStop"]) for o in outcomes)
    positive = sum(bool(o["PositiveSaving"]) for o in outcomes)
    prem = sum(bool(o["PrematureStop"]) for o in outcomes)
    severe = sum(bool(o["SevereFalseStop10"]) for o in outcomes)
    delays = [
        o["DelayDecisions"] for o in outcomes
        if o["StopStatus"] in {"ON_TARGET", "LATE"} and finite(o["DelayDecisions"])
    ]
    return {
        "truth_valid_run_count": n,
        "required_fired_run_count": math.ceil(0.80 * n),
        "required_positive_saving_run_count": math.ceil(0.70 * n),
        "fired_run_count": fired,
        "positive_saving_run_count": positive,
        "StopCoverage": fired / n,
        "PositiveSavingCoverage": positive / n,
        "premature_stop_count": prem,
        "SevereFalseStop10_count": severe,
        "median_nonpremature_DelayDecisions": med(delays),
        "mean_nonpremature_DelayDecisions": avg(delays),
        "nonpremature_delay_support": len(delays),
        "MeanSavedProgress": sum(float(o["SavedProgress"]) for o in outcomes) / n,
        "integrity_failure_count": 0,
    }


def admissible(a):
    return (
        a["premature_stop_count"] == 0
        and a["SevereFalseStop10_count"] == 0
        and a["fired_run_count"] >= a["required_fired_run_count"]
        and a["positive_saving_run_count"] >= a["required_positive_saving_run_count"]
        and finite(a["median_nonpremature_DelayDecisions"])
        and a["median_nonpremature_DelayDecisions"] <= 3
        and a["integrity_failure_count"] == 0
    )


def selector_key(row):
    return (
        float(row["median_nonpremature_DelayDecisions"]),
        float(row["mean_nonpremature_DelayDecisions"]),
        -int(row["positive_saving_run_count"]),
        -int(row["fired_run_count"]),
        float(row["tau_IG"]),
        int(row["c"]),
    )


def parity_outer(by_run, loro_rows):
    lmap = {r["heldout_run"]: r for r in loro_rows}
    failures = []
    rows = []
    for run in RUNS:
        accepted = lmap[run]
        tau_r, K = FROZEN_BACKBONE[run]
        stop, _ = replay(by_run[run], tau_r, K, base=True)
        out = score(stop, by_run[run])
        checks = {
            "family_F0": accepted["selected_family"] == "F0",
            "tau_R": same_num(float(accepted["selected_tau_R_pct"]) / 100.0, tau_r),
            "K": i(accepted["selected_K"]) == K,
            "c_NA": blank(accepted["selected_c"]),
            "CandidateStop": i(accepted["heldout_CandidateStop"]) == i(out["CandidateStop"]),
            "required_stop": i(out["CandidateStop"]) == ACCEPTED_BASE_STOPS[run],
            "Oracle": i(accepted["heldout_OracleStop_4"]) == out["OracleStop_4"],
            "Delay": same_num(accepted["heldout_DelayDecisions"], out["DelayDecisions"]),
            "Premature": b(accepted["heldout_PrematureStop"]) == out["PrematureStop"],
            "PrematureBy": i(accepted["heldout_PrematureByDecisions"]) == out["PrematureByDecisions"],
            "Severe": b(accepted["heldout_SevereFalseStop10"]) == out["SevereFalseStop10"],
        }
        passed = all(checks.values())
        if not passed:
            failures.append(f"{run}:{[k for k,v in checks.items() if not v]}")
        rows.append({"run_id": run, "parity_pass": int(passed), **out})
    return failures, rows


def fullfit_parity(primary, by_run):
    failures = []
    first_fires = {}
    for run in RUNS:
        stop, trace = replay(by_run[run], 0.05, 1, base=True)
        first_fires[run] = i(stop["_decision_id"]) if stop else None
        if first_fires[run] != ACCEPTED_FULL_FIRES[run]:
            failures.append(f"{run}:first_fire")
        for tr in trace:
            r = tr["row"]
            expected_reset = ""
            if tr["Qualify"]:
                expected_reset = ""
            elif not r["_R_available"]:
                expected_reset = "R_UNAVAILABLE"
            elif not tr["RSmall"]:
                expected_reset = "R_ABOVE_TAU"
            elif not r["_topo"]:
                expected_reset = "TOPO_INVALID:" + str(r["TopoValid_reason"])
            else:
                expected_reset = "QUALIFY_FALSE"
            checks = [
                r["FullFit_family"] == "F0",
                same_num(r["FullFit_tau_R_pct"], 5.0),
                i(r["FullFit_K"]) == 1,
                blank(r["FullFit_c"]),
                b(r["FullFit_RCondition"]) == tr["RSmall"],
                b(r["FullFit_Qualify"]) == tr["Qualify"],
                i(r["FullFit_streak"]) == tr["streak"],
                b(r["FullFit_first_fire_here"]) == tr["fire"],
                str(r["FullFit_reset_reason"]) == expected_reset,
            ]
            if not all(checks):
                failures.append(f"{run}:d{r['_decision_id']}:row_trace")
    return failures, first_fires


def audit_adversarial(by_run, casebook):
    cb = {(r["run_id"], int(r["decision_id"])): r for r in casebook}
    audits = []

    def row(run, did):
        return next(x for x in by_run[run] if x["_decision_id"] == did)

    # 1: mpx_001 d20
    r = row("mpx_001", 20)
    fvals = [frontier_clear(r, t)[0] for t in TAU_IGS]
    cvals = [coverage_clear(r, c)[0] for c in CS]
    pass1 = (
        r["_topo"] and r["_state"] == "B_SELECTED_FRONTIER_AVAILABLE"
        and all(v is False for v in fvals) and all(v is False for v in cvals)
        and r["_ig_rel"] > max(TAU_IGS)
    )
    audits.append({
        "case_family": "mpx_001_d20_premature_context", "run_id":"mpx_001","decision_id":20,
        "semantic_pass":int(pass1),"IGRel":r["_ig_rel"],"DeltaKnownArea_m2":r["DeltaKnownArea_m2"],
        "KnownAreaRate_m2_s":r["KnownAreaRate_m2_s"],
        "required_behavior":"FrontierClear=false all tau_IG; CoverageClear=false all c; veto ON",
    })

    # 2: mpx_002 d15
    r = row("mpx_002", 15)
    fvals = [frontier_clear(r, t)[0] for t in TAU_IGS]
    cvals = [coverage_clear(r, c)[0] for c in CS]
    pass2 = (
        r["_topo"] and r["_state"] == "B_SELECTED_FRONTIER_AVAILABLE"
        and all(v is False for v in fvals) and all(v is False for v in cvals)
        and r["_ig_rel"] > max(TAU_IGS)
    )
    audits.append({
        "case_family":"mpx_002_d15_premature_context","run_id":"mpx_002","decision_id":15,
        "semantic_pass":int(pass2),"IGRel":r["_ig_rel"],"DeltaKnownArea_m2":r["DeltaKnownArea_m2"],
        "KnownAreaRate_m2_s":r["KnownAreaRate_m2_s"],
        "required_behavior":"FrontierClear=false all tau_IG; CoverageClear=false all c; veto ON; K2 cannot bypass",
    })

    # 3: mpx_005 d16 retrospective fragmentation remains evaluator-only.
    r = row("mpx_005", 16); z = cb[("mpx_005",16)]
    pass3 = r["_topo"] and str(z["MX023_fragmented"]) in {"1","1.0"} and same_num(z["MX023_ReachableFutureFreeRetention"], 0.8368372093023256)
    audits.append({
        "case_family":"mpx_005_d16_fragmentation_context","run_id":"mpx_005","decision_id":16,
        "semantic_pass":int(pass3),"runtime_TopoValid":int(r["_topo"]),
        "retrospective_fragmented":z["MX023_fragmented"],
        "retrospective_future_free_retention":z["MX023_ReachableFutureFreeRetention"],
        "required_behavior":"retrospective fragmentation evaluator-only; no leakage into online veto",
    })

    # 4: mpx_010 d16 source topology.
    r = row("mpx_010",16); z=cb[("mpx_010",16)]
    pass4 = (
        not r["_topo"] and str(z["MX023_source_available"]) in {"1","1.0"}
        and str(z["MX023_source_prediction_traversable"]) in {"0","0.0"}
        and same_num(z["MX023_ReachableFutureFreeRetention"],0.0)
    )
    audits.append({
        "case_family":"mpx_010_d16_adverse_source_topology","run_id":"mpx_010","decision_id":16,
        "semantic_pass":int(pass4),"runtime_TopoValid":int(r["_topo"]),
        "retrospective_source_available":z["MX023_source_available"],
        "retrospective_source_prediction_traversable":z["MX023_source_prediction_traversable"],
        "required_behavior":"TopoClear=false and reset; retrospective retention not online",
    })

    # 5: State A no-frontier.
    r=row("mpx_001",30)
    pass5 = (
        r["_state"]=="A_NO_RUNTIME_SELECTED_FRONTIER"
        and all(frontier_clear(r,t)[0] for t in TAU_IGS)
        and all(coverage_clear(r,c)[0] for c in CS)
        and not r["_topo"]
        and (blank(r["IG_selected"]) or str(r["IG_selected"]).lower()=="nan")
    )
    audits.append({
        "case_family":"late_semantic_no_frontier_mpx001_d30","run_id":"mpx_001","decision_id":30,
        "semantic_pass":int(pass5),"FrontierState":r["_state"],"runtime_TopoValid":int(r["_topo"]),
        "required_behavior":"FrontierClear=true, IG stays NA, CoverageClear=true all c, TopoValid=0 blocks STOP",
    })

    # 6: State C planner suppressed.
    r=row("mpx_004",30)
    pass6 = r["_state"]=="C_FRONTIER_INDETERMINATE" and all(not frontier_clear(r,t)[0] for t in TAU_IGS)
    audits.append({
        "case_family":"planner_suppressed_mpx004_d30","run_id":"mpx_004","decision_id":30,
        "semantic_pass":int(pass6),"FrontierState":r["_state"],
        "required_behavior":"FrontierClear=false fail-closed; never reinterpret as no-frontier/IG=0",
    })

    # 7: signed Coverage regression.
    r=row("mpx_010",31)
    cvals=[coverage_clear(r,c) for c in CS]
    pass7 = (
        float(r["DeltaKnownArea_m2"]) < 0 and float(r["KnownAreaRate_m2_s"]) < 0
        and all(not x[0] and x[1]=="SIGNED_COVERAGE_REGRESSION_VETO" for x in cvals)
    )
    audits.append({
        "case_family":"negative_coverage_mpx010_d31","run_id":"mpx_010","decision_id":31,
        "semantic_pass":int(pass7),"DeltaKnownArea_m2":r["DeltaKnownArea_m2"],
        "KnownAreaRate_m2_s":r["KnownAreaRate_m2_s"],
        "required_behavior":"signed values preserved; CoverageClear=false all c; never clamp",
    })
    return audits


def main():
    RESULTS.mkdir(parents=True, exist_ok=True)

    blob_fail = []
    for path, expected in EXPECTED_BLOBS.items():
        actual = git_blob(path)
        if actual != expected:
            blob_fail.append(f"{path.relative_to(REPO)}:{actual}!={expected}")

    primary = read_csv(PRIMARY)
    loro = read_csv(LORO)
    mx026 = read_csv(MX026)
    mx027 = read_csv(MX027)
    mx027src = read_csv(MX027SRC)
    oracle = read_csv(ORACLE)
    casebook = read_csv(CASEBOOK)

    parity_fail = source_and_value_parity(primary, mx026, mx027, mx027src, oracle)
    by_run = prepare_rows(primary)

    outer_parity_fail, outer_base_rows = parity_outer(by_run, loro)
    full_parity_fail, full_first_fires = fullfit_parity(primary, by_run)
    hard_integrity_failures = len(blob_fail) + len(parity_fail) + len(outer_parity_fail) + len(full_parity_fail)

    # Training-only selector inside each outer fold.
    training_grid = []
    loro_selected = []
    per_decision_output = []
    channel_rows = []
    monotonicity_violations = 0
    selected_by_run = {}

    for held in RUNS:
        tau_r, K = FROZEN_BACKBONE[held]
        train_runs = [r for r in RUNS if r != held]
        candidates = []
        for tau_ig, c in Q_GRID:
            outcomes = []
            for run in train_runs:
                stop, _ = replay(by_run[run], tau_r, K, tau_ig, c, base=False)
                outcomes.append(score(stop, by_run[run]))
            a = aggregate(outcomes, 9)
            ok = admissible(a)
            row = {
                "heldout_run":held,"training_runs":";".join(train_runs),
                "tau_R":tau_r,"K":K,"tau_IG":tau_ig,"c":c,**a,
                "admissible":int(ok),"selector_rank":"",
                "reject_reason":""
            }
            reasons=[]
            if a["premature_stop_count"]>0: reasons.append("PREMATURE_STOP_GT0")
            if a["SevereFalseStop10_count"]>0: reasons.append("SEVERE_FALSE_STOP10_GT0")
            if a["fired_run_count"]<8: reasons.append("FIRED_RUN_COUNT_LT8")
            if a["positive_saving_run_count"]<7: reasons.append("POSITIVE_SAVING_RUN_COUNT_LT7")
            if not finite(a["median_nonpremature_DelayDecisions"]): reasons.append("NO_NONPREMATURE_DELAY_SUPPORT")
            elif a["median_nonpremature_DelayDecisions"]>3: reasons.append("MEDIAN_DELAY_GT3")
            row["reject_reason"]="|".join(reasons)
            candidates.append(row)
        ranked = sorted([x for x in candidates if x["admissible"]==1], key=selector_key)
        for rank, x in enumerate(ranked, 1):
            x["selector_rank"]=rank
        selected = ranked[0] if ranked else None
        training_grid.extend(candidates)

        accepted_loro = next(x for x in loro if x["heldout_run"]==held)
        base_stop, base_trace = replay(by_run[held], tau_r, K, base=True)
        base_score = score(base_stop, by_run[held])
        if selected is None:
            selected_by_run[held] = None
            loro_selected.append({
                "heldout_run":held,"fold_status":"NO_ADMISSIBLE_MX033_VETO",
                "tau_R":tau_r,"K":K,"selected_tau_IG":"","selected_c":"",
                "heldout_status":"NO_RULE","heldout_CandidateStop":"",
                "OracleStop_4":int(by_run[held][0]["OracleStop_4"]),
                "DelayDecisions":"","PrematureStop":0,"PrematureByDecisions":0,
                "SevereFalseStop10":0,"SavedDecisions":0,"PositiveSaving":0,
                "SavedProgress":0.0,"SavedTime_s":"",
                "paired_base_CandidateStop":base_score["CandidateStop"],
                "paired_base_DelayDecisions":base_score["DelayDecisions"],
                "veto_delay_vs_paired_base":"",
            })
            continue

        tau_ig, c = float(selected["tau_IG"]), int(selected["c"])
        selected_by_run[held]=(tau_ig,c)
        stop, trace = replay(by_run[held], tau_r, K, tau_ig, c, base=False)
        out = score(stop, by_run[held])
        if stop is not None and base_stop is not None and stop["_decision_index"] < base_stop["_decision_index"]:
            monotonicity_violations += 1
        loro_selected.append({
            "heldout_run":held,"fold_status":"ADMISSIBLE",
            "tau_R":tau_r,"K":K,"selected_tau_IG":tau_ig,"selected_c":c,
            "training_median_nonpremature_DelayDecisions":selected["median_nonpremature_DelayDecisions"],
            "training_mean_nonpremature_DelayDecisions":selected["mean_nonpremature_DelayDecisions"],
            "training_fired_run_count":selected["fired_run_count"],
            "training_positive_saving_run_count":selected["positive_saving_run_count"],
            "heldout_status":out["StopStatus"],"heldout_CandidateStop":out["CandidateStop"],
            "OracleStop_4":out["OracleStop_4"],"DelayDecisions":out["DelayDecisions"],
            "PrematureStop":out["PrematureStop"],"PrematureByDecisions":out["PrematureByDecisions"],
            "SevereFalseStop10":out["SevereFalseStop10"],"SavedDecisions":out["SavedDecisions"],
            "PositiveSaving":out["PositiveSaving"],"SavedProgress":out["SavedProgress"],
            "SavedTime_s":out["SavedTime_s"],
            "OracleRemainingFraction_GT_at_stop":out["OracleRemainingFraction_GT_at_stop"],
            "paired_base_CandidateStop":base_score["CandidateStop"],
            "paired_base_DelayDecisions":base_score["DelayDecisions"],
            "veto_delay_vs_paired_base":(
                int(out["CandidateStop"])-int(base_score["CandidateStop"])
                if not blank(out["CandidateStop"]) and not blank(base_score["CandidateStop"]) else ""
            ),
        })

        # Per-decision row for held-out selected q.
        for tr in trace:
            r=tr["row"]
            veto_reasons=[]
            if not tr["FrontierClear"]: veto_reasons.append(tr["FrontierClear_reason"])
            if not tr["CoverageClear"]: veto_reasons.append(tr["CoverageClear_reason"])
            if not tr["TopoClear"]: veto_reasons.append("TOPO_NOT_CLEAR")
            if not tr["RSmall"]: veto_reasons.append("R_NOT_SMALL_OR_UNAVAILABLE")
            per_decision_output.append({
                "run_id":held,"decision_id":r["_decision_id"],"decision_index":r["_decision_index"],
                "decision_count":r["_decision_count"],"normalized_progress":r["_progress"],
                "tau_R":tau_r,"K":K,"selected_tau_IG":tau_ig,"selected_c":c,
                "R_available":int(r["_R_available"]),"R_map":r["_R"],"RSmall":int(tr["RSmall"]),
                "TopoValid":int(r["_topo"]),"TopoValid_reason":r["TopoValid_reason"],"TopoClear":int(tr["TopoClear"]),
                "FrontierState":r["_state"],"IG_evaluable":r["IG_evaluable"],"IG_selected":r["IG_selected"],
                "IGPeakSoFar":r["_ig_peak"],"IGRel":r["_ig_rel"],
                "FrontierClear":int(tr["FrontierClear"]),"FrontierClear_reason":tr["FrontierClear_reason"],
                "Coverage_evaluable":r["Coverage_evaluable"],"DeltaKnownArea_m2":r["DeltaKnownArea_m2"],
                "KnownAreaRate_m2_s":r["KnownAreaRate_m2_s"],"raw_resolution":r["raw_resolution"],
                "CoverageClear":int(tr["CoverageClear"]),"CoverageClear_reason":tr["CoverageClear_reason"],
                "ActiveExplorationVeto":int(not (tr["FrontierClear"] and tr["CoverageClear"])),
                "veto_reason":"|".join(veto_reasons),"Qualify":int(tr["Qualify"]),
                "streak":tr["streak"],"first_fire_here":int(tr["fire"]),
                "OracleStop_4":r["OracleStop_4"],"OracleRemainingFraction_GT":r["OracleRemainingFraction_GT"],
            })

        # Held-out paired-base channel attribution using selected q.
        if base_stop is not None:
            btr = next(x for x in trace if x["row"]["_decision_id"]==base_stop["_decision_id"])
            frontier_veto = not btr["FrontierClear"]
            coverage_veto = not btr["CoverageClear"]
            channel_rows.append({
                "heldout_run":held,"paired_base_stop":base_stop["_decision_id"],
                "selected_tau_IG":tau_ig,"selected_c":c,
                "FrontierState":base_stop["_state"],"IGRel":base_stop["_ig_rel"],
                "DeltaKnownArea_m2":base_stop["DeltaKnownArea_m2"],
                "KnownAreaRate_m2_s":base_stop["KnownAreaRate_m2_s"],
                "FrontierWouldVeto":int(frontier_veto),
                "CoverageWouldVeto":int(coverage_veto),
                "BothWouldVeto":int(frontier_veto and coverage_veto),
                "NeitherWouldVeto":int(not frontier_veto and not coverage_veto),
            })

    # Ensure one per-decision selected-q trace per run. For NO_RULE folds, emit semantic rows with q absent.
    emitted_runs={r["run_id"] for r in per_decision_output}
    for run in RUNS:
        if run in emitted_runs: continue
        tau_r,K=FROZEN_BACKBONE[run]
        for r in by_run[run]:
            per_decision_output.append({
                "run_id":run,"decision_id":r["_decision_id"],"decision_index":r["_decision_index"],
                "decision_count":r["_decision_count"],"normalized_progress":r["_progress"],
                "tau_R":tau_r,"K":K,"selected_tau_IG":"","selected_c":"",
                "R_available":int(r["_R_available"]),"R_map":r["_R"],
                "TopoValid":int(r["_topo"]),"TopoValid_reason":r["TopoValid_reason"],
                "FrontierState":r["_state"],"IG_evaluable":r["IG_evaluable"],"IG_selected":r["IG_selected"],
                "IGPeakSoFar":r["_ig_peak"],"IGRel":r["_ig_rel"],
                "Coverage_evaluable":r["Coverage_evaluable"],"DeltaKnownArea_m2":r["DeltaKnownArea_m2"],
                "KnownAreaRate_m2_s":r["KnownAreaRate_m2_s"],"raw_resolution":r["raw_resolution"],
                "veto_reason":"NO_RULE_NO_Q_APPLIED","Qualify":"","streak":"","first_fire_here":"",
                "OracleStop_4":r["OracleStop_4"],"OracleRemainingFraction_GT":r["OracleRemainingFraction_GT"],
            })
    per_decision_output.sort(key=lambda r:(r["run_id"],int(r["decision_index"])))

    # Full-development descriptive selector, fixed F0 5% K1.
    full_rows=[]
    full_ranked=[]
    for tau_ig,c in Q_GRID:
        outcomes=[]
        for run in RUNS:
            stop,_=replay(by_run[run],0.05,1,tau_ig,c,base=False)
            outcomes.append(score(stop,by_run[run]))
        a=aggregate(outcomes,10)
        ok=admissible(a)
        row={"status":"CANDIDATE","tau_R":0.05,"K":1,"tau_IG":tau_ig,"c":c,**a,"admissible":int(ok),"selector_rank":"","selected_full_fit":0}
        full_rows.append(row)
    full_ranked=sorted([x for x in full_rows if x["admissible"]==1],key=selector_key)
    for rank,x in enumerate(full_ranked,1):
        x["selector_rank"]=rank
    q_full=None
    if full_ranked:
        full_ranked[0]["selected_full_fit"]=1
        q_full=(float(full_ranked[0]["tau_IG"]),int(full_ranked[0]["c"]))
        full_status="ADMISSIBLE_FULL_FIT"
    else:
        full_status="NO_ADMISSIBLE_MX033_FULL_FIT"

    # Gates.
    solved=sum(r["fold_status"]=="ADMISSIBLE" for r in loro_selected)
    held_fired=sum(not blank(r["heldout_CandidateStop"]) for r in loro_selected)
    held_positive=sum(b(r["PositiveSaving"]) for r in loro_selected)
    held_prem=sum(b(r["PrematureStop"]) for r in loro_selected)
    held_severe=sum(b(r["SevereFalseStop10"]) for r in loro_selected)
    held_delays=[r["DelayDecisions"] for r in loro_selected if r["heldout_status"] in {"ON_TARGET","LATE"} and finite(r["DelayDecisions"])]
    held_med=med(held_delays)
    held_mean=avg(held_delays)

    s1=solved==10
    s2=held_prem==0 and held_severe==0
    s3=held_fired>=8 and held_positive>=7 and finite(held_med) and held_med<=3
    solved_q=[(float(r["selected_tau_IG"]),int(r["selected_c"])) for r in loro_selected if r["fold_status"]=="ADMISSIBLE"]
    if q_full is None or not solved_q:
        s4=False; exact_q=0; tau_range=math.nan; c_range=math.nan
    else:
        exact_q=sum(q==q_full for q in solved_q)
        tau_range=max(q[0] for q in solved_q)-min(q[0] for q in solved_q)
        c_range=max(q[1] for q in solved_q)-min(q[1] for q in solved_q)
        s4=exact_q>=7 and tau_range<=0.05 and c_range<=1
    s5=(hard_integrity_failures==0 and monotonicity_violations==0)
    adversarial=audit_adversarial(by_run,casebook)
    s6=all(r["semantic_pass"]==1 for r in adversarial) and len(adversarial)==7

    gates=[
        {"gate":"S1","status":"PASS" if s1 else "FAIL","value":f"{solved}/10","requirement":"all 10 outer folds admissible"},
        {"gate":"S2","status":"PASS" if s2 else "FAIL","value":f"premature={held_prem};severe={held_severe}","requirement":"premature=0; severe=0"},
        {"gate":"S3","status":"PASS" if s3 else "FAIL","value":f"fired={held_fired};positive={held_positive};median_delay={held_med if finite(held_med) else 'NA'}","requirement":"fired>=8; positive>=7; median nonpremature delay<=3"},
        {"gate":"S4","status":"PASS" if s4 else "FAIL","value":f"full_status={full_status};q_full={q_full};exact_q={exact_q};tau_range={tau_range};c_range={c_range}","requirement":"full fit admissible; >=7 exact q; tau range<=0.05; c range<=1"},
        {"gate":"S5","status":"PASS" if s5 else "FAIL","value":f"outer_parity_fail={len(outer_parity_fail)};full_parity_fail={len(full_parity_fail)};source_fail={len(blob_fail)+len(parity_fail)};monotonicity={monotonicity_violations}","requirement":"all accepted-base/source/integrity parity + monotonicity"},
        {"gate":"S6","status":"PASS" if s6 else "FAIL","value":f"semantic_pass={sum(r['semantic_pass'] for r in adversarial)}/7","requirement":"all seven frozen adversarial case families pass"},
    ]

    # Classification order.
    if not s5 or not s6:
        classification="MX033_INVALID_EXECUTION"
    elif not s2:
        classification="ACTIVE_EXPLORATION_VETO_SAFETY_FAIL"
    elif not (s1 and s3 and s4):
        classification="VETO_SAFETY_SIGNAL_ONLY_NO_STABLE_CANDIDATE"
    else:
        classification="ACTIVE_EXPLORATION_VETO_DEVELOPMENT_CANDIDATE"

    # Mandatory outputs.
    write_csv(RESULTS/"MX033_PER_DECISION_VETO_REPLAY.csv",per_decision_output)
    write_csv(RESULTS/"MX033_TRAINING_VETO_GRID.csv",training_grid)
    write_csv(RESULTS/"MX033_LORO_SELECTED_VETO.csv",loro_selected)
    write_csv(RESULTS/"MX033_FULL_FIT_DESCRIPTIVE.csv",full_rows)
    write_csv(RESULTS/"MX033_STABILITY_GATES.csv",gates)
    write_csv(RESULTS/"MX033_ADVERSARIAL_CASE_AUDIT.csv",adversarial)
    write_csv(RESULTS/"MX033_VETO_CHANNEL_ATTRIBUTION.csv",channel_rows)

    dictionary={
        "schema":"mx033_metric_dictionary_v2_execution",
        "method_commit":METHOD,
        "architecture":"AV-DUAL only",
        "outer_backbones":{r:{"tau_R":FROZEN_BACKBONE[r][0],"K":FROZEN_BACKBONE[r][1]} for r in RUNS},
        "veto_grid":{"tau_IG":list(TAU_IGS),"c_cells":list(CS),"pair_count":12},
        "RSmall":"R_available=1 and finite R_map and R_map<=frozen fold tau_R",
        "TopoClear":"TopoValid=1; false is ordinary fail-closed reset, not proof of unsafe world",
        "IGPeakSoFar":"max finite nonnegative State-B IG_selected over same run decisions <=t",
        "IGRel":"State B current IG / IGPeakSoFar; if peak=current=0 then 0; State A/C NA",
        "FrontierClear":{
            "A":"true, IG remains NA",
            "B":"IGRel<=tau_IG",
            "C":"false ordinary fail-closed",
            "D":"hard integrity failure"
        },
        "CoverageClear":"Coverage evaluable, blank reasons, finite Delta/Rate, 0<=Delta<=c*r^2, Rate>=0",
        "signed_coverage_regression":"preserved; CoverageClear=false; SIGNED_COVERAGE_REGRESSION_VETO",
        "ExplorationClear":"FrontierClear AND CoverageClear",
        "Qualify":"RSmall AND TopoClear AND ExplorationClear",
        "online_fields":["R_map","TopoValid","FrontierState","State-B IG_selected","same-run past/current IG","DeltaKnownArea_m2","KnownAreaRate_m2_s","raw_resolution","past/current recognizer state"],
        "evaluator_only":["OracleStop_4","OracleRemainingFraction_GT","P correctness","truth-conditioned U","future/retrospective topology"],
        "NO_RULE":"fixed outer fold remains denominator member; no fire/no saving; no substituted q",
        "classification_order":[
            "MX033_INVALID_EXECUTION","ACTIVE_EXPLORATION_VETO_SAFETY_FAIL",
            "VETO_SAFETY_SIGNAL_ONLY_NO_STABLE_CANDIDATE",
            "ACTIVE_EXPLORATION_VETO_DEVELOPMENT_CANDIDATE"
        ],
    }
    write_json(RESULTS/"MX033_METRIC_DICTIONARY.json",dictionary)

    provenance={
        "schema":"mx033_execution_provenance_v2",
        "executor_role":"Data & Evidence Analyst successor 04",
        "authorization_task":"MX034",
        "method_commit":METHOD,
        "method_blob":METHOD_BLOB,
        "mx032_pack":MX032_PACK,
        "mx031_source_revision":MX031_REV,
        "source_blobs":{str(p.relative_to(REPO)):git_blob(p) for p in EXPECTED_BLOBS},
        "expected_source_blobs":{str(p.relative_to(REPO)):z for p,z in EXPECTED_BLOBS.items()},
        "source_blob_failures":blob_fail,
        "accepted_value_parity_failures":parity_fail,
        "outer_paired_base_parity_failures":outer_parity_fail,
        "fullfit_365row_parity_failures":full_parity_fail,
        "accepted_fullfit_first_fires":ACCEPTED_FULL_FIRES,
        "recomputed_fullfit_first_fires":full_first_fires,
        "monotonicity_violations":monotonicity_violations,
        "hard_replay_integrity_failure_count":hard_integrity_failures,
        "cohort":{"runs":list(RUNS),"decision_rows":len(primary)},
        "candidate_grid":{"tau_IG":list(TAU_IGS),"c":list(CS),"pairs":12},
        "guards":{"Hospital":False,"MX028":False,"P_online":False,"U_online":False,"simulation":False,"Engineer":False,"deployment":False,"robot_STOP":False,"retuning":False,"nearest_passing_substitution":False},
        "runtime":{"python":sys.version.split()[0],"github_run_id":os.environ.get("GITHUB_RUN_ID",""),"github_sha_at_start":os.environ.get("GITHUB_SHA","")},
        "classification":classification,
    }
    write_json(RESULTS/"MX033_EXECUTION_PROVENANCE.json",provenance)

    report=f"""# MX033 Analyst04 result candidate — executed under MX034 authorization

Status: COMPLETE_PENDING_IR1_RESULT_QA

Frozen Method V2: {METHOD}

Result classification: **{classification}**

## Outer held-out summary

- solved outer folds: {solved}/10
- held-out fired runs: {held_fired}/10
- held-out positive-saving runs: {held_positive}/10
- held-out premature stops: {held_prem}
- held-out SevereFalseStop10: {held_severe}
- median non-premature held-out delay: {held_med if finite(held_med) else 'NA'} decisions
- mean non-premature held-out delay: {held_mean if finite(held_mean) else 'NA'} decisions

## Frozen gates

{chr(10).join(f"- {g['gate']}: **{g['status']}** — {g['value']}" for g in gates)}

## Full-fit descriptive status

- status: {full_status}
- q_full: {q_full if q_full is not None else 'undefined'}

The full-fit result is descriptive only and cannot rescue outer failures.

## Integrity

- 365 primary decision rows.
- outer paired-base parity failures: {len(outer_parity_fail)}
- full-fit 365-row parity failures: {len(full_parity_fail)}
- accepted-source blob/value parity failures: {len(blob_fail)+len(parity_fail)}
- AV-DUAL-before-base monotonicity violations: {monotonicity_violations}
- mandatory semantic adversarial checks: {sum(r['semantic_pass'] for r in adversarial)}/7 PASS

## Scientific boundary

This is New Room retrospective development evidence only. It does not authorize
Hospital, Engineer work, deployment, or robot STOP. P/U/MX028 were not used in
the recognizer. No threshold/grid/backbone/gate was changed after outcome
inspection, and no nearest-passing substitute was used.
"""
    (RESULTS/"MX033_ANALYST_REPORT.md").write_text(report,encoding="utf-8")

    required=[
        "MX033_PER_DECISION_VETO_REPLAY.csv","MX033_TRAINING_VETO_GRID.csv",
        "MX033_LORO_SELECTED_VETO.csv","MX033_FULL_FIT_DESCRIPTIVE.csv",
        "MX033_STABILITY_GATES.csv","MX033_ADVERSARIAL_CASE_AUDIT.csv",
        "MX033_VETO_CHANNEL_ATTRIBUTION.csv","MX033_METRIC_DICTIONARY.json",
        "MX033_EXECUTION_PROVENANCE.json","MX033_ANALYST_REPORT.md"
    ]
    missing=[x for x in required if not (RESULTS/x).is_file()]
    if missing:
        raise RuntimeError(f"missing required outputs: {missing}")

    print(json.dumps({
        "classification":classification,"decision_rows":len(primary),
        "training_grid_rows":len(training_grid),"loro_rows":len(loro_selected),
        "solved_folds":solved,"heldout_fired":held_fired,"heldout_positive":held_positive,
        "heldout_premature":held_prem,"heldout_severe":held_severe,
        "heldout_median_delay":held_med if finite(held_med) else None,
        "full_status":full_status,"q_full":q_full,
        "gates":{g["gate"]:g["status"] for g in gates},
        "adversarial_pass":sum(r["semantic_pass"] for r in adversarial),
        "required_outputs":len(required),
    },sort_keys=True))


if __name__=="__main__":
    main()
