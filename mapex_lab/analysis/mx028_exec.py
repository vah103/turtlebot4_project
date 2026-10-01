#!/usr/bin/env python3
"""MX028 one-off deterministic Analyst04 execution.

Implements the frozen IR2-accepted MX028 V2 evidence contract.
Reads only committed New Room artifacts from the frozen technical source.
No model inference, simulation, prediction regeneration, or retuning.
"""
from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


SCRIPT = Path(__file__).resolve()
ANALYSIS = SCRIPT.parent
MAPEX_LAB = ANALYSIS.parent
REPO = MAPEX_LAB.parent
RESULTS = ANALYSIS / "mx028_exec_results"
FIGURES = RESULTS / "figures"
EXPERIMENTS = MAPEX_LAB / "experiments" / "mapex"
GT_PATH = MAPEX_LAB / "ground_truth" / "new_room" / "generated" / "new_room_structural_gt_v2.npz"
GATEP_PATH = ANALYSIS / "d1" / "d1_gate_p.py"

FROZEN_TECHNICAL_BASE = "4899ee95c85640965befeaf98c20f156c0d3181a"
ACCEPTED_METHOD = "429733307f760a5efd9e9641834146e71b927428"
ACCEPTED_QA = "0df38153a4ed8ce5d6b9e742c3c3b8edfcc45367"
ACCEPTED_MX023_P = "e50ccbda7755860e696dba6ae8265a1c4c3c6a4e"
ACCEPTED_MX024_U = "6fe651e383b3d88d1a824558d6ec398fc97eef1c"
ACCEPTED_MX026 = "79ee3ba8e2f82a55cd6e2d68fdb3d2ed08569e55"
EXPECTED_GATEP_BLOB = "c5bf4e3e6f45c81afef135166fc96e088719658e"
EXPECTED_GT_BLOB = "a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a"
RUNS = tuple(f"mpx_{i:03d}" for i in range(1, 11))
ORACLE = {
    "mpx_001": 21, "mpx_002": 18, "mpx_003": 21, "mpx_004": 16,
    "mpx_005": 16, "mpx_006": 20, "mpx_007": 18, "mpx_008": 19,
    "mpx_009": 18, "mpx_010": 19,
}
BINS = [
    ("B00_10", 0.0, 0.1, False),
    ("B10_20", 0.1, 0.2, False),
    ("B20_30", 0.2, 0.3, False),
    ("B30_40", 0.3, 0.4, False),
    ("B40_50", 0.4, 0.5, False),
    ("B50_60", 0.5, 0.6, False),
    ("B60_70", 0.6, 0.7, False),
    ("B70_80", 0.7, 0.8, False),
    ("B80_90", 0.8, 0.9, False),
    ("B90_100", 0.9, 1.0, True),
]
VAR_RTOL = 1e-5
VAR_ATOL = 1e-7


def git_blob_sha1(path: Path) -> str:
    b = path.read_bytes()
    return hashlib.sha1(b"blob " + str(len(b)).encode() + b"\0" + b).hexdigest()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_gatep():
    spec = importlib.util.spec_from_file_location("mx028_gatep_exact", GATEP_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("Unable to load exact Gate-P evaluator")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


GP = load_gatep()


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}


def finite(v) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError):
        return False


def qlinear(a: np.ndarray, q: float) -> float:
    a = np.asarray(a, dtype=np.float64)
    a = a[np.isfinite(a)]
    return float(np.quantile(a, q, method="linear")) if a.size else math.nan


def median(a: np.ndarray) -> float:
    return qlinear(a, 0.5)


def safe_div(n, d) -> float:
    return float(n / d) if d else math.nan


def clean_cell(v):
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
            w.writerow({k: clean_cell(row.get(k, "")) for k in keys})


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


def stratum_masks(u: np.ndarray, q25: float, q75: float):
    return {
        "LOW": u <= q25,
        "MID": (u > q25) & (u < q75),
        "HIGH": u >= q75,
    }


def side_metrics(pred_occ: np.ndarray, truth_occ: np.ndarray, u: np.ndarray,
                 q25: float, q75: float, side: str, collapsed: bool) -> dict:
    if side == "Occ":
        chosen, correct = pred_occ, truth_occ
        zero_reason = "NO_PREDICTED_OCCUPIED_SUPPORT_IN_STRATUM"
        no_correct, no_wrong = "NO_CORRECT_OCCUPIED_SUPPORT", "NO_WRONG_OCCUPIED_SUPPORT"
        label = "OCCUPIED"
    else:
        chosen, correct = ~pred_occ, ~truth_occ
        zero_reason = "NO_PREDICTED_FREE_SUPPORT_IN_STRATUM"
        no_correct, no_wrong = "NO_CORRECT_FREE_SUPPORT", "NO_WRONG_FREE_SUPPORT"
        label = "FREE"
    wrong = ~correct
    sm = stratum_masks(u, q25, q75)
    out = {f"{side}_predicted_count": int(np.count_nonzero(chosen))}

    for s in ("LOW", "MID", "HIGH"):
        m = chosen & sm[s]
        c = int(np.count_nonzero(m & correct))
        w = int(np.count_nonzero(m & wrong))
        n = c + w
        out[f"{side}_support_{s}"] = n
        out[f"{side}_correct_{s}"] = c
        out[f"{side}_wrong_{s}"] = w
        out[f"{side}Precision_{s}"] = safe_div(c, n)
        out[f"{side}Precision_{s}_reason"] = "" if n else zero_reason
        out[f"{side}WrongRate_{s}"] = safe_div(w, n)
        out[f"{side}WrongRate_{s}_reason"] = "" if n else zero_reason

    if collapsed:
        for k in (
            f"{side}Precision_LowMinusHigh",
            f"{side}WrongRate_HighMinusLow",
            f"{side}WrongRateRatio_HighOverLow",
        ):
            out[k] = math.nan
            out[f"{k}_reason"] = "U_QUANTILE_THRESHOLDS_COLLAPSED"
    else:
        lp, hp = out[f"{side}Precision_LOW"], out[f"{side}Precision_HIGH"]
        if finite(lp) and finite(hp):
            out[f"{side}Precision_LowMinusHigh"] = float(lp) - float(hp)
            out[f"{side}Precision_LowMinusHigh_reason"] = ""
        else:
            out[f"{side}Precision_LowMinusHigh"] = math.nan
            out[f"{side}Precision_LowMinusHigh_reason"] = "LOW_OR_HIGH_PRECISION_UNDEFINED"

        lw, hw = out[f"{side}WrongRate_LOW"], out[f"{side}WrongRate_HIGH"]
        if finite(lw) and finite(hw):
            out[f"{side}WrongRate_HighMinusLow"] = float(hw) - float(lw)
            out[f"{side}WrongRate_HighMinusLow_reason"] = ""
            if float(lw) > 0:
                out[f"{side}WrongRateRatio_HighOverLow"] = float(hw) / float(lw)
                out[f"{side}WrongRateRatio_HighOverLow_reason"] = ""
            else:
                out[f"{side}WrongRateRatio_HighOverLow"] = math.nan
                out[f"{side}WrongRateRatio_HighOverLow_reason"] = "ZERO_LOW_U_WRONG_RATE_DENOMINATOR"
        else:
            out[f"{side}WrongRate_HighMinusLow"] = math.nan
            out[f"{side}WrongRate_HighMinusLow_reason"] = "LOW_OR_HIGH_WRONG_RATE_UNDEFINED"
            out[f"{side}WrongRateRatio_HighOverLow"] = math.nan
            out[f"{side}WrongRateRatio_HighOverLow_reason"] = "LOW_OR_HIGH_WRONG_RATE_UNDEFINED"

    uc, uw = u[chosen & correct], u[chosen & wrong]
    out[f"{side}_correct_U_n"] = int(uc.size)
    out[f"{side}_wrong_U_n"] = int(uw.size)
    out[f"{side}_correct_U_median"] = median(uc)
    out[f"{side}_wrong_U_median"] = median(uw)
    if uc.size and uw.size:
        out[f"{side}_U_MedianGap_WrongMinusCorrect"] = median(uw) - median(uc)
        out[f"{side}_U_MedianGap_reason"] = ""
    else:
        out[f"{side}_U_MedianGap_WrongMinusCorrect"] = math.nan
        if not uc.size and not uw.size:
            out[f"{side}_U_MedianGap_reason"] = f"NO_CORRECT_OR_WRONG_{label}_SUPPORT"
        elif not uc.size:
            out[f"{side}_U_MedianGap_reason"] = no_correct
        else:
            out[f"{side}_U_MedianGap_reason"] = no_wrong
    return out


def avg_ranks(x: np.ndarray) -> np.ndarray:
    x = np.asarray(x, dtype=np.float64)
    order = np.argsort(x, kind="mergesort")
    ranks = np.empty(x.size, dtype=np.float64)
    i = 0
    while i < x.size:
        j = i + 1
        while j < x.size and x[order[j]] == x[order[i]]:
            j += 1
        ranks[order[i:j]] = ((i + 1) + j) / 2.0
        i = j
    return ranks


def spearman(progress: list[float], values: list[float]) -> dict:
    candidate = len(progress)
    pairs = [(float(x), float(y)) for x, y in zip(progress, values) if finite(x) and finite(y)]
    n = len(pairs)
    out = {
        "candidate_row_n": candidate,
        "excluded_nonfinite_n": candidate - n,
        "paired_finite_n": n,
        "rho": math.nan,
        "rho_reason": "",
    }
    if n < 5:
        out["rho_reason"] = "INSUFFICIENT_FINITE_PAIRED_SUPPORT_LT5"
        return out
    x = np.asarray([p[0] for p in pairs], dtype=np.float64)
    y = np.asarray([p[1] for p in pairs], dtype=np.float64)
    cx, cy = bool(np.all(x == x[0])), bool(np.all(y == y[0]))
    if cx and cy:
        out["rho_reason"] = "CONSTANT_BOTH_VECTORS"
        return out
    if cx:
        out["rho_reason"] = "CONSTANT_X_VECTOR"
        return out
    if cy:
        out["rho_reason"] = "CONSTANT_Y_VECTOR"
        return out
    out["rho"] = float(np.corrcoef(avg_ranks(x), avg_ranks(y))[0, 1])
    return out


def in_bin(p: float, lo: float, hi: float, inclusive_hi: bool) -> bool:
    return lo <= p <= hi if inclusive_hi else lo <= p < hi


def value_or_nan(row: dict, key: str) -> float:
    v = row.get(key, math.nan)
    return float(v) if finite(v) else math.nan


def source_guard():
    gate_blob, gt_blob = git_blob_sha1(GATEP_PATH), git_blob_sha1(GT_PATH)
    if gate_blob != EXPECTED_GATEP_BLOB:
        raise RuntimeError(f"Gate-P blob mismatch: {gate_blob}")
    if gt_blob != EXPECTED_GT_BLOB:
        raise RuntimeError(f"Structural GT blob mismatch: {gt_blob}")
    changed = subprocess.check_output(
        ["git", "diff", "--name-only", FROZEN_TECHNICAL_BASE, "HEAD"],
        cwd=REPO, text=True,
    ).splitlines()
    allowed = {
        ".github/workflows/mx028-oneoff.yml",
        "mapex_lab/analysis/mx028_exec.py",
    }
    bad = sorted(
        p for p in set(changed)
        if p not in allowed and not p.startswith("mapex_lab/analysis/mx028_exec_results/")
    )
    if bad:
        raise RuntimeError(f"Frozen technical source drift outside MX028 files: {bad}")
    return gate_blob, gt_blob, sorted(changed)


def analyze():
    gate_blob, gt_blob, execution_delta = source_guard()
    gt = GP.load_structural_gt(GT_PATH)
    RESULTS.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)

    main_rows, occ_struct_rows, occ_later_rows = [], [], []
    free_struct_rows, free_later_rows = [], []
    support_rows, parity_rows = [], []
    total_decisions_seen = 0

    for run_id in RUNS:
        run_dir = EXPERIMENTS / run_id
        decision_table = sorted(read_csv(run_dir / "decisions.csv"), key=lambda r: int(r["decision_id"]))
        ids = [int(r["decision_id"]) for r in decision_table]
        if len(ids) != len(set(ids)) or ids != list(range(1, len(ids) + 1)):
            raise RuntimeError(f"{run_id}: invalid decision identity")
        total = len(decision_table)
        total_decisions_seen += total
        decision_grids = [
            GP.load_raw_grid(GP._resolve_run_path(run_dir, row["raw_map"]))
            for row in decision_table
        ]
        metadata = load_json(run_dir / "metadata.json")
        expected_env = str(metadata.get("environment")) if metadata.get("environment") is not None else None

        for pos, row in enumerate(decision_table):
            decision_id = int(row["decision_id"])
            observed = decision_grids[pos]
            warnings = set()
            paths = {k: GP._resolve_run_path(run_dir, row[f"{k}_map"]) for k in ("g1", "g2", "g3", "mean", "variance")}
            g1 = GP.load_runtime_prediction(paths["g1"], observed, "G1", expected_env, warnings)
            g2 = GP.load_runtime_prediction(paths["g2"], observed, "G2", expected_env, warnings)
            g3 = GP.load_runtime_prediction(paths["g3"], observed, "G3", expected_env, warnings)
            mean = GP.load_runtime_prediction(paths["mean"], observed, "mean", expected_env, warnings)
            var_saved = GP.load_runtime_prediction(paths["variance"], observed, "variance", expected_env, warnings)

            arrays = (g1, g2, g3, mean, var_saved)
            if not all(np.all(np.isfinite(a)) for a in arrays):
                raise RuntimeError(f"{run_id} d{decision_id}: non-finite aligned source")
            if any(np.any((a < 0) | (a > 1)) for a in (g1, g2, g3, mean)):
                raise RuntimeError(f"{run_id} d{decision_id}: prediction outside [0,1]")
            if np.any(var_saved < 0):
                raise RuntimeError(f"{run_id} d{decision_id}: negative variance")

            recomputed = np.var(np.stack([g1, g2, g3], axis=0), axis=0, ddof=1)
            diff = np.abs(var_saved.astype(np.float64) - recomputed.astype(np.float64))
            parity_pass = bool(np.allclose(var_saved, recomputed, rtol=VAR_RTOL, atol=VAR_ATOL, equal_nan=False))
            parity_rows.append({
                "run_id": run_id, "decision_id": decision_id,
                "variance_parity_pass": parity_pass,
                "variance_max_abs_diff": float(np.max(diff)),
                "variance_mean_abs_diff": float(np.mean(diff)),
                "variance_rtol": VAR_RTOL, "variance_atol": VAR_ATOL,
                "variance_estimator": "np.var(G1,G2,G3,axis=0,ddof=1)",
                "g1_path": row["g1_map"], "g2_path": row["g2_map"], "g3_path": row["g3_map"],
                "mean_path": row["mean_map"], "variance_path": row["variance_map"],
            })
            if not parity_pass:
                raise RuntimeError(f"{run_id} d{decision_id}: VARIANCE_PARITY_FAIL")

            unknown = observed.data < 0
            uvals = var_saved[unknown].astype(np.float64)
            if uvals.size == 0:
                raise RuntimeError(f"{run_id} d{decision_id}: EMPTY_MAPEX_UNKNOWN_VALID_DOMAIN")
            q25, q50, q75 = (qlinear(uvals, q) for q in (0.25, 0.5, 0.75))
            collapsed = bool(q25 == q75)
            u_low, u_mid, u_high = uvals <= q25, (uvals > q25) & (uvals < q75), uvals >= q75
            low_ties, high_ties = int(np.count_nonzero(uvals == q25)), int(np.count_nonzero(uvals == q75))

            mean_u = mean[unknown].astype(np.float64)
            pred_occ_u, pred_free_u = mean_u >= 0.5, mean_u < 0.5
            occ_low, free_low = pred_occ_u & u_low, pred_free_u & u_low
            pred_occ_n, pred_free_n = int(np.count_nonzero(pred_occ_u)), int(np.count_nonzero(pred_free_u))
            runtime_area = float(observed.resolution * observed.resolution)
            progress = float(pos / (total - 1)) if total > 1 else 0.0

            base = {
                "run_id": run_id, "decision_id": decision_id, "decision_count": total,
                "normalized_progress": progress, "oracle4_decision_id": ORACLE[run_id],
                "is_oracle4": int(decision_id == ORACLE[run_id]),
                "D_U_count": int(uvals.size),
                "U_Q25": q25, "U_Q50": q50, "U_Q75": q75,
                "U_LOW_count": int(np.count_nonzero(u_low)),
                "U_MID_count": int(np.count_nonzero(u_mid)),
                "U_HIGH_count": int(np.count_nonzero(u_high)),
                "U_Q25_tie_count": low_ties, "U_Q75_tie_count": high_ties,
                "U_quantile_thresholds_collapsed": int(collapsed),
                "PredOcc_count": pred_occ_n, "PredFree_count": pred_free_n,
                "PredOcc_share_unknown": safe_div(pred_occ_n, int(uvals.size)),
                "PredFree_share_unknown": safe_div(pred_free_n, int(uvals.size)),
                "PredOcc_median_U": median(uvals[pred_occ_u]),
                "PredFree_median_U": median(uvals[pred_free_u]),
                "OccLowU_count": int(np.count_nonzero(occ_low)),
                "OccLowU_share_unknown": safe_div(int(np.count_nonzero(occ_low)), int(uvals.size)),
                "OccLowU_share_predOcc": safe_div(int(np.count_nonzero(occ_low)), pred_occ_n),
                "OccLowU_area_m2": float(np.count_nonzero(occ_low) * runtime_area),
                "FreeLowU_count": int(np.count_nonzero(free_low)),
                "FreeLowU_share_unknown": safe_div(int(np.count_nonzero(free_low)), int(uvals.size)),
                "FreeLowU_share_predFree": safe_div(int(np.count_nonzero(free_low)), pred_free_n),
                "FreeLowU_area_m2": float(np.count_nonzero(free_low) * runtime_area),
                "runtime_exact_p_eq_0_5_unknown_count": int(np.count_nonzero(mean_u == 0.5)),
                "variance_parity_pass": int(parity_pass),
                "variance_max_abs_diff": float(np.max(diff)),
                "source_warning_count": len(warnings),
                "source_warnings": " | ".join(sorted(warnings)),
            }

            context = GP._structural_context(observed, gt)
            smean = GP._structural_prediction_values(mean, context).astype(np.float64)
            su = GP._structural_prediction_values(var_saved, context).astype(np.float64)
            struth = np.asarray(context["truth_occupied"], dtype=bool)
            spred_occ = smean >= 0.5
            sboundary = int(np.count_nonzero(smean == 0.5))
            if sboundary:
                raise RuntimeError(f"{run_id} d{decision_id}: structural exact p==0.5 boundary parity failed")
            sm_occ = side_metrics(spred_occ, struth, su, q25, q75, "Occ", collapsed)
            sm_free = side_metrics(spred_occ, struth, su, q25, q75, "Free", collapsed)
            sbase = {
                "run_id": run_id, "decision_id": decision_id, "decision_count": total,
                "normalized_progress": progress, "domain": "STRUCTURAL_GT", "evaluable": 1, "reason": "",
                "truth_domain_count": int(struth.size),
                "truth_free_count": int(np.count_nonzero(~struth)),
                "truth_occupied_count": int(np.count_nonzero(struth)),
                "U_Q25": q25, "U_Q50": q50, "U_Q75": q75,
                "U_quantile_thresholds_collapsed": int(collapsed),
                "exact_p_eq_0_5_count": sboundary,
                "projection_ratio": int(context["ratio"]),
                "origin_rounding_residual_x_m": context["origin_rounding_residual_x_m"],
                "origin_rounding_residual_y_m": context["origin_rounding_residual_y_m"],
            }
            occ_struct_rows.append({**sbase, **sm_occ})
            free_struct_rows.append({**sbase, **sm_free})

            unknown_rows, unknown_cols = np.nonzero(unknown)
            labels, reveal_ids, reveal_steps, reveal_time = GP.first_later_observation_targets(
                observed, unknown_rows, unknown_cols, pos + 1, decision_table, decision_grids
            )
            later_mask = labels >= 0
            ltruth = labels[later_mask] > 0
            lmean = mean[unknown_rows[later_mask], unknown_cols[later_mask]].astype(np.float64)
            lu = var_saved[unknown_rows[later_mask], unknown_cols[later_mask]].astype(np.float64)
            lpred_occ = lmean >= 0.5
            lboundary = int(np.count_nonzero(lmean == 0.5))
            if lboundary:
                raise RuntimeError(f"{run_id} d{decision_id}: later exact p==0.5 boundary parity failed")
            levaluable = int(ltruth.size > 0)
            lreason = "" if levaluable else "NO_LATER_OBSERVED_TARGET_SUPPORT"
            lm_occ = side_metrics(lpred_occ, ltruth, lu, q25, q75, "Occ", collapsed)
            lm_free = side_metrics(lpred_occ, ltruth, lu, q25, q75, "Free", collapsed)
            lbase = {
                "run_id": run_id, "decision_id": decision_id, "decision_count": total,
                "normalized_progress": progress, "domain": "LATER_OBSERVED",
                "evaluable": levaluable, "reason": lreason,
                "truth_domain_count": int(ltruth.size),
                "truth_free_count": int(np.count_nonzero(~ltruth)),
                "truth_occupied_count": int(np.count_nonzero(ltruth)),
                "U_Q25": q25, "U_Q50": q50, "U_Q75": q75,
                "U_quantile_thresholds_collapsed": int(collapsed),
                "exact_p_eq_0_5_count": lboundary,
                "first_reveal_decision_id": int(np.min(reveal_ids[later_mask])) if np.any(later_mask) else "",
                "last_reveal_decision_id": int(np.max(reveal_ids[later_mask])) if np.any(later_mask) else "",
            }
            occ_later_rows.append({**lbase, **lm_occ})
            free_later_rows.append({**lbase, **lm_free})

            m = dict(base)
            for k in (
                "OccPrecision_LOW", "OccPrecision_MID", "OccPrecision_HIGH",
                "OccPrecision_LowMinusHigh", "OccWrongRate_LOW", "OccWrongRate_HIGH",
                "OccWrongRate_HighMinusLow", "OccWrongRateRatio_HighOverLow",
                "Occ_U_MedianGap_WrongMinusCorrect", "Occ_predicted_count",
                "Occ_support_LOW", "Occ_support_MID", "Occ_support_HIGH",
                "Occ_correct_U_n", "Occ_wrong_U_n",
                "Occ_correct_U_median", "Occ_wrong_U_median",
            ):
                m[f"Structural_{k}"] = sm_occ.get(k, math.nan)
                m[f"Later_{k}"] = lm_occ.get(k, math.nan)
            for k in (
                "FreePrecision_LOW", "FreePrecision_MID", "FreePrecision_HIGH",
                "FreePrecision_LowMinusHigh", "FreeWrongRate_LOW", "FreeWrongRate_HIGH",
                "FreeWrongRate_HighMinusLow", "FreeWrongRateRatio_HighOverLow",
                "Free_U_MedianGap_WrongMinusCorrect", "Free_predicted_count",
                "Free_support_LOW", "Free_support_MID", "Free_support_HIGH",
                "Free_correct_U_n", "Free_wrong_U_n",
                "Free_correct_U_median", "Free_wrong_U_median",
            ):
                m[f"Structural_{k}"] = sm_free.get(k, math.nan)
                m[f"Later_{k}"] = lm_free.get(k, math.nan)
            m["Structural_truth_domain_count"] = int(struth.size)
            m["Later_truth_domain_count"] = int(ltruth.size)
            m["Later_evaluable"] = levaluable
            m["Later_reason"] = lreason
            main_rows.append(m)

            support_rows.extend([
                {
                    "run_id": run_id, "decision_id": decision_id, "domain": "TRUTH_FREE_RUNTIME",
                    "evaluable": 1, "reason": "", "cell_count": int(uvals.size),
                    "predicted_occupied_count": pred_occ_n, "predicted_free_count": pred_free_n,
                    "q25_tie_count": low_ties, "q75_tie_count": high_ties,
                    "quantile_thresholds_collapsed": int(collapsed),
                },
                {
                    "run_id": run_id, "decision_id": decision_id, "domain": "STRUCTURAL_GT",
                    "evaluable": 1, "reason": "", "cell_count": int(struth.size),
                    "predicted_occupied_count": sm_occ["Occ_predicted_count"],
                    "predicted_free_count": sm_free["Free_predicted_count"],
                    "truth_free_count": int(np.count_nonzero(~struth)),
                    "truth_occupied_count": int(np.count_nonzero(struth)),
                    "quantile_thresholds_collapsed": int(collapsed),
                },
                {
                    "run_id": run_id, "decision_id": decision_id, "domain": "LATER_OBSERVED",
                    "evaluable": levaluable, "reason": lreason, "cell_count": int(ltruth.size),
                    "predicted_occupied_count": lm_occ["Occ_predicted_count"],
                    "predicted_free_count": lm_free["Free_predicted_count"],
                    "truth_free_count": int(np.count_nonzero(~ltruth)),
                    "truth_occupied_count": int(np.count_nonzero(ltruth)),
                    "quantile_thresholds_collapsed": int(collapsed),
                },
            ])

    if total_decisions_seen != 365 or len(main_rows) != 365:
        raise RuntimeError(f"Expected exactly 365 decisions, got {total_decisions_seen}/{len(main_rows)}")
    keys = [(r["run_id"], r["decision_id"]) for r in main_rows]
    if len(keys) != len(set(keys)):
        raise RuntimeError("Duplicate run_id+decision_id in primary output")

    metric_specs = [
        ("OccLowU_share_unknown", "TRUTH_FREE", "OccLowU_share_unknown"),
        ("OccLowU_share_predOcc", "TRUTH_FREE", "OccLowU_share_predOcc"),
        ("PredOcc_share_unknown", "TRUTH_FREE", "PredOcc_share_unknown"),
        ("PredOcc_median_U", "TRUTH_FREE", "PredOcc_median_U"),
        ("Structural_OccPrecision_LOW", "STRUCTURAL_GT", "Structural_OccPrecision_LOW"),
        ("Structural_OccPrecision_HIGH", "STRUCTURAL_GT", "Structural_OccPrecision_HIGH"),
        ("Structural_OccPrecision_LowMinusHigh", "STRUCTURAL_GT", "Structural_OccPrecision_LowMinusHigh"),
        ("Structural_Occ_U_MedianGap_WrongMinusCorrect", "STRUCTURAL_GT", "Structural_Occ_U_MedianGap_WrongMinusCorrect"),
        ("FreeLowU_share_unknown", "TRUTH_FREE_CONTROL", "FreeLowU_share_unknown"),
        ("PredFree_median_U", "TRUTH_FREE_CONTROL", "PredFree_median_U"),
        ("Structural_FreePrecision_LOW", "STRUCTURAL_GT_CONTROL", "Structural_FreePrecision_LOW"),
        ("Structural_FreePrecision_HIGH", "STRUCTURAL_GT_CONTROL", "Structural_FreePrecision_HIGH"),
        ("Structural_Free_U_MedianGap_WrongMinusCorrect", "STRUCTURAL_GT_CONTROL", "Structural_Free_U_MedianGap_WrongMinusCorrect"),
    ]

    by_run = {run: sorted([r for r in main_rows if r["run_id"] == run], key=lambda x: x["decision_id"]) for run in RUNS}
    per_run = []
    for run, rr in by_run.items():
        p = [r["normalized_progress"] for r in rr]
        for metric, domain, key in metric_specs:
            vals = [value_or_nan(r, key) for r in rr]
            valid_vals = [v for v in vals if finite(v)]
            sp = spearman(p, vals)
            per_run.append({
                "run_id": run, "metric": metric, "domain": domain,
                "first_valid": valid_vals[0] if valid_vals else math.nan,
                "final_valid": valid_vals[-1] if valid_vals else math.nan,
                "final_minus_first": valid_vals[-1] - valid_vals[0] if valid_vals else math.nan,
                "valid_n": len(valid_vals), "total_n": len(vals),
                "candidate_row_n": sp["candidate_row_n"],
                "excluded_nonfinite_n": sp["excluded_nonfinite_n"],
                "paired_finite_n": sp["paired_finite_n"],
                "spearman_rho": sp["rho"], "rho_reason": sp["rho_reason"],
            })

    temp = []
    for run, rr in by_run.items():
        for bin_name, lo, hi, inc in BINS:
            subset = [r for r in rr if in_bin(float(r["normalized_progress"]), lo, hi, inc)]
            for metric, domain, key in metric_specs:
                vals = [value_or_nan(r, key) for r in subset]
                fv = [v for v in vals if finite(v)]
                temp.append({
                    "run_id": run, "bin": bin_name, "metric": metric, "domain": domain,
                    "run_bin_median": median(np.asarray(fv)) if fv else math.nan,
                    "valid_decision_n": len(fv), "candidate_decision_n": len(subset),
                })
    bin_rows = []
    for row in temp:
        macro = [
            float(x["run_bin_median"]) for x in temp
            if x["bin"] == row["bin"] and x["metric"] == row["metric"] and finite(x["run_bin_median"])
        ]
        z = dict(row)
        z.update({
            "macro_median": qlinear(np.asarray(macro), 0.5) if macro else math.nan,
            "macro_q25": qlinear(np.asarray(macro), 0.25) if macro else math.nan,
            "macro_q75": qlinear(np.asarray(macro), 0.75) if macro else math.nan,
            "contributor_run_n": len(macro),
        })
        bin_rows.append(z)

    oracle_late = []
    occ_summary_specs = [
        ("OccLowU_share_unknown", "TRUTH_FREE", "OccLowU_share_unknown", "PredOcc_count"),
        ("PredOcc_share_unknown", "TRUTH_FREE", "PredOcc_share_unknown", "D_U_count"),
        ("PredOcc_median_U", "TRUTH_FREE", "PredOcc_median_U", "PredOcc_count"),
        ("OccPrecision_LOW", "STRUCTURAL_GT", "Structural_OccPrecision_LOW", "Structural_Occ_predicted_count"),
        ("OccPrecision_HIGH", "STRUCTURAL_GT", "Structural_OccPrecision_HIGH", "Structural_Occ_predicted_count"),
        ("OccPrecision_LowMinusHigh", "STRUCTURAL_GT", "Structural_OccPrecision_LowMinusHigh", "Structural_Occ_predicted_count"),
        ("Occ_U_MedianGap_WrongMinusCorrect", "STRUCTURAL_GT", "Structural_Occ_U_MedianGap_WrongMinusCorrect", "Structural_Occ_predicted_count"),
        ("OccPrecision_LOW", "LATER_OBSERVED", "Later_OccPrecision_LOW", "Later_Occ_predicted_count"),
        ("OccPrecision_HIGH", "LATER_OBSERVED", "Later_OccPrecision_HIGH", "Later_Occ_predicted_count"),
        ("OccPrecision_LowMinusHigh", "LATER_OBSERVED", "Later_OccPrecision_LowMinusHigh", "Later_Occ_predicted_count"),
        ("Occ_U_MedianGap_WrongMinusCorrect", "LATER_OBSERVED", "Later_Occ_U_MedianGap_WrongMinusCorrect", "Later_Occ_predicted_count"),
    ]
    for run, rr in by_run.items():
        scopes = [("ORACLE4", [r for r in rr if int(r["decision_id"]) == ORACLE[run]])]
        for bn in ("B80_90", "B90_100"):
            bdef = next(b for b in BINS if b[0] == bn)
            scopes.append((bn, [r for r in rr if in_bin(float(r["normalized_progress"]), bdef[1], bdef[2], bdef[3])]))
        for scope, subset in scopes:
            for metric, domain, key, support_key in occ_summary_specs:
                fv = [value_or_nan(r, key) for r in subset]
                fv = [v for v in fv if finite(v)]
                fs = [value_or_nan(r, support_key) for r in subset]
                fs = [v for v in fs if finite(v)]
                oracle_late.append({
                    "run_id": run, "scope": scope, "domain": domain, "metric": metric,
                    "value": fv[0] if scope == "ORACLE4" and fv else (median(np.asarray(fv)) if fv else math.nan),
                    "valid_decision_n": len(fv), "candidate_decision_n": len(subset),
                    "cell_support_exact_or_median": fs[0] if scope == "ORACLE4" and fs else (median(np.asarray(fs)) if fs else math.nan),
                })

    write_csv(RESULTS / "MX028_PXU_PER_DECISION.csv", main_rows)
    write_csv(RESULTS / "MX028_OCCUPIED_STRUCTURAL_PER_DECISION.csv", occ_struct_rows)
    write_csv(RESULTS / "MX028_OCCUPIED_LATER_PER_DECISION.csv", occ_later_rows)
    write_csv(RESULTS / "MX028_FREE_CONTROL_STRUCTURAL_PER_DECISION.csv", free_struct_rows)
    write_csv(RESULTS / "MX028_FREE_CONTROL_LATER_PER_DECISION.csv", free_later_rows)
    write_csv(RESULTS / "MX028_PXU_PER_RUN.csv", per_run)
    write_csv(RESULTS / "MX028_PXU_PROGRESS_BINS.csv", bin_rows)
    write_csv(RESULTS / "MX028_PXU_ORACLE_LATE_SUMMARY.csv", oracle_late)
    write_csv(RESULTS / "MX028_PXU_SUPPORT_INVENTORY.csv", support_rows)
    write_csv(RESULTS / "MX028_VARIANCE_SOURCE_PARITY.csv", parity_rows)

    for run, rr in by_run.items():
        x = np.asarray([r["normalized_progress"] for r in rr], dtype=float)
        oracle_p = next(r["normalized_progress"] for r in rr if r["decision_id"] == ORACLE[run])
        fig, axes = plt.subplots(6, 1, figsize=(12, 18), sharex=True)
        axes[0].plot(x, [value_or_nan(r, "PredOcc_share_unknown") for r in rr], label="PredOcc share unknown")
        axes[0].plot(x, [value_or_nan(r, "OccLowU_share_unknown") for r in rr], label="Occ+LOW_U share unknown")
        axes[0].set_ylabel("share"); axes[0].legend()

        for key, label in (("U_Q25","U Q25"),("U_Q50","U Q50"),("U_Q75","U Q75"),("PredOcc_median_U","PredOcc median U")):
            axes[1].plot(x, [value_or_nan(r, key) for r in rr], label=label)
        axes[1].set_ylabel("variance U"); axes[1].legend()

        for key, label in (("Structural_OccPrecision_LOW","LOW precision"),("Structural_OccPrecision_MID","MID precision"),("Structural_OccPrecision_HIGH","HIGH precision")):
            axes[2].plot(x, [value_or_nan(r, key) for r in rr], label=label)
        axes[2].set_ylabel("structural occ precision")
        support_ax = axes[2].twinx()
        for key, label in (("Structural_Occ_support_LOW","LOW support"),("Structural_Occ_support_MID","MID support"),("Structural_Occ_support_HIGH","HIGH support")):
            support_ax.plot(x, [value_or_nan(r, key) for r in rr], linestyle=":", alpha=0.35, label=label)
        support_ax.set_ylabel("support cells")
        h1, l1 = axes[2].get_legend_handles_labels()
        h2, l2 = support_ax.get_legend_handles_labels()
        axes[2].legend(h1 + h2, l1 + l2, loc="best")

        axes[3].plot(x, [value_or_nan(r, "Structural_Occ_correct_U_median") for r in rr], label="OO correct median U")
        axes[3].plot(x, [value_or_nan(r, "Structural_Occ_wrong_U_median") for r in rr], label="FO wrong median U")
        axes[3].set_ylabel("occupied median U"); axes[3].legend()

        axes[4].plot(x, [value_or_nan(r, "PredFree_share_unknown") for r in rr], label="PredFree share unknown")
        axes[4].plot(x, [value_or_nan(r, "FreeLowU_share_unknown") for r in rr], label="Free+LOW_U share unknown")
        axes[4].set_ylabel("free control share"); axes[4].legend()

        for key, label in (("Structural_FreePrecision_LOW","LOW"),("Structural_FreePrecision_MID","MID"),("Structural_FreePrecision_HIGH","HIGH")):
            axes[5].plot(x, [value_or_nan(r, key) for r in rr], label=label)
        axes[5].set_ylabel("structural free precision"); axes[5].set_xlabel("normalized progress"); axes[5].legend()

        for ax in axes:
            ax.axvline(float(oracle_p), linestyle="--", linewidth=0.9, label="_nolegend_")
            ax.grid(True, alpha=0.2)
        fig.suptitle(f"MX028 {run} — decision-relative U quartiles; Oracle-4 marker only")
        fig.tight_layout()
        fig.savefig(FIGURES / f"MX028_{run}_FULL_TRAJECTORY.svg", format="svg")
        plt.close(fig)

    metric_dictionary = {
        "schema": "mx028_pxu_metric_dictionary_v1",
        "status": "COMPLETE_PENDING_IR2_RESULT_QA",
        "interpretation_guard": "LOW/MID/HIGH are decision-relative quartiles on runtime D_U=raw<0; not fixed confidence, calibration, or safety classes.",
        "prediction_class": {"free": "p < 0.5", "occupied": "p >= 0.5"},
        "truth_domains": {
            "STRUCTURAL_GT": "Primary; exact Gate-P 0.10m to 0.05m projection and evaluation_mask.",
            "LATER_OBSERVED": "Secondary; exact first-later-policy-decision target; no Structural truth borrowing.",
        },
        "uncertainty": {
            "source": "saved original MapEx variance",
            "parity": "np.var(G1,G2,G3,axis=0,ddof=1), allclose rtol=1e-5 atol=1e-7",
            "quantiles": "numpy.quantile(method=linear) over finite U on runtime D_U before truth/projection/class masking",
            "LOW_U": "V <= U_Q25", "MID_U": "U_Q25 < V < U_Q75", "HIGH_U": "V >= U_Q75",
        },
        "class_labels": {
            "FF": "true free, predicted free", "FO": "true free, predicted occupied",
            "OF": "true occupied, predicted free", "OO": "true occupied, predicted occupied",
        },
        "primary_formulas": {
            "OccPrecision_s": "OO_s/(OO_s+FO_s)",
            "OccWrongRate_s": "FO_s/(OO_s+FO_s)",
            "OccPrecision_LowMinusHigh": "OccPrecision_LOW-OccPrecision_HIGH",
            "OccWrongRate_HighMinusLow": "OccWrongRate_HIGH-OccWrongRate_LOW",
            "OccWrongRateRatio_HighOverLow": "OccWrongRate_HIGH/OccWrongRate_LOW only when LOW>0; no pseudocount",
            "Occ_U_MedianGap_WrongMinusCorrect": "medianU(FO)-medianU(OO)",
            "FreePrecision_s": "FF_s/(FF_s+OF_s)",
            "Free_U_MedianGap_WrongMinusCorrect": "medianU(OF)-medianU(FF)",
            "OccLowU_share_unknown": "count(predicted occupied AND LOW_U)/count(D_U)",
            "OccLowU_share_predOcc": "count(predicted occupied AND LOW_U)/count(predicted occupied)",
        },
        "missingness": {
            "zero_class_denominator": "NA, never 0",
            "collapsed_q25_q75": "LOW-vs-HIGH comparisons NA with U_QUANTILE_THRESHOLDS_COLLAPSED",
            "no_interpolation_fill_imputation": True,
        },
        "progress_bins": [b[0] for b in BINS],
        "spearman": "Average ranks for ties; Pearson of ranks; finite pairs only; n<5 NA; constant-vector reasons preserved.",
        "oracle_decisions": ORACLE,
    }
    write_json(RESULTS / "MX028_PXU_METRIC_DICTIONARY.json", metric_dictionary)

    run_metric = {(r["run_id"], r["metric"]): r for r in per_run}
    def rhos(metric):
        return [run_metric[(run, metric)]["spearman_rho"] for run in RUNS if finite(run_metric[(run, metric)]["spearman_rho"])]
    collapsed_n = sum(int(r["U_quantile_thresholds_collapsed"]) for r in main_rows)
    later_eval_n = sum(int(r["Later_evaluable"]) for r in main_rows)
    boundary_total = sum(int(r["runtime_exact_p_eq_0_5_unknown_count"]) for r in main_rows)

    report = f"""# MX028 Analyst04 — Full-365 P×U result candidate

Status: **COMPLETE_PENDING_IR2_RESULT_QA**

Frozen method: {ACCEPTED_METHOD}
Frozen technical source: {FROZEN_TECHNICAL_BASE}

## Execution integrity

- 10/10 New Room runs; **365/365** unique decisions.
- Saved original-MapEx variance parity vs np.var(G1,G2,G3, ddof=1): **365/365 PASS**.
- Structural Gate-P evaluator blob: {gate_blob}.
- Structural GT blob: {gt_blob}.
- Runtime exact p == 0.5 unknown-cell count across all decisions: **{boundary_total}**.
- Decision-relative Q25/Q75 collapsed decisions: **{collapsed_n}/365**.
- Later-observed decisions with first-later target support: **{later_eval_n}/365**.

## Descriptive trajectory checks

Median per-run Spearman rho versus normalized progress:
- OccLowU_share_unknown: **{median(np.asarray(rhos("OccLowU_share_unknown"))):.6f}**
- PredOcc_share_unknown: **{median(np.asarray(rhos("PredOcc_share_unknown"))):.6f}**
- PredOcc_median_U: **{median(np.asarray(rhos("PredOcc_median_U"))):.6f}**
- Structural OccPrecision_LOW: **{median(np.asarray(rhos("Structural_OccPrecision_LOW"))):.6f}**
- Structural OccPrecision_HIGH: **{median(np.asarray(rhos("Structural_OccPrecision_HIGH"))):.6f}**

These are descriptive maker-side checks only. IR2 independent result QA owns acceptance/revision.

## Interpretation guard

LOW/MID/HIGH are relative quartiles within each decision's runtime unknown domain.
A change in OccLowU_share_* is a relative-composition trend; it is not an
absolute confidence, calibration, or safety claim. Structural and Later-observed
truth populations remain separate.

No model/simulation/prediction rerun, threshold optimization, pseudocount,
imputation, composite, winner, STOP/deployment rule, calibration claim, or
causal claim was introduced.
"""
    (RESULTS / "MX028_ANALYST_REPORT.md").write_text(report, encoding="utf-8")

    provenance = {
        "schema": "mx028_execution_provenance_v1",
        "status": "COMPLETE_PENDING_IR2_RESULT_QA",
        "executor_role": "Data & Evidence Analyst successor 04",
        "execution_type": "bounded one-off deterministic recomputation",
        "transport_commit_actor": "github-actions[bot]",
        "frozen_method_commit": ACCEPTED_METHOD,
        "method_qa_accept_commit": ACCEPTED_QA,
        "accepted_dependencies": {
            "MX023_P": ACCEPTED_MX023_P,
            "MX024_U": ACCEPTED_MX024_U,
            "MX026_frame": ACCEPTED_MX026,
        },
        "technical_source_base": FROZEN_TECHNICAL_BASE,
        "gate_p_path": str(GATEP_PATH.relative_to(REPO)),
        "gate_p_blob": gate_blob,
        "structural_gt_path": str(GT_PATH.relative_to(REPO)),
        "structural_gt_blob": gt_blob,
        "execution_branch_delta_from_frozen_base": execution_delta,
        "variance_parity_contract": {"ddof": 1, "rtol": VAR_RTOL, "atol": VAR_ATOL, "pass_n": 365},
        "cohort": {"runs": list(RUNS), "decision_n": 365},
        "oracle4_decisions": ORACLE,
        "guards": {
            "model_inference": False, "simulation": False, "prediction_regeneration": False,
            "threshold_retuning": False, "pseudocount": False, "imputation": False,
            "composite": False, "winner": False, "stop_rule": False,
            "calibration_claim": False, "causal_claim": False,
        },
        "runtime": {
            "python": sys.version.split()[0], "numpy": np.__version__,
            "matplotlib": matplotlib.__version__,
            "github_run_id": os.environ.get("GITHUB_RUN_ID", ""),
            "github_sha_at_start": os.environ.get("GITHUB_SHA", ""),
        },
    }
    write_json(RESULTS / "MX028_EXECUTION_PROVENANCE.json", provenance)

    artifacts = []
    for p in sorted(RESULTS.rglob("*")):
        if p.is_file() and p.name != "MX028_PXU_ARTIFACT_MANIFEST.json":
            artifacts.append({
                "path": str(p.relative_to(RESULTS)),
                "sha256": sha256(p),
                "size": p.stat().st_size,
            })
    manifest = {
        "schema": "mx028_pxu_artifact_manifest_v1",
        "status": "COMPLETE_PENDING_IR2_RESULT_QA",
        "frozen_method_commit": ACCEPTED_METHOD,
        "technical_source_base": FROZEN_TECHNICAL_BASE,
        "primary_row_count": len(main_rows),
        "unique_primary_keys": len(set(keys)),
        "artifact_count_excluding_manifest": len(artifacts),
        "artifacts": artifacts,
    }
    write_json(RESULTS / "MX028_PXU_ARTIFACT_MANIFEST.json", manifest)

    print(json.dumps({
        "status": "COMPLETE_PENDING_IR2_RESULT_QA",
        "primary_rows": len(main_rows),
        "unique_keys": len(set(keys)),
        "variance_parity_pass": sum(int(r["variance_parity_pass"]) for r in parity_rows),
        "later_evaluable_decisions": later_eval_n,
        "collapsed_decisions": collapsed_n,
        "runtime_p_eq_0_5_unknown_count": boundary_total,
        "artifact_count_excluding_manifest": len(artifacts),
    }, sort_keys=True))


if __name__ == "__main__":
    analyze()
