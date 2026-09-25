#!/usr/bin/env python3
"""Frozen H068 Gate-U Method V2 scoring primitives."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import pandas as pd
from scipy.stats import spearmanr

CANDIDATES = ("U_p95", "U_mean", "U_disagreement")
RUNS = tuple(f"mpx_{i:03d}" for i in range(1, 11))
PROGRESS_BINS = ("[0.00,0.25)", "[0.25,0.50)", "[0.50,0.75)", "[0.75,1.00]")
EXPECTED_COUNTS = dict(zip(RUNS, (35, 37, 41, 36, 34, 36, 35, 35, 36, 40)))
EXPECTED_INPUT_SHA256 = "993bf0a0e9f1aabfd240778aadd9f6cb34583059ba360cea68fefef97ea2660a"
METHOD_ID = "D1_GATE_U_METHOD_V2_H068_ACCEPTED"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite(values: pd.Series) -> pd.Series:
    return pd.to_numeric(values, errors="coerce").replace([np.inf, -np.inf], np.nan)


def empirical_cdf(reference: Iterable[float], values: Iterable[float]) -> np.ndarray:
    ref = np.sort(np.asarray(list(reference), dtype=float))
    vals = np.asarray(list(values), dtype=float)
    if not len(ref):
        return np.full(vals.shape, np.nan)
    return np.searchsorted(ref, vals, side="right") / len(ref)


def valid_spearman(u: Iterable[float], r: Iterable[float]) -> tuple[float | None, int]:
    pairs = pd.DataFrame({"u": u, "r": r}).replace([np.inf, -np.inf], np.nan).dropna()
    if len(pairs) < 5 or pairs.u.nunique() < 2 or pairs.r.nunique() < 2:
        return None, len(pairs)
    return float(spearmanr(pairs.u, pairs.r).statistic), len(pairs)


def ratio_low50(u: pd.Series, risk: pd.Series) -> tuple[float | None, int, int]:
    pairs = pd.DataFrame({"u": finite(u), "r": finite(risk)}).dropna()
    if pairs.empty:
        return None, 0, 0
    pairs["rank"] = empirical_cdf(pairs.u, pairs.u)
    low = pairs[pairs["rank"] <= 0.50]
    baseline = float(pairs.r.mean())
    if low.empty or baseline <= 0:
        return None, len(low), len(pairs)
    return float(low.r.mean() / baseline), len(low), len(pairs)


def cw25(u: pd.Series, risk: pd.Series) -> tuple[float | None, int, int]:
    pairs = pd.DataFrame({"u": finite(u), "r": finite(risk)}).dropna()
    if pairs.empty:
        return None, 0, 0
    pairs["u_rank"] = empirical_cdf(pairs.u, pairs.u)
    pairs["r_rank"] = empirical_cdf(pairs.r, pairs.r)
    low = pairs[pairs.u_rank <= 0.25]
    if low.empty:
        return None, 0, len(pairs)
    return float((low.r_rank >= 0.75).mean()), len(low), len(pairs)


def validate_input(frame: pd.DataFrame, input_sha256: str) -> dict[str, Any]:
    required = {
        "run_id", "decision_id", "decision_progress", "progress_bin", *CANDIDATES,
        "lost_reachable_future_free_fraction", "broad_free_error", "row_integrity_ok",
        "u_r_union_evaluable", "primary_topology_risk_evaluable",
    }
    missing = sorted(required - set(frame.columns))
    if missing:
        raise ValueError(f"missing required columns: {missing}")
    if input_sha256 != EXPECTED_INPUT_SHA256:
        raise ValueError(f"shared evidence hash mismatch: {input_sha256}")
    keys = frame[["run_id", "decision_id"]]
    if keys.duplicated().any():
        raise ValueError("duplicate run_id/decision_id")
    observed = frame.groupby("run_id").size().to_dict()
    if observed != EXPECTED_COUNTS or len(frame) != 365:
        raise ValueError(f"row universe mismatch: {observed}")
    if not frame["row_integrity_ok"].astype(str).str.lower().eq("true").all():
        raise ValueError("row_integrity_ok failure")
    return {"rows": 365, "runs": list(RUNS), "counts": observed, "input_sha256": input_sha256}


def development_candidate(frame: pd.DataFrame, candidate: str) -> dict[str, Any]:
    run_stats = []
    for run_id, run in frame.groupby("run_id", sort=True):
        rho, pairs = valid_spearman(finite(run[candidate]), finite(run["lost_reachable_future_free_fraction"]))
        srr, low50_n, _ = ratio_low50(run[candidate], run["lost_reachable_future_free_fraction"])
        cw, low25_n, _ = cw25(run[candidate], run["lost_reachable_future_free_fraction"])
        run_stats.append({"run_id": run_id, "rho": rho, "pairs": pairs, "srr50": srr,
                          "cw25": cw, "low50_n": low50_n, "low25_n": low25_n})
    rhos = [x["rho"] for x in run_stats if x["rho"] is not None]
    srrs = [x["srr50"] for x in run_stats if x["srr50"] is not None]
    cws = [x["cw25"] for x in run_stats if x["cw25"] is not None]
    summary = {
        "candidate": candidate,
        "valid_spearman_runs": len(rhos),
        "positive_rho_runs": sum(x > 0 for x in rhos),
        "rho_dev_macro": float(np.mean(rhos)) if rhos else None,
        "srr50_dev": float(np.mean(srrs)) if srrs else None,
        "cw25_dev": float(np.mean(cws)) if cws else None,
        "run_statistics": run_stats,
    }
    summary["admissible"] = bool(
        len(rhos) >= 6 and summary["positive_rho_runs"] >= 5
        and summary["rho_dev_macro"] is not None and summary["rho_dev_macro"] > 0
        and summary["srr50_dev"] is not None and summary["srr50_dev"] < 1.0
        and summary["cw25_dev"] is not None and summary["cw25_dev"] < 0.25
    )
    return summary


def select_candidate(development: pd.DataFrame) -> tuple[str | None, list[dict[str, Any]]]:
    stats = [development_candidate(development, c) for c in CANDIDATES]
    admissible = [x for x in stats if x["admissible"]]
    if not admissible:
        return None, stats
    order = {name: i for i, name in enumerate(CANDIDATES)}
    chosen = sorted(admissible, key=lambda x: (-x["rho_dev_macro"], x["srr50_dev"], x["cw25_dev"], order[x["candidate"]]))[0]
    return chosen["candidate"], stats


def development_percentile(development: pd.DataFrame, column: str, values: pd.Series) -> np.ndarray:
    mapped = []
    vals = finite(values).to_numpy()
    for _, run in development.groupby("run_id", sort=True):
        ref = finite(run[column]).dropna().to_numpy()
        mapped.append(empirical_cdf(ref, vals))
    return np.nanmean(np.vstack(mapped), axis=0)


def _risk_summary(rows: pd.DataFrame, cutoff: float | None) -> dict[str, Any]:
    eligible = rows.dropna(subset=["R_primary", "U_dev_percentile"])
    retained = eligible if cutoff is None else eligible[eligible.U_dev_percentile <= cutoff]
    return {
        "rows": len(retained), "eligible_rows": len(eligible),
        "retained_fraction": float(len(retained) / len(eligible)) if len(eligible) else None,
        "mean": float(retained.R_primary.mean()) if len(retained) else None,
        "median": float(retained.R_primary.median()) if len(retained) else None,
        "p90": float(retained.R_primary.quantile(.9)) if len(retained) >= 10 else None,
    }


def score_fold(frame: pd.DataFrame, held_out_run: str) -> tuple[dict[str, Any], pd.DataFrame]:
    development = frame[frame.run_id != held_out_run].copy()
    held = frame[frame.run_id == held_out_run].copy()
    selected, candidate_stats = select_candidate(development)
    fold: dict[str, Any] = {
        "fold_id": f"holdout_{held_out_run}", "held_out_run": held_out_run,
        "development_runs": sorted(development.run_id.unique().tolist()),
        "selected_candidate": selected, "selection_status": "SELECTED" if selected else "NO_ADMISSIBLE_U",
        "candidate_statistics": candidate_stats,
    }
    held["selected_candidate"] = selected or ""
    held["U_selected"] = np.nan
    held["U_dev_percentile"] = np.nan
    held["R_primary"] = finite(held["lost_reachable_future_free_fraction"])
    held["R_dev_percentile"] = np.nan
    if selected is None:
        fold.update({"primary_pairs": 0, "rho_primary": None, "srr50_run": None,
                     "cw25_rate_run": None, "low_u_primary_rows": 0, "severe_cw_events": []})
        return fold, held

    held["U_selected"] = finite(held[selected])
    u_valid = held.U_selected.notna()
    held.loc[u_valid, "U_dev_percentile"] = development_percentile(development, selected, held.loc[u_valid, "U_selected"])
    r_valid = held.R_primary.notna()
    held.loc[r_valid, "R_dev_percentile"] = development_percentile(development, "lost_reachable_future_free_fraction", held.loc[r_valid, "R_primary"])
    primary = held.dropna(subset=["U_selected", "R_primary", "U_dev_percentile", "R_dev_percentile"])
    rho, pair_n = valid_spearman(primary.U_selected, primary.R_primary)
    all_mean = float(primary.R_primary.mean()) if len(primary) else None
    low50 = primary[primary.U_dev_percentile <= .50]
    srr = float(low50.R_primary.mean() / all_mean) if len(low50) and all_mean and all_mean > 0 else None
    low25 = primary[primary.U_dev_percentile <= .25]
    cw = float((low25.R_dev_percentile >= .75).mean()) if len(low25) else None
    severe = low25[low25.R_primary >= .50]
    fold.update({
        "primary_pairs": pair_n, "rho_primary": rho, "srr50_run": srr,
        "cw25_rate_run": cw, "low_u_primary_rows": len(low25),
        "late_primary_rows": int(((held.progress_bin == "[0.75,1.00]") & held.R_primary.notna()).sum()),
        "risk_by_coverage": {str(c): _risk_summary(primary, c) for c in (.25, .50, .75)} | {"all": _risk_summary(primary, None)},
        "severe_cw_events": severe[["run_id", "decision_id", "R_primary", "U_dev_percentile"]].to_dict("records"),
    })
    broad = held.dropna(subset=["U_selected", "U_dev_percentile"]).copy()
    broad["broad_free_error"] = finite(broad["broad_free_error"])
    broad = broad.dropna(subset=["broad_free_error"])
    broad_rho, broad_n = valid_spearman(broad.U_selected, broad.broad_free_error)
    broad_low = broad[broad.U_dev_percentile <= .50]
    broad_mean = float(broad.broad_free_error.mean()) if len(broad) else None
    broad_srr = float(broad_low.broad_free_error.mean() / broad_mean) if len(broad_low) and broad_mean and broad_mean > 0 else None
    contradiction = broad_rho is not None and broad_srr is not None and broad_rho <= -.20 and broad_srr >= 1.10
    fold["broad_diagnostic"] = {"pairs": broad_n, "rho_valid": broad_rho is not None, "rho": broad_rho,
                                  "low_u_rows": len(broad_low), "srr50_valid": broad_srr is not None,
                                  "srr50": broad_srr, "contradiction": contradiction}
    stage = {}
    for bin_name in PROGRESS_BINS:
        subset = primary[primary.progress_bin == bin_name]
        bin_rho, bin_n = valid_spearman(subset.U_selected, subset.R_primary)
        bin_low = subset[subset.U_dev_percentile <= .50]
        baseline = float(subset.R_primary.mean()) if len(subset) else None
        bin_srr = float(bin_low.R_primary.mean() / baseline) if len(bin_low) and baseline and baseline > 0 else None
        stage[bin_name] = {"pairs": bin_n, "rho": bin_rho, "srr50": bin_srr, "low_u_rows": len(bin_low)}
    fold["stage_diagnostics"] = stage
    return fold, held


def classify(folds: list[dict[str, Any]]) -> dict[str, Any]:
    selected = [f for f in folds if f["selection_status"] == "SELECTED"]
    no_u = len(folds) - len(selected)
    valid_pairs = sum(f.get("primary_pairs", 0) >= 5 for f in selected)
    valid_srr = sum(f.get("srr50_run") is not None for f in selected)
    valid_cw = sum(f.get("low_u_primary_rows", 0) >= 1 for f in selected)
    sufficiency = len(selected) >= 8 and valid_pairs >= 8 and valid_srr >= 8 and valid_cw >= 6
    rhos = [f["rho_primary"] for f in selected if f.get("rho_primary") is not None]
    srrs = [f["srr50_run"] for f in selected if f.get("srr50_run") is not None]
    cws = [f["cw25_rate_run"] for f in selected if f.get("cw25_rate_run") is not None]
    severe_runs = sum(bool(f.get("severe_cw_events")) for f in selected)
    broad_contradictions = sum(bool(f.get("broad_diagnostic", {}).get("contradiction")) for f in selected)
    stage_summary = {}
    stage_bad = False
    for bin_name in PROGRESS_BINS:
        brhos = [f["stage_diagnostics"][bin_name]["rho"] for f in selected if f.get("stage_diagnostics", {}).get(bin_name, {}).get("rho") is not None]
        bsrr = [f["stage_diagnostics"][bin_name]["srr50"] for f in selected if f.get("stage_diagnostics", {}).get(bin_name, {}).get("srr50") is not None]
        bad = len(brhos) >= 5 and len(bsrr) >= 5 and float(np.median(brhos)) <= -.10 and float(np.mean(bsrr)) >= 1.0
        stage_bad |= bad
        stage_summary[bin_name] = {"rho_runs": len(brhos), "median_rho": float(np.median(brhos)) if brhos else None,
                                   "srr_runs": len(bsrr), "mean_srr50": float(np.mean(bsrr)) if bsrr else None, "contradiction": bad}
    summary = {
        "no_admissible_u_folds": no_u, "selected_folds": len(selected),
        "held_out_runs_with_5_pairs": valid_pairs, "valid_srr50_runs": valid_srr,
        "runs_with_low_u_primary_support": valid_cw, "general_sufficiency": sufficiency,
        "median_primary_rho": float(np.median(rhos)) if rhos else None,
        "positive_rho_runs": sum(x > 0 for x in rhos),
        "mean_srr50": float(np.mean(srrs)) if srrs else None,
        "mean_cw25_rate": float(np.mean(cws)) if cws else None,
        "severe_cw_runs": severe_runs, "broad_contradiction_runs": broad_contradictions,
        "stage_summary": stage_summary,
    }
    if no_u >= 5:
        outcome, reasons = "FAIL", ["NO_ADMISSIBLE_U occurred in at least 5/10 folds"]
    elif not sufficiency:
        outcome, reasons = "INSUFFICIENT_EVIDENCE", ["general primary all-stage sufficiency not met"]
    else:
        fail = []
        if summary["mean_srr50"] >= 1.0: fail.append("mean SRR50 >= 1.0")
        if summary["mean_cw25_rate"] >= .25: fail.append("mean CW25 rate >= 0.25")
        if severe_runs >= 2: fail.append("severe confidently-wrong events in at least 2 runs")
        if summary["median_primary_rho"] <= 0 and summary["positive_rho_runs"] <= 4: fail.append("non-positive rank direction")
        if broad_contradictions >= 6: fail.append("strong broad contradiction veto")
        support = (summary["median_primary_rho"] >= .20 and summary["positive_rho_runs"] >= 7
                   and summary["mean_srr50"] <= .80 and summary["mean_cw25_rate"] <= .15
                   and severe_runs <= 1 and not stage_bad and broad_contradictions < 6)
        if fail:
            outcome, reasons = "FAIL", fail
        elif support:
            outcome, reasons = "HISTORICAL_FEASIBILITY_SUPPORTED_PENDING_CONFIRMATION", ["all frozen historical-feasibility criteria met"]
        else:
            outcome, reasons = "INSUFFICIENT_EVIDENCE", ["results lie between frozen support and FAIL criteria"]
    summary.update({"outcome": outcome, "reason_chain": reasons})
    return summary


def bootstrap(folds: list[dict[str, Any]], seed: int = 20260924, replicates: int = 10_000) -> dict[str, Any]:
    rng = np.random.default_rng(seed)
    specs = {"median_primary_rho": ("rho_primary", np.median), "mean_srr50": ("srr50_run", np.mean),
             "mean_cw25_rate": ("cw25_rate_run", np.mean)}
    result = {"seed": seed, "replicates": replicates, "interval": "95% percentile", "metrics": {}}
    for name, (field, reducer) in specs.items():
        vals = np.asarray([f[field] for f in folds if f.get(field) is not None], dtype=float)
        draws = reducer(rng.choice(vals, size=(replicates, len(vals)), replace=True), axis=1) if len(vals) else np.array([])
        result["metrics"][name] = {"runs": len(vals), "lower": float(np.quantile(draws, .025)) if len(draws) else None,
                                    "upper": float(np.quantile(draws, .975)) if len(draws) else None}
    return result


def json_dump(data: Any, path: Path) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
