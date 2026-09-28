#!/usr/bin/env python3
"""Deterministic offline development replay for frozen MX012 V2."""
from __future__ import annotations

import argparse, csv, gzip, json, math, statistics
from collections import Counter
from hashlib import sha256
from pathlib import Path

import pandas as pd

from mapex_lab.analysis.d1.mx013_online_stop import (
    OnlineDecision, RecognizerParams, compute_topology_feature,
    evaluate_run, replay_recognizer, sha256_file,
)

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT / "mapex_lab/analysis/d1/results/shared_phase0_evidence_v1/shared_phase0_evidence.csv"
TRUTH = ROOT / "mapex_lab/analysis/d1/results/oracle_stop_retro_v1/oracle_decisions.csv"
U_RESULTS = ROOT / "mapex_lab/analysis/d1/results/gate_u_v1/final/per_decision_results.csv"
EXPERIMENTS = ROOT / "mapex_lab/experiments/mapex"
RUNS = [f"mpx_{i:03d}" for i in range(1, 11)]
GRIDS = {
    "PRIMARY": [i / 100 for i in range(1, 16)],
    "SENSITIVITY_A": [i / 200 for i in range(1, 31)],
    "SENSITIVITY_B": [i / 200 for i in range(1, 41)],
}
KS = [1, 2, 3, 4]


def clean(v):
    if pd.isna(v): return ""
    if isinstance(v, bool): return "true" if v else "false"
    if isinstance(v, float): return format(v, ".17g")
    return v


def write_csv(path, rows, columns=None):
    rows = list(rows)
    columns = columns or (list(rows[0]) if rows else [])
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=columns, lineterminator="\n")
        w.writeheader()
        for row in rows: w.writerow({k: clean(row.get(k)) for k in columns})


def deterministic_gzip(path):
    target = path.with_suffix(path.suffix + ".gz")
    with path.open("rb") as src, target.open("wb") as raw:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0, compresslevel=9) as dst:
            for block in iter(lambda: src.read(1024*1024), b""): dst.write(block)
    path.unlink()
    return target


def truth_by_run(frame):
    out = {}
    for run, group in frame.groupby("run_id", sort=True):
        group = group.sort_values("decision_index")
        vals = group["OracleRemainingFraction_GT"].astype(float).tolist()
        qualifies = [math.isfinite(v) and v <= .04 for v in vals]
        suffix = [False] * len(vals)
        ok = True
        for i in range(len(vals)-1, -1, -1):
            ok = ok and qualifies[i]; suffix[i] = ok
        if not any(suffix): raise RuntimeError(f"no OracleStop_4 for {run}")
        first = suffix.index(True) + 1
        out[run] = [{"decision_index": i+1, "oracle_stop_4": i+1 == first,
                     "oracle_remaining_fraction_gt": vals[i],
                     "reachable_pair_connectivity_retention": group.iloc[i].get("reachable_pair_connectivity_retention"),
                     "reachable_future_free_retention": group.iloc[i].get("reachable_future_free_retention"),
                     "lost_reachable_future_free_fraction": group.iloc[i].get("lost_reachable_future_free_fraction"),
                     "fragmented": group.iloc[i].get("fragmented"),
                     "topology_evaluable": group.iloc[i].get("primary_topology_risk_evaluable")}
                    for i in range(len(vals))]
    return out


def build_inputs(output):
    cache = output / "precomputed_online_features.csv"
    if cache.exists():
        cached = pd.read_csv(cache, keep_default_na=True)
        inputs = {}
        for run, group in cached.groupby("run_id", sort=True):
            decisions = []
            for _, s in group.sort_values("decision_index").iterrows():
                values = s.to_dict()
                for key in ("r_evaluable","topo_valid","source_prediction_traversable"):
                    values[key] = str(values[key]).lower() == "true" if not isinstance(values[key], bool) else values[key]
                values["low_u_25"] = None if pd.isna(values["low_u_25"]) else (str(values["low_u_25"]).lower()=="true")
                decisions.append(OnlineDecision(**values))
            inputs[run] = decisions
        if sorted(inputs) == RUNS:
            return inputs
    evidence = pd.read_csv(EVIDENCE, keep_default_na=True)
    evidence = evidence[evidence.run_id.isin(RUNS)].sort_values(["run_id", "decision_index"])
    u = pd.read_csv(U_RESULTS, usecols=["run_id", "decision_id", "low_u_25"])
    evidence = evidence.merge(u, on=["run_id", "decision_id"], how="left", validate="one_to_one")
    rows, inputs = [], {}
    for run, group in evidence.groupby("run_id", sort=True):
        decisions = []
        for _, s in group.iterrows():
            row = s.to_dict(); run_dir = EXPERIMENTS / run
            feature = compute_topology_feature(row, run_dir)
            low_u = None if pd.isna(s.low_u_25) else bool(s.low_u_25)
            r_hat = float(s.RemainingFraction) if pd.notna(s.RemainingFraction) else math.nan
            d = OnlineDecision(
                run, int(s.decision_id), int(s.decision_index), float(s.decision_time_s), float(s.decision_progress),
                r_hat, bool(s.d1_runtime_region_evaluable) and math.isfinite(r_hat), str(s.d1_reason or ""),
                **feature, u_p95=float(s.U_p95) if pd.notna(s.U_p95) else math.nan,
                u_mean=float(s.U_mean) if pd.notna(s.U_mean) else math.nan,
                u_disagreement=float(s.U_disagreement) if pd.notna(s.U_disagreement) else math.nan,
                low_u_25=low_u, raw_map_sha256=str(s.raw_map_sha256),
                prediction_member_sha256s="|".join(str(s[f"{m}_map_sha256"]) for m in ("g1","g2","g3")),
                mean_map_sha256=str(s.mean_map_sha256), source_mode=str(s.d1_source_mode or ""),
                source_lower_time_s=float(s.d1_source_lower_time_s) if pd.notna(s.d1_source_lower_time_s) else math.nan,
                source_upper_time_s=float(s.d1_source_upper_time_s) if pd.notna(s.d1_source_upper_time_s) else math.nan,
            )
            decisions.append(d); rows.append(d.__dict__)
        inputs[run] = decisions
    write_csv(cache, rows)
    return inputs


def aggregate(run_rows):
    n = len(run_rows); fired = [r for r in run_rows if r["candidate_stop"] is not None]
    nonprem = [r["delay_decisions"] for r in run_rows if r["stop_status"] in ("ON_TARGET","LATE")]
    premature = sum(r["stop_status"] == "PREMATURE" for r in run_rows)
    severe = sum(r["severe_false_stop_10"] for r in run_rows)
    return dict(run_count=n, premature_stops=premature, severe_false_stop_10=severe,
                stop_count=len(fired), stop_coverage=len(fired)/n,
                positive_saving_count=sum(r["positive_saving"] for r in run_rows),
                positive_saving_coverage=sum(r["positive_saving"] for r in run_rows)/n,
                median_nonpremature_delay=statistics.median(nonprem) if nonprem else None,
                mean_nonpremature_delay=statistics.mean(nonprem) if nonprem else None,
                mean_saved_progress=statistics.mean(r["saved_progress"] for r in run_rows))


def admissible(summary, interior=True):
    delay = summary["median_nonpremature_delay"]
    return bool(interior and summary["premature_stops"] == 0 and summary["severe_false_stop_10"] == 0
                and summary["stop_count"] >= math.ceil(.8*summary["run_count"])
                and summary["positive_saving_count"] >= math.ceil(.7*summary["run_count"])
                and delay is not None and delay <= 3)


def select(rows, grid, run_set):
    candidates = []
    lo, hi = min(GRIDS[grid]), max(GRIDS[grid])
    for (tau, k), group in rows.groupby(["tau_r","k"], sort=True):
        subset = group[group.run_id.isin(run_set)].to_dict("records")
        s = aggregate(subset); interior = tau > lo and tau < hi
        if admissible(s, interior):
            candidates.append((s["median_nonpremature_delay"], s["mean_nonpremature_delay"],
                               -s["stop_coverage"], -s["mean_saved_progress"], tau, -k, s))
    if not candidates: return None
    best = min(candidates)
    return dict(tau_r=best[4], k=-best[5], **best[6])


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    inputs = build_inputs(output)
    truth = truth_by_run(pd.read_csv(TRUTH))
    trace_rows, result_rows = [], []
    for grid, taus in GRIDS.items():
        for tau in taus:
            for k in KS:
                for run_id in RUNS:
                    trace = replay_recognizer(inputs[run_id], RecognizerParams(tau, k))
                    ev = evaluate_run(trace, truth[run_id])
                    result_rows.append(dict(grid=grid, run_id=run_id, tau_r=tau, k=k, variant="A0", **ev))
                    for x in trace: trace_rows.append(dict(grid=grid, **x))
    write_csv(output / "per_decision_all_grids.csv", trace_rows)
    deterministic_gzip(output / "per_decision_all_grids.csv")
    write_csv(output / "per_run_all_grids.csv", result_rows)
    results = pd.DataFrame(result_rows)

    aggregates = []
    for (grid,tau,k), group in results.groupby(["grid","tau_r","k"], sort=True):
        s = aggregate(group.to_dict("records")); lo,hi=min(GRIDS[grid]),max(GRIDS[grid])
        aggregates.append(dict(grid=grid,tau_r=tau,k=k,interior=tau>lo and tau<hi,
                               admissible=admissible(s,tau>lo and tau<hi),**s))
    write_csv(output / "pair_aggregates.csv", aggregates)

    primary = results[results.grid == "PRIMARY"]
    folds=[]
    for held in RUNS:
        sel=select(primary,"PRIMARY",[r for r in RUNS if r != held])
        if sel is None: folds.append(dict(held_out=held,status="NO_ADMISSIBLE_ONLINE_RULE")); continue
        held_row=primary[(primary.run_id==held)&(primary.tau_r==sel["tau_r"])&(primary.k==sel["k"])].iloc[0].to_dict()
        folds.append(dict(held_out=held,status="ADMISSIBLE",selected_tau_r=sel["tau_r"],selected_k=sel["k"],
                          held_out_stop_status=held_row["stop_status"],held_out_candidate_stop=held_row["candidate_stop"],
                          held_out_oracle_stop=held_row["oracle_stop"],held_out_delay_decisions=held_row["delay_decisions"],
                          held_out_positive_saving=held_row["positive_saving"],held_out_saved_decisions=held_row["saved_decisions"],
                          held_out_saved_progress=held_row["saved_progress"],held_out_severe_false_stop_10=held_row["severe_false_stop_10"]))
    write_csv(output / "loro_folds.csv", folds)

    full={grid:select(results[results.grid==grid],grid,RUNS) for grid in GRIDS}
    a0=full["PRIMARY"]
    criteria={}
    valid_folds=[f for f in folds if f["status"]=="ADMISSIBLE"]
    criteria["S1"] = len(valid_folds)>=8
    criteria["S2"] = bool(a0 and sum(abs(f["selected_tau_r"]-a0["tau_r"])<=.01+1e-12 for f in valid_folds)>=7
                          and valid_folds and max(f["selected_tau_r"] for f in valid_folds)-min(f["selected_tau_r"] for f in valid_folds)<=.04+1e-12)
    criteria["S3"] = bool(a0 and sum(f["selected_k"]==a0["k"] for f in valid_folds)>=7
                          and valid_folds and max(f["selected_k"] for f in valid_folds)-min(f["selected_k"] for f in valid_folds)<=1)
    held=[f for f in valid_folds]
    criteria["S4"] = len(held)==10 and not any(f["held_out_stop_status"]=="PREMATURE" or f["held_out_severe_false_stop_10"] for f in held)
    delays=[f["held_out_delay_decisions"] for f in held if f["held_out_stop_status"] in ("ON_TARGET","LATE")]
    criteria["S5"] = (len(held)==10 and sum(f["held_out_candidate_stop"] is not None for f in held)>=8
                       and sum(bool(f["held_out_positive_saving"]) for f in held)>=7
                       and bool(delays) and statistics.median(delays)<=3)
    sa,sb=full["SENSITIVITY_A"],full["SENSITIVITY_B"]
    criteria["S6"] = bool(a0 and sa and abs(sa["tau_r"]-a0["tau_r"])<=.01+1e-12 and sa["k"]==a0["k"])
    criteria["S7"] = bool(a0 and sb and abs(sb["tau_r"]-a0["tau_r"])<=.02+1e-12 and sb["k"]==a0["k"]
                          and min(GRIDS["SENSITIVITY_B"])<sb["tau_r"]<max(GRIDS["SENSITIVITY_B"]))

    ablations=[]; ablation_traces=[]; ablation_per_run=[]; topology_at_stops=[]
    if a0:
        variants={"A0":RecognizerParams(a0["tau_r"],a0["k"]),"A1":RecognizerParams(a0["tau_r"],a0["k"],use_topology=False),
                  "A2":RecognizerParams(a0["tau_r"],1),"A3":RecognizerParams(a0["tau_r"],a0["k"],use_u_veto=True)}
        for name,p in variants.items():
            per=[]
            for run_id in RUNS:
                trace=replay_recognizer(inputs[run_id],p); ev=evaluate_run(trace,truth[run_id]); record=dict(run_id=run_id,variant=name,**ev); per.append(record); ablation_per_run.append(record)
                if ev["candidate_stop"] is not None:
                    tr=truth[run_id][int(ev["candidate_stop"])-1]
                    topology_at_stops.append(dict(run_id=run_id,variant=name,candidate_stop=ev["candidate_stop"],
                        reachable_pair_connectivity_retention=tr["reachable_pair_connectivity_retention"],
                        reachable_future_free_retention=tr["reachable_future_free_retention"],
                        lost_reachable_future_free_fraction=tr["lost_reachable_future_free_fraction"],
                        fragmented=tr["fragmented"],topology_evaluable=tr["topology_evaluable"],gating_effect="NONE_EVALUATOR_ONLY"))
                for x in trace: ablation_traces.append(dict(variant=name,**x))
            ablations.append(dict(variant=name,**aggregate(per)))
    write_csv(output/"ablation_per_decision.csv",ablation_traces)
    deterministic_gzip(output/"ablation_per_decision.csv")
    write_csv(output/"ablation_per_run.csv",ablation_per_run)
    write_csv(output/"ablation_summary.csv",ablations)
    write_csv(output/"retrospective_topology_at_stops.csv",topology_at_stops)

    selected_rows=[]
    if a0:
        selected_rows=primary[(primary.tau_r==a0["tau_r"])&(primary.k==a0["k"])].to_dict("records")
    write_csv(output/"selected_pair_per_run.csv",selected_rows)
    false_stops=[]
    for f in folds:
        if f.get("held_out_stop_status")=="PREMATURE" or f.get("held_out_severe_false_stop_10"):
            false_stops.append(dict(scope="LORO_HELD_OUT",run_id=f["held_out"],tau_r=f.get("selected_tau_r"),k=f.get("selected_k"),
                                    stop_status=f.get("held_out_stop_status"),candidate_stop=f.get("held_out_candidate_stop"),
                                    oracle_stop=f.get("held_out_oracle_stop"),delay_decisions=f.get("held_out_delay_decisions"),
                                    severe_false_stop_10=f.get("held_out_severe_false_stop_10")))
    for r in selected_rows:
        if r["stop_status"]=="PREMATURE" or r["severe_false_stop_10"]:
            false_stops.append(dict(scope="FULL_DEVELOPMENT",run_id=r["run_id"],tau_r=r["tau_r"],k=r["k"],
                                    stop_status=r["stop_status"],candidate_stop=r["candidate_stop"],oracle_stop=r["oracle_stop"],
                                    delay_decisions=r["delay_decisions"],severe_false_stop_10=r["severe_false_stop_10"]))
    write_csv(output/"false_stop_inventory.csv",false_stops,["scope","run_id","tau_r","k","stop_status","candidate_stop","oracle_stop","delay_decisions","severe_false_stop_10"])
    resets=Counter()
    missing=Counter()
    if a0:
        for run_id in RUNS:
            for x in replay_recognizer(inputs[run_id],RecognizerParams(a0["tau_r"],a0["k"])):
                if x["reset_reason"]: resets[x["reset_reason"]]+=1
                if not x["r_evaluable"]: missing[x["r_missing_reason"] or "UNKNOWN"]+=1
    write_csv(output/"missing_reset_inventory.csv",
              [dict(kind="RESET",reason=k,count=v) for k,v in sorted(resets.items())]+
              [dict(kind="R_MISSING",reason=k,count=v) for k,v in sorted(missing.items())])

    classification=("ONLINE_RECOGNIZER_DEVELOPMENT_CANDIDATE_READY_FOR_CONFIRMATION"
                    if a0 and all(criteria.values()) else "NO_STABLE_ONLINE_RECOGNIZER_CANDIDATE")
    summary=dict(task="MX013",development_runs=RUNS,architecture_status="DEVELOPMENT_ONLY_NOT_INDEPENDENTLY_VALIDATED",
                 full_development=full,criteria=criteria,classification=classification,
                 primary_grid=[1,15,1],sensitivity_a_grid=[.5,15,.5],sensitivity_b_grid=[.5,20,.5],k_grid=KS,
                 oracle_target="retrospective persistent-suffix T*=4%; evaluator-only")
    json_default=lambda x: x.item() if hasattr(x,"item") else str(x)
    (output/"summary.json").write_text(json.dumps(summary,indent=2,sort_keys=True,default=json_default)+"\n")
    inventory=[]
    for p in sorted(output.iterdir()):
        if p.is_file() and p.name != "artifact_manifest.json" and p.suffix != ".log": inventory.append({"file":p.name,"sha256":sha256_file(p),"bytes":p.stat().st_size})
    manifest=dict(inputs={"shared_evidence":sha256_file(EVIDENCE),"oracle_truth":sha256_file(TRUTH),"gate_u":sha256_file(U_RESULTS)},artifacts=inventory)
    (output/"artifact_manifest.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n")
    print(json.dumps(summary,sort_keys=True,default=json_default))


if __name__ == "__main__":
    ap=argparse.ArgumentParser(); ap.add_argument("--output",type=Path,default=ROOT/"mapex_lab/analysis/d1/results/mx013_online_stop_v1")
    run(ap.parse_args().output)
