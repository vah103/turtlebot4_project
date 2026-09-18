#!/usr/bin/env python3
"""Regenerate Way2 quality evidence after the New Room v2 GT/ROI migration."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import audit_way2_observed_quality as obsq
import audit_way2_visible_unknown_guard as vug
import evaluate_way2_adaptive_confirmation as adaptive

ROOT = Path(__file__).resolve().parents[1]
RUN_NAMES = [*(f"mpx_{i:03d}" for i in range(1, 16)), "hpx_001", "hpx_002"]
LAMBDAS = [0.25, 0.30, 0.35, 0.40]
THRESHOLDS = [7.5, 10.0, 12.5, 15.0]


def write_csv(path: Path, fields, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def pct(value):
    return None if value is None or not math.isfinite(value) else 100.0 * value


def fmt(value, digits=6):
    if value is None:
        return "n/a"
    if isinstance(value, float) and not math.isfinite(value):
        return "n/a"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def runs():
    out = []
    for name in RUN_NAMES:
        path = ROOT / "experiments/mapex" / name
        if not path.is_dir():
            raise FileNotFoundError(path)
        out.append(path)
    return out


def state_cache(run_dirs):
    return {run.name: vug.candidate_state(run) for run in run_dirs}


def evaluate_variant(run_dirs, states, lam, threshold):
    rows = []
    for run in run_dirs:
        stop = adaptive.first_adaptive_stop(states[run.name], lam, threshold)
        rows.append(adaptive.evaluate_stop(run, ROOT, states[run.name], stop))
    return rows, adaptive.summarize(rows)


def first_r_only_stop(states, lam, k):
    streak = 0
    for decision in sorted(states):
        ratio = states[decision].get("R", math.nan)
        if math.isfinite(ratio) and ratio <= lam:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            return decision
    return None


def observed_loss(run: Path, stop):
    quality = obsq.decision_quality(run, ROOT)
    by_d = {row["decision_id"]: row for row in quality}
    if not quality or stop is None or stop not in by_d:
        return math.nan, math.nan, math.nan
    final = quality[-1]["observed_iou"]
    at_stop = by_d[stop]["observed_iou"]
    return at_stop, final, final - at_stop


def build_failure_cases(states):
    cases = []
    hpx = ROOT / "experiments/mapex/hpx_001"
    for lam, k in ((0.4, 2), (0.4, 3), (0.5, 2)):
        stop = first_r_only_stop(states["hpx_001"], lam, k)
        at_stop, final, loss = observed_loss(hpx, stop)
        cases.append({
            "run": "hpx_001",
            "environment": "hospital",
            "configuration": f"lambda={lam:g},K={k}",
            "stop_decision": stop,
            "observed_iou_at_stop": at_stop,
            "observed_iou_final": final,
            "observed_iou_loss": loss,
            "why_it_mattered": "R-only structural-loss diagnostic regenerated under current evaluator",
        })

    mpx = ROOT / "experiments/mapex/mpx_009"
    for k, label in ((2, "R<=0.3,U<=10,fixed K=2"), (3, "R<=0.3,U<=10,fixed K=3")):
        stop = adaptive.first_fixed_k_stop(states["mpx_009"], 0.3, 10.0, k)
        at_stop, final, loss = observed_loss(mpx, stop)
        cases.append({
            "run": "mpx_009",
            "environment": "new_room",
            "configuration": label,
            "stop_decision": stop,
            "observed_iou_at_stop": at_stop,
            "observed_iou_final": final,
            "observed_iou_loss": loss,
            "why_it_mattered": "New Room confirmation diagnostic recomputed with structural GT v2",
        })

    stop = adaptive.first_adaptive_stop(states["hpx_001"], 0.3, 15.0)
    at_stop, final, loss = observed_loss(hpx, stop)
    cases.append({
        "run": "hpx_001",
        "environment": "hospital",
        "configuration": "R<=0.3,U<=15,adaptive",
        "stop_decision": stop,
        "observed_iou_at_stop": at_stop,
        "observed_iou_final": final,
        "observed_iou_loss": loss,
        "why_it_mattered": "aggressive U-threshold boundary diagnostic",
    })
    return cases


def main():
    run_dirs = runs()
    states = state_cache(run_dirs)
    _, frozen = evaluate_variant(run_dirs, states, 0.30, 10.0)

    neighborhood = []
    for lam in LAMBDAS:
        for threshold in THRESHOLDS:
            _, summary = evaluate_variant(run_dirs, states, lam, threshold)
            nr = summary["new_room"]
            hp = summary["hospital"]
            bad = nr["bad"] + hp["bad"]
            if bad == 0 and hp["trigger"] == hp["runs"]:
                status = "safe_plateau"
            elif bad == 0:
                status = "too_conservative"
            else:
                status = "quality_failure"
            neighborhood.append({
                "lambda": lam,
                "u_threshold_cells_per_m": threshold,
                "candidate_cutoff": 1,
                "new_room_trigger": nr["trigger"],
                "new_room_total": nr["runs"],
                "hospital_trigger": hp["trigger"],
                "hospital_total": hp["runs"],
                "observed_iou_violations_gt_0p01": bad,
                "new_room_worst_observed_iou_loss": nr["worst"],
                "hospital_worst_observed_iou_loss": hp["worst"],
                "status": status,
                "note": "frozen candidate" if lam == 0.30 and threshold == 10.0 else "",
            })

    write_csv(
        ROOT / "analysis/way2_threshold_neighborhood.csv",
        list(neighborhood[0].keys()),
        neighborhood,
    )

    failure_cases = build_failure_cases(states)
    write_csv(
        ROOT / "analysis/way2_results/quality_failure_cases.csv",
        list(failure_cases[0].keys()),
        failure_cases,
    )

    verified = [{
        "stage": "evaluation_profile",
        "environment": "new_room",
        "configuration": "new_room_structural_gt_v2 + new_room_connected_free_v2",
        "metric": "frame_alignment",
        "value": "slam_start",
        "interpretation": "all New Room quality evidence regenerated after world->SLAM frame correction",
    }]
    for env in ("new_room", "hospital"):
        s = frozen[env]
        for metric, val in (
            ("trigger", f"{s['trigger']}/{s['runs']}"),
            ("bad_observed_iou_loss_runs", f"{s['bad']}/{s['trigger']}" if s["trigger"] else "0/0"),
            ("median_time_saved_percent", pct(s["median_time"])),
            ("median_distance_saved_percent", pct(s["median_distance"])),
            ("worst_observed_iou_loss", s["worst"]),
        ):
            verified.append({
                "stage": "frozen_backtest_v2",
                "environment": env,
                "configuration": "R<=0.30,U<=10,cutoff=1,adaptive 2/3 confirmation",
                "metric": metric,
                "value": val,
                "interpretation": "development-only evidence regenerated from corrected evaluation artifacts",
            })
    for row in neighborhood:
        verified.append({
            "stage": "robustness_v2",
            "environment": "all",
            "configuration": f"R={row['lambda']},U<={row['u_threshold_cells_per_m']},cutoff=1",
            "metric": "trigger_and_quality",
            "value": (
                f"NR {row['new_room_trigger']}/{row['new_room_total']}; "
                f"H {row['hospital_trigger']}/{row['hospital_total']}; "
                f"bad={row['observed_iou_violations_gt_0p01']}"
            ),
            "interpretation": row["status"],
        })
    write_csv(
        ROOT / "analysis/way2_results/verified_development_results.csv",
        ["stage", "environment", "configuration", "metric", "value", "interpretation"],
        verified,
    )

    frozen_rule_path = ROOT / "analysis/way2_results/frozen_rule.json"
    frozen_rule = json.loads(frozen_rule_path.read_text(encoding="utf-8"))
    frozen_rule["evaluation_profile"] = {
        "new_room": "new_room_v2",
        "hospital": "hospital_v2",
    }
    frozen_rule["development_evidence"] = {}
    for env in ("new_room", "hospital"):
        s = frozen[env]
        frozen_rule["development_evidence"][env] = {
            "triggered_runs": f"{s['trigger']}/{s['runs']}",
            "bad_observed_iou_loss_runs": f"{s['bad']}/{s['trigger']}" if s["trigger"] else "0/0",
            "median_time_saved_percent": pct(s["median_time"]),
            "median_distance_saved_percent": pct(s["median_distance"]),
            "worst_observed_iou_loss": None if not math.isfinite(s["worst"]) else s["worst"],
        }
    passed = all(frozen[env]["bad"] == 0 for env in ("new_room", "hospital"))
    frozen_rule["v2_quality_revalidation_passed"] = passed
    frozen_rule["status"] = (
        "frozen_candidate_for_validation"
        if passed
        else "frozen_candidate_pending_quality_review_after_v2_migration"
    )
    frozen_rule_path.write_text(json.dumps(frozen_rule, indent=2) + "\n", encoding="utf-8")

    table_rows = [
        (
            f"| {row['lambda']:.2f} | {row['u_threshold_cells_per_m']:.1f} | "
            f"{row['new_room_trigger']}/{row['new_room_total']} | "
            f"{row['hospital_trigger']}/{row['hospital_total']} | "
            f"{row['observed_iou_violations_gt_0p01']} | {row['status']} |"
        )
        for row in neighborhood
    ]
    failure_rows = [
        (
            f"| {row['run']} | {row['configuration']} | {row['stop_decision']} | "
            f"{fmt(row['observed_iou_loss'])} |"
        )
        for row in failure_cases
    ]

    nr, hp = frozen["new_room"], frozen["hospital"]
    md = f"""# Way2 threshold selection evidence — v2 quality refresh

_Last regenerated from corrected evaluation artifacts._

New Room quality evidence in this document uses `new_room_structural_gt_v2` and
`new_room_connected_free_v2`, which transform SDF world geometry into the
SLAM-start frame before evaluation. Hospital remains on `hospital_v2`.

## Frozen online rule

```text
R_t = max information_gain / distance_m
U_t = max visible_unknown_cells / distance_m
base_valid := R_t <= 0.30 AND U_t <= 10.0

1st consecutive valid: continue
2nd valid: stop when candidate_count <= 1; otherwise require one more valid state
3rd consecutive valid: stop
```

The migration **does not retune** the frozen constants. It only regenerates the
quality evidence with the corrected evaluator.

## Corrected development backtest at the frozen candidate

| Environment | Trigger | Bad observed-IoU-loss runs (>0.01) | Median time saved | Median distance saved | Worst observed IoU loss |
|---|---:|---:|---:|---:|---:|
| New Room | {nr['trigger']}/{nr['runs']} | {nr['bad']}/{nr['trigger']} | {fmt(pct(nr['median_time']),2)}% | {fmt(pct(nr['median_distance']),2)}% | {fmt(nr['worst'])} |
| Hospital | {hp['trigger']}/{hp['runs']} | {hp['bad']}/{hp['trigger']} | {fmt(pct(hp['median_time']),2)}% | {fmt(pct(hp['median_distance']),2)}% | {fmt(hp['worst'])} |

v2 quality revalidation passed: **{'yes' if passed else 'no'}**.

## Local neighborhood regenerated under the corrected evaluator

| R threshold | U threshold | New Room trigger | Hospital trigger | IoU-loss violations >0.01 | Status |
|---:|---:|---:|---:|---:|---|
{chr(10).join(table_rows)}

## Regenerated focused quality cases

| Run | Configuration | Stop decision | Observed IoU loss |
|---|---|---:|---:|
{chr(10).join(failure_rows)}

## Data-use boundary

Development runs remain development/tuning evidence only. Post-freeze online
Way2 runs remain validation/implementation evidence and are not used to retune
`0.30`, `10.0`, cutoff `1`, or the 2/3 confirmation state machine.

Machine-readable current evidence is stored in:

- `analysis/way2_results/frozen_rule.json`
- `analysis/way2_results/verified_development_results.csv`
- `analysis/way2_results/quality_failure_cases.csv`
- `analysis/way2_threshold_neighborhood.csv`
- `analysis/way2_results/reproduced_logs/`
"""
    (ROOT / "analysis/WAY2_THRESHOLD_SELECTION.md").write_text(md, encoding="utf-8")

    print("Way2 v2 evidence refreshed.")
    print("v2 quality revalidation passed:", passed)
    for env in ("new_room", "hospital"):
        print(env, frozen[env])


if __name__ == "__main__":
    main()
