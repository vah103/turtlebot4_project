#!/usr/bin/env python3
"""Inspect mpx_009 around the single near-threshold Way2 quality violation.

Development diagnostic only.

Focused candidate rule:
  R_t <= lambda, lambda in {0.3, 0.4}
  max_visible_per_m <= 10
  K=2

The current focused replay stops mpx_009 at d26 with observed-IoU loss
0.010516, only slightly above the 0.01 diagnostic threshold. This script prints
observed-only IoU, reconstructed IoU, R, visible-unknown guard, candidate count,
and savings around d22..final to determine whether one additional confirming
decision removes the loss or whether the completion signal remains genuinely
insufficient.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path

import audit_way2_observed_quality as obsq
import audit_way2_visible_unknown_guard as vug


def read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def ffloat(v, default=math.nan):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except Exception:
        return default


def fint(v, default=-1):
    try:
        return int(float(v))
    except Exception:
        return default


def main():
    root = Path(__file__).resolve().parents[1]
    run = root / "experiments" / "mapex" / "mpx_009"
    if not run.is_dir():
        raise FileNotFoundError(run)

    states = vug.candidate_state(run)
    qrows = obsq.decision_quality(run, root)
    q_by_d = {r["decision_id"]: r for r in qrows}
    final_obs = qrows[-1]["observed_iou"]

    early = {}
    for r in read_csv(run / "early_stopping_analysis.csv"):
        d = fint(r.get("decision_id"))
        if d >= 0:
            early[d] = r

    final_rec = math.nan
    if early:
        last = early[max(early)]
        iou = ffloat(last.get("iou_if_stop"))
        loss = ffloat(last.get("iou_loss_vs_final"), 0.0)
        if math.isfinite(iou) and math.isfinite(loss):
            final_rec = iou + loss

    print("=== mpx_009 quality trajectory ===")
    print("decision   R_t    guard(U/m)  cand  obsIoU   obs_loss   recIoU   rec_loss   time_saved  dist_saved")
    for d in sorted(states):
        if d < 22:
            continue
        s = states[d]
        q = q_by_d.get(d)
        e = early.get(d)
        if q is None:
            continue
        obs = q["observed_iou"]
        obs_loss = final_obs - obs
        rec = ffloat(e.get("iou_if_stop")) if e else math.nan
        rec_loss = ffloat(e.get("iou_loss_vs_final")) if e else math.nan
        ts = ffloat(e.get("time_saved_fraction")) if e else math.nan
        ds = ffloat(e.get("distance_saved_fraction")) if e else math.nan
        flags = []
        if d == 26:
            flags.append("STOP-K2")
        if d == 27:
            flags.append("STOP-K3")
        flag = " " + ",".join(flags) if flags else ""
        print(
            f"d{d:3d}  {s['R']:6.3f}  {s['max_visible_per_m']:10.3f}  "
            f"{s['candidate_count']:4d}  {obs:7.6f}  {obs_loss:9.6f}  "
            f"{rec:7.6f}  {rec_loss:9.6f}  {100*ts:9.2f}%  {100*ds:9.2f}%{flag}"
        )

    print(f"\nFinal observed-only IoU: {final_obs:.6f}")
    if math.isfinite(final_rec):
        print(f"Analyzer final reconstructed IoU: {final_rec:.6f}")

    if 26 in q_by_d and 27 in q_by_d:
        l26 = final_obs - q_by_d[26]["observed_iou"]
        l27 = final_obs - q_by_d[27]["observed_iou"]
        gain = q_by_d[27]["observed_iou"] - q_by_d[26]["observed_iou"]
        print("\nFocused comparison:")
        print(f"d26 observed loss = {l26:.6f}")
        print(f"d27 observed loss = {l27:.6f}")
        print(f"d26 -> d27 observed IoU gain = {gain:.6f}")
        if l27 <= 0.01:
            print("One additional confirming decision is enough to move this run below the 0.01 diagnostic loss threshold.")
        else:
            print("Even one additional confirming decision does not remove the quality violation; the completion guard itself remains insufficient here.")

    print("\nInterpretation: do not tune lambda or the visible-unknown threshold to this single run. Use this trace only to decide whether the remaining failure is a debounce issue or a missing completion signal.")


if __name__ == "__main__":
    main()
