#!/usr/bin/env python3
"""Inspect Way2 candidate-rule quality failures from local backtest outputs.

Reads mapex_lab/experiments/mapex/way2_candidate_quality_runs.csv and prints:
- all Hospital rows for lambda 0.4/0.5 and K 2/3;
- any triggered run with IoU loss > 0.01;
- key stop/savings/TU fields when available.

Development-only diagnostic. Does not freeze the Way2 rule.
"""
from __future__ import annotations

import csv
import math
from pathlib import Path


def read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def f(v, default=math.nan):
    try:
        x = float(v)
        return x if math.isfinite(x) else default
    except Exception:
        return default


def i(v, default=-1):
    try:
        return int(float(v))
    except Exception:
        return default


def pick(row: dict[str, str], *names: str) -> str:
    for name in names:
        if name in row and row[name] not in (None, ""):
            return row[name]
    return ""


def pct(x: float) -> str:
    return "n/a" if not math.isfinite(x) else f"{100*x:.2f}%"


def show(row: dict[str, str]):
    run_id = pick(row, "run_id")
    env = pick(row, "environment")
    lam = f(pick(row, "lambda", "lambda_value"))
    k = i(pick(row, "K", "guard_k"))
    stop = pick(row, "stop_decision", "stop_decision_id") or "n/a"
    ts = f(pick(row, "time_saved_fraction"))
    ds = f(pick(row, "distance_saved_fraction"))
    iou = f(pick(row, "iou_loss", "iou_loss_vs_final"))
    tu = f(pick(row, "tu_loss", "tu_loss_vs_final"))
    print(
        f"{run_id:8s} env={env:9s} lambda={lam:g} K={k} stop_d={stop} "
        f"time_saved={pct(ts)} distance_saved={pct(ds)} "
        f"iou_loss={iou:.9f} tu_loss={tu:.9f}"
    )


def main():
    root = Path(__file__).resolve().parents[1]
    path = root / "experiments" / "mapex" / "way2_candidate_quality_runs.csv"
    if not path.is_file():
        raise SystemExit(f"Missing {path}; run evaluate_way2_candidate_quality.py first")

    rows = read_csv(path)
    focus = []
    for row in rows:
        lam = f(pick(row, "lambda", "lambda_value"))
        k = i(pick(row, "K", "guard_k"))
        if lam in (0.4, 0.5) and k in (2, 3):
            focus.append(row)

    print("=== Hospital candidate rules ===")
    hospital = [r for r in focus if pick(r, "environment") == "hospital"]
    for row in sorted(hospital, key=lambda r: (f(pick(r, "lambda", "lambda_value")), i(pick(r, "K", "guard_k")), pick(r, "run_id"))):
        show(row)

    print("\n=== IoU loss > 0.01 ===")
    bad = [r for r in focus if f(pick(r, "iou_loss", "iou_loss_vs_final")) > 0.01]
    if not bad:
        print("none")
    else:
        for row in sorted(bad, key=lambda r: (pick(r, "environment"), pick(r, "run_id"), f(pick(r, "lambda", "lambda_value")), i(pick(r, "K", "guard_k")))):
            show(row)

    print("\nInterpretation: if the same Hospital run repeatedly dominates IoU loss, inspect its stop decision and local R_t trajectory before changing lambda/K. Do not retune to one run blindly.")


if __name__ == "__main__":
    main()
