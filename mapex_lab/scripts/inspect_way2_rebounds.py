#!/usr/bin/env python3
"""Inspect Way2 post-stop rebound trajectories for focused lambda values.

This is a diagnostic tool for development runs only. It recomputes

    R_t = max_f information_gain / distance_m

over selectable candidates, applies the K-distinct-state stop guard, and prints
all post-stop rebounds plus a short decision window around the hypothetical stop.

The goal is to determine whether a rebound is an immediate local fluctuation or
a later structural increase in frontier value. It does not choose/freeze lambda.
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _f(value, default=math.nan) -> float:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


def _i(value, default=0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _resolve_run(root: Path, value: str) -> Path:
    direct = Path(value).expanduser()
    if direct.is_dir():
        return direct.resolve()
    candidate = root / "experiments" / "mapex" / value
    if candidate.is_dir():
        return candidate.resolve()
    raise FileNotFoundError(f"Cannot resolve run: {value}")


def _signature(rows: list[dict[str, str]]) -> tuple:
    return tuple(sorted(
        (
            row.get("candidate_id", ""),
            row.get("row", ""),
            row.get("col", ""),
            row.get("information_gain", ""),
            row.get("distance_m", ""),
            row.get("selectable", ""),
            row.get("status", ""),
        )
        for row in rows
    ))


def _load_decisions(run_dir: Path) -> list[dict]:
    path = run_dir / "candidates.csv"
    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in _read_csv(path):
        decision_id = _i(row.get("decision_id"), -1)
        if decision_id >= 0:
            grouped[decision_id].append(row)

    output = []
    for decision_id in sorted(grouped):
        rows = grouped[decision_id]
        selectable = [row for row in rows if _i(row.get("selectable"), 0) == 1]
        ratios = []
        best_candidate = ""
        best_ratio = -math.inf
        for row in selectable:
            g = _f(row.get("information_gain"))
            c = _f(row.get("distance_m"))
            if not math.isfinite(g) or not math.isfinite(c) or c <= 0:
                continue
            ratio = g / c
            ratios.append(ratio)
            if ratio > best_ratio:
                best_ratio = ratio
                best_candidate = row.get("candidate_id", "")
        if not ratios:
            continue
        output.append({
            "decision_id": decision_id,
            "R": max(ratios),
            "best_candidate": best_candidate,
            "signature": _signature(rows),
        })
    return output


def _first_guarded_stop(decisions: list[dict], lambda_value: float, guard_k: int):
    streak = 0
    last_sig = None
    for idx, item in enumerate(decisions):
        if item["R"] <= lambda_value:
            if item["signature"] != last_sig:
                streak += 1
                last_sig = item["signature"]
        else:
            streak = 0
            last_sig = None
        if streak >= guard_k:
            return idx
    return None


def inspect_run(run_dir: Path, lambdas: list[float], guard_k: int, window: int) -> None:
    decisions = _load_decisions(run_dir)
    print(f"\n=== {run_dir.name} ===")
    if not decisions:
        print("no evaluable decisions")
        return

    for lambda_value in lambdas:
        stop_idx = _first_guarded_stop(decisions, lambda_value, guard_k)
        print(f"\nlambda={lambda_value:g}, K={guard_k}")
        if stop_idx is None:
            print("  no stop")
            continue

        stop = decisions[stop_idx]
        post = decisions[stop_idx + 1:]
        rebounds = [item for item in post if item["R"] > lambda_value]
        print(
            f"  stop: d{stop['decision_id']} R={stop['R']:.3f}; "
            f"post decisions={len(post)}; rebounds={len(rebounds)}"
        )

        if rebounds:
            first = rebounds[0]
            last = rebounds[-1]
            max_rb = max(rebounds, key=lambda item: item["R"])
            print(
                f"  first rebound: d{first['decision_id']} R={first['R']:.3f} "
                f"(+{first['decision_id'] - stop['decision_id']} decisions)"
            )
            print(
                f"  max rebound:   d{max_rb['decision_id']} R={max_rb['R']:.3f} "
                f"excess={max_rb['R'] - lambda_value:.3f}"
            )
            print(
                f"  last rebound:  d{last['decision_id']} R={last['R']:.3f} "
                f"(+{last['decision_id'] - stop['decision_id']} decisions)"
            )
            print("  rebound decisions:")
            for item in rebounds:
                print(
                    f"    d{item['decision_id']}: R={item['R']:.3f}, "
                    f"best_candidate={item['best_candidate']}"
                )
        else:
            print("  no post-stop rebound")

        lo = max(0, stop_idx - window)
        hi = min(len(decisions), stop_idx + window + 1)
        print(f"  local trajectory (±{window} evaluable decisions):")
        for idx in range(lo, hi):
            item = decisions[idx]
            marker = "STOP" if idx == stop_idx else ("RB" if item["R"] > lambda_value and idx > stop_idx else "")
            print(f"    d{item['decision_id']:>3}: R={item['R']:.3f} {marker}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Inspect Way2 rebound runs in detail.")
    parser.add_argument(
        "runs",
        nargs="*",
        default=["mpx_004", "mpx_012", "hpx_002"],
        help="Run IDs or directories (default: known rebound runs).",
    )
    parser.add_argument(
        "--lambdas",
        nargs="+",
        type=float,
        default=[0.4, 0.5],
        help="Lambda values to inspect (default: 0.4 0.5).",
    )
    parser.add_argument("--guard-k", type=int, default=2)
    parser.add_argument("--window", type=int, default=4)
    args = parser.parse_args()

    if args.guard_k < 1:
        parser.error("--guard-k must be >= 1")
    if args.window < 0:
        parser.error("--window must be >= 0")

    root = Path(__file__).resolve().parents[1]
    for value in args.runs:
        inspect_run(_resolve_run(root, value), args.lambdas, args.guard_k, args.window)

    print(
        "\nInterpretation: an immediate rebound suggests local score fluctuation; "
        "a delayed/large rebound suggests the utility sequence is genuinely non-monotonic."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
