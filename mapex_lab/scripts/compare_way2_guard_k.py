#!/usr/bin/env python3
"""Compare Way2 debounce guard K for focused lambda values.

Uses the same frozen historical formulation:
  G = raw information_gain
  C = Euclidean distance_m
  R_t = max_f G/C over selectable candidates

For each lambda and K, this script finds the first stop after K consecutive
EVALUABLE decisions with R_t <= lambda, then reports post-stop rebounds and
reconstructed time/distance savings using metrics.csv.

Development/diagnostic use only. It does not freeze lambda or K.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


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


def resolve_run(root: Path, value: str) -> Path:
    p = Path(value).expanduser()
    if p.is_dir():
        return p.resolve()
    p = root / "experiments" / "mapex" / value
    if p.is_dir():
        return p.resolve()
    raise FileNotFoundError(value)


def environment(run_dir: Path) -> str:
    import json
    p = run_dir / "metadata.json"
    if p.is_file():
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            return str(d.get("environment") or "unknown")
        except Exception:
            pass
    return "unknown"


def decision_ratios(run_dir: Path):
    rows = read_csv(run_dir / "candidates.csv")
    grouped = defaultdict(list)
    for r in rows:
        did = fint(r.get("decision_id"))
        if did >= 0:
            grouped[did].append(r)
    out = []
    for did in sorted(grouped):
        vals = []
        for r in grouped[did]:
            if fint(r.get("selectable"), 0) != 1:
                continue
            g = ffloat(r.get("information_gain"))
            c = ffloat(r.get("distance_m"))
            if math.isfinite(g) and math.isfinite(c) and c > 0:
                vals.append(g / c)
        if vals:
            out.append((did, max(vals)))
    return out


def decision_times(run_dir: Path):
    p = run_dir / "decisions.csv"
    result = {}
    if not p.is_file():
        return result
    for r in read_csv(p):
        did = fint(r.get("decision_id"))
        t = ffloat(r.get("time_s"))
        if did >= 0 and math.isfinite(t):
            result[did] = t
    return result


def metric_at_or_before(run_dir: Path, stop_time: float):
    p = run_dir / "metrics.csv"
    if not p.is_file() or not math.isfinite(stop_time):
        return None, None, None, None
    rows = read_csv(p)
    parsed = []
    for r in rows:
        t = ffloat(r.get("time_s"))
        if not math.isfinite(t):
            continue
        dist = ffloat(r.get("distance_m"))
        parsed.append((t, dist))
    if not parsed:
        return None, None, None, None
    parsed.sort()
    full_t, full_d = parsed[-1]
    eligible = [x for x in parsed if x[0] <= stop_time]
    if not eligible:
        return None, full_t, None, full_d
    stop_t, stop_d = eligible[-1]
    return stop_t, full_t, stop_d, full_d


def analyze(run_dir: Path, lam: float, k: int):
    seq = decision_ratios(run_dir)
    times = decision_times(run_dir)
    streak = 0
    stop_idx = None
    for i, (_, ratio) in enumerate(seq):
        if ratio <= lam:
            streak += 1
        else:
            streak = 0
        if streak >= k:
            stop_idx = i
            break

    base = {
        "run_id": run_dir.name,
        "environment": environment(run_dir),
        "lambda": lam,
        "K": k,
        "evaluable": len(seq),
        "triggered": False,
        "stop_decision": None,
        "stop_fraction": None,
        "time_saved_fraction": None,
        "distance_saved_fraction": None,
        "rebound_count": 0,
        "rebound_fraction": None,
        "max_post_R": None,
        "max_excess": None,
    }
    if stop_idx is None:
        return base

    stop_did, _ = seq[stop_idx]
    post = seq[stop_idx + 1 :]
    rebounds = [(d, r) for d, r in post if r > lam]
    stop_time = times.get(stop_did, math.nan)
    st, ft, sd, fd = metric_at_or_before(run_dir, stop_time)

    base.update(
        triggered=True,
        stop_decision=stop_did,
        stop_fraction=(stop_idx + 1) / len(seq) if seq else None,
        rebound_count=len(rebounds),
        rebound_fraction=(len(rebounds) / len(post)) if post else 0.0,
        max_post_R=max((r for _, r in post), default=math.nan),
    )
    if math.isfinite(base["max_post_R"]):
        base["max_excess"] = base["max_post_R"] - lam
    if st is not None and ft and ft > 0:
        base["time_saved_fraction"] = (ft - st) / ft
    if sd is not None and fd and fd > 0:
        base["distance_saved_fraction"] = (fd - sd) / fd
    return base


def median(vals):
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    return statistics.median(vals) if vals else None


def pct(v):
    return "n/a" if v is None or not math.isfinite(v) else f"{100*v:.1f}%"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*", default=[*(f"mpx_{i:03d}" for i in range(1,16)), "hpx_001", "hpx_002"])
    ap.add_argument("--lambdas", nargs="+", type=float, default=[0.4, 0.5])
    ap.add_argument("--ks", nargs="+", type=int, default=[2, 3, 4])
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    runs = [resolve_run(root, x) for x in args.runs]
    results = [analyze(r, lam, k) for lam in args.lambdas for k in args.ks for r in runs]

    for lam in args.lambdas:
        print(f"=== lambda={lam:g} ===")
        for k in args.ks:
            print(f"K={k}")
            subset = [x for x in results if x["lambda"] == lam and x["K"] == k]
            for env in ("new_room", "hospital"):
                rows = [x for x in subset if x["environment"] == env]
                trig = [x for x in rows if x["triggered"]]
                rb_runs = [x for x in trig if x["rebound_count"] > 0]
                print(
                    f"  {env}: trigger={len(trig)}/{len(rows)}, "
                    f"median time_saved={pct(median([x['time_saved_fraction'] for x in trig]))}, "
                    f"median distance_saved={pct(median([x['distance_saved_fraction'] for x in trig]))}, "
                    f"rebound_runs={len(rb_runs)}/{len(trig)}"
                )
                if rb_runs:
                    details = ", ".join(
                        f"{x['run_id']}(d{x['stop_decision']}, maxR={x['max_post_R']:.3f})"
                        for x in rb_runs
                    )
                    print(f"    rebounds: {details}")
        print()

    print("Interpretation: prefer the smallest fixed K that removes most immediate rebounds without materially delaying stopping. Do not freeze K from savings alone.")


if __name__ == "__main__":
    main()
