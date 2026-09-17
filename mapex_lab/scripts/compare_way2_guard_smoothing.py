#!/usr/bin/env python3
"""Compare consecutive-state and trailing-mean guards for Way2.

Development diagnostic only. Uses the frozen historical definitions:
  G = recorded information_gain
  C = recorded Euclidean distance_m
  R_t = max_f(G/C) over selectable candidates

The goal is not to tune for maximum savings. It checks whether smoothing the noisy
R_t sequence gives a more stable stopping signal than simply increasing K.

Candidate smoothing guard:
  trailing mean of the last W evaluable R_t values <= lambda

Default comparison uses W=3 and W=4 against the existing K=2/3/4 logic.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def _read_csv(path: Path):
    with path.open("r", newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _f(v, default=math.nan):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return default
    return x if math.isfinite(x) else default


def _i(v, default=0):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return default


def _resolve(root: Path, value: str) -> Path:
    p = Path(value).expanduser()
    if p.is_dir():
        return p.resolve()
    p = root / "experiments" / "mapex" / value
    if p.is_dir():
        return p.resolve()
    raise FileNotFoundError(value)


def _environment(run_dir: Path) -> str:
    p = run_dir / "metadata.json"
    if not p.is_file():
        return "unknown"
    try:
        x = json.loads(p.read_text(encoding="utf-8"))
    except Exception:
        return "unknown"
    return str(x.get("environment") or "unknown")


def _decision_ratios(run_dir: Path):
    rows = _read_csv(run_dir / "candidates.csv")
    grouped = defaultdict(list)
    for r in rows:
        d = _i(r.get("decision_id"), -1)
        if d >= 0:
            grouped[d].append(r)
    out = []
    for d in sorted(grouped):
        vals = []
        for r in grouped[d]:
            if _i(r.get("selectable"), 0) != 1:
                continue
            g = _f(r.get("information_gain"))
            c = _f(r.get("distance_m"))
            if math.isfinite(g) and math.isfinite(c) and c > 0:
                vals.append(g / c)
        if vals:
            out.append((d, max(vals)))
    return out


def _metrics(run_dir: Path):
    p = run_dir / "metrics.csv"
    if not p.is_file():
        return []
    rows = []
    for r in _read_csv(p):
        t = _f(r.get("time_s"))
        dist = _f(r.get("distance_m"))
        if math.isfinite(t) and math.isfinite(dist):
            rows.append((t, dist))
    rows.sort()
    return rows


def _decision_times(run_dir: Path):
    p = run_dir / "decisions.csv"
    if not p.is_file():
        return {}
    out = {}
    for r in _read_csv(p):
        d = _i(r.get("decision_id"), -1)
        t = _f(r.get("time_s"))
        if d >= 0 and math.isfinite(t):
            out[d] = t
    return out


def _metric_at(metrics, t):
    best = None
    for row in metrics:
        if row[0] <= t:
            best = row
        else:
            break
    return best


def _stop_consecutive(seq, lam, k):
    streak = 0
    for idx, (d, r) in enumerate(seq):
        streak = streak + 1 if r <= lam else 0
        if streak >= k:
            return idx, d
    return None


def _stop_mean(seq, lam, w):
    if len(seq) < w:
        return None
    for idx in range(w - 1, len(seq)):
        mean_r = statistics.fmean(r for _, r in seq[idx - w + 1 : idx + 1])
        if mean_r <= lam:
            return idx, seq[idx][0]
    return None


def _evaluate(run_dir: Path, lam: float, method: str, param: int):
    seq = _decision_ratios(run_dir)
    if method == "K":
        hit = _stop_consecutive(seq, lam, param)
    else:
        hit = _stop_mean(seq, lam, param)
    env = _environment(run_dir)
    base = {
        "run_id": run_dir.name,
        "environment": env,
        "lambda": lam,
        "method": method,
        "param": param,
        "trigger": 0,
        "stop_decision": "",
        "stop_fraction": math.nan,
        "time_saved_fraction": math.nan,
        "distance_saved_fraction": math.nan,
        "rebound": 0,
        "max_post_R": math.nan,
    }
    if hit is None:
        return base
    idx, d = hit
    base["trigger"] = 1
    base["stop_decision"] = d
    base["stop_fraction"] = (idx + 1) / len(seq)
    post = [r for _, r in seq[idx + 1 :]]
    base["rebound"] = int(any(r > lam for r in post))
    base["max_post_R"] = max(post) if post else math.nan

    times = _decision_times(run_dir)
    metrics = _metrics(run_dir)
    if d in times and metrics:
        stop_m = _metric_at(metrics, times[d])
        full_m = metrics[-1]
        if stop_m and full_m[0] > 0 and full_m[1] > 0:
            base["time_saved_fraction"] = (full_m[0] - stop_m[0]) / full_m[0]
            base["distance_saved_fraction"] = (full_m[1] - stop_m[1]) / full_m[1]
    return base


def _med(xs):
    vals = [x for x in xs if isinstance(x, (int, float)) and math.isfinite(x)]
    return statistics.median(vals) if vals else math.nan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*", default=[*(f"mpx_{i:03d}" for i in range(1,16)), "hpx_001", "hpx_002"])
    ap.add_argument("--lambdas", nargs="+", type=float, default=[0.4, 0.5])
    args = ap.parse_args()

    root = Path(__file__).resolve().parents[1]
    run_dirs = [_resolve(root, r) for r in args.runs]
    rows = []
    configs = [("K",2),("K",3),("K",4),("M",3),("M",4)]
    for lam in args.lambdas:
        for method, param in configs:
            for run in run_dirs:
                rows.append(_evaluate(run, lam, method, param))

    print("Way2 guard comparison: K=consecutive, M=trailing mean window")
    for lam in args.lambdas:
        print(f"=== lambda={lam:g} ===")
        for method, param in configs:
            label = f"K={param}" if method == "K" else f"meanW={param}"
            print(label)
            for env in ("new_room", "hospital"):
                s = [r for r in rows if r["lambda"] == lam and r["method"] == method and r["param"] == param and r["environment"] == env]
                trig = [r for r in s if r["trigger"]]
                rb = [r for r in trig if r["rebound"]]
                print(
                    f"  {env}: trigger={len(trig)}/{len(s)}, "
                    f"median time_saved={_med([r['time_saved_fraction'] for r in trig])*100:.1f}%, "
                    f"median distance_saved={_med([r['distance_saved_fraction'] for r in trig])*100:.1f}%, "
                    f"rebound_runs={len(rb)}/{len(trig) if trig else 0}"
                )
                if rb:
                    print("    rebounds: " + ", ".join(f"{r['run_id']}(d{r['stop_decision']}, maxR={r['max_post_R']:.3f})" for r in rb))
    print("Interpretation: smoothing is preferable only if it reduces rebound without collapsing trigger coverage. This is development-only; do not freeze from savings alone.")


if __name__ == "__main__":
    main()
