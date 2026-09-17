#!/usr/bin/env python3
"""Compare candidate Way2 gain definitions before freezing G or lambda.

This is an OFFLINE development diagnostic. It does not modify MapEx, choose a
lambda, or claim that visible-normalized gain is preferable.

Two gain definitions are compared on the same recorded selectable frontiers:

    raw:
        G_raw(f) = information_gain(f)

    visible-normalized:
        G_norm(f) = information_gain(f) / visible_unknown_cells(f)

For each evaluable decision, each definition independently computes its best
one-step gain/cost ratio using the recorded Euclidean distance C(f):

    R_raw(t)  = max_f G_raw(f)  / C(f)
    R_norm(t) = max_f G_norm(f) / C(f)

The normalized definition is intentionally a diagnostic candidate only. Dividing
by visible cells removes the explicit reward for seeing a larger unknown region,
which may or may not be desirable for MapEx.

To reduce sensitivity to the arbitrary spawn view, summaries report:
- first decision separately;
- early 25%, middle 50%, and late 25% of EVALUABLE decisions;
- post-early 75% (middle + late) as a spawn-reduced comparison window.

The script answers four design questions:
1. Does normalization reduce first-decision/spawn sensitivity?
2. Does it make New Room and Hospital scales more comparable after the early phase?
3. Does the stopping signal still decrease toward the end of a run?
4. How often would normalization change the best frontier under G/C?
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


DECISION_FIELDS = [
    "run_id",
    "environment",
    "decision_id",
    "evaluable_index",
    "phase",
    "candidate_selectable",
    "raw_best_candidate_id",
    "raw_best_gain",
    "raw_best_distance_m",
    "raw_best_ratio",
    "norm_best_candidate_id",
    "norm_best_gain",
    "norm_best_raw_gain",
    "norm_best_visible_unknown_cells",
    "norm_best_distance_m",
    "norm_best_ratio",
    "best_candidate_agreement",
]

RUN_FIELDS = [
    "run_id",
    "environment",
    "evaluable_decisions",
    "candidate_agreement_fraction",
    "raw_first_ratio",
    "norm_first_ratio",
    "raw_early_median",
    "norm_early_median",
    "raw_middle_median",
    "norm_middle_median",
    "raw_late_median",
    "norm_late_median",
    "raw_post_early_median",
    "norm_post_early_median",
    "raw_late_to_middle_ratio",
    "norm_late_to_middle_ratio",
    "raw_first_to_post_early_ratio",
    "norm_first_to_post_early_ratio",
]


def _read_csv(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _finite_float(value, default=math.nan) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _int_value(value, default=0) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError):
        return default


def _median(values: list[float]) -> float:
    finite = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.median(finite) if finite else math.nan


def _safe_ratio(numerator: float, denominator: float) -> float:
    if not math.isfinite(numerator) or not math.isfinite(denominator):
        return math.nan
    if denominator <= 0.0:
        return math.nan
    return numerator / denominator


def _fmt(value) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    if math.isnan(number):
        return "nan"
    if math.isinf(number):
        return "inf" if number > 0 else "-inf"
    return f"{number:.9f}"


def _resolve_run(root: Path, value: str) -> Path:
    direct = Path(value).expanduser()
    if direct.is_dir():
        return direct.resolve()
    candidate = root / "experiments" / "mapex" / value
    if candidate.is_dir():
        return candidate.resolve()
    raise FileNotFoundError(
        f"Cannot resolve run '{value}'. Expected a run directory or {candidate}"
    )


def _load_environment(run_dir: Path) -> str:
    path = run_dir / "metadata.json"
    if not path.is_file():
        return "unknown"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return "unknown"
    if not isinstance(payload, dict):
        return "unknown"
    return str(payload.get("environment") or "unknown")


def _validate_schema(fieldnames: list[str] | None, path: Path) -> None:
    required = {
        "decision_id",
        "candidate_id",
        "information_gain",
        "distance_m",
        "visible_unknown_cells",
        "selectable",
    }
    missing = sorted(required - set(fieldnames or []))
    if missing:
        raise ValueError(
            f"{path} is missing required comparison fields: {', '.join(missing)}"
        )


def _phase(index: int, count: int) -> str:
    """Quartile-like phase over evaluable decisions, preserving a 50% middle."""
    if count <= 1:
        return "early"
    fraction = index / count  # index is 1-based
    if fraction <= 0.25:
        return "early"
    if fraction <= 0.75:
        return "middle"
    return "late"


def _analyze_run(run_dir: Path) -> tuple[list[dict[str, str]], dict]:
    candidates_path = run_dir / "candidates.csv"
    if not candidates_path.is_file():
        raise FileNotFoundError(f"Missing candidates.csv: {candidates_path}")

    with candidates_path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        _validate_schema(reader.fieldnames, candidates_path)
        rows = list(reader)

    grouped: dict[int, list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        decision_id = _int_value(row.get("decision_id"), -1)
        if decision_id >= 0:
            grouped[decision_id].append(row)

    environment = _load_environment(run_dir)
    decision_results: list[dict] = []

    for decision_id in sorted(grouped):
        candidates = []
        for row in grouped[decision_id]:
            if _int_value(row.get("selectable"), 0) != 1:
                continue
            gain = _finite_float(row.get("information_gain"))
            distance = _finite_float(row.get("distance_m"))
            visible = _finite_float(row.get("visible_unknown_cells"))
            if (
                not math.isfinite(gain)
                or not math.isfinite(distance)
                or distance <= 0.0
                or not math.isfinite(visible)
                or visible < 0.0
            ):
                continue

            raw_ratio = gain / distance
            norm_gain = gain / visible if visible > 0.0 else math.nan
            norm_ratio = norm_gain / distance if math.isfinite(norm_gain) else math.nan
            candidates.append(
                {
                    "candidate_id": str(row.get("candidate_id", "")),
                    "raw_gain": gain,
                    "distance": distance,
                    "visible": visible,
                    "raw_ratio": raw_ratio,
                    "norm_gain": norm_gain,
                    "norm_ratio": norm_ratio,
                }
            )

        if not candidates:
            continue

        raw_best = max(candidates, key=lambda item: item["raw_ratio"])
        norm_valid = [item for item in candidates if math.isfinite(item["norm_ratio"])]
        if not norm_valid:
            # Cannot compare definitions for a decision with no positive visible count.
            continue
        norm_best = max(norm_valid, key=lambda item: item["norm_ratio"])

        decision_results.append(
            {
                "decision_id": decision_id,
                "candidate_selectable": len(candidates),
                "raw_best": raw_best,
                "norm_best": norm_best,
                "agreement": raw_best["candidate_id"] == norm_best["candidate_id"],
            }
        )

    n = len(decision_results)
    output_rows: list[dict[str, str]] = []
    for index, result in enumerate(decision_results, start=1):
        raw_best = result["raw_best"]
        norm_best = result["norm_best"]
        phase = _phase(index, n)
        output_rows.append(
            {
                "run_id": run_dir.name,
                "environment": environment,
                "decision_id": str(result["decision_id"]),
                "evaluable_index": str(index),
                "phase": phase,
                "candidate_selectable": str(result["candidate_selectable"]),
                "raw_best_candidate_id": raw_best["candidate_id"],
                "raw_best_gain": _fmt(raw_best["raw_gain"]),
                "raw_best_distance_m": _fmt(raw_best["distance"]),
                "raw_best_ratio": _fmt(raw_best["raw_ratio"]),
                "norm_best_candidate_id": norm_best["candidate_id"],
                "norm_best_gain": _fmt(norm_best["norm_gain"]),
                "norm_best_raw_gain": _fmt(norm_best["raw_gain"]),
                "norm_best_visible_unknown_cells": _fmt(norm_best["visible"]),
                "norm_best_distance_m": _fmt(norm_best["distance"]),
                "norm_best_ratio": _fmt(norm_best["norm_ratio"]),
                "best_candidate_agreement": "1" if result["agreement"] else "0",
            }
        )

    def ratios(kind: str, phases: set[str] | None = None) -> list[float]:
        field = "raw_best_ratio" if kind == "raw" else "norm_best_ratio"
        return [
            _finite_float(row[field])
            for row in output_rows
            if phases is None or row["phase"] in phases
        ]

    raw_early = _median(ratios("raw", {"early"}))
    norm_early = _median(ratios("norm", {"early"}))
    raw_middle = _median(ratios("raw", {"middle"}))
    norm_middle = _median(ratios("norm", {"middle"}))
    raw_late = _median(ratios("raw", {"late"}))
    norm_late = _median(ratios("norm", {"late"}))
    raw_post = _median(ratios("raw", {"middle", "late"}))
    norm_post = _median(ratios("norm", {"middle", "late"}))

    raw_first = _finite_float(output_rows[0]["raw_best_ratio"]) if output_rows else math.nan
    norm_first = _finite_float(output_rows[0]["norm_best_ratio"]) if output_rows else math.nan
    agreement_fraction = (
        sum(_int_value(row["best_candidate_agreement"], 0) for row in output_rows) / n
        if n
        else math.nan
    )

    summary = {
        "run_id": run_dir.name,
        "environment": environment,
        "evaluable_decisions": n,
        "phase_basis": "ordered evaluable decisions",
        "candidate_agreement_fraction": agreement_fraction,
        "raw": {
            "first_ratio": raw_first,
            "early_median": raw_early,
            "middle_median": raw_middle,
            "late_median": raw_late,
            "post_early_median": raw_post,
            "late_to_middle_ratio": _safe_ratio(raw_late, raw_middle),
            "first_to_post_early_ratio": _safe_ratio(raw_first, raw_post),
        },
        "visible_normalized": {
            "first_ratio": norm_first,
            "early_median": norm_early,
            "middle_median": norm_middle,
            "late_median": norm_late,
            "post_early_median": norm_post,
            "late_to_middle_ratio": _safe_ratio(norm_late, norm_middle),
            "first_to_post_early_ratio": _safe_ratio(norm_first, norm_post),
        },
        "development_only": True,
    }
    return output_rows, summary


def _run_summary_row(summary: dict) -> dict[str, str]:
    raw = summary["raw"]
    norm = summary["visible_normalized"]
    return {
        "run_id": summary["run_id"],
        "environment": summary["environment"],
        "evaluable_decisions": str(summary["evaluable_decisions"]),
        "candidate_agreement_fraction": _fmt(summary["candidate_agreement_fraction"]),
        "raw_first_ratio": _fmt(raw["first_ratio"]),
        "norm_first_ratio": _fmt(norm["first_ratio"]),
        "raw_early_median": _fmt(raw["early_median"]),
        "norm_early_median": _fmt(norm["early_median"]),
        "raw_middle_median": _fmt(raw["middle_median"]),
        "norm_middle_median": _fmt(norm["middle_median"]),
        "raw_late_median": _fmt(raw["late_median"]),
        "norm_late_median": _fmt(norm["late_median"]),
        "raw_post_early_median": _fmt(raw["post_early_median"]),
        "norm_post_early_median": _fmt(norm["post_early_median"]),
        "raw_late_to_middle_ratio": _fmt(raw["late_to_middle_ratio"]),
        "norm_late_to_middle_ratio": _fmt(norm["late_to_middle_ratio"]),
        "raw_first_to_post_early_ratio": _fmt(raw["first_to_post_early_ratio"]),
        "norm_first_to_post_early_ratio": _fmt(norm["first_to_post_early_ratio"]),
    }


def _environment_summary(summaries: list[dict]) -> dict[str, dict]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for summary in summaries:
        grouped[summary["environment"]].append(summary)

    result: dict[str, dict] = {}
    for environment, items in sorted(grouped.items()):
        def med(path: tuple[str, str]) -> float:
            values = [item[path[0]][path[1]] for item in items]
            return _median(values)

        agreements = [item["candidate_agreement_fraction"] for item in items]
        result[environment] = {
            "run_count": len(items),
            "median_candidate_agreement_fraction": _median(agreements),
            "raw": {
                "median_first_ratio": med(("raw", "first_ratio")),
                "median_early_ratio": med(("raw", "early_median")),
                "median_middle_ratio": med(("raw", "middle_median")),
                "median_late_ratio": med(("raw", "late_median")),
                "median_post_early_ratio": med(("raw", "post_early_median")),
                "median_late_to_middle_ratio": med(("raw", "late_to_middle_ratio")),
                "median_first_to_post_early_ratio": med(("raw", "first_to_post_early_ratio")),
            },
            "visible_normalized": {
                "median_first_ratio": med(("visible_normalized", "first_ratio")),
                "median_early_ratio": med(("visible_normalized", "early_median")),
                "median_middle_ratio": med(("visible_normalized", "middle_median")),
                "median_late_ratio": med(("visible_normalized", "late_median")),
                "median_post_early_ratio": med(("visible_normalized", "post_early_median")),
                "median_late_to_middle_ratio": med(("visible_normalized", "late_to_middle_ratio")),
                "median_first_to_post_early_ratio": med(("visible_normalized", "first_to_post_early_ratio")),
            },
        }
    return result


def _cross_environment_scale(environment_summary: dict[str, dict]) -> dict:
    """Compare scale without deciding which definition is scientifically better."""
    result = {}
    for definition in ("raw", "visible_normalized"):
        for window in ("median_post_early_ratio", "median_late_ratio"):
            values = {
                env: metrics[definition][window]
                for env, metrics in environment_summary.items()
                if math.isfinite(float(metrics[definition][window]))
                and float(metrics[definition][window]) > 0.0
            }
            if len(values) < 2:
                comparison = {
                    "environment_values": values,
                    "max_to_min_ratio": None,
                    "relative_spread": None,
                }
            else:
                minimum = min(values.values())
                maximum = max(values.values())
                comparison = {
                    "environment_values": values,
                    "max_to_min_ratio": maximum / minimum,
                    "relative_spread": (maximum - minimum) / minimum,
                }
            result[f"{definition}.{window}"] = comparison
    return result


def _write_csv(path: Path, fieldnames: list[str], rows) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare raw and visible-normalized Way2 gain definitions offline."
    )
    parser.add_argument(
        "runs",
        nargs="+",
        help="Run IDs (e.g. mpx_001) or explicit MapEx run directories.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output directory. Default: mapex_lab/experiments/mapex.",
    )
    parser.add_argument(
        "--no-write",
        action="store_true",
        help="Print comparison without writing CSV/JSON outputs.",
    )
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    all_rows: list[dict[str, str]] = []
    summaries: list[dict] = []

    for value in args.runs:
        run_dir = _resolve_run(root, value)
        rows, summary = _analyze_run(run_dir)
        all_rows.extend(rows)
        summaries.append(summary)

        raw = summary["raw"]
        norm = summary["visible_normalized"]
        print(
            "WAY2 G COMPARE: "
            f"run={summary['run_id']}, env={summary['environment']}, "
            f"n={summary['evaluable_decisions']}, "
            f"agree={summary['candidate_agreement_fraction']:.3f}, "
            f"raw[first/post/late]={raw['first_ratio']:.6f}/"
            f"{raw['post_early_median']:.6f}/{raw['late_median']:.6f}, "
            f"norm[first/post/late]={norm['first_ratio']:.6f}/"
            f"{norm['post_early_median']:.6f}/{norm['late_median']:.6f}"
        )

    env_summary = _environment_summary(summaries)
    for environment, metrics in env_summary.items():
        raw = metrics["raw"]
        norm = metrics["visible_normalized"]
        print(
            "WAY2 G COMPARE ENV: "
            f"env={environment}, runs={metrics['run_count']}, "
            f"agree={metrics['median_candidate_agreement_fraction']:.3f}, "
            f"raw[first/post/late]={raw['median_first_ratio']:.6f}/"
            f"{raw['median_post_early_ratio']:.6f}/{raw['median_late_ratio']:.6f}, "
            f"raw_late/mid={raw['median_late_to_middle_ratio']:.6f}, "
            f"norm[first/post/late]={norm['median_first_ratio']:.6f}/"
            f"{norm['median_post_early_ratio']:.6f}/{norm['median_late_ratio']:.6f}, "
            f"norm_late/mid={norm['median_late_to_middle_ratio']:.6f}"
        )

    cross_scale = _cross_environment_scale(env_summary)
    for key, comparison in cross_scale.items():
        print(
            "WAY2 G CROSS-ENV: "
            f"metric={key}, values={comparison['environment_values']}, "
            f"max/min={comparison['max_to_min_ratio']}, "
            f"spread={comparison['relative_spread']}"
        )

    if not args.no_write:
        output_dir = (
            args.output_dir.resolve()
            if args.output_dir is not None
            else root / "experiments" / "mapex"
        )
        _write_csv(
            output_dir / "way2_gain_definition_decisions.csv",
            DECISION_FIELDS,
            all_rows,
        )
        _write_csv(
            output_dir / "way2_gain_definition_runs.csv",
            RUN_FIELDS,
            (_run_summary_row(summary) for summary in summaries),
        )
        payload = {
            "run_count": len(summaries),
            "phase_basis": "evaluable decisions: early <=25%, middle 25-75%, late >75%",
            "definitions": {
                "raw": "information_gain",
                "visible_normalized": "information_gain / visible_unknown_cells",
                "cost": "recorded Euclidean distance_m",
            },
            "runs": summaries,
            "environments": env_summary,
            "cross_environment_scale": cross_scale,
            "interpretation_guardrails": [
                "Visible-normalized gain is a diagnostic candidate, not a frozen design choice.",
                "Smaller cross-environment scale spread alone is not sufficient to prefer a gain definition.",
                "A viable stopping signal should also retain a clear decline toward late exploration.",
                "Dividing by visible cells removes the explicit benefit of observing a larger unknown region.",
                "All supplied historical runs remain development/diagnostic data, not independent validation.",
            ],
        }
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "way2_gain_definition_comparison.json").write_text(
            json.dumps(payload, indent=2),
            encoding="utf-8",
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
