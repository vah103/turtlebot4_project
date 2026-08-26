#!/usr/bin/env python3
"""Aggregate lightweight run-level metrics from experiment folders.

This intentionally depends only on Python stdlib so it can be used early in the
project. More detailed stage/oracle analysis will be added after data logging is
finalized.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


def last_csv_row(path: Path) -> dict[str, str] | None:
    if not path.exists():
        return None
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    return rows[-1] if rows else None


def summarize_run(run_dir: Path) -> dict[str, object]:
    metadata_path = run_dir / "metadata.json"
    metadata = {}
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))

    last_metric = last_csv_row(run_dir / "metrics.csv") or {}
    last_traj = last_csv_row(run_dir / "trajectory.csv") or {}

    return {
        "method": metadata.get("method", run_dir.parent.name),
        "run_id": metadata.get("run_id", run_dir.name),
        "termination_reason": metadata.get("termination_reason"),
        "final_time_s": last_metric.get("time_s"),
        "final_distance_m": last_traj.get("cumulative_distance_m")
        or last_metric.get("distance_m"),
        "final_known_fraction": last_metric.get("known_fraction"),
        "final_coverage": last_metric.get("coverage"),
        "final_occupied_iou": last_metric.get("occupied_iou"),
        "final_tu": last_metric.get("tu"),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--experiments",
        default="mapex_hospital_research/experiments",
    )
    parser.add_argument(
        "--output",
        default="mapex_hospital_research/results/summary/runs.csv",
    )
    args = parser.parse_args()

    root = Path(args.experiments)
    rows = []
    if root.exists():
        for method_dir in sorted(p for p in root.iterdir() if p.is_dir()):
            for run_dir in sorted(p for p in method_dir.iterdir() if p.is_dir()):
                if (run_dir / "metadata.json").exists():
                    rows.append(summarize_run(run_dir))

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "method",
        "run_id",
        "termination_reason",
        "final_time_s",
        "final_distance_m",
        "final_known_fraction",
        "final_coverage",
        "final_occupied_iou",
        "final_tu",
    ]
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} runs to {output}")


if __name__ == "__main__":
    main()
