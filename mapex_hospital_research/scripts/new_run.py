#!/usr/bin/env python3
"""Create a standardized experiment run directory.

Usage:
  python3 mapex_hospital_research/scripts/new_run.py --method nearest --run-id nearest_001
  python3 mapex_hospital_research/scripts/new_run.py --method mapex --run-id mapex_001
"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=("nearest", "mapex"), required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument(
        "--root",
        default="mapex_hospital_research/experiments",
        help="Experiment root relative to repository root.",
    )
    args = parser.parse_args()

    run_dir = Path(args.root) / args.method / args.run_id
    if run_dir.exists():
        raise SystemExit(f"Run already exists: {run_dir}")

    for name in (
        "maps",
        "predictions",
        "variance",
        "visibility",
        "decisions",
        "logs",
    ):
        (run_dir / name).mkdir(parents=True, exist_ok=True)

    metadata = {
        "run_id": args.run_id,
        "method": args.method,
        "start_time": datetime.now(timezone.utc).isoformat(),
        "world": "hospital",
        "git_commit": None,
        "spawn": {"x": None, "y": None, "yaw": None},
        "termination_reason": None,
        "notes": "",
    }
    (run_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )

    (run_dir / "metrics.csv").write_text(
        "time_s,distance_m,known_fraction,coverage,occupied_iou,tu\n",
        encoding="utf-8",
    )
    (run_dir / "trajectory.csv").write_text(
        "time_s,x,y,yaw,cumulative_distance_m\n", encoding="utf-8"
    )

    decision_header = (
        "decision_id,time_s,known_fraction,num_candidates,selected_candidate_id,"
        "selected_distance_m,selected_ig,selected_score,selected_rank,result\n"
    )
    (run_dir / "decisions.csv").write_text(decision_header, encoding="utf-8")

    print(run_dir)


if __name__ == "__main__":
    main()
