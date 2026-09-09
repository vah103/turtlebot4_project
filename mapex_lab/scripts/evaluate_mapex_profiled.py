#!/usr/bin/env python3
"""Environment-aware wrapper around evaluate_mapex_run.py.

The base evaluator historically uses Hospital's ROI as its module default.
This wrapper lets mapex_run.py pass the active environment's structural GT and
connected-free ROI without duplicating the evaluator implementation.
"""
from __future__ import annotations

from pathlib import Path

import evaluate_mapex_run as base


ROS_FLOAT_ABS_TOL = 1e-6


def evaluate_run(
    run_dir: str | Path,
    ground_truth_path: str | Path,
    roi_path: str | Path,
    goal_count: int = base.TU_GOAL_COUNT,
    seed: int = base.TU_RANDOM_SEED,
) -> dict:
    """Evaluate one run with an explicit GT + ROI pair."""
    previous_roi = base.DEFAULT_ROI_RELATIVE
    previous_isclose = base.math.isclose

    def _ros_float_isclose(a, b, *, rel_tol=1e-9, abs_tol=0.0):
        """Accept normal float32 noise from ROS map metadata during evaluation."""
        return previous_isclose(
            a,
            b,
            rel_tol=rel_tol,
            abs_tol=max(abs_tol, ROS_FLOAT_ABS_TOL),
        )

    try:
        # Path joining with an absolute rhs preserves the absolute path, so the
        # base evaluator's ``mapex_lab / DEFAULT_ROI_RELATIVE`` remains valid.
        base.DEFAULT_ROI_RELATIVE = Path(roi_path).expanduser().resolve()

        # ROS OccupancyGrid commonly stores 0.1 m as 0.10000000149 (float32).
        # The base evaluator's 1e-9 absolute tolerance is too strict for this,
        # even though 0.1 m is exactly a 2x multiple of the 0.05 m canvas.
        # Relax only the evaluation-time floating-point comparison; all IoU/TU
        # formulas and canvas geometry remain unchanged.
        base.math.isclose = _ros_float_isclose

        return base.evaluate_run(
            run_dir,
            Path(ground_truth_path).expanduser().resolve(),
            goal_count,
            seed,
        )
    finally:
        base.math.isclose = previous_isclose
        base.DEFAULT_ROI_RELATIVE = previous_roi
