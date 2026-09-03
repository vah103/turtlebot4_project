#!/usr/bin/env python3
"""Environment-aware wrapper around evaluate_mapex_run.py.

The base evaluator historically uses Hospital's ROI as its module default.
This wrapper lets mapex_run.py pass the active environment's structural GT and
connected-free ROI without duplicating the evaluator implementation.
"""
from __future__ import annotations

from pathlib import Path

import evaluate_mapex_run as base


def evaluate_run(
    run_dir: str | Path,
    ground_truth_path: str | Path,
    roi_path: str | Path,
    goal_count: int = base.TU_GOAL_COUNT,
    seed: int = base.TU_RANDOM_SEED,
) -> dict:
    """Evaluate one run with an explicit GT + ROI pair."""
    previous_roi = base.DEFAULT_ROI_RELATIVE
    try:
        # Path joining with an absolute rhs preserves the absolute path, so the
        # base evaluator's ``mapex_lab / DEFAULT_ROI_RELATIVE`` remains valid.
        base.DEFAULT_ROI_RELATIVE = Path(roi_path).expanduser().resolve()
        return base.evaluate_run(
            run_dir,
            Path(ground_truth_path).expanduser().resolve(),
            goal_count,
            seed,
        )
    finally:
        base.DEFAULT_ROI_RELATIVE = previous_roi
