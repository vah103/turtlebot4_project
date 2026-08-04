#!/usr/bin/env python3
"""Validate consistency between TurtleBot project metadata and human-readable status."""

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROJECT_PATH = ROOT / ".joy" / "project.json"
ROADMAP_PATH = ROOT / ".joy" / "roadmap.json"
STATUS_PATH = ROOT / "PROJECT_STATUS.md"


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def fail(message: str) -> None:
    raise AssertionError(message)


def calculate_progress(stages: list[dict]) -> int:
    progress = 0.0
    for stage in stages:
        stage_weight = float(stage.get("weight", 0))
        checklist = stage.get("checklist") or []
        total_checklist_weight = sum(float(item.get("weight", 0)) for item in checklist)
        completed_checklist_weight = sum(
            float(item.get("weight", 0)) for item in checklist if item.get("done") is True
        )
        if stage.get("status") == "completed":
            completion = 1.0
        elif total_checklist_weight > 0:
            completion = completed_checklist_weight / total_checklist_weight
        else:
            completion = 0.0
        progress += stage_weight * completion
    return round(progress)


def main() -> None:
    project = load_json(PROJECT_PATH)
    roadmap = load_json(ROADMAP_PATH)
    status_text = STATUS_PATH.read_text(encoding="utf-8")
    stages = roadmap.get("stages") or []

    if len(stages) != 9:
        fail(f"expected 9 roadmap stages, found {len(stages)}")

    stage_ids = [stage.get("id") for stage in stages]
    if len(stage_ids) != len(set(stage_ids)):
        fail("roadmap stage IDs must be unique")

    active = [stage for stage in stages if stage.get("status") in {"in-progress", "verification"}]
    if len(active) != 1:
        fail(f"expected exactly one active roadmap stage, found {len(active)}")

    active_stage = active[0]
    if project.get("currentStageId") != active_stage.get("id"):
        fail(
            "project currentStageId does not match roadmap active stage: "
            f"{project.get('currentStageId')} != {active_stage.get('id')}"
        )
    if project.get("currentStatus") != active_stage.get("status"):
        fail(
            "project currentStatus does not match roadmap active status: "
            f"{project.get('currentStatus')} != {active_stage.get('status')}"
        )
    if project.get("lastReviewed") != roadmap.get("updatedAt"):
        fail(
            "project lastReviewed does not match roadmap updatedAt: "
            f"{project.get('lastReviewed')} != {roadmap.get('updatedAt')}"
        )

    active_number = int(active_stage.get("number", 0))
    for stage in stages:
        number = int(stage.get("number", 0))
        checklist = stage.get("checklist") or []
        if number < active_number and stage.get("status") != "completed":
            fail(f"{stage.get('id')} precedes the active stage but is not completed")
        if stage.get("status") == "completed" and not all(item.get("done") is True for item in checklist):
            fail(f"{stage.get('id')} is completed but has unfinished checklist items")

    stage4 = next((stage for stage in stages if stage.get("id") == "stage-4"), None)
    if not stage4 or stage4.get("status") != "completed":
        fail("Stage 4 must remain completed after the verified simulation handoff")
    if not all(item.get("done") is True for item in stage4.get("checklist") or []):
        fail("Stage 4 checklist must remain fully completed")

    stage4_evidence = {
        evidence
        for result in stage4.get("results") or []
        for evidence in result.get("evidence") or []
    }
    required_report = "report/2026-08-03.md"
    if required_report not in stage4_evidence:
        fail(f"Stage 4 must reference {required_report}")
    if not (ROOT / required_report).is_file():
        fail(f"Stage 4 report is missing: {required_report}")

    calculated_progress = calculate_progress(stages)
    progress_matches = [int(value) for value in re.findall(r"\b(\d{1,3})%", status_text)]
    if calculated_progress not in progress_matches:
        fail(
            f"PROJECT_STATUS.md does not contain calculated roadmap progress {calculated_progress}%"
        )

    active_label = f"Stage {active_number}"
    if active_label not in status_text:
        fail(f"PROJECT_STATUS.md does not mention active {active_label}")

    print(
        "Project state validated: "
        f"{active_stage.get('id')} {active_stage.get('status')}, "
        f"progress {calculated_progress}%"
    )


if __name__ == "__main__":
    main()
