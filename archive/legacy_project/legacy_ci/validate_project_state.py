#!/usr/bin/env python3
"""Validate consistency between TurtleBot project metadata and documentation."""

from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PROJECT_PATH = ROOT / ".joy" / "project.json"
ROADMAP_PATH = ROOT / ".joy" / "roadmap.json"
STATUS_PATH = ROOT / "PROJECT_STATUS.md"
README_PATH = ROOT / "README.md"
STAGE4_REPORT = ROOT / "report" / "2026-08-03.md"
STAGE4_EVIDENCE_FILES = (
    ROOT / "evidence" / "stage4_2026_08_03" / "README.md",
    ROOT / "evidence" / "stage4_2026_08_03" / "scenario_matrix.md",
    ROOT / "evidence" / "stage4_2026_08_03" / "verification_commands.md",
)


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


def validate_stage_order(stages: list[dict], active_stage: dict) -> None:
    active_number = int(active_stage.get("number", 0))
    for stage in stages:
        number = int(stage.get("number", 0))
        checklist = stage.get("checklist") or []
        if number < active_number and stage.get("status") != "completed":
            fail(f"{stage.get('id')} precedes the active stage but is not completed")
        if stage.get("status") == "completed" and not all(
            item.get("done") is True for item in checklist
        ):
            fail(f"{stage.get('id')} is completed but has unfinished checklist items")


def validate_stage4(stages: list[dict]) -> None:
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
    if not STAGE4_REPORT.is_file():
        fail(f"Stage 4 report is missing: {required_report}")

    for path in STAGE4_EVIDENCE_FILES:
        if not path.is_file():
            fail(f"Stage 4 evidence documentation is missing: {path.relative_to(ROOT)}")


def validate_human_docs(
    status_text: str,
    readme_text: str,
    active_stage: dict,
    calculated_progress: int,
) -> None:
    active_number = int(active_stage.get("number", 0))
    active_label = f"Stage {active_number}"

    progress_matches = [int(value) for value in re.findall(r"\b(\d{1,3})%", status_text)]
    if calculated_progress not in progress_matches:
        fail(
            f"PROJECT_STATUS.md does not contain calculated roadmap progress {calculated_progress}%"
        )
    if active_label not in status_text:
        fail(f"PROJECT_STATUS.md does not mention active {active_label}")

    if active_label not in readme_text:
        fail(f"README.md does not mention active {active_label}")
    if f"{calculated_progress}%" not in readme_text:
        fail(f"README.md does not contain calculated roadmap progress {calculated_progress}%")

    stale_readme_fragments = (
        "Stage 4 — Simulation & Scenarios: chưa bắt đầu",
        "Tiến độ kỹ thuật theo checklist có trọng số: 32%",
        "Stage 4 được thực hiện chủ yếu tại nhà",
    )
    for fragment in stale_readme_fragments:
        if fragment in readme_text:
            fail(f"README.md still contains stale project state: {fragment}")

    required_readme_links = (
        "report/2026-08-03.md",
        "evidence/stage4_2026_08_03/README.md",
        "evidence/stage4_2026_08_03/scenario_matrix.md",
        "evidence/stage4_2026_08_03/verification_commands.md",
    )
    for reference in required_readme_links:
        if reference not in readme_text:
            fail(f"README.md is missing required Stage 4 reference: {reference}")


def main() -> None:
    project = load_json(PROJECT_PATH)
    roadmap = load_json(ROADMAP_PATH)
    status_text = STATUS_PATH.read_text(encoding="utf-8")
    readme_text = README_PATH.read_text(encoding="utf-8")
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

    validate_stage_order(stages, active_stage)
    validate_stage4(stages)

    calculated_progress = calculate_progress(stages)
    validate_human_docs(status_text, readme_text, active_stage, calculated_progress)

    print(
        "Project state validated: "
        f"{active_stage.get('id')} {active_stage.get('status')}, "
        f"progress {calculated_progress}%"
    )


if __name__ == "__main__":
    main()
