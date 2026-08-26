#!/usr/bin/env python3
"""Validate Hospital Nearest run integrity and experiment health.

A PASS now means more than files being readable: official runs must also prove
position-only frontier-goal semantics, successful planner action statuses,
periodic no-path revalidation, effective Nav2 configuration provenance, and no
obvious "goal succeeded without physical motion" pathology.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
CANVAS_SHAPE = (2123, 1504)
PLANNER_STATUS_SUCCEEDED = 4
EXPECTED_REVALIDATION_SWEEPS = 5
EXPECTED_REVALIDATION_PERIOD_S = 2.0


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def as_float(value: str, label: str, errors: list[str]) -> float | None:
    try:
        return float(value)
    except Exception:  # noqa: BLE001
        errors.append(f"invalid numeric value for {label}: {value!r}")
        return None


def as_count(value: str, label: str, errors: list[str]) -> int | None:
    try:
        parsed = int(float(value))
    except Exception:  # noqa: BLE001
        errors.append(f"invalid count value for {label}: {value!r}")
        return None
    if parsed < 0:
        errors.append(f"negative count for {label}: {parsed}")
        return None
    return parsed


def check_npz(path: Path, errors: list[str], *, canvas: bool = False) -> None:
    if not path.exists():
        errors.append(f"missing snapshot file: {path}")
        return
    try:
        with np.load(path, allow_pickle=False) as data:
            if "data" not in data:
                errors.append(f"{path}: missing data array")
                return
            arr = data["data"]
            if arr.ndim != 2:
                errors.append(f"{path}: data is not 2-D")
            if canvas and tuple(arr.shape) != CANVAS_SHAPE:
                errors.append(f"{path}: canvas shape {arr.shape} != {CANVAS_SHAPE}")
    except Exception as exc:  # noqa: BLE001
        errors.append(f"cannot read {path}: {exc}")


def nested(mapping: dict, *keys, default=None):
    cur = mapping
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def cumulative_distance_at(trajectory: list[dict[str, str]], t: float) -> float | None:
    best = None
    for row in trajectory:
        try:
            rt = float(row["time_s"])
            rd = float(row["cumulative_distance_m"])
        except Exception:
            continue
        if rt <= t + 1e-6:
            best = rd
        else:
            break
    return best


def candidate_signature(rows: list[dict[str, str]]) -> tuple:
    ordered = sorted(rows, key=lambda r: int(float(r.get("raw_rank", "0") or 0)))
    return tuple((r.get("row", ""), r.get("col", "")) for r in ordered)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-id", default="nearest_pilot_010")
    parser.add_argument("--allow-running", action="store_true")
    args = parser.parse_args()

    run_dir = WORKSPACE / "experiments" / "nearest" / args.run_id
    errors: list[str] = []
    warnings: list[str] = []
    health_issues: list[str] = []
    official = args.run_id.startswith("nearest_") and not args.run_id.startswith(
        "nearest_pilot_"
    )

    if not run_dir.is_dir():
        print(f"NEAREST RUN VALIDATION: FAIL\n- missing run directory: {run_dir}")
        return 1

    required = [
        "metadata.json",
        "metrics.csv",
        "trajectory.csv",
        "decisions.csv",
        "policy_decisions.csv",
        "candidates.csv",
        "snapshots.csv",
    ]
    for name in required:
        if not (run_dir / name).exists():
            errors.append(f"missing {name}")

    metadata: dict = {}
    metadata_path = run_dir / "metadata.json"
    if metadata_path.exists():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            errors.append(f"invalid metadata.json: {exc}")

    termination = metadata.get("termination_reason")
    if not termination and not args.allow_running:
        errors.append("run has no termination_reason; use --allow-running for live checks")

    if metadata.get("exploration_start_source") != "first_policy_decision_before_compute":
        errors.append("benchmark clock was not started before first policy computation")

    seed_policy = str(metadata.get("sim_seed_policy", ""))
    if "intentionally_uncontrolled_gazebo_default" not in seed_policy:
        errors.append("metadata missing explicit simulator seed policy")

    execution_goal_semantics = str(metadata.get("execution_goal_semantics", ""))
    accepted_semantics = {"exact_frontier_center_xy"}
    if official and execution_goal_semantics not in accepted_semantics:
        errors.append(
            "official run metadata must use exact_frontier_center_xy execution semantics"
        )
    elif execution_goal_semantics and execution_goal_semantics not in accepted_semantics:
        warnings.append(
            f"legacy execution_goal_semantics={execution_goal_semantics!r}"
        )

    yaw_semantics = str(metadata.get("goal_yaw_semantics", ""))
    if official and yaw_semantics != "ignored_by_hospital_goal_checker":
        errors.append("official run must use position-only frontier goals (yaw ignored)")
    elif yaw_semantics and yaw_semantics != "ignored_by_hospital_goal_checker":
        warnings.append(f"legacy goal_yaw_semantics={yaw_semantics!r}")

    if metadata.get("selected_path_length_semantics") not in {
        None,
        "planner_validation_path_length_not_executed_trajectory",
    }:
        warnings.append("unexpected selected_path_length_semantics metadata")

    dirty = metadata.get("git_dirty_at_recorder_start")
    if official and dirty is not False:
        errors.append("official run must start from a clean git worktree")
    elif dirty:
        warnings.append("pilot started from a dirty git worktree")

    hashes = metadata.get("config_sha256")
    required_hashes = {
        "nearest_official_policy",
        "nearest_hospital_adapted",
        "research_navigation_manager",
        "official_recorder",
        "nearest_config",
        "experiment_protocol",
        "data_schema",
        "installed_hospital_flat_stack",
        "installed_hospital_flat_simulation",
        "installed_tb4_simulation_safe",
        "installed_hospital_nav2_launch",
        "installed_frontier_config",
        "installed_hospital_slam",
        "installed_hospital_nav2_override",
        "installed_hospital_robot_urdf",
        "installed_create3_hospital_urdf",
        "installed_hospital_world",
        "installed_nav2_base_params",
        "runtime_nav2_merged",
    }
    if not isinstance(hashes, dict):
        errors.append("metadata missing config_sha256 provenance")
        hashes = {}
    else:
        for key in sorted(required_hashes):
            if not hashes.get(key):
                if official:
                    errors.append(f"metadata config_sha256 missing {key}")
                else:
                    warnings.append(f"older pilot provenance missing {key}")

    merged_name = str(metadata.get("runtime_nav2_merged_file", ""))
    merged_path = run_dir / merged_name if merged_name else None
    merged: dict = {}
    if not merged_name or merged_path is None or not merged_path.exists():
        if official:
            errors.append("official run missing archived runtime_nav2_merged.yaml")
        else:
            warnings.append("older pilot has no archived effective Nav2 YAML")
    else:
        expected_hash = str(hashes.get("runtime_nav2_merged", ""))
        if expected_hash and sha256_file(merged_path) != expected_hash:
            errors.append("runtime_nav2_merged.yaml SHA-256 mismatch")
        try:
            merged = yaml.safe_load(merged_path.read_text(encoding="utf-8")) or {}
        except Exception as exc:  # noqa: BLE001
            errors.append(f"cannot parse archived effective Nav2 YAML: {exc}")

    if merged:
        tolerance = nested(
            merged, "planner_server", "ros__parameters", "GridBased", "tolerance"
        )
        yaw_tol = nested(
            merged,
            "controller_server",
            "ros__parameters",
            "general_goal_checker",
            "yaw_goal_tolerance",
        )
        goal_angle_enabled = nested(
            merged,
            "controller_server",
            "ros__parameters",
            "FollowPath",
            "GoalAngleCritic",
            "enabled",
        )
        vx_max = nested(
            merged,
            "controller_server",
            "ros__parameters",
            "FollowPath",
            "vx_max",
        )
        if tolerance is None or abs(float(tolerance)) > 1e-9:
            errors.append(f"effective Nav2 GridBased tolerance is not 0.0: {tolerance!r}")
        if yaw_tol is None or float(yaw_tol) < math.pi - 0.01:
            errors.append(f"effective Nav2 still constrains frontier goal yaw: {yaw_tol!r}")
        if goal_angle_enabled is not False:
            errors.append("effective Nav2 GoalAngleCritic must be disabled for frontier goals")
        if vx_max is None or abs(float(vx_max) - 0.45) > 1e-6:
            errors.append(f"effective Hospital mapping vx_max is not 0.45: {vx_max!r}")

    metrics = read_csv(run_dir / "metrics.csv")
    trajectory = read_csv(run_dir / "trajectory.csv")
    decisions = read_csv(run_dir / "decisions.csv")
    policy_decisions = read_csv(run_dir / "policy_decisions.csv")
    candidates = read_csv(run_dir / "candidates.csv")
    snapshots = read_csv(run_dir / "snapshots.csv")

    if not metrics:
        errors.append("metrics.csv has no samples")
    prev_t = -math.inf
    prev_d = -math.inf
    for index, row in enumerate(metrics, start=1):
        t = as_float(row.get("time_s", ""), f"metrics row {index} time_s", errors)
        d = as_float(row.get("distance_m", ""), f"metrics row {index} distance_m", errors)
        c = as_float(row.get("coverage", ""), f"metrics row {index} coverage", errors)
        k = as_float(
            row.get("known_fraction", ""),
            f"metrics row {index} known_fraction",
            errors,
        )
        if t is not None:
            if t + 1e-9 < prev_t:
                errors.append(f"metrics time is not monotonic at row {index}")
            prev_t = t
        if d is not None:
            if d + 1e-6 < prev_d:
                errors.append(f"metrics cumulative distance decreased at row {index}")
            prev_d = d
        if c is not None and not 0.0 <= c <= 1.0:
            errors.append(f"coverage outside [0,1] at row {index}: {c}")
        if k is not None and not 0.0 <= k <= 1.0:
            errors.append(f"known_fraction outside [0,1] at row {index}: {k}")

    if not trajectory:
        errors.append("trajectory.csv has no samples")
    if not policy_decisions:
        errors.append("no policy decisions logged")

    policy_by_id = {
        row.get("policy_decision_id", ""): row
        for row in policy_decisions
        if row.get("policy_decision_id", "")
    }
    candidate_groups: dict[str, list[dict[str, str]]] = {}
    for row in candidates:
        pid = row.get("policy_decision_id", "")
        candidate_groups.setdefault(pid, []).append(row)
        if not row.get("candidate_id", ""):
            errors.append(f"{pid}: candidate row missing candidate_id")
        if row.get("status") == "rejected_lt_1m":
            errors.append(f"{pid}: below-1m candidate was incorrectly rejected")
        if str(row.get("selected", "")).lower() == "true":
            action_status = row.get("nav2_action_status", "")
            if action_status == "":
                if official:
                    errors.append(f"{pid}: selected candidate missing Nav2 action status")
                else:
                    warnings.append(f"{pid}: older pilot selected row has no action status")
            elif int(float(action_status)) != PLANNER_STATUS_SUCCEEDED:
                errors.append(
                    f"{pid}: selected candidate action status={action_status}, expected 4"
                )

    for pid, row in policy_by_id.items():
        decision_dir = run_dir / "decisions" / pid
        required_decision_files = [
            decision_dir / "decision.json",
            decision_dir / "candidates.csv",
            decision_dir / "observed_map_raw.npz",
            decision_dir / "observed_map_canvas.npz",
        ]
        for path in required_decision_files:
            if not path.exists():
                errors.append(f"{pid}: missing {path.name}")
        check_npz(decision_dir / "observed_map_raw.npz", errors)
        check_npz(decision_dir / "observed_map_canvas.npz", errors, canvas=True)
        count = int(float(row.get("candidate_count", "0") or 0))
        if count > 0 and not candidate_groups.get(pid):
            errors.append(f"{pid}: candidate_count={count} but no candidate rows")

    exact_goal_execution_ok = bool(decisions)
    legacy_goal_semantics_seen = False
    for row in decisions:
        did = row.get("decision_id", "")
        pid = row.get("policy_decision_id", "")
        cid = row.get("selected_candidate_id", "")
        if not pid or pid not in policy_by_id:
            errors.append(f"{did}: missing/unknown policy_decision_id")
        if not cid:
            errors.append(f"{did}: missing selected_candidate_id")
        elif pid:
            matches = [
                item
                for item in candidate_groups.get(pid, [])
                if item.get("candidate_id") == cid
            ]
            if not matches:
                errors.append(f"{did}: selected candidate {pid}/{cid} not found")
            elif not any(str(item.get("selected", "")).lower() == "true" for item in matches):
                errors.append(f"{did}: linked candidate is not marked selected")
        if row.get("coverage", "") == "":
            errors.append(f"{did}: missing coverage")
        if row.get("robot_map_x", "") == "" or row.get("robot_map_y", "") == "":
            errors.append(f"{did}: missing map-frame robot pose")
        if termination and row.get("result") == "PENDING":
            errors.append(f"{did}: terminated run still has PENDING result")
        if row.get("result") == "FAILED" and not row.get("failure_reason", ""):
            errors.append(f"{did}: FAILED goal missing failure_reason")

        exact_fields = ["frontier_x", "frontier_y", "goal_x", "goal_y"]
        if any(row.get(field, "") == "" for field in exact_fields):
            exact_goal_execution_ok = False
            if official:
                errors.append(f"{did}: missing exact frontier execution coordinates")
            else:
                legacy_goal_semantics_seen = True
        else:
            fx = as_float(row["frontier_x"], f"{did} frontier_x", errors)
            fy = as_float(row["frontier_y"], f"{did} frontier_y", errors)
            gx = as_float(row["goal_x"], f"{did} goal_x", errors)
            gy = as_float(row["goal_y"], f"{did} goal_y", errors)
            if None not in {fx, fy, gx, gy}:
                mismatch = math.hypot(gx - fx, gy - fy)
                if mismatch > 1e-3:
                    exact_goal_execution_ok = False
                    message = (
                        f"{did}: execution goal differs from exact frontier by "
                        f"{mismatch:.4f} m"
                    )
                    if official:
                        errors.append(message)
                    else:
                        legacy_goal_semantics_seen = True

        if row.get("goal_source", "") != "exact_frontier_center":
            exact_goal_execution_ok = False
            if official:
                errors.append(
                    f"{did}: goal_source={row.get('goal_source', '')!r} != "
                    "'exact_frontier_center'"
                )
            else:
                legacy_goal_semantics_seen = True
        if row.get("goal_yaw_semantics", "") != "ignored_by_hospital_goal_checker":
            if official:
                errors.append(f"{did}: frontier goal yaw is not explicitly ignored")
            else:
                legacy_goal_semantics_seen = True

        px_text = row.get("planner_endpoint_x", "")
        py_text = row.get("planner_endpoint_y", "")
        offset_text = row.get("planner_endpoint_to_frontier_m", "")
        if official and (px_text == "" or py_text == "" or offset_text == ""):
            errors.append(f"{did}: missing planner-endpoint audit fields")
        elif px_text != "" and py_text != "" and offset_text != "":
            px = as_float(px_text, f"{did} planner_endpoint_x", errors)
            py = as_float(py_text, f"{did} planner_endpoint_y", errors)
            fx = as_float(row.get("frontier_x", ""), f"{did} frontier_x", errors)
            fy = as_float(row.get("frontier_y", ""), f"{did} frontier_y", errors)
            reported = as_float(offset_text, f"{did} planner_endpoint_to_frontier_m", errors)
            if None not in {px, py, fx, fy, reported}:
                recomputed = math.hypot(px - fx, py - fy)
                if abs(recomputed - reported) > 2e-3:
                    errors.append(
                        f"{did}: planner endpoint offset audit mismatch: "
                        f"reported={reported:.4f}, recomputed={recomputed:.4f}"
                    )

    if legacy_goal_semantics_seen and not official:
        warnings.append("pilot contains legacy execution-goal semantics")

    # Health check for the exact pathology seen in pilot_007: a goal much farther
    # away than the robot physically travelled is reported SUCCEEDED.
    for index, row in enumerate(decisions[:-1]):
        if row.get("result") != "SUCCEEDED":
            continue
        try:
            selected_distance = float(row.get("selected_distance_m", ""))
            t0 = float(row.get("time_s", ""))
            t1 = float(decisions[index + 1].get("time_s", ""))
        except Exception:
            continue
        if selected_distance < 0.35:
            continue
        d0 = cumulative_distance_at(trajectory, t0)
        d1 = cumulative_distance_at(trajectory, t1)
        if d0 is None or d1 is None:
            continue
        physical_motion = max(0.0, d1 - d0)
        if physical_motion < 0.10:
            health_issues.append(
                f"{row.get('decision_id')}: SUCCEEDED at selected_distance="
                f"{selected_distance:.3f}m but odom motion before next decision was "
                f"only {physical_motion:.3f}m"
            )

    if not snapshots:
        errors.append("snapshots.csv is empty")
    for row in snapshots:
        raw_rel = row.get("raw_map_file", "")
        canvas_rel = row.get("canvas_map_file", "")
        if not raw_rel or not canvas_rel:
            errors.append(f"snapshot {row.get('snapshot_id')}: missing file reference")
            continue
        check_npz(run_dir / raw_rel, errors)
        check_npz(run_dir / canvas_rel, errors, canvas=True)
    if termination and not any(row.get("event") == "final" for row in snapshots):
        errors.append("terminated run has no final recorder snapshot")

    terminal_audit_ok = False
    revalidation_audit_ok = False
    if termination and policy_decisions:
        terminal_outcomes = {
            "exhausted_no_ranked_candidate",
            "no_nav2_reachable_ranked_candidate",
        }
        terminal_rows = [
            row for row in policy_decisions if row.get("outcome") in terminal_outcomes
        ]
        if not terminal_rows:
            message = "terminated run has no explicit exhausted/no-Nav2-reachable policy state"
            if official:
                errors.append(message)
            else:
                warnings.append(message + "; older/interrupted pilot")
        else:
            terminal = terminal_rows[-1]
            pid = terminal.get("policy_decision_id", "")
            outcome = terminal.get("outcome", "")
            if outcome == "exhausted_no_ranked_candidate":
                reason = terminal.get("terminal_reason", "")
                if reason and reason != "zero_ranked_candidates":
                    errors.append(f"{pid}: unexpected terminal_reason={reason!r}")
                terminal_audit_ok = True
                revalidation_audit_ok = True
            else:
                required_terminal_fields = [
                    "nav2_checked_count",
                    "nav2_path_success_count",
                    "nav2_no_path_count",
                    "nav2_rejected_count",
                    "nav2_error_count",
                    "terminal_reason",
                    "planner_revalidation_period_s",
                ]
                missing = [
                    field for field in required_terminal_fields if terminal.get(field, "") == ""
                ]
                if missing:
                    message = f"{pid}: terminal Nav2 audit missing {', '.join(missing)}"
                    if official:
                        errors.append(message)
                    else:
                        warnings.append(message + " (older pilot schema)")
                else:
                    candidate_count = as_count(
                        terminal.get("candidate_count", ""), f"{pid} candidate_count", errors
                    )
                    checked = as_count(
                        terminal.get("nav2_checked_count", ""), f"{pid} nav2_checked_count", errors
                    )
                    success = as_count(
                        terminal.get("nav2_path_success_count", ""),
                        f"{pid} nav2_path_success_count",
                        errors,
                    )
                    no_path = as_count(
                        terminal.get("nav2_no_path_count", ""), f"{pid} nav2_no_path_count", errors
                    )
                    rejected = as_count(
                        terminal.get("nav2_rejected_count", ""),
                        f"{pid} nav2_rejected_count",
                        errors,
                    )
                    nav_errors = as_count(
                        terminal.get("nav2_error_count", ""), f"{pid} nav2_error_count", errors
                    )
                    period = as_float(
                        terminal.get("planner_revalidation_period_s", ""),
                        f"{pid} planner_revalidation_period_s",
                        errors,
                    )
                    if period is not None and abs(period - EXPECTED_REVALIDATION_PERIOD_S) > 1e-6:
                        errors.append(
                            f"{pid}: planner_revalidation_period_s={period} != "
                            f"{EXPECTED_REVALIDATION_PERIOD_S}"
                        )
                    if None not in {
                        candidate_count,
                        checked,
                        success,
                        no_path,
                        rejected,
                        nav_errors,
                    }:
                        expected_checked = success + no_path + rejected + nav_errors
                        if checked != expected_checked:
                            errors.append(
                                f"{pid}: nav2_checked_count={checked} != outcome sum={expected_checked}"
                            )
                        if candidate_count != checked:
                            errors.append(
                                f"{pid}: terminal candidate_count={candidate_count} != "
                                f"nav2_checked_count={checked}"
                            )
                        if success != 0:
                            errors.append(
                                f"{pid}: no_nav2_reachable terminal state has "
                                f"nav2_path_success_count={success}"
                            )
                    if terminal.get("terminal_reason") != (
                        "all_ranked_candidates_failed_nav2_path_validation"
                    ):
                        errors.append(
                            f"{pid}: unexpected terminal_reason="
                            f"{terminal.get('terminal_reason')!r}"
                        )
                    terminal_audit_ok = True

                if len(policy_decisions) >= EXPECTED_REVALIDATION_SWEEPS:
                    final_sweeps = policy_decisions[-EXPECTED_REVALIDATION_SWEEPS:]
                    expected_outcome = "no_nav2_reachable_ranked_candidate"
                    if not all(
                        row.get("outcome") == expected_outcome for row in final_sweeps
                    ):
                        errors.append(
                            "the final policy decisions are not all no-path planner "
                            "revalidation sweeps"
                        )
                    else:
                        signatures = [
                            candidate_signature(
                                candidate_groups.get(row["policy_decision_id"], [])
                            )
                            for row in final_sweeps
                        ]
                        if not all(sig == signatures[0] for sig in signatures[1:]):
                            errors.append(
                                "completion window did not use a stable candidate set "
                                f"across the final {EXPECTED_REVALIDATION_SWEEPS} sweeps"
                            )
                        periods_ok = True
                        for row in final_sweeps:
                            try:
                                sweep_period = float(row["planner_revalidation_period_s"])
                            except Exception:
                                periods_ok = False
                                errors.append(
                                    f"{row.get('policy_decision_id')}: missing/invalid "
                                    "planner_revalidation_period_s"
                                )
                                continue
                            if abs(sweep_period - EXPECTED_REVALIDATION_PERIOD_S) > 1e-6:
                                periods_ok = False
                                errors.append(
                                    f"{row.get('policy_decision_id')}: planner "
                                    f"revalidation period {sweep_period} != "
                                    f"{EXPECTED_REVALIDATION_PERIOD_S}"
                                )
                        if (
                            all(sig == signatures[0] for sig in signatures[1:])
                            and periods_ok
                        ):
                            revalidation_audit_ok = True
                else:
                    message = (
                        f"only {len(policy_decisions)} policy decisions logged; expected "
                        f"at least {EXPECTED_REVALIDATION_SWEEPS} final revalidation sweeps"
                    )
                    if official:
                        errors.append(message)
                    else:
                        warnings.append(message)

    if health_issues:
        if official:
            errors.extend(f"experiment health: {item}" for item in health_issues)
        else:
            warnings.extend(f"experiment health: {item}" for item in health_issues)

    if errors:
        print("NEAREST RUN VALIDATION: FAIL")
        print("DATA / PROTOCOL / HEALTH CHECKS:")
        for error in errors:
            print(f"- {error}")
        if warnings:
            print("Warnings:")
            for warning in warnings:
                print(f"- {warning}")
        return 1

    print("NEAREST RUN VALIDATION: PASS")
    print(f"- metrics samples: {len(metrics)}")
    print(f"- selected decisions: {len(decisions)}")
    print(f"- policy decisions: {len(policy_decisions)}")
    print(f"- candidate rows: {len(candidates)}")
    print(f"- periodic/final snapshots: {len(snapshots)}")
    print(f"- termination: {termination or 'RUNNING'}")
    print("- benchmark t=0: pre-compute")
    print("- Hospital below-1m adaptation: consistent")
    print("- planner action status: audited")
    print("- frontier goal yaw: ignored")
    print("- candidate linkage + snapshot integrity: OK")
    if exact_goal_execution_ok:
        print("- exact frontier x/y execution goal: OK")
    if terminal_audit_ok:
        print("- terminal Nav2 audit: OK")
    if revalidation_audit_ok:
        print("- terminal planner revalidation: OK")
    print("- effective Nav2 config provenance: OK")
    print("- experiment health: no success-without-motion pathology detected")
    if warnings:
        print("Warnings:")
        for warning in warnings:
            print(f"- {warning}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
