from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import mx049_generator as mx048
from mx060_acquisition import content_neutral_inventory, validate_bundle
from mx060_full_preflight import ARTIFACT_NAMES, build_denylist, technical_fixtures
from mx060_contract import (
    LAYOUT_SEEDS, PARAMETER_ORDER, acquisition_only_allows, derive_seed32,
    metadata_only, pairwise_diversity, retry_disposition, run_id,
    select_active_subset, validate_decision_sequence,
)


def test_seed_domain_and_exact_derivation():
    assert derive_seed32(60001, 1, "gazebo") == 2040208160
    assert derive_seed32(60001, 1, "gazebo") != derive_seed32(60001, 2, "gazebo")
    assert run_id(60004, 2) == "mx060_com1_l60004_r2"
    with pytest.raises(ValueError, match="MX060_SEED_DOMAIN_VIOLATION"):
        derive_seed32(60005, 1, "gazebo")


def test_mx048_draw_is_deterministic_and_first_attempt_is_not_skipped():
    for seed in LAYOUT_SEEDS:
        assert mx048.draw_candidate(seed, 0) == mx048.draw_candidate(seed, 0)
        assert mx048.candidate_key(seed, 0) != mx048.candidate_key(seed, 1)


def test_pairwise_gate_and_subset_order():
    base = {key: 0 for key in PARAMETER_ORDER}
    distinct = {key: index + 1 for index, key in enumerate(PARAMETER_ORDER)}
    result = pairwise_diversity(base, distinct)
    assert result["status"] == "PASS"
    layouts = {}
    for seed in LAYOUT_SEEDS:
        layouts[seed] = {"admissible": True, "pairwise": {}}
    for a in LAYOUT_SEEDS:
        for b in LAYOUT_SEEDS:
            if a < b:
                layouts[a]["pairwise"][str(b)] = result
    assert select_active_subset(layouts) == (60001, 60002, 60003)


def test_acquisition_only_and_seal_fail_closed():
    assert acquisition_only_allows("ordinary_exploration")
    assert not acquisition_only_allows("scientific_evaluator")
    assert not acquisition_only_allows("online_stop")
    metadata_only({"path": "x", "sha256": "0" * 64, "record_count": 2})
    with pytest.raises(RuntimeError, match="MX060_SEALED_NUMERIC_PAYLOAD"):
        metadata_only({"mean": 0.5})


def test_retry_budget_and_no_scientific_retry():
    assert retry_disposition(1, 0, True, False) == "ALLOW_EXACT_ATTEMPT02"
    assert retry_disposition(1, 2, True, False) == "EXCLUDE_LAYOUT_OR_TECHNICAL_INSUFFICIENT"
    assert retry_disposition(2, 1, True, False) == "EXCLUDE_WHOLE_LAYOUT"
    assert retry_disposition(1, 0, False, False) == "RETAIN_NO_RETRY"
    assert retry_disposition(1, 0, True, True) == "BLOCK_RECOVERY_REVIEW"


def test_exact_next_clock_and_odometry_contract():
    rows = [
        {"ordinal": 0, "decision_id": "d0", "predecessor_decision_id": "RUN_START", "successor_decision_id": "d1", "decision_exploration_time_s": 1.0, "cumulative_path_m": 0.0},
        {"ordinal": 1, "decision_id": "d1", "predecessor_decision_id": "d0", "successor_decision_id": "TERMINAL", "decision_exploration_time_s": 2.0, "cumulative_path_m": 0.4},
    ]
    validate_decision_sequence(rows)
    bad = copy.deepcopy(rows); bad[1]["decision_exploration_time_s"] = 1.0
    with pytest.raises(RuntimeError, match="MX060_SCIENTIFIC_CLOCK_INVALID"):
        validate_decision_sequence(bad)


def test_bundle_hash_clock_and_future_leakage(tmp_path):
    maps = {}; predictions = {}
    import hashlib
    for group, names in ((maps, ("observed_raw", "observed_fixed_canvas")), (predictions, ("G1", "G2", "G3", "ensemble_mean", "ensemble_variance"))):
        for name in names:
            path = tmp_path / name; path.write_bytes(name.encode())
            group[name] = {"path": name, "sha256": hashlib.sha256(name.encode()).hexdigest()}
    bundle = {
        "decision_id": "d0", "ordinal": 0, "predecessor_decision_id": "RUN_START",
        "successor_decision_id": "TERMINAL", "decision_exploration_time_s": 1.0,
        "clock_source": "ROS_/clock", "recorder_use_sim_time": True,
        "cumulative_path_m": 0.0, "odometry_topic": "/odom", "odometry_frame": "odom",
        "maps": maps, "predictions": predictions, "r_b1_source": {}, "f1_source": {},
        "f2_source": {}, "f4_source": {}, "runtime_current_only": {},
        "contains_evaluator_only_values": False,
    }
    validate_bundle(bundle, tmp_path)
    inventory = content_neutral_inventory([tmp_path / "G1"], tmp_path)
    assert inventory["files"][0]["bytes"] == 2


def test_full_preflight_fixture_artifacts_pass(tmp_path):
    results = technical_fixtures(tmp_path)
    assert all(row["status"] == "PASS" for row in results.values())
    assert len(ARTIFACT_NAMES) == 16


def test_prior_geometry_denylist_exact_sources(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    doc = build_denylist(repo, tmp_path / "denylist.json")
    assert doc["generated_geometry_cardinality"] == 24
    assert len(doc["generated_geometry_entries"]) == 24
    assert doc["source_hashes"]["MX049_OLD_MX046_GEOMETRY_DENYLIST.json"] == "e2c6989d94f703a40f227a7c3146f0612df337c7743491d7050b1f332707b8ec"
