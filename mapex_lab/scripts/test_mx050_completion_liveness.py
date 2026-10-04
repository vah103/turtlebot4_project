#!/usr/bin/env python3
"""Focused P1-P9 preflight for MX050 completion liveness recovery."""

from __future__ import annotations

import ast
import copy
import json
from pathlib import Path
import tempfile
import unittest

import mx050_completion_liveness as live


PID = 4242


def heartbeat(
    emitted: float,
    sequence: int = 1,
    state: str = "EXPLORING",
    map_age: float | None = 1.0,
    active: bool = False,
    pending: bool = False,
    pending_age: float | None = None,
    completed: bool = False,
) -> dict:
    return {
        "schema": live.HEARTBEAT_SCHEMA,
        "channel": live.CHANNEL,
        "sequence": sequence,
        "explorer_pid": PID,
        "run_slot": "mx049_fresh_dell_l49001_r1",
        "attempt_id": "attempt02",
        "event": "fixture",
        "emitted_monotonic_s": emitted,
        "state": state,
        "completion_reason": None,
        "completion_streak": 0,
        "map_generation": 1,
        "last_map_callback_age_s": map_age,
        "revalidation_active": active,
        "revalidation_index": 0,
        "revalidation_candidate_count": 1 if active else 0,
        "planner_action_pending": pending,
        "planner_action_pending_age_s": pending_age,
        "completed": completed,
    }


class MX050RecoveryPreflight(unittest.TestCase):
    def policy_with_observation(self, payload: dict, now_s: float = 0.0):
        policy = live.LivenessPolicy(started_monotonic_s=0.0)
        self.assertTrue(policy.observe(payload, now_s, PID))
        return policy

    def test_p1_heartbeat_cadence_and_strict_w1_boundary(self):
        clock_value = [0.0]
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "heartbeat.json"
            writer = live.HeartbeatWriter(
                target,
                "mx049_fresh_dell_l49001_r1",
                "attempt02",
                clock=lambda: clock_value[0],
            )
            fields = heartbeat(0.0)
            for key in (
                "schema", "channel", "sequence", "explorer_pid", "run_slot",
                "attempt_id", "event", "emitted_monotonic_s",
            ):
                fields.pop(key)
            first = writer.emit("startup", fields)
            clock_value[0] = live.HEARTBEAT_PERIOD_S
            second = writer.emit("periodic_heartbeat", fields)
            self.assertEqual(second["sequence"], first["sequence"] + 1)
            self.assertEqual(second["emitted_monotonic_s"] - first["emitted_monotonic_s"], 5.0)
            self.assertEqual(json.loads(target.read_text()), second)

        policy = self.policy_with_observation(heartbeat(0.0), 0.0)
        self.assertIsNone(policy.evaluate(heartbeat(0.0), 30.0))
        self.assertEqual(policy.evaluate(heartbeat(0.0), 30.001)[0], live.W1)

    def test_p2_monotonic_deadline_source(self):
        source = Path(live.__file__).read_text(encoding="utf-8")
        self.assertIn("time.monotonic", source)
        self.assertNotIn("time.time(", source)
        policy = self.policy_with_observation(heartbeat(100.0), 100.0)
        self.assertIsNone(policy.evaluate(heartbeat(100.0), 130.0))

    def test_p3_single_pending_action_strict_w2_boundary(self):
        base = heartbeat(
            0.0, state="VERIFYING_COMPLETE", active=True, pending=True,
            pending_age=119.0,
        )
        policy = self.policy_with_observation(base)
        self.assertIsNone(policy.evaluate(base, 0.0))
        at_boundary = {**base, "planner_action_pending_age_s": 120.0}
        self.assertIsNone(policy.evaluate(at_boundary, 0.0))
        over = {**base, "planner_action_pending_age_s": 120.001}
        self.assertEqual(policy.evaluate(over, 0.0)[0], live.W2)
        self.assertFalse(over["completed"])

    def test_p4_map_stale_scope_and_strict_w3_boundary(self):
        policy = self.policy_with_observation(heartbeat(0.0))
        at_119 = heartbeat(0.0, state="VERIFYING_COMPLETE", map_age=119.0)
        self.assertIsNone(policy.evaluate(at_119, 0.0))
        at_120 = heartbeat(0.0, state="VERIFYING_COMPLETE", map_age=120.0)
        self.assertIsNone(policy.evaluate(at_120, 0.0))
        over = heartbeat(0.0, state="VERIFYING_COMPLETE", map_age=120.001)
        self.assertEqual(policy.evaluate(over, 0.0)[0], live.W3)
        outside = heartbeat(0.0, state="EXPLORING", map_age=1000.0)
        self.assertIsNone(policy.evaluate(outside, 0.0))

    def test_p5_ordinary_completion_is_observed_without_interference(self):
        payload = heartbeat(
            20.0, sequence=5, state="COMPLETE", map_age=1.0, completed=True
        )
        payload["completion_streak"] = 5
        before = copy.deepcopy(payload)
        policy = self.policy_with_observation(payload, 20.0)
        self.assertIsNone(policy.evaluate(payload, 20.0))
        self.assertEqual(payload, before)
        source = Path(live.__file__).read_text(encoding="utf-8")
        self.assertNotIn("mark_exploration_complete", source)

    def test_p6_long_aggregate_verification_is_not_a_deadline(self):
        payload = heartbeat(
            400.0,
            sequence=80,
            state="VERIFYING_COMPLETE",
            map_age=1.0,
            active=True,
            pending=True,
            pending_age=119.0,
        )
        policy = self.policy_with_observation(payload, 400.0)
        self.assertIsNone(policy.evaluate(payload, 400.0))

    def test_p7_trigger_has_no_scientific_payload_dependency(self):
        scientific_names = {
            "coverage", "known_area", "delta_known_area", "f1", "f2", "u3",
            "oracle", "ground_truth", "frontier_coordinates",
        }
        tree = ast.parse(Path(live.__file__).read_text(encoding="utf-8"))
        identifiers = {node.id.lower() for node in ast.walk(tree) if isinstance(node, ast.Name)}
        self.assertTrue(scientific_names.isdisjoint(identifiers))
        self.assertEqual(set(heartbeat(0.0)), live.HEARTBEAT_FIELDS)

    def test_p8_attempt_namespaces_are_disjoint_and_attempt01_is_non_reserve(self):
        attempt01 = Path("mapex_lab/experiments/mapex/mx049_fresh_dell_l49001_r1")
        attempt02 = Path(
            "mapex_lab/experiments/mapex/mx050_recovery_attempts/attempt02/"
            "mx049_fresh_dell_l49001_r1"
        )
        self.assertNotEqual(attempt01, attempt02)
        disposition = live.recovery_disposition(
            "attempt01", "PRE_AMENDMENT_ABORT_COMPLETION_LIVENESS_METHOD_GAP"
        )
        self.assertFalse(disposition["activate_reserve_49011"])
        self.assertEqual(disposition["classification"], "AUDIT_ONLY_NON_RESERVE")

    def test_p9_attempt02_w2_transaction_excludes_layout_and_allows_no_retry(self):
        disposition = live.recovery_disposition("attempt02", live.W2)
        self.assertEqual(
            disposition["classification"], "TECHNICALLY_INVALID_LAYOUT_EXCLUDED"
        )
        self.assertTrue(disposition["exclude_layout_49001"])
        self.assertTrue(disposition["activate_reserve_49011"])
        self.assertFalse(disposition["allow_third_49001_r1_retry"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
