#!/usr/bin/env python3
"""Focused P1-P9 preflight for MX050 completion liveness recovery."""

from __future__ import annotations

import ast
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

import mx050_completion_liveness as live
import mapex_lama_bridge as bridge


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
        "run_slot": "mx049_fresh_dell_l49011_r1",
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

    def test_p1_startup_heartbeat_continuity_and_strict_w1_boundary(self):
        nf_source = Path("mapex_lab/scripts/nf_basic.py").read_text(encoding="utf-8")
        self.assertIn("from pathlib import Path as FilePath", nf_source)
        self.assertIn("FilePath(heartbeat_path)", nf_source)
        clock_value = [0.0]
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "heartbeat.json"
            writer = live.HeartbeatWriter(
                target,
                "mx049_fresh_dell_l49011_r1",
                "attempt02",
                clock=lambda: clock_value[0],
            )
            fields = heartbeat(0.0)
            for key in (
                "schema", "channel", "sequence", "explorer_pid", "run_slot",
                "attempt_id", "event", "emitted_monotonic_s",
            ):
                fields.pop(key)
            fields.update(state="INITIALIZING_LAMA", map_generation=0,
                          last_map_callback_age_s=None)
            first = writer.emit("lama_initializing", fields)
            last = first
            policy = live.LivenessPolicy(started_monotonic_s=0.0)
            self.assertTrue(policy.observe(first, 0.0, first["explorer_pid"]))
            for elapsed_s in range(5, 126, 5):
                clock_value[0] = float(elapsed_s)
                last = writer.emit("lama_loading_heartbeat", fields)
                self.assertTrue(
                    policy.observe(last, float(elapsed_s), last["explorer_pid"])
                )
                self.assertIsNone(policy.evaluate(last, float(elapsed_s)))
            clock_value[0] = 126.0
            ready = writer.emit("lama_ready", fields)
            fields.update(state="EXPLORING")
            control = writer.emit("explorer_control_ready", fields)
            ordinary = writer.emit("periodic_heartbeat", fields)
            self.assertEqual(
                [ready["sequence"], control["sequence"], ordinary["sequence"]],
                [last["sequence"] + 1, last["sequence"] + 2, last["sequence"] + 3],
            )
            self.assertEqual(json.loads(target.read_text()), ordinary)

        source = Path(bridge.__file__).read_text(encoding="utf-8")
        self.assertIn("while True:", source)
        self.assertIn("readiness_heartbeat()", source)
        self.assertNotIn("threading", source)
        self.assertNotIn("Thread(", source)

        policy = self.policy_with_observation(heartbeat(0.0), 0.0)
        self.assertIsNone(policy.evaluate(heartbeat(0.0), 30.0))
        self.assertEqual(policy.evaluate(heartbeat(0.0), 30.001)[0], live.W1)

    def test_p1_worker_exit_before_ready_fails_closed(self):
        class FakeStdout:
            pass

        class FakeProcess:
            stdout = FakeStdout()
            returncode = 17

            @staticmethod
            def poll():
                return 17

        class FakeSelector:
            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def register(self, *_args):
                return None

            def select(self, _timeout):
                return []

        instance = object.__new__(bridge.LamaEnsembleBridge)
        instance.process = FakeProcess()
        with mock.patch.object(bridge.selectors, "DefaultSelector", FakeSelector):
            result = instance._read_startup_response(lambda: None)
        self.assertEqual(result["status"], "fatal")
        self.assertIn("exited before ready", result["error"])

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

    def test_p8_historical_attempt_is_quarantined_and_49001_stays_excluded(self):
        attempt01 = Path(
            "mapex_lab/experiments/mapex/mx050_recovery_attempts/attempt01/"
            "mx049_fresh_dell_l49011_r1"
        )
        attempt02 = Path(
            "mapex_lab/experiments/mapex/mx050_recovery_attempts/attempt02/"
            "mx049_fresh_dell_l49011_r1"
        )
        self.assertNotEqual(attempt01, attempt02)
        runner_source = Path(".run_core").read_text(encoding="utf-8")
        self.assertIn(
            'mx050_recovery_attempts/$MX050_RECOVERY_ATTEMPT_ID_ARG/$RUN_ID',
            runner_source,
        )
        disposition = live.r2_reserve_disposition(
            "attempt01", "PRE_R2_ABORT_LAMA_STARTUP_HEARTBEAT_SCOPE_METHOD_GAP"
        )
        self.assertTrue(disposition["allow_49011_r1_attempt02"])
        self.assertFalse(disposition["activate_additional_reserve"])
        self.assertEqual(
            disposition["classification"], "AUDIT_ONLY_PRE_R2_METHOD_GAP"
        )
        runner_source = Path(".run_core").read_text(encoding="utf-8")
        self.assertIn('MX045_LAYOUT_SEED:-}" != "49011"', runner_source)
        self.assertIn('mx049_fresh_dell_l49011_r1', runner_source)

    def test_p9_exact_reserve_attempt02_terminal_transaction(self):
        failed = live.r2_reserve_disposition("attempt02", live.W2)
        self.assertEqual(failed["classification"], "INSUFFICIENT_NEW_DATA")
        self.assertFalse(failed["allow_third_49011_r1_attempt"])
        self.assertFalse(failed["activate_additional_reserve"])
        success = live.r2_reserve_disposition("attempt02", "ordinary_completion")
        self.assertTrue(success["scientific_slot_filled"])
        self.assertEqual(
            success["classification"], "PENDING_COLLECTION_INTEGRITY_QA"
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
