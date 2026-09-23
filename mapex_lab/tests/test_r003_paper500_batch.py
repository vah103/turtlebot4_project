"""ROS-free H031 batch/watchdog regressions."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import socket
import tempfile
import unittest
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def load(name: str):
    spec = importlib.util.spec_from_file_location(name, SCRIPTS / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


batch = load("run_r003_paper500_batch")
watchdog = load("r003_deadlock_watchdog")


class Paper500BatchTests(unittest.TestCase):
    def test_exact_official_matrix(self):
        matrix = batch.official_matrix()
        self.assertEqual(len(matrix), 20)
        self.assertEqual(matrix[0]["run_id"], "nf_p500_001")
        self.assertEqual(matrix[9]["run_id"], "nf_p500_010")
        self.assertEqual(matrix[10]["run_id"], "mpx_p500_001")
        self.assertEqual(matrix[-1]["run_id"], "mpx_p500_010")
        self.assertEqual(len({item["run_id"] for item in matrix}), 20)

    def test_resume_selects_first_unfinished(self):
        matrix = batch.official_matrix()
        state = {"runs": {matrix[0]["run_id"]: {"status": "complete"}}}
        self.assertEqual(batch.first_unfinished(matrix, state), matrix[1])

    def test_atomic_state_replaces_temporary(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "state.json"
            batch.atomic_json(target, {"value": 1})
            self.assertEqual(target.read_text(), '{\n  "value": 1\n}\n')
            self.assertFalse(target.with_suffix(".json.tmp").exists())

    def test_watchdog_requires_active_action_and_odom(self):
        detector = watchdog.DeadlockDetector(window_s=10, startup_grace_s=0)
        detector.set_action_active(True, 0)
        self.assertIsNone(detector.check(20))
        detector.observe_odom(0, 0, 20)
        self.assertIsNone(detector.check(29.9))
        self.assertIsNotNone(detector.check(30))
        detector.set_action_active(False, 31)
        self.assertIsNone(detector.check(100))

    def test_runner_and_watchdog_share_effective_ros_isolation(self):
        env = batch.effective_ros_env({
            "MAPEX_SIM_ROS_DOMAIN_ID": "77",
            "MAPEX_SIM_LOCALHOST_ONLY": "0",
        })
        self.assertEqual(env["ROS_DOMAIN_ID"], "77")
        self.assertEqual(env["ROS_LOCALHOST_ONLY"], "0")
        source = (SCRIPTS / "run_r003_paper500_batch.py").read_text(encoding="utf-8")
        self.assertEqual(source.count("env=ros_env"), 2)

    def test_terminal_and_interrupted_states_do_not_resume(self):
        self.assertEqual(
            batch.resume_blocker({"status": "blocked_repeated_technical_invalidity"}),
            "blocked_repeated_technical_invalidity",
        )
        self.assertEqual(
            batch.resume_blocker({"status": "blocked_ambiguous_failure"}),
            "blocked_ambiguous_failure",
        )
        self.assertEqual(
            batch.resume_blocker({"status": "running"}), "recovery_required"
        )
        self.assertEqual(
            batch.resume_blocker({"status": "unexpected"}), "recovery_required"
        )
        self.assertIsNone(batch.resume_blocker({"status": "retry_pending_same_id"}))

    def test_invalidity_checkpoint_precedes_delete_and_binds_provenance(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "mapex_lab/experiments/nearest/nf_p500_001"
            target.mkdir(parents=True)
            evidence_dir = root / "results"
            watchdog_path = evidence_dir / "invalid_attempts/watchdog.json"
            watchdog_path.parent.mkdir(parents=True)
            watchdog_path.write_text('{"deadlock": true}\n', encoding="utf-8")
            log = evidence_dir / "logs/run.log"
            log.parent.mkdir(parents=True)
            log.write_text("log\n", encoding="utf-8")
            state_path = evidence_dir / "batch_state.json"
            record = {"attempts": 1}
            state = {
                "execution_host": socket.gethostname(),
                "source_sha": "a" * 40,
                "runs": {"nf_p500_001": record},
            }
            item = {"method": "nf", "run_id": "nf_p500_001"}

            def assert_checkpoint_before_delete(path, _root, _run_id):
                checkpoint = evidence_dir / "invalid_attempts/nf_p500_001_attempt01_checkpoint.json"
                self.assertTrue(checkpoint.is_file())
                persisted = json.loads(state_path.read_text(encoding="utf-8"))
                self.assertFalse(persisted["runs"]["nf_p500_001"]["deletion_completed"])
                self.assertEqual(persisted["source_sha"], "a" * 40)
                self.assertEqual(persisted["execution_host"], socket.gethostname())
                self.assertEqual(path, target)

            with mock.patch.object(
                batch, "delete_invalid_run", side_effect=assert_checkpoint_before_delete
            ):
                evidence_path = batch.invalidate_and_delete(
                    state=state,
                    state_path=state_path,
                    record=record,
                    item=item,
                    target=target,
                    root=root,
                    evidence_dir=evidence_dir,
                    log=log,
                    classification="watchdog_confirmed_physical_deadlock",
                    reason="test",
                    watchdog_evidence=watchdog_path,
                )
            payload = json.loads(evidence_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["execution_host"], socket.gethostname())
            self.assertEqual(payload["source_sha"], "a" * 40)
            self.assertIsNotNone(payload["watchdog_evidence"]["sha256"])
            self.assertTrue(record["deletion_completed"])

    def test_integrity_faults_are_rejected_symmetrically(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for method_dir, run_id in (
                ("nearest", "nf_p500_001"),
                ("mapex", "mpx_p500_001"),
            ):
                target = root / method_dir / run_id
                target.mkdir(parents=True)
                (target / "summary.json").write_text(json.dumps({
                    "paper500_integrity_faults": ["odom_gap"],
                }), encoding="utf-8")
                self.assertEqual(batch.paper500_integrity_faults(target), ["odom_gap"])

    def test_empty_integrity_faults_remain_valid(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory)
            (target / "metadata.json").write_text(json.dumps({
                "paper500_integrity_faults": [],
            }), encoding="utf-8")
            self.assertEqual(batch.paper500_integrity_faults(target), [])

    def test_watchdog_progress_resets_window(self):
        detector = watchdog.DeadlockDetector(
            min_progress_m=0.05, window_s=10, startup_grace_s=0
        )
        detector.set_action_active(True, 0)
        detector.observe_odom(0, 0, 0)
        detector.observe_odom(0.03, 0, 4)
        detector.observe_odom(0.06, 0, 8)
        self.assertIsNone(detector.check(17.9))
        self.assertIsNotNone(detector.check(18))

    def test_nf_mapex_cutoff_reason_parity(self):
        nf = (SCRIPTS / "_nf_run_core.py").read_text(encoding="utf-8")
        mapex = (SCRIPTS / "mapex_run.py").read_text(encoding="utf-8")
        self.assertIn('reason != "budget_500_reached"', nf)
        self.assertIn('reason != "budget_500_reached"', mapex)
        self.assertNotIn("budget_1000_reached", nf + mapex)


if __name__ == "__main__":
    unittest.main()
