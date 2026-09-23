"""ROS-free H031 batch/watchdog regressions."""
from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


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
        self.assertIsNotNone(detector.check(20))
        detector.set_action_active(False, 21)
        self.assertIsNone(detector.check(100))

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

