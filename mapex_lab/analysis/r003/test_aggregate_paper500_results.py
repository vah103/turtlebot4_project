import tempfile
import unittest
from pathlib import Path

from mapex_lab.analysis.r003 import aggregate_paper500_results as aggregate


class Paper500AggregateTests(unittest.TestCase):
    def test_official_universe_has_exact_class_counts_and_order(self):
        run_ids = [run_id for _, _, run_id in aggregate.RUNS]
        aggregate.validate_run_ids(run_ids)
        self.assertEqual(sum(algorithm == "NF" for algorithm, _, _ in aggregate.RUNS), 10)
        self.assertEqual(sum(algorithm == "MapEx" for algorithm, _, _ in aggregate.RUNS), 10)
        self.assertEqual(run_ids[0], "nf_p500_001")
        self.assertEqual(run_ids[-1], "mpx_p500_010")

    def test_missing_and_duplicate_runs_are_rejected(self):
        run_ids = [run_id for _, _, run_id in aggregate.RUNS]
        with self.assertRaisesRegex(ValueError, "universe mismatch"):
            aggregate.validate_run_ids(run_ids[:-1])
        with self.assertRaisesRegex(ValueError, "duplicate run ID"):
            aggregate.validate_run_ids(run_ids[:-1] + [run_ids[0]])

    def test_common_support_mismatch_and_duplicates_are_rejected(self):
        aggregate.validate_steps(aggregate.STEPS)
        with self.assertRaisesRegex(ValueError, "support mismatch"):
            aggregate.validate_steps(aggregate.STEPS[:-1])
        with self.assertRaisesRegex(ValueError, "duplicate adapted step"):
            aggregate.validate_steps(list(aggregate.STEPS[:-1]) + [aggregate.STEPS[-2]])

    def test_endpoint_parity_guard(self):
        aggregate.require_close(0.5, 0.5, "endpoint", atol=1e-12)
        with self.assertRaisesRegex(ValueError, "endpoint mismatch"):
            aggregate.require_close(0.5, 0.6, "endpoint", atol=1e-12)

    def test_deterministic_trapezoid_auc(self):
        self.assertEqual(aggregate.trapezoid([0, 10, 20], [0.0, 1.0, 0.0]), 10.0)

    def test_summary_uses_sample_std_across_runs(self):
        summary = aggregate.summarize(range(10))
        self.assertEqual(summary["n"], 10)
        self.assertAlmostEqual(summary["mean"], 4.5)
        self.assertAlmostEqual(summary["sample_std"], 3.0276503540974917)

    def test_group_summary_requires_ten_per_algorithm(self):
        rows = []
        for algorithm in ("NF", "MapEx"):
            for index in range(10):
                row = {"algorithm": algorithm}
                row.update({key: float(index) for key, _, _, _ in aggregate.METRICS})
                rows.append(row)
        result = aggregate.group_summaries(rows)
        self.assertTrue(all(row["n"] == 10 for row in result))
        self.assertEqual(len(result), 12)

    def test_tu_auc_is_explicitly_project_added(self):
        tu = next(item for item in aggregate.METRICS if item[0] == "tu_auc_project_added")
        self.assertTrue(tu[3])
        self.assertFalse(next(item for item in aggregate.METRICS if item[0] == "coverage_auc")[3])

    def test_curve_summary_order_is_deterministic(self):
        rows = []
        for algorithm in ("MapEx", "NF"):
            for step in reversed(aggregate.STEPS):
                for run in range(10):
                    rows.append({"algorithm": algorithm, "run_id": str(run), "adapted_step": step,
                                 "coverage": run / 10, "occupied_iou": run / 20, "tu": run / 25})
        result = aggregate.curve_summaries(rows)
        self.assertEqual((result[0]["algorithm"], result[0]["adapted_step"]), ("NF", 0))
        self.assertEqual((result[-1]["algorithm"], result[-1]["adapted_step"]), ("MapEx", 500))
        self.assertEqual(len(result), 102)

    def test_changed_data_identity_is_rejected(self):
        aggregate.require_data_identity(aggregate.DATA_SHA)
        with self.assertRaisesRegex(ValueError, "data identity mismatch"):
            aggregate.require_data_identity("different")

    def test_fingerprinting_does_not_mutate_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "source.csv"
            path.write_text("a,b\n1,2\n", encoding="utf-8")
            before = aggregate.sha256(path)
            self.assertEqual(before, aggregate.sha256(path))
            self.assertEqual(path.read_text(encoding="utf-8"), "a,b\n1,2\n")

    def test_generated_csv_uses_lf_line_endings(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "output.csv"
            aggregate.write_csv(path, [{"a": 1, "b": 2}])
            self.assertEqual(path.read_bytes(), b"a,b\n1,2\n")


if __name__ == "__main__":
    unittest.main()
