import math
import unittest

import numpy as np

from mapex_lab.analysis.r004 import evaluate_free_error_diagnostic as d


class FreeErrorDiagnosticTests(unittest.TestCase):
    def test_identity_and_union_decomposition(self):
        pred = np.array([0.1, 0.2, 0.9, 0.9])
        truth = np.array([False, True, False, True])
        row = d.decision_metrics(pred, truth, 2, 2, 0, np.array([0, 3, 0, 1]))
        self.assertEqual((row["tp_free_count"], row["false_free_count"], row["missed_free_count"]), (1, 1, 1))
        self.assertEqual(row["free_iou_union_count"], 3)
        self.assertAlmostEqual(row["false_free_union_share"], 1 / 3)
        self.assertAlmostEqual(row["missed_free_union_share"], 1 / 3)

    def test_threshold_is_strict(self):
        tp, ff, miss = d.classify(np.array([0.5, np.nextafter(0.5, 1.0)]), np.array([False, False]))
        self.assertEqual((int(tp.sum()), int(ff.sum()), int(miss.sum())), (1, 0, 1))

    def test_supported_future_free_miss_denominator(self):
        row = d.decision_metrics(np.array([0.1, 0.9, 0.9]), np.array([False, False, True]), 5, 2, 3, np.zeros(3))
        self.assertAlmostEqual(row["supported_future_free_miss_fraction"], 0.5)
        self.assertEqual(row["supported_future_free_count"], 2)

    def test_unsupported_free_is_support_not_error(self):
        row = d.decision_metrics(np.array([0.1]), np.array([False]), 4, 1, 3, np.zeros(1))
        self.assertEqual(row["missed_free_count"], 0)
        self.assertEqual(row["unsupported_future_free_count"], 3)
        self.assertAlmostEqual(row["unsupported_future_free_fraction"], 0.75)

    def test_euclidean_axial_and_diagonal_depth(self):
        final = np.ones((5, 5), dtype=int)
        final[0, 0] = 0
        depth = d.occupied_depth(final)
        self.assertAlmostEqual(depth[0, 2], 2.0)
        self.assertAlmostEqual(depth[2, 2], math.sqrt(8))

    def test_exact_depth_band_boundaries(self):
        bands = d.depth_band(np.array([1.0, np.nextafter(1.0, 2.0), 2.0, np.nextafter(2.0, 3.0)]))
        self.assertEqual(list(bands), ["boundary", "near_interior", "near_interior", "deep_occupied"])

    def test_depth_independent_of_support_or_observability(self):
        final = np.array([[0, 1, 1], [-1, 1, 1]])
        first = d.occupied_depth(final)
        support = np.array([[True, False, True], [False, True, False]])
        observed = np.array([[False, True, False], [True, False, True]])
        second = d.occupied_depth(final)
        np.testing.assert_array_equal(first, second)
        self.assertFalse(np.array_equal(support, observed))

    def test_denominator_controlled_rate(self):
        pred = np.array([0.1, 0.1, 0.9, 0.9])
        truth = np.ones(4, dtype=bool)
        row = d.decision_metrics(pred, truth, 0, 0, 0, np.array([3.0, 3.0, 3.0, 3.0]))
        self.assertAlmostEqual(row["false_free_deep_occupied_rate"], 0.5)
        self.assertAlmostEqual(row["false_free_deep_occupied_share"], 1.0)

    def test_run_macro_equal_weight(self):
        rows = []
        for rid, values in (("mpx_001", [0.0] * 100), ("mpx_002", [1.0])):
            for value in values:
                row = {"run_id": rid, "progress_bin": d.BINS[0],
                       "supported_future_free_count": 1, "unsupported_future_free_count": 0}
                row.update({metric: value for metric in d.metric_names()})
                rows.append(row)
        _, macro = d.aggregate(rows)
        self.assertAlmostEqual(macro[0]["false_free_fraction_mean"], 0.5)
        self.assertEqual(macro[0]["false_free_fraction_n"], 2)

    def test_deep_exclusion_changes_precision_iou_not_recall(self):
        pred = np.array([0.1, 0.1, 0.9])
        truth = np.array([False, True, False])
        row = d.decision_metrics(pred, truth, 2, 2, 0, np.array([0.0, 3.0, 0.0]))
        self.assertGreater(row["free_precision_excluding_deep"], row["free_precision"])
        self.assertGreater(row["free_iou_excluding_deep"], row["free_iou"])
        self.assertAlmostEqual(row["free_recall"], 0.5)

    def test_no_known_free_seed_rejected(self):
        with self.assertRaisesRegex(ValueError, "no known-free seed"):
            d.occupied_depth(np.ones((3, 3), dtype=int))


if __name__ == "__main__":
    unittest.main()
