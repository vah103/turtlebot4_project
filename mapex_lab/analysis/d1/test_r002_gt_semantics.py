#!/usr/bin/env python3
"""Toy tests required by frozen R002 V2 boundary-matching semantics."""

import math
import unittest

import numpy as np

import r002_gt_semantics as r002


class BoundaryMatchingTests(unittest.TestCase):
    def test_duplicate_prediction_cannot_receive_repeated_gt_credit(self):
        gt = np.asarray([[10, 10]], dtype=np.int32)
        pred = np.asarray([[10, 10], [10, 11]], dtype=np.int32)
        result = r002._boundary_matching(gt, pred, 0.05, 0.10)
        self.assertEqual(result["matched"], 1)
        self.assertAlmostEqual(result["precision"], 0.5)
        self.assertAlmostEqual(result["recall"], 1.0)
        self.assertAlmostEqual(result["f1"], 2.0 / 3.0)

    def test_parallel_boundary_is_one_to_one(self):
        gt = np.asarray(
            [[10, 10], [11, 10], [12, 10]],
            dtype=np.int32,
        )
        pred = np.asarray(
            [[10, 11], [11, 11], [12, 11], [11, 12]],
            dtype=np.int32,
        )
        result = r002._boundary_matching(gt, pred, 0.05, 0.10)
        self.assertEqual(result["matched"], 3)
        self.assertAlmostEqual(result["precision"], 0.75)
        self.assertAlmostEqual(result["recall"], 1.0)
        self.assertEqual(
            len(
                {
                    (p["gt_row"], p["gt_col"])
                    for p in result["pairs"]
                }
            ),
            3,
        )
        self.assertEqual(
            len(
                {
                    (p["pred_row"], p["pred_col"])
                    for p in result["pairs"]
                }
            ),
            3,
        )

    def test_exact_tolerance_matches_only_same_cell(self):
        gt = np.asarray(
            [[3, 4], [8, 9]],
            dtype=np.int32,
        )
        pred = np.asarray(
            [[3, 4], [8, 10]],
            dtype=np.int32,
        )
        result = r002._boundary_matching(
            gt,
            pred,
            0.05,
            r002.EXACT_BOUNDARY_TOLERANCE_M,
        )
        self.assertEqual(result["matched"], 1)
        self.assertEqual(
            result["pairs"][0],
            {
                "gt_row": 3,
                "gt_col": 4,
                "pred_row": 3,
                "pred_col": 4,
                "distance_m": 0.0,
            },
        )

    def test_equal_distance_tie_is_deterministic_lexicographic(self):
        gt = np.asarray([[5, 5]], dtype=np.int32)
        pred = np.asarray([[5, 4], [5, 6]], dtype=np.int32)
        first = r002._boundary_matching(
            gt,
            pred,
            0.05,
            0.05,
        )
        second = r002._boundary_matching(
            gt[::-1],
            pred[::-1],
            0.05,
            0.05,
        )
        self.assertEqual(first["pairs"], second["pairs"])
        self.assertEqual(first["pairs"][0]["pred_row"], 5)
        self.assertEqual(first["pairs"][0]["pred_col"], 4)

    def test_empty_support_rules(self):
        empty = np.empty((0, 2), dtype=np.int32)
        one = np.asarray([[0, 0]], dtype=np.int32)

        both_empty = r002._boundary_matching(
            empty,
            empty,
            0.05,
            0.10,
        )
        self.assertTrue(math.isnan(both_empty["precision"]))
        self.assertTrue(math.isnan(both_empty["recall"]))
        self.assertTrue(math.isnan(both_empty["f1"]))

        gt_empty = r002._boundary_matching(
            empty,
            one,
            0.05,
            0.10,
        )
        self.assertEqual(gt_empty["precision"], 0.0)
        self.assertTrue(math.isnan(gt_empty["recall"]))
        self.assertEqual(gt_empty["f1"], 0.0)

        pred_empty = r002._boundary_matching(
            one,
            empty,
            0.05,
            0.10,
        )
        self.assertTrue(math.isnan(pred_empty["precision"]))
        self.assertEqual(pred_empty["recall"], 0.0)
        self.assertEqual(pred_empty["f1"], 0.0)


    def test_large_component_keeps_maximum_cardinality_before_distance(self):
        # Full matching costs 0.05 m per pair. Dropping the first GT would
        # allow all remaining pairs to match exactly, so this catches any
        # implementation that trades cardinality for a lower total distance.
        n = 500
        gt = np.column_stack(
            (
                np.zeros(n, dtype=np.int32),
                np.arange(n, dtype=np.int32),
            )
        )
        pred = np.column_stack(
            (
                np.zeros(n, dtype=np.int32),
                np.arange(1, n + 1, dtype=np.int32),
            )
        )
        result = r002._boundary_matching(
            gt,
            pred,
            0.05,
            0.10,
        )
        self.assertEqual(result["matched"], n)

    def test_runtime_projection_uses_zero_free_positive_occupied(self):
        observed = r002.gate_p.RawGrid(
            data=np.asarray(
                [[-1, 0], [10, 100]],
                dtype=np.int16,
            ),
            resolution=0.10,
            origin_x=0.0,
            origin_y=0.0,
            origin_yaw=0.0,
            source_stamp_s=1.0,
        )
        gt = r002.gate_p.StructuralGT(
            data=np.zeros((4, 4), dtype=np.int16),
            evaluation_mask=np.ones((4, 4), dtype=bool),
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            ground_truth_id="toy",
            canvas_id="toy",
        )
        projection = r002._canonical_projection(observed, gt)
        self.assertEqual(
            int(np.count_nonzero(projection["unknown"])),
            4,
        )
        self.assertEqual(
            int(np.count_nonzero(projection["observed_free"])),
            4,
        )
        self.assertEqual(
            int(np.count_nonzero(projection["observed_occupied"])),
            8,
        )
        self.assertTrue(
            np.array_equal(
                projection["universe"],
                projection["unknown"],
            )
        )


class C0SeedTests(unittest.TestCase):
    def _toy_gt(self):
        return r002.gate_p.StructuralGT(
            data=np.zeros((7, 7), dtype=np.int16),
            evaluation_mask=np.ones((7, 7), dtype=bool),
            resolution=0.05,
            origin_x=0.0,
            origin_y=0.0,
            ground_truth_id="toy",
            canvas_id="toy",
        )

    def test_seed_fallback_uses_observed_free_and_lexicographic_tie(self):
        gt = self._toy_gt()
        observed_free = np.zeros(gt.shape, dtype=bool)
        topology_domain = np.ones(gt.shape, dtype=bool)

        # Robot lies at the center of cell (3,3), but that cell is not observed
        # free. Two observed-free candidates are equally distant at 0.05 m.
        observed_free[3, 2] = True
        observed_free[3, 4] = True
        robot_x, robot_y = r002._cell_center(3, 3, gt)

        seed, kind = r002._select_seed(
            robot_x,
            robot_y,
            observed_free,
            topology_domain,
            gt,
        )
        self.assertEqual(kind, "fallback")
        self.assertEqual(seed, (3, 2))

    def test_seed_invalid_without_observed_known_free(self):
        gt = self._toy_gt()
        observed_free = np.zeros(gt.shape, dtype=bool)
        topology_domain = np.ones(gt.shape, dtype=bool)
        robot_x, robot_y = r002._cell_center(3, 3, gt)
        seed, kind = r002._select_seed(
            robot_x,
            robot_y,
            observed_free,
            topology_domain,
            gt,
        )
        self.assertIsNone(seed)
        self.assertEqual(kind, "invalid")


class C0MetricTests(unittest.TestCase):
    def test_empty_positive_support_is_undefined_not_perfect(self):
        empty = np.zeros((2, 2), dtype=bool)
        result = r002._classification_positive_metrics(
            empty,
            empty,
        )
        self.assertTrue(math.isnan(result["precision"]))
        self.assertTrue(math.isnan(result["recall"]))
        self.assertTrue(math.isnan(result["iou"]))

    def test_false_positive_only(self):
        gt = np.zeros((2, 2), dtype=bool)
        pred = np.zeros((2, 2), dtype=bool)
        pred[0, 0] = True
        result = r002._classification_positive_metrics(
            gt,
            pred,
        )
        self.assertEqual(result["precision"], 0.0)
        self.assertTrue(math.isnan(result["recall"]))
        self.assertEqual(result["iou"], 0.0)


if __name__ == "__main__":
    unittest.main()
