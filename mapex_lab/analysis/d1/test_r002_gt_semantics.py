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
