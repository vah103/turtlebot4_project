#!/usr/bin/env python3
from __future__ import annotations

import tempfile
import time
import unittest
import csv
import json
from pathlib import Path

import numpy as np

import r003_paper500 as r003
import predict_alltrain_offline as alltrain
import evaluate_r003_paper500 as evaluator


class R003GridTests(unittest.TestCase):
    def test_odd_row_padding_is_asymmetric(self):
        source = np.zeros((r003.CANVAS_H, r003.CANVAS_W), dtype=bool)
        occupied = r003.reduce_any(source, pad_value=True)
        valid = r003.reduce_any(source, pad_value=False)
        self.assertTrue(np.all(occupied[-1]))
        self.assertFalse(np.any(valid[-1]))

    def test_observed_reduction_and_unknown_support(self):
        source = np.full((r003.CANVAS_H, r003.CANVAS_W), -1, dtype=np.int16)
        source[0, 0] = 0
        source[0, 1] = 100
        reduced = r003.reduce_observed(source)
        self.assertEqual(reduced[0, 0], 100)  # occupied wins
        self.assertEqual(reduced[0, 1], -1)

    def test_prediction_threshold_is_strictly_greater_than_half(self):
        probability = np.zeros((r003.REDUCED_H, r003.REDUCED_W), dtype=np.float32)
        gt = np.zeros_like(probability, dtype=bool)
        domain = np.zeros_like(gt)
        domain[0, :2] = True
        probability[0, 0] = 0.5
        probability[0, 1] = np.nextafter(np.float32(0.5), np.float32(1.0))
        gt[0, 1] = True
        result = r003.occupied_iou(probability, gt, domain)
        self.assertEqual(result["value"], 1.0)
        self.assertEqual((result["tp"], result["fp"], result["fn"]), (1, 0, 0))

    def test_axis_aligned_projection_leaves_outside_unknown(self):
        raw = np.asarray([[0, 100], [-1, 0]], dtype=np.int16)
        projected, support = r003.project_runtime_observed(
            raw, 0.1, r003.CANVAS_X, r003.CANVAS_Y, 0.0
        )
        self.assertTrue(np.all(support[:4, :4]))
        self.assertFalse(support[-1, -1])
        self.assertEqual(projected[-1, -1], -1)
        with self.assertRaises(ValueError):
            r003.project_runtime_observed(raw, 0.1, 0.0, 0.0, 0.1)

    def test_prediction_projection_preserves_float_probabilities(self):
        probability = np.asarray(
            [[0.49, 0.50], [0.51, 0.80]],
            dtype=np.float32,
        )
        projected, support = r003.project_runtime_prediction(
            probability,
            0.1,
            r003.CANVAS_X,
            r003.CANVAS_Y,
            0.0,
        )
        self.assertTrue(np.all(support[:4, :4]))
        self.assertFalse(support[-1, -1])
        self.assertAlmostEqual(float(projected[0, 0]), 0.49, places=6)
        self.assertAlmostEqual(float(projected[0, 2]), 0.50, places=6)
        self.assertAlmostEqual(float(projected[2, 0]), 0.51, places=6)
        self.assertAlmostEqual(float(projected[2, 2]), 0.80, places=6)

        reduced = r003.reduce_prediction(projected)
        np.testing.assert_allclose(
            reduced[:2, :2],
            probability,
            rtol=0.0,
            atol=1e-6,
        )


class R003BudgetTests(unittest.TestCase):
    def test_frozen_paper500_horizon_and_crossings(self):
        self.assertEqual(r003.MAX_STEPS, 500)
        self.assertAlmostEqual(r003.BUDGET_METERS, 150.0)
        self.assertEqual(r003.PROFILE_ID, "new_room_mapex_paper500_v1")
        self.assertEqual(r003.BUDGET_ID, "odom_progress_0p30m_500step_v1")
        crossings = r003.common_crossings(range(1, 501))
        self.assertEqual(crossings[-1], 500)
        self.assertEqual(len(crossings), 50)

    def test_residual_carries_across_goal_and_recovery_motion(self):
        budget = r003.OdomProgressBudget(step_m=0.3, max_steps=10)
        budget.start(0.0, 0.0, 1.0, "odom")
        first = budget.update(0.19, 0.0, 2.0, "odom")
        second = budget.update(0.35, 0.0, 3.0, "odom")
        third = budget.update(0.25, 0.0, 4.0, "odom")  # backward recovery counts
        self.assertEqual(first.step, 0)
        self.assertEqual(second.step, 1)
        self.assertEqual(third.step, 1)
        self.assertAlmostEqual(third.distance_m, 0.45)
        self.assertAlmostEqual(third.residual_m, 0.15)

    def test_integrity_faults_are_latched(self):
        budget = r003.OdomProgressBudget()
        budget.start(0.0, 0.0, 1.0, "odom")
        with self.assertRaises(ValueError):
            budget.update(0.1, 0.0, 1.0, "odom")
        self.assertTrue(budget.integrity_faults)

    def test_gap_and_reset_jump_are_integrity_faults(self):
        gap = r003.OdomProgressBudget(max_gap_s=0.5)
        gap.start(0.0, 0.0, 1.0, "odom")
        with self.assertRaisesRegex(ValueError, "odom_gap"):
            gap.update(0.1, 0.0, 2.0, "odom")
        jump = r003.OdomProgressBudget(max_increment_m=1.0)
        jump.start(0.0, 0.0, 1.0, "odom")
        with self.assertRaisesRegex(ValueError, "position_jump"):
            jump.update(2.0, 0.0, 1.1, "odom")

    def test_cutoff_can_latch_while_scoring_thread_is_busy(self):
        budget = r003.OdomProgressBudget(step_m=0.3, max_steps=1)
        budget.start(0.0, 0.0, 1.0, "odom")
        crossed = r003.request_cutoff_during_scoring(
            budget,
            [(0.31, 0.0, 2.0, "odom")],
            lambda: time.sleep(0.01),
        )
        self.assertTrue(crossed)
        self.assertTrue(budget.cutoff_latched)


class R003TUTests(unittest.TestCase):
    def test_fixed_goals_report_no_path_and_gt_collision(self):
        shape = (4, 5)
        probability = np.ones(shape, dtype=np.float32)
        evaluation = np.ones(shape, dtype=bool)
        occupied = np.zeros(shape, dtype=bool)
        probability[1, 0:5] = 0.0
        goals = np.asarray([[1, 4], [3, 4]], dtype=np.int32)
        result = r003.topological_understanding(
            probability, occupied, evaluation, (1, 0), goals
        )
        self.assertEqual(result["denominator"], 2)
        self.assertEqual(result["success"], 1)
        self.assertEqual(result["predicted_occupied"], 1)

        occupied[1, 2] = True
        collision = r003.topological_understanding(
            probability, occupied, evaluation, (1, 0), goals[:1]
        )
        self.assertEqual(collision["gt_collision"], 1)

    def test_nonfinite_is_evaluation_error_not_denominator_drop(self):
        probability = np.zeros((2, 2), dtype=np.float32)
        probability[0, 0] = np.nan
        result = r003.topological_understanding(
            probability, np.zeros((2, 2), bool), np.ones((2, 2), bool),
            (0, 0), np.asarray([[1, 1]], dtype=np.int32),
        )
        self.assertEqual(result["status"], "evaluation_error")
        self.assertEqual(result["denominator"], 1)

    def test_authoritative_final_selection(self):
        rows = [
            {"event": "progress", "sample_id": "1"},
            {"event": "budget_cutoff", "sample_id": "2"},
        ]
        self.assertEqual(r003.select_final_sample(rows)["sample_id"], "2")
        with self.assertRaises(ValueError):
            r003.select_final_sample(rows + [{"event": "natural_completion"}])

    def test_abnormal_final_is_authoritative_failure_support(self):
        rows = [
            {"event": "initial", "sample_id": "1"},
            {"event": "progress", "sample_id": "2"},
            {"event": "abnormal_final", "sample_id": "3"},
        ]
        self.assertEqual(r003.select_final_sample(rows)["sample_id"], "3")


class R003CurveTests(unittest.TestCase):
    @staticmethod
    def _row(sample_id, event, step, value):
        return {
            "sample_id": sample_id,
            "event": event,
            "progress_step": step,
            "distance_m": step * r003.STEP_METERS,
            "coverage": value,
            "known_fraction": value,
            "occupied_iou": value,
            "empty_iou_union": False,
            "tu": value,
            "tu_success": 0,
            "tu_predicted_occupied": 0,
            "tu_no_path": 0,
            "tu_gt_collision": 0,
            "observed_support_fraction": 1.0,
            "prediction_support_fraction": 1.0,
            "held": False,
            "held_from_sample_id": "",
        }

    def test_trapezoid_known_area(self):
        rows = [
            self._row(1, "initial", 0, 0.0),
            self._row(2, "progress", 10, 1.0),
            self._row(3, "abnormal_final", 15, 0.5),
        ]
        result = evaluator.trapezoidal_auc(rows, "coverage")
        self.assertAlmostEqual(result["area"], 8.75)
        self.assertEqual(result["start_step"], 0)
        self.assertEqual(result["end_step"], 15)
        self.assertEqual(result["point_count"], 3)

    def test_natural_completion_holds_to_500_and_marks_rows(self):
        rows = [
            self._row(1, "initial", 0, 0.0),
            self._row(2, "progress", 10, 1.0),
            self._row(3, "natural_completion", 15, 0.5),
            self._row(4, "post_cancellation", 16, 0.1),
        ]
        raw, held, curve = evaluator.build_curve_support(rows, rows[2])
        self.assertEqual([row["progress_step"] for row in raw], [0, 10, 15])
        self.assertEqual(curve[-1]["progress_step"], 500)
        self.assertTrue(all(row["held"] for row in held))
        self.assertTrue(
            all(row["event"] == "natural_completion_hold" for row in held)
        )
        self.assertTrue(
            all(row["held_from_sample_id"] == 3 for row in held)
        )
        self.assertAlmostEqual(
            evaluator.trapezoidal_auc(curve, "coverage")["area"],
            251.25,
        )

    def test_algorithmic_failure_stops_support_without_hold(self):
        rows = [
            self._row(1, "initial", 0, 0.0),
            self._row(2, "progress", 10, 1.0),
            self._row(3, "abnormal_final", 15, 0.5),
            self._row(4, "post_cancellation", 20, 0.9),
        ]
        raw, held, curve = evaluator.build_curve_support(rows, rows[2])
        self.assertEqual([row["progress_step"] for row in raw], [0, 10, 15])
        self.assertEqual(held, [])
        self.assertEqual(curve[-1]["progress_step"], 15)
        self.assertAlmostEqual(
            evaluator.trapezoidal_auc(curve, "coverage")["area"],
            8.75,
        )

    def test_same_step_final_replaces_earlier_progress_sample(self):
        rows = [
            self._row(1, "initial", 0, 0.0),
            self._row(2, "progress", 10, 0.2),
            self._row(3, "natural_completion", 10, 0.7),
        ]
        raw, held, curve = evaluator.build_curve_support(rows, rows[2])
        self.assertEqual(len(raw), 2)
        self.assertEqual(raw[-1]["sample_id"], 3)
        self.assertEqual(raw[-1]["coverage"], 0.7)
        self.assertTrue(held)
        self.assertEqual(curve[-1]["progress_step"], 500)


class R003ProfileTests(unittest.TestCase):
    def test_generated_profile_contract(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            manifest = r003.build_profile(
                root / "map" / "new_room.sdf", Path(directory), git_commit="test"
            )
            self.assertEqual(manifest["reduced_canvas"]["shape"], [1062, 752])
            self.assertEqual(manifest["budget"]["max_steps"], 500)
            self.assertEqual(manifest["budget"]["budget_m"], 150.0)
            self.assertEqual(manifest["budget"]["common_step_spacing"], 10)
            self.assertEqual(manifest["tu"]["goal_count"], 100)
            self.assertEqual(manifest["tu"]["seed"], 3001)
            self.assertEqual(
                manifest["counts"]["reduced"]["valid"],
                manifest["counts"]["reduced"]["evaluation"],
            )

    def test_snapshot_inference_dedup_and_evaluator_pipeline(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            temp = Path(directory)
            run = temp / "run"
            maps = run / "maps"
            maps.mkdir(parents=True)
            raw = maps / "same_raw.npz"
            data = np.zeros((r003.REDUCED_H, r003.REDUCED_W), dtype=np.int16)
            np.savez_compressed(
                raw, data=data, resolution=np.float64(0.1),
                width=np.int64(r003.REDUCED_W), height=np.int64(r003.REDUCED_H),
                origin_x=np.float64(r003.CANVAS_X), origin_y=np.float64(r003.CANVAS_Y),
                origin_yaw=np.float64(0.0),
            )
            samples = run / "paper500_snapshots.csv"
            fields = [
                "sample_id", "event", "progress_step", "distance_m", "residual_m",
                "receive_time_s", "source_map_stamp_s", "map_age_s", "repeated_sample",
                "raw_map_file", "raw_map_sha256", "cutoff_detection_overshoot_m",
            ]
            with samples.open("w", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for sid, event in ((1, "initial"), (2, "natural_completion")):
                    writer.writerow({
                        "sample_id": sid, "event": event, "progress_step": 0,
                        "distance_m": 0.0, "residual_m": 0.0,
                        "receive_time_s": sid, "source_map_stamp_s": sid,
                        "map_age_s": 0.0, "repeated_sample": int(sid == 2),
                        "raw_map_file": "maps/same_raw.npz",
                        "raw_map_sha256": alltrain.sha256(raw),
                        "cutoff_detection_overshoot_m": 0.0,
                    })
            checkpoint = temp / "model" / "models" / "best.ckpt"
            checkpoint.parent.mkdir(parents=True)
            checkpoint.write_bytes(b"test-checkpoint")
            (checkpoint.parent.parent / "config.yaml").write_text("test: true\n")
            manifest_path = alltrain.generate(
                run, checkpoint,
                lambda observed: np.zeros_like(observed, dtype=np.float32),
                snapshots=True,
            )
            prediction_manifest = json.loads(manifest_path.read_text())
            self.assertEqual(prediction_manifest["unique_inference_count"], 1)
            self.assertEqual(prediction_manifest["decisions"][1]["duplicate_of_id"], 1)

            profile_dir = temp / "profile"
            r003.build_profile(root / "map" / "new_room.sdf", profile_dir, git_commit="test")
            result = evaluator.evaluate(
                run,
                profile_dir / f"{r003.EVAL_ID}.npz",
                profile_dir / f"{r003.EVAL_ID}_tu_goals.npy",
            )
            self.assertEqual(result["status"], "ok")
            self.assertEqual(result["sample_count"], 2)
            self.assertEqual(result["final"]["sample_id"], 2)
            self.assertEqual(result["termination"]["kind"], "natural_completion")
            self.assertEqual(result["raw_support_count"], 1)
            self.assertEqual(result["raw_record_count_through_terminal"], 2)
            self.assertEqual(result["held_support_count"], 50)
            self.assertEqual(result["curve_support_count"], 51)
            self.assertEqual(result["endpoint_at_500"]["progress_step"], 500)
            self.assertTrue(result["endpoint_at_500"]["held"])
            self.assertTrue(
                (run / "evaluation" / "paper500" / "curve.csv").is_file()
            )


if __name__ == "__main__":
    unittest.main()
