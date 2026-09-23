#!/usr/bin/env python3
from __future__ import annotations

import tempfile
import time
import unittest
import csv
import json
from pathlib import Path

import numpy as np

import r003_paper1000 as r003
import predict_alltrain_offline as alltrain
import evaluate_r003_paper1000 as evaluator


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


class R003BudgetTests(unittest.TestCase):
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


class R003ProfileTests(unittest.TestCase):
    def test_generated_profile_contract(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            manifest = r003.build_profile(
                root / "map" / "new_room.sdf", Path(directory), git_commit="test"
            )
            self.assertEqual(manifest["reduced_canvas"]["shape"], [1062, 752])
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
            samples = run / "paper1000_snapshots.csv"
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


if __name__ == "__main__":
    unittest.main()
