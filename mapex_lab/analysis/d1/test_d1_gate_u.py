import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

from d1_gate_u import (CANDIDATES, EXPECTED_COUNTS, EXPECTED_INPUT_SHA256,
                       RUNS, classify, cw25, empirical_cdf, heldout_srr, ratio_low50,
                       score_fold, select_candidate, sha256_file, valid_spearman,
                       validate_input)
from run_gate_u_low_touch import validate_fold_cache


ROOT = Path(__file__).resolve().parent
INPUT = ROOT / "results/shared_phase0_evidence_v1/shared_phase0_evidence.csv"


class GateUTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frame = pd.read_csv(INPUT, low_memory=False)

    def test_01_exact_input_hash(self):
        self.assertEqual(sha256_file(INPUT), EXPECTED_INPUT_SHA256)

    def test_02_exact_universe(self):
        result = validate_input(self.frame, EXPECTED_INPUT_SHA256)
        self.assertEqual(result["counts"], EXPECTED_COUNTS)
        self.assertEqual(len(result["runs"]), 10)

    def test_03_duplicate_rejected(self):
        dup = pd.concat([self.frame, self.frame.iloc[:1]])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            validate_input(dup, EXPECTED_INPUT_SHA256)

    def test_04_changed_hash_rejected(self):
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            validate_input(self.frame, "0" * 64)

    def test_05_empirical_cdf_uses_leq_and_preserves_ties(self):
        got = empirical_cdf([1, 1, 2, 3], [1, 2, 0])
        np.testing.assert_allclose(got, [.5, .75, 0])

    def test_06_low50_ties_not_split(self):
        ratio, retained, total = ratio_low50(pd.Series([1, 1, 2, 3]), pd.Series([1, 2, 3, 4]))
        self.assertEqual((retained, total), (2, 4))
        self.assertAlmostEqual(ratio, 1.5 / 2.5)

    def test_07_low25_can_be_empty_due_to_ties(self):
        rate, retained, total = cw25(pd.Series([1, 1, 1, 2]), pd.Series([1, 2, 3, 4]))
        self.assertIsNone(rate)
        self.assertEqual((retained, total), (0, 4))

    def test_08_spearman_support_and_constant_rules(self):
        self.assertEqual(valid_spearman([1, 2, 3, 4], [1, 2, 3, 4])[0], None)
        self.assertEqual(valid_spearman([1] * 5, range(5))[0], None)
        self.assertAlmostEqual(valid_spearman(range(5), range(5))[0], 1.0)

    def test_09_candidate_name_set_frozen(self):
        self.assertEqual(CANDIDATES, ("U_p95", "U_mean", "U_disagreement"))

    def test_10_holdout_excluded_from_selection(self):
        held = RUNS[0]
        fold, _ = score_fold(self.frame, held)
        self.assertNotIn(held, fold["development_runs"])
        self.assertEqual(len(fold["development_runs"]), 9)

    def test_11_fold_is_deterministic(self):
        one, rows_one = score_fold(self.frame, RUNS[0])
        two, rows_two = score_fold(self.frame.sample(frac=1, random_state=3), RUNS[0])
        self.assertEqual(one["selected_candidate"], two["selected_candidate"])
        self.assertEqual(one["selection_status"], two["selection_status"])
        self.assertEqual(set(rows_one.decision_id), set(rows_two.decision_id))

    def test_12_non_evaluable_rows_preserved(self):
        _, rows = score_fold(self.frame, RUNS[0])
        self.assertEqual(len(rows), EXPECTED_COUNTS[RUNS[0]])

    def test_13_percentiles_do_not_use_heldout_risk_for_u(self):
        _, original = score_fold(self.frame, RUNS[0])
        changed = self.frame.copy()
        changed.loc[changed.run_id == RUNS[0], "lost_reachable_future_free_fraction"] = 0.999
        _, mutated = score_fold(changed, RUNS[0])
        np.testing.assert_allclose(original.U_dev_percentile, mutated.U_dev_percentile, equal_nan=True)

    def test_14_no_admissible_precedence(self):
        folds = [{"selection_status": "NO_ADMISSIBLE_U"} for _ in range(5)] + [
            {"selection_status": "SELECTED", "primary_pairs": 0, "rho_primary": None,
             "srr50_run": None, "cw25_rate_run": None, "low_u_primary_rows": 0,
             "severe_cw_events": [], "broad_diagnostic": {}, "stage_diagnostics": {}}
            for _ in range(5)]
        self.assertEqual(classify(folds)["outcome"], "FAIL")

    def test_15_general_sufficiency_precedes_other_failures(self):
        folds = [{"selection_status": "SELECTED", "primary_pairs": 4, "rho_primary": -1.0,
                  "srr50_run": 2.0, "cw25_rate_run": .8, "low_u_primary_rows": 1,
                  "severe_cw_events": [{"x": 1}], "broad_diagnostic": {"contradiction": True},
                  "stage_diagnostics": {}} for _ in range(4)]
        folds += [{"selection_status": "NO_ADMISSIBLE_U"} for _ in range(6)]
        self.assertEqual(classify(folds)["outcome"], "FAIL")  # structural rule fires first

    def test_16_broad_undefined_not_contradiction(self):
        fold, _ = score_fold(self.frame, RUNS[0])
        broad = fold.get("broad_diagnostic", {})
        if not broad.get("rho_valid") or not broad.get("srr50_valid"):
            self.assertFalse(broad.get("contradiction", False))

    def test_17_stage_bins_are_frozen(self):
        fold, _ = score_fold(self.frame, RUNS[0])
        if fold["selection_status"] == "SELECTED":
            self.assertEqual(set(fold["stage_diagnostics"]), {"[0.00,0.25)", "[0.25,0.50)", "[0.50,0.75)", "[0.75,1.00]"})

    def test_18_fold_output_has_no_operational_fields(self):
        fold, _ = score_fold(self.frame, RUNS[0])
        text = json.dumps(fold).lower()
        for forbidden in ("k_confirm", "stop_threshold", "gate_r"):
            self.assertNotIn(forbidden, text)

    def test_19_per_decision_audit_flags_and_undefined(self):
        _, rows = score_fold(self.frame, RUNS[0])
        for name in ("low_u_25", "high_risk_75", "cw25", "severe_cw"):
            self.assertIn(name, rows.columns)
        relative = rows.dropna(subset=["U_dev_percentile", "R_dev_percentile"])
        self.assertTrue((relative.low_u_25 == (relative.U_dev_percentile <= .25)).all())
        self.assertTrue((relative.high_risk_75 == (relative.R_dev_percentile >= .75)).all())
        self.assertTrue((relative.cw25 == (relative.low_u_25 & relative.high_risk_75)).all())
        undefined = rows[rows.U_dev_percentile.isna()]
        self.assertTrue(undefined.low_u_25.isna().all())
        self.assertTrue(undefined.cw25.isna().all())

    def test_20_srr25_srr50_srr75_explicit(self):
        fold, rows = score_fold(self.frame, RUNS[0])
        primary = rows.dropna(subset=["U_selected", "R_primary", "U_dev_percentile", "R_dev_percentile"])
        for cutoff, name in ((.25, "srr25_run"), (.50, "srr50_run"), (.75, "srr75_run")):
            self.assertIn(name, fold)
            expected = heldout_srr(primary, cutoff)
            self.assertEqual(fold[name], expected)

    def test_21_srr_undefined_for_empty_or_zero_baseline(self):
        empty = pd.DataFrame({"R_primary": [1.0], "U_dev_percentile": [.9]})
        self.assertIsNone(heldout_srr(empty, .25))
        zero = pd.DataFrame({"R_primary": [0.0, 0.0], "U_dev_percentile": [.1, .2]})
        self.assertIsNone(heldout_srr(zero, .25))

    def _cache_fixture(self, root: Path):
        identity = {"method_id": "method", "input_sha256": "input", "implementation_sha256": "implementation"}
        (root / "fold_summary.json").write_text("{}\n")
        (root / "per_decision.csv").write_text("run_id,decision_id\nmpx_001,1\n")
        manifest = {"identity": identity, "fold": "mpx_001",
                    "fold_summary_sha256": sha256_file(root / "fold_summary.json"),
                    "per_decision_sha256": sha256_file(root / "per_decision.csv")}
        (root / "completion_manifest.json").write_text(json.dumps(manifest))
        return identity

    def test_22_cache_rejects_changed_input_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); identity = self._cache_fixture(root)
            changed = dict(identity, input_sha256="changed")
            with self.assertRaisesRegex(RuntimeError, "identity"):
                validate_fold_cache(root, changed, "mpx_001")

    def test_23_cache_rejects_changed_method_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); identity = self._cache_fixture(root)
            changed = dict(identity, method_id="changed")
            with self.assertRaisesRegex(RuntimeError, "identity"):
                validate_fold_cache(root, changed, "mpx_001")

    def test_24_cache_rejects_changed_implementation_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); identity = self._cache_fixture(root)
            changed = dict(identity, implementation_sha256="changed")
            with self.assertRaisesRegex(RuntimeError, "identity"):
                validate_fold_cache(root, changed, "mpx_001")

    def test_25_cache_rejects_corrupt_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); identity = self._cache_fixture(root)
            (root / "fold_summary.json").write_text("corrupt")
            with self.assertRaisesRegex(RuntimeError, "fold_summary_sha256"):
                validate_fold_cache(root, identity, "mpx_001")

    def test_26_cache_rejects_corrupt_per_decision(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); identity = self._cache_fixture(root)
            (root / "per_decision.csv").write_text("corrupt")
            with self.assertRaisesRegex(RuntimeError, "per_decision_sha256"):
                validate_fold_cache(root, identity, "mpx_001")

    def test_27_cache_rejects_missing_artifact(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); identity = self._cache_fixture(root)
            (root / "per_decision.csv").unlink()
            with self.assertRaisesRegex(RuntimeError, "missing required"):
                validate_fold_cache(root, identity, "mpx_001")


if __name__ == "__main__":
    unittest.main()
