#!/usr/bin/env python3
from __future__ import annotations

import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "mapex_lab" / "scripts"
sys.path.insert(0, str(SCRIPTS))

import mx049_acquisition as acq
import mx049_generator as gen
import run_mx049_preflight as preflight


class MX049ContractTests(unittest.TestCase):
    def test_fresh_identity_is_exact(self) -> None:
        self.assertEqual(tuple(range(49001, 49013)), gen.LAYOUT_SEEDS)
        self.assertEqual("MX049_FRESH_DELL_V1", gen.COHORT_ID)
        self.assertEqual("mx049_fresh_dell_l49001_r1", acq.run_id(49001, 1))
        self.assertIn("mx049_fresh_dell", gen.GENERATED_WORLD_REL)
        self.assertNotIn("mx045_shadow", gen.GENERATED_WORLD_REL)

    def test_run_seed_formula_remains_mx048_and_is_effective(self) -> None:
        expected = {
            tag: acq.derive_seed32(49001, 1, tag) for tag in acq.COMPONENT_TAGS
        }
        repeated = {
            tag: acq.derive_seed32(49001, 1, tag) for tag in acq.COMPONENT_TAGS
        }
        alternate = {
            tag: acq.derive_seed32(49001, 2, tag) for tag in acq.COMPONENT_TAGS
        }
        self.assertEqual(expected, repeated)
        self.assertTrue(all(expected[tag] != alternate[tag] for tag in expected))

    def test_old_denylist_is_five_field_identity_only(self) -> None:
        with tempfile.TemporaryDirectory(prefix="mx049_denylist_test_") as tmp:
            root = Path(tmp)
            shutil.copy2(ROOT / preflight.OLD_MANIFEST_REL, root / preflight.OLD_MANIFEST_REL)
            obj = preflight.build_old_geometry_denylist(root)
            self.assertEqual("MX049_OLD_GEOMETRY_DENYLIST_V1", obj["schema_version"])
            self.assertEqual(list(range(45001, 45013)), obj["source_seed_domain"])
            self.assertEqual(12, len(obj["entries"]))
            allowed = {
                "layout_seed", "accepted_attempt", "geometry_parameter_digest",
                "world_sha256", "generator_id",
            }
            self.assertTrue(all(set(entry) == allowed for entry in obj["entries"]))
            frozen = json.loads((root / acq.DENYLIST_REL).read_text(encoding="utf-8"))
            self.assertEqual(obj, frozen)

    def test_cross_cohort_collision_is_not_reserve_eligible(self) -> None:
        source = (SCRIPTS / "mx049_generator.py").read_text(encoding="utf-8")
        self.assertIn('"CROSS_COHORT_GEOMETRY_IDENTITY_COLLISION"', source)
        decision = acq.reserve_decision(
            "development",
            [{
                "layout_seed": 49001,
                "primary_or_reserve": "primary",
                "technical_invalid": False,
                "scientific_weakness": False,
                "identity_collision": True,
            }],
        )
        self.assertEqual("NO_RESERVE", decision["action"])

    def test_confirmation_reader_stays_sealed(self) -> None:
        recorder = (SCRIPTS / "mapex_run.py").read_text(encoding="utf-8")
        self.assertIn("if self.mx048_confirmation_sealed", recorder)
        self.assertIn("should_run_offline_evaluator(args.acquisition_only)", recorder)
        with self.assertRaises(PermissionError) as ctx:
            acq.require_unsealed("roi", None)
        self.assertEqual(acq.SEAL_ERROR, str(ctx.exception))

    def test_only_fresh_pass_token_can_release_collection(self) -> None:
        self.assertEqual(
            "MX049_DELL_PREFLIGHT_PASS_READY_FOR_MX050_COLLECTION",
            acq.PASS_CLASSIFICATION,
        )
        self.assertEqual("MX049_DELL_PREFLIGHT_PASS_TOKEN.json", acq.PASS_TOKEN_REL)


if __name__ == "__main__":
    unittest.main()
