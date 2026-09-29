import json
import math
import tempfile
import unittest
from pathlib import Path
import numpy as np

from mapex_lab.analysis.d1.mx018_hospital_gt_recovery import RESULT_TO_ACTION, SCHEMA, canonical_manifest, f64be_hex, recovery_result

class MX018Tests(unittest.TestCase):
    def test_float_encoding(self): self.assertEqual(f64be_hex(.05),"3fa999999999999a")
    def test_float_rejects_nan(self):
        with self.assertRaises(ValueError): f64be_hex(math.nan)
    def test_closed_mapping(self):
        self.assertEqual(len(RESULT_TO_ACTION),9)
        self.assertEqual(RESULT_TO_ACTION["SEMANTIC_RECONSTRUCTION_SUPPORTED_NOT_PROVEN"],"GENERATE_GT_V2_AFTER_SEPARATE_ARTIFACT_GATE")
    def test_nonexact_never_passes_g2(self):
        value=recovery_result("SEMANTIC_RECONSTRUCTION_SUPPORTED_NOT_PROVEN","a"*64,"b"*64)
        self.assertEqual(value["mx015_g2_literal_status"],"FAIL_EXACT_SHA")
        self.assertFalse(value["decision_30_used_for_gt_construction"])
    def test_exact_mapping_is_only_g2_pass(self):
        for name in RESULT_TO_ACTION:
            value=recovery_result(name,"a"*64,"b"*64)
            self.assertEqual(value["mx015_g2_literal_status"]=="PASS_EXACT_SHA",name in {"ORIGINAL_EXACT_ARTIFACT_RECOVERED","EXACT_BINARY_RECONSTRUCTION"})
    def test_canonical_manifest_deterministic(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/"gt.npz"; data=np.full((2123,1504),-1,np.int16); mask=np.zeros_like(data,bool)
            np.savez_compressed(p,data=data,evaluation_mask=mask,resolution=np.float64(.05),origin_x=np.float64(-25.6),origin_y=np.float64(-60.1),canvas_id=np.asarray("hospital_canvas_v1"),hospital_scale=np.float64(1),spawn_world_x=np.float64(0),spawn_world_y=np.float64(12),spawn_world_yaw=np.float64(-1.57),source_world_sha256=np.asarray("x"*64),source_wall_mesh_sha256=np.asarray("y"*64))
            m1,b1,h1=canonical_manifest(p,gt_id="hospital_structural_gt_v1",evaluation_domain_rule="historical_hospital_bounds_then_start_connected_free_plus_structural_occupied")
            m2,b2,h2=canonical_manifest(p,gt_id="hospital_structural_gt_v1",evaluation_domain_rule="historical_hospital_bounds_then_start_connected_free_plus_structural_occupied")
            self.assertEqual((m1,b1,h1),(m2,b2,h2)); self.assertEqual(m1["schema"],SCHEMA); self.assertFalse(b1.endswith(b"\n"))

if __name__=="__main__": unittest.main()
