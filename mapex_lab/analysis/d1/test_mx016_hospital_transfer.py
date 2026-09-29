import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from mapex_lab.analysis.d1.mx013_online_stop import OnlineDecision, RecognizerParams, replay_recognizer
from mapex_lab.analysis.d1.mx016_hospital_transfer import K, TAU_R, assert_phase_a_schema, invalid_result, phase_a_schema, run_phase_b

def decision(r=.03,topo=True,u=None):
    return OnlineDecision("hpx_001",1,1,0.,0.,r,True,"",topo,"OK" if topo else "BAD",topo,1,1.,1.,1.,low_u_25=u)

class MX016Tests(unittest.TestCase):
    def test_frozen_configuration(self): self.assertEqual((TAU_R,K),(.03,1))
    def test_equality_and_topology(self):
        self.assertTrue(replay_recognizer([decision()],RecognizerParams(TAU_R,K))[0]["first_fire"])
        self.assertFalse(replay_recognizer([decision(topo=False)],RecognizerParams(TAU_R,K))[0]["first_fire"])
    def test_u_non_gating(self): self.assertTrue(replay_recognizer([decision(u=None)],RecognizerParams(TAU_R,K,use_u_veto=False))[0]["first_fire"])
    def test_first_fire_one_shot(self): self.assertEqual(sum(x["first_fire"] for x in replay_recognizer([decision(),decision()],RecognizerParams(TAU_R,K))),1)
    def test_phase_a_schema_truth_free(self): assert_phase_a_schema(phase_a_schema())
    def test_phase_a_schema_rejects_truth(self):
        with self.assertRaises(AssertionError): assert_phase_a_schema(phase_a_schema()+["n_gt_h"])
    def test_invalid_gate_refuses_timing_classification(self):
        result=invalid_result({"candidate_stop_decision":4,"candidate_stop_time_s":10.,"final_decision_time_s":20.,"phase_a_trace_sha256":"x"},{"fatal_failures":["G2"]})
        self.assertIsNone(result["timing_status"]); self.assertEqual(result["transfer_label"],"TRANSFER_SANITY_INCONCLUSIVE_ORACLE_INVALID")
        self.assertEqual(result["time_saved_s"],10.)
    def test_phase_b_requires_seal(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(RuntimeError): run_phase_b(Path(d),Path(d),Path(d)/"gt.npz")
    def test_no_sweep_or_simulation_surface(self):
        source=Path(__file__).with_name("run_mx016_hospital_transfer.py").read_text()
        self.assertNotIn("tau-grid",source); self.assertNotIn("gazebo",source.lower()); self.assertNotIn("simulation.launch",source)

if __name__=="__main__": unittest.main()
