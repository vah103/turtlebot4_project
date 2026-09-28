import math
import unittest
import numpy as np

from mapex_lab.analysis.d1.mx013_online_stop import OnlineDecision, RecognizerParams, replay_recognizer, assert_truth_free_api, online_masks


def d(i, r=0.03, evaluable=True, topo=True, u=True):
    return OnlineDecision("r", i, i, float(i), (i-1)/2, r, evaluable, "missing" if not evaluable else "",
                          topo, "bad" if not topo else "OK", topo, 1, 1.0, 1.0, 1.0,
                          low_u_25=u)


class RecognizerTests(unittest.TestCase):
    def test_truth_free_api(self): assert_truth_free_api()
    def test_threshold_equality_qualifies(self): self.assertTrue(replay_recognizer([d(1, .04)], RecognizerParams(.04, 1))[0]["first_fire"])
    def test_k_consecutive(self): self.assertEqual([x["first_fire"] for x in replay_recognizer([d(1),d(2)], RecognizerParams(.04,2))], [False,True])
    def test_reset_above(self): self.assertFalse(any(x["first_fire"] for x in replay_recognizer([d(1),d(2,.05),d(3)], RecognizerParams(.04,2))))
    def test_reset_missing_r(self): self.assertFalse(any(x["first_fire"] for x in replay_recognizer([d(1),d(2,math.nan,False),d(3)], RecognizerParams(.04,2))))
    def test_reset_topology(self): self.assertFalse(any(x["first_fire"] for x in replay_recognizer([d(1),d(2,topo=False),d(3)], RecognizerParams(.04,2))))
    def test_u_missing_no_primary_effect(self): self.assertTrue(replay_recognizer([d(1,u=None)], RecognizerParams(.04,1))[0]["first_fire"])
    def test_first_fire_only(self): self.assertEqual(sum(x["first_fire"] for x in replay_recognizer([d(1),d(2),d(3)], RecognizerParams(.04,1))),1)
    def test_u_veto_ablation_blocks_missing(self): self.assertFalse(replay_recognizer([d(1,u=None)], RecognizerParams(.04,1,use_u_veto=True))[0]["first_fire"])
    def test_frozen_k(self):
        with self.assertRaises(ValueError): replay_recognizer([d(1)], RecognizerParams(.04,5))
    def test_strict_occupancy_half_is_free(self):
        _, free=online_masks(np.array([[-1,-1]]),np.array([[.5,.50001]]),np.array([[1,1]],bool))
        self.assertEqual(free.tolist(),[[True,False]])
    def test_unsupported_unknown_nontraversable(self):
        domain,free=online_masks(np.array([[-1]]),np.array([[0.0]]),np.array([[0]],bool))
        self.assertFalse(domain[0,0]); self.assertFalse(free[0,0])


if __name__ == "__main__": unittest.main()
