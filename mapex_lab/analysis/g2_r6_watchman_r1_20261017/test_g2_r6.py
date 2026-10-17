import unittest
import numpy as np

from g2_r6_watchman import RoomWatchman, R6Config
from trial_d02 import make


class SyntheticR6Tests(unittest.TestCase):
    def setUp(self):
        obs = np.zeros((45, 55), np.uint8)
        obs[0, :] = obs[-1, :] = 2
        obs[:, 0] = obs[:, -1] = 2
        obs[12:31, 35:53] = 1
        obs[22:26, 34] = 2
        mean = np.full(obs.shape, .15, np.float32)
        var = np.full(obs.shape, .01, np.float32)
        sem = np.ones(obs.shape, np.uint8)
        nav = (obs == 0).astype(np.uint8)
        self.p = RoomWatchman(obs, mean, var, sem, (22, 12), nav,
                              R6Config(max_goals=4, max_candidates=15))

    def test_goals_routes_never_unknown(self):
        result = self.p.plan()
        self.assertEqual(result["status"], "CONDITIONAL_SAFE_ROUTE")
        self.assertTrue(result["goals_only_native_known_free"])
        for p in result["planned_grid_route_core_rc"]:
            self.assertEqual(int(self.p.obs[tuple(p)]), 0)
        self.assertTrue(1 <= len(result["goal_sequence_core_rc"]) <= 4)

    def test_fast_scan_is_no_lama(self):
        result = self.p.plan()
        new = self.p.obs.copy()
        new[12:14, 35:39] = 0
        update = self.p.scan_update(new, self.p.robot)
        self.assertIn(update["action"], ("KEEP", "PRUNE_TAIL"))
        self.assertFalse(update["lama_inference_called"])
        self.assertEqual(update["new_observed_cells"], 8)
        self.assertEqual(self.p.prediction_sha256, result["LaMa_mean_sha256"])

    def test_known_wall_on_planned_path_causes_replan(self):
        result = self.p.plan()
        path = result["planned_grid_route_core_rc"]
        self.assertGreater(len(path), 4)
        new = self.p.obs.copy()
        new[tuple(path[3])] = 2
        update = self.p.scan_update(new, self.p.robot)
        self.assertEqual(update["action"], "REPLAN_REQUIRED")
        self.assertIn(update["reason"], ("GOAL_OR_ROUTE_INVALIDATED", "COMMITTED_PATH_BLOCKED"))

    def test_refine_keeps_first_goal(self):
        result = self.p.plan()
        refined = self.p.refine_remaining(self.p.robot)
        self.assertIn(refined["action"], ("KEEP", "PROPOSE_REVISED_ROUTE"))
        self.assertTrue(refined["first_goal_unchanged"])
        self.assertFalse(refined["LaMa_inference_called"])
        self.assertEqual(refined["goal_sequence_core_rc"][0],
                         result["goal_sequence_core_rc"][0])


class FrozenD02Tests(unittest.TestCase):
    def test_native_scope_and_runtime(self):
        model, _, native, _ = make(2)
        result = model.plan()
        self.assertFalse(result["used_gt"])
        self.assertFalse(result["used_future_observations"])
        self.assertTrue(result["goals_only_native_known_free"])
        for p in result["planned_grid_route_core_rc"]:
            self.assertTrue(native[tuple(p)])
            self.assertEqual(int(model.obs[tuple(p)]), 0)
        self.assertEqual(result["mission_unknown"], 3889)
        self.assertLess(result["extra_G2_planning_wall_s"], 3.0)

    def test_deterministic_proposal(self):
        a, _, _, _ = make(2)
        b, _, _, _ = make(2)
        p, q = a.plan(), b.plan()
        self.assertEqual(p["goal_sequence_core_rc"], q["goal_sequence_core_rc"])
        self.assertEqual(p["input_observed_sha256"], q["input_observed_sha256"])


if __name__ == "__main__":
    unittest.main(verbosity=2)