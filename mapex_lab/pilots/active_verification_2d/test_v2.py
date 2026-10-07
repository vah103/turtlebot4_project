"""V2 invariants: physical visibility, resource prefixes, error and topology."""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from .core import (GridWorld, PolicyInput, UNKNOWN, shortest_paths, array_hash,
                   clearance_free, reachable_component)
from .policy_v2 import (PredictedSensor, visible_targets, route_anchors, routes_from_views,
                        counterfactual_gain, choose_v2)
from .risk_model import fit_logistic, ErrorRiskModel, feature_grid
from .run_v2 import config


class ConstantRisk:
    def risk(self, state):
        return np.where(state.observed == UNKNOWN, .5, 0.)


class Numerics:
    def visibility(self, pose, mean, observed):
        return observed == UNKNOWN


class Tests(unittest.TestCase):
    def state(self, observed, mean, pose, budget=8.):
        return PolicyInput(observed.copy(), mean.copy(), np.zeros_like(mean),
                           np.repeat(mean[None], 3, axis=0), pose, .1, budget)

    def test_vectorized_scan_matches_world_first_hit(self):
        occupied = np.zeros((70, 70), dtype=bool)
        occupied[[0, -1], :] = True; occupied[:, [0, -1]] = True
        occupied[15:55, 38] = True
        world = GridWorld(occupied, .1, (35, 25), 4., 720, 0.)
        world.sense()
        mask = PredictedSensor(.1, 4., 720).visible(occupied, world.pose)
        self.assertTrue(np.array_equal(mask, world.observed != UNKNOWN))
        self.assertTrue(mask[35, 38])
        self.assertFalse(mask[35, 45])

    def test_target_rays_include_first_wall_and_block_cells_behind(self):
        occupied = np.zeros((50, 50), dtype=bool); occupied[:, 25] = True
        targets = np.asarray([(20, 24), (20, 25), (20, 26), (20, 40)])
        seen = visible_targets(occupied, (20, 10), targets, .1, 2.)
        self.assertEqual(seen.tolist(), [True, True, False, False])

    def test_route_has_budget_prefix_and_no_zero_cost_scan(self):
        observed = np.zeros((30, 30), dtype=np.float32)
        state = self.state(observed, observed, (10, 10), .5)
        _, parent = shortest_paths(np.ones_like(observed, bool), state.pose)
        cfg = config()
        routes = routes_from_views(state, [(10, 20), (10, 21)], parent, cfg)
        self.assertEqual(len(routes), 1)
        self.assertEqual(routes[0].goal, (10, 15))
        self.assertEqual(routes[0].cost_m, .5)
        self.assertEqual(routes[0].anchors, [(10, 15)])
        self.assertTrue(all(state.observed[p] == 0 for p in routes[0].path))

    def test_route_anchors_keep_endpoint_between_regular_samples(self):
        path = [(5, c) for c in range(5, 29)]
        self.assertEqual(route_anchors(path, 10), [path[10], path[20], path[-1]])

    def test_partial_first_return_can_correct_a_false_connection(self):
        free = np.zeros((90, 90), bool)
        free[10:80, 5:40] = True; free[10:80, 45:80] = True
        free[39:48, 40:45] = True
        seed = (43, 20)
        alternative = free.copy(); alternative[39:48, 40:45] = False
        partial = free.copy(); partial[39:48, 40] = False
        base = reachable_component(clearance_free(free, 1.5), seed)
        full = reachable_component(clearance_free(alternative, 1.5), seed)
        corrected = reachable_component(clearance_free(partial, 1.5), seed)
        self.assertEqual(counterfactual_gain(base, full, base, np.ones_like(free), .1), 0.)
        self.assertGreater(counterfactual_gain(base, full, corrected, np.ones_like(free), .1), 20.)

    def test_error_estimator_uses_prediction_features_and_preserves_known_cells(self):
        x = np.zeros((120, 6)); x[60:, 0] = 1
        y = np.r_[np.zeros(60), np.ones(60)]
        model = ErrorRiskModel(fit_logistic(x, y, np.ones(120), .01))
        self.assertGreater(model.predict_features(x[60:]).mean(), model.predict_features(x[:60]).mean())
        observed = np.full((20, 20), UNKNOWN, dtype=np.float32)
        mean = np.ones_like(observed); observed[10, 10] = 0; mean[10, 10] = 0
        state = self.state(observed, mean, (10, 10))
        self.assertEqual(model.risk(state)[10, 10], 0.)
        self.assertEqual(feature_grid(state).shape, (20, 20, 6))

    def test_new_policy_repeatable_and_does_not_mutate_input(self):
        mean = np.ones((80, 80), dtype=np.float32)
        mean[10:70, 5:35] = 0; mean[10:70, 42:75] = 0; mean[34:44, 35:42] = 0
        observed = np.full_like(mean, UNKNOWN); observed[15:65, 7:33] = mean[15:65, 7:33]
        state = self.state(observed, mean, (40, 20))
        before = array_hash(state.observed)
        a = choose_v2(state, "structural_v2", Numerics(), config(), ConstantRisk())
        b = choose_v2(state, "structural_v2", Numerics(), config(), ConstantRisk())
        self.assertEqual(a[0], b[0]); self.assertEqual(a[2], b[2])
        self.assertEqual(before, array_hash(state.observed))
        self.assertIsNotNone(a[0]); self.assertLessEqual(a[2]["path_m"], state.remaining_m+1e-8)
        self.assertEqual(a[2]["selected_route"], a[2]["candidate_records"][0])

    def test_full_v2_branch_executes_only_observed_paths_and_respects_budget(self):
        from . import run as runner
        class Predictor:
            calls = 0
            inference_s = 0.
            def predict(self, observed):
                mean = np.where(observed == UNKNOWN, .7, observed).astype(np.float32)
                return np.repeat(mean[None], 3, axis=0), mean, np.zeros_like(mean)
        occupied = np.zeros((80, 80), bool)
        occupied[[0, -1], :] = True; occupied[:, [0, -1]] = True
        cfg = config(); cfg.update(sensor_range_m=1., sensor_rays=360, branch_budget_m=.5)
        world = GridWorld(occupied, .1, (40, 40), 1., 360); world.sense()
        warm = dict(observed=world.observed, pose=world.pose, requested_distance_m=0., actual_distance_m=0.,
                    observed_hash=array_hash(world.observed))
        asset = dict(occupied=occupied, domain=np.ones_like(occupied), resolution=np.array(.1), start=np.array(world.pose))
        predictor = Predictor()
        chooser = lambda s, m, n, c: choose_v2(s, m, n, c, ConstantRisk())
        with tempfile.TemporaryDirectory() as directory, patch.object(runner, "choose", chooser):
            row = runner.run_branch("fixture__warm0", "structural_v2", asset, warm, predictor, Numerics(), cfg,
                                    predictor.predict(world.observed), "test", Path(directory))
            self.assertEqual(row["termination"], "DISTANCE_BUDGET")
            self.assertAlmostEqual(row["distance_m"], .5)
            self.assertEqual(row["collisions"], 0)


if __name__ == "__main__":
    unittest.main()
