"""Scientific invariants: sensor occlusion, physics, truth separation, topology."""
from __future__ import annotations

from dataclasses import fields
import json
from pathlib import Path
import unittest

import numpy as np

from .core import (GridWorld, PolicyInput, UNKNOWN, observed_traversable, shortest_paths,
                   recover_path, frontier_representatives, evaluate, array_hash,
                   MapExNumerics)
from .policy import hypotheses, choose


class Tests(unittest.TestCase):
    def cfg(self):
        return json.loads(Path(__file__).with_name("protocol.json").read_text())

    def room(self, n=45):
        occupied = np.zeros((n, n), dtype=bool)
        occupied[[0, -1], :] = True
        occupied[:, [0, -1]] = True
        return occupied

    def state(self, observed, mean, pose):
        predictions = np.repeat(mean[None, :, :], 3, axis=0)
        return PolicyInput(observed.copy(), mean.copy(), np.zeros_like(mean), predictions,
                            pose, 0.1, 8.0)

    def test_sensor_does_not_see_through_wall(self):
        occupied = self.room()
        occupied[:, 20] = True
        world = GridWorld(occupied, .1, (20, 10), sensor_range_m=10, sensor_rays=720)
        world.sense()
        self.assertEqual(world.observed[20, 19], 0)
        self.assertEqual(world.observed[20, 20], 1)
        self.assertTrue(np.all(world.observed[:, 21:] == UNKNOWN))

    def test_sensing_agrees_with_truth_and_is_repeatable(self):
        occupied = self.room()
        world = GridWorld(occupied, .1, (20, 10), sensor_range_m=1, sensor_rays=720)
        world.sense()
        known = world.observed != UNKNOWN
        self.assertTrue(np.array_equal(world.observed[known], occupied[known].astype(np.float32)))
        self.assertFalse(world.sense().any())
        self.assertEqual(world.observed[20, 35], UNKNOWN)

    def test_teleportation_forbidden_and_motion_cost_exact(self):
        world = GridWorld(self.room(), .1, (20, 10))
        with self.assertRaises(ValueError):
            world.step((20, 20))
        self.assertTrue(world.step((20, 11)))
        self.assertAlmostEqual(world.distance_m, .1)

    def test_footprint_collision_is_reported(self):
        occupied = self.room()
        occupied[:, 20] = True
        world = GridWorld(occupied, .1, (20, 18), robot_radius_m=.15)
        self.assertFalse(world.step((20, 19)))
        self.assertEqual(world.collisions, 1)
        self.assertEqual(world.distance_m, 0)

    def test_planning_never_traverses_unknown(self):
        observed = np.full((12, 12), UNKNOWN)
        observed[2:10, 2:5] = 0
        traversable = observed_traversable(observed, 0)
        dist, parent = shortest_paths(traversable, (3, 3))
        self.assertEqual(dist[3, 8], -1)
        self.assertIsNone(recover_path(parent, (3, 3), (3, 8)))
        path = recover_path(parent, (3, 3), (8, 4))
        self.assertEqual(len(path)-1, 6)
        self.assertTrue(all(observed[p] == 0 for p in path))

    def test_policy_input_contains_no_truth_and_is_readonly(self):
        observed = np.zeros((6, 6), dtype=np.float32)
        state = self.state(observed, observed, (3, 3))
        self.assertEqual({f.name for f in fields(state)}, {"observed", "mean", "variance", "predictions", "pose", "resolution", "remaining_m"})
        with self.assertRaises(ValueError):
            state.observed[0, 0] = 1

    def test_predicted_map_cannot_overwrite_observation(self):
        observed = np.zeros((6, 6), dtype=np.float32)
        mean = observed.copy()
        mean[1, 1] = 1
        with self.assertRaises(ValueError):
            self.state(observed, mean, (3, 3))

    def test_frontier_threshold_is_strictly_greater_than_ten(self):
        observed = np.ones((20, 20), dtype=np.float32)
        observed[2:12, 3] = 0
        observed[2:12, 4] = UNKNOWN
        self.assertEqual(frontier_representatives(observed, 10), [])
        observed[12, 3], observed[12, 4] = 0, UNKNOWN
        self.assertEqual(len(frontier_representatives(observed, 10)), 1)

    def test_closing_bridge_changes_structural_metric(self):
        occupied = self.room()
        occupied[:, 22] = True
        occupied[18:25, 22] = False
        mean = occupied.astype(np.float32)
        observed = np.full_like(mean, UNKNOWN)
        observed[5:35, 2:15] = mean[5:35, 2:15]
        domain = np.ones_like(occupied)
        good = evaluate(observed, mean, occupied, domain, (20, 10), .1)
        wrong = mean.copy()
        wrong[18:25, 22] = 1
        bad = evaluate(observed, wrong, occupied, domain, (20, 10), .1)
        self.assertEqual(good["reachable_mismatch_m2"], 0)
        self.assertGreater(bad["missed_reachable_free_m2"], 5)

    def test_hypotheses_do_not_modify_known_cells_or_depend_on_disagreement(self):
        mean = np.ones((90, 90), dtype=np.float32)
        mean[15:75, 8:35] = 0
        mean[15:75, 43:75] = 0
        mean[42:49, 35:43] = 0
        observed = np.full_like(mean, UNKNOWN)
        observed[20:70, 10:30] = mean[20:70, 10:30]
        state = self.state(observed, mean, (45, 20))
        before = array_hash(state.observed)
        hs, _ = hypotheses(state, self.cfg())
        self.assertTrue(hs)
        self.assertEqual(before, array_hash(state.observed))
        self.assertTrue(all(np.all(state.observed[h.patch_cells[:, 0], h.patch_cells[:, 1]] == UNKNOWN) for h in hs))
        self.assertTrue(all(h.ensemble_agreement == 1 for h in hs))

    def test_policy_reproducible_without_world(self):
        class Numerics:
            def visibility(self, pose, mean, observed):
                return observed == UNKNOWN
        mean = np.ones((60, 60), dtype=np.float32)
        mean[10:50, 5:25] = 0
        mean[10:50, 32:55] = 0
        mean[25:34, 25:32] = 0
        observed = np.full_like(mean, UNKNOWN)
        observed[12:48, 7:24] = mean[12:48, 7:24]
        state = self.state(observed, mean, (30, 15))
        a = choose(state, "structural", Numerics(), self.cfg())
        b = choose(state, "structural", Numerics(), self.cfg())
        self.assertEqual(a[0], b[0])
        self.assertEqual(a[2], b[2])

    def test_ast_reuses_exact_numerics_without_importing_ros(self):
        import importlib.util
        if importlib.util.find_spec("shapely") is None:
            self.skipTest("Shapely unavailable in this test environment")
        source = Path(__file__).resolve().parents[2] / "scripts/mapex.py"
        numerical = MapExNumerics(source, .1, 2.0, 50)
        mean = np.zeros((60, 60), dtype=np.float32)
        mean[30, :] = 1
        observed = np.full_like(mean, UNKNOWN)
        observed[10, 20] = 0
        mask = numerical.visibility((10, 20), mean, observed)
        self.assertEqual(mask.shape, mean.shape)
        self.assertFalse(mask[10, 20])
        self.assertFalse(mask[40, 20])


if __name__ == "__main__":
    unittest.main()
