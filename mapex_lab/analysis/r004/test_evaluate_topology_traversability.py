import math
import json
import unittest
from pathlib import Path

import numpy as np
import yaml

from mapex_lab.analysis.r004 import evaluate_topology_traversability as t


class TopologyTraversabilityTests(unittest.TestCase):
    def test_strict_prediction_threshold(self):
        occupied = t.strict_occupied([0.5, np.nextafter(0.5, 1.0)])
        self.assertEqual(occupied.tolist(), [False, True])

    def test_frozen_radius_and_resolution(self):
        self.assertEqual(t.RADIUS_M, 0.189)
        self.assertEqual(t.RESOLUTION_M, 0.05)
        self.assertEqual(t.footprint_radius(t.CANONICAL_FOOTPRINT), 0.189)

    def test_resolution_mismatch_is_blocker(self):
        t.validate_resolution(0.05)
        with self.assertRaisesRegex(ValueError, "canonical resolution"):
            t.validate_resolution(0.06)

    def test_common_domain_and_observed_identity(self):
        observed = np.array([[0, 1, -1, -1]])
        final = np.array([[0, 1, 0, 0]])
        prediction = np.array([[0.9, 0.1, 0.1, 0.9]])
        support = np.array([[False, False, True, False]])
        known, future, scoreable, domain, _, reference, predicted, _ = t.completed_masks(
            observed, prediction, final, support)
        np.testing.assert_array_equal(domain, known | scoreable)
        np.testing.assert_array_equal(reference[known], predicted[known])
        self.assertTrue(future[0, 3])
        self.assertFalse(domain[0, 3])

    def test_occupied_clearance_accepts_ros_occupancy_values(self):
        final = np.array([[-1, 0, 100]])
        clearance = t.occupied_clearance(final)
        self.assertEqual(clearance[0, 2], 0.0)
        self.assertEqual(clearance[0, 1], 1.0)

    def test_collision_stencil_axial_and_diagonal(self):
        stencil = t.collision_stencil()
        center = stencil.shape[0] // 2
        self.assertTrue(stencil[center, center + 4])
        self.assertFalse(stencil[center, center + 5])
        self.assertTrue(stencil[center + 3, center + 3])
        self.assertFalse(stencil[center + 4, center + 2])

    def test_collision_stencil_touching_counts_as_collision(self):
        radius = math.hypot(0.125, 0.125)
        stencil = t.collision_stencil(radius=radius, resolution=0.05)
        center = stencil.shape[0] // 2
        self.assertTrue(stencil[center + 3, center + 3])

    def test_outside_domain_is_collision(self):
        mask = np.ones((21, 21), dtype=bool)
        result = t.cspace(mask, mask, t.collision_stencil())
        self.assertFalse(result[0, 0])
        self.assertTrue(result[10, 10])

    def test_unsupported_cells_cannot_be_traversed(self):
        domain = np.ones((21, 21), dtype=bool)
        domain[:, 10] = False
        result = t.cspace(domain, domain, np.ones((1, 1), dtype=bool))
        self.assertFalse(result[:, 10].any())
        self.assertFalse(t.reachable(result, (10, 2))[10, 18])

    def test_no_corner_cutting(self):
        mask = np.array([[True, False], [False, True]])
        self.assertFalse(t.valid_move(mask, 0, 0, 1, 1))
        self.assertFalse(t.reachable(mask, (0, 0))[1, 1])

    def test_cardinal_and_clear_diagonal_moves(self):
        mask = np.ones((2, 2), dtype=bool)
        self.assertTrue(t.valid_move(mask, 0, 0, 0, 1))
        self.assertTrue(t.valid_move(mask, 0, 0, 1, 1))

    def test_component_order_is_deterministic(self):
        mask = np.zeros((4, 5), dtype=bool)
        mask[0, 4] = True
        mask[3, 0:2] = True
        labels, records = t.components(mask)
        self.assertEqual([record["cells"][0] for record in records], [(0, 4), (3, 0)])
        self.assertEqual((labels[0, 4], labels[3, 0]), (1, 2))

    def test_pair_retention_keeps_prediction_occupied_in_denominator(self):
        self.assertAlmostEqual(t.pair_retention([2, 1], 4), 1 / 6)
        self.assertTrue(math.isnan(t.pair_retention([1], 1)))

    def test_largest_piece_fraction(self):
        self.assertAlmostEqual(t.largest_piece_fraction([4, 2], 8), 0.5)

    def test_component_inventory_ranking(self):
        labels = np.array([[2, 2, 0], [1, 0, 3]])
        source = labels > 0
        records = t.component_inventory(labels, source, (5, 7))
        self.assertEqual([record["label"] for record in records], [2, 1, 3])
        self.assertEqual(records[0]["minimum_cell"], [5, 7])
        json.dumps([{key: value for key, value in record.items() if key != "local_cells"}
                    for record in records])

    def test_representative_prefers_clearance_then_row_col(self):
        clearance = np.array([[1.0, 3.0], [3.0, 2.0]])
        self.assertEqual(t.representative([(0, 0), (0, 1), (1, 0)], clearance), (0, 1))

    def test_shortest_path_is_repeatable(self):
        mask = np.ones((4, 4), dtype=bool)
        first = t.shortest_path(mask, (0, 0), (3, 2))
        self.assertEqual(first, t.shortest_path(mask, (0, 0), (3, 2)))
        self.assertEqual((first[0], first[-1]), ((0, 0), (3, 2)))

    def test_world_to_cell_uses_floor(self):
        self.assertEqual(t.world_to_cell(-0.001, 0.099, 0.0, 0.0, 0.05), (1, -1))

    def test_source_exact_alignment(self):
        source = t.align_source(1.0, [(0.0, 0.0, 0.0, 0.0), (1.0, 2.0, 4.0, 0.5)])
        self.assertTrue(source["available"])
        self.assertEqual(source["mode"], "exact")
        self.assertEqual((source["x"], source["y"]), (2.0, 4.0))

    def test_source_strict_bracket_interpolation(self):
        source = t.align_source(1.0, [(0.0, 0.0, 0.0, 0.0), (2.0, 4.0, 8.0, 1.0)])
        self.assertEqual(source["mode"], "interpolated")
        self.assertEqual((source["x"], source["y"], source["yaw"]), (2.0, 4.0, 0.5))

    def test_source_never_extrapolates(self):
        trajectory = [(1.0, 0.0, 0.0, 0.0), (2.0, 1.0, 1.0, 0.0)]
        self.assertFalse(t.align_source(0.5, trajectory)["available"])
        self.assertFalse(t.align_source(2.5, trajectory)["available"])

    def test_domain_crop_preserves_global_offset(self):
        domain = np.zeros((5, 6), dtype=bool)
        domain[2:4, 3:6] = True
        offset, (cropped,) = t.crop_domain(domain)
        self.assertEqual(offset, (2, 3))
        self.assertEqual(cropped.shape, (2, 3))

    def test_reference_corridor_survives_identity_stencil(self):
        mask = np.zeros((5, 7), dtype=bool)
        mask[2, 1:6] = True
        traversable = t.cspace(mask, mask, np.ones((1, 1), dtype=bool))
        self.assertTrue(t.reachable(traversable, (2, 1))[2, 5])

    def test_benign_wall_thickening_does_not_disconnect(self):
        prediction = np.ones((7, 9), dtype=bool)
        prediction[0:2, :] = False
        self.assertTrue(t.reachable(prediction, (4, 1))[4, 7])

    def test_room_door_room_closure_disconnects(self):
        reference = np.ones((7, 11), dtype=bool)
        reference[:, 5] = False
        reference[3, 5] = True
        prediction = reference.copy()
        prediction[3, 5] = False
        self.assertTrue(t.reachable(reference, (3, 2))[3, 8])
        self.assertFalse(t.reachable(prediction, (3, 2))[3, 8])

    def test_predicted_barrier_fragments_reference_component(self):
        reference = np.ones((5, 7), dtype=bool)
        prediction = reference.copy()
        prediction[:, 3] = False
        source_set = t.reachable(reference, (2, 1))
        labels, _ = t.components(prediction)
        inventory = t.component_inventory(labels, source_set, (0, 0))
        self.assertEqual(len(inventory), 2)
        self.assertLess(t.pair_retention([row["size"] for row in inventory], int(source_set.sum())), 1.0)

    def test_historical_footprints_match_frozen_provenance(self):
        root = Path(__file__).resolve().parents[3]
        for run_id in t.base.RUNS:
            path = root / "mapex_lab/experiments/mapex" / run_id / "runtime_nav2_merged.yaml"
            config = yaml.safe_load(path.read_text(encoding="utf-8"))
            footprints = [t.parse_footprint(value) for value in t._collect_key(config, "footprint")]
            self.assertTrue(footprints)
            self.assertTrue(all(value == t.CANONICAL_FOOTPRINT for value in footprints))


if __name__ == "__main__":
    unittest.main()
