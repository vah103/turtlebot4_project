"""A fake RoomRun only for native adaptor contract smoke testing.
No MapEx physics, no LaMa inference, no future snapshots, no GT.
"""
import tempfile
import unittest
import numpy as np
from pathlib import Path

from g2_r6_watchman import room_connected, _KnownFreePaths
# DELL test environment lacks pyastar2d. This stub is test-only; COM1 native
# must use the original installed pyastar2d and retain actual A* validation.
import sys, types
try:
    import pyastar2d
except ModuleNotFoundError:
    def test_only_astar_path(grid, start, goal, allow_diagonal=False):
        legal = np.isfinite(grid)
        route = _KnownFreePaths(legal).path(tuple(start), tuple(goal))
        return np.asarray(route, dtype=np.int32) if route is not None else None
    sys.modules['pyastar2d'] = types.SimpleNamespace(astar_path=test_only_astar_path)
from g2_r6_native_integration import G2Run
from trial_d02 import make, SRC


class FrozenFakeRoomRun:
    def __init__(self):
        m, obs, scope, semantic = make(2)
        self.active_mask = room_connected(obs, semantic, m.robot)
        self.last_scope = {"mode": "LOCAL", "room_id": 1}
        self.epoch = 2
        self.inferdir = str(SRC / "room_r41/inference")
        h, w = obs.shape
        self.scope_grid = np.full((h+1000,w+1000), np.inf, np.float32)
        self.scope_grid[500:500+h,500:500+w][scope.astype(bool)] = 1.
        self.obs = obs
        self.native_scope = scope

    def grid(self, _):
        return self.scope_grid


class AdapterTest(unittest.TestCase):
    def test_choose_and_advance_without_lama(self):
        fake = FrozenFakeRoomRun()
        with tempfile.TemporaryDirectory(prefix="r6_native_contract_") as root:
            service = G2Run(root)
            try:
                start = np.asarray([147,111])+500
                first = service.choose(12,fake,start)
                self.assertIsNotNone(first)
                self.assertTrue(fake.native_scope[tuple(first-500)])
                self.assertTrue(service.local_lock)
                second = service.cached_next_goal(15,fake,first, fake.scope_grid)
                self.assertIsNotNone(second)
                self.assertFalse(np.array_equal(first,second))
                self.assertTrue(service.local_lock)
                self.assertTrue(fake.native_scope[tuple(second-500)])
            finally:
                service.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)