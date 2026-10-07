"""Resource-feasibility regression; no research result uses this fixture."""
from __future__ import annotations

import json
from pathlib import Path
import unittest
import numpy as np

from .core import PolicyInput
from .budget_ablation import choose_budget_feasible


class BudgetTests(unittest.TestCase):
    def test_verification_goal_fits_remaining_budget(self):
        mean = np.ones((90, 90), dtype=np.float32)
        mean[15:75, 8:35] = 0
        mean[15:75, 43:75] = 0
        mean[42:49, 35:43] = 0
        observed = np.full_like(mean, .5)
        observed[20:70, 10:30] = mean[20:70, 10:30]
        state = PolicyInput(observed.copy(), mean.copy(), np.zeros_like(mean),
                            np.repeat(mean[None], 3, axis=0), (45, 20), .1, .3)
        cfg = json.loads(Path(__file__).with_name("protocol.json").read_text())
        class Numerics:
            def visibility(self, pose, mean, observed):
                return observed == .5
        goal, parent, detail = choose_budget_feasible(state, "structural", Numerics(), cfg)
        if detail["action"] == "VERIFY":
            self.assertLessEqual(detail["path_m"], state.remaining_m+1e-8)
        self.assertEqual(detail["variant"], "BUDGET_FEASIBILITY_ONLY")


if __name__ == "__main__":
    unittest.main()
