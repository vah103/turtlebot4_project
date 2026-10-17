import unittest
from pathlib import Path

from g2_r6_sidecar import Service

BASE = Path("/home/dell/mx072_g2_r4_structural_predicted_run11_20261017/"
            "native_g2_r4_structural_predicted_run11/"
            "G2_R4_STRUCTURAL_PREDICTED_RUN11_visvarprob")


def snapshot(i=2):
    return {"action": "CHOOSE", "native_tick": {2:12, 3:28, 4:52}[i],
            "room_id": 1, "robot_core_rc": [147, 111] if i == 2 else
            ([171, 135] if i == 3 else [179, 103]),
            "room_node_input": str(BASE/"room_r41/inference"/f"decision_{i:06d}_input.npz"),
            "room_node_semantic": str(BASE/"room_r41/inference"/f"decision_{i:06d}_semantic.npz"),
            "native_scope_npz": str(BASE/"g2_r4"/f"native_scope_{i:06d}.npz")}


class SidecarTest(unittest.TestCase):
    def test_choose_and_no_false_goal_ack(self):
        s = Service()
        a = s.handle(snapshot())
        self.assertEqual(a["event"], "RESULT")
        self.assertEqual(a["decision"], "GOAL")
        self.assertEqual(a["plan_epoch_count"], 1)
        self.assertTrue(a["requires_native_ASTAR_validation"])
        self.assertFalse(s.handle({"action":"ACK_GOAL","robot_core_rc":[147,111]})["accepted"])
        ack = s.handle({"action":"ACK_GOAL","robot_core_rc":a["goal_core_rc"]})
        self.assertTrue(ack["accepted"])
        self.assertEqual(len(ack["remaining_goals_rc"]), len(a["goal_sequence_core_rc"])-1)

    def test_same_map_update_reuses_lama_epoch(self):
        s = Service()
        a = s.handle(snapshot())
        upd = s.handle({"action":"UPDATE", "native_tick": 12,
                        "observed_npz": snapshot()["room_node_input"],
                        "native_scope_npz": snapshot()["native_scope_npz"],
                        "semantic_npz": snapshot()["room_node_semantic"],
                        "robot_core_rc": [147,111]})
        self.assertEqual(upd["event"], "UPDATE")
        self.assertIn(upd["fast"]["action"], ("KEEP","PRUNE_TAIL"))
        self.assertFalse(upd["refine"]["LaMa_inference_called"])
        self.assertEqual(s.handle({"action":"STATUS"})["plan_epochs"], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)