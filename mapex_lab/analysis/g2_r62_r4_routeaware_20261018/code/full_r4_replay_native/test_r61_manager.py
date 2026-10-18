"""G2-R6.1 causal frozen-D02 + synthetic post-scan contract tests.
No real post-viewpoint LiDAR is implied by artificial test observations.
"""
import tempfile
import unittest
from pathlib import Path
import numpy as np

from trial_d02 import make, SRC
from r61_plan_manager import R61PlanManager
from r61_structural_events import S0,S1,S2,S3
from g2_r61_sidecar import R61Service

def initial_manager():
    p, obs, nav, sem = make(2)
    p.plan()
    return R61PlanManager(p,room_id=1,init_seq=12),obs,nav,sem

def req_d02():
    f="decision_000002"
    return {"action":"CHOOSE","room_id":1,"native_tick":12,
            "robot_core_rc":[147,111],
            "room_node_input":str(SRC/"room_r41/inference"/(f+"_input.npz")),
            "room_node_semantic":str(SRC/"room_r41/inference"/(f+"_semantic.npz")),
            "native_scope_npz":str(SRC/"g2_r4"/"native_scope_000002.npz")}

class ManagerContract(unittest.TestCase):
    def test_01_no_automatic_ack_before_arrival(self):
        m,obs,nav,sem=initial_manager()
        e=m.checkpoint(obs,m.monitor.pose,13,nav)
        self.assertEqual(e["action"],"WAIT_ARRIVAL")
        self.assertEqual(len(m.consumed),0)

    def test_02_old_proposal_rejected_after_next_scan(self):
        m,obs,nav,sem=initial_manager()
        m.on_scan(obs,(147,111),13,nav)
        r=m.propose_light()
        self.assertEqual(r["action"],"LIGHT_PROPOSAL")
        saved=m.cached_proposal
        m.on_scan(obs,(147,111),14,nav)
        rejected=m.try_commit(saved,obs,(147,111),nav,14)
        self.assertEqual(rejected["action"],"REJECT")
        self.assertEqual(rejected["reason"],"STALE_OBSERVATION_OR_ROOM")

    def test_03_native_safety_rejects_proposal(self):
        m,obs,nav,sem=initial_manager()
        m.on_scan(obs,(147,111),13,nav)
        m.propose_light()
        saved=m.cached_proposal
        r=m.try_commit(saved,obs,(147,111),nav,13,native_path_valid=False)
        self.assertEqual(r["action"],"REJECT")
        self.assertEqual(r["reason"],"NATIVE_ASTAR_REJECTED")

    def test_04_post_arrival_checkpoint_reconsiders_goals(self):
        m,obs,nav,sem=initial_manager()
        first=m.active_goals[0]
        e=m.checkpoint(obs,first,13,nav,source_goal=first)
        self.assertEqual(e["action"],"NEXT_GOAL")
        self.assertTrue(e["canceled_old_future_chain"])
        self.assertEqual(e["source"],"NEW_LIGHT_ROUTE")
        self.assertTrue(m.consumed)
        self.assertEqual(m.consumed[0][1],first)

    def test_05_immediate_S0_overrides_cooldown(self):
        m,obs,nav,sem=initial_manager()
        first=m.active_goals[0]
        e=m.checkpoint(obs,first,13,nav,native_path_valid=False)
        self.assertEqual(e["action"],"SAFE_HOLD")
        self.assertEqual(len(m.consumed),0)

    def test_06_unexpected_wall_blocks_old_future_chain(self):
        m,obs,nav,sem=initial_manager()
        pts=np.argwhere((obs==1)&m.r6.room&(m.r6.mean < .25))
        self.assertGreaterEqual(len(pts),40)
        nxt=obs.copy()
        for y,x in pts[:40]:
            nxt[y,x]=2
        event=m.on_scan(nxt,(147,111),13,nav)
        self.assertEqual(event["event"],S3)
        first=m.active_goals[0]
        verdict=m.checkpoint(nxt,first,13,nav,source_goal=first)
        self.assertEqual(verdict["action"],"WAIT_STRUCTURAL_REFRESH")
        self.assertEqual(len(m.consumed),0)

    def test_07_manual_light_replan_never_uses_gt(self):
        m,obs,nav,sem=initial_manager()
        m.on_scan(obs,(147,111),13,nav)
        p=m.propose_light()
        self.assertFalse(p["LaMa_rerun"])
        self.assertFalse(p["RoomNode_rerun"])
        self.assertTrue(len(p["goal_sequence_rc"])>0)
        for goal in p["goal_sequence_rc"]:
            self.assertTrue(nav[tuple(goal)])
            self.assertEqual(obs[tuple(goal)],0)

    def test_08_duplicate_arrival_no_second_goal_consumed(self):
        m,obs,nav,sem=initial_manager()
        first=m.active_goals[0]
        m.checkpoint(obs,first,13,nav,source_goal=first)
        consumed=len(m.consumed)
        verdict=m.checkpoint(obs,first,13,nav,source_goal=first)
        self.assertEqual(verdict["action"],"WAIT_ARRIVAL")
        self.assertEqual(consumed,len(m.consumed))


class SidecarContract(unittest.TestCase):
    def test_09_choose_emits_mandatory_checkpoint_contract(self):
        service=R61Service()
        answer=service.handle(req_d02())
        self.assertEqual(answer["event"],"RESULT")
        self.assertTrue(answer["mandatory_post_arrival_checkpoint"])
        self.assertEqual(answer["goal_sequence_core_rc"][0],answer["goal_core_rc"])
        self.assertTrue(answer["source_coherent_LaMa_RoomNode"])

    def test_10_sameepoch_update_and_checkpoint(self):
        service=R61Service()
        a=service.handle(req_d02())
        with tempfile.TemporaryDirectory() as td:
            root=Path(td)
            p,obs,nav,sem=make(2)
            np.savez_compressed(root/"obs.npz",observed=obs)
            np.savez_compressed(root/"nav.npz",reachable_knownfree=nav)
            req={"native_tick":13,"observed_npz":str(root/"obs.npz"),
                 "native_scope_npz":str(root/"nav.npz"),
                 "robot_core_rc":[147,111]}
            b=service.handle({"action":"UPDATE",**req})
            self.assertEqual(b["structural"]["event"],S1)
            self.assertFalse(b["need_fresh_coherent_LaMa_RoomNode"])
            # Arrival is a NEW scan epoch: the robot cannot teleport from
            # start pose to waypoint under the same scan id.
            c=service.handle({"action":"CHECKPOINT",**{**req,
                "native_tick":14, "robot_core_rc":a["goal_core_rc"]}})
            self.assertEqual(c["action"],"NEXT_GOAL")
            self.assertTrue(c["must_revalidate_native_ASTAR"])
            self.assertEqual(service.base.active.plan_order[0],tuple(c["goal_rc"]))

if __name__=="__main__":
    unittest.main(verbosity=2)
