"""G2-R7-R3 structural mission contracts (no GT, no native robot actuation)."""
import unittest
from pathlib import Path
import json,numpy as np
from r7_missions import R7Missions
from r7_dual_plan import R7DualPlan,inspect_contradiction

FROZEN="/home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz"
BASE="/home/dell/DELL_NATIVE2D/runs/NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41"


def initial():
    a=np.load(FROZEN)
    return (a["observed"].copy(),a["mean"].copy(),a["variance"].copy(),
            a["semantic"].copy(),tuple(map(int,a["robot_core_rc"])),
            a["native_walkable"].copy())


def after(index):
    a=np.load(f"{BASE}/inference/decision_{index:06d}_input.npz")
    s=np.load(f"{BASE}/inference/decision_{index:06d}_semantic.npz")
    meta=json.loads(Path(f"{BASE}/decisions/decision_{index:06d}.json").read_text())
    return (a["observed"].copy(),a["mean"].copy(),a["variance"].copy(),
            s["semantic"].copy(),tuple(int(x)-500 for x in meta["robot_rc"]),
            (a["observed"]==0))


class Tests(unittest.TestCase):
    def test_d02_two_distinct_structural_missions(self):
        p=R7Missions(*initial())
        v=p.plan("INITIAL")
        self.assertEqual(v["zones"],2)
        self.assertEqual(len(v["goals"]),2)
        self.assertEqual(v["tasks"][0]["purpose"],"SURVEY_PREDICTED_CHAMBER")
        self.assertEqual(v["tasks"][1]["purpose"],"VIEW_ENTRY_OBJECT_REAR")
        self.assertEqual(v["tasks"][1]["corner"],"bottom_left")
        self.assertFalse(v["uses_gt"])
        self.assertFalse(v["uses_future_lidar"])

    def test_goal2_is_on_backside_and_far_from_divider(self):
        p=R7Missions(*initial())
        v=p.plan()
        o=p.objects[0]
        goal=tuple(v["goals"][1])
        self.assertGreater(p._opposite(goal,o["center"],p.entry_anchor),0)
        self.assertLess(goal[1],p.portal[1]-p.cfg.inset_cells)
        self.assertGreater(goal[0],o["center"][0])
        self.assertTrue(p.geo.edt[goal]>=p.cfg.inset_cells)

    def test_d03_right_lower_then_upper_by_unseen_distribution(self):
        entry=initial()[4]
        p=R7Missions(*after(3),entry_anchor=entry)
        v=p.plan("SUCCESSOR")
        self.assertEqual(len(v["goals"]),2)
        self.assertEqual([s["corner"] for s in v["tasks"]],
                         ["bottom_right","top_right"])
        self.assertGreater(v["tasks"][0]["unknown_in_corner_half"],
                           v["tasks"][1]["unknown_in_corner_half"])
        self.assertFalse(any(s["movement_authorized"] for s in v["tasks"]))

    def test_d04_uses_persisted_entry_zone_not_current_zone(self):
        p=R7Missions(*after(4),entry_anchor=initial()[4])
        v=p.plan("SUCCESSOR")
        self.assertEqual([s["corner"] for s in v["tasks"]],
                         ["bottom_right","top_right"])

    def test_reflection_is_not_tuned_to_left_side_coordinates(self):
        raw=initial()
        obs,mean,var,sem,robot,native=raw
        w=obs.shape[1]
        reflected=(np.fliplr(obs),np.fliplr(mean),np.fliplr(var),
                   np.fliplr(sem),(robot[0],w-1-robot[1]),np.fliplr(native))
        orig=R7Missions(*raw).plan()
        back=R7Missions(*reflected).plan()
        self.assertEqual(len(back["goals"]),2)
        self.assertLessEqual(abs(back["goals"][0][0]-orig["goals"][0][0]),5)
        self.assertLessEqual(abs(back["goals"][0][1]-(w-1-orig["goals"][0][1])),5)
        self.assertLessEqual(abs(back["goals"][1][1]-(w-1-orig["goals"][1][1])),5)

    def test_empty_room_does_not_force_four_goals(self):
        shape=(70,95)
        obs=np.full(shape,2,np.uint8)
        obs[7:64,8:88]=0
        sem=np.zeros(shape,np.uint8)
        sem[7:64,8:88]=1
        mean=np.where(obs==2,1,0).astype(np.float32)
        var=np.zeros_like(mean)
        p=R7Missions(obs,mean,var,sem,(20,32),obs==0)
        v=p.plan()
        self.assertEqual(v["goals"],[])

    def test_native_routes_are_advisory_not_authorized(self):
        x=R7Missions(*initial()).plan()
        self.assertTrue(all(s["native_inflated_astar_required"] for s in x["tasks"]))
        self.assertTrue(all(s["movement_authorized"] is False for s in x["tasks"]))
        self.assertTrue(x["native_control_unmodified"])

    def test_contradiction_requires_previous_semantic_label(self):
        o,m,_,sem,_,_=initial()
        now,_,_,now_sem,_,_=after(3)
        e=inspect_contradiction(o,m,now,sem)
        self.assertTrue(e["has_contradiction"])
        self.assertGreater(e["unexpected_obstacle_cells"],0)
        e_wrong=inspect_contradiction(o,m,now,now_sem)
        self.assertLess(e_wrong["unexpected_obstacle_cells"],
                        e["unexpected_obstacle_cells"])

    def test_dual_plan_concurrent_preparation_preserves_goal2(self):
        obs,mean,var,sem,robot,native=initial()
        p=R7Missions(obs,mean,var,sem,robot,native)
        v=p.plan()
        mgr=R7DualPlan(entry_anchor=robot,room_id=1)
        try:
            out=mgr.initialize(v,obs,mean,sem,12)
            self.assertEqual(out["action"],"PLAN1_COMMITTED_INTENT")
            self.assertEqual(mgr.advance_on_arrival(v["goals"][0],obs,native)["action"],
                             "GOAL_ARRIVAL_CONFIRMED")
            self.assertEqual(mgr.active,[tuple(v["goals"][1])])
            ob,me,va,se,_,na=after(3)
            e=mgr.ingest_scan(ob,me,va,se,na,26,1,1)
            self.assertEqual(e["action"],
                             "PLAN2_PREPARATION_STARTED_IN_BACKGROUND")
            self.assertEqual(mgr.active,[tuple(v["goals"][1])])
            result=mgr.pending.result(timeout=15)
            self.assertEqual([s["corner"] for s in result["tasks"]],
                             ["bottom_right","top_right"])
            self.assertEqual(mgr.poll()["action"],"PLAN2_READY")
            hold=mgr.checkpoint(v["goals"][1],ob,me,va,se,na,27,1,1,
                                route_safe=False,local_wfd_exists=True)
            self.assertEqual(hold["action"],"STOP_AND_NATIVE_REPLAN")
            self.assertTrue(mgr.active)
        finally:mgr.close()

    def test_no_hardcoded_d02_goals_in_planner_source(self):
        content=Path(__file__).with_name("r7_missions.py").read_text()
        for q in ("(169,132)","(169, 132)","(196,96)","(196, 96)",
                  "(196,206)","(196, 206)","(148,206)","(148, 206)"):
            self.assertNotIn(q,content)


if __name__=="__main__":
    unittest.main(verbosity=2)
