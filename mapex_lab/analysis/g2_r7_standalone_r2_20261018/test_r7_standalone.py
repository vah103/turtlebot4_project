import unittest
import numpy as np
import cv2
from r7_standalone import R7Standalone, RoomStructure, Sight, R7Settings
from r7_plan_manager import R7DualPlanManager

FROZEN="/home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz"


def d02():
    a=np.load(FROZEN)
    return (a["observed"].copy(),a["mean"].copy(),a["variance"].copy(),
            a["semantic"].copy(),tuple(int(v) for v in a["robot_core_rc"]),
            a["native_walkable"].astype(bool).copy())


def synthetic():
    h,w=74,104
    obs=np.full((h,w),2,np.uint8)
    obs[6:68,6:98]=0
    obs[8:33,48:51]=2
    obs[41:65,48:51]=2
    obs[36:41,48:51]=0
    obs[20:25,25:30]=2
    obs[10:65,55:97]=1
    # A plausible structure predicted FREE to the right, no future walls.
    mean=np.zeros((h,w),np.float32)
    mean[obs==2]=1
    var=np.zeros_like(mean)
    sem=np.ones_like(obs,np.uint8)
    sem[obs==2]=4
    sem[20:25,25:30]=3
    robot=(18,24)
    legal=(obs==0)
    legal=cv2.erode(legal.astype("uint8"),
                    np.ones((3,3),np.uint8)).astype(bool)
    return obs,mean,var,sem,robot,legal


class R7R2Tests(unittest.TestCase):
    def test_d02_chambers_and_portal_derived_not_hardcoded(self):
        obs,mean,var,sem,robot,native=d02()
        r=R7Standalone(obs,mean,var,sem,robot,native)
        self.assertEqual(len(r.geo.zone_ids),2)
        self.assertTrue(r.geo.portal_candidates)
        for c in r.geo.portal_candidates:
            self.assertTrue(r.geo.predfree[c])
        self.assertGreaterEqual(len(r.geo.obstacles),1)

    def test_d02_inferred_opposing_viewpoints_and_determinism(self):
        payload=d02()
        a=R7Standalone(*payload);r=a.plan()
        b=R7Standalone(*payload);s=b.plan()
        self.assertEqual(r["goals_rc"],s["goals_rc"])
        self.assertEqual(r["input_obs_sha256"],s["input_obs_sha256"])
        self.assertGreaterEqual(len(r["goals_rc"]),2)
        self.assertTrue(all(leg["requires_native_astar"] for leg in r["legs"]))
        self.assertTrue(r["uses_GT"] is False and r["uses_future_observation"] is False)
        self.assertGreater(r["structural_quality"],.8)

    def test_direction_is_not_merely_unknown_gain(self):
        obs,mean,var,sem,robot,native=d02()
        p=R7Standalone(obs,mean,var,sem,robot,native)
        a=p.sight.at(robot)
        c=p.sight.at((187,103))
        self.assertTrue(len(c)==3)
        self.assertTrue(any(x>0 for x in p.sight.angle_fractions(
            tuple(a[i]|c[i] for i in range(3))).values()))

    def test_virtual_goal_never_becomes_executable_without_lidar(self):
        obs,mean,var,sem,robot,native=synthetic()
        p=R7Standalone(obs,mean,var,sem,robot,native)
        r=p.plan()
        self.assertTrue(all(leg["executable_as_is"] is False or
                            (obs[tuple(leg["goal_rc"])]==0 and
                             native[tuple(leg["goal_rc"])])
                            for leg in r["legs"]))
        prefix=r["first_goal_observed_free_prefix_rc"]
        self.assertTrue(all(obs[tuple(q)]==0 and native[tuple(q)] for q in prefix))
        self.assertTrue(r["native_known_free_Astar_required"])

    def test_surface_view_in_synthetic(self):
        obs,mean,var,sem,robot,native=synthetic()
        p=R7Standalone(obs,mean,var,sem,robot,native)
        before=p.sight.at((14,24))
        after=p.sight.at((53,24))
        merged=tuple(before[i]|after[i] for i in range(3))
        qa=p.sight.angle_fractions(merged)
        self.assertGreater(sum(qa.values()),sum(p.sight.angle_fractions(before).values()))

    def test_no_d02_hardcoded_coordinate_in_source(self):
        from pathlib import Path
        s=Path(__file__).with_name("r7_standalone.py").read_text()
        self.assertNotIn("(169, 132)",s)
        self.assertNotIn("(168, 133)",s)
        self.assertNotIn("(199, 129)",s)

    def test_manager_maintains_active_plan_and_vetoes_unsafe(self):
        obs,mean,var,sem,robot,native=synthetic()
        planner=R7Standalone(obs,mean,var,sem,robot,native)
        initial=planner.plan()
        manager=R7DualPlanManager(room_id=1)
        try:
            ans=manager.install_initial(initial,scan_tick=0,structural_epoch=1)
            self.assertEqual(ans["action"],"PLAN1_INTENT_INSTALLED")
            self.assertIsNone(ans["robot_command"])
            current=tuple(manager.active[0])
            # After new LiDAR, a synthetic right-side obstacle is discovered.
            newer=obs.copy()
            newer[43:48,75:80]=2
            newer_mean=mean.copy()
            newer_mean[newer==2]=1.
            newer_sem=sem.copy()
            newer_sem[newer==2]=4
            result=manager.propose_successor_async(
                newer,newer_mean,var,newer_sem,native,scan_tick=10,
                structural_epoch=2,room_id=1,active_goal=current,
                structural_contradiction=True)
            self.assertEqual(result["action"],"PLAN2_BACKGROUND_STARTED")
            self.assertEqual(tuple(manager.active[0]),current)
            manager.task.result(timeout=35)
            answer=manager.poll()
            self.assertEqual(answer["action"],"PLAN2_READY_ADVISORY")
            self.assertTrue(answer["native_Astar_required"])
            hold=manager.safety_checkpoint(observed=newer,mean=newer_mean,
                    variance=var,semantic=newer_sem,native_walkable=native,
                    robot=current,scan_tick=11,structural_epoch=2,
                    room_id=1,active_route_safe=False,
                    native_local_wfd_exists=True)
            self.assertEqual(hold["action"],"SAFE_HOLD")
            exit_gate=manager.safety_checkpoint(observed=newer,mean=newer_mean,
                    variance=var,semantic=newer_sem,native_walkable=native,
                    robot=current,scan_tick=11,structural_epoch=2,
                    room_id=1,active_route_safe=True,
                    native_local_wfd_exists=False)
            self.assertEqual(exit_gate["action"],"G1_WFD_EMPTY_EXIT_GATE")
        finally:manager.close()

    def test_opposing_viewpoints_not_forced_for_obstacle_free_room(self):
        obs=np.full((40,40),2,np.uint8)
        obs[3:37,3:37]=0
        sem=np.ones_like(obs,np.uint8)
        mean=np.where(obs==2,1,0).astype(np.float32)
        r=R7Standalone(obs,mean,np.zeros_like(mean),sem,(20,20),
                       obs==0)
        plan=r.plan()
        self.assertEqual(len(r.geo.obstacles),0)
        self.assertLessEqual(len(plan["goals_rc"]),1)


if __name__=="__main__":
    unittest.main(verbosity=2)
