"""Focused R7 tests, including no-future-data and safe promotion contracts."""
import unittest
import numpy as np
import cv2
from r7_structural_watchman import R7Planner,R7Config,SurfaceEvidence
from r7_successor import R7Supervisor


def small_scene():
    shape=(52,78)
    obs=np.ones(shape,np.uint8)
    obs[5:47,5:72]=0
    obs[20:33,38:44]=2
    # Unknown patches behind the object; these are NOT marked FREE.
    obs[9:19,26:57]=1
    obs[34:44,26:57]=1
    sem=np.ones(shape,np.uint8)
    sem[0,:]=0;sem[-1,:]=0
    mean=np.zeros(shape,np.float32)
    mean[obs==2]=1.
    var=np.zeros(shape,np.float32)
    legal=(obs==0)
    # Native-inflated mask proxy for synthetic testing.
    legal=cv2.erode(legal.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
    return obs,mean,var,sem,legal


class R7Tests(unittest.TestCase):
    def test_complementary_surface_coverage(self):
        obs,mean,_,sem,_=small_scene()
        surf=SurfaceEvidence(obs,mean,sem==1,R7Config())
        self.assertGreater(len(surf.samples),0)
        a=surf.at((26,18));b=surf.at((26,63))
        self.assertGreater(a.bit_count(),0)
        self.assertGreater(b.bit_count(),0)
        self.assertGreater((b&~a).bit_count(),0,
                           "opposite viewpoint should expose unseen obstacle faces")

    def test_no_unknown_execution(self):
        obs,mean,var,sem,legal=small_scene()
        p=R7Planner(obs,mean,var,sem,(26,12),legal)
        answer=p.plan()
        self.assertTrue(answer["all_goals_observed_free"])
        self.assertTrue(all(obs[tuple(x)]==0 and legal[tuple(x)] for x in answer["goal_sequence_core_rc"]))
        self.assertTrue(all(obs[tuple(x)]==0 for x in answer["path_core_rc"]))
        self.assertFalse(answer["planner_used_gt"])
        self.assertFalse(answer["planner_used_future_observations"])

    def test_cached_prediction_not_mutated_by_planning(self):
        obs,mean,var,sem,legal=small_scene()
        p=R7Planner(obs,mean,var,sem,(26,12),legal)
        before=p.rays.at(p.robot)
        frozen=(before[0].copy(),before[1].copy())
        p.plan()
        after=p.rays.at(p.robot)
        self.assertTrue(np.array_equal(frozen[0],after[0]))
        self.assertTrue(np.array_equal(frozen[1],after[1]))

    def test_known_obstacle_outside_semantic_room_is_counted(self):
        obs,mean,var,sem,legal=small_scene()
        sem=sem.copy()
        sem[obs==2]=3
        p=R7Planner(obs,mean,var,sem,(26,12),legal)
        self.assertGreater(len(p.faces.samples),0)
        self.assertGreater(p.faces.at((26,18)).bit_count(),0)
        self.assertGreater(p.faces.at((26,63)).bit_count(),0)

    def test_future_plan_non_executable(self):
        obs,mean,var,sem,legal=small_scene()
        p=R7Planner(obs,mean,var,sem,(26,12),legal)
        for c in p.predicted_future_candidates():
            self.assertFalse(c["executable"])
            self.assertEqual(obs[tuple(c["goal_rc"])],1)

    def test_plan2_background_then_checkpoint(self):
        obs,mean,var,sem,legal=small_scene()
        anchor=(26,18)
        self.assertTrue(legal[anchor])
        path=[(26,j) for j in range(12,19)]
        sup=R7Supervisor(room_id=1,structural_epoch=2)
        try:
            request=sup.prepare_during_motion(observed=obs,mean=mean,variance=var,
                semantic=sem,native_walkable=legal,seq=11,room_id=1,structural_epoch=2,
                anchor=anchor,active_prefix=path)
            self.assertEqual(request["action"],"BACKGROUND_PREPARATION_STARTED")
            sup._pending.result(timeout=15)
            rep=sup.poll()
            self.assertEqual(rep["action"],"SUCCESSOR_READY_ADVISORY")
            waiting=sup.checkpoint(observed=obs,mean=mean,variance=var,semantic=sem,
                native_walkable=legal,pose=(26,17),seq=11,room_id=1,structural_epoch=2)
            self.assertEqual(waiting["action"],"WAIT_ARRIVAL")
            out=sup.checkpoint(observed=obs,mean=mean,variance=var,semantic=sem,
                native_walkable=legal,pose=anchor,seq=12,room_id=1,structural_epoch=2)
            self.assertIn(out["action"],("SUCCESSOR_PROMOTED_ADVISORY","RESIDUAL_ROOM_WFD"))
            self.assertIsNone(out.get("robot_command"))
            self.assertTrue(out.get("must_call_native_astar_before_dispatch",True))
        finally:sup.close()

    def test_safety_and_structural_epoch_veto(self):
        obs,mean,var,sem,legal=small_scene()
        sup=R7Supervisor(room_id=1,structural_epoch=2)
        try:
            request=sup.prepare_during_motion(observed=obs,mean=mean,variance=var,
                semantic=sem,native_walkable=legal,seq=11,room_id=1,structural_epoch=2,
                anchor=(26,18),active_prefix=[(26,j) for j in range(12,19)])
            sup._pending.result(timeout=15);sup.poll()
            held=sup.checkpoint(observed=obs,mean=mean,variance=var,semantic=sem,
                native_walkable=legal,pose=(26,18),seq=12,room_id=1,structural_epoch=2,
                active_route_safe=False)
            self.assertEqual(held["action"],"SAFE_HOLD")
            veto=sup.checkpoint(observed=obs,mean=mean,variance=var,semantic=sem,
                native_walkable=legal,pose=(26,18),seq=13,room_id=1,structural_epoch=3)
            self.assertEqual(veto["action"],"REJECT_AND_REPLAN")
        finally:sup.close()

    def test_room_exit_is_native_g1_gate(self):
        obs,mean,var,sem,legal=small_scene()
        sup=R7Supervisor(room_id=1,structural_epoch=2)
        try:
            resp=sup.checkpoint(observed=obs,mean=mean,variance=var,
                semantic=sem,native_walkable=legal,pose=(26,12),seq=15,
                room_id=1,structural_epoch=2,local_frontiers_exist=False)
            self.assertEqual(resp["action"],"G1_LOCAL_WFD_EXIT_GATE")
        finally:sup.close()


if __name__=="__main__":unittest.main(verbosity=2)
