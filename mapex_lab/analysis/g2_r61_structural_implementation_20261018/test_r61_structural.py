import unittest
import numpy as np
from r61_structural_events import (StructuralEventMonitor, EventConfig,
                                   S0,S1,S2,S3,S4, observation_delta)

class TestStructuralEvents(unittest.TestCase):
    def setup_map(self,shape=(64,84)):
        obs=np.ones(shape,dtype=np.uint8)
        obs[5:60,5:22]=0
        room=np.ones(shape,bool)
        mean=np.full(shape,.10,np.float32)
        var=np.full(shape,.01,np.float32)
        return obs,room,mean,var

    def monitor(self,obs=None,room=None,mean=None,var=None,pose=(20,10),cfg=None):
        if obs is None:obs,room,mean,var=self.setup_map()
        return StructuralEventMonitor(obs,mean,var,room,pose,cfg=cfg)

    def test_01_expected_free_no_heavy(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v);now=obs.copy();now[25:35,22:26]=0
        e=s.refresh(now,(20,10),1)
        self.assertNotEqual(e["event"],S3)
        self.assertEqual(e["new_free"],40)

    def test_02_1000_expected_free_no_auto_heavy(self):
        obs,room,m,v=self.setup_map((70,120))
        s=self.monitor(obs,room,m,v)
        now=obs.copy();now[12:52,30:55]=0
        e=s.refresh(now,(20,10),1)
        self.assertEqual(e["new_free"],1000)
        self.assertNotIn(e["event"],(S0,S3,S4))

    def test_03_small_bridge_two_free_components_is_S3(self):
        obs,room,m,v=self.setup_map()
        obs[20:28,35:43]=0
        s=self.monitor(obs,room,m,v)
        now=obs.copy();now[24,22:36]=0
        e=s.refresh(now,(20,10),1)
        self.assertEqual(e["event"],S3)
        self.assertEqual(e["reason"],"OBSERVED_COMPONENT_BRIDGE")
        self.assertGreaterEqual(e["bridge_count"],1)

    def test_04_new_free_branch_is_S2(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        now=obs.copy();now[20:32,22:29]=0
        e=s.refresh(now,(20,10),1)
        self.assertEqual(e["event"],S2)

    def test_05_blocked_active_segment_immediate_S0(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        e=s.refresh(obs.copy(),(20,10),1,active_route_safe=False)
        self.assertEqual(e["event"],S0)

    def test_06_new_observed_wall_contradicts_prediction(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        now=obs.copy();now[30:36,28:35]=2
        e=s.refresh(now,(20,10),1)
        self.assertEqual(e["event"],S3)
        self.assertEqual(e["unexpected_wall_strong_count"],42)

    def test_07_future_goal_loses_value(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        e=s.refresh(obs.copy(),(20,10),1,future_goal_gain_drop=.25)
        self.assertEqual(e["event"],S2)

    def test_08_stale_sequence_rejected(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v,pose=(20,10))
        s.refresh(obs.copy(),(20,10),5)
        e=s.refresh(obs.copy(),(20,10),4)
        self.assertFalse(e["accepted"])
        self.assertEqual(e["reason"],"STALE_SCAN_REJECTED")

    def test_09_duplicate_exact_scan_idempotent(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        first=s.refresh(obs.copy(),(20,10),3)
        second=s.refresh(obs.copy(),(20,10),3)
        self.assertFalse(second["accepted"])
        self.assertEqual(s.event_no,1)

    def test_10_same_seq_new_map_forbidden(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        nxt=obs.copy();nxt[40,30]=0
        with self.assertRaises(ValueError):s.refresh(nxt,(20,10),0)

    def test_11_no_local_frontier_exit_gate(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        e=s.refresh(obs.copy(),(20,10),1,local_frontiers_exist=False)
        self.assertEqual(e["event"],S4)

    def test_12_semantic_corridor_exit(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        sem=np.ones(obs.shape,np.uint8)
        sem[20,10]=2
        e=s.refresh(obs.copy(),(20,10),1,confirmed_new_semantic=sem)
        self.assertEqual(e["event"],S4)

    def test_13_outside_old_room_free_supports_S3(self):
        obs,room,m,v=self.setup_map()
        room[:,22:]=False
        s=self.monitor(obs,room,m,v)
        now=obs.copy();now[20:25,22:27]=0
        e=s.refresh(now,(20,10),1)
        self.assertEqual(e["event"],S3)
        self.assertEqual(e["reason"],"OBSERVED_EXPANSION_OUTSIDE_ACCEPTED_ROOM")

    def test_14_noise_single_portal_not_S3(self):
        obs,room,m,v=self.setup_map()
        # Walls support one tentative passage. Need two different scans.
        obs[15:35,35]=2;obs[15:35,41]=2
        s=self.monitor(obs,room,m,v)
        now=obs.copy();now[20:23,38]=0
        e=s.refresh(now,(20,10),1)
        self.assertNotEqual(e["event"],S3)

    def test_15_shape_and_prediction_contract(self):
        obs,room,m,v=self.setup_map()
        with self.assertRaises(ValueError):
            observation_delta(obs,obs,np.ones((3,3)),v,room)

    def test_16_speed_and_no_gt(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        e=s.refresh(obs.copy(),(20,10),1)
        self.assertLess(e["detection_wall_s"],.10)
        self.assertFalse(e["used_gt"])
        self.assertFalse(e["LaMa_inference_called"])
        self.assertFalse(e["RoomNode_inference_called"])

    def test_17_free_to_wall_correction_is_safety(self):
        obs,room,m,v=self.setup_map()
        s=self.monitor(obs,room,m,v)
        now=obs.copy();now[20,12]=2
        e=s.refresh(now,(20,10),1)
        self.assertEqual(e["event"],S0)

    def test_18_diagonal_touch_is_not_real_connectivity(self):
        obs=np.ones((45,45),np.uint8)
        obs[10:20,10:20]=0
        obs[21:31,21:31]=0
        room=np.ones(obs.shape,bool)
        m=np.full(obs.shape,.15,np.float32)
        v=np.full(obs.shape,.01,np.float32)
        s=self.monitor(obs,room,m,v,pose=(15,15))
        nxt=obs.copy();nxt[20,20]=0
        e=s.refresh(nxt,(15,15),1)
        self.assertNotEqual(e["event"],S3)
        self.assertEqual(e["bridge_count"],0)

if __name__ == "__main__":
    unittest.main(verbosity=2)