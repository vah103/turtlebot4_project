import unittest,sys,time
from pathlib import Path
import numpy as np
BASE=Path('/home/dell/DELL_NATIVE2D/runs/NR-G2R61-03/native_g2_r61_watchman_run01/G2_R61_STRUCTURAL_RUN01_visvarprob')
from g2_r6_watchman import RoomWatchman
from r61_structural_events import StructuralEventMonitor,S0,S1,S2,S3,S4

def frozen(i):
    with np.load(BASE/'room_r41/inference'/('decision_%06d_input.npz'%i)) as z:
        obs=z['observed'].copy();m=z['mean'].copy();v=z['variance'].copy()
    with np.load(BASE/'room_r41/inference'/('decision_%06d_semantic.npz'%i)) as z:
        sem=z['semantic'].copy()
    with np.load(BASE/'g2_r61'/('native_scope_%06d.npz'%i)) as z:
        nav=z['reachable_knownfree'].copy()
    import json
    log=json.loads((BASE/'room_r41/decisions'/('decision_%06d.json'%i)).read_text())
    pose=tuple(x-500 for x in log['robot_rc'])
    return obs,m,v,sem,nav,pose

class Regression(unittest.TestCase):
    def test_d11_positive_viewpoint_is_selected_without_floor(self):
        obs,m,v,sem,nav,pose=frozen(11)
        p=RoomWatchman(obs,m,v,sem,pose,nav)
        result=p.plan()
        self.assertEqual(result['status'],'CONDITIONAL_SAFE_ROUTE')
        self.assertGreater(len(result['goal_sequence_core_rc']),0)
        self.assertTrue(result['goals_only_native_known_free'])
        for goal in result['goal_sequence_core_rc']:
            self.assertTrue(nav[tuple(goal)] and obs[tuple(goal)]==0)

    def test_d02_remains_causal(self):
        obs,m,v,sem,nav,pose=frozen(2)
        p=RoomWatchman(obs,m,v,sem,pose,nav)
        r=p.plan()
        self.assertFalse(r['used_gt'])
        self.assertTrue(r['goals_only_native_known_free'])
        self.assertLess(r['extra_G2_planning_wall_s'],5)

    def test_small_free_outside_old_room_not_immediate_s3(self):
        obs=np.ones((50,60),np.uint8)
        obs[10:35,10:22]=0
        room=np.zeros(obs.shape,bool);room[:,10:22]=True
        m=np.full(obs.shape,.1,np.float32);v=np.zeros(obs.shape,np.float32)
        mon=StructuralEventMonitor(obs,m,v,room,(20,15),0)
        nxt=obs.copy();nxt[20,22]=0
        ev=mon.refresh(nxt,(20,15),1)
        self.assertNotEqual(ev['event'],S3)
        self.assertNotEqual(ev['event'],S0)

    def test_remote_free_to_wall_correction_not_s0(self):
        obs=np.ones((50,60),np.uint8)
        obs[10:40,5:22]=0
        room=np.ones(obs.shape,bool)
        m=np.full(obs.shape,.1,np.float32);v=np.zeros(obs.shape,np.float32)
        mon=StructuralEventMonitor(obs,m,v,room,(20,12),0)
        nxt=obs.copy();nxt[38,21]=2 # real map correction, far off active path
        ev=mon.refresh(nxt,(20,12),1,active_route_safe=True)
        self.assertNotEqual(ev['event'],S0)
        self.assertEqual(nxt[38,21],2)

    def test_active_native_path_block_still_immediate_s0(self):
        obs=np.ones((50,60),np.uint8)
        obs[10:40,5:22]=0
        room=np.ones(obs.shape,bool)
        m=np.full(obs.shape,.1,np.float32);v=np.zeros(obs.shape,np.float32)
        mon=StructuralEventMonitor(obs,m,v,room,(20,12),0)
        ev=mon.refresh(obs,(20,12),1,active_route_safe=False)
        self.assertEqual(ev['event'],S0)

    def test_real_observed_bridge_is_structural(self):
        obs=np.ones((64,84),np.uint8)
        obs[5:60,5:22]=0
        obs[20:28,35:43]=0
        room=np.ones(obs.shape,bool)
        m=np.full(obs.shape,.1,np.float32);v=np.zeros(obs.shape,np.float32)
        mon=StructuralEventMonitor(obs,m,v,room,(20,10),0)
        nxt=obs.copy();nxt[24,22:36]=0
        ev=mon.refresh(nxt,(20,10),1)
        self.assertEqual(ev['event'],S3)

    def test_new_contradictory_wall_off_path_does_not_force_heavy(self):
        obs=np.ones((64,84),np.uint8);obs[5:60,5:22]=0
        room=np.ones(obs.shape,bool)
        m=np.full(obs.shape,.1,np.float32);v=np.full(obs.shape,.01,np.float32)
        mon=StructuralEventMonitor(obs,m,v,room,(20,10),0)
        nxt=obs.copy();nxt[30:36,28:35]=2
        ev=mon.refresh(nxt,(20,10),1)
        self.assertNotEqual(ev['event'],S3)
        self.assertNotEqual(ev['event'],S0)

    def test_stable_nochange_keeps(self):
        obs=np.ones((64,84),np.uint8);obs[5:60,5:22]=0
        room=np.ones(obs.shape,bool)
        m=np.full(obs.shape,.1,np.float32);v=np.full(obs.shape,.01,np.float32)
        mon=StructuralEventMonitor(obs,m,v,room,(20,10),0)
        ev=mon.refresh(obs,(20,10),1)
        self.assertEqual(ev['event'],S1)

    def test_native_source_has_residual_wfd_guard(self):
        source=(Path(__file__).parent/'explore_g2_r62_lama3_smallroi.py').read_text()
        self.assertIn('G2_R62_RESIDUAL_ROOM_WFD_FALLBACK',source)
        self.assertIn('if candidate_g2_goal is None:',source)
        self.assertNotIn('LOCAL_VIEWPOINT_UNAVAILABLE_WITH_REACHABLE_WFD',source)

if __name__=='__main__':unittest.main(verbosity=2)