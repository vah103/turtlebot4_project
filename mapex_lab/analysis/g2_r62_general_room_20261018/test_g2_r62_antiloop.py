import unittest,tempfile
from pathlib import Path
import numpy as np
from g2_r6_watchman import RoomWatchman
from r61_plan_manager import R61PlanManager
from g2_r61_sidecar import R61Service
from test_g2_r62_regression import frozen,BASE

class AntiLoopContract(unittest.TestCase):
 def new_manager(self):
  obs,m,v,sem,nav,pose=frozen(2)
  p=RoomWatchman(obs,m,v,sem,pose,nav);p.plan()
  return R61PlanManager(p,1,12),obs,nav

 def test_zero_real_progress_at_waypoint_switches_to_wfd(self):
  manager,obs,nav=self.new_manager()
  goal=manager.active_goals[0]
  reply=manager.checkpoint(obs,goal,13,nav,source_goal=goal)
  self.assertEqual(reply['action'],'RESIDUAL_ROOM_WFD')
  self.assertEqual(reply['observed_new_cells'],0)
  self.assertEqual(len(manager.consumed),1)

 def test_new_observation_progress_does_not_trigger_residual_mode(self):
  manager,obs,nav=self.new_manager()
  goal=manager.active_goals[0]
  new=obs.copy()
  candidates=np.argwhere((obs==1)&manager.r6.room)
  self.assertTrue(len(candidates)>5)
  for y,x in candidates[:5]:new[y,x]=0
  reply=manager.checkpoint(new,goal,13,nav,source_goal=goal)
  self.assertNotEqual(reply['action'],'RESIDUAL_ROOM_WFD')

 def test_sidecar_signals_wfd_mode_on_zero_progress(self):
  obs,m,v,sem,nav,pose=frozen(2)
  root=BASE
  s=R61Service()
  req={'action':'CHOOSE','room_id':1,'native_tick':12,
    'robot_core_rc':list(pose),
    'room_node_input':str(root/'room_r41/inference/decision_000002_input.npz'),
    'room_node_semantic':str(root/'room_r41/inference/decision_000002_semantic.npz'),
    'native_scope_npz':str(root/'g2_r61/native_scope_000002.npz')}
  chosen=s.handle(req)
  self.assertEqual(chosen['decision'],'GOAL')
  with tempfile.TemporaryDirectory(prefix='r62_zero_obs_') as folder:
   f=Path(folder)
   np.savez_compressed(str(f/'obs.npz'),observed=obs)
   np.savez_compressed(str(f/'nav.npz'),reachable_knownfree=nav)
   cp=s.handle({'action':'CHECKPOINT','native_tick':13,'robot_core_rc':chosen['goal_core_rc'],
    'observed_npz':str(f/'obs.npz'),'native_scope_npz':str(f/'nav.npz')})
   self.assertEqual(cp['action'],'RESIDUAL_ROOM_WFD')
   self.assertTrue(cp['need_fresh_coherent_LaMa_RoomNode'])

 def test_room_wfd_fallback_not_predicted_unknown(self):
  from pathlib import Path
  text=(Path(__file__).parent/'explore_g2_r62_lama3_smallroi.py').read_text()
  self.assertIn('G2_R62_RESIDUAL_ROOM_WFD_FALLBACK',text)
  self.assertIn("getattr(g2_run,'force_room_wfd',False)",text)
  self.assertIn('frontier_region_centers[np.argmin(frontier_cost_list)]',text)
  self.assertIn("pyastar2d.astar_path(room_run.grid",text)

if __name__=='__main__': unittest.main(verbosity=2)