import unittest
from pathlib import Path
import sys,json,numpy as np
ROOT=Path('/home/dell/DELL_NATIVE2D/build/g2_r63_room_global_r1_20261018')
sys.path.insert(0,str(ROOT))
from r63_exit_planner import RoomGlobalWatchman,observed_external_frontiers,distance_to_frontier
from g2_r6_watchman import RoomWatchman
DATA=Path('/home/dell/DELL_NATIVE2D/runs/NR-G2R62-03/native_g2_r62_general_r2_run01/G2_R62_GENERAL_R2_RUN01_visvarprob')
def maker(i,cls):
 f='decision_%06d'%i
 with np.load(DATA/'room_r41/inference'/(f+'_input.npz')) as z:obs,mean,var=z['observed'],z['mean'],z['variance']
 with np.load(DATA/'room_r41/inference'/(f+'_semantic.npz')) as z:sem=z['semantic']
 with np.load(DATA/'g2_r62'/('native_scope_%06d.npz'%i)) as z:native=z['reachable_knownfree']
 log=json.loads((DATA/'room_r41/decisions'/(f+'.json')).read_text())
 pose=tuple(q-500 for q in log['robot_rc'])
 return cls(obs,mean,var,sem,pose,native)
class R63(unittest.TestCase):
 def test_01_D02_PARETO_CANDIDATE(self):
  a=maker(2,RoomWatchman);b=maker(2,RoomGlobalWatchman)
  p=a.plan();q=b.plan()
  self.assertEqual(set(map(tuple,p['goal_sequence_core_rc'])),set(map(tuple,q['goal_sequence_core_rc'])))
  self.assertLessEqual(q['chosen_total_predicted_cost_m'],q['baseline_total_predicted_cost_m'])
  self.assertTrue(q['goals_only_native_known_free'])
  self.assertFalse(q['used_gt'])
 def test_02_D08_same_goal_set(self):
  a=maker(8,RoomWatchman);b=maker(8,RoomGlobalWatchman)
  p=a.plan();q=b.plan()
  self.assertEqual(set(map(tuple,p['goal_sequence_core_rc'])),set(map(tuple,q['goal_sequence_core_rc'])))
 def test_03_D11_global_cost(self):
  p=maker(11,RoomGlobalWatchman).plan()
  self.assertLess(p['chosen_total_predicted_cost_m'],p['baseline_total_predicted_cost_m'])
  self.assertTrue(p['score_per_total_cost_gain']>0)
 def test_04_D14_stability(self):
  p=maker(14,RoomGlobalWatchman).plan()
  self.assertEqual(p['policy_version'],'G2_R63_ROOM_TO_GLOBAL_R1')
  self.assertFalse(p['actual_global_goal_commanded'])
 def test_05_D16_known_FREE(self):
  b=maker(16,RoomGlobalWatchman);p=b.plan()
  self.assertTrue(p['goals_only_native_known_free'])
  for goal in p['goal_sequence_core_rc']:self.assertTrue(b.legal[tuple(goal)])
 def test_06_NO_OUTER_FRONTIER_KEEP_ROUTE(self):
  obs=np.ones((45,45),np.uint8);obs[5:40,5:40]=0
  sem=np.ones((45,45),np.uint8)
  mean=np.full((45,45),.1,np.float32);var=np.zeros((45,45),np.float32)
  native=(obs==0)
  planner=RoomGlobalWatchman(obs,mean,var,sem,(20,20),native)
  result=planner.plan()
  self.assertEqual(result['exit_policy'],'NO_OBSERVED_REACHABLE_EXTERNAL_FRONTIER_KEEP_R62')
 def test_07_NO_GT_DEPENDENCY(self):
  s=(ROOT/'r63_exit_planner.py').read_text()
  self.assertNotIn('gt_map',s.lower());self.assertNotIn('ground_truth',s.lower())
  self.assertIn('native',s.lower())
 def test_08_EXIT_ESTIMATOR_ONLY(self):
  s=(ROOT/'r63_exit_planner.py').read_text()
  self.assertNotIn('GenUI',s)
  q=maker(2,RoomGlobalWatchman).plan()
  self.assertFalse(q['actual_global_goal_commanded'])
 def test_09_COMPAT_R62_ANTILOOP(self):
  s=(ROOT/'explore_g2_r63_lama3_smallroi.py').read_text()
  self.assertIn('force_room_wfd',s)
  self.assertIn('frontier_region_centers[np.argmin(frontier_cost_list)]',s)
if __name__=='__main__':unittest.main(verbosity=2)