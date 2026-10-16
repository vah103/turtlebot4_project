import sys,tempfile,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mapex_g2_r3.structural import MissionMemory,StructuralConfig,analyze,accepted_component

def fixture():
 obs=np.full((48,70),2,dtype=np.uint8)
 obs[7:41,5:65]=1
 obs[14:35,8:32]=0
 obs[24,34:46]=0
 obs[20:28,40:58]=0
 mean=np.full(obs.shape,.1,dtype=np.float32)
 mean[obs==2]=1
 mean[17:30,32:35]=.9
 var=np.full(obs.shape,.02,dtype=np.float32)
 sem=np.zeros(obs.shape,np.uint8);sem[6:42,4:66]=1
 robot=(25,20)
 return obs,mean,var,sem,robot
class TestG2R3StructuralMemory(unittest.TestCase):
 def call(self,obs,mean,var,sem,pose,mem,decision='t'):
  return analyze(obs,mean,var,sem,pose,mem,StructuralConfig(
   beams=64,ray_step_cells=.5,max_vantages=30,grid_step_cells=4),decision)
 def test_initial_room_frozen(self):
  o,m,v,s,p=fixture()
  mem=MissionMemory.begin(o,s,p)
  report,labs,room=self.call(o,m,v,s,p,mem)
  self.assertEqual(report['original_room_unknown'],int((mem.initial&(o==1)).sum()))
  self.assertEqual(report['unknown_outside_current_room'],0)
  self.assertGreater(len(report['regions']),0)
  self.assertEqual(report['mission_unknown'],report['current_room_unknown'])
 def test_boundary_shrink_preserves_historic_obligation(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  self.call(o,m,v,s,p,mem)
  s2=s.copy();s2[:,45:]=2
  report,_,_=self.call(o,m,v,s2,p,mem,'shrink')
  self.assertGreater(report['unknown_outside_current_room'],0)
  self.assertEqual(report['mission_unknown'],int((mem.initial&(o==1)).sum()))
  self.assertEqual(report['new_mission_cells_since_last_snapshot'],0)
  self.assertNotEqual(report['status'],'OBSERVED_MISSION_NO_UNKNOWN')
 def test_expansion_keeps_initial(self):
  o,m,v,s,p=fixture()
  mem=MissionMemory.begin(o,s,p);self.call(o,m,v,s,p,mem)
  # Replace mission with a smaller initially accepted connected Room Node,
  # then allow it to expand on the SAME raster without erasing original cells.
  cropped_sem=np.zeros(s.shape,np.uint8);cropped_sem[10:39,7:39]=1
  mem=MissionMemory.begin(o,cropped_sem,p)
  first,_,_=self.call(o,m,v,cropped_sem,p,mem)
  second,_,_=self.call(o,m,v,s,p,mem)
  self.assertGreater(second['new_mission_cells_since_last_snapshot'],0)
  self.assertTrue(np.all(mem.mission[mem.initial]))
 def test_known_evidence_cannot_erase(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p);self.call(o,m,v,s,p,mem)
  edited=o.copy();edited[25,19]=1
  with self.assertRaisesRegex(ValueError,'cannot revert'):self.call(edited,m,v,s,p,mem)
 def test_new_free_only_from_observation(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  first,lab,cur=self.call(o,m,v,s,p,mem)
  earlier=mem.mission&(o==1)
  coords=np.argwhere(earlier)
  nextobs=o.copy()
  for y,x in coords[:40]:nextobs[y,x]=0
  second,_,_=self.call(nextobs,m,v,s,p,mem)
  self.assertEqual(second['mission_unknown'],first['mission_unknown']-40)
 def test_prediction_change_does_not_clear_evidence(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  first,lab,cur=self.call(o,m,v,s,p,mem)
  m2=m.copy();m2[o==1]=.99
  second,_,_=self.call(o,m2,v,s,p,mem)
  self.assertEqual(first['mission_unknown'],second['mission_unknown'])
  self.assertEqual(second['predicted_free_like_unknown'],0)
  self.assertEqual(second['unverified_predicted_wall_unknown'],second['mission_unknown'])
 def test_identity_stability_and_no_duplicate_ids(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  first,_,_=self.call(o,m,v,s,p,mem)
  second,lab,_=self.call(o,m,v,s,p,mem)
  self.assertEqual(set(q['region_id'] for q in first['regions']),set(q['region_id'] for q in second['regions']))
  ids=[q['region_id'] for q in second['regions']]
  self.assertEqual(len(ids),len(set(ids)))
 def test_split_gives_new_identity(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  self.call(o,m,v,s,p,mem)
  edited=o.copy()
  # observed wall splits a previously unknown room region
  portion=edited[9:39,36];portion[portion==1]=2
  rr,_,_=self.call(edited,m,v,s,p,mem)
  ids=[x['region_id'] for x in rr['regions']]
  self.assertEqual(len(ids),len(set(ids)))
 def test_roundtrip_safe_serialization(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  self.call(o,m,v,s,p,mem)
  with tempfile.TemporaryDirectory() as tmp:
   path=Path(tmp)/'state.npz';mem.save(path)
   copy=MissionMemory.load(path)
   self.assertTrue(np.array_equal(mem.initial,copy.initial))
   self.assertTrue(np.array_equal(mem.labels,copy.labels))
   self.assertEqual(len(mem.history),len(copy.history))
 def test_empty_unknown_is_completion_observed_only(self):
  o,m,v,s,p=fixture()
  o[7:41,5:65]=0
  mem=MissionMemory.begin(o,s,p)
  res,_,_=self.call(o,m,v,s,p,mem)
  self.assertEqual(res['mission_unknown'],0)
  self.assertEqual(res['status'],'OBSERVED_MISSION_NO_UNKNOWN')
 def test_candidate_routes_are_in_observed_free(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  res,_,_=self.call(o,m,v,s,p,mem)
  for region in res['regions']:
   for candidate in region['top_conditional_viewpoints']:
    pos=tuple(candidate['core_rc'])
    self.assertEqual(o[pos],0)
    self.assertTrue(candidate['not_a_final_G2_route_selection'])
 def test_robot_not_room_is_invalid_at_begin(self):
  o,m,v,s,p=fixture();s[:]=0
  with self.assertRaisesRegex(ValueError,'not in accepted'):MissionMemory.begin(o,s,p)
 def test_counterfeit_gt_not_in_interface(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  report,_,_=self.call(o,m,v,s,p,mem)
  self.assertFalse(report['input_used_gt'])
  self.assertNotIn('ground_truth',str(report))
 def test_invalid_map_shape_fails(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  with self.assertRaisesRegex(ValueError,'same 2D shape'):self.call(o,m[:10],v,s,p,mem)
 def test_disconnected_reassignment_rejected(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  self.call(o,m,v,s,p,mem)
  ss=np.zeros(s.shape,np.uint8);ss[22:30,15:22]=1
  # subset overlap allowed: should retain
  report,_,_=self.call(o,m,v,ss,p,mem)
  self.assertGreater(report['unknown_outside_current_room'],0)
 def test_occlusion_explanation_partitions_every_region(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  result,_,_=self.call(o,m,v,s,p,mem)
  for r in result['regions']:
   self.assertEqual(r['cells'],
    r['current_robot_nominal_line_of_sight_cells']+
    r['occluded_by_predicted_structure_proxy_cells']+
    r['occluded_by_observed_structure_or_geometry_proxy_cells'])
 def test_region_cells_partition_mission_unknown_exactly(self):
  o,m,v,s,p=fixture();mem=MissionMemory.begin(o,s,p)
  result,_,_=self.call(o,m,v,s,p,mem)
  self.assertEqual(sum(r['cells'] for r in result['regions']),result['mission_unknown'])
  self.assertEqual(result['predicted_free_like_unknown']+
   result['unverified_predicted_wall_unknown'],result['mission_unknown'])
if __name__=='__main__':unittest.main()
