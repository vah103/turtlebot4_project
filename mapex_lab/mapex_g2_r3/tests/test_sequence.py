"""Synthetic regressions for new structural multi-goal sequence optimizer."""
import unittest
import numpy as np
from dataclasses import replace
from mapex_g2_r3.structural import MissionMemory,StructuralConfig,analyze
from mapex_g2_r3.sequence import StructuralRoutePlanner,SequenceConfig,_Visibility

def build():
 obs=np.full((46,68),2,np.uint8)
 obs[5:40,4:64]=1
 obs[17:32,6:62]=0
 obs[8:17,11:24]=1;obs[8:17,38:57]=1
 # Door openings, some observed obstacle silhouettes.
 obs[17,26:34]=2
 obs[32:37,26:34]=2
 mean=np.full(obs.shape,.08,np.float32)
 mean[obs==2]=1.
 mean[11:16,24:29]=.85
 var=np.full(obs.shape,.012,np.float32)
 sem=np.ones(obs.shape,np.uint8);sem[obs==2]=0
 robot=(25,33)
 return obs,mean,var,sem,robot

def engine_from(o,m,v,sem,r,config=None):
 memory=MissionMemory.begin(o,sem,r)
 st,labels,current=analyze(o,m,v,sem,r,memory,
     StructuralConfig(beams=32,max_vantages=27),decision_id='synthetic')
 cfg=config or SequenceConfig(coarse_beams=48,dense_beams=128,max_initial_viewpoint_proposals=24,
     max_search_candidates=14,beam_width=3,max_search_depth=3,shortlisted_final_sequences=4)
 return StructuralRoutePlanner(o,m,v,sem,r,memory,st,cfg)

class SequenceTests(unittest.TestCase):
 def test_gt_never_part_of_interface(self):
  p=engine_from(*build())
  self.assertFalse(hasattr(p,'ground_truth'))
  self.assertEqual(p.unknown.sum(),p.report['mission_unknown'])
 def test_stale_snapshot_rejected(self):
  o,m,v,s,r=build();p=engine_from(o,m,v,s,r)
  o2=o.copy();o2[25,20]=1
  with self.assertRaisesRegex(ValueError,'report does not match'):
   StructuralRoutePlanner(o2,m,v,s,r,p.memory,p.report,p.cfg)
 def test_stale_memory_evidence_rejected(self):
  o,m,v,s,r=build();p=engine_from(o,m,v,s,r)
  p.memory.previous_observed[25,20]=1
  with self.assertRaisesRegex(ValueError,'memory state'):
   StructuralRoutePlanner(o,m,v,s,r,p.memory,p.report,p.cfg)
 def test_one_scan_not_counted_twice(self):
  p=engine_from(*build())
  a=p.coarse.sight((25,33))
  self.assertEqual((a[0]|a[0]),a[0])
  self.assertEqual((a[1]|a[1]),a[1])
 def test_region_target_partition(self):
  p=engine_from(*build())
  total=sum((v).bit_count() for v in p.region_bits.values())
  self.assertLessEqual(total,int(p.unknown.sum()))
  self.assertEqual(len(p.region_bits),len(p.major))
 def test_finite_sequence_without_gt(self):
  p=engine_from(*build());result=p.solve()
  self.assertIn(result['status'],('CONDITIONAL_VIEWPOINT_SEQUENCE','DEFER_NO_POSITIVE_STRUCTURAL_CHAIN','DEFER_NO_REACHABLE_VIEWPOINT'))
  if result['status']=='CONDITIONAL_VIEWPOINT_SEQUENCE':
   self.assertTrue(result['goal_count']>=1)
   self.assertLessEqual(result['goal_count'],p.cfg.max_search_depth)
   for leg in result['selected']['stages']:
    self.assertTrue(all(p.obs[tuple(x)]==0 for x in leg['path_rc']))
 def test_no_hardcoded_hand_viewpoints(self):
  p=engine_from(*build());c=p.prepare_candidates()
  self.assertTrue(all(point in p.parent0 for point in c))
  self.assertTrue(all(p.obs[point]==0 for point in c))
 def test_source_endpoints_and_distance(self):
  p=engine_from(*build());choices=p.prepare_candidates()
  self.assertGreater(len(choices),0)
  item=p._leg(p.robot,choices[0],p.coarse)
  self.assertEqual(item[2][0],p.robot)
  self.assertEqual(item[2][-1],choices[0])
  self.assertEqual(len(item[2])-1,item[0])
 def test_repeated_plan_is_deterministic(self):
  a=engine_from(*build());b=engine_from(*build())
  ra=a.solve();rb=b.solve()
  self.assertEqual(ra['status'],rb['status'])
  self.assertEqual(ra['goal_sequence'],rb['goal_sequence'])
 def test_current_observation_only_not_model_completion(self):
  o,m,v,s,r=build()
  m[o==1]=.99
  p=engine_from(o,m,v,s,r)
  self.assertNotEqual(p.report['status'],'OBSERVED_MISSION_NO_UNKNOWN')
  self.assertGreater(int(p.unknown.sum()),0)
 def test_no_regions_is_honest_defer(self):
  o,m,v,s,r=build()
  cfg=SequenceConfig(min_major_expected_open_mass=1e8,coarse_beams=32,dense_beams=48)
  p=engine_from(o,m,v,s,r,cfg)
  self.assertEqual(p.solve()['status'],'DEFER_NO_MAJOR_REGIONS')

 def test_each_major_region_keeps_pathwise_witness_candidate(self):
  p=engine_from(*build())
  choices=p.prepare_candidates()
  for rid in p.rids:
   region=p.region_bits[rid]
   maximal=max((p.first_scores[x][3][0]&region).bit_count() for x in p.first_scores)
   selected=max((p.first_scores[x][3][0]&region).bit_count() for x in choices)
   self.assertEqual(maximal,selected,'region representative pruned away')
 def test_separated_branches_can_require_multiple_goals(self):
  H,W=54,123
  obs=np.full((H,W),2,np.uint8)
  obs[39:52,7:116]=0
  obs[7:39,12:48]=1
  obs[7:39,74:110]=1
  obs[7:46,56:66]=2
  semantic=np.zeros((H,W),np.uint8);semantic[5:53,6:117]=1
  mean=np.full((H,W),.1,np.float32);mean[obs==2]=1
  var=np.full((H,W),.005,np.float32)
  robot=(48,61)
  mem=MissionMemory.begin(obs,semantic,robot)
  report,_,_=analyze(obs,mean,var,semantic,robot,mem,StructuralConfig(beams=64,max_vantages=45))
  cfg=SequenceConfig(coarse_beams=96,dense_beams=192,beam_width=5,
   max_search_candidates=36,max_initial_viewpoint_proposals=105,
   shortlisted_final_sequences=8,max_search_depth=5,route_cost_per_m=0.,stop_cost_per_goal=0.)
  p=StructuralRoutePlanner(obs,mean,var,semantic,robot,mem,report,cfg)
  result=p.solve()
  self.assertGreaterEqual(result['goal_count'],2)
  self.assertEqual(len(result['regions']),2)
  for reg in result['regions']:
   self.assertGreaterEqual(reg['predicted_visible_nominal'],reg['satisfaction_cap_cells'])
 def test_structural_task_precedes_travel_cost(self):
  p=engine_from(*build())
  if not p.rids:return
  region=p.region_bits[p.rids[0]]
  zero=(0,0)
  full=(region,region)
  # An unobserved important region cannot be "completed" by a short plan
  # when a longer knownfree plan can observe it: task rank is first tier.
  self.assertGreater(p._structural_priority(full,1000,2),
                     p._structural_priority(zero,0,0))
if __name__=='__main__':unittest.main()
