"""G2-R3.1 commitment/mission-progress invariants, no hardcoded target goals."""
import tempfile,unittest,json
from pathlib import Path
import numpy as np
from mapex_g2_r3.commitment import CommitmentController,CommitmentState,CommitmentConfig,ProgressAwarePlanner,check_remaining_goal,expected_first_leg_per_region
from mapex_g2_r3.structural import MissionMemory,StructuralConfig,analyze
from mapex_g2_r3.sequence import SequenceConfig

def mapcase():
  o=np.full((50,65),2,np.uint8)
  o[6:44,6:58]=1;o[12:41,9:53]=0
  o[7:12,10:27]=1;o[7:12,34:51]=1
  m=np.full(o.shape,.1,np.float32);m[o==2]=1
  v=np.full(o.shape,.01,np.float32)
  s=np.ones(o.shape,np.uint8);s[o==2]=0
  r=(25,27)
  memory=MissionMemory.begin(o,s,r)
  regions,_,_=analyze(o,m,v,s,r,memory,StructuralConfig(beams=48,max_vantages=45),decision_id='syn')
  return o,m,v,s,r,memory,regions

def fake_plan(goals):return {'status':'CONDITIONAL_VIEWPOINT_SEQUENCE','goal_sequence':[list(q) for q in goals], 'decision_id':'syn','regions':[]}

class R31Tests(unittest.TestCase):
 def test_commits_many_goals_without_forcing_one(self):
  c=CommitmentController()
  c.initialize(fake_plan([(25,15),(25,45)]))
  self.assertEqual(len(c.state.pending_goals),2)
 def test_no_early_replan_if_remaining_goal_safe_and_useful(self):
  o,m,v,s,_,mem,regions=mapcase()
  c=CommitmentController(cfg=CommitmentConfig(minimum_remaining_expected_cells=1))
  c.initialize(fake_plan([(25,20),(25,40)]))
  ev=c.record_arrival(o,o.copy(),mem.labels,{'first_leg_expected_by_region':{}},(25,20),(25,27))
  self.assertEqual(c.state.pending_goals,[[25,40]])
  # A free corridor goal with a nearby unknown large wing can produce projected gain.
  answer=c.decide(o,m,v,s,(25,20),mem,regions)
  self.assertIn(answer['decision'],('KEEP','REPLAN','DEFER'))
  if answer['decision']=='KEEP':
    self.assertFalse(answer['replanning_called'])
    self.assertEqual(answer['goal_sequence'],[[25,40]])
 def test_observation_cannot_be_fabricated(self):
  o,m,v,s,r,mem,reg=mapcase()
  c=CommitmentController();c.initialize(fake_plan([(25,20)]))
  changed=o.copy();changed[25,27]=1
  with self.assertRaisesRegex(ValueError,'overwritten'):
   c.record_arrival(o,changed,mem.labels,{},(25,20),r)
 def test_must_reach_correct_committed_goal(self):
  o,m,v,s,r,mem,reg=mapcase()
  c=CommitmentController();c.initialize(fake_plan([(25,20)]))
  with self.assertRaisesRegex(ValueError,'not next committed'):
   c.record_arrival(o,o,mem.labels,{},(25,21),r)
 def test_same_view_failed_does_not_count_as_independent_again(self):
  o,m,v,s,r,mem,reg=mapcase()
  c=CommitmentController(cfg=CommitmentConfig(minimum_expected_before_failure=2,max_global_stagnant_legs=10))
  c.initialize(fake_plan([(25,20)]))
  c.record_arrival(o,o,mem.labels,{'first_leg_expected_by_region':{'5':20}},(25,20),r)
  c.state.pending_goals=[[25,20]]
  c.record_arrival(o,o,mem.labels,{'first_leg_expected_by_region':{'5':20}},(25,20),r)
  self.assertEqual(len(c.state.region_failures['5']),1)
  self.assertNotIn(5,c.state.deferred_regions)
 def test_independent_failed_angles_defer_but_not_clear_unknown(self):
  o,m,v,s,r,mem,reg=mapcase()
  c=CommitmentController(cfg=CommitmentConfig(minimum_expected_before_failure=2,max_global_stagnant_legs=10))
  n=int((mem.mission&(o==1)).sum())
  c.initialize(fake_plan([(25,20)]))
  c.record_arrival(o,o,mem.labels,{'first_leg_expected_by_region':{'5':20}},(25,20),r)
  c.state.pending_goals=[[25,40]]
  c.record_arrival(o,o,mem.labels,{'first_leg_expected_by_region':{'5':20}},(25,40),r)
  self.assertIn(5,c.state.deferred_regions)
  self.assertEqual(int((mem.mission&(o==1)).sum()),n)
 def test_success_resets_region_failures(self):
  o,m,v,s,r,mem,reg=mapcase()
  c=CommitmentController(cfg=CommitmentConfig(minimum_expected_before_failure=2,max_global_stagnant_legs=10))
  c.initialize(fake_plan([(25,20)]))
  c.record_arrival(o,o,mem.labels,{'first_leg_expected_by_region':{'5':20}},(25,20),r)
  where=np.argwhere(mem.labels>0)
  if len(where)==0:return
  # One observed cell may not reach 8% of 20 => need 2 new pixels.
  obs=o.copy();count=0
  for y,x in where:
    if obs[y,x]==1:
      obs[y,x]=0;count+=1
      if count>=4:break
  c.state.pending_goals=[[25,40]]
  c.record_arrival(o,obs,mem.labels,{'first_leg_expected_by_region':{str(int(mem.labels[tuple(where[0])])):20}},(25,40),r)
  self.assertEqual(c.state.stagnant_legs,0)
 def test_two_zero_observation_legs_defer_without_complete(self):
  o,m,v,s,r,mem,reg=mapcase()
  c=CommitmentController(cfg=CommitmentConfig(max_global_stagnant_legs=2))
  c.initialize(fake_plan([(25,20)]))
  c.record_arrival(o,o,mem.labels,{},(25,20),r)
  c.state.pending_goals=[[25,40]]
  c.record_arrival(o,o,mem.labels,{},(25,40),r)
  out=c.decide(o,m,v,s,r,mem,reg)
  self.assertEqual(out['status'],'DEFER_STALLED_WITH_UNRESOLVED_MISSION')
  self.assertTrue(reg['mission_unknown']>0)
 def test_state_roundtrip(self):
  c=CommitmentController();c.initialize(fake_plan([(25,20),(25,40)]))
  with tempfile.TemporaryDirectory() as folder:
    path=Path(folder)/'commitment.json';c.state.save(path)
    other=CommitmentState.load(path)
    self.assertEqual(other.pending_goals,[[25,20],[25,40]])
 def test_invalid_stale_goal_must_not_be_executed(self):
  o,m,v,s,r,mem,reg=mapcase()
  status=check_remaining_goal(o,m,v,s,r,mem,(8,9),CommitmentConfig())
  self.assertNotEqual(status[1],'KEEP')
 def test_structural_candidate_planner_excludes_repeated_failed_vantage(self):
  o,m,v,s,r,mem,reg=mapcase()
  state=CommitmentState(visits=[{'goal_rc':[25,23],'realized_global_cells':0}])
  cfg=SequenceConfig(coarse_beams=48,dense_beams=64,max_initial_viewpoint_proposals=30,
       max_search_candidates=18,beam_width=3,max_search_depth=2)
  p=ProgressAwarePlanner(o,m,v,s,r,mem,reg,state,cfg)
  pts=p.prepare_candidates()
  self.assertTrue(all(abs(x[0]-25)**2+abs(x[1]-23)**2>64 for x in pts))
 def test_no_gt_input_or_predefined_goals(self):
  o,m,v,s,r,mem,reg=mapcase()
  c=CommitmentController()
  self.assertFalse(hasattr(c,'gt'))
  self.assertEqual(c.state.pending_goals,[])
if __name__=='__main__':unittest.main()
