import unittest,os,sys,cv2,numpy as np
from g2_predicted_goals import find_predicted
class TestPredicted(unittest.TestCase):
 def fixture(self):
  a=np.full((85,85),2,np.uint8)
  a[5:80,5:80]=1
  a[10:38,10:38]=0
  mean=np.zeros_like(a,np.float32)
  var=np.zeros_like(a,np.float32)
  sem=np.ones_like(a,np.uint8)
  scope=cv2.erode(np.uint8(a==0),np.ones((5,5),np.uint8)).astype(bool)
  return a,mean,var,sem,scope
 def test_goal_never_assumed_observed_free(self):
  a,m,v,s,scope=self.fixture();t=(24,24)
  x=find_predicted(a,m,v,s,t,s==1,(30,30),None,scope)
  if x is not None:
   self.assertEqual(int(a[tuple(x['predicted_target_rc'])]),1)
   self.assertEqual(int(a[tuple(x['safe_waypoint_rc'])]),0)
   self.assertTrue(scope[tuple(x['safe_waypoint_rc'])])
 def test_uncertain_unknown_never_promoted_to_safe(self):
  a,m,v,s,scope=self.fixture();t=(24,24)
  m[a==1]=.8
  self.assertIsNone(find_predicted(a,m,v,s,t,s==1,(30,30),None,scope))
 def test_wall_target_invalid(self):
  a,m,v,s,scope=self.fixture();t=(24,24)
  prev=(50,50);a[prev]=2
  x=find_predicted(a,m,v,s,t,s==1,(30,30),prev,scope)
  self.assertTrue(x is None or x.get('predicted_target_rc')!=list(prev))
 def test_native_scope_required(self):
  a,m,v,s,scope=self.fixture()
  with self.assertRaises(ValueError):find_predicted(a,m,v,s,(24,24),s==1,(30,30),None,None)
 def test_source_frozen_room_d02(self):
  r='/home/dell/mx072_g2_r4_exact_run08_com1_20261017/native_g2_r4_goal_exact_run08/G2_R4_EXACT_GOAL_RUN08_visvarprob/room_r41/inference'
  with np.load('%s/decision_000002_input.npz'%r,allow_pickle=False) as f:
   o=f['observed'].copy();m=f['mean'].copy();v=f['variance'].copy()
  with np.load('%s/decision_000002_semantic.npz'%r,allow_pickle=False) as f:s=f['semantic'].copy()
  t=(147,111)
  known=(o==0)&(s==1)
  scope=cv2.erode(known.astype('uint8'),np.ones((5,5),np.uint8))>0
  scope[t]=True
  x=find_predicted(o,m,v,s,t,s==1,(171,135),None,scope)
  self.assertIsNotNone(x)
  self.assertEqual(o[tuple(x['predicted_target_rc'])],1)
  self.assertEqual(o[tuple(x['safe_waypoint_rc'])],0)
  self.assertTrue(scope[tuple(x['safe_waypoint_rc'])])
  print('D02_PROPOSED',x)
if __name__=='__main__':unittest.main()
