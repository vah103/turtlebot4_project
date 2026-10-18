import sys,unittest,json
from pathlib import Path
import numpy as np
from scipy.ndimage import binary_dilation
sys.path.insert(0,str(Path(__file__).parent))
from r4_exit_guard import classify_exit,maybe_recheck
from r4_visibility_gate import pathwise_dominance
from r4_offline import R3,indata,native_inflated,official_wfd,room_at,trial_candidate

class R4GuardTest(unittest.TestCase):
  def get_scene(self,decision):
    x,mean,var,sem,pos,d=indata(R3,decision)
    room=room_at(x,sem,pos)
    group,_=official_wfd(x,room)
    feasible=[p for size,p,_,_ in group if size>10]
    full=np.full((1203,1263),np.inf,dtype=np.float32)
    free=native_inflated(x)
    full[500:703,500:763][free]=1.
    return x,sem,pos,d,room,feasible,full
  def test_d10_transient_start_blocker_detected(self):
    x,sem,pos,d,room,goals,full=self.get_scene(10)
    self.assertEqual(len(goals),3)
    s,out=classify_exit('GLOBAL',goals,pos,(full[500:703,500:763]),room,264)
    self.assertTrue(s.pending)
    self.assertEqual(out['guard'],'TEMPORARY_START_CLEARANCE_BLOCKED')
    self.assertFalse(out['authorizes_local_exit'])
  def test_still_blocked_cannot_switch(self):
    x,sem,pos,d,room,goals,full=self.get_scene(10)
    s,_=classify_exit('GLOBAL',goals,pos,full[500:703,500:763],room,264)
    switched,reason=maybe_recheck(s,pos,full[500:703,500:763],True,5.)
    self.assertFalse(switched)
  def test_d10_recovers_after_motion(self):
    x,sem,pos,d,room,goals,full=self.get_scene(10)
    s,_=classify_exit('GLOBAL',goals,pos,full[500:703,500:763],room,264)
    # Scan at tick 266, not available to planner until robot actually gets there
    od=np.load(R3/'odom.npy')
    t=266
    raw=np.load(R3/f'room_r41/steps/tick_{t:06d}_postscan.npz')['observed'][500:703,500:763]
    observed=np.ones_like(raw,dtype=np.uint8)
    observed[raw==0]=0;observed[raw==1]=2
    inflated=np.where(native_inflated(observed),1.,np.inf)
    next_pose=tuple((od[t]-500).tolist())
    fired,why=maybe_recheck(s,next_pose,inflated,observed[next_pose]==0,5.)
    self.assertTrue(fired,why)
    self.assertEqual(why,'RECHECK_OFFICIAL_LOCAL_WFD_NOW')
    self.assertFalse(s.pending)
  def test_d10_t266_official_candidates_recovered(self):
    from r4_offline import strict_free_nav,official_wfd,room_at
    import cv2
    x,sem,pos,d,room,goals,full=self.get_scene(10)
    t=266
    raw=np.load(R3/f'room_r41/steps/tick_{t:06d}_postscan.npz')['observed'][500:703,500:763]
    obs=np.ones_like(raw,dtype=np.uint8)
    obs[raw==0]=0;obs[raw==1]=2
    od=np.load(R3/'odom.npy')
    newpose=tuple((od[t]-500).tolist())
    newroom=room_at(obs,sem,newpose)
    nav=strict_free_nav(obs,newroom,newpose)
    WFD,edge=official_wfd(obs,newroom)
    accepted=[center for size,center,lo,hi in WFD if size>10 and nav[center]]
    self.assertEqual(len(accepted),3)
    self.assertTrue(nav[newpose])
  def test_not_imply_official_completion(self):
    x,sem,pos,d,room,goals,full=self.get_scene(10)
    s,_=classify_exit('GLOBAL',goals,pos,full[500:703,500:763],room,264)
    self.assertFalse(s.rechecks)
    self.assertTrue(s.pending)
  def test_no_guard_when_no_frontier(self):
    x,sem,pos,d,room,goals,full=self.get_scene(10)
    s,out=classify_exit('GLOBAL',[],pos,full[500:703,500:763],room,264)
    self.assertFalse(s.pending)
    self.assertEqual(out['guard'],'OFFICIAL_WFD_EMPTY')
  def test_no_guard_on_legal_local(self):
    x,sem,pos,d,room,goals,full=self.get_scene(9)
    s,out=classify_exit('LOCAL',goals,pos,full[500:703,500:763],room,198)
    self.assertFalse(s.pending)
  def test_nonnative_pose_denied(self):
    x,sem,pos,d,room,goals,full=self.get_scene(10)
    s,_=classify_exit('GLOBAL',goals,pos,full[500:703,500:763],room,264)
    invalid=np.full_like(full[500:703,500:763],np.inf)
    pose=(11,155)
    r,_=maybe_recheck(s,pose,invalid,True,5.)
    self.assertFalse(r)
  def test_C_r3_d03_rejected_due_to_visibility_loss(self):
    t=trial_candidate(R3,3,(180,133))
    self.assertGreater(t['lost_nominal_cells'],0)
    self.assertFalse(t['R4_safe_nonregression_pass'])
  def test_C_r3_d05_rejected_due_to_visibility_loss(self):
    t=trial_candidate(R3,5,(186,203))
    self.assertGreater(t['lost_stress_cells'],0)
    self.assertFalse(t['R4_safe_nonregression_pass'])
  def test_dominance_rejects_unknown_coverage_replacement(self):
    o=np.zeros((3,3),bool);a=o.copy();o[0,0]=True;a[0,1]=True
    ans=pathwise_dominance([(1,1),(1,2)],[(1,1),(1,0)],o,o,a,a)
    self.assertFalse(ans['allow'])
  def test_dominance_accepts_shorter_same_or_more_coverage(self):
    o=np.zeros((3,3),bool);a=o.copy();o[0,0]=True;a[0,0]=True;a[0,1]=True
    ans=pathwise_dominance([(1,1),(1,2),(1,3)],[(1,1),(1,2)],o,o,a,a)
    self.assertTrue(ans['allow'])
if __name__=='__main__':unittest.main(verbosity=2)
