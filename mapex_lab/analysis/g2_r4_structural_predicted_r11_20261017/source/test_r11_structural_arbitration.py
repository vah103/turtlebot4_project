"""Pure deterministic contract tests; uses a fake G2-R3 planner, no GT or native run."""
import sys,types,unittest
import numpy as np
pkg=types.ModuleType('mapex_g2_r3')
seq=types.ModuleType('mapex_g2_r3.sequence')
seq.StructuralRoutePlanner=object
seq.SequenceConfig=lambda **kw:types.SimpleNamespace(resolution_m=.1)
sys.modules.setdefault('mapex_g2_r3',pkg)
sys.modules.setdefault('mapex_g2_r3.sequence',seq)
from g2_structural_arbitration import arbitrate,choose_structural_backstop


class FakePlanner:
    def __init__(self,mask_by_goal,candidates=None,ray=0b111,endpoint_by_goal=None):
        self.robot=(2,2)
        self.cfg=types.SimpleNamespace(resolution_m=.1)
        self.masks=mask_by_goal
        self.choices=candidates or []
        self.endpoint=endpoint_by_goal or {}
        self.coarse=types.SimpleNamespace(sight=lambda p:(self.endpoint.get(tuple(p),ray),self.endpoint.get(tuple(p),ray)))
        self.major=['region']
    def _leg(self,src,dst,ray):
        x=self.masks.get(tuple(dst))
        return (20,(x,x),[]) if x is not None else None
    def quality(self,masks,length,num_goals):
        q=(masks[0].bit_count()+masks[1].bit_count())/2
        return q,q-.01*length-.45*num_goals,10-q
    def _structural_priority(self,masks,length,n):
        q=masks[0].bit_count()
        return (1 if q>=3 else 0,q,-length,-n)
    def prepare_candidates(self):
        return self.choices


class R11ArbitrationTests(unittest.TestCase):
    def setUp(self):
        o=np.zeros((12,12),np.uint8)
        self.base=(4,4);self.sub=(4,5);self.target=(4,7)
        o[self.target]=1
        self.kw=dict(obs=o,mean=np.zeros(o.shape,np.float32),
            variance=np.zeros(o.shape,np.float32),
            semantic=np.ones(o.shape,np.uint8),robot=(2,2),
            memory=None,report=None,native_walkable=np.ones(o.shape,bool),
            baseline_goal=self.base,
            predicted_proposal={'safe_waypoint_rc':list(self.sub),
                'predicted_target_rc':list(self.target),
                'hypothetical_unknown_steps':5})
    def test_reject_bad_actual_subgoal_regardless_of_target_potential(self):
        out=arbitrate(**self.kw,planner=FakePlanner({self.base:0b111111,self.sub:0b1},ray=0b1111111111111))
        self.assertEqual(out['decision'],'KEEP_R4_GOAL')
        self.assertIn('SAFE_PREFIX_',out['reason'])
    def test_allow_good_subgoal_with_meaningful_extra_structural_gain(self):
        out=arbitrate(**self.kw,planner=FakePlanner({self.base:0b111,self.sub:0b111},ray=0b11111111))
        self.assertEqual(out['decision'],'USE_PREDICTED_GOAL')
    def test_bad_endpoint_viewpoint_is_rejected_even_if_pathwise_good(self):
        p=FakePlanner({self.base:0b111111,self.sub:0b111111},
            ray=0b111111111111,
            endpoint_by_goal={self.base:0b111111,self.sub:0b1})
        out=arbitrate(**self.kw,planner=p)
        self.assertEqual(out['decision'],'KEEP_R4_GOAL')
        self.assertEqual(out['reason'],'SAFE_WAYPOINT_ARRIVAL_VIEW_QUALITY_TOO_LOW')
    def test_reject_illusory_future_gain(self):
        out=arbitrate(**self.kw,planner=FakePlanner({self.base:0b111,self.sub:0b111},ray=0b111))
        self.assertEqual(out['reason'],'NO_MEANINGFUL_DISCOUNTED_FUTURE_STRUCTURAL_GAIN')
    def test_never_move_through_unknown(self):
        self.kw['native_walkable'][self.sub]=False
        out=arbitrate(**self.kw,planner=FakePlanner({self.base:0b111,self.sub:0b111},ray=0b11111111))
        self.assertEqual(out['reason'],'GOAL_NOT_NATIVE_CERTIFIED_FREE')
    def test_reject_observed_wall_predicted_target(self):
        self.kw['obs'][self.target]=2
        out=arbitrate(**self.kw,planner=FakePlanner({self.base:0b111,self.sub:0b111},ray=0b11111111))
        self.assertEqual(out['decision'],'KEEP_R4_GOAL')
    def test_reject_target_outside_room(self):
        self.kw['semantic'][self.target]=2
        out=arbitrate(**self.kw,planner=FakePlanner({self.base:0b111,self.sub:0b111},ray=0b11111111))
        self.assertEqual(out['decision'],'KEEP_R4_GOAL')
    def test_backstop_uses_r3_structural_witnesses_not_wfd(self):
        p=FakePlanner({(3,4):0b111,(5,5):0b1111},candidates=[(3,4),(5,5)])
        x=choose_structural_backstop(
            self.kw['obs'],self.kw['mean'],self.kw['variance'],
            self.kw['semantic'],(2,2),None,None,
            self.kw['native_walkable'],planner=p)
        self.assertEqual(x['goal_core_rc'],[5,5])
        self.assertFalse(x['used_WFD_for_goal_generation'])
    def test_no_gain_means_no_claimed_good_viewpoint(self):
        p=FakePlanner({(3,4):0},candidates=[(3,4)])
        x=choose_structural_backstop(
            self.kw['obs'],self.kw['mean'],self.kw['variance'],
            self.kw['semantic'],(2,2),None,None,
            self.kw['native_walkable'],planner=p)
        self.assertIsNone(x)

if __name__=='__main__':unittest.main()
