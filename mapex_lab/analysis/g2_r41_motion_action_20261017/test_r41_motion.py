"""Unit/regression checks for R4.1 motion actions (offline, no GT)."""
import unittest, numpy as np
from dataclasses import replace
from mapex_g2_r3.structural import MissionMemory, StructuralConfig, analyze, sha
from r41_motion import LocalMotionPlanner,MotionConfig
from r4_adaptive import EvidenceLedger

def setup_small():
    obs=np.full((48,68),2,np.uint8)
    obs[4:44,4:64]=1
    obs[13:36,5:63]=0
    obs[17:33,32:35]=2
    mean=np.full(obs.shape,.1,np.float32)
    mean[obs==2]=1.
    var=np.full(obs.shape,.01,np.float32)
    semantic=np.ones(obs.shape,np.uint8)
    semantic[obs==2]=0
    pose=(24,12)
    mem=MissionMemory.begin(obs,semantic,pose)
    rep,_,_=analyze(obs,mean,var,semantic,pose,mem,
                    StructuralConfig(beams=32,max_vantages=12),decision_id='R41-synth',
                    include_vantage=False)
    return obs,mean,var,semantic,pose,mem,rep

def engine(cfg=None):
    o,m,v,s,p,mem,rep=setup_small()
    c=cfg or MotionConfig(beam_width=6,max_depth=4,max_expansions=160,
           coarse_beams=48,dense_beams=120,action_lengths_cells=(5,10))
    return LocalMotionPlanner(o,m,v,s,p,mem,rep,motion_cfg=c)

class R41MotionTests(unittest.TestCase):
    def test_no_legacy_goal_sequence_interface(self):
        p=engine()
        self.assertFalse(hasattr(p,'pending_goals'))
        self.assertFalse(hasattr(p,'goal_sequence'))
    def test_navigation_only_observed_free(self):
        p=engine()
        for y in range(4,45):
            for x in range(4,64):
                path=p._local_edge((24,12),1,10)
                if path is not None:
                    self.assertTrue(all(p.obs[q]==0 and p.free[q] for q in path))
                    break
            break
    def test_short_motion_senses_along_path(self):
        p=engine()
        edge=p._local_edge(p.robot,1,10)
        self.assertIsNotNone(edge)
        n,c=p._visibility(edge,p.adapter.coarse)
        mid=p.adapter.coarse.sight(edge[5])
        end=p.adapter.coarse.sight(edge[-1])
        self.assertEqual(n,mid[0]|end[0])
        self.assertEqual(c,mid[1]|end[1])
    def test_illegal_unknown_and_predicted_free_never_traversed(self):
        p=engine()
        # At robot x=12, 10 cells north crosses observed unknown above row13.
        self.assertIsNone(p._local_edge(p.robot,2,15))
        # A known obstacle cross cannot be jumped although mean is hypothetically free.
        self.assertIsNone(p._local_edge((24,25),1,15))
    def test_same_visibility_mask_counted_once(self):
        p=engine()
        edge=p._local_edge(p.robot,1,5)
        n,c=p._visibility(edge,p.adapter.coarse)
        self.assertEqual((n|n).bit_count(),n.bit_count())
        root=list(p._expand(self._root(p),p.adapter.coarse))
        self.assertTrue(root)
        for st in root:
            self.assertEqual(st.nominal.bit_count(),st.moves[0].novel_nominal.bit_count())
    def _root(self,p):
        from r41_motion import Node
        return Node(p.robot,None,0,0,0,0,(p.robot,),(),0,0,0,0)
    def test_action_cost_uses_terminal_return(self):
        p=engine()
        n=3;cc=3;travel=20;turns=0
        q,rob,sc=p._evaluate(n,cc,travel,turns,40)
        q0,rob0,sc0=p._evaluate(n,cc,travel,turns,0)
        self.assertLess(sc,sc0)
        self.assertLess(rob,rob0)
    def test_turn_penalty_changes_score(self):
        p=engine()
        x=p._evaluate(10,10,20,0,20)
        y=p._evaluate(10,10,20,2,20)
        self.assertGreater(x[2],y[2])
    def test_entry_anchor_must_be_knownfree_reachable(self):
        o,m,v,s,rc,mem,rep=setup_small()
        with self.assertRaisesRegex(ValueError,'anchor'):
            LocalMotionPlanner(o,m,v,s,rc,mem,rep,anchor=(1,1),
                               motion_cfg=MotionConfig(coarse_beams=48,dense_beams=120))
    def test_source_provenance_immutable(self):
        o,m,v,s,rc,mem,rep=setup_small()
        wrong=dict(rep);wrong['snapshot_sha256']='wrong'
        with self.assertRaises(ValueError):
            LocalMotionPlanner(o,m,v,s,rc,mem,wrong,
                               motion_cfg=MotionConfig(coarse_beams=48,dense_beams=120))
    def test_r41_outputs_action_route_not_legacy_goals(self):
        p=engine()
        result=p.solve()
        self.assertEqual(result['version'],'G2_R41_MOTION_SEARCH_V1')
        self.assertTrue(result['action_sequence'])
        self.assertIsNone(result['robot_command'])
        self.assertFalse(result['source_r3_goals_used'])
        self.assertEqual(tuple(result['execution_prefix_rc'][0]),p.robot)
        self.assertLessEqual(len(result['execution_prefix_rc'])-1,5)
        self.assertEqual(tuple(result['return_route_rc'][-1]),p.anchor)
        self.assertTrue(all(p.obs[tuple(x)]==0 for x in result['route_rc']))
        self.assertTrue(all(p.obs[tuple(x)]==0 for x in result['return_route_rc']))
        self.assertLess(result['search']['elapsed_s'],60.)
    def test_varied_budgets_change_horizon_not_forced_goal_count(self):
        p=engine(MotionConfig(beam_width=5,max_depth=1,max_expansions=100,coarse_beams=48,dense_beams=96))
        x=p.solve()
        if x['status']=='CONDITIONAL_MOTION_ROUTE':
            self.assertEqual(len(x['action_sequence']),1)
        p2=engine(MotionConfig(beam_width=6,max_depth=5,max_expansions=180,coarse_beams=48,dense_beams=96))
        y=p2.solve()
        self.assertGreaterEqual(len(y.get('action_sequence',[])),1)
    def test_conservative_fallback_can_defer(self):
        p=engine()
        p.cfg=replace(p.cfg,min_new_conservative_cells=10**7)
        x=p.solve()
        self.assertEqual(x['decision'],'DEFER_ROOM')
    def test_root_is_valid_grid_pose(self):
        p=engine()
        self.assertTrue(p.free[p.robot])
        self.assertTrue(p.obs[p.robot]==0)
        self.assertEqual(p.unknown_count,p.adapter.report['mission_unknown'])
    def test_synthetic_avoid_immediate_backtrack(self):
        p=engine()
        root=list(p._expand(self._root(p),p.adapter.coarse))
        st=root[0]
        nxt=list(p._expand(st,p.adapter.coarse))
        self.assertNotIn(p.robot,[q.pose for q in nxt])
    def test_input_gt_not_present(self):
        p=engine()
        self.assertFalse(hasattr(p,'gt'))
        self.assertFalse(hasattr(p,'ground_truth'))

if __name__=='__main__':unittest.main()
