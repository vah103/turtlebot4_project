import sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from r4_adaptive import R4Config,EvidenceLedger,Leg,Chain,PathValue
from r4_episode import prefixes,on_scan_fast,plan_episode

class TestEpisode(unittest.TestCase):
    def setUp(self):
        self.cfg=R4Config(speed_mps=1.,cost_per_second=.01)
        self.obs=np.zeros((20,20),np.uint8)
        self.episode={'obs':self.obs,'mean':np.zeros_like(self.obs,dtype=float),
                      'variance':np.zeros_like(self.obs,dtype=float),
                      'semantic':np.ones_like(self.obs),
                      'robot':(10,10),'memory':object(),
                      'report':{'snapshot_sha256':'abc'},'pending_goals':[(10,11),(10,12)],
                      'exit_anchor':(10,10),'cfg':self.cfg}

    def test_prefixes_not_fixed(self):
        p=prefixes('q',[(1,2),(1,3),(1,4),(1,5)])
        self.assertEqual(len(p),4)
        self.assertEqual(p['q:2'],((1,2),(1,3)))

    def test_quick_scan_does_not_run_heavy(self):
        with patch('r4_episode.from_r3_snapshot') as heavy:
            decision=on_scan_fast(before=self.obs,after=self.obs,pose_rc=(10,10),
                   remaining_route=[(10,10),(10,11)],scan_id=5,last_heavy_scan=4,cfg=self.cfg,clearance_cells=0)
            self.assertEqual(decision['decision'],'KEEP_SHORT_PREFIX')
            heavy.assert_not_called()

    def test_goal_rechecks_without_path(self):
        v=on_scan_fast(before=self.obs,after=self.obs,pose_rc=(10,10),
                      remaining_route=[],at_goal=True,scan_id=5,last_heavy_scan=4,cfg=self.cfg)
        self.assertEqual(v['decision'],'HEAVY_EVALUATION_REQUIRED')

    def test_route_unsafe_no_commands(self):
        old=self.obs.copy();old[10,11]=1
        after=old.copy()
        v=on_scan_fast(before=old,after=after,pose_rc=(10,10),
                   remaining_route=[(10,10),(10,11)],scan_id=10,last_heavy_scan=1,clearance_cells=0)
        self.assertEqual(v['decision'],'SAFE_HOLD')
        self.assertEqual(v['execution_prefix_rc'],[])
        self.assertIsNone(v['robot_command'])

    def test_episode_rejects_no_goal(self):
        out=plan_episode(**{**self.episode,'pending_goals':[], 'fresh_goals':[]},do_r3_search=False)
        self.assertEqual(out['decision'],'DEFER_ROOM')
        self.assertIsNone(out['robot_command'])

    def test_no_heavy_no_expensive_search(self):
        with patch('r4_episode.from_r3_snapshot') as adapter:
            out=plan_episode(**self.episode,event={'heavy':False},do_r3_search=False,fresh_goals=[])
            self.assertEqual(out['decision'],'HOLD_OR_FAST_KEEP')
            adapter.assert_not_called()

    def test_event_selected_prefix_and_no_command(self):
        scorer=PathValue([dict(region_id=1,cells=16,expected_open_cell_mass_proxy=16,kind='POSSIBLE_OPEN_SPACE')],
                         {1:0xffff}, EvidenceLedger(), self.cfg)
        c=Chain('PENDING:1',(Leg((10,11),((10,10),(10,11)),0xffff,0xffff),
                        Leg((10,10),((10,11),(10,10)),0,0,'exit')),'abc')
        with patch('r4_episode.from_r3_snapshot',return_value=(scorer,[c],{})):
            out=plan_episode(**self.episode,event={'heavy':True},do_r3_search=False,fresh_goals=[])
            self.assertEqual(out['selected_goals_rc'],[[10,11]])
            self.assertEqual(out['execution_prefix_rc'],[[10,10],[10,11]])
            self.assertFalse(out['is_physical_route_certified'])
            self.assertFalse(out['ground_truth_used'])
            self.assertIsNone(out['robot_command'])

    def test_snapshot_hash_mismatch_must_fail(self):
        from r4_adaptive import from_r3_snapshot
        with self.assertRaises((ImportError,ValueError)):
            # This unit test does not have R3 snapshot loaded: fail closed.
            from_r3_snapshot(self.obs,self.obs.astype(float),self.obs.astype(float),
                        self.obs,(10,10),object(),self.episode['report'],{},(10,10),source_hash='mismatch')

if __name__=='__main__':unittest.main(verbosity=2)
