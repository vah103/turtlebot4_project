import unittest
from dataclasses import replace
import numpy as np

from mapex_g2_v1 import G2Config,G2Snapshot,plan,assess_replan


class G2Tests(unittest.TestCase):
    def setUp(self):
        # Two potentially occluded parts of one accepted Room Node.
        h,w=26,40
        self.obs=np.ones((h,w),np.uint8)*2
        self.sem=np.ones((h,w),np.uint8)
        self.obs[4:22,4:36]=1
        self.obs[13:17,5:35]=0
        self.obs[17:20,5:14]=0
        self.obs[17:20,26:35]=0
        self.obs[4:13,18]=2
        self.obs[18:22,18]=2
        self.mean=np.ones((h,w),np.float32)*.08
        self.variance=np.ones((h,w),np.float32)*.04
        self.robot=(14,7)
        self.cfg=G2Config(resolution_m=.1,lidar_range_m=4.,planning_beams=48,
            ray_step_cells=1,scan_every_m=.5,candidate_spacing_m=.4,clearance_m=.1,
            max_candidates=100,beam_width=3,branch_width=7,
            unknown_penalty_s_per_cell=.4,risk_penalty_s_per_cell=.0,
            goal_stop_time_s=.2,min_net_improvement_s=.1)
        self.s=G2Snapshot(self.obs,self.mean,self.variance,self.sem,self.robot)

    def test_deterministic_planning_and_all_routes_known_free(self):
        p=plan(self.s,self.cfg)
        self.assertGreaterEqual(p['goal_count'],1)
        self.assertEqual(p['goal_count'],len(p['goal_sequence']))
        for leg in p['route_segments']:
            self.assertTrue(leg['goal_is_observed_free'])
            self.assertTrue(leg['route_is_observed_free'])
            for r,c in leg['path_rc']:
                self.assertEqual(self.obs[r,c],0)
        self.assertLessEqual(p['cost']['objective_s_equivalent'],p['baseline_cost']['objective_s_equivalent'])
        second=plan(self.s,self.cfg)
        self.assertEqual(p['goal_sequence'],second['goal_sequence'])

    def test_two_goals_can_beat_one_when_geometry_requires_it(self):
        observed=np.ones((24,40),np.uint8)
        observed[10:16,4:33]=0
        mean=np.zeros(observed.shape,np.float32)
        snap=G2Snapshot(observed,mean,mean,np.ones_like(observed),(12,6))
        cfg=G2Config(lidar_range_m=4,planning_beams=64,clearance_m=0)
        p=plan(snap,cfg)
        self.assertGreaterEqual(p['goal_count'],2)
        self.assertEqual(len(p['route_segments']),p['goal_count'])

    def test_updated_mean_triggers_replan_without_ground_truth(self):
        nm=self.mean.copy()
        nm[9,12]=.85
        out=assess_replan(self.s,self.obs,after_mean=nm,config=self.cfg)
        self.assertTrue(out['needs_replan'])
        self.assertIn('LAMA_PREDICTION_CHANGED',out['reasons'])

    def test_replan_obstacle(self):
        p=plan(self.s,self.cfg)
        assert p['route_segments']
        updated=self.obs.copy()
        # Known UNKNOWN cell in current room -> newly observed wall.
        updated[11,12]=2
        event=assess_replan(self.s,updated,config=self.cfg)
        self.assertTrue(event['needs_replan'])
        self.assertIn('NEW_OBSERVED_OBSTACLE',event['reasons'])
        self.assertEqual(event['required_action'],'RERUN_LAMA_AND_ROOM_NODE_THEN_G2')

    def test_no_change_keep_chain(self):
        event=assess_replan(self.s,self.obs.copy(),config=self.cfg)
        self.assertFalse(event['needs_replan'])

    def test_unknown_not_drivable_despite_predictions(self):
        # Make nearly everything predicted free, but only one free island.
        observed=np.ones((16,16),np.uint8)
        observed[7:9,7:9]=0
        semantic=np.ones_like(observed)
        mean=np.zeros((16,16),np.float32)
        variance=np.zeros_like(mean)
        snap=G2Snapshot(observed,mean,variance,semantic,(7,7))
        p=plan(snap,replace(self.cfg,clearance_m=0))
        for leg in p['route_segments']:
            self.assertTrue(all(observed[tuple(rc)]==0 for rc in leg['path_rc']))
        self.assertNotEqual(p['status'],'COMPLETE_OBSERVED')

    def test_complete_only_with_observed_evidence(self):
        obs=np.zeros((12,12),np.uint8)
        mean=np.zeros_like(obs,dtype=np.float32)
        sem=np.ones_like(obs)
        snap=G2Snapshot(obs,mean,mean,sem,(5,5))
        p=plan(snap,replace(self.cfg,clearance_m=0))
        self.assertEqual(p['status'],'COMPLETE_OBSERVED')
        self.assertEqual(p['goal_count'],0)

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            plan(replace(self.s,observed=self.obs*5),self.cfg)
        with self.assertRaises(ValueError):
            plan(replace(self.s,robot_rc=(0,0)),self.cfg)

    def test_ensemble_and_variance(self):
        ens=np.stack([self.mean,self.mean*.5,np.minimum(self.mean*2,1)],axis=0)
        p=plan(replace(self.s,ensemble=ens),self.cfg)
        self.assertEqual(p['provenance']['ensemble_members'],3)
        self.assertGreaterEqual(p['cost']['uncertainty_gap_weight'],0)

    def test_remaining_planned_route_invalidates(self):
        after=self.obs.copy()
        # changing a known free cell is intentionally forbidden, a bad sensor update
        after[14,9]=2
        with self.assertRaises(ValueError):
            assess_replan(self.s,after,remaining_route_rc=[(14,9)],config=self.cfg)


if __name__=='__main__': unittest.main()
