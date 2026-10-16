"""R2 tests on synthetic data, not D02-tuned assertions."""
import unittest
from dataclasses import replace
import numpy as np
from mapex_g2_v1.planner import (G2Config,G2Snapshot,ProbabilisticVisibility,
                                  _room,_cost,plan)
from mapex_g2_v1.visualization import make_coverage


class R2Tests(unittest.TestCase):
    def sample(self):
        o=np.ones((30,35),np.uint8)*2
        o[3:26,3:30]=1
        o[13:20,4:29]=0
        m=np.zeros(o.shape,np.float32)
        v=np.zeros(o.shape,np.float32)
        sem=np.ones(o.shape,np.uint8)
        return G2Snapshot(o,m,v,sem,(16,10))

    def test_union_is_idempotent_across_scans_and_legs(self):
        s=self.sample();cfg=G2Config(lidar_range_m=4,planning_beams=100,final_beams=200,
              candidate_spacing_m=.4,clearance_m=0)
        vis=ProbabilisticVisibility(s,_room(s),cfg)
        a=vis.view((16,11)); b=vis.view((16,11))
        self.assertTrue(np.array_equal(a,b))
        cov,n=vis.project_leg([(16,11),(16,11)])
        self.assertEqual(n,1)
        self.assertTrue(np.array_equal(a,cov))
        self.assertTrue(np.array_equal(a|a,a))
        self.assertEqual(vis.gain(cov)[0],vis.gain(cov|cov)[0])

    def test_unknown_predicted_wall_is_still_observable_as_first_hit(self):
        s=self.sample()
        m=s.mean.copy();m[12,10]=1.
        cfg=G2Config(lidar_range_m=3.,planning_beams=360,ray_step_cells=.5,
                    clearance_m=0)
        vis=ProbabilisticVisibility(replace(s,mean=m),_room(s),cfg)
        output=vis.view((16,10))
        yy,xx=np.where(vis.target)
        ind={(int(y),int(x)):i for i,(y,x) in enumerate(zip(yy,xx))}
        self.assertTrue(bool(output[0,ind[(12,10)]]))
        # Interior behind that wall at col10 cannot be seen through it from below.
        self.assertFalse(bool(output[0,ind[(10,10)]]))

    def test_pessimistic_fallback_is_explicit_not_fabricated_ensemble(self):
        s=self.sample()
        m=s.mean.copy();m[8:14,15]=.35
        v=s.variance.copy();v[8:14,15]=.08
        cfg=G2Config(lidar_range_m=4,planning_beams=180)
        vis=ProbabilisticVisibility(replace(s,mean=m,variance=v),_room(s),cfg)
        self.assertEqual(vis.scenario_mode,'MEAN_VARIANCE_TWO_PROXY_MAPS_NOT_CALIBRATED')
        self.assertEqual(vis.n_scenarios,2)
        self.assertGreaterEqual(vis.gain(vis.view((16,14)))[1],0)
        members=np.stack([m,m*0,m*0+1],axis=0)
        ensemble_vis=ProbabilisticVisibility(replace(s,mean=m,variance=v,ensemble=members),_room(s),cfg)
        self.assertEqual(ensemble_vis.scenario_mode,'ENSEMBLE_MEMBERS')
        self.assertEqual(ensemble_vis.n_scenarios,3)

    def test_dense_full_plan_has_no_dominated_single_goal_deletion(self):
        s=self.sample()
        cfg=G2Config(lidar_range_m=3,planning_beams=100,final_beams=250,
            candidate_spacing_m=.3,clearance_m=.1,beam_width=4,branch_width=8,
            unknown_penalty_s_per_cell=.5,goal_stop_time_s=.2)
        out=plan(s,cfg)
        vv=ProbabilisticVisibility(s,_room(s),cfg,beams=cfg.final_beams,ray_step_cells=cfg.final_ray_step_cells)
        all_goal=out['goal_sequence']
        # Construct dense recalculation for every reduced chain using same knownfree shortest paths.
        from mapex_g2_v1.planner import _reach,_trace,_navigable
        legal=_navigable(s,_room(s),cfg)
        def cost_without(j):
            goals=[tuple(x) for i,x in enumerate(all_goal) if i!=j]
            cov=vv.zero.copy();start=s.robot_rc;length=0
            for goal in goals:
                prev,dist=_reach(start,legal);route=_trace(start,goal,prev)
                bit,_=vv.project_leg(route);cov|=bit
                length+=dist[goal]*cfg.resolution_m;start=goal
            return _cost(cfg,vv,cov,length,len(goals))['objective_s_equivalent']
        for i in range(len(all_goal)):
            self.assertGreaterEqual(cost_without(i)+1e-6,out['cost']['objective_s_equivalent'])

    def test_coverage_stages_use_dense_final_scenarios_and_do_not_overlap(self):
        s=self.sample()
        cfg=G2Config(lidar_range_m=4,planning_beams=90,final_beams=180,
            candidate_spacing_m=.5,clearance_m=.1)
        result=plan(s,cfg)
        room,total,stages=make_coverage(s,result,cfg)
        self.assertEqual(len(stages),result['goal_count'])
        if stages:
            self.assertTrue(np.allclose(np.stack(stages).sum(axis=0),total,atol=1e-6))
            self.assertTrue(np.all(total<=1.0001))
            self.assertTrue(np.all(total[~room]==0))


if __name__=='__main__': unittest.main()
