"""Static navigation diagnostic must count infeasible plans, not only existence."""
import unittest
import json
import numpy as np

from .core import shortest_paths
from .analyze_v2 import navigation_scores, make_navigation_queries


class Tests(unittest.TestCase):
    def test_fixed_queries_are_reachable_repeatable_and_json_serializable(self):
        physical=np.ones((20,30),bool);physical[:,15]=False
        start=tuple(np.asarray([12,6],dtype=np.int64))
        a,goals,dist=make_navigation_queries(physical,np.ones_like(physical),start,np.random.default_rng(6100703))
        b,_,_=make_navigation_queries(physical,np.ones_like(physical),start,np.random.default_rng(6100703))
        self.assertEqual(a,b)
        self.assertTrue(all(dist[p]>0 and p[1]<15 for p in goals))
        self.assertEqual(json.loads(json.dumps(a)),a)

    def test_predicted_shortcut_through_real_wall_is_unsafe(self):
        physical=np.ones((20,30),bool)
        physical[:,15]=False;physical[3:6,15]=True
        start=(12,6);goal=(12,24)
        true_dist,_=shortest_paths(physical,start)
        wrong=navigation_scores(np.ones_like(physical),physical,start,[goal],true_dist)
        correct=navigation_scores(physical,physical,start,[goal],true_dist)
        self.assertEqual(wrong["plans_found"],1)
        self.assertEqual(wrong["safe_plans"],0)
        self.assertEqual(wrong["unsafe_plans"],1)
        self.assertEqual(correct["safe_plans"],1)
        self.assertEqual(correct["mean_safe_path_stretch"],1.)


if __name__=="__main__":
    unittest.main()
