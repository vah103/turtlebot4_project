"""Run10 gate tests: frontiers EXIT ONLY, G2 controls every LOCAL motion goal."""
from pathlib import Path
import ast,hashlib,json,unittest
import cv2,numpy as np
from g2_local_backstop import generate_local_viewpoint
class Run10Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root=Path(__file__).resolve().parent
        cls.r09=Path('/home/dell/mx072_g2_r4_predicted_goal_run09_20261017')
        cls.base=cls.r09/'native_g2_r4_predicted_run09/G2_R4_PREDICTED_GOAL_RUN09_visvarprob/room_r41'
    def frozen(self,i):
        d=json.loads((self.base/'decisions'/('decision_%06d.json'%i)).read_text())
        with np.load(self.base/'inference'/('decision_%06d_input.npz'%i)) as z:
            obs=z['observed'].copy();mean=z['mean'].copy();var=z['variance'].copy()
        with np.load(self.base/'inference'/('decision_%06d_semantic.npz'%i)) as z:sem=z['semantic'].copy()
        robot=tuple(np.array(d['robot_rc'])-500)
        from mapex_g2_r3.structural import accepted_component
        room=accepted_component(obs,sem,robot)
        native=cv2.erode(np.uint8(room&(obs==0)),np.ones((5,5),np.uint8))>0
        native[robot]=True
        return d,obs,mean,var,sem,robot,room,native
    def test_d02_room_not_exhausted_on_defer(self):
        d,obs,mean,var,sem,robot,room,native=self.frozen(6)
        self.assertEqual(d['mode'],'LOCAL')
        self.assertGreater(d['local_candidate_count'],0)
        goal=generate_local_viewpoint(obs,mean,var,sem,robot,room,native)
        self.assertIsNotNone(goal)
        self.assertEqual(obs[tuple(goal['goal_core_rc'])],0)
        self.assertTrue(native[tuple(goal['goal_core_rc'])])
        self.assertFalse(goal['used_WFD_for_goal_generation'])
    def test_late_room_defer_also_has_viewpoint(self):
        for i in (24,27):
            with self.subTest(decision=i):
                d,obs,mean,var,sem,robot,room,native=self.frozen(i)
                self.assertEqual(d['mode'],'LOCAL')
                self.assertGreater(d['local_candidate_count'],0)
                goal=generate_local_viewpoint(obs,mean,var,sem,robot,room,native)
                self.assertIsNotNone(goal)
                self.assertGreater(goal['predicted_visible_unknown_weighted'],0)
    def test_no_unknown_motion(self):
        _,obs,mean,var,sem,robot,room,native=self.frozen(6)
        out=generate_local_viewpoint(obs,mean,var,sem,robot,room,native)
        self.assertTrue(native[tuple(out['goal_core_rc'])])
        bad=native.copy()
        bad[:]=False
        bad[robot]=True
        self.assertIsNone(generate_local_viewpoint(obs,mean,var,sem,robot,room,bad))
    def test_no_frontier_as_movement_source(self):
        ex=(self.root/'explore_g2_r4_lama3_smallroi_run06.py').read_text()
        self.assertIn("LOCAL_VIEWPOINT_UNAVAILABLE_WITH_REACHABLE_WFD",ex)
        self.assertIn("candidate_g2_goal = g2_run.choose(t,room_run,cur_pose)",ex)
        self.assertNotIn("if candidate_g2_goal is None:\n                                frontier_region_centers",ex)
        local=(self.root/'g2_local_backstop.py').read_text()
        tree=ast.parse(local)
        fn=next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='generate_local_viewpoint')
        self.assertNotIn('frontier',','.join(x.arg for x in fn.args.args))
    def test_frozen_run09_sources_unchanged(self):
        for file in ['fast_native_lama_3model.py','room_native_integration.py',
                     'g2_predicted_goals.py','g2src/r4_adaptive.py','g2src/mapex_g2_r3/sequence.py']:
            self.assertEqual(hashlib.sha256((self.root/file).read_bytes()).digest(),
                             hashlib.sha256((self.r09/file).read_bytes()).digest())
if __name__=='__main__':unittest.main()
