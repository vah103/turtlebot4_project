import numpy as np
import unittest
from g2_native_integration import G2Run
from pathlib import Path
import difflib
class GoalArrivalTests(unittest.TestCase):
    def setUp(self):
        self.g=G2Run.__new__(G2Run)
        self.g.local_lock=True
        self.map=np.ones((80,80),dtype=np.float32)
        self.target=np.array([40,40],dtype=np.int32)
    def test_within_old_threshold_does_not_replan(self):
        for distance in range(1,10):
            p=np.array([40,40+distance])
            self.assertFalse(self.g.needs_replan(p,self.map,self.target,1.0,10))
    def test_exact_goal_requires_replan(self):
        self.assertTrue(self.g.needs_replan(self.target,self.map,self.target,1.0,10))
    def test_obstacle_causes_fail_closed(self):
        self.map[40,40]=float('inf')
        self.assertTrue(self.g.needs_replan(np.array([40,39]),self.map,self.target,1.0,10))
    def test_severed_path_causes_fail_closed(self):
        self.map[39,:]=float('inf')
        self.assertTrue(self.g.needs_replan(np.array([38,40]),self.map,self.target,1.0,10))
    def test_g2_nonlocal_does_not_force_select(self):
        self.g.local_lock=False
        self.assertIsNone(self.g.needs_replan(np.array([40,39]),self.map,self.target,1.0,10))
    def test_native_three_step_exact_endpoint(self):
        import ast
        source='/work/com1/mapex_single_audit/source/MapEx/scripts/sim_utils.py'
        tree=ast.parse(Path(source).read_text())
        fn=next(f for f in tree.body if isinstance(f,ast.FunctionDef) and f.name=='psuedo_traj_controller')
        ns={'np':np}
        exec(compile(ast.Module(body=[fn],type_ignores=[]),source,'exec'),ns)
        sim_utils=type('NativeController',(),{'psuedo_traj_controller':staticmethod(ns['psuedo_traj_controller'])})
        for N in range(2,35):
            planx=np.arange(N,dtype=np.int64)
            plany=np.zeros(N,dtype=np.int64)
            last=np.array([0,0],dtype=np.int64)
            counter=0
            while counter<N-1:
                cur=sim_utils.psuedo_traj_controller(planx[counter:],plany[counter:],3)
                advance=min(3,N-1-counter)
                self.assertEqual(int(cur[0]),counter+advance)
                counter+=advance
                if counter==N-1:break
            self.assertEqual(int(cur[0]),N-1)
    def test_exactly_two_old_source_sections_changed(self):
        R=Path(__file__).parent
        O=Path('/work/com1/mx072_g2_r4_native_20261017')
        a=(O/'explore_g2_r4_lama3_smallroi_run06.py').read_text()
        b=(R/'explore_g2_r4_lama3_smallroi_run06.py').read_text()
        self.assertIn('and not (g2_run is not None and g2_run.local_lock)',b)
        self.assertEqual(b.replace(' and not (g2_run is not None and g2_run.local_lock)','')
                        .replace('                    # Preserve MapEx GLOBAL/UPEN frontier arrival policy (1 m).\n','')
                        .replace('                    # LOCAL G2 exact viewpoint arrival is checked by G2Run.needs_replan.\n',''),a)
        a=(O/'g2_native_integration.py').read_text()
        b=(R/'g2_native_integration.py').read_text()
        self.assertIn('np.array_equal(g,p)',b)
        self.assertIn('np.linalg.norm(g-p)<threshold_m*ppm',a)
    def test_shared_model_and_roomnode_byte_identical(self):
        import hashlib
        R=Path(__file__).parent
        O=Path('/work/com1/mx072_g2_r4_native_20261017')
        for p in ['fast_native_lama_3model.py','room_native_integration.py',
                  'g2_r4_policy_sidecar.py','g2src/r4_episode.py','g2src/r4_adaptive.py',
                  'g2src/mapex_g2_r3/commitment.py']:
            self.assertEqual(hashlib.sha256((R/p).read_bytes()).digest(),
                             hashlib.sha256((O/p).read_bytes()).digest())
if __name__=='__main__':unittest.main()
