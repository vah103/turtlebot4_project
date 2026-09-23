import importlib.util, math, unittest
from pathlib import Path
import numpy as np
P=Path(__file__).with_name("evaluate_prediction_vs_final_observed.py"); S=importlib.util.spec_from_file_location("r004",P); r=S.loader.load_module()
class Tests(unittest.TestCase):
 def test_strict_threshold(self):
  m=r.metrics(np.array([.5,.50001]),np.array([0,1])); self.assertEqual(m["macro_iou"],1)
 def test_undefined(self):
  m=r.metrics(np.array([0.1]),np.array([0])); self.assertTrue(math.isnan(m["occupied_iou"])); self.assertTrue(math.isnan(m["macro_iou"]))
 def test_progress(self): self.assertEqual(r.progress(1,5),0); self.assertEqual(r.progress(5,5),1); self.assertTrue(math.isnan(r.progress(1,1)))
 def test_bins(self): self.assertEqual(r.pbin(.25),"[0.25,0.50)"); self.assertEqual(r.pbin(1),"[0.75,1.00]")
 def test_seed(self): self.assertEqual(r.seed("total","x",1,2),r.seed("total","x",1,2)); self.assertNotEqual(r.seed("total","x",1,2),r.seed("class","x",1,2))
 def test_support_independent_zero(self):
  values=np.array([0.,1.]); support=np.array([True,False]); F=np.array([True,True]); self.assertTrue((F&support)[0]); self.assertEqual(values[0],0)
if __name__=="__main__": unittest.main()
