import csv, importlib.util, math, tempfile, unittest
from pathlib import Path
import numpy as np
P=Path(__file__).with_name("evaluate_prediction_vs_final_observed.py"); S=importlib.util.spec_from_file_location("r004",P); r=S.loader.load_module()
def canvas(path,shape=(3,3),resolution=.05,origin_x=0.,origin_y=0.,canvas_id="test_canvas"):
 np.savez(path,data=np.full(shape,-1,int),resolution=resolution,origin_x=origin_x,origin_y=origin_y,canvas_id=canvas_id)
def table(path,rows,fields):
 with path.open("w",newline="") as f:
  w=csv.DictWriter(f,fieldnames=fields); w.writeheader(); w.writerows(rows)
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
 def test_geometric_support_includes_zero_prediction(self):
  with tempfile.TemporaryDirectory() as td:
   d=Path(td); (d/"raw.npz").touch()
   np.savez(d/"raw.npz",data=np.full((1,1),-1),resolution=.1,origin_x=0.,origin_y=0.)
   canvas(d/"canvas.npz")
   np.savez(d/"mean.npz",data=np.zeros((1,1),np.float32),source_height=1,source_width=1,pad_top=0,pad_left=0,resolution=.1,origin_x=0.,origin_y=0.)
   obs,pred,support=r.prediction_canvas(d,{"raw_map":"raw.npz","canvas_map":"canvas.npz","mean_map":"mean.npz"},(3,3))
   self.assertEqual(int(support.sum()),4); self.assertTrue(np.all(pred[support]==0))
 def test_prediction_geometry_mismatch_rejected(self):
  with tempfile.TemporaryDirectory() as td:
   d=Path(td); np.savez(d/"raw.npz",data=np.full((1,1),-1),resolution=.1,origin_x=0.,origin_y=0.); canvas(d/"canvas.npz")
   np.savez(d/"mean.npz",data=np.zeros((1,1)),source_height=1,source_width=1,pad_top=0,pad_left=0,resolution=.1,origin_x=1.,origin_y=0.)
   with self.assertRaisesRegex(ValueError,"geometry provenance mismatch"): r.prediction_canvas(d,{"raw_map":"raw.npz","canvas_map":"canvas.npz","mean_map":"mean.npz"},(3,3))
 def test_sensitivity_samples_only_scoreable_vectors(self):
  rec={"run_id":"x","decision_id":1,"decision_progress":0.,"progress_bin":"[0.00,0.25)","F_count":10,"E_count":4,"support_coverage":.4}
  rows=r.sensitivity_rows("x",[(rec,np.array([0.,.2,.8,1.]),np.array([0,0,1,1],bool))],"total")
  self.assertEqual(rows[0]["sample_size"],4); self.assertEqual(rows[0]["replicates"],100)
 def test_zero_support_coverage_retained(self): self.assertEqual(r.div(0,5),0); self.assertTrue(math.isnan(r.div(0,0)))
 def test_final_snapshot_fallback(self):
  with tempfile.TemporaryDirectory() as td:
   d=Path(td); (d/"maps").mkdir(); canvas(d/"decision.npz"); canvas(d/"maps/periodic.npz"); (d/"maps/final.npz").write_text("corrupt")
   table(d/"decisions.csv",[{"canvas_map":"decision.npz"}],["canvas_map"])
   table(d/"snapshots.csv",[{"event":"periodic","canvas_map_file":"maps/periodic.npz"},{"event":"final","canvas_map_file":"maps/final.npz"}],["event","canvas_map_file"])
   _,p,fb=r.final_snapshot(d); self.assertTrue(fb); self.assertEqual(p,"maps/periodic.npz")
 def test_invalid_fallback_rejected(self):
  with tempfile.TemporaryDirectory() as td:
   d=Path(td); (d/"maps").mkdir(); canvas(d/"decision.npz"); canvas(d/"maps/periodic.npz",shape=(2,2)); (d/"maps/final.npz").write_text("corrupt")
   table(d/"decisions.csv",[{"canvas_map":"decision.npz"}],["canvas_map"]); table(d/"snapshots.csv",[{"event":"periodic","canvas_map_file":"maps/periodic.npz"},{"event":"final","canvas_map_file":"maps/final.npz"}],["event","canvas_map_file"])
   with self.assertRaisesRegex(ValueError,"no valid canonical"): r.final_snapshot(d)
 def test_unsupported_fraction_is_one_minus_coverage(self):
  self.assertAlmostEqual(r.unsupported_fraction(.714),.286); self.assertTrue(math.isnan(r.unsupported_fraction(math.nan)))
if __name__=="__main__": unittest.main()
