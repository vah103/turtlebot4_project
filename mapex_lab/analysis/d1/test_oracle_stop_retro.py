import tempfile,unittest
from pathlib import Path
import numpy as np,pandas as pd
from oracle_stop_retro import *

class Tests(unittest.TestCase):
 def test_01_universe_constants(self): self.assertEqual((len(RUNS),sum(EXPECTED_COUNTS.values()),SPAWN,RADIUS_M,GT_RESOLUTION),(10,365,(1202,512),.189,.05))
 def test_02_integer_crossing(self): self.assertEqual(persistent_oracle([6,5,5],5,100)["index"],1)
 def test_03_transient_crossing(self):
  x=persistent_oracle([4,7,3],5,100); self.assertEqual((x["naive_index"],x["index"],x["persistence_changed"]),(0,2,True))
 def test_04_no_crossing(self): self.assertEqual(persistent_oracle([8,7],5,100)["status"],"NO_ACCEPTABLE_ORACLE_STOP")
 def test_05_zero_remaining(self): self.assertEqual(persistent_oracle([1,0,0],1,100)["index"],0)
 def test_06_missing_suffix_blocks(self): self.assertEqual(persistent_oracle([4,3,None],5,100)["status"],"INSUFFICIENT_TRUTH_SUPPORT")
 def test_07_best_tie_earliest(self): self.assertEqual(persistent_oracle([5,2,2],1,100)["best_index"],1)
 def test_08_monotonicity(self): self.assertEqual(monotonicity([5,3,4,2]),{"upward_count":1,"max_upward_cells":1,"affected_next_indices":[2]})
 def test_09_no_corner_cut(self):
  ref=Path(__file__).resolve().parents[4]/"d1-r004-reference"; topo,_=load_r004_helper(ref)
  mask=np.array([[1,0],[0,1]],dtype=bool); self.assertEqual(int(topo.reachable(mask,(0,0)).sum()),1)
 def test_10_known_projection_uses_known_both(self):
  from d1_gate_p import RawGrid,StructuralGT
  raw=RawGrid(np.array([[0,100],[-1,0]]),.1,0,0,0,0); gt=StructuralGT(np.zeros((4,4),dtype=np.int16),np.ones((4,4),bool),.05,0,0,"x","x")
  known,_=project_known(raw,gt); self.assertEqual(int(known.sum()),12)
 def test_11_outside_support_unresolved(self):
  from d1_gate_p import RawGrid,StructuralGT
  raw=RawGrid(np.array([[0]]),.1,0,0,0,0); gt=StructuralGT(np.zeros((4,4),dtype=np.int16),np.ones((4,4),bool),.05,0,0,"x","x")
  known,_=project_known(raw,gt); self.assertEqual(int(known.sum()),4)
 def test_12_prediction_absent(self): self.assertNotIn("prediction",score_decisions.__code__.co_names)
 def test_13_fixed_denominator(self):
  x=persistent_oracle([10,5],5,100); self.assertEqual(x['index'],1)
 def test_14_universe_reject_duplicates(self):
  f=pd.DataFrame([{"run_id":"mpx_001","decision_id":1}]*365)
  with self.assertRaises(ValueError): validate_universe(f)
 def test_15_signal_offsets(self):
  d=pd.DataFrame({"run_id":["x"]*3,"decision_id":[1,2,3],"decision_index":[1,2,3],"U_p95":[1,2,3]}); r=pd.DataFrame([{"run_id":"x","oracle_5_decision":2}]); self.assertEqual(signal_around(d,r).offset.tolist(),[-1,0,1])
 def test_16_determinism(self): self.assertEqual(persistent_oracle([9,4,4],5,100),persistent_oracle([9,4,4],5,100))
 def test_17_tolerances_frozen(self): self.assertEqual(TOLERANCES,(1,5,10))
 def test_18_na_preserved(self): self.assertEqual(persistent_oracle([None],5,100)["status"],"INSUFFICIENT_TRUTH_SUPPORT")
 def test_19_closed_stencil_boundary(self):
  ref=Path(__file__).resolve().parents[4]/"d1-r004-reference"; topo,_=load_r004_helper(ref)
  stencil=topo.collision_stencil(RADIUS_M,GT_RESOLUTION); self.assertTrue(stencil[stencil.shape[0]//2,stencil.shape[1]//2]); self.assertEqual(stencil.dtype,bool)
 def test_20_geometry_identity(self): self.assertEqual(GT_BLOB,"a5653a7ec7f4550287a3ebeb05b922dbc7a5bc3a")
 def test_21_helper_identity(self): self.assertEqual(GATE_P_BLOB,"c5bf4e3e6f45c81afef135166fc96e088719658e")
 def test_22_shared_identity(self): self.assertEqual(SHARED_BLOB,"3e80124d545ea37b2791594dd775a7d953368bea")
 def test_23_reference_identity(self): self.assertEqual(R004_REFERENCE,"61b91640ca1d4fd608f716e152a91573ec072b5a")
 def test_24_narrow_corridor_rejected(self):
  ref=Path(__file__).resolve().parents[4]/"d1-r004-reference"; topo,_=load_r004_helper(ref)
  domain=np.ones((15,15),bool); free=np.zeros_like(domain); free[:,7]=True
  self.assertEqual(int(topo.cspace(free,domain,topo.collision_stencil()).sum()),0)
 def test_25_disconnected_pocket_excluded(self):
  ref=Path(__file__).resolve().parents[4]/"d1-r004-reference"; topo,_=load_r004_helper(ref)
  mask=np.zeros((7,7),bool); mask[1:3,1:3]=True; mask[4:6,4:6]=True
  self.assertEqual(int(topo.reachable(mask,(1,1)).sum()),4)
 def test_26_runtime_growth_projection(self):
  from d1_gate_p import RawGrid,StructuralGT
  gt=StructuralGT(np.zeros((8,8),dtype=np.int16),np.ones((8,8),bool),.05,0,0,"x","x")
  small=RawGrid(np.zeros((1,1),dtype=np.int16),.1,0,0,0,0); large=RawGrid(np.zeros((2,2),dtype=np.int16),.1,0,0,0,0)
  self.assertLess(project_known(small,gt)[0].sum(),project_known(large,gt)[0].sum())
 def test_27_raw_input_hash_mismatch(self):
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp); run=root/'mpx_001'; run.mkdir(); (run/'raw.npz').write_bytes(b'x')
   frame=pd.DataFrame([{'run_id':'mpx_001','decision_id':1,'raw_map':'raw.npz','raw_map_sha256':'bad'}])
   with self.assertRaises(ValueError): raw_input_fingerprint(frame,root)
if __name__=="__main__": unittest.main()
