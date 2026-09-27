import unittest
import numpy as np,pandas as pd
from gate_r_v2 import *

class T(unittest.TestCase):
 def test_01_constants(self): self.assertEqual((len(RUNS),sum(COUNTS.values()),SEED),(10,365,20260927))
 def test_02_ecdf_ties(self): self.assertEqual(ecdf_le(np.array([1,1,2]),np.array([1,2])).tolist(),[2/3,1])
 def test_03_empty_ecdf(self): self.assertTrue(np.isnan(ecdf_le(np.array([]),np.array([1]))[0]))
 def frame(self,c,y,ok=None):
  n=len(c);return pd.DataFrame({'run_id':['x']*n,'decision_id':range(n),'decision_index':range(n),'N_GT':[100]*n,'oracle_truth_evaluable':[True]*n,'d1_runtime_region_evaluable':ok or [True]*n,'RemainingFraction':c,'OracleRemainingFraction_GT':y,'A_mean_m2':np.array(c)*.25,'A_true_remaining_m2':np.array(y)*.25})
 def test_04_perfect_rho(self): self.assertAlmostEqual(within_metrics(self.frame(range(8),range(8)),'RemainingFraction')['rho'],1)
 def test_05_reversed_rho(self): self.assertAlmostEqual(within_metrics(self.frame(range(8),range(7,-1,-1)),'RemainingFraction')['rho'],-1)
 def test_06_constant_invalid(self): self.assertTrue(np.isnan(within_metrics(self.frame([1]*8,range(8)),'RemainingFraction')['rho']))
 def test_07_na_preserved(self): self.assertEqual(len(pair(self.frame([1,np.nan],[1,2]),'RemainingFraction')),1)
 def test_08_flag_required(self): self.assertEqual(len(pair(self.frame([1,2],[1,2],[False,True]),'RemainingFraction')),1)
 def test_09_pairing_norm(self): self.assertEqual(CANDIDATES['RemainingFraction'][1],'OracleRemainingFraction_GT')
 def test_10_pairing_abs(self): self.assertEqual(CANDIDATES['A_mean_m2'][1],'A_true_remaining_m2')
 def test_11_cal_sign(self): self.assertTrue((residual(self.frame([.1,.2],[.2,.3]),'RemainingFraction')<0).all())
 def test_12_low25_empty_ties(self): self.assertTrue(np.isnan(within_metrics(self.frame([1]*8,range(8)),'RemainingFraction')['fc25']))
 def test_13_trr_perfect(self): self.assertLess(within_metrics(self.frame(range(8),range(8)),'RemainingFraction')['trr50'],1)
 def test_14_allowed_no_pass(self): self.assertNotIn('PASS',ALLOWED)
 def test_15_choose_none(self): self.assertIsNone(choose([{'admissible':False}]))
 def test_16_choose_tie(self):
  base={'admissible':True,'rho_dev_macro':1,'TRR50_dev':.5,'FC25_dev':0,'CalMAE_dev':0}
  self.assertEqual(choose([{**base,'candidate':'A_mean_m2'},{**base,'candidate':'RemainingFraction'}]),'RemainingFraction')
 def test_17_fail_precedence(self): self.assertEqual(classify(pd.DataFrame(),[],[],5)[0],'FAIL')
 def test_18_bootstrap_deterministic(self):
  d=pd.DataFrame({'rho':[1,2],'TRR50':[1,2],'FC25':[0,1],'calibration_bias_mean':[0,1]})
  self.assertEqual(bootstrap(d,20),bootstrap(d,20))
 def test_19_bins_frozen(self): self.assertEqual(len(BINS),4)
 def test_20_counts(self): self.assertEqual(COUNTS['mpx_003'],41)
 def test_21_unique_allowed(self): self.assertEqual(len(set(ALLOWED)),3)
 def test_22_area_residual_scale(self): self.assertAlmostEqual(residual(self.frame([.1],[.2]),'A_mean_m2')[0],-.1)
 def test_23_min_rho_pairs(self): self.assertTrue(np.isnan(rho_value(self.frame(range(4),range(4)),'RemainingFraction')))
 def test_24_whole_tie_no_split(self): self.assertEqual(len(set(ecdf_le(np.array([1,1]),np.array([1,1])))),1)
 def test_25_no_full_pass_label(self): self.assertTrue(all(x!='PASS' for x in ALLOWED))
 def test_26_seed_exact(self): self.assertEqual(SEED,20260927)
 def test_27_fc_formula(self):
  m=within_metrics(self.frame(range(8),range(7,-1,-1)),'RemainingFraction');self.assertGreaterEqual(m['fc25'],0)
 def test_28_candidate_names(self): self.assertEqual(set(CANDIDATES),{'RemainingFraction','A_mean_m2'})
if __name__=='__main__':unittest.main()
