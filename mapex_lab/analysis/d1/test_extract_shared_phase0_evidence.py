import json
import math
import os
from pathlib import Path
import tempfile
import unittest

import numpy as np

import extract_shared_phase0_evidence as e
import run_shared_phase0_extract as runner


class ContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.topo, cls.tables, cls.refs = e.load_reference(Path(os.environ['D1_REFERENCE']))

    def test_exact_universe_and_accepted_one_to_one(self):
        self.assertEqual(len(e.expected_keys()),365)
        self.assertEqual(len(e.COUNTS),10)
        for table in self.tables.values():
            self.assertEqual(set(table),e.expected_keys())

    def test_duplicate_and_missing_rejected(self):
        row = dict(run_id='mpx_001',decision_id=1)
        with self.assertRaisesRegex(ValueError,'duplicate'):
            e.index_rows([row,row],{('mpx_001',1)})
        with self.assertRaisesRegex(ValueError,'membership'):
            e.index_rows([row],e.expected_keys())

    def test_exact_zero_and_nonzero_fraction(self):
        self.assertEqual(e.remaining_fraction(0,[0,0,0],0.1),0)
        self.assertAlmostEqual(e.remaining_fraction(2,[3,6,9],0.1),0.75)
        self.assertTrue(math.isnan(e.remaining_fraction(None,None,0.1)))

    def test_union_and_disagreement_only_on_union(self):
        regions = [np.array([[True,False,False]]),np.array([[False,True,False]]),np.zeros((1,3),bool)]
        members = [np.array([[0.1,0.9,0.1]]),np.array([[0.9,0.1,0.1]]),np.array([[0.1,0.9,0.1]])]
        union,u = e.uncertainty(regions,members,np.array([[0.1,0.3,999.0]]))
        np.testing.assert_array_equal(union,[[True,True,False]])
        self.assertAlmostEqual(u['U_disagreement'],1/3)
        self.assertAlmostEqual(u['U_mean'],0.2)
        self.assertAlmostEqual(u['U_p95'],0.29)

    def test_valid_empty_and_invalid_source(self):
        raw = np.zeros((15,15))
        members = [np.zeros(raw.shape)]*3
        valid = e.runtime_fields(raw,members,np.zeros(raw.shape),0.1,(7,7),self.topo)
        self.assertTrue(valid['u_empty_valid'])
        self.assertEqual(valid['U_p95'],0)
        self.assertEqual(valid['RemainingFraction'],0)
        invalid = e.runtime_fields(raw,members,np.zeros(raw.shape),0.1,None,self.topo)
        self.assertFalse(invalid['u_empty_valid'])
        for f in e.RUNTIME_FIELDS:
            self.assertTrue(math.isnan(invalid[f]),f)

    def test_strict_free_threshold(self):
        raw = np.zeros((21,21)); raw[:,14:] = -1
        members = [np.full(raw.shape,0.5)]*3
        equal = e.runtime_fields(raw,members,np.zeros(raw.shape),0.1,(10,7),self.topo)
        below = e.runtime_fields(raw,[np.full(raw.shape,np.nextafter(0.5,0.0))]*3,np.zeros(raw.shape),0.1,(10,7),self.topo)
        self.assertEqual(equal['R_union_count'],0)
        self.assertGreater(below['R_union_count'],0)

    def test_observed_occupied_priority(self):
        raw = np.zeros((21,21)); raw[:,10]=100; raw[:,11:]=-1
        row = e.runtime_fields(raw,[np.zeros(raw.shape)]*3,np.zeros(raw.shape),0.1,(10,5),self.topo)
        self.assertTrue(row['u_r_union_evaluable'])
        self.assertEqual(row['R_union_count'],0)

    def test_known_only_and_area_conversion(self):
        raw = np.zeros((21,21)); raw[:,14:] = -1; raw[0,:]=100
        row = e.runtime_fields(raw,[np.zeros(raw.shape)]*3,np.zeros(raw.shape),0.1,(10,7),self.topo)
        known = self.topo.cspace(raw==0,np.ones(raw.shape,bool),self.topo.collision_stencil(0.189,0.1))
        expected = int(self.topo.reachable(known,(10,7)).sum())
        self.assertEqual(row['KnownReachableFree_count'],expected)
        self.assertEqual(row['KnownReachableFree_m2'],expected*0.1**2)
        self.assertLess(expected,int((raw==0).sum()))
        self.assertGreater(row['A_mean_m2'],0)

    def test_footprint_touching_and_outside_blocked(self):
        stencil = self.topo.collision_stencil(0.189,0.1)
        self.assertTrue(stencil[stencil.shape[0]//2,stencil.shape[1]//2+2])
        raw = np.zeros((15,15)); raw[7,9]=100
        row = e.runtime_fields(raw,[np.zeros(raw.shape)]*3,np.zeros(raw.shape),0.1,(7,7),self.topo)
        self.assertFalse(row['d1_source_available'])
        edge = e.runtime_fields(np.zeros((15,15)),[np.zeros((15,15))]*3,np.zeros((15,15)),0.1,(0,0),self.topo)
        self.assertFalse(edge['d1_source_available'])
        touch = self.topo.collision_stencil(0.15,0.1)
        self.assertTrue(touch[touch.shape[0]//2,touch.shape[1]//2+2])

    def test_no_corner_cut(self):
        mask = np.array([[True,False],[False,True]])
        self.assertEqual(int(self.topo.reachable(mask,(0,0)).sum()),1)

    def test_source_no_extrapolation_and_floor(self):
        trajectory = [(1.,0.,0.,0.),(3.,2.,4.,0.)]
        self.assertFalse(self.topo.align_source(0,trajectory)['available'])
        self.assertFalse(self.topo.align_source(4,trajectory)['available'])
        self.assertEqual(self.topo.align_source(2,trajectory)['x'],1)
        self.assertEqual(self.topo.align_source(1,trajectory)['mode'],'exact')
        self.assertEqual(self.topo.world_to_cell(-0.01,0.01,0,0,0.1),(0,-1))

    def test_non_safe_source_never_snaps(self):
        raw=np.zeros((21,21)); raw[10,10]=-1
        row=e.runtime_fields(raw,[np.zeros(raw.shape)]*3,np.zeros(raw.shape),0.1,(10,10),self.topo)
        self.assertEqual(row['d1_reason'],'source_not_known_footprint_safe')
        self.assertTrue(math.isnan(row['KnownReachableFree_count']))
        self.assertTrue(math.isnan(row['RemainingFraction']))

    def test_nan_reference_preserved(self):
        row=self.tables['topology'][('mpx_001',1)]
        self.assertTrue(math.isnan(e.number(row['lost_reachable_future_free_fraction'])))
        self.assertTrue(math.isnan(1-e.number('nan')))

    def test_identity_hash_sensitivity(self):
        baseline={'contract':'a','implementation':'b','input':'c'}
        for field in baseline:
            altered=dict(baseline);altered[field]='different'
            self.assertNotEqual(e.object_digest(baseline),e.object_digest(altered))

    def test_resume_identity_hash_and_key_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp);ident={'contract':'x','implementation':'y'};inputs={'raw':'z'}
            rows=[dict(run_id='mpx_001',decision_id=i) for i in range(1,36)]
            self.topo.write_csv(path/'evidence.csv',rows)
            runner.atomic_json(path/'completion_manifest.json',dict(identity=ident,input_fingerprints=inputs,
                output_sha256=e.digest(path/'evidence.csv'),row_count=35))
            self.assertEqual(len(runner.validate_unit(path,ident,inputs,'mpx_001')),35)
            for new_identity,new_inputs in (({'contract':'changed'},inputs),(ident,{'raw':'changed'})):
                with self.assertRaisesRegex(ValueError,'changed identity/input'):
                    runner.validate_unit(path,new_identity,new_inputs,'mpx_001')
            with (path/'evidence.csv').open('a') as stream:stream.write('corrupt\n')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                runner.validate_unit(path,ident,inputs,'mpx_001')

    def test_absent_completion_not_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FileNotFoundError):
                runner.validate_unit(Path(tmp),{}, {},'mpx_001')

    def test_artifact_identity_crop_and_stale_timestamp(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp); raw=np.zeros((5,6));row={'decision_id':'1'}
            row['raw_map']='decision_000001_raw.npz';row['canvas_map']='decision_000001_canvas.npz'
            np.savez(run/row['raw_map'],data=raw,resolution=0.1,width=6,height=5,
                     origin_x=0.,origin_y=0.,origin_yaw=0.,frame_id='map',source_stamp_s=10.)
            def prediction(key,member,**overrides):
                row[key]=f'decision_000001_{member.lower()}.npz'
                metadata=dict(data=np.zeros((7,8)),member=member,resolution=0.1,origin_x=0.,origin_y=0.,
                              source_height=5,source_width=6,pad_top=1,pad_left=1,source_map_stamp_s=10.)
                metadata.update(overrides);np.savez(run/row[key],**metadata)
            for key,member in zip(e.ARTIFACTS[2:],('G1','G2','G3','mean','variance')):prediction(key,member)
            self.assertEqual(e.load_runtime(run,row)[1][0].shape,raw.shape)
            prediction('g1_map','G1',source_map_stamp_s=11.)
            with self.assertRaisesRegex(ValueError,'timestamp'):e.load_runtime(run,row)
            prediction('g1_map','G1',pad_top=99)
            with self.assertRaisesRegex(ValueError,'crop'):e.load_runtime(run,row)
            prediction('g1_map','G1');row['decision_id']='2'
            with self.assertRaisesRegex(ValueError,'identity'):e.load_runtime(run,row)

    def test_escape_path_rejected(self):
        with self.assertRaisesRegex(ValueError,'escapes'):
            e.local_path(Path('/tmp/run'),'../other')


if __name__ == '__main__':
    unittest.main()
