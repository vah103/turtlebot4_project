import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from r4_adaptive import (R4Config,EvidenceLedger,PathValue,Leg,Chain,
                         safe_event,bounded_execution)

class TestR4(unittest.TestCase):
    def setUp(self):
        self.cfg=R4Config(cost_per_second=0.3,speed_mps=1.,conservative_gate=True)
        self.regions=[dict(region_id=1,cells=100,expected_open_cell_mass_proxy=80,kind='POSSIBLE_OPEN_SPACE'),
                      dict(region_id=2,cells=100,expected_open_cell_mass_proxy=60,kind='UNVERIFIED_PREDICTED_WALL')]
        self.ledger=EvidenceLedger()
        self.scorer=PathValue(self.regions,{1:0xffff,2:0xffff0000},self.ledger,self.cfg)

    def leg(self,rc=(0,1),path=None,n=0,c=0,kind='view'):
        if path is None:path=((0,0),rc)
        return Leg(rc,tuple(path),n,c,kind)

    def chain(self,label='A',n1=0xff,n2=0xff,n3=0,con=None):
        if con is None:con=(n1,n2,n3)
        return Chain(label,(self.leg((0,1),((0,0),(0,1)),n1,con[0]),
                            self.leg((0,2),((0,1),(0,2)),n2,con[1]),
                            self.leg((0,0),((0,2),(0,1),(0,0)),n3,con[2],'exit')),'t0')

    def test_double_counting(self):
        row=self.scorer.evaluate(self.chain())
        self.assertEqual(row['nominal_unique_cells'],8)
        self.assertEqual(row['legs'][1]['unique_nominal_cells'],0)

    def test_return_leg_observations_count(self):
        row=self.scorer.evaluate(self.chain(n3=0xf00))
        self.assertEqual(row['legs'][-1]['unique_nominal_cells'],4)
        self.assertEqual(row['nominal_unique_cells'],12)

    def test_robust_conservative_gate(self):
        row=self.scorer.choose(None,[self.chain(con=(0,0,0))])
        self.assertEqual(row['decision'],'DEFER_ROOM')

    def test_no_mandatory_goal_number(self):
        lone=Chain('one',(self.leg((0,1),((0,0),(0,1)),0xffff,0xffff),
                          self.leg((0,0),((0,1),(0,0)),0,0,'exit')),'t0')
        self.assertEqual(self.scorer.choose(None,[lone])['decision'],'REPLAN')

    def test_late_benefit_redeems_low_first_leg(self):
        # First segment zero gain; full route has positive gain in a later section.
        c=self.chain(n1=0,n2=0xffff,n3=0)
        r=self.scorer.evaluate(c)
        self.assertLess(r['legs'][0]['net_step_proxy'],0)
        self.assertGreater(r['objective'],0)
        self.assertEqual(self.scorer.choose(None,[c])['decision'],'REPLAN')

    def test_unknown_region_cost_not_completed(self):
        n=1<<16
        self.assertGreater(self.scorer.gain(n),0)
        self.assertLess(self.scorer.gain(n),self.scorer.gain(1))

    def test_exit_required(self):
        c=Chain('X',(self.leg((0,1),((0,0),(0,1)),0xffff,0xffff),),'t0')
        self.assertEqual(self.scorer.choose(None,[c])['decision'],'DEFER_ROOM')

    def test_reference_novel_lost(self):
        out=self.scorer.evaluate(self.chain(n1=0b0011,n2=0,n3=0),baseline_mask=(0b1100,0b1100))
        self.assertEqual(out['baseline_comparison']['nominal_novel_cells'],2)
        self.assertEqual(out['baseline_comparison']['nominal_lost_cells'],2)

    def test_keep_hysteresis(self):
        existing=self.chain('KEEP',n1=0xffff,n2=0)
        candidate=self.chain('NEW',n1=0xffff,n2=0)
        self.assertEqual(self.scorer.choose(existing,[candidate])['decision'],'KEEP')

    def test_bad_horizon_no_gain(self):
        zero=self.chain(n1=0,n2=0)
        self.assertEqual(self.scorer.choose(None,[zero])['decision'],'DEFER_ROOM')

    def test_contradictory_evidence_reduces_value(self):
        base=self.scorer.gain(0xffff)
        before=np.ones((20,20),dtype=np.uint8)
        after=before.copy()
        labels=np.ones_like(before,dtype=np.int32)
        result=self.ledger.observe_completed_leg(before,after,labels,{1:50},(5,5),1,self.cfg)
        self.assertEqual(result['global_realized_cells'],0)
        self.assertLess(self.scorer.gain(0xffff),base)

    def test_evidence_from_observed_not_prediction(self):
        before=np.ones((20,20),np.uint8);after=before.copy();after[1:3,1:3]=0
        labels=np.ones_like(before,dtype=np.int32)
        r=self.ledger.observe_completed_leg(before,after,labels,{1:10},(5,5),1,self.cfg)
        self.assertEqual(r['by_region']['1']['realized'],4)
        self.assertEqual(r['global_realized_cells'],4)

    def test_replayed_evidence_rejected(self):
        before=np.ones((20,20),np.uint8)
        self.ledger.observe_completed_leg(before,before,np.ones_like(before),{1:20},(5,5),5,self.cfg)
        with self.assertRaises(ValueError):
            self.ledger.observe_completed_leg(before,before,np.ones_like(before),{1:20},(5,5),5,self.cfg)

    def test_nonmonotonic_map_rejected(self):
        before=np.zeros((20,20),np.uint8);after=before.copy();after[1,1]=1
        with self.assertRaises(ValueError):
            self.ledger.observe_completed_leg(before,after,np.ones_like(before),{1:20},(5,5),1,self.cfg)

    def test_ledger_roundtrip(self):
        before=np.ones((20,20),np.uint8)
        self.ledger.observe_completed_leg(before,before,np.ones_like(before),{1:20},(5,5),1,self.cfg)
        self.assertEqual(EvidenceLedger.from_json(self.ledger.to_json()).to_json(),self.ledger.to_json())

    def test_failure_views_require_spatial_independence(self):
        before=np.ones((40,40),np.uint8);labels=np.ones_like(before)
        for i,pos in enumerate(((4,4),(5,5),(24,24)),1):
            self.ledger.observe_completed_leg(before,before,labels,{1:20},pos,i,self.cfg)
        self.assertEqual(len(self.ledger.regions['1'].failed_views),2)

    def test_event_fast_keep(self):
        a=np.zeros((30,30),np.uint8)
        r=safe_event(a,a,(15,15),[(15,15),(15,16)],clearance_cells=1)
        self.assertEqual(r['event'],'FAST_KEEP')

    def test_event_goal_arrival(self):
        a=np.zeros((30,30),np.uint8)
        r=safe_event(a,a,(15,15),[(15,15),(15,16)],at_goal=True,clearance_cells=0)
        self.assertEqual(r['event'],'REVALUE_NOW')

    def test_event_room_version_change(self):
        a=np.zeros((30,30),np.uint8)
        r=safe_event(a,a,(15,15),[(15,15),(15,16)],structural_changed=True,clearance_cells=0)
        self.assertTrue(r['heavy'])

    def test_event_new_wall_on_route_holds(self):
        a=np.zeros((30,30),np.uint8);a[15,19]=1
        b=a.copy();b[15,19]=2
        r=safe_event(a,b,(15,15),[(15,15),(15,16),(15,17),(15,18)],clearance_cells=1)
        self.assertEqual(r['event'],'SAFE_HOLD')

    def test_event_new_wall_far_triggers_delayed(self):
        a=np.zeros((30,30),np.uint8);a[1,1]=1;b=a.copy();b[1,1]=2
        short=safe_event(a,b,(15,15),[(15,15),(15,16)],clearance_cells=1,scans_since_heavy=1)
        long=safe_event(a,b,(15,15),[(15,15),(15,16)],clearance_cells=1,scans_since_heavy=3)
        self.assertEqual(short['event'],'FAST_RECHECK')
        self.assertEqual(long['event'],'REVALUE_NOW')

    def test_event_mismatch_triggers(self):
        a=np.zeros((30,30),np.uint8)
        r=safe_event(a,a,(15,15),[(15,15)],expected_cells=70,realized_cells=0,
                     scans_since_heavy=3,clearance_cells=0)
        self.assertEqual(r['reason'],'DISCREPANCY')

    def test_event_unknown_route_holds(self):
        a=np.zeros((30,30),np.uint8);a[15,16]=1
        r=safe_event(a,a,(15,15),[(15,15),(15,16)],clearance_cells=0)
        self.assertEqual(r['event'],'SAFE_HOLD')

    def test_event_no_route_holds(self):
        a=np.zeros((30,30),np.uint8)
        r=safe_event(a,a,(15,15),[],clearance_cells=0)
        self.assertEqual(r['event'],'SAFE_HOLD')

    def test_event_bad_obs_rejected(self):
        a=np.zeros((30,30),np.uint8);b=a.copy();b[0,0]=1
        with self.assertRaises(ValueError):safe_event(a,b,(15,15),[(15,15)],clearance_cells=0)

    def test_bounded_execution_prefix(self):
        path=[(0,i) for i in range(30)]
        r=bounded_execution(path,R4Config(max_execution_m=0.5))
        self.assertEqual(len(r),6)
        self.assertEqual(r[-1],[0,5])

    def test_disconnected_chain_fails(self):
        with self.assertRaises(ValueError):
            Chain('bad',(self.leg((0,1)),self.leg((1,2))), 't0')

    def test_invalid_observed_path_cannot_be_certified(self):
        self.assertFalse(self.scorer.evaluate(Chain('bad',(self.leg((0,1)),),'t0'))['feasible'])

if __name__=='__main__':unittest.main(verbosity=2)

# Additional regression cases (independent of the frozen D18 failure).
class TestProvenance(unittest.TestCase):
    def test_mixed_snapshot_comparison_rejected(self):
        p=PathValue([dict(region_id=1,cells=10,expected_open_cell_mass_proxy=10,kind='POSSIBLE_OPEN_SPACE')],
                     {1:0x3ff},EvidenceLedger(),R4Config(cost_per_second=0))
        leg=Leg((0,1),((0,0),(0,1)),0x3ff,0x3ff)
        exit_=Leg((0,0),((0,1),(0,0)),0,0,'exit')
        a=Chain('A',(leg,exit_),'scan_11');b=Chain('B',(leg,exit_),'scan_12')
        with self.assertRaises(ValueError):p.choose(a,[b])

    def test_roi_only_change_avoids_false_heavy(self):
        obs=np.zeros((30,30),np.uint8);obs[1,1]=1
        new=obs.copy();new[1,1]=2
        roi=np.zeros_like(obs,dtype=bool);roi[10:20,10:20]=True
        event=safe_event(obs,new,(15,15),[(15,15),(15,16)],
                         clearance_cells=0,scans_since_heavy=10,roi_mask=roi)
        self.assertEqual(event['event'],'FAST_KEEP')

    def test_roi_shape_fails_closed(self):
        obs=np.zeros((30,30),np.uint8)
        with self.assertRaises(ValueError):
            safe_event(obs,obs,(15,15),[(15,15)],clearance_cells=0,
                       roi_mask=np.zeros((20,20),bool))
