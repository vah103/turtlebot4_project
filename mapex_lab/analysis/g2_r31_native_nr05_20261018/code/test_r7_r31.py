"""R7-R3.1 tests: actual prior native scans, mirrored geometry, no GT."""
import unittest,subprocess,json,os,tempfile
from pathlib import Path
import numpy as np
from r7_stable_hypothesis import StableHypothesis

PRE=Path('/home/dell/DELL_NATIVE2D/runs/NR-G2R7-03/native_g2_r7_structural_missions_run01/G2_R7_STRUCTURAL_MISSIONS_RUN01_visvarprob')
BUILD=Path(__file__).resolve().parent


def freeze():
    f=PRE/'room_r41/inference/decision_000002'
    a=np.load(str(f)+'_input.npz')
    sem=np.load(str(f)+'_semantic.npz')['semantic']
    return a['observed'].copy(),a['mean'].copy(),a['variance'].copy(),sem.copy(),(147,111)

def scan(tick):
    with np.load(PRE/'room_r41/steps'/('tick_%06d_postscan.npz'%tick)) as z:
        raw=z['observed'][500:-500,500:-500]
    obs=np.ones(raw.shape,np.uint8);obs[raw==0]=0;obs[raw==1]=2
    return obs

def service_req(p,action,**kwargs):
    message=dict(action=action,**kwargs)
    p.stdin.write(json.dumps(message)+'\n')
    p.stdin.flush()
    reply=json.loads(p.stdout.readline())
    if reply.get("event")=="ERROR":raise RuntimeError(str(reply))
    return reply

class Tests(unittest.TestCase):
    def test_01_baseline_structure_frozen_and_goal2(self):
        o,m,v,s,robot=freeze()
        z=StableHypothesis(o,m,v,s,robot,o==0)
        self.assertEqual(z.plan1['goals'],[[169,132],[196,96]])
        self.assertEqual(len(z.zoneids),2)
        self.assertEqual(z.entry_zone,1)
        self.assertEqual(z.portal,(169,132))

    def test_02_actual_obstacles_stay_in_original_right_chamber(self):
        o,m,v,s,robot=freeze()
        z=StableHypothesis(o,m,v,s,robot,o==0)
        initial_hash=z.fingerprint
        report=z.update(scan(27),scan(27)==0,27)
        self.assertEqual(report['confirmed_unexpected_obstacle_cells'],43)
        self.assertEqual(len(report['objects_in_other_chambers']),2)
        self.assertTrue(all(x['zone']!=z.entry_zone for x in report['objects_in_other_chambers']))
        self.assertEqual(z.fingerprint,initial_hash)
        self.assertEqual(report['frozen_portal_rc'],[169,132])

    def test_03_plan2_opposite_right_corners_ordered_by_unknown(self):
        o,m,v,s,robot=freeze();z=StableHypothesis(o,m,v,s,robot,o==0)
        report=z.update(scan(27),scan(27)==0,27)
        self.assertEqual([x['corner'] for x in report['tasks']],['bottom_right','top_right'])
        self.assertEqual([x['goal_rc'] for x in report['tasks']],[[196,206],[148,206]])
        self.assertEqual([x['unseen_half_cells'] for x in report['tasks']],[562,276])
        self.assertFalse(any(x['native_motion_authorized'] for x in report['tasks']))

    def test_04_updated_g1_changes_do_not_change_chambers(self):
        o,m,v,s,robot=freeze();z=StableHypothesis(o,m,v,s,robot,o==0)
        fingerprint=z.fingerprint
        for tick in (27,35,55,75,105):
            report=z.update(scan(tick),scan(tick)==0,tick)
            self.assertEqual(report['frozen_zone_sha256'],fingerprint)
            self.assertFalse(report['semantic_zone_relabeled'])
        self.assertGreater(z.version,1)

    def test_05_no_plan2_without_new_observed_blocker(self):
        o,m,v,s,robot=freeze()
        z=StableHypothesis(o,m,v,s,robot,o==0)
        self.assertEqual(z.update(o,o==0,12)['tasks'],[])

    def test_06_mirror_invariance_not_manual_d02_coordinates(self):
        o,m,v,s,robot=freeze();h,w=o.shape
        left=StableHypothesis(o,m,v,s,robot,o==0)
        goals=[t['goal_rc'] for t in left.update(scan(27),scan(27)==0,27)['tasks']]
        reflected=tuple(np.fliplr(a) for a in (o,m,v,s))
        right=StableHypothesis(*reflected,(robot[0],w-1-robot[1]),
                               np.fliplr(o)==0)
        now=np.fliplr(scan(27))
        matched=[t['goal_rc'] for t in right.update(now,now==0,27)['tasks']]
        expected=[[g[0],w-1-g[1]] for g in goals]
        self.assertEqual(len(matched),2)
        self.assertEqual(matched,expected)

    def test_07_same_room_service_replay_plan1_to_plan2(self):
        o,m,v,s,robot=freeze()
        base=PRE/'room_r41/inference'
        scope=PRE/'g2_r7'
        proc=subprocess.Popen(['/usr/bin/python3','-u',str(BUILD/'r7_native_sidecar.py')],
                               cwd=str(BUILD),stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE,text=True)
        self.assertEqual(json.loads(proc.stdout.readline())['event'],'READY')
        try:
            def choose(i,t,pose):
                b=base/('decision_%06d'%i)
                return service_req(proc,'CHOOSE',
                   native_tick=t,room_id=1,robot_core_rc=list(pose),
                   room_node_input=str(b)+'_input.npz',
                   room_node_semantic=str(b)+'_semantic.npz',
                   native_scope_npz=str(scope/('native_scope_%06d.npz'%i)))
            initial=choose(2,12,(147,111))
            self.assertEqual(initial['goal_sequence_core_rc'],[[169,132],[196,96]])
            # At tick14, some unexpected occupied pixels have appeared,
            # but Goal1 is not yet confirmed: NO premature Plan2 commitment.
            prior=scan(14)
            with tempfile.TemporaryDirectory() as tmp:
                obsfile=Path(tmp)/'obs.npz';navfile=Path(tmp)/'nav.npz'
                np.savez_compressed(obsfile,observed=prior)
                np.savez_compressed(navfile,reachable_knownfree=(prior==0).astype('uint8'))
                upd=service_req(proc,'UPDATE',native_tick=14,
                     robot_core_rc=[155,111],observed_npz=str(obsfile),
                     native_scope_npz=str(navfile))
                self.assertEqual(upd['plan2_prepared'],[])
                self.assertGreaterEqual(upd['new_obstacle_cells'],3)
            ack=service_req(proc,'ACK_GOAL',robot_core_rc=[169,132])
            self.assertEqual(ack['remaining_goals_rc'],[[196,96]])
            # New *current* D03 observation, cannot inspect D04 yet.
            atgoal1=choose(3,27,(169,132))
            self.assertEqual(atgoal1['goal_core_rc'],[196,96])
            self.assertEqual(atgoal1['plan2_witness']['confirmed_obstacles'],43)
            self.assertEqual([x['goal_rc'] for x in atgoal1['plan2_witness']['obligations']],
                             [[196,206],[148,206]])
            ack2=service_req(proc,'ACK_GOAL',robot_core_rc=[196,96])
            self.assertTrue(ack2['PLAN1_completed'])
            # Current D04 data at Goal2, safely promote Plan2.
            after=choose(4,55,(196,96))
            self.assertEqual(after['plan_phase'],'PLAN2')
            self.assertEqual(after['goal_sequence_core_rc'],[[196,206],[148,206]])
            self.assertEqual(after['goal_core_rc'],[196,206])
            g3=service_req(proc,'ACK_GOAL',robot_core_rc=[196,206])
            self.assertEqual(g3['remaining_goals_rc'],[[148,206]])
            self.assertEqual(service_req(proc,'STATUS')['remaining_goals'],[[148,206]])
            g4=service_req(proc,'ACK_GOAL',robot_core_rc=[148,206])
            self.assertEqual(g4['remaining_goals_rc'],[])
        finally:
            try:service_req(proc,'CLOSE')
            except Exception:pass
            proc.wait(timeout=15)

    def test_08_no_hardcoded_goal_coordinates_in_stable_code(self):
        for name in ('r7_stable_hypothesis.py','r7_native_sidecar.py'):
            s=(BUILD/name).read_text()
            for point in ('169,132','196,96','196,206','148,206'):
                self.assertNotIn(point,s)
    def test_09_separate_room_state_resets_without_reusing_old_predictions(self):
        o,m,v,s,robot=freeze()
        one=StableHypothesis(o,m,v,s,robot,o==0)
        revised=one.update(scan(27),scan(27)==0,27)
        fresh=StableHypothesis(o,m,v,s,robot,o==0)
        self.assertEqual(fresh.version,1)
        self.assertEqual(fresh.update(o,o==0,12)['tasks'],[])
    def test_10_new_predicted_free_alone_cannot_authorize_movement(self):
        o,m,v,s,robot=freeze()
        h=StableHypothesis(o,m,v,s,robot,o==0)
        for t in h.update(scan(27),scan(27)==0,27)['tasks']:
            self.assertFalse(t['native_motion_authorized'])
        self.assertFalse(h.plan1['tasks'][0]['movement_authorized'])

if __name__=='__main__':unittest.main(verbosity=2)
