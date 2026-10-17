"""Read-only R4 comparison on an existing frozen R3.1 stage, not a rollout.

Example (on DELL, if R3 package is accessible):
PYTHONPATH=/home/dell/mx072_g2_r31_structure_commitment_20261016 \
 python3 shadow_readonly.py /home/dell/mx072_g2_r31_structure_commitment_20261016/D18_R31_CAUSAL_20261016/rollout 3
"""
import argparse,json,sys
from pathlib import Path
import numpy as np
from r4_adaptive import from_r3_snapshot,R4Config,EvidenceLedger,RegionEvidence
from r4_episode import prefixes


def load_stage(root,stage):
    from mapex_g2_r3.structural import MissionMemory,sha
    root=Path(root);key=f'stage_{stage:02d}'
    rep=json.loads((root/(key+'_structural_regions.json')).read_text())
    with np.load(root/(key+'_lama.npz'),allow_pickle=False) as data:
        obs=data['observed'].copy();mean=data['mean'].copy();variance=data['variance'].copy()
    matches=[]
    for path in sorted(root.glob('room_stage_*.npz')):
        with np.load(path,allow_pickle=False) as data:
            if sha(data['semantic'])==rep['semantic_sha256']:
                matches.append((path,data['semantic'].copy()))
    if not matches:raise RuntimeError('no exact Room Node semantic hash for this stage')
    memory=MissionMemory.load(root/(key+'_mission_memory.npz'))
    if sha(obs)!=rep['snapshot_sha256']:raise RuntimeError('snapshot SHA mismatch')
    state=json.loads((root/(key+'_commitment_state.json')).read_text())
    return obs,mean,variance,matches[-1][1],memory,rep,state,matches[-1][0].name


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('root');parser.add_argument('stage',type=int)
    parser.add_argument('--output',type=str,default='')
    args=parser.parse_args()
    obs,mean,var,sem,memory,rep,state,matched=load_stage(args.root,args.stage)
    robot=tuple(rep['robot_core_rc'])
    pending=state['pending_goals']
    if not pending:
        print(json.dumps({'status':'NO_PENDING_GOALS','robot_rc':robot}));return
    # Fallback to source stage's robot position as a validated local rejoin anchor.
    # This is NOT the whole MapEx frontier handoff or mission exit certificate.
    anchor=robot
    candidates=prefixes('PENDING',pending)
    # Reconstruct ONLY prior realized evidence from R3.1 commitment receipts.
    # Do not use future state or hidden-GT retrospective metrics.
    cfg=R4Config(); ledger=EvidenceLedger()
    for i,visit in enumerate(state.get('visits',[]),1):
        for rid,v in visit.get('by_region',{}).items():
            expected=int(v['expected']);realized=int(v['realized'])
            row=ledger.regions.setdefault(str(int(rid)),RegionEvidence())
            row.expected+=expected;row.realized+=realized;row.last_scan=i
            goal=tuple(visit['goal_rc'])
            if expected>=cfg.mismatch_min_expected and realized<cfg.mismatch_fraction*expected:
                if all(sum((x-y)**2 for x,y in zip(goal,q))>=100 for q in row.failed_views):
                    row.failed_views.append(goal)
    scorer,chains,invalid=from_r3_snapshot(obs,mean,var,sem,robot,memory,rep,candidates,
             anchor,ledger,cfg,source_hash=rep['snapshot_sha256'])
    outcomes=[scorer.evaluate(chain) for chain in chains]
    current=next((c for c in chains if c.label==f'PENDING:{len(pending)}'),None)
    selected=scorer.choose(current,[c for c in chains if c is not current])
    report={'run_folder':str(Path(args.root).resolve()),'stage':args.stage,'semantic_source':matched,
            'room_rc':robot,'mission_unknown':rep['mission_unknown'],
            'planning_without_GT':True,'not_a_counterfactual_rollout':True,
            'anchor_is_current_robot_not_native_mapex_rejoin':True,
            'pending_goals':pending,'prior_observation_evidence':ledger.to_json(),
            'candidate_rejections':invalid,
            'recommendation':selected,'scored_prefixes':outcomes}
    output=json.dumps(report,ensure_ascii=False,indent=2)
    if args.output:Path(args.output).write_text(output)
    else:print(output)

if __name__=='__main__':main()
