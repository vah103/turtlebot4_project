"""CLI: python -m mapex_g2_v1.cli --snapshot D02_input.npz --semantic D02_semantic.npz --robot-rc 147,111 --output /tmp/plan.json"""
import argparse
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from .planner import G2Snapshot,G2Config,plan


def main():
    p=argparse.ArgumentParser(description='G2-V1 causal room viewpoint sequence planner')
    p.add_argument('--snapshot',required=True,help='NPZ with observed/mean/variance of ONE current decision')
    p.add_argument('--semantic',required=True,help='NPZ with current G1 semantic array, not future G1')
    p.add_argument('--robot-rc',required=True,help='row,col in SAME array coordinates, e.g. 147,111')
    p.add_argument('--output',required=True,help='JSON destination')
    p.add_argument('--decision-id',default='current')
    p.add_argument('--ensemble',help='Optional NPZ containing ensemble (K,H,W) as array "ensemble"')
    p.add_argument('--config-json',help='Optional JSON patch to documented G2Config defaults; no hidden per-D02 tuning')
    p.add_argument('--frontiers',help='Optional JSON list of row,col frontier positions')
    p.add_argument('--visualization-png',help='Optional schematic plan PNG')
    p.add_argument('--coverage-npz',help='Optional current-map-only projected coverage mask NPZ')
    a=p.parse_args()
    robot=tuple(int(x) for x in a.robot_rc.split(','))
    if len(robot)!=2:raise ValueError('robot-rc must be row,col')
    with np.load(a.snapshot,allow_pickle=False) as f:
        observed=f['observed'].copy();mean=f['mean'].copy();variance=f['variance'].copy()
    with np.load(a.semantic,allow_pickle=False) as f:
        semantic=f['semantic'].copy()
    ensemble=None
    if a.ensemble:
        with np.load(a.ensemble,allow_pickle=False) as f:ensemble=f['ensemble'].copy()
    frontiers=[]
    if a.frontiers:frontiers=[tuple(int(v) for v in row) for row in json.loads(Path(a.frontiers).read_text())]
    conf=dict(json.loads(Path(a.config_json).read_text())) if a.config_json else {}
    cfg=G2Config(**conf)
    result=plan(G2Snapshot(observed,mean,variance,semantic,robot,frontiers,ensemble,a.decision_id),cfg)
    dest=Path(a.output);dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    if a.visualization_png or a.coverage_npz:
        from .visualization import save_visualization
        import tempfile
        name=a.visualization_png or str(Path(tempfile.gettempdir())/'g2_v1_preview.png')
        save_visualization(G2Snapshot(observed,mean,variance,semantic,robot,frontiers,ensemble,a.decision_id),
                           result,cfg,name,a.coverage_npz)
    print(json.dumps({'output':str(dest),'status':result['status'],'goals':result['goal_count'],
                      'distance_m':result['route_distance_m'],'candidates':result['search']['candidate_goals'],
                      'runtime_s':result['planning_runtime_s']}))


if __name__=='__main__':main()
