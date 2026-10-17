"""R4.1 D02 motion-action search at a frozen snapshot (no R3 goals supplied)."""
from __future__ import annotations
import json,io,zipfile,argparse,os
from pathlib import Path
import numpy as np
from mapex_g2_r3.structural import MissionMemory,sha
from shadow_readonly import load_stage
from r41_motion import LocalMotionPlanner,MotionConfig
SRC=Path(os.environ.get('R41_D02_SOURCE_ROOT', '/home/dell/mx072_g2_r31_structure_commitment_20261016/D02_R31_FULL_CAUSAL_20261016'))
def load_initial():
    with zipfile.ZipFile(SRC/'D02_G2_R31_CAUSAL_EVIDENCE.zip') as z:
        with np.load(io.BytesIO(z.read('source_D02/decision_000002_input.npz')),allow_pickle=False) as a:
            obs=a['observed'].copy();mean=a['mean'].copy();var=a['variance'].copy()
        with np.load(io.BytesIO(z.read('source_D02/decision_000002_semantic.npz')),allow_pickle=False) as a:
            semantic=a['semantic'].copy()
        with np.load(io.BytesIO(z.read('initial/memory_D02.npz')),allow_pickle=False) as a:
            meta=json.loads(str(a['metadata']))
            mem=MissionMemory(a['original_room'],a['mission'],a['previous_observed'],a['labels'],
                              meta['next_id'],meta['history'])
        report=json.loads(z.read('initial/regions_D02.json'))
    return obs,mean,var,semantic,mem,report

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--stage',type=int,default=0)
    ap.add_argument('--width',type=int,default=12)
    ap.add_argument('--depth',type=int,default=9)
    ap.add_argument('--max-expansions',type=int,default=1050)
    ap.add_argument('--dense-beams',type=int,default=750)
    ap.add_argument('--output',default='')
    a=ap.parse_args()
    if a.stage==0:
        obs,mean,var,semantic,mem,report=load_initial()
    else:
        obs,mean,var,semantic,mem,report,_,_=load_stage(SRC,a.stage)
    cfg=MotionConfig(beam_width=a.width,max_depth=a.depth,max_expansions=a.max_expansions,dense_beams=a.dense_beams)
    planner=LocalMotionPlanner(obs,mean,var,semantic,report['robot_core_rc'],mem,report,
                               motion_cfg=cfg)
    res=planner.solve()
    print(json.dumps({k:v for k,v in res.items() if k not in ('route_rc','return_route_rc','search','action_sequence','execution_prefix_rc')},ensure_ascii=False,indent=2),flush=True)
    print('STAGES',res.get('search',{}).get('depths',[]),flush=True)
    print('ACTIONS',res.get('action_sequence',[]),flush=True)
    print('SEARCH',res.get('search'),flush=True)
    if a.output:
        out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(res,indent=2,ensure_ascii=False))
        print('OUTPUT',out,'BYTES',out.stat().st_size,flush=True)
if __name__=='__main__':main()
