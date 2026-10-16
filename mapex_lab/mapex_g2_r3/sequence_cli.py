"""Pure current-snapshot G2-R3 sequence CLI. Reads precomputed R3 stage A report."""
import argparse,json
from pathlib import Path
import numpy as np
from .structural import MissionMemory
from .sequence import StructuralRoutePlanner,SequenceConfig
from .sequence_viz import make_route_image

def main():
 a=argparse.ArgumentParser()
 a.add_argument('--snapshot',required=True)
 a.add_argument('--semantic',required=True)
 a.add_argument('--memory',required=True,help='R3 memory already updated to this snapshot')
 a.add_argument('--regions',required=True,help='R3 region JSON for same snapshot')
 a.add_argument('--robot-rc',required=True)
 a.add_argument('--decision-id',default='current')
 a.add_argument('--output',required=True)
 a.add_argument('--visualization-png',default=None)
 a.add_argument('--config-json',default=None,help='Optional explicit fixed engineering search budget')
 args=a.parse_args()
 rc=tuple(int(q) for q in args.robot_rc.split(','))
 with np.load(args.snapshot,allow_pickle=False) as f:
  data={k:f[k] for k in ('observed','mean','variance')}
 with np.load(args.semantic,allow_pickle=False) as f:sem=f['semantic']
 mem=MissionMemory.load(args.memory)
 report=json.loads(Path(args.regions).read_text())
 defaults=json.loads(Path(args.config_json).read_text()) if args.config_json else {}
 config=SequenceConfig(decision_id=args.decision_id,**defaults)
 engine=StructuralRoutePlanner(data['observed'],data['mean'],data['variance'],sem,rc,mem,report,config)
 result=engine.solve()
 Path(args.output).parent.mkdir(exist_ok=True,parents=True)
 Path(args.output).write_text(json.dumps(result,ensure_ascii=False,indent=2))
 if args.visualization_png:make_route_image(data['observed'],mem,report,result,rc,args.visualization_png)
 print(json.dumps({'status':result['status'],'decision':args.decision_id,
 'goals':result['goal_sequence'],'mission_unknown':result.get('mission_unknown'),
 'output':args.output,'visualization':args.visualization_png,
 'runtime_s':result.get('search',{}).get('wall_time_s')},ensure_ascii=False))
if __name__=='__main__':main()
