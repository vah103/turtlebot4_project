"""R3 structural/mission-memory CLI (does not select or execute a goal)."""
from pathlib import Path
import argparse,json
import numpy as np
from .structural import MissionMemory,StructuralConfig,analyze
from .visualize import visualize

def main():
 p=argparse.ArgumentParser()
 p.add_argument('--snapshot',required=True,help='npz observed,mean,variance from current causal inference')
 p.add_argument('--semantic',required=True,help='npz semantic from current R4.1 persistent tracker')
 p.add_argument('--robot-rc',required=True,help='row,col core coordinates')
 p.add_argument('--decision-id',default='current')
 p.add_argument('--memory-in',default=None)
 p.add_argument('--memory-out',required=True)
 p.add_argument('--output',required=True)
 p.add_argument('--visualization-png',default=None)
 p.add_argument('--beams',type=int,default=250)
 args=p.parse_args()
 rc=tuple(int(v) for v in args.robot_rc.split(','))
 with np.load(args.snapshot,allow_pickle=False) as a:
  arrays={key:a[key] for key in ('observed','mean','variance')}
 with np.load(args.semantic,allow_pickle=False) as a:sem=a['semantic']
 mem=MissionMemory.load(args.memory_in) if args.memory_in else MissionMemory.begin(arrays['observed'],sem,rc)
 report,labels,current=analyze(arrays['observed'],arrays['mean'],arrays['variance'],
  sem,rc,mem,StructuralConfig(beams=args.beams),args.decision_id)
 Path(args.output).parent.mkdir(parents=True,exist_ok=True)
 Path(args.memory_out).parent.mkdir(parents=True,exist_ok=True)
 Path(args.output).write_text(json.dumps(report,ensure_ascii=False,indent=2))
 mem.save(args.memory_out)
 if args.visualization_png:visualize(arrays['observed'],arrays['mean'],current,mem,labels,report,args.visualization_png)
 print(json.dumps({'decision':args.decision_id,'status':report['status'],
  'mission_unknown':report['mission_unknown'],'current_room_unknown':report['current_room_unknown'],
  'historical_unknown_outside_current':report['unknown_outside_current_room'],
  'regions':len(report['regions']),'vantages':report['currently_safe_viewpoint_candidates'],
  'report':args.output,'memory':args.memory_out,'visualization':args.visualization_png}))

if __name__=='__main__':main()
