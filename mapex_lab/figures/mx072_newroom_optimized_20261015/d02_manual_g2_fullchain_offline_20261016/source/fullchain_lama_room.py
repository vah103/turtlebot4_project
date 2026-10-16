#!/usr/bin/env python3
"""Replay R4.1 tracker along frozen counterfactual G2 route (no GT as model input)."""
import os,subprocess,json,time,numpy as np,cv2
from pathlib import Path
B=Path('/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_MANUAL_G2_FULL_CHAIN_20261016')
R=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED')
S=R/'native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41/inference'
def sidecar_action(proc,seq,inp,output,report):
 request={'action':'INFER','decision':seq,'input':str(inp),'output':str(output),'report':str(report),'resolution':.1}
 proc.stdin.write(json.dumps(request)+'\n');proc.stdin.flush()
 reply=json.loads(proc.stdout.readline())
 assert reply['event']=='OK',reply
proc=subprocess.Popen(['/usr/bin/python3',str(R/'sidecar.py')],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,bufsize=1)
assert json.loads(proc.stdout.readline())['event']=='READY'
for j in [1,2]:
 src=S/f'decision_{j:06d}_input.npz'
 out=B/f'replay_{j:02d}_semantic.npz'
 sidecar_action(proc,j,src,out,B/f'replay_{j:02d}_report.json')
 got=np.load(out)['semantic'];source=np.load(S/f'decision_{j:06d}_semantic.npz')['semantic']
 err=int((got!=source).sum())
 print('REPLAY_SOURCE',j,err,flush=True)
 assert err==0
results=[]
for j in [1,2,3,4]:
 path=B/f'stage_{j:02d}_postscan.npz'
 obs=np.load(path)['observed'];rc=tuple(np.load(path)['robot_rc'])
 dst=B/f'stage_{j:02d}_inference.npz'
 env=os.environ.copy();env.update({'PYTHONPATH':'/home/dell/MapEx/scripts:/home/dell/MapEx/lama','OPENBLAS_NUM_THREADS':'2','OMP_NUM_THREADS':'2','MKL_NUM_THREADS':'2'})
 command=['/home/dell/miniforge3/envs/lama/bin/python','/tmp/g2_r2_d02_lama_reinfer.py',str(path),str(dst)]
 p=subprocess.run(command,env=env,capture_output=True,text=True,timeout=120)
 if p.returncode:raise RuntimeError(p.stderr[-1500:])
 sidecar_action(proc,j+2,dst,B/f'stage_{j:02d}_semantic.npz',B/f'stage_{j:02d}_r41_report.json')
 sem=np.load(B/f'stage_{j:02d}_semantic.npz')['semantic']
 _,components=cv2.connectedComponents(((sem==1)&(obs!=2)).astype('uint8'),connectivity=4)
 rid=int(components[rc])
 active=(components==rid)&(rid!=0)
 original=np.load(path)['original_room']
 remaining=(original&(obs==1))
 report=json.load(open(B/f'stage_{j:02d}_r41_report.json'))
 summary={'stage':j,'robot_core_rc':list(map(int,rc)),
 'current_robot_semantic':int(sem[rc]),'current_room_cells':int(active.sum()),
 'current_room_unknown':int((active&(obs==1)).sum()),
 'original_room_unknown':int(remaining.sum()),
 'original_remaining_outside_current_room':int((remaining&~active).sum()),
 'accepted_hypotheses_count':len(report.get('room_hypotheses',[])),
 'inference_source':'original LaMa G1/G2/G3 + stateful V4-R4.1'}
 if j==1:
  ref=Path('/tmp/MX072_RUN04_D02_G2A_20261016/VP1_R41_REPLAY_03_semantic.npz')
  if ref.exists():
   prev=np.load(ref)['semantic'];summary['VP1_R41_previous_semantic_mismatch']=int((sem!=prev).sum())
 print('ROOM_STAGE',json.dumps(summary),flush=True)
 results.append(summary)
proc.stdin.write('{"action":"CLOSE"}\n');proc.stdin.flush()
print('SIDECAR_CLOSED',proc.stdout.readline().strip(),flush=True)
proc.wait(timeout=10)
(B/'fullchain_lama_r41_report.json').write_text(json.dumps(results,indent=2))
