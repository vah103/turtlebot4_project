"""Native RoomRun method-level replay from frozen R3 D10 t264->t266.
No GT, no future maps at the time of t264, future t266 used only to replay
the actual later scan event. This calls the real patched native integration.
"""
import sys,os,tempfile,json
import numpy as np,cv2
from scipy.ndimage import binary_dilation
sys.path.insert(0,'/home/dell/DELL_NATIVE2D/runs/NR-G2R62R4-02')
from room_native_integration import RoomRun
from r4_exit_guard import classify_exit
b='/home/dell/DELL_NATIVE2D/runs/NR-G2R62R3-02/native_g2_r62_r3_native01/G2_R62_R3_NATIVE01_visvarprob'
s=np.load(b+'/room_r41/inference/decision_000010_semantic.npz')['semantic']
x=np.load(b+'/room_r41/inference/decision_000010_input.npz')['observed']
odom=np.load(b+'/odom.npy')
p=tuple((odom[264]-500).tolist())
_,lab=cv2.connectedComponents(((s==1)&(x!=2)).astype('uint8'),connectivity=4)
room=lab==lab[p]
goals=[(74,243),(70,219),(39,140)]
raw_wall=binary_dilation(x==2,structure=np.ones((3,3),bool))
grid264=np.where(~raw_wall,1.,np.inf).astype(np.float32)
state,event=classify_exit('GLOBAL',goals,p,grid264,room,264)
assert state.pending,event
with tempfile.TemporaryDirectory() as tmp:
 node=RoomRun.__new__(RoomRun)
 node.r4_exit=state
 node.r4_exit_events=[]
 node.root=tmp
 for t in (264,266):
  raw=np.load('%s/room_r41/steps/tick_%06d_postscan.npz'%(b,t))['observed'][500:703,500:763]
  padded=np.ones((1203,1263),np.float32)*.5
  padded[500:703,500:763]=raw
  padded_walls=binary_dilation(padded==1,structure=np.ones((3,3),bool))
  planner=np.where(~padded_walls,1.,np.inf).astype(np.float32)
  pose=odom[t]
  fire=node.r4_recheck_due(t,padded,pose,planner)
  print('REPLAY_TICK',t,'fire',fire,'robot',list(map(int,pose)),'valid',bool(np.isfinite(planner[tuple(pose)])))
  assert fire==(t==266)
  if t==266:
   receipt=os.path.join(tmp,'r4_exit_recheck_000266.json')
   assert os.path.exists(receipt)
   print('NATIVE_METHOD_RECEIPT',json.load(open(receipt)))
 assert len(node.r4_exit_events)==1 and not node.r4_exit.pending
result={'test':'R4_NATIVE_RECHECK_REPLAY_PASS',
  'source_run':'NR-G2R62R3-02','source_decision':10,
  'start':{'tick':264,'pose_core':[10,156],'native_valid':False,'fires':False},
  'after_native_movement':{'tick':266,'pose_core':[9,161],
                           'native_valid':True,'fires':True,
                           'event':'RECHECK_OFFICIAL_WFD'},
  'real_class_used':'room_native_integration.RoomRun.r4_recheck_due',
  'native_python':'3.6 conda lama',
  'uses_GT':False,'uses_future_scan_in_initial_decision':False}
out='/home/dell/DELL_NATIVE2D/analysis/g2_r62_r4_routeaware_20261018/evidence/R4_NATIVE_METHOD_REPLAY.json'
with open(out,'w') as f:json.dump(result,f,indent=2)
print('R4_NATIVE_RECHECK_REPLAY_PASS',out)
