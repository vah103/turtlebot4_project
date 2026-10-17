"""Python 3.6 adapter: exact RoomRun epoch -> Python3.12 G2-R4 goal planner.

Native MapEx (LaMa, WFD, A*, LiDAR, robot) remains unchanged except goal
selection in LOCAL room mode. GT is never passed to the G2 subprocess.
"""
from __future__ import print_function
import os,sys,json,time,hashlib,subprocess,select,atexit
import numpy as np
import pyastar2d
from room_native_integration import safe_json
from pathlib import Path

ROOT=Path(__file__).resolve().parent
class G2Run(object):
    def __init__(self,exp_dir,timeout=240):
        self.root=os.path.join(os.path.abspath(exp_dir),"g2_r4")
        if os.path.exists(self.root):raise RuntimeError('G2 duplicate output')
        os.makedirs(self.root)
        self.decdir=os.path.join(self.root,"decisions");os.makedirs(self.decdir)
        self.err=open(os.path.join(self.root,"sidecar.stderr"),"wb")
        self.proc=subprocess.Popen(["/usr/bin/python3","-u",str(ROOT/"g2_r4_policy_sidecar.py")],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.err,
            universal_newlines=True,bufsize=1)
        self.timeout=timeout
        self.local_lock=False
        self.selected_goal=None
        self.native_pad=500
        hello=self._recv()
        if hello.get("event")!="READY":raise RuntimeError("G2-R4 sidecar not ready "+repr(hello))
        safe_json(os.path.join(self.root,"identity.json"),
                  {"sidecar":hello,"policy":"G2_R4_GOAL_CHAIN","user_go":True,
                   "native_run04_backend":True,"no_robot_control":True,
                   "same_native_sensor_scan_and_3_lama":True})
        atexit.register(self.close)
    def _recv(self):
        until=time.time()+self.timeout
        while time.time()<until:
            ready,_,_=select.select([self.proc.stdout],[],[],max(.02,min(1,until-time.time())))
            if ready:
                row=self.proc.stdout.readline()
                if not row:raise RuntimeError('G2-R4 sidecar EOF')
                return json.loads(row)
            if self.proc.poll() is not None:raise RuntimeError('G2-R4 sidecar exit')
        raise RuntimeError('G2-R4 planning timeout, fail closed')
    def close(self):
        p=getattr(self,'proc',None)
        if p is not None and p.poll() is None:
            try:p.stdin.write('{"action":"CLOSE"}\n');p.stdin.flush();p.wait(timeout=3)
            except Exception:p.terminate()
        e=getattr(self,'err',None)
        if e and not e.closed:e.close()
    def needs_replan(self,pose,obsgrid,goal,threshold_m,ppm):
        if not self.local_lock:return None
        if goal is None:return True
        p=np.asarray(pose,dtype=int);g=np.asarray(goal,dtype=int)
        # Exact cell arrival for G2 viewpoint goals only. Native A* pseudo
        # controller clamps the final <=3-cell step to the actual endpoint.
        # Never mark a viewpoint complete merely because it is within 1 m.
        if np.array_equal(g,p):return True
        if np.any(g<0) or np.any(g>=np.asarray(obsgrid.shape)):return True
        if not np.isfinite(obsgrid[tuple(g)]):return True
        # Existing native planner remains collision authority; the room scope
        # is the same cached one from the last Room Node epoch.
        path=pyastar2d.astar_path(np.ascontiguousarray(obsgrid,dtype=np.float32),
            p.astype(np.int32),g.astype(np.int32),allow_diagonal=False)
        return path is None
    def choose(self,t,room_run,cur_pose):
        if room_run.last_scope is None or room_run.last_scope['mode']!='LOCAL':
            self.local_lock=False;self.selected_goal=None
            return None
        n=room_run.epoch
        base=os.path.join(room_run.inferdir,"decision_%06d"%n)
        req={"action":"CHOOSE","native_tick":int(t),
             "room_id":int(room_run.last_scope["room_id"]),
             "robot_core_rc":(np.asarray(cur_pose,dtype=int)-self.native_pad).tolist(),
             "room_node_input":base+"_input.npz",
             "room_node_semantic":base+"_semantic.npz"}
        t0=time.time()
        self.proc.stdin.write(json.dumps(req)+"\n");self.proc.stdin.flush()
        res=self._recv()
        if res.get('event')!='RESULT':raise RuntimeError('G2-R4 planner failure '+str(res)[:2000])
        if res['native_tick']!=int(t) or res['room_id']!=req['room_id']:
            raise RuntimeError('G2-R4 epoch mismatch')
        expected_hash=room_run.last_scope
        if res['decision']=='GOAL':
            g=np.asarray(res['goal_core_rc'],dtype=np.int32)+self.native_pad
            if not (0<=res['goal_core_rc'][0]<room_run.active_mask.shape[0] and
                    0<=res['goal_core_rc'][1]<room_run.active_mask.shape[1] and
                    room_run.active_mask[tuple(res['goal_core_rc'])]):
                raise RuntimeError('G2-R4 goal outside active Room Node')
            grid=np.asarray(room_run.grid(np.zeros((1,1))),dtype=np.float32)
            if grid.shape[0]<=int(g[0]) or grid.shape[1]<=int(g[1]):
                raise RuntimeError('G2-R4 grid frame mismatch')
            route=pyastar2d.astar_path(np.ascontiguousarray(grid),np.asarray(cur_pose,dtype=np.int32),
                                     g.astype(np.int32),allow_diagonal=False)
            if route is None:raise RuntimeError('G2-R4 goal violates native knownfree A*')
            self.local_lock=True;self.selected_goal=g
        else:
            g=None
            self.local_lock=False;self.selected_goal=None
        res['native_goal_global_rc']=g.tolist() if g is not None else None
        res['native_goal_validation']='knownfree_room_ASTAR' if g is not None else 'fallback_global'
        res['native_adaptor_time_s']=round(time.time()-t0,3)
        res['was_RoomRun_LOCAL']=True
        safe_json(os.path.join(self.decdir,'decision_%06d.json'%n),res)
        return g
