"""G2-R7-R3 native LOCAL goal adapter, Python3.6; native A* executes.

R7 runs in independent modern Python sidecar. No GT is passed. No R4/R6
goal heuristics imported. Every next goal certified through current MapEx
inflated observed-FREE grid and EXACT native pyastar2d.
"""
from __future__ import print_function
import os,json,time,select,subprocess,threading,atexit
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import numpy as np
import pyastar2d

ROOT=Path(__file__).resolve().parent


def saved_json(file,data):
    with Path(file).open("w") as f:
        json.dump(data,f,indent=2,sort_keys=True)
        f.write("\n")


class G2Run(object):
    def __init__(self,exp_dir):
        self.root=Path(exp_dir)/"g2_r7"
        self.root.mkdir(parents=True,exist_ok=False)
        (self.root/"decisions").mkdir()
        (self.root/"updates").mkdir()
        self.native_pad=500
        self.timeout=50
        self.err=(self.root/"sidecar.stderr").open("w")
        self.proc=subprocess.Popen(["/usr/bin/python3","-u",
                       str(ROOT/"r7_native_sidecar.py")],
                       stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                       stderr=self.err,universal_newlines=True,bufsize=1)
        self.lock=threading.Lock()
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.pending=None
        self.local_lock=False
        self.current_goal=None
        self.room_id=None
        self.last_scope_mask=None
        self.last_scan_pose=None
        self.force_room_wfd=False
        self.heavy_required=False
        self.scan_count=0
        self._first_structural_checkpoint_pending=True
        hello=self._recv()
        if hello.get("event")!="READY":
            raise RuntimeError("R7_SERVICE_NOT_READY "+str(hello))
        saved_json(self.root/"identity.json",
                   {"policy":"G2_R7_R3_STANDALONE",
                    "no_hindsight_gt":True,"sidecar":str(ROOT/"r7_native_sidecar.py"),
                    "native_astar_mandatory":True})
        atexit.register(self.close)

    def _recv(self):
        deadline=time.monotonic()+self.timeout
        while time.monotonic()<deadline:
            ready,_,_=select.select([self.proc.stdout],[],[],0.2)
            if ready:
                line=self.proc.stdout.readline()
                if not line:raise RuntimeError("R7_SERVICE_EOF")
                msg=json.loads(line)
                if msg.get("event")=="ERROR":
                    raise RuntimeError("R7_SERVICE_ERROR "+str(msg))
                return msg
            if self.proc.poll() is not None:raise RuntimeError("R7_SERVICE_EXITED")
        raise RuntimeError("R7_SERVICE_TIMEOUT")

    def _request(self,req):
        with self.lock:
            if self.proc.poll() is not None:
                raise RuntimeError("R7_SERVICE_NOT_ALIVE")
            self.proc.stdin.write(json.dumps(req,default=lambda value:int(value) if isinstance(value,np.integer) else value.tolist() if isinstance(value,np.ndarray) else str(value))+"\n")
            self.proc.stdin.flush()
            return self._recv()

    def _drain(self):
        if self.pending is not None and self.pending.done():
            answer=self.pending.result()
            self.pending=None
            saved_json(self.root/"updates"/
                       ("result_%06d.json"%answer.get("native_tick",0)),answer)

    def close(self):
        future=getattr(self,"pool",None)
        if future is not None:
            future.shutdown(wait=True);self.pool=None
        proc=getattr(self,"proc",None)
        if proc is not None and proc.poll() is None:
            try:
                self._request({"action":"CLOSE"})
                proc.wait(timeout=4)
            except Exception:
                proc.terminate()
        if getattr(self,"err",None) and not self.err.closed:self.err.close()

    def _scope(self,room_run,pose):
        scope=room_run.active_mask
        if scope is None:raise RuntimeError("NO_ACTIVE_ROOM")
        h,w=scope.shape
        grid=room_run.grid(None)
        mask=np.isfinite(grid[500:500+h,500:500+w]) & scope
        robot=tuple((np.asarray(pose,dtype=int)-500).tolist())
        if not mask[robot]:raise RuntimeError("ROBOT_NOT_IN_NATIVE_ROOM_FREE")
        name=self.root/("native_scope_%06d.npz"%room_run.epoch)
        np.savez_compressed(str(name),reachable_knownfree=mask.astype(np.uint8))
        return name,mask

    def _validate(self,gcore,curpose,grid,mask):
        if gcore is None:return None
        y,x=list(map(int,gcore))
        if y<0 or x<0 or y>=mask.shape[0] or x>=mask.shape[1]:
            return None
        if not mask[y,x]:return None
        to=np.asarray((y+500,x+500),dtype=np.int32)
        if not np.isfinite(grid[tuple(to)]):return None
        path=pyastar2d.astar_path(np.ascontiguousarray(grid,dtype=np.float32),
                    np.asarray(curpose,dtype=np.int32),to,allow_diagonal=False)
        return to if path is not None else None

    def choose(self,t,room_run,cur_pose):
        if room_run.last_scope is None or room_run.last_scope["mode"]!="LOCAL":
            self.local_lock=False;self.current_goal=None;self.room_id=None
            self.force_room_wfd=False
            return None
        room_id=int(room_run.last_scope["room_id"])
        if room_id!=self.room_id:
            self.force_room_wfd=False
            self._first_structural_checkpoint_pending=True
        scope,mask=self._scope(room_run,cur_pose)
        n=room_run.epoch
        base=Path(room_run.inferdir)/("decision_%06d"%n)
        req={"action":"CHOOSE","native_tick":int(t),
             "room_id":room_id,
             "robot_core_rc":[int(q) for q in (np.asarray(cur_pose)-500)],
             "room_node_input":str(base)+"_input.npz",
             "room_node_semantic":str(base)+"_semantic.npz",
             "native_scope_npz":str(scope)}
        started=time.perf_counter()
        reply=self._request(req)
        if reply.get("event")!="RESULT" or reply.get("native_tick")!=int(t):
            raise RuntimeError("R7_EPOCH_DRIFT")
        native=self._validate(reply.get("goal_core_rc"),cur_pose,
                              room_run.grid(None),mask)
        if reply["decision"]=="GOAL" and native is None:
            raise RuntimeError("R7_INTENDED_GOAL_NATIVE_ASTAR_FAILED")
        self.local_lock=native is not None
        self.current_goal=native
        self.room_id=room_id
        self.last_scope_mask=room_run.active_mask.copy()
        self.last_scan_pose=np.asarray(cur_pose).copy()
        self.heavy_required=False
        self.force_room_wfd=native is None
        reply["native_goal_rc"]=native.tolist() if native is not None else None
        reply["native_validation"]="INFLATED_KNOWNFREE_ASTAR" if native is not None else "DEFER_WFD"
        reply["adapter_s"]=round(time.perf_counter()-started,4)
        saved_json(self.root/"decisions"/("decision_%06d.json"%n),reply)
        print("R7_R3_NATIVE_CHOOSE",int(t),room_id,reply["plan_phase"],
              reply["goal_sequence_core_rc"],reply["native_validation"],flush=True)
        return native

    def needs_replan(self,pose,grid,goal,threshold_m,ppm):
        if not self.local_lock:return False
        self._drain()
        if self.heavy_required:return True
        if goal is None or np.array_equal(pose,goal):return True
        if not np.isfinite(grid[tuple(np.asarray(goal,dtype=int))]):return True
        return pyastar2d.astar_path(np.ascontiguousarray(grid,dtype=np.float32),
                    np.asarray(pose,dtype=np.int32),
                    np.asarray(goal,dtype=np.int32),allow_diagonal=False) is None

    def cached_next_goal(self,t,room_run,cur_pose,grid):
        if not self.local_lock or self.current_goal is None:return None
        self._drain()
        if self.heavy_required:return None
        if not np.array_equal(np.asarray(cur_pose,dtype=np.int32),self.current_goal):
            return None
        reply=self._request({"action":"ACK_GOAL",
                  "robot_core_rc":[int(q) for q in (np.asarray(cur_pose)-500)]})
        if not reply.get("accepted"):raise RuntimeError("R7_ACK_INCOMPLETE "+str(reply))
        rest=reply.get("remaining_goals_rc",[])
        saved_json(self.root/"decisions"/("arrived_tick_%06d.json"%int(t)),reply)
        print("R7_R3_NATIVE_ARRIVAL",int(t),reply["finished_goal_rc"],
              "next",rest,flush=True)
        if not rest:
            self.local_lock=False
            self.current_goal=None
            self.heavy_required=True
            return None
        # Since previous G1 scope, only known-FREE A* from native is allowed
        # to validate second goal, never the predicted FREE R7 path.
        mask=self.last_scope_mask
        g=self._validate(rest[0],cur_pose,room_run.grid(grid),mask)
        if g is None:
            self.local_lock=False
            self.current_goal=None
            self.heavy_required=True
            return None
        # Refresh LaMa+G1 at the FIRST structural checkpoint (after Goal1)
        # while preserving the already planned Goal2. This is intentionally
        # more expensive than an R6 cached hop but makes Plan2 causal.
        if self._first_structural_checkpoint_pending:
            self._first_structural_checkpoint_pending=False
            self.local_lock=False
            self.current_goal=None
            self.heavy_required=True
            print("R7_R3_FORCE_NEW_LAMA_G1_AFTER_GOAL1",int(t),flush=True)
            return None
        self.current_goal=g
        saved_json(self.root/"decisions"/("cached_tick_%06d.json"%t),
                   {"native_tick":t,"goal_rc":g.tolist(),
                    "native_astar":True,"LaMa_recomputed":False,
                    "cached_Plan1_continued":True})
        return g

    def on_scan(self,t,pose,mapper_obs,room_run):
        if not self.local_lock or not room_run or room_run.last_scope is None:
            return
        if room_run.last_scope["mode"]!="LOCAL":return
        self._drain()
        if self.pending is not None or self.heavy_required:return
        point=np.asarray(pose,dtype=np.int32)
        if self.last_scan_pose is not None and (
            np.linalg.norm(point-self.last_scan_pose)*.1 < .5):return
        mask=self.last_scope_mask
        if mask is None:return
        h,w=mask.shape
        full=np.asarray(mapper_obs)
        core=full[500:500+h,500:500+w]
        observed=np.ones((h,w),np.uint8)
        observed[core==0]=0;observed[core==1]=2
        scope=room_run.grid(None)
        nav=np.isfinite(scope[500:500+h,500:500+w])&mask&(observed==0)
        if not nav[tuple((point-500).tolist())]:
            self.heavy_required=True
            return
        a=self.root/"updates"/("scan_%06d.npz"%t)
        b=self.root/"updates"/("scope_%06d.npz"%t)
        np.savez_compressed(str(a),observed=observed)
        np.savez_compressed(str(b),reachable_knownfree=nav.astype(np.uint8))
        req={"action":"UPDATE","native_tick":int(t)+1,
             "observed_npz":str(a),"native_scope_npz":str(b),
             "robot_core_rc":list((point-500).tolist())}
        self.last_scan_pose=point.copy()
        self.pending=self.pool.submit(self._request,req)
