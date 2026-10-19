"""Native2D Room Node V4-R4.1 scope controller (Python 3.6 host).
Only execution-scoring scope changes. LaMa input and physical map untouched.
"""
from __future__ import print_function
import os, sys, time, json, hashlib, subprocess, select, atexit
import numpy as np
import cv2
import pyastar2d
import torch

ROOT=os.path.dirname(os.path.abspath(__file__))
def sha(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def safe_json(path,obj):
    if os.path.exists(path):raise IOError("output collision "+path)
    temp=path+".partial"
    with open(temp,"w") as f:
        json.dump(obj,f,indent=2,sort_keys=True)
        f.flush();os.fsync(f.fileno())
    os.rename(temp,path)

import g2_pathwise_union_r2 as g2_pvu

class RoomRun(object):
    def __init__(self,run_dir,timeout=180):
        self.root=os.path.join(os.path.abspath(run_dir),"room_r41")
        if os.path.exists(self.root):raise RuntimeError("R41 run output collision")
        os.makedirs(self.root)
        self.inferdir=os.path.join(self.root,"inference");os.makedirs(self.inferdir)
        self.decdir=os.path.join(self.root,"decisions");os.makedirs(self.decdir)
        self.stepsdir=os.path.join(self.root,"steps");os.makedirs(self.stepsdir)
        self.err=open(os.path.join(self.root,"sidecar.stderr"),"wb")
        self.proc=subprocess.Popen([os.environ.get("MX_R41_PYTHON","/usr/bin/python3"),"-u",os.path.join(ROOT,"sidecar.py")],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.err,universal_newlines=True,bufsize=1)
        self.timeout=timeout
        self.epoch=0;self.last_mode="GLOBAL";self.room_id=None;self.next_room_id=1
        self.prior_mask=None;self.scope_grid=None;self.active_mask=None;self.last_scope=None
        self.last_score_params=None;self.last_native_tick=-1;self.distance_m=0.;self.previous_pose_rc=None
        atexit.register(self.close)
        hello=self._recv()
        if hello.get("event")!="READY":raise RuntimeError("R41 init failure: "+repr(hello))
        safe_json(os.path.join(self.root,"identity.json"),{"source":hello,"host_python":sys.version,
             "model_scope":"CURRENT_ONLY","no_future_or_gt_online":True})
    def _recv(self):
        deadline=time.time()+self.timeout
        while time.time()<deadline:
            ready,_,_=select.select([self.proc.stdout],[],[],max(.01,min(1,deadline-time.time())))
            if ready:
                line=self.proc.stdout.readline()
                if not line:raise RuntimeError("R41 sidecar EOF")
                return json.loads(line)
            if self.proc.poll() is not None:raise RuntimeError("R41 sidecar exit "+str(self.proc.returncode))
        raise RuntimeError("R41 sidecar timeout")
    def close(self):
        proc=getattr(self,"proc",None)
        if proc is not None and proc.poll() is None:
            try:
                proc.stdin.write('{"action":"CLOSE"}\n');proc.stdin.flush();proc.wait(timeout=5)
            except Exception:
                proc.terminate()
                try:proc.wait(timeout=3)
                except Exception:proc.kill()
        err=getattr(self,"err",None)
        if err and not err.closed:err.close()
    def _inference(self,obs,mean,var,res):
        n=self.epoch
        base=os.path.join(self.inferdir,"decision_%06d"%n)
        paths=(base+"_input.npz",base+"_semantic.npz",base+"_report.json")
        if any(os.path.exists(x) for x in paths):raise RuntimeError("duplicate inference")
        np.savez_compressed(paths[0],observed=obs,mean=mean.astype(np.float32),variance=var.astype(np.float32))
        request={"action":"INFER","decision":n,"input":paths[0],"output":paths[1],"report":paths[2],"resolution":res}
        self.proc.stdin.write(json.dumps(request)+"\n");self.proc.stdin.flush()
        msg=self._recv()
        if msg.get("event")!="OK" or msg.get("decision")!=n:
            raise RuntimeError("exact R4.1 failed: "+repr(msg))
        with np.load(paths[1],allow_pickle=False) as f:sem=f["semantic"].copy()
        if sem.shape!=obs.shape or not np.isin(sem,[0,1,2,3,4]).all():
            raise RuntimeError("R4.1 output semantic invalid")
        return sem,paths
    @staticmethod
    def _room_containing_pose(obs,sem,point):
        comp_mask=(sem==1)&(obs!=2)
        n,lab=cv2.connectedComponents(comp_mask.astype("uint8"),connectivity=4)
        y,x=point
        if obs[y,x]!=0 or lab[y,x]==0:return None
        mask=lab==lab[y,x]
        seen=int((mask&(obs==0)).sum())
        if seen<40:return None
        y0=max(0,y-2);y1=min(obs.shape[0],y+3)
        x0=max(0,x-2);x1=min(obs.shape[1],x+3)
        free=obs[y0:y1,x0:x1]==0
        agree=float((mask[y0:y1,x0:x1]&free).sum())/max(1,int(free.sum()))
        if agree<.60:return None
        return mask
    @staticmethod
    def _astar(grid,robot,goal):
        if not np.isfinite(grid[tuple(robot)]) or not np.isfinite(grid[tuple(goal)]):return None
        return pyastar2d.astar_path(np.ascontiguousarray(grid,dtype=np.float32),
            np.asarray(robot,dtype=np.int32),np.asarray(goal,dtype=np.int32),allow_diagonal=False)
    def score(self, t, mapper_obs, robot, planner_grid, candidates, native_frontier_fn, native_scorer, pose_list,
              native_pred, pred_cfg, padded_obs, mean, var, native_pad, model_pad_top, model_pad_left, ppm, prediction_wall_time_s):
        started=time.time()
        if mean is None or var is None:raise RuntimeError("R41 needs same-epoch LaMa mean/variance")
        full=np.asarray(mapper_obs)
        if full.ndim!=2 or any(z<=2*native_pad for z in full.shape):raise ValueError("bad native obs shape")
        h=full.shape[0]-2*native_pad;w=full.shape[1]-2*native_pad
        core=full[native_pad:native_pad+h,native_pad:native_pad+w]
        if not np.isin(core,[0,.5,1]).all():raise ValueError("MapEx obs coding changed")
        obs=np.ones(core.shape,np.uint8);obs[core==0]=0;obs[core==1]=2
        m=np.asarray(mean)
        v=var.detach().cpu().numpy() if hasattr(var,"detach") else np.asarray(var)
        model_y=native_pad+model_pad_top;model_x=native_pad+model_pad_left
        if m.shape!=v.shape or np.shape(native_pred)!=m.shape or np.shape(padded_obs)!=m.shape:
            raise ValueError("LaMa maps not aligned")
        if model_y+h>m.shape[0] or model_x+w>m.shape[1]:raise ValueError("invalid LaMa offset")
        meanraw=m[model_y:model_y+h,model_x:model_x+w].copy()
        varraw=v[model_y:model_y+h,model_x:model_x+w].copy()
        self.epoch+=1
        sem,paths=self._inference(obs,meanraw,varraw,1./ppm)
        r=np.asarray(robot).astype(int)
        if self.previous_pose_rc is None:
            self.previous_pose_rc=r.astype(float)
        self.cell_m=1./float(ppm)
        pts=np.asarray(candidates).reshape((-1,2)).astype(int)
        if len(pts)==0:raise RuntimeError("native candidate set empty")
        if np.shape(planner_grid)!=full.shape:raise ValueError("planner mismatch")
        at=tuple((r-np.array([native_pad,native_pad])).tolist())
        if not(0<=at[0]<h and 0<=at[1]<w):raise RuntimeError("robot outside raw map")
        mask=self._room_containing_pose(obs,sem,at)
        mode="GLOBAL";reason="ROBOT_NOT_IN_CONFIRMED_ROOM";accepted=list(range(len(pts)))
        plan_grid=None
        if mask is not None:
            room_id=(self.room_id if self.room_id is not None and self.prior_mask is not None and
                     int((mask&self.prior_mask).sum())>max(8,int(mask.sum()*.15))
                     else self.next_room_id)
            if room_id==self.next_room_id:self.next_room_id+=1
            self.room_id=room_id
            self.prior_mask=mask.copy()
            nav=np.asarray(planner_grid).copy()
            fullmask=np.zeros(full.shape,bool)
            fullmask[native_pad:native_pad+h,native_pad:native_pad+w]=mask
            nav[~fullmask]=np.inf
            # Re-run the ORIGINAL MapEx WFD on a room-scoped observed map.
            # Filtering global WFD representatives alone can miss valid local frontiers.
            scoped_frontier_obs=np.ones(full.shape,dtype=full.dtype)
            scoped_frontier_obs[fullmask]=full[fullmask]
            local_pts,_,_=native_frontier_fn(scoped_frontier_obs)
            local_pts=np.asarray(local_pts).reshape((-1,2)).astype(int)
            eligible_local=[]
            for p in local_pts:
                q=tuple((p-np.array([native_pad,native_pad])).tolist())
                if (0<=q[0]<h and 0<=q[1]<w and mask[q] and obs[q]==0 and
                    self._astar(nav,r,p) is not None):
                    eligible_local.append(p)
            if eligible_local:
                mode="LOCAL";reason="ROOM_WFD_REACHABLE_FRONTIER"
                plan_grid=nav
            else:reason="LOCAL_WFD_EXHAUSTED_RETURN_GLOBAL"
        else:
            self.room_id=None;self.prior_mask=None
        filtered=np.asarray(eligible_local).reshape((-1,2)) if mode=="LOCAL" else pts
        original_scorer=native_scorer
        if mode=="LOCAL":
            roi=np.zeros(m.shape,bool)
            roi[model_y:model_y+h,model_x:model_x+w]=mask
            scope_obs=np.asarray(padded_obs).copy();scope_obs[~roi]=1.
            scope_mean=np.asarray(mean).copy();scope_mean[~roi]=1.
            scope_pred=np.asarray(native_pred).copy();scope_pred[~roi]=1.
            scope_var=np.asarray(v).copy();scope_var[~roi]=0.
            scope_var=torch.from_numpy(scope_var).to(var.device) if hasattr(var,"device") else torch.from_numpy(scope_var)
            inputs=(scope_pred,scope_obs,scope_mean,scope_var)
        else:inputs=(native_pred,padded_obs,mean,var)
        score_start=time.time()
        scored=original_scorer(filtered,robot,pose_list,inputs[0],pred_cfg,
                               obs_map=inputs[1],mean_map=inputs[2],var_map=inputs[3])
        score_sec=time.time()-score_start
        costs=np.asarray(scored[1]).reshape(-1)
        if len(costs)!=len(filtered) or not np.isfinite(costs).all():
            raise RuntimeError("invalid scoped scores")
        g2_pvu_audit={"enabled":False,"reason":"ORIGINAL_GLOBAL_MAPEX_UNCHANGED"}
        if mode=="LOCAL" and len(filtered)>1:
            # G1 WFD candidate set and full native clearance-weighted A* map
            # are passed UNMODIFIED. Only current obs+LaMa are used.
            navcore=np.asarray(nav[native_pad:native_pad+h,native_pad:native_pad+w],dtype=np.float32)
            origin=np.asarray(robot,dtype=np.int32)-native_pad
            targets=np.asarray(filtered,dtype=np.int32)-native_pad
            chosen,g2_pvu_audit=g2_pvu.select_first_frontier(
                origin,targets,costs.copy(),navcore,
                core,meanraw,varraw,mask,self.cell_m,
                int(pred_cfg['laser_range_m']*pred_cfg['pixel_per_meter']),
                int(pred_cfg['num_laser']),
                1.0)
            # Retain original cost ordering except the best G2 candidate.
            # Signed offset is chosen to make the argmin deterministic,
            # not a new hand-tuned score/utility weight.
            modified=costs.copy()
            modified[chosen]=float(np.min(costs)-max(1.,abs(float(np.min(costs)))))
            scored=list(scored);scored[1]=modified;scored=tuple(scored)
            costs=np.asarray(scored[1]).reshape(-1)
            print("G2_PVU_FIRST",int(t),"room",int(room_id),
                  "original",int(g2_pvu_audit["baseline_index"]),
                  "first",int(chosen),"pair",g2_pvu_audit.get("second_index"),
                  "runtime_s",g2_pvu_audit.get("runtime_s"),flush=True)
        self.scope_grid=plan_grid;self.active_mask=mask if mode=="LOCAL" else None
        self.last_scope={"mode":mode,"reason":reason,"room_id":self.room_id,"decision":self.epoch}
        self.last_score_params=(t,robot,pose_list,native_pred,pred_cfg,padded_obs,mean,var,pts,original_scorer)
        rows=[{"index":int(i),"row":int(p[0]),"col":int(p[1]),
             "cost":float(costs[i]),"origin":"ROOM_WFD" if mode=="LOCAL" else "MAPEX_GLOBAL_WFD"}
             for i,p in enumerate(filtered)]
        safe_json(os.path.join(self.decdir,"decision_%06d.json"%self.epoch),
            {"decision":self.epoch,"native_tick":int(t),"mode":mode,"reason":reason,
             "room_id":self.last_scope["room_id"],"robot_rc":r.tolist(),
             "frontiers":rows,"original_global_frontier_rc":pts.tolist(),
             "global_candidate_count":len(pts),
             "local_candidate_count":len(filtered) if mode=="LOCAL" else 0,
             "seen_room_free":int((mask&(obs==0)).sum()) if mask is not None else 0,
             "input_sha256":[sha(obs),sha(meanraw),sha(varraw)],
             "room_node_input":paths[0],"room_node_semantic":paths[1],"room_node_report":paths[2],
             "elapsed_room_plus_score_s":time.time()-started,"elapsed_score_s":score_sec,
             "g2_pvu":g2_pvu_audit,
             "elapsed_lama_prediction_s":float(prediction_wall_time_s),"cumulative_distance_m_before_decision":self.distance_m,
             "initial_native_map_unchanged":True,"scoring_scope":"ROOM_ROI" if mode=="LOCAL" else "GLOBAL",
             "previous_mode":self.last_mode,"model_used":"MapEx LaMa original","GT_used_as_input":False})
        self.last_mode=mode
        return scored
    def grid(self,original):
        return self.scope_grid if self.scope_grid is not None else original
    def log_plan(self,t,pose,goal,path):
        route=np.asarray(path)
        if route.ndim!=2 or route.shape[1]!=2:raise RuntimeError("invalid A* path")
        if self.active_mask is not None:
            # Validate EVERY native A* point lies in currently classified ROOM.
            a=route.astype(int)
            y=a[:,0]-500;x=a[:,1]-500
            if np.any(y<0)|np.any(x<0)|np.any(y>=self.active_mask.shape[0])|np.any(x>=self.active_mask.shape[1]) or not self.active_mask[y,x].all():
                raise RuntimeError("LOCAL route exits Room Node region")
        safe_json(os.path.join(self.stepsdir,"tick_%06d_plan.json"%int(t)),
          {"tick":int(t),"room_epoch":self.epoch,"scope":self.last_scope,
          "from_rc":np.asarray(pose).tolist(),"goal_rc":np.asarray(goal).tolist(),
          "real_native_astar_rc":route.tolist(),"astar_length_m":float(np.abs(np.diff(route,axis=0)).sum())*self.cell_m})
    def log_scan(self,t,pose,obs,step_time_s):
        file=os.path.join(self.stepsdir,"tick_%06d_postscan.npz"%int(t))
        if os.path.exists(file):raise RuntimeError("duplicate postscan")
        p=np.asarray(pose,dtype=float)
        if self.previous_pose_rc is not None:
            self.distance_m+=float(np.linalg.norm(p-self.previous_pose_rc))*self.cell_m
        self.previous_pose_rc=p.copy()
        np.savez_compressed(file,pose_rc=np.asarray(pose),observed=np.asarray(obs))
        safe_json(os.path.join(self.stepsdir,"tick_%06d_summary.json"%int(t)),
            {"native_tick":int(t),"score_epoch":self.epoch,"current_scope":self.last_scope,
             "pose_rc":p.astype(int).tolist(),"time_step_s":float(step_time_s),
             "cumulative_distance_m":self.distance_m,"postscan_hash":sha(obs)})
    def recover_global(self,tick):
        """If LOCAL path becomes invalid, preserve causal inputs and rerank GLOBAL."""
        if self.last_score_params is None or self.last_scope is None or self.last_scope["mode"]!="LOCAL":
            raise RuntimeError("no LOCAL epoch to recover")
        t,robot,poses,pred,cfg,obs,mean,var,pts,scorer=self.last_score_params
        if int(tick)!=int(t):raise RuntimeError("recovery belongs to different scoring epoch")
        result=scorer(pts,robot,poses,pred,cfg,obs_map=obs,mean_map=mean,var_map=var)
        costs=np.asarray(result[1])
        if not len(costs) or not np.isfinite(costs).all():raise RuntimeError("bad global fallback scores")
        self.scope_grid=None
        self.active_mask=None
        old=dict(self.last_scope)
        self.last_scope={"mode":"GLOBAL","reason":"LOCAL_PATH_REJECTED_REJOIN_GLOBAL","room_id":old["room_id"],"decision":self.epoch}
        self.last_mode="GLOBAL"
        safe_json(os.path.join(self.decdir,"decision_%06d_global_fallback.json"%self.epoch),
            {"decision":self.epoch,"native_tick":int(tick),"previous":old,
             "new_scope":self.last_scope,"global_candidate_scores":costs.astype(float).tolist(),
             "global_candidate_rc":pts.tolist(),"reason":"LOCAL_PATH_FAILURE_REPLAN"})
        return result
