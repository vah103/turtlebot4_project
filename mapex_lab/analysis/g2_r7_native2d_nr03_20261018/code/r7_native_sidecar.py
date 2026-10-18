"""R7 R3 JSONL policy: no GT, no robot actuation, no R4/R6.2 planner.

CHOOSE has exactly current epoch LaMa/G1 map.
UPDATE has observed-only cumulative LiDAR and last *frozen* hypothesis;
async compute Plan2 from this newly observed evidence. Its proposal is only
a structural intention, not native navigation permission.
ACK_GOAL is issued only on exact native goal arrival.
"""
import json,sys,time
import numpy as np
from r7_missions import R7Missions
from r7_dual_plan import R7DualPlan, inspect_contradiction


def npz(path, names):
    with np.load(path,allow_pickle=False) as z:
        return tuple(z[n].copy() for n in names)


class Policy:
    def __init__(self):
        self.manager=None
        self.room_id=None
        self.entry=None
        self.last_tick=-1
        self.obs=None;self.mean=None;self.var=None;self.sem=None;self.nav=None
        self.last_phase=None
        self.number_plan1=0
        self.number_plan2=0
        self.completed=[]
        self.current=None
        self.scan_updates=0
        self.new_obstacles=0

    def _validate_candidates(self,goals,obs,nav):
        # The sidecar provides intent, never claims native-A* validity.
        out=[]
        for q in goals:
            t=tuple(map(int,q))
            if (0<=t[0]<obs.shape[0] and 0<=t[1]<obs.shape[1] and
                obs[t]==0 and nav[t]):
                out.append(t)
            else:
                break  # Never jump ahead across an unverified goal
        return out

    def choose(self,r):
        tick=int(r["native_tick"])
        if tick<self.last_tick:raise ValueError("NONMONOTONIC_NATIVE_TICK")
        self.last_tick=tick
        roomid=int(r["room_id"])
        robot=tuple(map(int,r["robot_core_rc"]))
        obs,mean,var=npz(r["room_node_input"],("observed","mean","variance"))
        sem=npz(r["room_node_semantic"],("semantic",))[0]
        nav=npz(r["native_scope_npz"],("reachable_knownfree",))[0].astype(bool)
        self.obs,self.mean,self.var,self.sem,self.nav=obs,mean,var,sem,nav

        if self.manager is None or self.room_id!=roomid:
            if self.manager:self.manager.close()
            self.manager=R7DualPlan(entry_anchor=robot,room_id=roomid)
            self.room_id=roomid
            self.entry=robot
            mission=R7Missions(obs,mean,var,sem,robot,nav,entry_anchor=robot)
            plan=mission.plan("INITIAL")
            self.manager.initialize(plan,obs,mean,sem,tick)
            self.number_plan1+=1
            self.last_phase="INITIAL"
        else:
            # At the first physical GOAL1 arrival we require a fresh native
            # LaMa/G1 epoch to test the earlier predicted-empty chamber.
            # KEEP goal2 fixed; start Plan2 from newly inferred structure.
            if (len(self.completed)==1 and self.manager.active
                and self.last_phase=="INITIAL"):
                evidence=inspect_contradiction(
                    self.manager.before_obs,self.manager.before_mean,
                    obs,self.manager.before_semantic)
                if evidence["has_contradiction"]:
                    event=self.manager.ingest_scan(
                        obs,mean,var,sem,nav,scan_tick=tick,
                        structural_epoch=self.manager.epoch+1,room_id=roomid)
                else:
                    event={"action":"KEEP_PLAN1"}
                plan=self.manager.plan1
                self.last_phase="INITIAL"
                # Plan2 executes in background; don't replace goal2.
            else:
                phase="SUCCESSOR" if self.completed else "INITIAL"
                mission=R7Missions(obs,mean,var,sem,robot,nav,
                                   entry_anchor=self.entry)
                # Completed structural viewpoints are one-off obligations.
                # Do not reissue identical goals or force them after new scans.
                pending=mission.plan(phase)
                filtered=[t for t in pending["tasks"]
                          if not any((t["goal_rc"][0]-q[0])**2+
                                     (t["goal_rc"][1]-q[1])**2<=9
                                     for q in self.completed)]
                plan=dict(pending)
                plan["tasks"]=filtered
                plan["goals"]=[t["goal_rc"] for t in filtered]
                self.manager.active=[tuple(p) for p in plan["goals"]]
                self.last_phase=phase
                if phase=="SUCCESSOR":self.number_plan2+=1
        proposed=list(self.manager.active)
        safe=self._validate_candidates(proposed,obs,nav)
        # On an unavailable structural goal, native G1 LOCAL WFD chooses
        # frontier instead; never teleport or treat predicted FREE as valid.
        accepted=safe[0] if safe else None
        self.current=accepted
        if accepted is None:
            self.manager.active=[]
        status={"event":"RESULT","native_tick":tick,"room_id":roomid,
                "decision":"GOAL" if accepted is not None else "DEFER",
                "goal_core_rc":list(accepted) if accepted is not None else None,
                "goal_sequence_core_rc":[list(p) for p in proposed],
                "plan_phase":self.last_phase,
                "camera_future_guess_only":False,
                "GT_used":False,"native_Astar_required":True,
                "structural_missions":plan.get("tasks",[]),
                "compute_s":plan.get("planning_s"),
                "known_free_check":accepted is not None,
                "async_ready":self.manager.prepared is not None}
        # avoid enormous route arrays in JSONL response/logs
        for t in status["structural_missions"]:
            t.pop("virtual_predicted_path_rc",None)
        return status

    def ack(self,r):
        if self.manager is None or self.current is None:
            return {"event":"ACK_GOAL","accepted":False,"reason":"NO_ACTIVE_GOAL"}
        position=tuple(map(int,r["robot_core_rc"]))
        if position!=self.current:
            return {"event":"ACK_GOAL","accepted":False,
                    "reason":"ARRIVAL_NOT_VERIFIED","expected":list(self.current)}
        # Native adapter checks exact arrival; manager confirms current FREE.
        if self.obs is None:raise RuntimeError("NO_OBSERVATION")
        if self.obs[position]!=0:
            raise RuntimeError("ARRIVAL_OCCUPANCY_NOT_FREE")
        if self.manager.active and self.manager.active[0]==position:
            self.manager.active.pop(0)
        self.completed.append(position)
        self.current=None
        prepared=None
        if self.manager.pending is not None and self.manager.pending.done():
            prepared=self.manager.poll()
        remaining=[list(p) for p in self.manager.active]
        self.current=tuple(remaining[0]) if remaining else None
        # A prepared Plan2 is NOT blindly activated at goal1. It can only
        # be adopted after Plan1 completion and at next fresh native epoch.
        return {"event":"ACK_GOAL","accepted":True,
                "finished_goal_rc":list(position),
                "remaining_goals_rc":remaining,
                "Plan2_prepared":prepared if prepared else None,
                "next_plan_requires_fresh_G1_LaMa":not bool(remaining)}

    def update(self,r):
        if self.manager is None or self.current is None:
            return {"event":"UPDATE","action":"NO_ACTIVE_R7"}
        tick=int(r["native_tick"])
        if tick<=self.last_tick:
            return {"event":"UPDATE","action":"STALE","native_tick":tick}
        self.last_tick=tick
        robot=tuple(map(int,r["robot_core_rc"]))
        obs=npz(r["observed_npz"],("observed",))[0]
        nav=npz(r["native_scope_npz"],("reachable_knownfree",))[0].astype(bool)
        if obs.shape!=self.obs.shape:raise ValueError("SCAN_SHAPE_CHANGED")
        # Cached semantic G1 hypothesis is previous, not current inference.
        # Occupied cells from LiDAR override historical predicted room FREE.
        updated_sem=self.sem.copy()
        updated_sem[obs==2]=4
        evidence=inspect_contradiction(self.manager.before_obs,
                                       self.manager.before_mean,
                                       obs,self.manager.before_semantic)
        self.new_obstacles=max(self.new_obstacles,evidence["unexpected_obstacle_cells"])
        self.scan_updates+=1
        decision=self.manager.ingest_scan(
            obs,self.mean,self.var,updated_sem,nav,scan_tick=tick,
            structural_epoch=self.manager.epoch+int(evidence["has_contradiction"]),
            room_id=self.room_id)
        self.obs=obs
        self.nav=nav
        return {"event":"UPDATE","native_tick":tick,
                "action":decision["action"],"contradiction":evidence,
                "plan2_status":self.manager.poll() if (self.manager.pending is not None
                                     and self.manager.pending.done()) else "WORKING_OR_NONE",
                "preserve_current_goal_rc":list(self.current),
                "native_motion_unchanged":True}

    def handle(self,r):
        a=r.get("action")
        if a=="CHOOSE":return self.choose(r)
        if a=="ACK_GOAL":return self.ack(r)
        if a=="UPDATE":return self.update(r)
        if a=="STATUS":
            return {"event":"STATUS","room_id":self.room_id,"active":[list(q) for q in
                   (self.manager.active if self.manager else [])],
                   "finished":[list(q) for q in self.completed],
                   "scan_updates":self.scan_updates,
                   "max_new_obstacle_cells":self.new_obstacles,
                   "plan1_count":self.number_plan1,
                   "plan2_count":self.number_plan2,
                   "GT_used":False}
        if a=="CLOSE":
            if self.manager:self.manager.close()
            return {"event":"CLOSED"}
        raise ValueError("UNKNOWN_ACTION "+repr(a))


def main():
    p=Policy()
    print(json.dumps({"event":"READY","policy":"G2_R7_R3_STRUCTURAL_MISSIONS",
                      "no_GT":True,"no_R4_R6_choice":True}),flush=True)
    for line in sys.stdin:
        obj={}
        try:
            obj=json.loads(line)
            result=p.handle(obj)
        except Exception as e:
            result={"event":"ERROR","error":type(e).__name__,
                    "detail":str(e),"action":obj.get("action")}
        print(json.dumps(result,allow_nan=False,ensure_ascii=False),flush=True)
        if obj.get("action")=="CLOSE":break
if __name__=="__main__":main()
