"""G2-R7-R3.1: frozen-chamber structural planning JSONL service.

No R4/R6 choice logic, GT, future maps, or robot motion commands.
PLAN1 at ROOM entry. PLAN2 generated once from observed unexpected
occluders in stable non-entry chamber, and retained while finishing PLAN1.
The native A* adapter validates every selected goal and path.
"""
import json,sys
import numpy as np
from r7_stable_hypothesis import StableHypothesis


def from_npz(file,columns):
    with np.load(file,allow_pickle=False) as z:
        return tuple(z[k].copy() for k in columns)


class Policy:
    def __init__(self):
        self.roomid=None
        self.struct=None
        self.active=[]
        self.plan2=[]
        self.plan2_witness=None
        self.goal=None
        self.visited=[]
        self.stage="NONE"
        self.latest_tick=-1
        self.latest_obs=None
        self.latest_nav=None
        self.replans=0
        self.updates=0
        self.armed=False

    def _validate(self,goals,obs,nav):
        result=[]
        for p in goals:
            y,x=map(int,p)
            if 0<=y<obs.shape[0] and 0<=x<obs.shape[1] and obs[y,x]==0 and nav[y,x]:
                result.append((y,x))
            else:
                break
        return result

    def _evaluate(self,obs,nav,tick):
        report=self.struct.update(obs,nav,tick)
        # Only structural objects which are confirmed by current LiDAR can
        # induce Plan2. Tentative LaMa segmentation changes alone cannot.
        # Plan2 is a response to what the robot actually sees at GOAL1.
        # Prematurely freezing order during the approach to GOAL1 can
        # prioritize the wrong chamber half from sparse first rays.
        # Until the native ACK of Goal1: update evidence provisionally only.
        if (self.visited and self.stage=="PLAN1" and
            not self.plan2 and not self.armed and report["tasks"]):
            self.plan2=[tuple(t["goal_rc"]) for t in report["tasks"]]
            self.plan2_witness={
                "detected_native_tick":int(tick),
                "version":report["revision_version"],
                "confirmed_obstacles":report["confirmed_unexpected_obstacle_cells"],
                "obligations":report["tasks"]}
            self.armed=True
        self.latest_obs=obs.copy()
        self.latest_nav=nav.copy()
        return report

    def choose(self,r):
        tick=int(r["native_tick"])
        if tick<self.latest_tick:raise ValueError("STALE_NATIVE_TICK")
        roomid=int(r["room_id"])
        robot=tuple(map(int,r["robot_core_rc"]))
        obs,mean,var=from_npz(r["room_node_input"],("observed","mean","variance"))
        sem=from_npz(r["room_node_semantic"],("semantic",))[0]
        nav=from_npz(r["native_scope_npz"],("reachable_knownfree",))[0].astype(bool)
        self.latest_tick=tick
        new_room=(roomid!=self.roomid or self.struct is None)
        if new_room:
            self.struct=StableHypothesis(obs,mean,var,sem,robot,nav)
            self.roomid=roomid
            self.active=[tuple(map(int,g)) for g in self.struct.plan1["goals"]]
            self.plan2=[]
            self.plan2_witness=None
            self.goal=None
            self.stage="PLAN1"
            self.visited=[]
            self.armed=False
        evidence=self._evaluate(obs,nav,tick)
        # The INITIAL plan stays committed through both goals, and only
        # an A* failure can veto it. The structure can evolve at the next
        # epoch without altering its entry chamber identity.
        if not self.active and self.stage=="PLAN1":
            self.stage="PLAN2"
            self.active=[tuple(g) for g in self.plan2 if tuple(g) not in self.visited]
        elif not self.active and self.stage=="PLAN2":
            # Plan2 exhausted; let G1 WFD finish residual unknown.
            self.stage="RESIDUAL_WFD"
        wanted=[list(g) for g in self.active]
        safe=self._validate(self.active,obs,nav)
        selected=safe[0] if safe else None
        self.goal=selected
        self.replans+=1
        result={
            "event":"RESULT","native_tick":tick,"room_id":roomid,
            "decision":"GOAL" if selected else "DEFER",
            "goal_core_rc":list(selected) if selected else None,
            "goal_sequence_core_rc":wanted,
            "plan_phase":self.stage,
            "native_astar_required":True,
            "GT_used":False,
            "structural_stability":{
                "frozen_zone_sha256":self.struct.fingerprint,
                "frozen_portal":list(self.struct.portal) if self.struct.portal else None,
                "semantic_zone_relabeled":False,
                "revision":evidence["revision_version"],
                "new_obstacle_cells":evidence["confirmed_unexpected_obstacle_cells"]},
            "plan2_witness":self.plan2_witness,
            "sidecar_native_motion_authorized":False,
            "real_native_observed_free_for_goal":selected is not None}
        return result

    def ack(self,r):
        if self.goal is None:
            return {"event":"ACK_GOAL","accepted":False,"reason":"NO_GOAL"}
        pose=tuple(map(int,r["robot_core_rc"]))
        if pose!=self.goal:
            return {"event":"ACK_GOAL","accepted":False,
                    "reason":"NOT_EXACT_ARRIVAL","expected":list(self.goal)}
        if self.latest_obs[pose]!=0:
            raise RuntimeError("GOAL_OCCUPANCY_NOT_OBSERVED_FREE")
        if self.active and self.active[0]==pose:
            self.active.pop(0)
        self.visited.append(pose)
        self.goal=tuple(self.active[0]) if self.active else None
        return {"event":"ACK_GOAL","accepted":True,
            "finished_goal_rc":list(pose),
            "remaining_goals_rc":[list(g) for g in self.active],
            "plan_phase":self.stage,
            "PLAN1_completed":self.stage=="PLAN1" and not self.active,
            "plan2_prepared":[list(p) for p in self.plan2]}

    def update(self,r):
        if self.struct is None:
            return {"event":"UPDATE","action":"NO_ROOM"}
        tick=int(r["native_tick"])
        if tick<=self.latest_tick:return {"event":"UPDATE","action":"STALE_SCAN","native_tick":tick}
        self.latest_tick=tick
        obs=from_npz(r["observed_npz"],("observed",))[0]
        nav=from_npz(r["native_scope_npz"],("reachable_knownfree",))[0].astype(bool)
        report=self._evaluate(obs,nav,tick)
        self.updates+=1
        return {"event":"UPDATE","native_tick":tick,
           "action":"STABLE_STRUCTURE_REVISION" if report["objects_in_other_chambers"] else "KEEP_FROZEN_STRUCTURE",
           "revision":report["revision_version"],
           "new_obstacle_cells":report["confirmed_unexpected_obstacle_cells"],
           "plan2_prepared":[list(g) for g in self.plan2],
           "active_goal_preserved":[list(g) for g in self.active],
           "stable_zone_sha256":self.struct.fingerprint,
           "GT_used":False}

    def status(self):
        return {"event":"STATUS","room_id":self.roomid,"stage":self.stage,
                "remaining_goals":[list(g) for g in self.active],
                "plan2_prepared":[list(g) for g in self.plan2],
                "visited":[list(g) for g in self.visited],
                "revision":self.struct.version if self.struct else None,
                "new_obstacles":self.struct._last_blocker_count if self.struct else None,
                "frozen_portal":list(self.struct.portal) if self.struct and self.struct.portal else None,
                "snapshots_queued":self.updates,
                "GT_used":False}

    def handle(self,r):
        action=r.get("action")
        if action=="CHOOSE":return self.choose(r)
        if action=="UPDATE":return self.update(r)
        if action=="ACK_GOAL":return self.ack(r)
        if action=="STATUS":return self.status()
        if action=="CLOSE":return {"event":"CLOSED"}
        raise ValueError("UNKNOWN_ACTION")

def main():
    p=Policy()
    print(json.dumps({"event":"READY","policy":"G2_R7_R31_STABLE_CHAMBERS",
                       "no_gt":True,"sidecar_requires_native_astar":True}),flush=True)
    for line in sys.stdin:
        req={}
        try:
            req=json.loads(line)
            ans=p.handle(req)
        except Exception as e:
            ans={"event":"ERROR","action":req.get("action"),
                 "error":type(e).__name__,"detail":str(e)}
        print(json.dumps(ans,ensure_ascii=False,allow_nan=False),flush=True)
        if req.get("action")=="CLOSE":break

if __name__=="__main__":main()
