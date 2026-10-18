"""G2-R6.1 JSONL causal structural sidecar, for isolated native adapter.

The sidecar NEVER has direct robot control. The native layer validates
actual observed-FREE inflated A* before dispatching a goal.
"""
import json
import sys
from pathlib import Path
import numpy as np

from g2_r6_sidecar import Service as BaseService, get_map
from r61_plan_manager import R61PlanManager
from r61_structural_events import S0,S1,S2,S3,S4

class R61Service:
    def __init__(self):
        self.base=BaseService()
        self.manager=None
        self.pending_heavy=False
        self.scan_seq=-1
        self.latest_scans=0

    @staticmethod
    def _scan(req):
        observed=get_map(req["observed_npz"],("observed",))[0]
        known=get_map(req["native_scope_npz"],("reachable_knownfree",))[0].astype(bool)
        return observed,known

    def choose(self,req):
        # CHOOSE is authorized ONLY after the calling native loop has performed
        # its real same-epoch LaMa + Room Node + LOCAL WFD selection.
        # Never recycle old R6's semantic/raycast at a structural rebuild.
        self.base.active=None
        self.base.room_id=None
        reply=self.base.choose(req)
        if self.base.active is not None:
            self.manager=R61PlanManager(self.base.active,int(req["room_id"]),
                                        int(req["native_tick"]))
        else:
            self.manager=None
        self.pending_heavy=False
        self.scan_seq=int(req["native_tick"])
        reply.update({"policy":"G2_R62_GENERAL_ROOM",
            "structure_epoch_native_tick":int(req["native_tick"]),
            "mandatory_post_arrival_checkpoint":True,
            "source_coherent_LaMa_RoomNode":True})
        return reply

    def update(self,req):
        if self.manager is None:
            return {"event":"UPDATE","fast":{"action":"REPLAN_REQUIRED",
                    "reason":"NO_ACTIVE_R61_PLANNER"}}
        seq=int(req["native_tick"])
        obs,known=self._scan(req)
        pose=tuple(map(int,req["robot_core_rc"]))
        ev=self.manager.on_scan(obs,pose,seq,known,
            active_route_safe=bool(req.get("active_route_safe",True)),
            local_frontiers_exist=bool(req.get("local_frontiers_exist",True)),
            official_room_valid=bool(req.get("official_room_valid",True)))
        self.scan_seq=max(self.scan_seq,seq)
        if ev.get("accepted"):
            self.latest_scans+=1
        heavy=ev["event"] in (S0,S3,S4) or self.manager.pending_full
        self.pending_heavy=self.pending_heavy or heavy
        light=None
        if ev["event"]==S2 and not self.pending_heavy:
            light=self.manager.propose_light()
        return {"event":"UPDATE","native_tick":seq,
                "fast":{"action":"REPLAN_REQUIRED" if self.pending_heavy else "KEEP",
                        "reason":ev["reason"],"event_level":ev["event"]},
                "structural":ev,"light_advisory":light,
                "need_fresh_coherent_LaMa_RoomNode":self.pending_heavy,
                "LaMa_rerun_by_sidecar":False,
                "RoomNode_rerun_by_sidecar":False}

    def checkpoint(self,req):
        if self.manager is None:
            return {"event":"CHECKPOINT","action":"WAIT_STRUCTURAL_REFRESH",
                    "reason":"NO_ACTIVE_ROOM"}
        seq=int(req["native_tick"])
        obs,known=self._scan(req)
        pose=tuple(map(int,req["robot_core_rc"]))
        current_goal=self.manager.active_goals[0] if self.manager.active_goals else None
        verdict=self.manager.checkpoint(
            obs,pose,seq,known,
            source_goal=current_goal,
            native_path_valid=bool(req.get("native_path_valid",True)),
            local_frontiers_exist=bool(req.get("local_frontiers_exist",True)),
            official_room_valid=bool(req.get("official_room_valid",True)))
        if verdict["action"]=="NEXT_GOAL":
            # Synchronize R6 state with the newly *reviewed* R61 chain.
            self.base.active.plan_order=list(self.manager.active_goals)
        if verdict["action"] in ("SAFE_HOLD","WAIT_STRUCTURAL_REFRESH",
                                 "EXIT_OR_REFRESH_ROOMNODE","RESIDUAL_ROOM_WFD"):
            self.pending_heavy=True
        return {"event":"CHECKPOINT","native_tick":seq,
                "source_observation_seq":self.manager.monitor.obs_seq,
                "must_revalidate_native_ASTAR":True,
                "need_fresh_coherent_LaMa_RoomNode":self.pending_heavy,
                **verdict}

    def status(self):
        return {"event":"STATUS","policy":"G2_R62_GENERAL_ROOM",
            "room_id":self.manager.room_id if self.manager else None,
            "latest_scan_seq":self.scan_seq,
            "scan_updates":self.latest_scans,
            "pending_heavy":self.pending_heavy,
            "pending_goals": [list(g) for g in self.manager.active_goals] if self.manager else [],
            "last_structural_event":self.manager.last_event if self.manager else None,
            "no_GT":True}

    def handle(self,req):
        action=req.get("action")
        if action=="CHOOSE":return self.choose(req)
        if action=="UPDATE":return self.update(req)
        if action=="CHECKPOINT":return self.checkpoint(req)
        if action=="STATUS":return self.status()
        if action=="CLOSE":return {"event":"CLOSED"}
        raise ValueError("unsupported R61 action")

def main():
    server=R61Service()
    print(json.dumps({"event":"READY","policy":"G2_R62_GENERAL_ROOM",
                       "protocol":["CHOOSE","UPDATE","CHECKPOINT","STATUS","CLOSE"],
                       "native_ASTAR_required":True,"no_gt":True}),flush=True)
    for line in sys.stdin:
        request={}
        try:
            request=json.loads(line)
            result=server.handle(request)
        except Exception as e:
            result={"event":"ERROR","type":type(e).__name__,"reason":str(e)}
        print(json.dumps(result,ensure_ascii=False,allow_nan=False),flush=True)
        if request.get("action")=="CLOSE":
            break

if __name__=="__main__":
    main()