"""G2-R7 R2 asynchronous Plan1/Plan2 supervisor (research advisory).

Only incoming same-time observation/LaMa/semantic data; no GT and no
future LiDAR. The current active leg remains committed unless native
safety or mission validity fails. Background proposed routes are never
robot commands and must be revalidated by native inflated known-FREE A*.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import numpy as np
from r7_standalone import R7Standalone, checksum


@dataclass
class Pending:
    room_id: int
    structural_epoch: int
    source_tick: int
    source_obs_hash: str
    anchor: tuple
    source_plan_generation: int
    advisory: dict


def _solve_at_anchor(arrays,anchor,source_tick,room_id,structural_epoch,generation,settings):
    obs,mean,var,sem,native=arrays
    # The anchor is the destination of a still-active leg, and may still be
    # UNKNOWN when the background planner runs. Virtual-anchor planning is
    # not authorization to travel through UNKNOWN.
    solver=R7Standalone(obs,mean,var,sem,anchor,native,settings,
                        virtual_anchor=True)
    proposal=solver.plan()
    return Pending(int(room_id),int(structural_epoch),int(source_tick),
                   checksum(obs),tuple(anchor),int(generation),proposal)


class R7DualPlanManager:
    def __init__(self,room_id,settings=None):
        self.room_id=int(room_id)
        self.settings=settings
        self.active=[]
        self.generation=0
        self.plan1=None
        self.prepared=None
        self.executor=ThreadPoolExecutor(max_workers=1,thread_name_prefix="r7-plan2")
        self.task=None
        self.current_epoch=0
        self.last_tick=-1

    def install_initial(self,initial,scan_tick,structural_epoch):
        if self.generation!=0:raise RuntimeError("initial plan already installed")
        if initial.get("status")!="STRUCTURAL_PLAN_INTENT_NOT_ROBOT_COMMAND":
            raise ValueError("invalid initial plan")
        self.active=[tuple(g) for g in initial["goals_rc"]]
        self.plan1=initial
        self.generation=1
        self.current_epoch=int(structural_epoch)
        self.last_tick=int(scan_tick)
        return {"action":"PLAN1_INTENT_INSTALLED","goals":initial["goals_rc"],
                "native_Astar_required":True,"robot_command":None}

    def complete_active_goal(self, *, robot, observed, native_walkable):
        """A goal is consumed ONLY if the robot physically reached it."""
        if not self.active or tuple(robot)!=self.active[0]:
            return {"action":"WAIT_GOAL_ARRIVAL"}
        if np.asarray(observed)[tuple(robot)]!=0 or not np.asarray(
            native_walkable,bool)[tuple(robot)]:
            return {"action":"SAFE_HOLD","reason":"INVALID_ARRIVAL"}
        finished=self.active.pop(0)
        return {"action":"GOAL_CONSUMED","goal":list(finished),
                "remaining_goals":[list(p) for p in self.active],
                "robot_command":None}

    def propose_successor_async(self,observed,mean,variance,semantic,native_walkable,
                                *,scan_tick,structural_epoch,room_id,active_goal,
                                structural_contradiction):
        if int(room_id)!=self.room_id:
            return {"action":"REFUSE","reason":"WRONG_ROOM"}
        if not structural_contradiction:
            return {"action":"KEEP_PLAN1","reason":"NO_STRUCTURAL_EVIDENCE"}
        if not self.active or tuple(active_goal)!=self.active[0]:
            return {"action":"REFUSE","reason":"NO_MATCHING_ACTIVE_GOAL"}
        if self.task is not None and not self.task.done():
            return {"action":"BUSY","reason":"PLAN2_ALREADY_COMPUTING"}
        if scan_tick<=self.last_tick:
            return {"action":"REFUSE","reason":"STALE_SCAN"}
        if structural_epoch<=self.current_epoch:
            return {"action":"REFUSE","reason":"STALE_STRUCTURAL_EPOCH"}
        self.current_epoch=int(structural_epoch)
        self.last_tick=int(scan_tick)
        source=tuple(np.array(x,copy=True) for x in
                   (observed,mean,variance,semantic,native_walkable))
        self.prepared=None
        self.task=self.executor.submit(_solve_at_anchor,source,tuple(active_goal),
                      scan_tick,room_id,structural_epoch,self.generation,
                      self.settings)
        return {"action":"PLAN2_BACKGROUND_STARTED",
                "current_active_goal_preserved":list(active_goal),
                "source_scan_tick":scan_tick,
                "robot_command":None}

    def poll(self):
        if self.task is None:return {"action":"NO_PLAN2"}
        if not self.task.done():return {"action":"PLAN2_COMPUTING"}
        try:
            prepared=self.task.result()
            self.prepared=prepared
            self.task=None
            return {"action":"PLAN2_READY_ADVISORY",
                    "anchor":list(prepared.anchor),
                    "goals":prepared.advisory.get("goals_rc",[]),
                    "source_scan_tick":prepared.source_tick,
                    "native_Astar_required":True,
                    "robot_command":None}
        except Exception as exc:
            self.task=None;self.prepared=None
            return {"action":"PLAN2_FAILED","reason":repr(exc)}

    def safety_checkpoint(self,*,observed,mean,variance,semantic,native_walkable,
                          robot,scan_tick,structural_epoch,room_id,active_route_safe,
                          native_local_wfd_exists):
        robot=tuple(robot)
        if not active_route_safe:
            return {"action":"SAFE_HOLD","reason":"ACTIVE_LEG_UNSAFE"}
        if room_id!=self.room_id:
            return {"action":"REBUILD_ROOM","reason":"ROOM_ID_CHANGED"}
        if not native_local_wfd_exists:
            return {"action":"G1_WFD_EMPTY_EXIT_GATE",
                    "requires_native_confirmation":True}
        p=self.prepared
        if p is None:return {"action":"CONTINUE_PLAN1_OR_WAIT"}
        if robot!=p.anchor:
            return {"action":"CONTINUE_PLAN1","anchor":list(p.anchor)}
        if scan_tick<p.source_tick or p.structural_epoch!=int(structural_epoch):
            self.prepared=None
            return {"action":"REBUILD_PLAN2","reason":"PLAN2_STALE_EPOCH"}
        if p.source_plan_generation!=self.generation:
            self.prepared=None
            return {"action":"REBUILD_PLAN2","reason":"ACTIVE_PLAN_ALREADY_CHANGED"}
        # Arrival and current-time validation; speculative predictions
        # remain non-commandable even after plan promotion.
        if np.asarray(observed)[robot]!=0 or not np.asarray(native_walkable,bool)[robot]:
            return {"action":"SAFE_HOLD","reason":"ROBOT_NOT_NATIVE_OBSERVED_FREE"}
        goals=p.advisory.get("goals_rc",[])
        if not goals:
            self.prepared=None
            return {"action":"G1_LOCAL_WFD_FALLBACK","reason":"NO_PLAN2_GOALS"}
        solver=R7Standalone(observed,mean,variance,semantic,robot,
                            native_walkable,self.settings)
        at=robot
        for g in goals:
            target=tuple(g)
            if not solver.geo.predfree[target] or solver.paths.path(at,target) is None:
                self.prepared=None
                return {"action":"REBUILD_PLAN2","reason":"HYPOTHETICAL_ROUTE_INVALID"}
            at=target
        self.active=[tuple(g) for g in goals]
        self.generation+=1
        self.prepared=None
        return {"action":"PLAN2_INTENT_PROMOTED",
                "goals":goals,
                "native_Astar_required_each_leg":True,
                "observed_free_only_execution":True,
                "generation":self.generation,
                "robot_command":None}

    def close(self):
        self.executor.shutdown(wait=True,cancel_futures=True)
