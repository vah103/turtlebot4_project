"""G2-R7-R3 Plan1 execution / asynchronous Plan2 advisory supervisor.

No robot actuator calls. New structural contradictions are detected only from
actual *earlier vs present* observed LiDAR, not GT or any future observations.
Predictive routes remain INTENT until revalidated by native inflated A*.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from r7_missions import R7Missions,Params


def inspect_contradiction(before_obs,before_mean,present_obs,before_semantic):
    """Evidence is unexpected OCCUPIED cell where prior LaMa believed FREE."""
    if before_obs.shape != present_obs.shape:
        raise ValueError("SNAPSHOT_SHAPE_CHANGED")
    changed=((before_obs==1)&(before_mean<.5)&
             (present_obs==2)&(before_semantic==1))
    return dict(unexpected_obstacle_cells=int(changed.sum()),
                has_contradiction=bool(changed.any()))


def _prepare(inputs,anchor,entry_anchor,params):
    obs,mean,var,sem,native=inputs
    solver=R7Missions(obs,mean,var,sem,anchor,native,
                      cfg=params,entry_anchor=entry_anchor,virtual_anchor=True)
    return solver.plan("SUCCESSOR")


class R7DualPlan:
    """At most one active plan and one future speculative plan."""

    def __init__(self,entry_anchor,room_id,params=None):
        self.entry_anchor=tuple(map(int,entry_anchor))
        self.room_id=int(room_id)
        self.params=params or Params()
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix="r7-plan2")
        self.generation=0
        self.active=[]
        self.pending=None
        self.prepared=None
        self.plan1=None
        self.before_obs=None
        self.before_mean=None
        self.before_semantic=None
        self.epoch=0
        self.last_scan=-1

    def initialize(self,plan1,obs,mean,semantic,scan_tick,structural_epoch=0):
        if self.generation!=0:raise RuntimeError("Plan1 already installed")
        if plan1.get("phase")!="INITIAL":
            raise ValueError("invalid initial phase")
        self.plan1=plan1
        self.active=[tuple(v) for v in plan1["goals"]]
        self.before_obs=np.array(obs,copy=True)
        self.before_mean=np.array(mean,copy=True)
        self.before_semantic=np.array(semantic,copy=True)
        self.last_scan=int(scan_tick)
        self.epoch=int(structural_epoch)
        self.generation=1
        return {"action":"PLAN1_COMMITTED_INTENT","goals":plan1["goals"],
                "robot_command":None}

    def advance_on_arrival(self,robot,observed,native_mask):
        if not self.active or tuple(robot)!=self.active[0]:
            return {"action":"KEEP_ACTIVE_GOAL"}
        if observed[tuple(robot)]!=0 or not native_mask[tuple(robot)]:
            return {"action":"STOP_UNSAFE_GOAL_REACHED_INVALID"}
        reached=self.active.pop(0)
        return {"action":"GOAL_ARRIVAL_CONFIRMED",
                "finished":list(reached),
                "next_active":list(self.active[0]) if self.active else None,
                "robot_command":None}

    def ingest_scan(self,observed,mean,variance,semantic,native_mask,
                    scan_tick,structural_epoch,room_id):
        if room_id!=self.room_id:return {"action":"RESTART_ROOM","reason":"ROOM_CHANGED"}
        if int(scan_tick)<=self.last_scan:return {"action":"IGNORE_STALE_SCAN"}
        if self.before_obs is None:raise RuntimeError("not initialized")
        evidence=inspect_contradiction(self.before_obs,self.before_mean,
                                       observed,self.before_semantic)
        self.last_scan=int(scan_tick)
        if int(structural_epoch)<self.epoch:return {"action":"IGNORE_OLD_STRUCTURE"}
        self.epoch=int(structural_epoch)
        if not evidence["has_contradiction"]:
            return {"action":"KEEP_PLAN1","evidence":evidence}
        # Need actual evidence of newly discovered interior object geometry,
        # not isolated noisy occupancy along an external wall.
        temporary=R7Missions(observed,mean,variance,semantic,self.entry_anchor,
                             native_mask,cfg=self.params,entry_anchor=self.entry_anchor)
        new_rooms=[z for z in temporary.zoneids if z!=temporary.entry_zone]
        interior_revealed=any(o["zone"] in new_rooms for o in temporary.objects)
        if not interior_revealed:
            return {"action":"KEEP_PLAN1","reason":"NO_INTERIOR_STRUCTURAL_CHANGE",
                    "evidence":evidence}
        if self.pending is not None and not self.pending.done():
            return {"action":"PLAN2_ALREADY_COMPUTING","evidence":evidence}
        if len(self.active)==0:
            return {"action":"NO_ACTIVE_GOAL_TO_PRESERVE","evidence":evidence}
        # Anchor future plan at end of committed active work. No speculative
        # movement is issued to that anchor by this planner.
        anchor=self.active[-1]
        arrays=tuple(np.array(x,copy=True) for x in
                     (observed,mean,variance,semantic,native_mask))
        self.pending=self.pool.submit(_prepare,arrays,anchor,
                                      self.entry_anchor,self.params)
        self.prepared=None
        return {"action":"PLAN2_PREPARATION_STARTED_IN_BACKGROUND",
                "preserve_active_goals":[list(p) for p in self.active],
                "future_anchor":list(anchor),
                "new_unexpected_obstacle_cells":evidence["unexpected_obstacle_cells"],
                "native_motion_unchanged":True,"robot_command":None}

    def poll(self):
        if self.pending is None:return {"action":"NO_PENDING_PLAN2"}
        if not self.pending.done():return {"action":"PLAN2_WORKING"}
        try:
            self.prepared=self.pending.result()
            self.pending=None
            return {"action":"PLAN2_READY",
                    "goals":self.prepared["goals"],
                    "mission_phase":"SUCCESSOR",
                    "robot_command":None}
        except Exception as exc:
            self.pending=None
            self.prepared=None
            return {"action":"PLAN2_FAILED","reason":str(exc)}

    def checkpoint(self,robot,obs,mean,var,sem,native_mask,
                   scan_tick,structural_epoch,room_id,route_safe,local_wfd_exists):
        if not route_safe:
            return {"action":"STOP_AND_NATIVE_REPLAN","reason":"UNSAFE_ROUTE"}
        if room_id!=self.room_id:
            return {"action":"ROOM_CHANGED_NATIVE_HANDOFF"}
        if not local_wfd_exists:
            return {"action":"G1_LOCAL_WFD_EXIT_GATE","native_validation_required":True}
        if self.active:
            return {"action":"EXECUTE_COMMITTED_PLAN1",
                    "next_intent":list(self.active[0]),
                    "native_Astar_required":True}
        if self.prepared is None:
            return {"action":"WAIT_OR_G1_WFD_FALLBACK","reason":"PLAN2_NOT_READY"}
        if int(structural_epoch)<self.epoch:
            return {"action":"REFRESH_PLAN2","reason":"STALE_STRUCTURE"}
        current=tuple(map(int,robot))
        if obs[current]!=0 or not native_mask[current]:
            return {"action":"STOP_AND_NATIVE_REPLAN","reason":"ROBOT_NOT_VERIFIED_FREE"}
        # Re-check successor obligations using latest scan; do not blindly
        # commit stale predicted goals after traveling to the anchor.
        solver=R7Missions(obs,mean,var,sem,current,native_mask,
                          cfg=self.params,entry_anchor=self.entry_anchor)
        new=solver.plan("SUCCESSOR")
        self.prepared=None
        self.generation+=1
        self.active=[tuple(v) for v in new["goals"]]
        if not self.active:
            return {"action":"G1_WFD_LOCAL_COMPLETION_FALLBACK",
                    "reason":"NO_REMAINING_STRUCTURAL_VIEWPOINTS"}
        return {"action":"PLAN2_INTENT_PROMOTED",
                "goals":new["goals"],
                "native_Astar_required_for_every_leg":True,
                "predicted_FREE_not_motion_authority":True,
                "robot_command":None}

    def close(self):
        self.pool.shutdown(wait=True,cancel_futures=True)
