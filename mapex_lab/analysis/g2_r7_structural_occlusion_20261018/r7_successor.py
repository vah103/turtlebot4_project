"""R7 versioned Plan1/Plan2 advisory supervisor.

Computes a successor from a predicted safe future *anchor* while the active
goal is still executing. Nothing here commands the robot. A native 2D adapter
must supply inflated reachable observed FREE and re-run its own A* at dispatch.
"""
from __future__ import annotations
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import hashlib
import numpy as np
from g2_r6_watchman import _KnownFreePaths
from r7_structural_watchman import R7Planner, R7Config


def digest(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


@dataclass(frozen=True)
class Prepared:
    seq: int
    room_id: int
    structural_epoch: int
    snapshot_hash: str
    anchor: tuple
    goals: tuple
    target_gain: float
    path_guess_m: float
    source: str


def _prepare(snapshot, room_id, structural_epoch, seq, anchor, active_prefix, cfg):
    obs,mean,var,sem,legal=snapshot
    if obs[anchor]!=0 or not legal[anchor]:
        raise ValueError("FUTURE_ANCHOR_NOT_YET_OBSERVED_FREE")
    if not active_prefix or tuple(active_prefix[-1])!=anchor:
        raise ValueError("MISSING_VERIFIED_PREFIX_TO_FUTURE_ANCHOR")
    if any(obs[tuple(p)]!=0 or not legal[tuple(p)] for p in active_prefix):
        raise ValueError("SPECULATIVE_OR_UNSAFE_PREFIX")
    planner=R7Planner(obs,mean,var,sem,anchor,legal,cfg)
    seen_nom,seen_stress=planner.rays.route(active_prefix)
    seen_surface=planner.faces.along(active_prefix,planner.r7.scan_stride_cells)
    plan=planner.plan(assumed_before=(seen_nom,seen_stress,seen_surface))
    goals=tuple(map(tuple,plan["goal_sequence_core_rc"]))
    return Prepared(seq,int(room_id),int(structural_epoch),digest(obs),
                    anchor,goals,0.,float(plan["predicted_distance_m"]),
                    "R7_BACKGROUND_SUCCESSOR")


class R7Supervisor:
    """One active plan and at most one asynchronous successor proposal."""
    def __init__(self,room_id, structural_epoch, cfg=None):
        self.room_id=int(room_id)
        self.structural_epoch=int(structural_epoch)
        self.cfg=cfg or R7Config()
        self.active=[]
        self.generation=0
        self.worker=ThreadPoolExecutor(max_workers=1,thread_name_prefix="r7-next")
        self._pending=None
        self._prepared=None
        self._last_seq=-1

    def close(self):
        self.worker.shutdown(wait=True,cancel_futures=True)

    def install_initial(self,plan,seq):
        if seq<=self._last_seq:
            raise ValueError("NONMONOTONE_INITIAL_SEQ")
        self.active=list(map(tuple,plan["goal_sequence_core_rc"]))
        self._last_seq=int(seq)
        self.generation+=1
        return {"action":"ACTIVE_PLAN_ADVISORY","generation":self.generation,
                "goals":[list(p) for p in self.active],"native_astar_required":True}

    def prepare_during_motion(self, *, observed,mean,variance,semantic,native_walkable,
                               seq,room_id,structural_epoch,anchor,active_prefix):
        """Non-blocking; snapshot is copied before worker sees it."""
        if int(room_id)!=self.room_id:
            return {"action":"REJECT","reason":"ROOM_CHANGED"}
        if int(structural_epoch)<self.structural_epoch:
            return {"action":"REJECT","reason":"OLD_STRUCTURE_EPOCH"}
        if self._pending is not None and not self._pending.done():
            return {"action":"BUSY","reason":"ONLY_ONE_SUCCESSOR_IN_FLIGHT"}
        arrs=tuple(np.array(a,copy=True) for a in
                   (observed,mean,variance,semantic,native_walkable))
        self._prepared=None
        self._pending=self.worker.submit(
            _prepare,arrs,int(room_id),int(structural_epoch),int(seq),
            tuple(map(int,anchor)),list(map(tuple,active_prefix)),self.cfg)
        return {"action":"BACKGROUND_PREPARATION_STARTED",
                "source_seq":int(seq),"future_anchor":list(anchor),
                "active_plan_preserved":True,"robot_command":None}

    def poll(self):
        if self._pending is None:return {"action":"NO_PREPARATION"}
        if not self._pending.done():return {"action":"BACKGROUND_RUNNING"}
        try:
            self._prepared=self._pending.result()
        except Exception as exc:
            self._pending=None
            return {"action":"BACKGROUND_FAILED","reason":str(exc)}
        self._pending=None
        return {"action":"SUCCESSOR_READY_ADVISORY",
                "source_seq":self._prepared.seq,
                "anchor":list(self._prepared.anchor),
                "goals":[list(p) for p in self._prepared.goals],
                "requires_arrival_checkpoint":True,
                "native_astar_required":True,"robot_command":None}

    def checkpoint(self, *, observed,mean,variance,semantic,native_walkable,
                   pose,seq,room_id,structural_epoch,
                   local_frontiers_exist=True,active_route_safe=True):
        """Promote only at verified arrival and a fresh scan; never dispatch."""
        pose=tuple(map(int,pose))
        if not active_route_safe:
            return {"action":"SAFE_HOLD","reason":"CURRENT_ROUTE_UNSAFE"}
        if int(room_id)!=self.room_id:
            return {"action":"WAIT_FRESH_ROOM_PLAN","reason":"ROOM_CHANGED"}
        if not local_frontiers_exist:
            return {"action":"G1_LOCAL_WFD_EXIT_GATE","native_gate_required":True}
        if self._prepared is None:
            return {"action":"WAIT_SUCCESSOR","reason":"BACKGROUND_NOT_READY"}
        p=self._prepared
        if p.room_id!=int(room_id) or p.structural_epoch!=int(structural_epoch):
            self._prepared=None
            return {"action":"REJECT_AND_REPLAN","reason":"STRUCTURAL_EPOCH_CHANGED"}
        if pose!=p.anchor or int(seq)<p.seq:
            return {"action":"WAIT_ARRIVAL","anchor":list(p.anchor)}
        obs=np.asarray(observed)
        legal=np.asarray(native_walkable,bool)
        if obs[pose]!=0 or not legal[pose]:
            return {"action":"SAFE_HOLD","reason":"ROBOT_NOT_NATIVE_FREE"}
        # New scan may reveal that a prospective goal/leg is blocked. Re-check
        # paths; never promote arbitrary speculative predicted FREE cells.
        try:
            planner=R7Planner(obs,mean,variance,semantic,pose,legal,self.cfg)
            valid=[]
            cur=pose
            for goal in p.goals:
                if not planner.legal[goal] or planner.paths.path(cur,goal) is None:
                    return {"action":"REJECT_AND_REPLAN","reason":"PROPOSED_ROUTE_NOT_NATIVE_FREE"}
                valid.append(goal)
                cur=goal
            # Prune visits that no longer contribute UNKNOWN or occluder faces,
            # using the actual current scan, not future knowledge.
            while valid:
                full=planner._score_route(valid)
                short=planner._score_route(valid[:-1])
                if full is None or short is None:
                    return {"action":"REJECT_AND_REPLAN","reason":"ROUTE_LOST"}
                if ((full["nominal"] & ~short["nominal"]).sum()==0 and
                    (full["stress"] & ~short["stress"]).sum()==0 and
                    (full["surfaces"] & ~short["surfaces"])==0):
                    valid.pop()
                else:break
        except ValueError as exc:
            return {"action":"REJECT_AND_REPLAN","reason":str(exc)}
        self._prepared=None
        if not valid:
            return {"action":"RESIDUAL_ROOM_WFD",
                    "reason":"PREPARED_GOALS_REDUNDANT_AFTER_REAL_SCAN",
                    "native_roomnode_wfd_fallback":True}
        self.active=valid
        self.generation+=1
        self.structural_epoch=int(structural_epoch)
        self._last_seq=int(seq)
        return {"action":"SUCCESSOR_PROMOTED_ADVISORY",
                "goals":[list(x) for x in valid],
                "generation":self.generation,
                "computed_while_previous_goal_executed":True,
                "must_call_native_astar_before_dispatch":True,
                "robot_command":None}
