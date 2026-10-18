"""G2-R6.1: versioned causal manager for post-scan Watchman replanning.

This service is advisory. Native observed-FREE inflated A* remains the
execution authority; the manager can only propose/withhold future goals.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import time
import numpy as np

from g2_r6_watchman import RoomWatchman, _KnownFreePaths
from r61_structural_events import StructuralEventMonitor, digest, S0,S1,S2,S3,S4

@dataclass
class RouteProposal:
    obs_seq:int
    event_no:int
    room_id:int
    pose:tuple
    observed_hash:str
    goals:tuple
    distance_m:float
    solve_seconds:float

class R61PlanManager:
    def __init__(self, r6_planner:RoomWatchman, room_id:int,
                 init_seq:int, initial_goal_id:int=0):
        self.r6=r6_planner
        self.room_id=int(room_id)
        self.monitor=StructuralEventMonitor(self.r6.obs,self.r6.mean,
                       self.r6.var,self.r6.room,self.r6.robot,init_seq)
        self.active_goals=[tuple(g) for g in r6_planner.plan_order]
        self.consumed=[]
        self.next_goal_id=initial_goal_id
        self.plan_gen=1
        self.cached_proposal=None
        self.dirty=False
        self.pending_full=False
        self.last_event=None
        self.last_scan_native=None
        self.arrival_events=[]
        self.last_native_mask=self.r6.legal.copy()
        # Causal evidence for useful progress since the previous waypoint.
        # Works regardless of room size, shape or occupancy resolution.
        self.last_arrival_observed=self.r6.obs.copy()

    def on_scan(self, obs,pose,seq,native_walkable, *,
                active_route_safe=True,local_frontiers_exist=True,
                official_room_valid=True, future_goal_gain_drop=0.):
        pose=tuple(map(int,pose))
        obs=np.asarray(obs,dtype=np.uint8)
        native=np.asarray(native_walkable,bool)
        if native.shape != self.r6.obs.shape:
            raise ValueError("R61 invalid native scope")
        if self.active_goals:
            active=self.active_goals[0]
            if not native[active] or obs[active]!=0:
                active_route_safe=False
        ev=self.monitor.refresh(obs,pose,seq,active_route_safe=active_route_safe,
                  local_frontiers_exist=local_frontiers_exist,
                  official_room_valid=official_room_valid,
                  future_goal_gain_drop=future_goal_gain_drop)
        if not ev["accepted"]:
            return ev
        self.last_event=ev
        self.last_native_mask=native.copy()
        self.last_scan_native=int(seq)
        self.cached_proposal=None
        if ev["event"] in (S0,S3,S4):
            self.pending_full=True
        elif ev["event"]==S2:
            self.dirty=True
        # A newly observed map may invalidate R6's raycast cache. A fresh
        # lightweight candidate planner will be constructed at checkpoint.
        return ev

    def propose_light(self):
        if self.pending_full or self.monitor.pending_level in (S0,S3,S4):
            return {"action":"WAIT_STRUCTURAL_REFRESH"}
        start=time.perf_counter()
        obs=self.monitor.observed
        pose=self.monitor.pose
        # The old Room Node is only a TENTATIVE LOCAL boundary here. This does
        # not falsely claim that Room Node was re-inferred on this observation.
        scope=np.asarray(self.last_native_mask,bool)&(obs==0)&self.r6.room
        if not scope[pose]:
            self.pending_full=True
            return {"action":"WAIT_NATIVE_SAFETY"}
        try:
            candidate=RoomWatchman(obs,self.r6.mean,self.r6.var,
                                   self.r6.semantic,pose,scope,self.r6.cfg)
            report=candidate.plan()
        except (ValueError,RuntimeError) as exc:
            return {"action":"DEFER_LIGHT_ROUTE","reason":str(exc)[:180]}
        seq=self.monitor.obs_seq
        self.cached_proposal=RouteProposal(
            obs_seq=seq,event_no=self.monitor.event_no,room_id=self.room_id,
            pose=pose,observed_hash=digest(obs),
            goals=tuple(tuple(g) for g in report["goal_sequence_core_rc"]),
            distance_m=float(report["predicted_route_distance_m"]),
            solve_seconds=time.perf_counter()-start)
        return {"action":"LIGHT_PROPOSAL",
                "source_obs_seq":seq,"goal_sequence_rc":[list(g) for g in self.cached_proposal.goals],
                "distance_m":round(self.cached_proposal.distance_m,3),
                "compute_s":round(self.cached_proposal.solve_seconds,4),
                "RoomNode_rerun":False,"LaMa_rerun":False,
                "still_needs_native_ASTAR":True}

    def try_commit(self, proposal:RouteProposal,latest_observed,pose,native_walkable,
                   latest_seq:int, *, native_path_valid=True):
        """No unverified proposal is an executable robot command."""
        if proposal is None:
            return {"action":"REJECT","reason":"NO_PROPOSAL"}
        pose=tuple(map(int,pose))
        if (proposal.obs_seq!=int(latest_seq) or
            proposal.obs_seq!=self.monitor.obs_seq or
            proposal.room_id!=self.room_id or
            proposal.observed_hash!=digest(latest_observed)):
            return {"action":"REJECT","reason":"STALE_OBSERVATION_OR_ROOM"}
        if proposal.pose!=pose:
            return {"action":"REJECT","reason":"STALE_ROBOT_POSE"}
        if self.pending_full or self.monitor.pending_level in (S0,S3,S4):
            return {"action":"REJECT","reason":"UNRESOLVED_STRUCTURAL_EVENT"}
        obs=np.asarray(latest_observed)
        safe=np.asarray(native_walkable,bool)&(obs==0)&self.r6.room
        if not safe[pose] or not native_path_valid:
            return {"action":"REJECT","reason":"NATIVE_ASTAR_REJECTED"}
        bfs=_KnownFreePaths(safe)
        p=pose
        for g in proposal.goals:
            if not safe[g] or bfs.path(p,g) is None:
                return {"action":"REJECT","reason":"UNKNOWN_OR_UNREACHABLE_GOAL"}
            p=g
        self.active_goals=list(proposal.goals)
        self.plan_gen+=1
        self.dirty=False
        self.cached_proposal=None
        self.monitor.acknowledge_replan()
        return {"action":"COMMITTED","plan_generation":self.plan_gen,
                "goals":[list(x) for x in self.active_goals],
                "source_obs_seq":proposal.obs_seq,
                "needs_native_ASTAR_at_dispatch":True}

    def checkpoint(self,observed,pose,seq,native_walkable, *, source_goal=None,
                   native_path_valid=True,local_frontiers_exist=True,
                   official_room_valid=True):
        """MANDATORY before dispatching the next cached waypoint.

        The native caller supplies its newest observed LiDAR snapshot, even
        if a 0.5m background update was skipped.
        """
        pose=tuple(map(int,pose))
        if seq>self.monitor.obs_seq or not np.array_equal(observed,self.monitor.observed):
            ev=self.on_scan(observed,pose,seq,native_walkable,
                            active_route_safe=native_path_valid,
                            local_frontiers_exist=local_frontiers_exist,
                            official_room_valid=official_room_valid)
        else:
            ev=self.last_event
        if not native_path_valid or self.monitor.pending_level==S0:
            return {"action":"SAFE_HOLD","reason":"NATIVE_PATH_UNSAFE","event":S0}
        if not local_frontiers_exist or not official_room_valid or self.monitor.pending_level==S4:
            return {"action":"EXIT_OR_REFRESH_ROOMNODE","event":S4,
                    "native_mode_gate_required":True}
        if self.pending_full or self.monitor.pending_level==S3:
            return {"action":"WAIT_STRUCTURAL_REFRESH","event":S3,
                    "LaMa_RoomNode_rebuild_needed":True}
        if not self.active_goals:
            return {"action":"WAIT_NEW_ROUTE","reason":"NO_PENDING_GOALS"}
        active=self.active_goals[0]
        if pose!=active or (source_goal is not None and tuple(source_goal)!=active):
            return {"action":"WAIT_ARRIVAL","expected_goal":[*active]}
        # Verify actual observed data at arrival before consuming a goal.
        self.active_goals.pop(0)
        self.consumed.append((self.plan_gen,active,seq))
        fresh=(self.last_arrival_observed==1)&(np.asarray(observed)!=1)&self.r6.room
        new_cells=int(np.count_nonzero(fresh))
        self.last_arrival_observed=np.asarray(observed,dtype=np.uint8).copy()
        # ZERO ACTUAL observational progress from a visited waypoint is a
        # negative measurement about local predicted sight. Do not reselect
        # speculative adjacent viewpoints until the robot returns to a real,
        # reachable LOCAL WFD frontier. No tuned distance/benefit threshold.
        if new_cells == 0:
            self.active_goals=[]
            self.cached_proposal=None
            self.dirty=False
            self.monitor.acknowledge_replan()
            return {"action":"RESIDUAL_ROOM_WFD",
                    "reason":"NO_NEW_ACTUAL_ROOM_OBSERVATION_SINCE_PREVIOUS_VIEWPOINT",
                    "consumed_goal_rc":list(active),
                    "observed_new_cells":0,
                    "no_G2_replanning_until_room_id_changes":True}
        # Future goals may now be redundant after D02->viewpoint1 scan.
        # Refresh Watchman without re-inferencing LaMa (from CURRENT pose).
        if self.dirty or self.monitor.obs_seq>0:
            answer=self.propose_light()
            if answer["action"]=="LIGHT_PROPOSAL":
                verdict=self.try_commit(self.cached_proposal,observed,pose,
                                        native_walkable,seq,
                                        native_path_valid=native_path_valid)
                if verdict["action"]=="COMMITTED":
                    self.arrival_events.append({"arrival":list(active),
                        "observation_seq":seq, "action":"REPLACED_OLD_CHAIN"})
                    return {"action":"NEXT_GOAL" if self.active_goals else "WAIT_NEW_ROUTE",
                            "source":"NEW_LIGHT_ROUTE",
                            "goal_rc":list(self.active_goals[0]) if self.active_goals else None,
                            "canceled_old_future_chain":True,
                            "replan":answer,"committed":verdict}
                return {"action":"WAIT_NEW_ROUTE","reason":verdict["reason"]}
            return {"action":"WAIT_NEW_ROUTE","reason":answer.get("reason",answer["action"])}
        if not self.active_goals:
            return {"action":"WAIT_NEW_ROUTE","reason":"OLD_ROUTE_EXHAUSTED"}
        current=self.active_goals[0]
        safe=np.asarray(native_walkable,bool)&(np.asarray(observed)==0)&self.r6.room
        if not safe[current] or _KnownFreePaths(safe).path(pose,current) is None:
            return {"action":"WAIT_NEW_ROUTE","reason":"OLD_NEXT_GOAL_UNSAFE"}
        self.arrival_events.append({"arrival":list(active),"observation_seq":seq,"action":"KEEP_CHECKED"})
        return {"action":"NEXT_GOAL","source":"CHECKED_CACHED_CHAIN",
                "goal_rc":list(current), "canceled_old_future_chain":False}
