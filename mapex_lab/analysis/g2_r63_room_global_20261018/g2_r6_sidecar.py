"""Process-isolated G2-R6 policy service. JSONL; never reads GT or future maps.

Messages:
 CHOOSE: room_id, robot_core_rc, room_node_input, room_node_semantic,
         native_scope_npz, native_tick. Reuses cached route if safe.
 UPDATE: observed_npz, robot_core_rc, native_scope_npz, native_tick
 ACK_GOAL: robot_core_rc (must EXACTLY equal active goal before next).
 STATUS, CLOSE.

The caller remains responsible for the native RoomNode/WFD exit gate,
A* safety validation, and scan/control timing. UPDATE/REFINE are advisories.
"""
from __future__ import annotations
import json
import sys
import time
from dataclasses import asdict

import numpy as np

from g2_r6_watchman import R6Config
from r63_exit_planner import RoomGlobalWatchman as RoomWatchman


def get_map(path, names):
    with np.load(path, allow_pickle=False) as f:
        return [f[k].copy() for k in names]


class Service:
    def __init__(self):
        self.active = None
        self.room_id = None
        self.last_tick = -1
        self.last_update = None
        self.plan_epoch_count = 0
        self.update_count = 0

    @staticmethod
    def _load(req):
        obs, mean, var = get_map(req["room_node_input"], ("observed", "mean", "variance"))
        sem = get_map(req["room_node_semantic"], ("semantic",))[0]
        nav = get_map(req["native_scope_npz"], ("reachable_knownfree",))[0]
        return obs, mean, var, sem, nav

    @staticmethod
    def _safe(robot, goal, observed, nav):
        y, x = goal
        return (0 <= y < observed.shape[0] and 0 <= x < observed.shape[1]
                and observed[y, x] == 0 and bool(nav[y, x]))

    def choose(self, req):
        tick = int(req["native_tick"])
        room_id = int(req["room_id"])
        robot = tuple(map(int, req["robot_core_rc"]))
        if tick < self.last_tick:
            raise ValueError("stale native tick")
        self.last_tick = tick
        obs, mean, var, sem, nav = self._load(req)
        rebuilt = False
        refresh = None
        if self.active is not None and self.room_id == room_id and self.active.plan_order:
            refresh = self.active.scan_update(obs, robot, nav, sem)
            if refresh["action"] == "REPLAN_REQUIRED":
                self.active = None
            else:
                self.active.refine_remaining(robot, nav)
        if self.active is None or self.room_id != room_id or not self.active.plan_order:
            self.active = RoomWatchman(obs, mean, var, sem, robot, nav, R6Config())
            report = self.active.plan()
            self.plan_epoch_count += 1
            self.room_id = room_id
            rebuilt = True
        else:
            report = self.active.last_decision
        selected = self.active.plan_order[0] if self.active.plan_order else None
        if selected is not None and not self._safe(robot, selected, obs, nav):
            raise ValueError("R6 selected non-certified native cell")
        return {
            "event": "RESULT", "native_tick": tick, "room_id": room_id,
            "decision": "GOAL" if selected is not None else "DEFER",
            "goal_core_rc": list(selected) if selected is not None else None,
            "goal_sequence_core_rc": [list(g) for g in self.active.plan_order],
            "recomputed_G2_plan": rebuilt, "plan_epoch_count": self.plan_epoch_count,
            "used_causal_snapshot": True, "used_gt": False,
            "first_plan_score": report.get("projected_weighted_gain"),
            "first_plan_route_m": report.get("predicted_route_distance_m"),
            "first_plan_wall_s": report.get("extra_G2_planning_wall_s"),
            "last_update": refresh,
            "requires_native_ASTAR_validation": True,
            "ROOM_WFD_ONLY_EXIT_GATE": True,
        }

    def update(self, req):
        if self.active is None:
            return {"event": "UPDATE", "action": "NO_ACTIVE_ROOM"}
        tick = int(req["native_tick"])
        if tick < self.last_tick:
            return {"event": "UPDATE", "action": "STALE_SCAN_DROPPED"}
        self.last_tick = tick
        obs = get_map(req["observed_npz"], ("observed",))[0]
        nav = get_map(req["native_scope_npz"], ("reachable_knownfree",))[0]
        sem = get_map(req["semantic_npz"], ("semantic",))[0] if req.get("semantic_npz") else None
        robot = tuple(map(int, req["robot_core_rc"]))
        decision = self.active.scan_update(obs, robot, nav, sem)
        self.update_count += 1
        self.last_update = decision
        # Expensive refinement runs in its own process, never robot callback.
        refine = None
        if decision["action"] != "REPLAN_REQUIRED":
            refine = self.active.refine_remaining(robot, nav)
        return {"event": "UPDATE", "native_tick": tick,
                "fast": decision, "refine": refine, "update_count": self.update_count}

    def ack(self, req):
        robot = tuple(map(int, req["robot_core_rc"]))
        if self.active is None or not self.active.plan_order:
            return {"event": "ACK_GOAL", "accepted": False, "reason": "NO_PENDING_GOAL"}
        expected = self.active.plan_order[0]
        if robot != expected:
            return {"event": "ACK_GOAL", "accepted": False, "reason": "NOT_AT_EXACT_GOAL",
                    "expected": list(expected)}
        self.active.plan_order.pop(0)
        return {"event": "ACK_GOAL", "accepted": True,
                "remaining_goals_rc": [list(g) for g in self.active.plan_order]}

    def status(self):
        return {"event": "STATUS", "room_id": self.room_id, "last_tick": self.last_tick,
                "plan_epochs": self.plan_epoch_count, "updates": self.update_count,
                "pending_goals_rc": [list(g) for g in self.active.plan_order] if self.active else [],
                "GT_used": False}

    def handle(self, req):
        action = req.get("action")
        if action == "CHOOSE":
            return self.choose(req)
        if action == "UPDATE":
            return self.update(req)
        if action == "ACK_GOAL":
            return self.ack(req)
        if action == "STATUS":
            return self.status()
        if action == "CLOSE":
            return {"event": "CLOSED"}
        raise ValueError("unsupported action")


def main():
    engine = Service()
    print(json.dumps({"event": "READY", "policy": "G2_R6_WATCHMAN",
                      "protocol": ["CHOOSE", "UPDATE", "ACK_GOAL", "STATUS", "CLOSE"],
                      "no_hidden_GT": True}), flush=True)
    for line in sys.stdin:
        request = {}
        try:
            request = json.loads(line)
            output = engine.handle(request)
        except Exception as exc:
            output = {"event": "ERROR", "type": type(exc).__name__, "message": str(exc)}
        print(json.dumps(output, ensure_ascii=False, allow_nan=False), flush=True)
        if request.get("action") == "CLOSE":
            break


if __name__ == "__main__":
    main()