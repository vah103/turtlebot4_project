"""G2-R6 native LOCAL adapter. Independent subprocess; native A* is authority.

Exports G2Run interface for the frozen Run11 worker fork.
CHOOSE can be heavy, but cached_next_goal() skips LaMa at intermediate goals.
on_scan() queues 0.5m observation updates to a background worker, never waits
on G2 optimization inside the LiDAR/motion callback.

NOT a certified robot safety controller and NOT an executed native run.
"""
from __future__ import annotations

import json
import os
import select
import subprocess
import threading
import time
import atexit
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import pyastar2d

ROOT = Path(__file__).resolve().parent


def _safe_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False)+"\n")


class G2Run:
    def __init__(self, exp_dir, timeout=30):
        self.root = Path(exp_dir) / "g2_r61"
        self.root.mkdir(parents=True, exist_ok=False)
        (self.root/"decisions").mkdir()
        (self.root/"updates").mkdir()
        self.native_pad = 500
        self.timeout = timeout
        self.lock = threading.Lock()
        self.err = (self.root/"sidecar.stderr").open("w")
        self.proc = subprocess.Popen(["/usr/bin/python3", "-u", str(ROOT/"g2_r61_sidecar.py")],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=self.err, text=True, bufsize=1)
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="R6_scan")
        self.update_future = None
        self.current_goal = None
        self.local_lock = False
        self.heavy_required = False
        self.room_id = None
        self.last_scan_pose = None
        self.last_scope_mask = None
        self.last_epoch = 0
        self.latest_observation = None
        self.latest_scan_tick = -1
        hello = self._recv()
        if hello.get("event") != "READY":
            raise RuntimeError("G2-R6 sidecar startup error "+str(hello))
        _safe_json(self.root/"identity.json",
                   {"policy": "G2_R61_EVENT_DRIVEN_R1", "source": str(ROOT),
                    "replay_offline_verified": True, "native_science_verified": False,
                    "knownfree_occupancy_required": True, "no_GT_passed_to_policy": True})
        atexit.register(self.close)

    def _recv(self):
        stop = time.monotonic() + self.timeout
        while time.monotonic() < stop:
            ready, _, _ = select.select([self.proc.stdout], [], [], 0.2)
            if ready:
                line = self.proc.stdout.readline()
                if not line:
                    raise RuntimeError("G2-R6 sidecar EOF")
                return json.loads(line)
            if self.proc.poll() is not None:
                raise RuntimeError("G2-R6 sidecar exited")
        raise TimeoutError("G2-R6 sidecar timeout; native must fail closed")

    def _request(self, request):
        with self.lock:
            if self.proc.poll() is not None:
                raise RuntimeError("G2-R6 subprocess unavailable")
            self.proc.stdin.write(json.dumps(request)+"\n")
            self.proc.stdin.flush()
            reply = self._recv()
            if reply.get("event") == "ERROR":
                raise RuntimeError("G2-R6 sidecar error "+str(reply))
            return reply

    def close(self):
        exe = getattr(self, "executor", None)
        if exe is not None:
            exe.shutdown(wait=True)
            self.executor = None
        p = getattr(self, "proc", None)
        if p is not None and p.poll() is None:
            try:
                self._request({"action": "CLOSE"})
                p.wait(timeout=3)
            except Exception:
                p.terminate()
        err = getattr(self, "err", None)
        if err is not None and not err.closed:
            err.close()

    @staticmethod
    def _pose(p, native_pad):
        return tuple((np.asarray(p, dtype=np.int32)-native_pad).tolist())

    def _scope(self, room_run, cur_pose, tick):
        n = room_run.epoch
        room_mask = room_run.active_mask
        if room_mask is None:
            raise ValueError("native LOCAL RoomNode scope is required")
        h, w = room_mask.shape
        grid = room_run.grid(None)
        known = np.isfinite(grid[self.native_pad:self.native_pad+h,
                                 self.native_pad:self.native_pad+w]) & room_mask
        pose = self._pose(cur_pose, self.native_pad)
        if not known[pose]:
            raise RuntimeError("robot not reachable in LOCAL knownfree")
        path = self.root/("native_scope_%06d.npz"%n)
        np.savez_compressed(path, reachable_knownfree=known.astype(np.uint8))
        return path, known

    def _validate_goal(self, gcore, pose, grid, native_mask=None):
        if gcore is None:
            return None
        p = np.asarray(gcore, dtype=np.int32) + self.native_pad
        if p[0]<0 or p[1]<0 or p[0]>=grid.shape[0] or p[1]>=grid.shape[1]:
            return None
        if native_mask is not None and not native_mask[tuple(gcore)]:
            return None
        if not np.isfinite(grid[tuple(p)]):
            return None
        route = pyastar2d.astar_path(np.ascontiguousarray(grid,dtype=np.float32),
                                     np.asarray(pose,dtype=np.int32), p,
                                     allow_diagonal=False)
        return p if route is not None else None

    def choose(self, t, room_run, cur_pose):
        if not room_run.last_scope or room_run.last_scope["mode"] != "LOCAL":
            self.local_lock = False
            self.current_goal = None
            self.room_id = None
            return None
        scope_path, known = self._scope(room_run, cur_pose, t)
        n = room_run.epoch
        base = Path(room_run.inferdir)/("decision_%06d"%n)
        req = {"action": "CHOOSE", "native_tick": int(t),
               "room_id": int(room_run.last_scope["room_id"]),
               "robot_core_rc": list(self._pose(cur_pose, self.native_pad)),
               "room_node_input": str(base)+"_input.npz",
               "room_node_semantic": str(base)+"_semantic.npz",
               "native_scope_npz": str(scope_path)}
        started = time.perf_counter()
        answer = self._request(req)
        if answer["event"] != "RESULT" or answer["native_tick"] != t:
            raise RuntimeError("G2-R6 epoch mismatch")
        goal = self._validate_goal(answer["goal_core_rc"], cur_pose,
                                   room_run.grid(None), known)
        if answer["decision"] == "GOAL" and goal is None:
            raise RuntimeError("G2-R6 goal failed native knownfree A*")
        self.local_lock = goal is not None
        self.current_goal = goal
        self.last_scope_mask = room_run.active_mask.copy()
        self.room_id = req["room_id"]
        self.last_epoch = n
        self.heavy_required = False
        self.last_scan_pose = np.asarray(cur_pose).copy()
        self.latest_observation = None
        self.latest_scan_tick = int(t)
        answer["native_goal_rc"] = goal.tolist() if goal is not None else None
        answer["native_validation"] = "KNOWNFREE_ASTAR" if goal is not None else "DEFER"
        answer["native_adapter_s"] = round(time.perf_counter()-started, 4)
        _safe_json(self.root/"decisions"/("decision_%06d.json"%n), answer)
        return goal

    def needs_replan(self, pose, obsgrid, goal, threshold_m, ppm):
        if not self.local_lock:
            return None
        self._drain()
        if self.heavy_required:
            return True
        if goal is None or np.array_equal(pose, goal):
            return True
        g = np.asarray(goal, dtype=np.int32)
        if g[0]<0 or g[1]<0 or g[0]>=obsgrid.shape[0] or g[1]>=obsgrid.shape[1]:
            return True
        if not np.isfinite(obsgrid[tuple(g)]):
            return True
        return pyastar2d.astar_path(np.ascontiguousarray(obsgrid,dtype=np.float32),
                np.asarray(pose,dtype=np.int32),g,allow_diagonal=False) is None

    def cached_next_goal_legacy_unchecked(self, t, room_run, cur_pose, grid):
        """OBSOLETE R6 behavior: intentionally NOT called by R6.1."""
        if not self.local_lock or self.heavy_required or self.current_goal is None:
            return None
        if not np.array_equal(np.asarray(cur_pose,dtype=np.int32), self.current_goal):
            return None
        self._drain()
        if self.heavy_required:
            return None
        answer = self._request({"action": "ACK_GOAL",
                                "robot_core_rc": list(self._pose(cur_pose,self.native_pad))})
        if not answer.get("accepted"):
            raise RuntimeError("G2-R6 tried to advance unvisited goal "+str(answer))
        rest = answer["remaining_goals_rc"]
        if not rest:
            self.local_lock = False
            self.current_goal = None
            return None
        goal = self._validate_goal(rest[0], cur_pose, room_run.grid(grid), self.last_scope_mask)
        if goal is None:
            self.local_lock = False
            self.current_goal = None
            self.heavy_required = True
            return None
        self.current_goal = goal
        _safe_json(self.root/"decisions"/("cached_tick_%06d.json"%t),
                   {"tick": t, "native_goal_rc": goal.tolist(),
                    "laMa_recomputed": False, "goal_ack_verified": True,
                    "remaining": rest, "GT_used": False})
        return goal

    def _background_update(self, request):
        return self._request(request)

    def _drain(self):
        fut = self.update_future
        if fut is not None and fut.done():
            result = fut.result()  # error fail closed; do not conceal broken sidecar
            self.update_future = None
            if result.get("event") == "UPDATE" and result.get("fast",{}).get("action") == "REPLAN_REQUIRED":
                self.heavy_required = True
            if result.get("event") == "UPDATE":
                _safe_json(self.root/"updates"/("result_%06d.json"%result["native_tick"]),result)

    def on_scan(self, t, cur_pose, mapper_obs, room_run):
        """Capture EVERY causal scan, then optionally queue a 0.5m background job."""
        if not self.local_lock or not room_run or not room_run.last_scope:
            return
        # Snapshot is cheap; all intervening scans are included in the latest
        # cumulative observed map even when the background worker is busy.
        mask=self.last_scope_mask
        if mask is not None:
            h,w=mask.shape
            full=np.asarray(mapper_obs)
            core=full[self.native_pad:self.native_pad+h,self.native_pad:self.native_pad+w]
            snap=np.ones((h,w),np.uint8)
            snap[core==0]=0; snap[core==1]=2
            self.latest_observation=snap
            self.latest_scan_tick=int(t)+1
        if room_run.last_scope["mode"] != "LOCAL":
            return
        self._drain()
        if self.heavy_required or (self.update_future and not self.update_future.done()):
            return
        pose = np.asarray(cur_pose,dtype=np.int32)
        if self.last_scan_pose is not None and np.linalg.norm(pose-self.last_scan_pose)*.1 < .5:
            return
        mask = self.last_scope_mask
        if mask is None:
            return
        h, w = mask.shape
        full = np.asarray(mapper_obs)
        core = full[self.native_pad:self.native_pad+h,self.native_pad:self.native_pad+w]
        observed = np.ones((h,w), np.uint8)
        observed[core == 0] = 0
        observed[core == 1] = 2
        # Conservative cached native scope: never grant access to newly predicted FREE.
        nav = np.isfinite(room_run.scope_grid[self.native_pad:self.native_pad+h,
                                               self.native_pad:self.native_pad+w])
        nav &= mask & (observed==0)
        if not nav[tuple(pose-self.native_pad)]:
            self.heavy_required = True
            return
        obsfile=self.root/"updates"/("scan_%06d.npz"%t)
        navfile=self.root/"updates"/("scope_%06d.npz"%t)
        np.savez_compressed(obsfile, observed=observed)
        np.savez_compressed(navfile, reachable_knownfree=nav.astype(np.uint8))
        request={"action":"UPDATE","native_tick":int(t)+1,
                 "observed_npz":str(obsfile),"native_scope_npz":str(navfile),
                 "robot_core_rc":list(self._pose(pose,self.native_pad))}
        self.last_scan_pose=pose.copy()
        self.update_future=self.executor.submit(self._background_update,request)


    def cached_next_goal(self, t, room_run, cur_pose, grid):
        """R6.1 mandatory post-arrival structural checkpoint.

        We never ACK a cached goal by itself. A post-arrival observed snapshot
        is sent to the sidecar; only CHECKPOINT's fresh NEXT_GOAL may dispatch.
        """
        if not self.local_lock or self.heavy_required or self.current_goal is None:
            return None
        if not np.array_equal(np.asarray(cur_pose,np.int32),self.current_goal):
            return None
        self._drain()
        if self.heavy_required:
            return None
        obs=self.latest_observation
        if obs is None or self.latest_scan_tick<0:
            self.heavy_required=True
            return None
        h,w=obs.shape
        mask=self.last_scope_mask
        if mask is None or mask.shape!=obs.shape:
            self.heavy_required=True
            return None
        nav=(np.isfinite(np.asarray(grid)[self.native_pad:self.native_pad+h,
                                          self.native_pad:self.native_pad+w]) &
             (obs==0) & mask)
        robot_core=list(self._pose(cur_pose,self.native_pad))
        if not nav[tuple(robot_core)]:
            self.heavy_required=True
            return None
        scan_npz=self.root/"updates"/("arrival_scan_%06d.npz"%t)
        scope_npz=self.root/"updates"/("arrival_scope_%06d.npz"%t)
        np.savez_compressed(scan_npz,observed=obs)
        np.savez_compressed(scope_npz,reachable_knownfree=nav.astype(np.uint8))
        reply=self._request({"action":"CHECKPOINT",
             "native_tick":int(self.latest_scan_tick),
             "observed_npz":str(scan_npz),
             "native_scope_npz":str(scope_npz),
             "robot_core_rc":robot_core,
             "native_path_valid":True})
        if reply.get("event")!="CHECKPOINT":
            raise RuntimeError("R61 malformed checkpoint reply")
        _safe_json(self.root/"decisions"/("checkpoint_tick_%06d.json"%t),reply)
        if reply.get("action")!="NEXT_GOAL":
            self.heavy_required=True
            return None
        nextgoal=reply.get("goal_rc")
        goal=self._validate_goal(nextgoal,cur_pose,room_run.grid(grid),mask)
        if goal is None:
            self.heavy_required=True
            return None
        self.current_goal=goal
        self.local_lock=True
        return goal