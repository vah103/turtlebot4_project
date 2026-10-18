"""G2-R6.1: causal, observation-only structural event monitor.

All shape/topology conclusions use real observed LiDAR; prior LaMa mean is
used only to quantify surprise, not certify walls or FREE movement.
No GT or future snapshot is accepted as an input.
"""
from __future__ import annotations
from dataclasses import dataclass
import hashlib
import time
from typing import Optional
import cv2
import numpy as np

S0, S1, S2, S3, S4 = ("S0_SAFETY_HOLD", "S1_KEEP", "S2_LIGHT_REPLAN",
                       "S3_STRUCTURAL_REBUILD", "S4_EXIT_LOCAL")

@dataclass(frozen=True)
class EventConfig:
    min_new_branch_cells: int = 40
    min_new_frontier_cells: int = 10
    min_old_component_cells: int = 12
    min_bridge_cells: int = 3
    max_bridge_gap_width_cells: int = 5
    min_portal_wall_pairs: int = 4
    min_portal_confirm_scans: int = 2
    unexpected_wall_min_cells: int = 32
    large_goal_value_change_fraction: float = .20
    max_replan_time_ms: float = 100.
    border_connectivity: int = 4

def digest(a):
    return hashlib.sha256(np.asarray(a).tobytes()).hexdigest()

def check_maps(prev, cur, room):
    a,b,m = (np.asarray(x) for x in (prev,cur,room))
    if a.shape != b.shape or b.shape != m.shape or a.ndim != 2:
        raise ValueError("2D same-shape observation/ROOM maps required")
    if not (np.isin(a,(0,1,2)).all() and np.isin(b,(0,1,2)).all()):
        raise ValueError("observed maps must use 0 FREE / 1 UNKNOWN / 2 WALL")
    return a.astype(np.uint8,copy=False), b.astype(np.uint8,copy=False), m.astype(bool,copy=False)

def observation_delta(prev, cur, mean, variance, accepted_room):
    a,b,room=check_maps(prev,cur,accepted_room)
    m=np.asarray(mean)
    v=np.asarray(variance)
    if m.shape != a.shape or v.shape != a.shape:
        raise ValueError("prediction shape mismatch")
    if not np.isfinite(m).all() or not np.isfinite(v).all():
        raise ValueError("non-finite predicted occupancy")
    new_free=(a==1)&(b==0)
    new_wall=(a==1)&(b==2)
    changed_wall=(a==0)&(b==2)
    nonmonotonic=((a==0)&(b==1)) | ((a==2)&(b!=2))
    uncertainty=1.0/(1.0+np.sqrt(np.maximum(v,0)))
    return {
        "new_free":new_free, "new_wall":new_wall,
        "changed_wall":changed_wall, "nonmonotonic":nonmonotonic,
        "num_new_free":int(new_free.sum()),"num_new_wall":int(new_wall.sum()),
        "num_new_free_inside":int((new_free&room).sum()),
        "num_new_free_outside":int((new_free&(~room)).sum()),
        "num_changed_wall":int(changed_wall.sum()),
        "unexpected_free_mass":float((np.clip(m,0,1)*uncertainty*new_free).sum()),
        "unexpected_wall_mass":float(((1-np.clip(m,0,1))*uncertainty*new_wall).sum()),
        "unexpected_wall_strong_count":int((new_wall&(m<.25)).sum()),
        "old_sha256":digest(a),"new_sha256":digest(b)
    }

def _components(a):
    n,l,stats,centroids=cv2.connectedComponentsWithStats(np.asarray(a,np.uint8),connectivity=4)
    return l,stats

def topology_features(prev, cur, room, delta, cfg:EventConfig):
    oldfree=(prev==0)&room
    newfree=(cur==0)&room
    oldlabels,oldstats=_components(oldfree)
    newlabels,newstats=_components(newfree)
    fresh=(delta["new_free"]&room)
    patched,patchstats=_components(fresh)
    h,w=prev.shape
    # A newly observed FREE bridge joining two formerly disconnected, sizable
    # FREE components is direct structural evidence, independent of mean LaMa.
    bridge_patches=[]
    branch_patches=[]
    walls=(prev==2)
    cross = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    old_boundary = cv2.dilate(oldfree.astype(np.uint8),cross,iterations=1)>0
    old_frontier=(old_boundary & (prev==1) & room)
    cur_boundary=cv2.dilate(newfree.astype(np.uint8),cross,iterations=1)>0
    new_frontier=cur_boundary & (cur==1) & room & (~old_frontier)
    for label in range(1, len(patchstats)):
        size=int(patchstats[label,cv2.CC_STAT_AREA])
        if size==0:
            continue
        patch=patched==label
        # A verified 4-connected bridge must NOT be inferred from mere
        # diagonal contact across two FREE components.
        neighborhood=cv2.dilate(patch.astype(np.uint8),cross,iterations=1)>0
        oldids=set(np.unique(oldlabels[neighborhood&oldfree]).tolist())-{0}
        major=sorted(i for i in oldids if oldstats[i,cv2.CC_STAT_AREA] >= cfg.min_old_component_cells)
        new_frontier_cells=int(np.count_nonzero(new_frontier & neighborhood))
        item={"patch_cells":size, "touches_old_components":len(major),
              "new_frontier_cells":new_frontier_cells,
              "bbox_xywh":tuple(map(int,patchstats[label,:4]))}
        if len(major)>=2 and size>=cfg.min_bridge_cells:
            bridge_patches.append(item)
        if size>=cfg.min_new_branch_cells and new_frontier_cells>=cfg.min_new_frontier_cells:
            branch_patches.append(item)
    # Supporting geometry: a fresh FREE gap flanked by *observed* WALL, in
    # either cardinal direction. Never call this a confirmed door alone.
    y,x=np.where(fresh)
    pairs=0
    for gap in (2,3):
        xx=x-gap
        yy=y-gap
        validx=(xx>=0)&(x+gap<w)
        validy=(yy>=0)&(y+gap<h)
        pairs += int(np.count_nonzero(validx & walls[y,np.clip(xx,0,w-1)]
             & walls[y,np.clip(x+gap,0,w-1)]))
        pairs += int(np.count_nonzero(validy & walls[np.clip(yy,0,h-1),x]
             & walls[np.clip(y+gap,0,h-1),x]))
    return {"new_free_patches":int(len(patchstats)-1),
            "bridge_patches":bridge_patches,"branch_patches":branch_patches,
            "new_frontier_cells":int(new_frontier.sum()),
            "flanking_observed_wall_pairs":pairs,
            "verified_observed_bridge":bool(bridge_patches),
            "portal_candidate":pairs >= cfg.min_portal_wall_pairs and bool(fresh.any())}

def event_priority(code):
    return {S1:1,S2:2,S3:3,S4:4,S0:5}[code]

class StructuralEventMonitor:
    """One ROOM monitor per accepted planner epoch; deterministic and idempotent."""
    def __init__(self, observed, mean, variance, accepted_room, pose,
                 obs_seq=0, cfg:Optional[EventConfig]=None):
        self.cfg=cfg or EventConfig()
        self.observed=np.asarray(observed,dtype=np.uint8).copy()
        self.mean=np.asarray(mean,dtype=np.float32).copy()
        self.var=np.asarray(variance,dtype=np.float32).copy()
        self.room=np.asarray(accepted_room,dtype=bool).copy()
        check_maps(self.observed,self.observed,self.room)
        if self.mean.shape!=self.observed.shape or self.var.shape!=self.observed.shape:
            raise ValueError("bad LaMa epoch")
        self.pose=tuple(map(int,pose))
        self.obs_seq=int(obs_seq)
        self.portal_count=0
        self.event_no=0
        self.pending_level=S1
        self.last=None

    def refresh(self, cur, pose, obs_seq, *, active_route_safe=True,
                local_frontiers_exist=True, official_room_valid=True,
                future_goal_gain_drop=0.0, confirmed_new_semantic=None):
        start=time.perf_counter()
        seq=int(obs_seq)
        b=np.asarray(cur,np.uint8)
        if seq<self.obs_seq:
            return {"event":S1,"reason":"STALE_SCAN_REJECTED","accepted":False,
                    "observation_seq":self.obs_seq}
        if seq==self.obs_seq:
            if not np.array_equal(self.observed,b):
                raise ValueError("same observation sequence with different cells")
            if self.last is not None:
                return {**self.last,"reason":"DUPLICATE_SCAN_IDEMPOTENT","accepted":False}
        delta=observation_delta(self.observed,b,self.mean,self.var,self.room)
        features=topology_features(self.observed,b,self.room,delta,self.cfg)
        robot=tuple(map(int,pose))
        if not (0<=robot[0]<b.shape[0] and 0<=robot[1]<b.shape[1]):
            raise ValueError("robot pose out of range")
        # Any observed FREE->WALL correction invalidates the cached safe
        # corridor until native inflation/A* explicitly recertifies motion.
        # Native inflated A* validates the active route. A correction outside
        # that path must NOT cause repeated S0/laMa relaunches. Report it for
        # native re-certification, but do not mislabel remote walls as unsafe.
        unsafe=(not active_route_safe or b[robot]!=0)
        reason="EXPECTED_OBSERVATION_PROGRESS"
        level=S1
        if unsafe:
            level,reason=S0,"NATIVE_PATH_UNSAFE"
        elif not official_room_valid or not local_frontiers_exist:
            level,reason=S4,"NO_ACCEPTED_ROOM_OR_LOCAL_WFD"
        elif features["verified_observed_bridge"]:
            level,reason=S3,"OBSERVED_COMPONENT_BRIDGE"
        elif not self.room[robot]:
            level,reason=S3,"ROBOT_EXITED_ACCEPTED_ROOM"
        else:
            self.portal_count = self.portal_count+1 if features["portal_candidate"] else 0
            if self.portal_count>=self.cfg.min_portal_confirm_scans:
                level,reason=S3,"TEMPORALLY_SUPPORTED_PORTAL"
            elif features["branch_patches"]:
                level,reason=S2,"NEW_FREE_BRANCH_WITH_FRONTIER"
            elif features["new_frontier_cells"] and delta["num_new_free_outside"]:
                level,reason=S2,"NEW_FRONTIER_NEAR_OLD_ROOM"
            elif delta["unexpected_wall_strong_count"] and features["new_free_patches"]:
                level,reason=S2,"PREDICTION_DISAGREEMENT_LIGHT"
            elif future_goal_gain_drop >= self.cfg.large_goal_value_change_fraction:
                level,reason=S2,"FUTURE_GOALS_LOST_VALUE"
            elif features["portal_candidate"]:
                level,reason=S2,"PROVISIONAL_PORTAL_WAIT_FOR_CONFIRMATION"
        if confirmed_new_semantic is not None:
            newsem=np.asarray(confirmed_new_semantic)
            if newsem.shape!=b.shape:
                raise ValueError("new semantic shape mismatch")
            robot_room=newsem[robot]==1
            if not robot_room and not unsafe:
                level,reason=S4,"FRESH_ROOMNODE_NO_ROBOT_ROOM"
            else:
                fresh_free=(newsem==1)&(b!=2)
                _,fresh_labels=cv2.connectedComponents(fresh_free.astype(np.uint8),connectivity=4)
                current_room=fresh_labels==fresh_labels[robot]
                if np.count_nonzero(current_room^self.room)>=max(20,int(.05*self.room.sum())) and not unsafe:
                    level,reason=S3,"FRESH_ROOMNODE_SCOPE_CHANGED"
        # Do not infer a new Room Node mask merely by reusing cached semantic.
        self.observed=b.copy()
        self.pose=robot
        self.obs_seq=seq
        self.event_no+=1
        if event_priority(level)>event_priority(self.pending_level):
            self.pending_level=level
        result={"event":level,"reason":reason,"accepted":True,
                "observation_seq":seq,"event_no":self.event_no,
                "map_sha256":delta["new_sha256"],
                "new_free":delta["num_new_free"],
                "new_wall":delta["num_new_wall"],
                "outside_old_room_new_free":delta["num_new_free_outside"],
                "unexpected_wall_strong_count":delta["unexpected_wall_strong_count"],
                "unexpected_free_mass":round(delta["unexpected_free_mass"],3),
                "unexpected_wall_mass":round(delta["unexpected_wall_mass"],3),
                "bridge_count":len(features["bridge_patches"]),
                "branch_count":len(features["branch_patches"]),
                "portal_candidate":features["portal_candidate"],
                "portal_support_scans":self.portal_count,
                "frontier_new_cells":features["new_frontier_cells"],
                "geometry":features,
                "pending_level":self.pending_level,
                "detection_wall_s":round(time.perf_counter()-start,5),
                "used_gt":False,"LaMa_inference_called":False,
                "RoomNode_inference_called":False}
        self.last=result
        return result

    def acknowledge_replan(self):
        self.pending_level=S1
        self.portal_count=0
