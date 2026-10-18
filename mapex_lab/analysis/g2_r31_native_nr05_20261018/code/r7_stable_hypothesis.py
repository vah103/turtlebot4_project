"""R7-R3.1 stable structural hypotheses (standalone; no GT or robot commands).

The first accepted LOCAL room snapshot freezes CHAMBER ID and portal semantics.
New LiDAR can refute predicted occupancy cellwise but cannot silently relabel
the entire chamber partition on every later LaMa/RoomNode epoch. Room ID
replacement is ONLY by the native G1 controller at an explicit room transition.
"""
from __future__ import annotations
import hashlib,math,time
import cv2
import numpy as np
from r7_missions import R7Missions,Params

class StableHypothesis:
    def __init__(self, observed,mean,variance,semantic,robot,native_walkable,
                 cfg=None):
        self.cfg=cfg or Params()
        self.initial_obs=np.asarray(observed,np.uint8).copy()
        self.initial_mean=np.asarray(mean,np.float32).copy()
        self.initial_sem=np.asarray(semantic,np.uint8).copy()
        self.entry=tuple(map(int,robot))
        self.first=R7Missions(observed,mean,variance,semantic,robot,native_walkable,cfg=self.cfg)
        self.zones=self.first.zone.copy()
        self.zoneids=tuple(self.first.zoneids)
        self.entry_zone=int(self.first.entry_zone)
        self.others=tuple(z for z in self.zoneids if z!=self.entry_zone)
        self.portal=self.first.portal
        self.stable_corners={z:dict(self.first.corners(z)) for z in self.zoneids}
        self.version=1
        self.revisions=[]
        self.fingerprint=hashlib.sha256(self.zones.tobytes()).hexdigest()
        self.observed=self.initial_obs.copy()
        self.nav=np.asarray(native_walkable,bool).copy()
        self.plan1=self.first.plan("INITIAL")
        self.has_active_room_hypothesis=bool(self.portal is not None and self.others)
        self._last_blocker_count=0
        self.last_report=None

    def _new_blockers(self, obs):
        # Contradiction requires actual LIDAR occupancy on a cell which was
        # UNKNOWN + predicted FREE at entry, and belonged to frozen chamber.
        candidate=(self.initial_obs==1)&(self.initial_mean<.5)&(obs==2)
        regions=[]
        for z in self.others:
            mask=np.uint8(candidate&(self.zones==z))
            n,labels=cv2.connectedComponents(mask,connectivity=8)
            for j in range(1,n):
                points=np.argwhere(labels==j)
                if len(points)<self.cfg.min_obstacle_cells:
                    continue
                center=points.mean(axis=0)
                regions.append({"zone":int(z),
                                "centroid":tuple(map(float,center)),
                                "size":int(len(points)),
                                "observed_confirmed":True})
        return regions

    def _viewpoints(self,zone,objects,obs):
        # Go to the far boundary *beyond* obstacles relative to the frozen
        # entrance/portal, not a high-IG near-divider compromise.
        if not objects or self.portal is None:return []
        center=np.average([b["centroid"] for b in objects],
                          axis=0,weights=[b["size"] for b in objects])
        direction=center-np.asarray(self.portal,float)
        if float(np.linalg.norm(direction))<1:
            return []
        axis=int(np.argmax(np.abs(direction)))
        forward="bottom" if axis==0 and direction[0]>0 else \
                "top" if axis==0 else "right" if direction[1]>0 else "left"
        # Choose transverse corners of the far wall.
        if axis==0:
            labels=[forward+"_left",forward+"_right"]
            other_axis=1
        else:
            labels=["top_"+forward,"bottom_"+forward]
            other_axis=0
        options=self.stable_corners.get(zone,{})
        valid=[]
        for label in labels:
            p=options.get(label)
            if p is None:continue
            # It really must lie beyond the occluder in the entry direction.
            if float(np.dot(np.asarray(p,float)-center,direction))<=0:
                continue
            unknown=(obs==1)&(self.zones==zone)
            coord=np.indices(unknown.shape)[other_axis]
            mid=np.median(np.argwhere(self.zones==zone)[:,other_axis])
            cells=int((unknown&(coord>mid if p[other_axis]>mid else coord<=mid)).sum())
            # Do not add a viewpoint for a half with no unseen cells. Keep
            # the full room's WFD end condition as independent exit gate.
            if cells==0:continue
            valid.append({"task_id":"STABLE_ZONE_%d_%s"%(int(zone),label),
                "zone":int(zone),"corner":label,"goal_rc":list(map(int,p)),
                "unseen_half_cells":cells,
                "object_centers":[list(b["centroid"]) for b in objects],
                "confirmed_observed_objects":len(objects),
                "opposite_to_entry":True,"native_motion_authorized":False})
        valid.sort(key=lambda v:(-v["unseen_half_cells"],v["corner"]))
        return valid

    def update(self,obs,native_walkable,scan_tick,robot=None):
        started=time.perf_counter()
        obs=np.asarray(obs,np.uint8)
        nav=np.asarray(native_walkable,bool)
        if obs.shape!=self.initial_obs.shape or nav.shape!=obs.shape:
            raise ValueError("ROOM_SHAPE_CHANGED")
        if not np.isin(obs,[0,1,2]).all():raise ValueError("BAD_LIDAR_MAP")
        # Nothing is allowed to change previously observed known cells. A
        # contradiction to this is a sensor/map consistency error, not a
        # reason to silently change frozen chamber identity.
        inconsistency=int((((self.observed==0)&(obs==2))|
                           ((self.observed==2)&(obs==0))).sum())
        # The live mapper may revise a few previously labelled cells as
        # rays accumulate. Log every free/occupied flip and let the current
        # *native inflated map* veto unsafe motion; never distort chamber
        # identity to accommodate these local sensor updates.
        self.observed=obs.copy()
        self.nav=nav.copy()
        blockers=self._new_blockers(obs)
        count=sum(b["size"] for b in blockers)
        changed=count>self._last_blocker_count
        if changed:
            self.version+=1
            self.revisions.append({"version":self.version,
                                   "native_tick":int(scan_tick),
                                   "additional_confirmed_blocker_cells":count-self._last_blocker_count,
                                   "confirmed_blocker_cells":count})
            self._last_blocker_count=count
        tasks=[]
        for z in self.others:
            items=[b for b in blockers if b["zone"]==z]
            tasks.extend(self._viewpoints(z,items,obs))
        self.last_report={
            "revision_version":self.version,
            "native_tick":int(scan_tick),
            "frozen_portal_rc":list(self.portal) if self.portal else None,
            "frozen_zone_sha256":self.fingerprint,
            "entry_zone":self.entry_zone,
            "objects_in_other_chambers":blockers,
            "confirmed_unexpected_obstacle_cells":count,
            "tasks":tasks,
            "snapshot_from_future":False,
            "GT_used":False,
            "semantic_zone_relabeled":False,
            "current_map_known_cell_label_flips":inconsistency,
            "planning_s":round(time.perf_counter()-started,4)}
        return self.last_report

    def close(self):
        return None
