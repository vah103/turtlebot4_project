"""Independent G2-R7-R3: structural observation missions, not goal scoring.

Predicted LaMa structure is an explicit, defeasible hypothesis. R7-R3 first
finds chambers, passages, interior occluders and observation *obligations*,
then chooses safe inset far-side viewpoints per obligation. It does NOT
maximize UNKNOWN gain / use MCTS / reuse R4/R6.2 planner logic.

This code creates INTENT goals only; NEVER commands a robot. Predicted FREE
is not a native navigation permission. GT and future observations are absent.
"""
from __future__ import annotations
from dataclasses import dataclass
from collections import Counter
import math,time
import numpy as np
from geometry_reference_r2 import RoomStructure,GridPaths,Sight,R7Settings,checksum


@dataclass(frozen=True)
class Params:
    resolution_m: float = .10
    robot_radius_m: float = .20
    additional_clearance_m: float = .10
    min_obstacle_cells: int = 3

    @property
    def inset_cells(self):
        return max(2,int(math.ceil((self.robot_radius_m+
                                   self.additional_clearance_m)/
                                  self.resolution_m)))


class R7Missions:
    def __init__(self,obs,mean,variance,semantic,robot,native_walkable,
                 cfg=None, virtual_anchor=False, entry_anchor=None):
        self.cfg=cfg or Params()
        self.obs=np.asarray(obs,np.uint8).copy()
        self.mean=np.asarray(mean,np.float32).copy()
        self.variance=np.asarray(variance,np.float32).copy()
        self.semantic=np.asarray(semantic,np.uint8).copy()
        self.robot=tuple(map(int,robot))
        # Persisted from the first LOCAL entry event, not the latest pose.
        # As the robot moves among chambers the identity of the entry
        # chamber remains unchanged.
        self.entry_anchor=(tuple(map(int,entry_anchor))
                           if entry_anchor is not None else self.robot)
        self.native=np.asarray(native_walkable,bool).copy()
        self.geo=RoomStructure(self.obs,self.mean,self.semantic,self.robot,
                               virtual_anchor=virtual_anchor)
        self.zoneids=self.geo.zone_ids
        self.zone=self.geo.zones
        self.entry_zone=(int(self.zone[self.entry_anchor])
                         if self.zone[self.entry_anchor]>0
                         else int(self._closest_zone_at_anchor(self.entry_anchor)))
        self.portal=tuple(self.geo.portal_candidates[0]) if self.geo.portal_candidates else None
        self.pred_safe=self.geo.predfree&(self.geo.edt>=self.cfg.inset_cells)
        self.pred_safe[self.robot]=True
        self.pred_paths=GridPaths(self.geo.predfree)
        self.native_safe=(self.obs==0)&self.native
        self.native_paths=GridPaths(self.native_safe)
        self.bounds={}
        for z in self.zoneids:
            yy,xx=np.nonzero(self.zone==z)
            self.bounds[z]=(int(yy.min()),int(yy.max()),
                            int(xx.min()),int(xx.max()))
        self.objects=self._interior_occluders()
        self._corners={}
        self._zone_unknown={z:int((self.geo.unknown&(self.zone==z)).sum())
                            for z in self.zoneids}

    def _closest_zone_at_anchor(self,anchor):
        best=None
        for z in self.zoneids:
            y,x=np.nonzero(self.zone==z)
            if not len(y):continue
            cost=np.min((y-anchor[0])**2+(x-anchor[1])**2)
            pair=(int(cost),int(z))
            if best is None or pair<best:best=pair
        return 0 if best is None else best[1]

    def _closest_zone(self,center):
        cy,cx=center
        best=None
        for z in self.zoneids:
            y,x=np.nonzero((self.zone==z))
            if not len(y):continue
            dist2=(y-cy)**2+(x-cx)**2
            score=float(dist2.min())
            candidate=(score,z)
            if best is None or candidate<best:best=candidate
        return None if best is None else best[1]

    def _interior_occluders(self):
        detected=[]
        miny=min(v[0] for v in self.bounds.values())
        maxy=max(v[1] for v in self.bounds.values())
        minx=min(v[2] for v in self.bounds.values())
        maxx=max(v[3] for v in self.bounds.values())
        for i,raw in enumerate(self.geo.obstacles):
            if raw["component_area"]<self.cfg.min_obstacle_cells:
                continue
            y,x=raw["center"]
            # Reject outer enclosure, already excluded in R2 but check again.
            clearance=self.cfg.inset_cells
            enclosure_margin=2*clearance
            if (abs(y-miny)<enclosure_margin or abs(y-maxy)<enclosure_margin or
                abs(x-minx)<enclosure_margin or abs(x-maxx)<enclosure_margin):
                continue
            z=self._closest_zone((y,x))
            if z is None:continue
            ys=[p[0] for p in raw["boundary"]]
            xs=[p[1] for p in raw["boundary"]]
            h=max(ys)-min(ys)+1
            w=max(xs)-min(xs)+1
            # A long wall touching the boundary between two chambers
            # defines a DIVIDER, not an isolated interior object with a
            # circumferential 'rear face' obligation.
            if self.portal is not None:
                pz=self.portal
                neighbor_zones=[k for k in self.zoneids if k!=z]
                nearest_side=min((abs(x-self.bounds[k][2]) for k in neighbor_zones),
                                 default=100000)
                left_boundary=self.bounds[z][2]
                right_boundary=self.bounds[z][3]
                at_split=(abs(x-pz[1])<=clearance+1 and
                          (abs(x-left_boundary)<=clearance+2 or
                           abs(x-right_boundary)<=clearance+2))
                edge_near_portal=(sum(1 for p in raw["boundary"]
                        if abs(p[1]-pz[1])<=clearance) /
                        max(1,len(raw["boundary"])))
                if at_split and (h>=2*w or edge_near_portal>=.55):
                    continue
            detected.append(dict(id=i+1,center=(float(y),float(x)),
                                 zone=int(z),height_cells=int(h),
                                 width_cells=int(w),area=raw["component_area"],
                                 boundary=[tuple(p) for p in raw["boundary"]]))
        return detected

    def corners(self,z):
        if z in self._corners:return self._corners[z]
        y0,y1,x0,x1=self.bounds[z]
        k=self.cfg.inset_cells
        goals={}
        mask=(self.zone==z)&self.pred_safe
        yy,xx=np.nonzero(mask)
        if not len(yy):return {}
        available=np.column_stack([yy,xx])
        # Look for a deep corner with footprint clearance, not a mathematical
        # wall intersection. Snapping is to predicted-safe FREE, not GT.
        for rowname,ty in (("top",y0+k),("bottom",y1-k)):
            for colname,tx in (("left",x0+k),("right",x1-k)):
                # Keep candidates near the corresponding *zone corner*.
                # A nonrectangular chamber may have its nearest corner on
                # an inset bend; pick the nearest safe physically possible.
                dr=available[:,0]-ty;dc=available[:,1]-tx
                cost=dr*dr+dc*dc
                idx=int(np.argmin(cost))
                target=tuple(map(int,available[idx]))
                if self.pred_paths.path(self.robot,target) is not None:
                    goals[rowname+"_"+colname]=target
        self._corners[z]=goals
        return goals

    def _portal_target(self):
        if self.portal is None:return None
        # Portal core is the entry view; snap to native-verified FREE if
        # reasonably possible, else retain as hypothetical intent.
        p=self.portal
        if self.geo.predfree[p] and self.pred_paths.path(self.robot,p):
            return p
        return None

    def _opposite(self,point,center,arrival):
        # A strictly geometric behind-the-object criterion. A goal near
        # the entry side of an obstacle cannot satisfy the rear obligation.
        cy,cx=center
        inward=(cy-arrival[0],cx-arrival[1])
        return (point[0]-cy)*inward[0]+(point[1]-cx)*inward[1]

    def _rear_corners(self,z,objects,arrival):
        """Far-side corners as viewpoints for occlusion obligations.

        Choosing opposite depth is a constraint, not a coverage-weight bonus.
        The entrance can be the robot for current chamber or a G1 portal
        for an adjacent chamber. The preference for deeper far wall is a
        geometric tie-break, and resolves left/right under mirror reflection.
        """
        options=self.corners(z)
        if not options or not objects:return []
        center=(sum(o["center"][0] for o in objects)/len(objects),
                sum(o["center"][1] for o in objects)/len(objects))
        candidates=[]
        for label,point in options.items():
            opposite=self._opposite(point,center,arrival)
            if opposite<=0:continue
            # Interior corners with sufficient clearance are preferred to
            # divider-side shortcuts. The direction is FROM portal toward
            # occluder; maximize depth past the occluder.
            dist_from_arrival=math.dist(point,arrival)
            corridor_avoid=math.dist(point,self.portal) if self.portal else dist_from_arrival
            score=(opposite,dist_from_arrival,corridor_avoid)
            candidates.append((score,label,point))
        # After crossing the obstacle's backside constraint, prefer
        # viewpoint DEEP inside the room, far from the separating portal.
        # Rewarding the largest dot product instead would drag D02 toward
        # the divider although the lower-left view is safer structurally.
        candidates.sort(key=lambda x:(-x[0][2],-x[0][1],-x[0][0],x[1]))
        # Keep two candidates on opposite transverse sides when they exist.
        if not candidates:return []
        main=(candidates[0][1],candidates[0][2])
        selected=[main]
        for score,label,point in candidates[1:]:
            # Distinct corners, preferably separated along the axis
            # perpendicular to the entry->obstacle bearing.
            va=(center[0]-arrival[0],center[1]-arrival[1])
            cp=lambda p:va[1]*(p[0]-center[0])-va[0]*(p[1]-center[1])
            if cp(point)*cp(main[1])<0:
                selected.append((label,point));break
        return selected

    def _unknown_half(self,z,location):
        """Current UNKNOWN count near the corresponding structural half."""
        yy,xx=np.nonzero(self.zone==z)
        if not len(yy):return 0
        center=np.median(yy)
        mask=(self.zone==z)&self.geo.unknown
        cy=location[0]
        if cy>center:return int((mask&(np.indices(mask.shape)[0]>center)).sum())
        return int((mask&(np.indices(mask.shape)[0]<=center)).sum())

    def obligations(self, mission_phase="INITIAL",completed=None):
        completed=set(completed or [])
        tasks=[]
        current_zone=self.entry_zone
        others=[z for z in self.zoneids if z!=current_zone]
        # Expected empty adjacent predicted room: a portal overview
        # obligation. It is not scored as an INFO-GAIN frontier.
        if mission_phase=="INITIAL" and self.portal and others and (
            sum(self._zone_unknown[z] for z in others)>0):
            if "PORTAL_SURVEY" not in completed:
                tasks.append(dict(id="PORTAL_SURVEY",purpose="SURVEY_PREDICTED_CHAMBER",
                     target=self._portal_target(),zone=others[0],
                     reason="Chamber boundary/doorway gives a first survey line of sight"))
        # One rear-of-occluder mission for the entry chamber; the farther
        # opposing zone corner is a *geometric obligation*, not an IG maximum.
        if mission_phase=="INITIAL":
            objs=[o for o in self.objects if o["zone"]==current_zone]
            if objs:
                arrival=self.entry_anchor
                opposite=self._rear_corners(current_zone,objs,arrival)
                # Choose one far corner for this compact entry-room obstacle.
                if opposite:
                    label,point=opposite[0]
                    if "ENTRY_REAR" not in completed:
                        tasks.append(dict(id="ENTRY_REAR",purpose="VIEW_ENTRY_OBJECT_REAR",
                                          target=point,zone=current_zone,corner=label,
                                          object_ids=[o["id"] for o in objs],
                                          reason="Beyond occluder relative to incoming LiDAR side"))
        # After unexpected obstacles appear in another chamber, form
        # complementary far-side missions. Two opposite transverse corners
        # are used ONLY when geometrically distinct and both sides retain
        # unseen portions. Their *order* is determined by current UNKNOWN
        # evidence, rather than fixed top/bottom preference.
        for z in others:
            objs=[o for o in self.objects if o["zone"]==z]
            if not objs:continue
            arrival=self.portal if self.portal is not None else self.robot
            opposite=self._rear_corners(z,objs,arrival)
            if not opposite:continue
            if mission_phase=="INITIAL":
                # Defer new structural obligations until actual evidence of
                # new obstacles. At D02, LaMa predicts no object in room2.
                continue
            ranking=[]
            for label,point in opposite:
                rank=self._unknown_half(z,point)
                ranking.append((rank,label,point))
            ranking.sort(key=lambda v:(-v[0],v[1]))
            # Evidence-based *necessity* test for additional far-side view:
            # if travel to the next flank predicts no new hidden cells OR
            # obstacle faces relative to an already chosen flank, skip it.
            # This is not a per-goal reward or an environment-tuned score.
            seen_on_previous_route=None
            prev_goal=arrival
            sight=Sight(self.geo,R7Settings(resolution_m=self.cfg.resolution_m))
            for rank,label,point in ranking:
                segment=self.pred_paths.path(prev_goal,point)
                if segment is None:continue
                projected=sight.route(segment)
                if seen_on_previous_route is not None:
                    extra_unknown=(projected[0]&~seen_on_previous_route[0]).bit_count()
                    extra_faces=(projected[1]&~seen_on_previous_route[1]).bit_count()
                    if extra_unknown==0 and extra_faces==0:
                        continue
                if seen_on_previous_route is None:
                    seen_on_previous_route=projected
                else:
                    seen_on_previous_route=tuple(
                        seen_on_previous_route[j]|projected[j] for j in range(3))
                prev_goal=point
                taskid=f"ZONE_{z}_{label}_REAR"
                if taskid in completed:continue
                tasks.append(dict(id=taskid,purpose="VIEW_NEW_OCCLUDER_OPPOSING_FLANK",
                                  target=point,zone=z,corner=label,
                                  unknown_in_corner_half=rank,
                                  object_ids=[o["id"] for o in objs],
                                  reason="View hidden surfaces from opposite deep chamber corners"))
        return tasks

    def plan(self, mission_phase="INITIAL", completed=None):
        t=time.perf_counter()
        tasks=self.obligations(mission_phase,completed)
        current=self.robot;legs=[];native_current=self.robot
        for task in tasks:
            target=task.get("target")
            if target is None:continue
            target=tuple(target)
            virtual_route=self.pred_paths.path(current,target)
            if virtual_route is None:continue
            native_route=(self.native_paths.path(native_current,target)
                          if self.native_safe[native_current] and self.native_safe[target]
                          else None)
            task=dict(task)
            task["goal_rc"]=list(target)
            task["predicted_path_m"]=round((len(virtual_route)-1)*self.cfg.resolution_m,3)
            task["goal_observed_free"]=bool(self.obs[target]==0)
            task["native_known_free_path_exists"]=bool(native_route is not None)
            task["native_known_free_path_m"]=(round((len(native_route)-1)*
                         self.cfg.resolution_m,3) if native_route is not None else None)
            task["native_inflated_astar_required"]=True
            task["movement_authorized"]=False
            task["virtual_predicted_path_rc"]=[list(p) for p in virtual_route]
            legs.append(task)
            current=target;native_current=target
        return dict(version="G2_R7_R3_STRUCTURAL_OBLIGATIONS",
                    status="ADVISORY_ONLY_NOT_NATIVE_ROBOT_COMMAND",
                    phase=mission_phase,
                    robot_rc=list(self.robot),
                    zones=len(self.zoneids),
                    zone_bounds={str(z):list(b) for z,b in self.bounds.items()},
                    portal=[list(p) for p in self.geo.portal_candidates],
                    objects=[{k:v for k,v in obj.items() if k!="boundary"}
                             for obj in self.objects],
                    goals=[q["goal_rc"] for q in legs],
                    tasks=legs,
                    D02_coordinates_hardcoded=False,
                    uses_gt=False,uses_future_lidar=False,
                    native_control_unmodified=True,
                    planning_s=round(time.perf_counter()-t,4),
                    input_obs_sha256=checksum(self.obs),
                    input_mean_sha256=checksum(self.mean))
