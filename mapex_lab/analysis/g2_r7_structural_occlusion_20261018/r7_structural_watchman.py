"""G2-R7 research prototype: structural occlusion-aware pathwise Watchman.

R4: value the structural regions/surfaces across the *whole path*.
R6.2: greedy incremental room coverage, native known-FREE candidate paths.
R7: explicitly reward predicted object surfaces not already visible on approach,
    produce non-executable virtual viewpoint forecasts, and prepare successors.

Inputs: current-time observed/LaMa mean+variance/RoomNode/native FREE only.
No GT, no future snapshots, no robot actuation, no physical safety claims.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import time
import numpy as np
import cv2
from g2_r6_watchman import RoomWatchman, R6Config, _KnownFreePaths, _PredictedSight, sha


@dataclass(frozen=True)
class R7Config:
    resolution_m: float = 0.10
    scan_stride_cells: int = 8
    max_candidates: int = 24        # bounded compute budget, not learned New Room geometry
    max_goals: int = 5              # bounded proposal, not a mandatory goal count
    max_surface_samples: int = 120
    max_ray_cells: int = 200        # 20m at 0.1m resolution
    pose_clearance_cells: int = 2  # require native inflated FREE mask independently
    speculative_max: int = 4
    nominal_fraction: float = 0.7
    # Occluder weight is a proportion of unresolved UNKNOWN, not a per-map count threshold.
    structural_fraction: float = 0.20


def _bresenham_clear(a, b, blockers):
    """Sight to blocker surface b; cells before b must not be occluders."""
    y0,x0 = map(int,a)
    y1,x1 = map(int,b)
    dy=abs(y1-y0);dx=abs(x1-x0)
    sy=1 if y0<y1 else -1
    sx=1 if x0<x1 else -1
    err=dx-dy
    while (y0,x0)!=(y1,x1):
        if blockers[y0,x0]:
            return False
        twice=2*err
        if twice>-dy:
            err-=dy;x0+=sx
        if twice<dx:
            err+=dx;y0+=sy
    return True


class SurfaceEvidence:
    """Sparse predicted/observed occluder-boundary samples; causal heuristic.

    A sample is credited once per chain, whether first seen on the route or at
    the endpoint. This encourages complementary viewing directions without
    hardcoding 'go to room corner'.
    """
    def __init__(self, obs, mean, room, cfg: R7Config, semantic=None):
        self.cfg=cfg
        self.obs=obs
        self.room=room
        # G1's ROOM label (1) excludes OBSTACLE (3) and walls (4).
        # Looking only INSIDE semantic ROOM misses the very occluders that
        # the planner must observe from the opposite side.
        near_room=cv2.dilate(room.astype(np.uint8),
                            np.ones((7,7),np.uint8)).astype(bool)
        structural=np.zeros_like(room,bool) if semantic is None else (semantic==3)
        occupied=(obs==2)|((obs==1)&(mean>=.5))|structural
        self.blockers=occupied|(~near_room)
        solids=near_room & occupied
        n,lab,stats,centroids=cv2.connectedComponentsWithStats(
            solids.astype(np.uint8),connectivity=8)
        self.objects=[]
        samples=[]
        # Keep a bounded, spatially balanced contour sample per object.
        for oid in range(1,n):
            mask=(lab==oid).astype(np.uint8)
            edge=(mask!=0)&(cv2.erode(mask,np.ones((3,3),np.uint8))==0)
            pts=np.argwhere(edge)
            if not len(pts):
                continue
            self.objects.append((int(oid),tuple(float(v) for v in centroids[oid][::-1])))
            samples.append((oid,pts))
        capacity=max(1,cfg.max_surface_samples)
        total=sum(len(pts) for _,pts in samples)
        # Proportionally allocate sample count, then drop excess deterministically.
        points=[]
        for oid,pts in samples:
            take=max(1,round(capacity*len(pts)/max(1,total)))
            ix=np.unique(np.linspace(0,len(pts)-1,min(len(pts),take),dtype=int))
            points.extend([(oid,tuple(map(int,pts[i]))) for i in ix])
        if len(points)>capacity:
            ix=np.linspace(0,len(points)-1,capacity,dtype=int)
            points=[points[i] for i in ix]
        self.samples=points
        self.cache={}
        self.route_cache={}

    def at(self, pose):
        pose=tuple(map(int,pose))
        if pose in self.cache:return self.cache[pose]
        h,w=self.obs.shape
        if not(0<=pose[0]<h and 0<=pose[1]<w):
            raise ValueError("surface viewpoint outside map")
        bits=0
        range2=self.cfg.max_ray_cells**2
        for i,(_,p) in enumerate(self.samples):
            if (p[0]-pose[0])**2+(p[1]-pose[1])**2 >range2:continue
            if _bresenham_clear(pose,p,self.blockers):
                bits|=1<<i
        self.cache[pose]=bits
        return bits

    def along(self,path,stride):
        if not path:return 0
        key=(tuple(path[0]),tuple(path[-1]),len(path))
        if key in self.route_cache:return self.route_cache[key]
        chosen=list(path[::stride])
        if chosen[-1]!=path[-1]:chosen.append(path[-1])
        bits=0
        for p in chosen:bits|=self.at(p)
        self.route_cache[key]=bits
        return bits


class R7Planner(RoomWatchman):
    """R6.2-compatible safe local planner with R4-inspired face complement."""
    def __init__(self, observed, mean, variance, semantic, robot,
                 native_walkable, cfg:R7Config|None=None):
        self.r7=cfg or R7Config()
        base=R6Config(resolution_m=self.r7.resolution_m,
                      scan_stride_cells=self.r7.scan_stride_cells,
                      max_candidates=self.r7.max_candidates,
                      max_goals=self.r7.max_goals,
                      nominal_weight=self.r7.nominal_fraction)
        super().__init__(observed,mean,variance,semantic,robot,native_walkable,base)
        self.faces=SurfaceEvidence(self.obs,self.mean,self.room,self.r7,self.semantic)
        self.surface_scale=(self.r7.structural_fraction*int(self.mission.sum())/
                            max(1,self.r7.max_surface_samples))
        self._last_route=None

    def candidates(self):
        original=super().candidates()
        legal_pts=np.argwhere(self.legal)
        if not len(legal_pts):return original
        # Candidates sampled on distinct sides of each predicted occluder.
        # The goal is visibility diversity, not proximity to an arbitrary corner.
        added=[]
        for _,center in self.faces.objects:
            cy,cx=center
            dy=legal_pts[:,0]-cy
            dx=legal_pts[:,1]-cx
            angle=np.arctan2(dy,dx)
            # Eight angular sectors: general in rooms, corridors and hospitals.
            sector=np.floor((angle+np.pi)/(2*np.pi)*8).astype(int)%8
            for k in range(8):
                which=np.flatnonzero(sector==k)
                if len(which)==0:continue
                # Prefer deep viewpoints on that side with a long baseline,
                # while limiting extreme distance and excessive detours.
                dist=np.hypot(dy[which],dx[which])
                travel=np.hypot(legal_pts[which,0]-self.robot[0],
                                legal_pts[which,1]-self.robot[1])
                value=dist/(1+0.25*travel)
                j=which[int(np.argmax(value))]
                point=tuple(map(int,legal_pts[j]))
                if point!=self.robot and point not in added:
                    added.append(point)
        # Pre-rank added corners using new surface direction evidence before
        # applying a compute budget; reserve candidate diversity for G2.
        initial=self.faces.at(self.robot)
        ranked=[]
        for p in added:
            gain=(self.faces.at(p)&~initial).bit_count()
            ranked.append((gain,p))
        ranked.sort(key=lambda x:(-x[0],x[1]))
        mixed=list(dict.fromkeys([p for _,p in ranked[:self.r7.max_candidates//2]]+original))
        return mixed[:self.r7.max_candidates]

    def _leg_cover(self,seg):
        return self._route_cover(seg), self.faces.along(seg,self.r7.scan_stride_cells)

    def _score_route(self,goals,start=None,initial_covered=None):
        current=self.robot if start is None else tuple(start)
        cov_nom=np.zeros(self.obs.shape,bool)
        cov_stress=cov_nom.copy()
        surf=0
        if initial_covered is not None:
            cov_nom,cov_stress,surf=(initial_covered[0].copy(),
                                  initial_covered[1].copy(),int(initial_covered[2]))
        total_distance=0.
        legs=[]
        fullpath=[]
        for p in goals:
            seg=self.paths.path(current,p)
            if seg is None:return None
            vis,sbits=self._leg_cover(seg)
            new=(vis[0]&~cov_nom,vis[1]&~cov_stress)
            face_new=(sbits&~surf).bit_count()
            novel=self._weighted(new)+face_new*self.surface_scale
            cov_nom|=vis[0];cov_stress|=vis[1];surf|=sbits
            distance=(len(seg)-1)*self.cfg.resolution_m
            total_distance+=distance
            legs.append(dict(goal_rc=list(p),path_m=round(distance,3),
                             new_unknown_value=round(self._weighted(new),2),
                             new_surfaces=face_new,combined_gain=round(novel,2)))
            fullpath.extend(seg if not fullpath else seg[1:])
            current=p
        return dict(nominal=cov_nom,stress=cov_stress,surfaces=surf,
                    gain=self._weighted((cov_nom,cov_stress))+
                          surf.bit_count()*self.surface_scale,
                    distance_m=total_distance,legs=legs,path=fullpath)

    def plan(self,assumed_before=None):
        """Plan on this snapshot only; assumed_before = current-time predicted
        coverage of an *already committed* preceding leg, never future LiDAR.
        """
        started=time.perf_counter()
        current=self.robot
        points=self.candidates()
        # Clone cached ray masks: mutating shared cache masks would make
        # hypothetical future observations look as if they occurred already.
        robot_sight=self.rays.at(current)
        seen_nom,seen_stress=robot_sight[0].copy(),robot_sight[1].copy()
        seen_surfaces=self.faces.at(current)
        if assumed_before is not None:
            seen_nom|=assumed_before[0]
            seen_stress|=assumed_before[1]
            seen_surfaces|=int(assumed_before[2])
        seen=(seen_nom.copy(),seen_stress.copy(),seen_surfaces)
        covered_nom,covered_stress,covered_surfs=(seen_nom.copy(),
                                                  seen_stress.copy(),seen_surfaces)
        chosen=[];trace=[]
        for _ in range(self.r7.max_goals):
            options=[]
            for p in points:
                if p in chosen:continue
                seg=self.paths.path(current,p)
                if seg is None or len(seg)<=1:continue
                (a,b),surface=self._leg_cover(seg)
                new=(a&~covered_nom,b&~covered_stress)
                novel_unknown=self._weighted(new)
                novel_face=(surface&~covered_surfs).bit_count()
                gain=novel_unknown+self.surface_scale*novel_face
                length=(len(seg)-1)*self.cfg.resolution_m
                if gain<=0:continue
                options.append((gain/(length+.8),gain,-length,p,a,b,surface,
                                novel_unknown,novel_face))
            if not options:break
            best=max(options,key=lambda v:(v[0],v[1],v[2],tuple(-k for k in v[3])))
            _,gain,neg_len,p,a,b,surface,nu,nf=best
            chosen.append(p)
            trace.append(dict(goal_rc=list(p),gain=round(gain,2),
                              unknown_gain=round(nu,2),new_faces=nf,
                              leg_m=round(-neg_len,3)))
            covered_nom|=a;covered_stress|=b;covered_surfs|=surface
            current=p
        # Improve order only if combined coverage is unchanged and distance decreases.
        score=self._score_route(chosen,self.robot,seen)
        if score is None:raise RuntimeError("R7 route invalid")
        for _ in range(5):
            improvement=None
            for i in range(len(chosen)-1):
                for j in range(i+1,len(chosen)):
                    alt=chosen[:i]+list(reversed(chosen[i:j+1]))+chosen[j+1:]
                    other=self._score_route(alt,self.robot,seen)
                    if other is not None and other['distance_m']<score['distance_m']-0.099 and (
                        (other['nominal']|score['nominal']).sum() == other['nominal'].sum() and
                        (other['stress']|score['stress']).sum() == other['stress'].sum() and
                        (score['surfaces'] & ~other['surfaces'])==0):
                        key=(other['distance_m'],alt)
                        if improvement is None or key<improvement[0]:
                            improvement=(key,alt,other)
            if improvement is None:break
            _,chosen,score=improvement
        # Prune goals which do not add sight/structure value after earlier legs.
        # This is a whole-path test, so the later route can replace a stop.
        changed=True
        while changed and chosen:
            changed=False
            for i in range(len(chosen)-1,-1,-1):
                alt=chosen[:i]+chosen[i+1:]
                other=self._score_route(alt,self.robot,seen)
                if other is not None and other['distance_m']<=score['distance_m'] and (
                    (score['nominal'] & ~other['nominal']).sum()==0 and
                    (score['stress'] & ~other['stress']).sum()==0 and
                    (score['surfaces'] & ~other['surfaces'])==0):
                    chosen,score=alt,other
                    changed=True
                    break
        self.plan_order=list(chosen)
        self._last_route=score
        result=dict(version="G2_R7_STRUCTURAL_OCCLUSION_R1",
                    action="LOCAL_VIEWPOINT_PROPOSAL" if chosen else "RESIDUAL_ROOM_WFD",
                    goal_sequence_core_rc=[list(p) for p in chosen],
                    legs=score['legs'],
                    path_core_rc=[list(p) for p in score['path']],
                    new_structural_surface_samples=int((score['surfaces']&~seen[2]).bit_count()),
                    surface_samples_total=len(self.faces.samples),
                    unknown_nominal_new=int((score['nominal']&~seen[0]).sum()),
                    unknown_stress_new=int((score['stress']&~seen[1]).sum()),
                    predicted_distance_m=round(score['distance_m'],3),
                    candidate_count=len(points),
                    compute_s=round(time.perf_counter()-started,4),
                    input_observed_sha256=sha(self.obs),
                    prediction_sha256=sha(self.mean),
                    all_goals_observed_free=all(bool(self.legal[p]) for p in chosen),
                    planner_used_gt=False,planner_used_future_observations=False,
                    native_astar_required_at_dispatch=True,
                    no_actual_coverage_claim=True)
        self.last_decision=result
        return result

    def predicted_future_candidates(self):
        """Speculative goals; never published as native-safe/commandable.
        This supports reasoning about behind-wall viewpoints before LiDAR clears
        the route. The native runner must plan only to VERIFIED FREE segments.
        """
        potential=self.room&((self.obs==0)|((self.obs==1)&(self.mean<.5)))
        if not potential[self.robot]:return []
        n,comp=cv2.connectedComponents(potential.astype(np.uint8),connectivity=4)
        mask=(comp==comp[self.robot])
        future=np.argwhere(mask&(self.obs==1))
        if len(future)==0:return []
        stride=max(1,int(math.sqrt(len(future)/max(1,3*self.r7.speculative_max))))
        before=self.faces.at(self.robot)
        q=[]
        for p in future[::stride]:
            p=tuple(map(int,p))
            face=(self.faces.at(p)&~before).bit_count()
            nom,stress=self.rays.at(p)
            gain=self._weighted((nom,stress))+face*self.surface_scale
            if gain>0:q.append((gain,p,face))
        q.sort(key=lambda x:(-x[0],x[1]))
        result=[]
        for gain,p,faces in q:
            if any(math.dist(p,x['goal_rc'])<self.cfg.scan_stride_cells for x in result):
                continue
            result.append(dict(goal_rc=list(p),reason="PREDICTED_FREE_UNCONFIRMED",
                               projected_gain=round(gain,2),surfaces_new=faces,
                               executable=False,requires_lidar_confirmation=True))
            if len(result)>=self.r7.speculative_max:break
        return result
