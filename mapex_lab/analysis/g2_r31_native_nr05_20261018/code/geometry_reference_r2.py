"""G2-R7 R2 STANDALONE: prediction-driven structural coverage obligations.

No import from R3/R4/R5/R6/R6.2. No GT, future scan, annotated D02 goal or
native movement command. Predicted FREE supports INTENT routes only; the
certified observed-FREE mask and native A* remain execution authority.
The planner enumerates alternatives to cover chamber and object-face
obligations and optimizes WHOLE PLAN, not greedy waypoint reward.

Prototype for offline 2D tests, NOT a certified native runtime.
"""
from __future__ import annotations
from collections import deque, defaultdict
from dataclasses import dataclass
import hashlib
import heapq
import math
import time

import cv2
import numpy as np


@dataclass(frozen=True)
class R7Settings:
    resolution_m: float = .1
    lidar_range_m: float = 20.0
    robot_radius_m: float = .20
    sampled_cell_stride: int = 3  # compute resolution of obligation sampling
    path_scan_m: float = .5
    max_candidates: int = 24      # search CPU budget, NOT required goal count
    beam_width: int = 12
    max_plan_depth: int = 5       # search CPU budget, NOT a 5-goal commitment
    planned_goal_cost: float = .025
    normalized_distance_cost: float = .06
    zone_weight: float = .58
    object_face_weight: float = .42


def checksum(arr):
    return hashlib.sha256(np.ascontiguousarray(arr).tobytes()).hexdigest()


def cc(mask, connectivity=4):
    return cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity)


def locs_on_grid(mask, stride):
    y,x=np.nonzero(mask)
    sub=(y%stride==0)&(x%stride==0)
    return np.stack([y[sub],x[sub]],axis=1).astype(np.int32)


class GridPaths:
    """Cache BFS on the predicted-free graph; every output is hypothetical."""
    def __init__(self,legal):
        self.legal=np.asarray(legal,bool)
        self.h,self.w=self.legal.shape
        self.cache={}
    def _bfs(self,start):
        start=tuple(start)
        h,w=self.h,self.w
        parents=np.full(h*w,-2,np.int32)
        if not self.legal[start]:return parents
        s=start[0]*w+start[1]
        parents[s]=-1
        q=deque([s])
        flat=self.legal.ravel()
        while q:
            p=q.popleft();y,x=divmod(p,w)
            for nxt in ((p-w if y else -1),
                        (p+w if y<h-1 else -1),
                        (p-1 if x else -1),
                        (p+1 if x<w-1 else -1)):
                if nxt>=0 and flat[nxt] and parents[nxt]==-2:
                    parents[nxt]=p;q.append(nxt)
        return parents
    def path(self,source,target):
        source=tuple(source);target=tuple(target)
        if source not in self.cache:self.cache[source]=self._bfs(source)
        a=self.cache[source];index=target[0]*self.w+target[1]
        if index<0 or index>=a.size or a[index]==-2:return None
        pts=[]
        while index>=0:
            pts.append((int(index//self.w),int(index%self.w)))
            index=int(a[index])
        pts.reverse()
        return pts


class RoomStructure:
    """Infer chambers at the FIRST intrinsic width bottleneck, not D02 XY."""
    def __init__(self,obs,mean,semantic,robot,virtual_anchor=False):
        self.obs=np.asarray(obs,np.uint8)
        self.mean=np.asarray(mean,np.float32)
        self.semantic=np.asarray(semantic,np.uint8)
        self.robot=tuple(map(int,robot))
        if not (self.obs.shape==self.mean.shape==self.semantic.shape):
            raise ValueError("inconsistent map shape")
        if self.semantic[self.robot]!=1:
            raise ValueError("robot must be inside an accepted G1 room")
        if self.obs[self.robot]!=0 and not (
            virtual_anchor and self.obs[self.robot]==1 and self.mean[self.robot]<.5):
            raise ValueError("virtual anchor requires predicted FREE; actual robot requires LiDAR FREE")
        if not np.isin(self.obs,(0,1,2)).all():raise ValueError("bad occupancy values")
        candidate=(self.semantic==1)&(self.obs!=2)
        _,label,_,_=cc(candidate)
        key=label[self.robot]
        if key<=0:raise ValueError("no connected local room")
        self.room=label==key
        self.predfree=self.room & ((self.obs==0)|((self.obs==1)&(self.mean<.5)))
        self.unknown=self.room&(self.obs==1)
        self.edt=cv2.distanceTransform(self.predfree.astype('uint8'),cv2.DIST_L2,5)
        self.zones=self._find_zones()
        self.zone_ids=sorted(int(v) for v in np.unique(self.zones) if v>0)
        self.portal_candidates=self._portals()
        self.obstacles=self._observed_occluders()

    def _find_zones(self):
        """Scale-space erosion: first clear split of connected travel space.

        Erode by increments of one map cell until at least two substantial
        surviving cores appear. A substantial core covers >= 1% of the room
        and >= 1 robot-footprint area. No New Room coordinates or room counts.
        """
        if not self.predfree[self.robot]:return np.where(self.room,1,0).astype(np.int32)
        area=int(self.predfree.sum())
        # The splitting scale below comes from geometry; a conservative
        # one-cell footprint lower bound avoids map-specific dimensions.
        robot_cells=max(1,int(round(math.pi*2**2)))
        require=max(robot_cells,round(.01*area))
        selected=None;best_radius=None
        for radius in range(1,int(self.edt.max())+1):
            core=self.edt>radius
            n,l,st,_=cc(core)
            idx=[i for i in range(1,n) if st[i,cv2.CC_STAT_AREA]>=require]
            if len(idx)>=2:
                selected=[l==i for i in idx];best_radius=radius
                break
        self.neck_radius_cells=best_radius
        if not selected:
            ret=np.zeros(self.obs.shape,np.int32)
            ret[self.predfree]=1
            return ret
        # Multi-source geodesic flood fill in predicted FREE. Preserves
        # distinct chambers across narrow portals rather than Voronoi lines.
        labels=np.zeros(self.obs.shape,np.int32)
        queue=deque()
        for k,seed in enumerate(selected,1):
            labels[seed]=k
            for row,col in np.argwhere(seed):
                queue.append((int(row),int(col)))
        h,w=labels.shape
        while queue:
            y,x=queue.popleft()
            for yy,xx in ((y-1,x),(y+1,x),(y,x-1),(y,x+1)):
                if 0<=yy<h and 0<=xx<w and self.predfree[yy,xx] and labels[yy,xx]==0:
                    labels[yy,xx]=labels[y,x];queue.append((yy,xx))
        return labels

    def _portals(self):
        """Boundary between inferred chambers = portal candidate clusters."""
        z=self.zones
        edges=np.zeros(z.shape,bool)
        for dy,dx in ((0,1),(1,0)):
            shifted=np.roll(z,(-dy,-dx),axis=(0,1))
            valid=(z>0)&(shifted>0)&(z!=shifted)
            if dx:valid[:,-1]=False
            if dy:valid[-1,:]=False
            edges|=valid
        n,l,st,_=cc(edges,8)
        candidates=[]
        for i in range(1,n):
            pts=np.argwhere(l==i)
            if not len(pts):continue
            # Center of portal with greatest wall clearance.
            scores=self.edt[pts[:,0],pts[:,1]]
            best=pts[int(np.argmax(scores))]
            candidates.append(tuple(map(int,best)))
        return candidates

    def _observed_occluders(self):
        """Obstacle components adjacent to room; filter long exterior walls.

        Count a component as an interior occluder when its boundary has
        accepted-room on opposing sides, or when it has G1 OBSTACLE label.
        Objects derived solely from information available at this time.
        """
        near=cv2.dilate(self.room.astype('uint8'),np.ones((7,7),np.uint8)).astype(bool)
        solids=near & ((self.obs==2)|(self.semantic==3))
        n,l,st,cen=cc(solids,8)
        result=[]
        ry,rx=np.nonzero(self.room)
        outer=(int(ry.min()),int(ry.max()),int(rx.min()),int(rx.max()))
        for idx in range(1,n):
            cells=st[idx,cv2.CC_STAT_AREA]
            if cells<3:continue
            y,x=np.nonzero(l==idx)
            cy,cx=cen[idx][1],cen[idx][0]
            # External enclosure components can form a single WALL ring
            # with a centroid *inside* the room. BBox support must be checked
            # before centroid-based interior-object classification.
            span_horizontal=(x.min()<=outer[2] and x.max()>=outer[3])
            span_vertical=(y.min()<=outer[0] and y.max()>=outer[1])
            if span_horizontal or span_vertical:
                continue
            if (abs(cy-outer[0])<=2 or abs(cy-outer[1])<=2 or
                abs(cx-outer[2])<=2 or abs(cx-outer[3])<=2):
                # External enclosure is not an internal object whose back
                # faces must be explored.
                continue
            # Evidence of room on either side, not only the outer perimeter.
            pad=4
            top=max(0,int(y.min())-pad);bottom=min(self.obs.shape[0],int(y.max())+pad+1)
            left=max(0,int(x.min())-pad);right=min(self.obs.shape[1],int(x.max())+pad+1)
            surrounds=self.predfree[top:bottom,left:right]
            horizontal=(self.predfree[y, np.clip(x-4,0,self.obs.shape[1]-1)].any() and
                        self.predfree[y, np.clip(x+4,0,self.obs.shape[1]-1)].any())
            vertical=(self.predfree[np.clip(y-4,0,self.obs.shape[0]-1),x].any() and
                      self.predfree[np.clip(y+4,0,self.obs.shape[0]-1),x].any())
            if not horizontal and not vertical and not (self.semantic[y,x]==3).any():
                continue
            mask=l==idx
            edge=mask & (cv2.erode(mask.astype(np.uint8),
                                 np.ones((3,3),np.uint8))==0)
            outline=np.argwhere(edge)
            # Order by angle around center, sample distributed around silhouette.
            angles=np.arctan2(outline[:,0]-cy,outline[:,1]-cx)
            outline=outline[np.argsort(angles)]
            count=min(40,len(outline))
            outline=outline[np.unique(np.linspace(0,len(outline)-1,count,dtype=int))]
            result.append({"center":(float(cy),float(cx)),
                           "boundary":[tuple(map(int,p)) for p in outline],
                           "component_area":int(cells)})
        return result


class Sight:
    """Line of sight to unknown samples and obstacle faces, from a pose."""
    def __init__(self,geo:RoomStructure,cfg:R7Settings):
        self.geo=geo;self.cfg=cfg
        self.targets=locs_on_grid(geo.unknown,cfg.sampled_cell_stride)
        self.target_zones=geo.zones[self.targets[:,0],self.targets[:,1]] if len(self.targets) else np.empty(0,int)
        self.face_targets=[]
        self.face_groups=[]
        for k,obj in enumerate(geo.obstacles,1):
            for pos in obj["boundary"]:
                self.face_targets.append(pos)
                self.face_groups.append(k)
        self.face_targets=np.asarray(self.face_targets,dtype=np.int32).reshape(-1,2)
        self.face_groups=np.asarray(self.face_groups,dtype=np.int32)
        self.nom_block=(geo.obs==2)|((geo.obs==1)&(geo.mean>=.5))|(~geo.room)
        self.cache={}
        self.zone_target_bits={}
        for z in geo.zone_ids:
            bits=0
            for i in np.flatnonzero(self.target_zones==z):bits|=1<<int(i)
            self.zone_target_bits[z]=bits
        self.face_bits={}
        for k in range(1,len(geo.obstacles)+1):
            bits=0
            for i in np.flatnonzero(self.face_groups==k):bits|=1<<int(i)
            self.face_bits[k]=bits
        self.angle_sectors=12

    def _see(self,pose,points,faces=False):
        if len(points)==0:return 0
        py,px=map(int,pose)
        dr=points[:,0]-py;dc=points[:,1]-px
        steps=np.maximum(np.abs(dr),np.abs(dc))
        maxlen=self.cfg.lidar_range_m/self.cfg.resolution_m
        candidates=np.flatnonzero(steps<=maxlen)
        if len(candidates)==0:return 0
        rem=np.zeros(len(points),bool)
        rem[candidates]=True
        # Vectorized straight-ray stepping. Endpoints may be blockers when
        # sampling object surface; only intermediate cells occlude faces.
        for j in range(1,int(steps[candidates].max(initial=0))+1):
            sel=np.flatnonzero(rem & (steps>=j))
            if not len(sel):continue
            den=steps[sel]
            rr=py+np.rint(dr[sel]*j/den).astype(int)
            cc=px+np.rint(dc[sel]*j/den).astype(int)
            if faces:
                at_endpoint=j==den
                blocked=self.nom_block[rr,cc]&(~at_endpoint)
            else:
                # A target UNKNOWN is visible, as long as its own map cell
                # isn't already predicted to be occupied (via 0.5 cutoff).
                blocked=self.nom_block[rr,cc] & (j<den)
            rem[sel[blocked]]=False
        out=0
        for i in np.flatnonzero(rem):out|=1<<int(i)
        return out

    def at(self,pose):
        pose=tuple(map(int,pose))
        if pose not in self.cache:
            known=self._see(pose,self.face_targets,True)
            angles=0
            for i,obj in enumerate(self.geo.obstacles):
                if not (known & self.face_bits[i+1]):
                    continue  # Cannot credit a side without a LOS to object
                cy,cx=obj["center"]
                theta=math.atan2(pose[0]-cy,pose[1]-cx)
                sector=int(math.floor((theta+math.pi)/(2*math.pi)*
                                      self.angle_sectors))%self.angle_sectors
                angles|=1<<(i*self.angle_sectors+sector)
            self.cache[pose]=(self._see(pose,self.targets,False),known,angles)
        return self.cache[pose]

    def route(self,path):
        if not path:return 0,0
        stride=max(1,round(self.cfg.path_scan_m/self.cfg.resolution_m))
        pts=list(path[::stride])
        if pts[-1]!=path[-1]:pts.append(path[-1])
        t=f=angles=0
        for pt in pts:
            a,b,c=self.at(pt)
            t|=a;f|=b;angles|=c
        return t,f,angles

    def fractions(self,seen):
        zone={}
        for z,bits in self.zone_target_bits.items():
            zone[z]=(seen[0]&bits).bit_count()/max(1,bits.bit_count())
        faces={}
        for k,bits in self.face_bits.items():
            faces[k]=(seen[1]&bits).bit_count()/max(1,bits.bit_count())
        return zone,faces

    def angle_fractions(self,seen):
        result={}
        for i,obj in enumerate(self.geo.obstacles):
            observed=[k for k in range(self.angle_sectors)
                      if (seen[2]>>(i*self.angle_sectors+k))&1]
            if len(observed)<2:
                value=0.
            else:
                # The best two bearings should approach opposite sides
                # (180 degrees). More scans from one side cannot fake that.
                separations=[min((a-b)%self.angle_sectors,(b-a)%self.angle_sectors)
                             for a in observed for b in observed if a!=b]
                value=max(separations)/(self.angle_sectors/2)
            result[i+1]=float(value)
        return result

    def obligation_quality(self,seen):
        zone,faces=self.fractions(seen)
        angles=self.angle_fractions(seen)
        # Equal structural chamber coverage obligations, regardless of area.
        vz=sum(zone.values())/len(zone) if zone else 1.
        vf=sum(faces.values())/len(faces) if faces else 1.
        va=sum(angles.values())/len(angles) if angles else 1.
        if not faces:return vz
        # Occluder completion values *opposing viewing direction*, not
        # simply counting already visible tiny wall segments.
        return self.cfg.zone_weight*vz + self.cfg.object_face_weight*(.2*vf+.8*va)


class R7Standalone:
    def __init__(self,obs,mean,variance,semantic,robot,native_reachable,
                 cfg=None,virtual_anchor=False):
        self.cfg=cfg or R7Settings()
        self.obs=np.asarray(obs,np.uint8).copy()
        self.mean=np.asarray(mean,np.float32).copy()
        self.variance=np.asarray(variance,np.float32).copy()
        self.sem=np.asarray(semantic,np.uint8).copy()
        self.robot=tuple(map(int,robot))
        self.native=np.asarray(native_reachable,bool).copy()
        self.geo=RoomStructure(self.obs,self.mean,self.sem,self.robot,
                               virtual_anchor=virtual_anchor)
        self.sight=Sight(self.geo,self.cfg)
        self.paths=GridPaths(self.geo.predfree)
        self.native_free=(self.obs==0)&self.native
        self.radius_cells=int(math.ceil(self.cfg.robot_radius_m/self.cfg.resolution_m))
        # Predicted-clearance is only advisory; actual native inflated
        # known-FREE remains the only movement authorization.
        self.pred_safe=self.geo.predfree&(self.geo.edt>=self.radius_cells)
        self.pred_safe[self.robot]=True
        self.candidate_info={}

    def _candidate(self,pt,kind,zone):
        pt=tuple(map(int,pt))
        if not self.pred_safe[pt]:return
        if self.paths.path(self.robot,pt) is None:return
        info=self.candidate_info.setdefault(pt,{"kinds":set(),"zones":set()})
        info["kinds"].add(kind)
        if zone>0:info["zones"].add(int(zone))

    def candidates(self):
        """Structural chamber cores + portal + back-side corner viewpoints."""
        candidates=self.candidate_info
        for pt in self.geo.portal_candidates:
            self._candidate(pt,"CHAMBER_PORTAL",0)
        # Chamber core with maximum predicted free clearance.
        for z in self.geo.zone_ids:
            points=np.argwhere((self.geo.zones==z)&self.pred_safe)
            if not len(points):continue
            radii=self.geo.edt[points[:,0],points[:,1]]
            self._candidate(points[int(np.argmax(radii))],"CHAMBER_CORE",z)
            center=np.mean(points,axis=0)
            scale=np.std(points,axis=0)+1
            # Extreme directional regions provide complementary viewpoint
            # candidates, notably deep corners opposite approach direction.
            for theta in np.linspace(0,2*math.pi,8,endpoint=False):
                vec=np.array([math.sin(theta),math.cos(theta)])
                proj=((points-center)/scale*vec).sum(axis=1)
                # Near-wall viewpoint penalized; corner inset is guided by
                # physical robot radius and the same native inflated mask.
                utility=proj+.1*np.minimum(radii,5)
                ix=int(np.argmax(utility))
                self._candidate(points[ix],"CHAMBER_EXTREME",z)
        # Opposing sides of interior occluders, including obstacle surfaces
        # that were previously visible only along the entry approach.
        for k,obj in enumerate(self.geo.obstacles,1):
            cy,cx=obj["center"]
            pts=np.argwhere(self.pred_safe)
            if not len(pts):continue
            dist=np.hypot(pts[:,0]-cy,pts[:,1]-cx)
            for theta in np.linspace(0,2*math.pi,8,endpoint=False):
                direction=np.array([math.sin(theta),math.cos(theta)])
                unit=(pts-np.array([cy,cx]))/np.maximum(1,dist[:,None])
                align=unit@direction
                score=align*np.minimum(dist,20)-.10*np.hypot(
                    pts[:,0]-self.robot[0],pts[:,1]-self.robot[1])
                ix=int(np.argmax(score))
                v=pts[ix]
                self._candidate(v,f"OBJECT_{k}_OPPOSING_SIDE",self.geo.zones[tuple(v)])
        # Ranking is by structural diversity and individual *obligation*
        # support, not by a single frontier information-gain score.
        seed=self.sight.at(self.robot)
        zone_weights={}
        for z in self.geo.zone_ids:
            zone_weights[z]=1/max(1,self.sight.zone_target_bits[z].bit_count())
        ranked=[]
        for pt,info in candidates.items():
            u,f,angle=self.sight.at(pt)
            area=sum((u & mask).bit_count()*weight
                     for z,mask in self.sight.zone_target_bits.items()
                     for weight in [zone_weights[z]])
            face=sum((f & mask).bit_count()/max(1,mask.bit_count())
                     for mask in self.sight.face_bits.values()) + angle.bit_count()/max(1,3*len(self.geo.obstacles))
            diversity=len(info["zones"])+len(info["kinds"])
            ranked.append((area+face+.04*diversity,pt))
        ranked.sort(key=lambda x:(-x[0],x[1]))
        # Reserve portal, chamber centers and sides of different obstacles
        # before using the bounded search budget on high-quality alternatives.
        mandatory=[]
        for kind in ("CHAMBER_PORTAL","CHAMBER_CORE"):
            for _,p in ranked:
                if kind in candidates[p]["kinds"] and p not in mandatory:
                    mandatory.append(p)
        # Preserve actual opposite-side options for EACH chamber.
        # Otherwise the candidate cap can discard the right room entirely.
        for z in self.geo.zone_ids:
            arr=np.argwhere(self.geo.zones==z)
            center=np.mean(arr,axis=0)
            ext=[p for _,p in ranked if z in candidates[p]["zones"]
                 and "CHAMBER_EXTREME" in candidates[p]["kinds"]]
            for qy,qx in ((1,1),(1,-1),(-1,1),(-1,-1)):
                eligible=[p for p in ext if
                          (p[0]-center[0])*qy>=0 and
                          (p[1]-center[1])*qx>=0]
                if eligible:
                    p=max(eligible,key=lambda p:((p[0]-center[0])*qy+
                                                 (p[1]-center[1])*qx,p))
                    if p not in mandatory:mandatory.append(p)
        for k in range(1,len(self.geo.obstacles)+1):
            tag=f"OBJECT_{k}_OPPOSING_SIDE"
            for _,p in ranked:
                if tag in candidates[p]["kinds"] and p not in mandatory:
                    mandatory.append(p);break
        rest=[p for _,p in ranked if p not in mandatory]
        shortlist=(mandatory+rest)[:self.cfg.max_candidates]
        return shortlist

    def _edge(self,a,b):
        key=(tuple(a),tuple(b))
        if not hasattr(self,"edge_cache"):self.edge_cache={}
        if key not in self.edge_cache:
            path=self.paths.path(a,b)
            if path is None:return None
            # Moving also observes: credit intermediate LiDAR poses.
            seen=self.sight.route(path)
            self.edge_cache[key]=(len(path)-1,seen,path)
        return self.edge_cache[key]

    def _state_score(self,seen,distance_m,n_goals):
        quality=self.sight.obligation_quality(seen)
        # Normalized by room diameter, not tuned to map name/room size.
        yy,xx=np.nonzero(self.geo.room)
        diagonal=math.hypot(int(yy.max()-yy.min())+1,
                             int(xx.max()-xx.min())+1)*self.cfg.resolution_m
        return quality-self.cfg.planned_goal_cost*n_goals-(
            self.cfg.normalized_distance_cost*distance_m/max(diagonal,.1))

    def plan(self):
        started=time.perf_counter()
        if not self.geo.unknown.any() and not self.geo.obstacles:
            return {"version":"G2_R7_STANDALONE_R2",
                    "status":"NO_ROOM_OBLIGATIONS",
                    "goals_rc":[],"compute_g2_s":round(time.perf_counter()-started,3),
                    "uses_GT":False,"uses_future_observation":False,
                    "native_known_free_Astar_required":True}
        points=self.candidates()
        initial=self.sight.at(self.robot)
        active=[(self._state_score(initial,0,0),(),self.robot,0.,initial)]
        best=None
        # Beam search of WHOLE coverage obligations, not greedy per-viewpoint.
        for depth in range(1,self.cfg.max_plan_depth+1):
            pool=[]
            for _,prefix,at,dist,covered in active:
                for p in points:
                    if p in prefix:continue
                    edge=self._edge(at,p)
                    if edge is None:continue
                    steps,seen,_=edge
                    novel=(covered[0]|seen[0],covered[1]|seen[1],
                           covered[2]|seen[2])
                    meter=dist+steps*self.cfg.resolution_m
                    candidate=prefix+(p,)
                    score=self._state_score(novel,meter,len(candidate))
                    pool.append((score,candidate,p,meter,novel))
                    if best is None or score>best[0]:
                        best=(score,candidate,p,meter,novel)
            if not pool:break
            pool.sort(key=lambda item:(-item[0],item[3],item[1]))
            # Deduplicate equal coverage signatures among partial hypotheses.
            next_active=[];signatures=set()
            for it in pool:
                sig=(it[4][0],it[4][1],it[4][2],it[2])
                if sig in signatures:continue
                signatures.add(sig);next_active.append(it)
                if len(next_active)>=self.cfg.beam_width:break
            active=next_active
        if best is None:
            return {"version":"G2_R7_STANDALONE_R2","status":"NO_ROOM_PROPOSAL"}
        score,goals,_,total,combined=best
        legs=[]
        at=self.robot;known_before=self.sight.at(self.robot)
        first_verified_prefix=[]
        native_paths=GridPaths(self.native_free)
        for p in goals:
            steps,view,path=self._edge(at,p)
            new=(view[0]&~known_before[0],view[1]&~known_before[1],
                 view[2]&~known_before[2])
            # Whole-path incremental structural obligations measured by
            # NEW sub-region and obstacle-face contributions.
            before_z,before_f=self.sight.fractions(known_before)
            after=(known_before[0]|view[0],known_before[1]|view[1],
                   known_before[2]|view[2])
            after_z,after_f=self.sight.fractions(after)
            reachable_goal=bool(self.obs[p]==0 and self.native[p])
            # Predicted route is not the same as native-inflated FREE path.
            # Search existing observation graph for a safe alternative.
            native_route=(native_paths.path(at,p)
                          if (self.native_free[at] and reachable_goal)
                          else None)
            native_route_ok=native_route is not None
            observed_prefix=[]
            for q in path:
                if self.obs[q]!=0 or not self.native[q]:break
                observed_prefix.append(q)
            if not legs:first_verified_prefix=(
                native_route if native_route is not None else observed_prefix)
            legs.append({
                "goal_rc":list(p),
                "predicted_free_only":not reachable_goal,
                "native_known_free_route_exists":bool(native_route_ok),
                "executable_as_is":False,  # advisory until native inflated A*
                "requires_native_astar":True,
                "native_known_free_route_m":(
                    round((len(native_route)-1)*self.cfg.resolution_m,3)
                    if native_route_ok else None),
                "route_predicted_m":round(steps*self.cfg.resolution_m,3),
                "new_unknown_samples":new[0].bit_count(),
                "new_obstacle_face_samples":new[1].bit_count(),
                "new_view_bearing_sectors":new[2].bit_count(),
                "opposing_angle_before":{str(k):round(v,3) for k,v in self.sight.angle_fractions(known_before).items()},
                "opposing_angle_after":{str(k):round(v,3) for k,v in self.sight.angle_fractions(after).items()},
                "zone_fraction_before":{str(k):round(v,3) for k,v in before_z.items()},
                "zone_fraction_after":{str(k):round(v,3) for k,v in after_z.items()},
                "face_fraction_before":{str(k):round(v,3) for k,v in before_f.items()},
                "face_fraction_after":{str(k):round(v,3) for k,v in after_f.items()},
                "candidate_reasons":sorted(self.candidate_info[p]["kinds"])
            })
            known_before=after;at=p
        zone,face=self.sight.fractions(combined)
        return {
            "version":"G2_R7_STANDALONE_R2",
            "status":"STRUCTURAL_PLAN_INTENT_NOT_ROBOT_COMMAND",
            "robot_core_rc":list(self.robot),
            "room_unknown_cells":int(self.geo.unknown.sum()),
            "geometric_chambers":len(self.geo.zone_ids),
            "neck_radius_cells":self.geo.neck_radius_cells,
            "portal_candidates_rc":[list(x) for x in self.geo.portal_candidates],
            "observed_occluder_components":len(self.geo.obstacles),
            "goals_rc":[list(x) for x in goals],
            "legs":legs,
            "predicted_plan_m":round(total,3),
            "structural_quality":round(self.sight.obligation_quality(combined),4),
            "zone_obligation_fraction":{str(k):round(v,3) for k,v in zone.items()},
            "face_obligation_fraction":{str(k):round(v,3) for k,v in face.items()},
            "opposing_angle_fraction":{str(k):round(v,3) for k,v in self.sight.angle_fractions(combined).items()},
            "score":round(score,5),
            "candidate_count":len(points),
            "compute_g2_s":round(time.perf_counter()-started,3),
            "first_goal_observed_free_prefix_rc":[list(p) for p in first_verified_prefix],
            "input_obs_sha256":checksum(self.obs),
            "mean_sha256":checksum(self.mean),
            "uses_GT":False,"uses_future_observation":False,
            "native_known_free_Astar_required":True,
            "estimated_coverage_only":True
        }
