"""G2-R3 groundwork: geometric room obligations and durable mission memory.

This is NOT a multi-goal path optimizer and NOT a robot-completion oracle.
No access to GT, future observations or prior hand-picked goal positions.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from collections import deque
from pathlib import Path
from math import ceil,pi
from typing import Any
import hashlib,json
import numpy as np
import cv2

@dataclass(frozen=True)
class StructuralConfig:
    resolution_m:float=.1
    visibility_range_m:float=20.
    beams:int=250
    ray_step_cells:float=.5
    grid_step_cells:int=4
    max_vantages:int=180
    clearance_cells:int=2
    scan_stride_cells:int=5
    # Threshold partitions predicted geometry, not certified probabilities.
    occupancy_cutoff:float=.5

def sha(a):
    return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()

def _validate(obs,mean,variance,semantic,robot):
    if obs.ndim!=2 or any(a.shape!=obs.shape for a in [mean,variance,semantic]):
        raise ValueError('maps need same 2D shape')
    if not np.isin(obs,[0,1,2]).all():
        raise ValueError('observed codes must be 0 free,1 unknown,2 occupied')
    if not (np.isfinite(mean).all() and np.isfinite(variance).all()):
        raise ValueError('invalid mean/variance')
    if np.min(mean)<0 or np.max(mean)>1 or np.min(variance)<0:
        raise ValueError('invalid prediction range')
    y,x=robot
    if not (0<=y<obs.shape[0] and 0<=x<obs.shape[1] and obs[y,x]==0):
        raise ValueError('robot must be in observed free space')

def accepted_component(obs,semantic,robot):
    mask=((semantic==1)&(obs!=2)).astype(np.uint8)
    _,lab=cv2.connectedComponents(mask,connectivity=4)
    id_=lab[robot]
    return (lab==id_) if id_ else np.zeros(obs.shape,bool)

def components(mask):
    n,label,stats,centroid=cv2.connectedComponentsWithStats(mask.astype(np.uint8),connectivity=8)
    return [(i,label==i,int(stats[i,cv2.CC_STAT_AREA]),(int(stats[i,cv2.CC_STAT_TOP]),int(stats[i,cv2.CC_STAT_LEFT]),
        int(stats[i,cv2.CC_STAT_TOP]+stats[i,cv2.CC_STAT_HEIGHT]-1),
        int(stats[i,cv2.CC_STAT_LEFT]+stats[i,cv2.CC_STAT_WIDTH]-1)),[float(centroid[i,0]),float(centroid[i,1])]) for i in range(1,n)]

def _navigable(obs,room,robot,cfg):
    radius=cfg.clearance_cells
    knownfree=obs==0
    inflated=cv2.dilate((~knownfree).astype(np.uint8),np.ones((2*radius+1,2*radius+1),np.uint8))!=0 if radius else ~knownfree
    valid=room&knownfree&~inflated
    valid[robot]=True
    return valid

def _bfs(robot,free):
    que=deque([robot]);prv={robot:None};dist={robot:0}
    ny,nx=free.shape
    while que:
        y,x=que.popleft()
        for nxt in [(y+1,x),(y-1,x),(y,x+1),(y,x-1)]:
            if 0<=nxt[0]<ny and 0<=nxt[1]<nx and free[nxt] and nxt not in prv:
                prv[nxt]=(y,x);dist[nxt]=dist[(y,x)]+1;que.append(nxt)
    return prv,dist

def _route(parent,start,dest):
    current=dest;points=[dest]
    while current!=start:
        current=parent[current];points.append(current)
    return points[::-1]

class MissionMemory:
    """Never silently drop historically accepted-but-unobserved room cells.

    Region labels are persistent via maximum-pixel overlap; split regions get
    a new id for extra fragments; no id may be assigned twice in one update.
    """
    def __init__(self,initial,mission,previous_observed,labels=None,next_id=1,history=None):
        self.initial=initial.astype(bool).copy()
        self.mission=mission.astype(bool).copy()
        self.previous_observed=previous_observed.copy()
        self.labels=np.zeros(initial.shape,np.int32) if labels is None else labels.astype(np.int32).copy()
        self.next_id=int(next_id)
        self.history=list(history or [])
        if any(a.shape!=initial.shape for a in (self.mission,self.previous_observed,self.labels)):
            raise ValueError('memory shape mismatch')

    @classmethod
    def begin(cls,obs,semantic,robot):
        current=accepted_component(obs,semantic,robot)
        if not current.any():raise ValueError('robot not in accepted Room Node')
        return cls(current,current,obs)

    def preserve_and_check(self,obs,current):
        if obs.shape!=self.mission.shape or current.shape!=self.mission.shape:
            raise ValueError('memory-map shape mismatch')
        if np.any((self.previous_observed!=1)&(obs!=self.previous_observed)):
            raise ValueError('observed evidence cannot revert or change labels')
        old=self.mission.copy()
        if current.any():
            # If current room disconnected from mission, retain old without
            # silently switching the target mission to an unrelated room.
            if not np.any(current&self.mission):
                raise ValueError('new accepted room disconnected from mission; new mission required')
            self.mission|=current
        return int((self.mission&~old).sum())

    def assign_labels(self,regions):
        old=self.labels.copy()
        fresh=np.zeros(old.shape,np.int32)
        used=set()
        ordering=sorted(regions,key=lambda r:(-r['cells'],r['bbox_rc']))
        for row in ordering:
            indices=row.pop('_pixels')
            ov=old[indices]
            values,counts=np.unique(ov[ov>0],return_counts=True)
            pairs=sorted(zip(counts.tolist(),values.tolist()),reverse=True)
            match=next((label for n,label in pairs if label not in used),None)
            if match is None:match=self.next_id;self.next_id+=1
            used.add(match)
            fresh[indices]=match
            row['region_id']=int(match)
        self.labels=fresh
        return sorted(regions,key=lambda x:x['region_id'])

    def record(self,obs,current,report):
        self.previous_observed=obs.copy()
        self.history.append({
            'decision_id':report['decision_id'],
            'original_room_unknown':report['original_room_unknown'],
            'mission_unknown':report['mission_unknown'],
            'current_room_unknown':report['current_room_unknown'],
            'unknown_outside_current_room':report['unknown_outside_current_room'],
            'region_count':len(report['regions']),
            'snapshot_sha256':report['snapshot_sha256']
        })

    def save(self,path):
        payload=json.dumps({'format':'G2_R3_MISSION_V1','history':self.history,'next_id':self.next_id})
        np.savez_compressed(path,original_room=self.initial,mission=self.mission,
            previous_observed=self.previous_observed,labels=self.labels,
            metadata=np.asarray(payload))

    @classmethod
    def load(cls,path):
        with np.load(path,allow_pickle=False) as f:
            data=json.loads(str(f['metadata']))
            if data['format']!='G2_R3_MISSION_V1':raise ValueError('unknown mission format')
            return cls(f['original_room'],f['mission'],f['previous_observed'],
                labels=f['labels'],next_id=data['next_id'],history=data['history'])

def visibility_grid(obs,mean,mission,vantage,cfg,cache=None,block_predicted=True):
    key=(vantage,block_predicted)
    if cache is not None and key in cache:return cache[key]
    y,x=vantage
    lengths=np.arange(0,cfg.visibility_range_m/cfg.resolution_m+1e-9,cfg.ray_step_cells,dtype=np.float32)
    theta=np.linspace(0,2*pi,cfg.beams,endpoint=False,dtype=np.float32)[:,None]
    yy=np.rint(y+np.cos(theta)*lengths[None,:]).astype(np.int32)
    xx=np.rint(x+np.sin(theta)*lengths[None,:]).astype(np.int32)
    height,width=obs.shape
    valid=(yy>=0)&(xx>=0)&(yy<height)&(xx<width)
    yy=np.clip(yy,0,height-1);xx=np.clip(xx,0,width-1)
    blocks=(obs==2)|~mission
    if block_predicted:blocks|=((obs==1)&(mean>=cfg.occupancy_cutoff))
    stop=blocks[yy,xx]|~valid
    seen=(~np.maximum.accumulate(stop,axis=1))&valid
    # Predicted walls may be first-hit observations; not traversable.
    first=np.where(stop,np.arange(stop.shape[1])[None,:],stop.shape[1]).min(axis=1)
    first_hit=(np.arange(stop.shape[1])[None,:]==first[:,None])&valid
    mask=np.zeros(obs.shape,bool);mask[yy[seen|first_hit],xx[seen|first_hit]]=True
    if cache is not None:cache[key]=mask
    return mask

def analyze(obs,mean,variance,semantic,robot,memory,cfg=None,decision_id='current',include_vantage=True):
    cfg=cfg or StructuralConfig()
    _validate(obs,mean,variance,semantic,robot)
    current=accepted_component(obs,semantic,robot)
    newly_added=memory.preserve_and_check(obs,current)
    envelope=memory.mission
    unknown=envelope&(obs==1)
    predicted_open=unknown&(mean<cfg.occupancy_cutoff)
    predicted_wall=unknown&~predicted_open
    bins=[
        ('POSSIBLE_OPEN_SPACE',predicted_open),
        ('UNVERIFIED_PREDICTED_OBSTACLE',predicted_wall)]
    rows=[]
    for kind,mask in bins:
        for _,pixels,count,bbox,center in components(mask):
            ms=mean[pixels]
            var=variance[pixels]
            entropy=np.zeros_like(ms,float);ok=(ms>1e-8)&(ms<1-1e-8)
            entropy[ok]=-(ms[ok]*np.log2(ms[ok])+(1-ms[ok])*np.log2(1-ms[ok]))
            unknown_inside=int((pixels&current).sum())
            row={'kind':kind,'cells':count,'bbox_rc':list(bbox),'center_xy_m':[round(center[0]*cfg.resolution_m,2),round(center[1]*cfg.resolution_m,2)],
                'expected_open_cell_mass_proxy':round(float((1-ms).sum()),3),
                'mean_entropy_proxy':round(float(entropy.mean()),4),
                'mean_variance':round(float(var.mean()),5),
                'currently_accepted_cells':unknown_inside,
                'historically_retained_outside_current':count-unknown_inside,
                'lifecycle':'UNOBSERVED_UNRESOLVED',
                '_pixels':pixels}
            rows.append(row)
    rows=memory.assign_labels(rows)
    # Explanation of *why* a region remains hidden from the CURRENT robot pose.
    # All values are current-map ray geometry, never GT or a future scan.
    nominal_robot=visibility_grid(obs,mean,envelope,robot,cfg)
    without_unknown_blocks=visibility_grid(obs,mean,envelope,robot,cfg,block_predicted=False)
    for region in rows:
        pixels=(memory.labels==region['region_id'])
        nominal=int((pixels&nominal_robot).sum())
        projected_shadow=int((pixels&without_unknown_blocks&~nominal_robot).sum())
        observed_shadow=int((pixels&~without_unknown_blocks).sum())
        region['current_robot_nominal_line_of_sight_cells']=nominal
        region['occluded_by_predicted_structure_proxy_cells']=projected_shadow
        region['occluded_by_observed_structure_or_geometry_proxy_cells']=observed_shadow
        # Account for all cells exactly; "observed_structure_or_geometry" can
        # include finite sensor range/ray discretization, not just physical walls.
        assert nominal+projected_shadow+observed_shadow==region['cells']
    nav=_navigable(obs,current,robot,cfg) if current.any() else np.zeros(obs.shape,bool)
    parent,dists=_bfs(robot,nav)
    candidates=[p for p in parent if p!=robot and p[0]%cfg.grid_step_cells==robot[0]%cfg.grid_step_cells and p[1]%cfg.grid_step_cells==robot[1]%cfg.grid_step_cells]
    candidates=sorted(candidates)
    if len(candidates)>cfg.max_vantages:
        idx=np.linspace(0,len(candidates)-1,cfg.max_vantages,dtype=int)
        candidates=[candidates[i] for i in idx]
    targets=np.zeros(obs.shape,np.int32)
    for region in rows:targets[memory.labels==region['region_id']]=region['region_id']
    cache={}
    region_views={int(r['region_id']):[] for r in rows}
    # Pathwise candidate diagnostics. This is NOT a route optimizer:
    # it shows which unresolved structures have an observable witness.
    for v in candidates:
        path=_route(parent,robot,v)
        sample=[path[i] for i in range(cfg.scan_stride_cells,len(path),cfg.scan_stride_cells)]
        if not sample or sample[-1]!=v:sample.append(v)
        mask=np.zeros(obs.shape,bool)
        for p in sample:mask|=visibility_grid(obs,mean,envelope,p,cfg,cache)
        samples=targets[mask&unknown]
        nonzero=samples[samples>0]
        if nonzero.size==0:continue
        ids,counts=np.unique(nonzero,return_counts=True)
        for rid,n in zip(ids.tolist(),counts.tolist()):
            region_views[rid].append((n,dists[v],v))
    for region in rows:
        rid=region['region_id'];views=region_views[rid]
        views.sort(key=lambda q:(-q[0],q[1],q[2]))
        ranked=views[:3]
        region['candidate_vantage_count']=len(views)
        region['top_conditional_viewpoints']=[{
            'core_rc':list(q[2]),
            'xy_m':[round(q[2][1]*cfg.resolution_m,2),round(q[2][0]*cfg.resolution_m,2)],
            'route_distance_m':round(q[1]*cfg.resolution_m,2),
            'predicted_visible_region_cells_along_route':q[0],
            'is_valid_goal':True,
            'not_a_final_G2_route_selection':True} for q in ranked] if include_vantage else []
        if region['historically_retained_outside_current']>0:
            region['attention']='RETAINED_AFTER_ROOM_RECLASSIFICATION'
        elif not views:region['attention']='NO_KNOWNFREE_PATHWISE_VIEW_IN_SAMPLE'
        elif region['kind']=='POSSIBLE_OPEN_SPACE':region['attention']='POTENTIAL_FREE_SPACE_TO_CHECK'
        else:region['attention']='VERIFY_PREDICTED_WALL_NOT_AUTO_CLEARED'
        region['planning_evidence']='NO_FUTURE_OR_GT: approximate raycast on current mean; goals are candidate witnesses only'
    original_unknown=int((memory.initial&(obs==1)).sum())
    mission_unknown=int(unknown.sum())
    current_unknown=int(((obs==1)&current).sum())
    status='OBSERVED_MISSION_NO_UNKNOWN' if mission_unknown==0 else ('STRUCTURE_READY_FOR_G2' if any(q['candidate_vantage_count']>0 for q in rows) else 'UNRESOLVED_NO_KNOWNFREE_VIEW')
    report={'version':'G2_R3_STRUCTURAL_MEMORY_V1','decision_id':decision_id,'robot_core_rc':list(robot),
        'snapshot_sha256':sha(obs),'observed_sha256':sha(obs),'mean_sha256':sha(mean),'variance_sha256':sha(variance),'semantic_sha256':sha(semantic),
        'input_used_gt':False,'planning_is_current_snapshot_only':True,
        'original_room_cells':int(memory.initial.sum()),'mission_envelope_cells':int(envelope.sum()),
        'new_mission_cells_since_last_snapshot':newly_added,
        'mission_unknown':mission_unknown,'original_room_unknown':original_unknown,
        'current_room_cells':int(current.sum()),'current_room_unknown':current_unknown,
        'unknown_outside_current_room':int((unknown&~current).sum()),
        'predicted_free_like_unknown':int(predicted_open.sum()),
        'unverified_predicted_wall_unknown':int(predicted_wall.sum()),
        'currently_safe_viewpoint_candidates':len(candidates),
        'status':status,'regions':rows,
        'cautions':['This is structural evidence; NOT a replacement sequence optimizer',
          'LaMa occupancy mean is not a calibrated probability of physical free space',
          'Do not convert predicted walls or former Room Node cells into observed evidence',
          'Candidate viewpoint rays are approximate and do not certify robot collision safety',
          'Historical mission regions remain unresolved until actually observed or independently adjudicated without GT']}
    memory.record(obs,current,report)
    return report,memory.labels.copy(),current.copy()
