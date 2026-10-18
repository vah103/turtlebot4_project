"""G2 R6.2 R3 frontier-witness residual local candidate extension.
Causal only: frontier witnesses derive from LiDAR observed UNKNOWN and G1 ROOM.
All travel uses currently observed native-reachable FREE. No GT/future data.
No arbitrary minimum new cells, local completion is certified ONLY by G1 WFD.
"""
from dataclasses import dataclass
import numpy as np
import cv2
from g2_r6_watchman import RoomWatchman

CROSS = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))

def _pts(mask):
    return [tuple(map(int,x)) for x in np.argwhere(mask)]

def _bresenham(a,b):
    y0,x0=a; y1,x1=b
    dy=abs(y1-y0); dx=abs(x1-x0)
    sy=1 if y0<y1 else -1; sx=1 if x0<x1 else -1
    err=dx-dy
    while True:
        yield y0,x0
        if (y0,x0)==(y1,x1): break
        e2=2*err
        if e2>-dy: err-=dy;x0+=sx
        if e2<dx: err+=dx;y0+=sy

def _line_to_frontier(obs,origin,frontier,max_range_cells):
    if (origin[0]-frontier[0])**2+(origin[1]-frontier[1])**2 > max_range_cells**2:
        return False
    points=list(_bresenham(origin,frontier))
    return all(obs[p]==0 for p in points[:-1]) and obs[frontier]==1

def connected_frontier(obs,room,legal):
    # Frontier cells are observed UNKNOWN next to native-reachable observed FREE.
    # The independent G1 WFD remains the OFFICIAL room exit condition.
    adj=(cv2.dilate(legal.astype('uint8'),CROSS)>0)
    mask=adj & (obs==1) & room
    n,labels=cv2.connectedComponents(mask.astype('uint8'),connectivity=8)
    groups=tuple(tuple(_pts(labels==k)) for k in range(1,n))
    return mask,groups

def _path_frontiers(r, path, targets):
    """Conservative line-of-sight to real frontier witnesses from path scan poses."""
    if not path or not targets:return frozenset()
    limit=r.cfg.lidar_range_m/r.cfg.resolution_m
    stride=r.cfg.scan_stride_cells
    poses=path[::stride]
    if poses[-1]!=path[-1]:poses=poses+[path[-1]]
    # Candidates cannot claim coverage through already unobserved cells.
    visible=set()
    for pose in poses:
        for target in targets:
            if target not in visible and _line_to_frontier(r.obs,pose,target,limit):
                visible.add(target)
    return frozenset(visible)

@dataclass
class R3Choice:
    goal: tuple | None
    route: tuple
    source: str
    first_route_m: float
    frontier_covered: int
    frontier_total: int
    witness_rc: tuple
    base_goal: tuple | None
    base_projected_nominal: int
    selected_projected_nominal: int
    selected_projected_stress: int
    candidate_frontier_count: int
    suggested_goal_chain: tuple

def choose(observed,mean,variance,semantic,robot,native_walkable,precomputed=None):
    r=precomputed if precomputed is not None else RoomWatchman(
        observed,mean,variance,semantic,robot,native_walkable)
    original=r.last_decision if r.last_decision is not None else r.plan()
    base_goal=tuple(original['goal_sequence_core_rc'][0]) if original['goal_sequence_core_rc'] else None
    frontier,groups=connected_frontier(r.obs,r.room,r.legal)
    fpoints=_pts(frontier)
    base_path=r.paths.path(robot,base_goal) if base_goal is not None else None
    base_cover=_path_frontiers(r,base_path,fpoints) if base_path else frozenset()
    base_pred=r.rays.route(base_path) if base_path else (np.zeros_like(r.room),np.zeros_like(r.room))
    base_nom=int(base_pred[0].sum())
    # KULICH-STYLE: create ONE representative per uncovered frontier component,
    # but choose it from *actual* reachable observed FREE cells adjacent to frontier.
    # No fixed max candidate count. This is a residual method, not an exhaustive
    # visibility-region solution; G1 WFD is authoritative on missed frontiers.
    proposals=[]
    frontier_candidate_cnt=0
    for group in groups:
        segment=np.zeros_like(frontier)
        yy,xx=zip(*group)
        segment[yy,xx]=True
        adjacent=(cv2.dilate(segment.astype('uint8'),CROSS)>0)&r.legal
        cells=_pts(adjacent)
        frontier_candidate_cnt+=len(cells)
        best=None
        for cell in cells:
            path=r.paths.path(robot,cell)
            if not path or len(path)<=1:continue
            item=(len(path)-1,cell,path)
            if best is None or item[:2]<best[:2]:best=item
        if best is not None:
            _,goal,path=best
            covered=_path_frontiers(r,path,fpoints)
            projected=r.rays.route(path)
            proposals.append((goal,path,covered,projected))
    selected=None
    if proposals:
        # Choose target resolving most currently unrepresented REAL frontier
        # per safe metre, not a speculative gain-per-metre cutoff.
        def sort_key(item):
            goal,path,cover,projected=item
            return (-len(cover)/max(1,len(path)-1),-len(cover),
                    len(path)-1,goal)
        selected=min(proposals,key=sort_key)
    source='R62_ORIGINAL'
    goal=base_goal
    path=base_path
    cover=base_cover
    pred=base_pred
    if selected:
        g,p,c,v=selected
        target_nom=int(v[0].sum())
        target_stress=int(v[1].sum())
        # Frontier coverage priority when base leg misses real frontier entirely.
        # Otherwise prefer a shorter option with strictly stronger count-wise
        # real frontier progress and no smaller nominal predicted visibility.
        # This is NOT exact set dominance: uncovered targets remain for G1/WFD
        # and subsequent R3 replanning after an arrival or mid-route checkpoint.
        base_misses=(not base_cover and c)
        target_dominates=(len(c)>len(base_cover)
                     and target_nom>=base_nom
                     and len(p)<=len(base_path) if base_path else True)
        if base_misses or target_dominates:
            goal,path,cover,pred=g,p,c,v
            source='FRONTIER_RESIDUAL'
    if goal is None:
        source='DEFER_TO_G1_WFD'
        path=[]
        cover=frozenset()
        pred=(np.zeros_like(r.room),np.zeros_like(r.room))
    witness=tuple(sorted(cover)) if source=='FRONTIER_RESIDUAL' else ()
    if source=='FRONTIER_RESIDUAL':
        # Do not commit the original multi-goal chain: after frontier resolution
        # perform a fresh G1/native-validated checkpoint. No forced 5-goal route.
        chain=(goal,)
    else:chain=tuple(tuple(x) for x in original['goal_sequence_core_rc'])
    return R3Choice(goal,tuple(path or ()),source,
            round((len(path)-1)*r.cfg.resolution_m,4) if path else 0.,
            len(cover),len(fpoints),witness,base_goal,base_nom,
            int(pred[0].sum()),int(pred[1].sum()),
            frontier_candidate_cnt,chain)

def cheap_progress(observed, witness_rc):
    """No future observation: output only facts about originally assigned targets."""
    if not witness_rc:return {'action':'KEEP','witness_count':0}
    unresolved=[p for p in witness_rc if observed[tuple(p)]==1]
    return {'action':'CHECK_G1_FRONTIER_NOW' if not unresolved else 'KEEP',
            'witness_count':len(witness_rc),'unresolved_witness_count':len(unresolved)}
