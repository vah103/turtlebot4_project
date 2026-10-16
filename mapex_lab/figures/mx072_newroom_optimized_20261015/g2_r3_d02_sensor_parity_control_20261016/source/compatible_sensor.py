#!/usr/bin/env python3
"""Isolated hidden-GT sensor for the independent G2-R3 offline controller.

Only THIS subprocess loads GT. It never chooses viewpoints or returns GT.
One request = observed-free route of current plan; synthesizes scans every 0.5m.
"""
import sys,json,traceback
from pathlib import Path
import numpy as np
from PIL import Image
SROOT=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
GT=np.asarray(Image.open(SROOT/'gt_map.png'))[500:703,500:763,0]==255
assert GT.shape==(203,263)
theta=np.linspace(0,2*np.pi,2500,endpoint=True,dtype=np.float64)[:,None]
ray_range=np.arange(801,dtype=np.float64)[None,:]*.25
ridx=np.arange(801)[None,:]
H,W=GT.shape

def sensor(p):
    y,x=p
    yy=np.rint(y+np.cos(theta)*ray_range).astype(np.int32)
    xx=np.rint(x+np.sin(theta)*ray_range).astype(np.int32)
    ok=(yy>=0)&(xx>=0)&(yy<H)&(xx<W)
    cy=np.clip(yy,0,H-1);cx=np.clip(xx,0,W-1)
    hit=np.where(~ok|GT[cy,cx],ridx,801).min(axis=1)
    free_ray=(ridx<hit[:,None])&ok
    free=np.zeros((H,W),dtype=bool)
    occ=np.zeros_like(free)
    free[cy[free_ray],cx[free_ray]]=True
    hi=np.flatnonzero(hit<801)
    if len(hi):
        ry=cy[hi,hit[hi]];rx=cx[hi,hit[hi]]
        valid=ok[hi,hit[hi]]&GT[ry,rx]
        occ[ry[valid],rx[valid]]=True
    free&=~occ
    return free,occ

def handle(j):
    src=Path(j['observed_source']);dst=Path(j['observed_destination'])
    if dst.exists():raise RuntimeError('EXISTS '+str(dst))
    with np.load(src,allow_pickle=False) as z:observed=z['observed'].copy()
    route=[tuple(int(i) for i in p) for p in j['route_rc']]
    if not route:raise ValueError('route missing')
    if observed.shape!=GT.shape:raise ValueError('grid shape mismatch')
    if any(observed[p]!=0 for p in route):raise ValueError('planned route not fully observed-free')
    if any((abs(a[0]-b[0])+abs(a[1]-b[1]))!=1 for a,b in zip(route,route[1:])):
        raise ValueError('route not contiguous')
    # This is a simulator safety check, NOT used to select a next pose or viewpoint:
    if any(GT[p] for p in route):
        return {'event':'ROUTE_COLLISION_WITH_HIDDEN_ENV','status':'ABORTED','route_cells':len(route)}
    scan_positions=[route[i] for i in range(5,len(route),5)]
    if not scan_positions or scan_positions[-1]!=route[-1]:
        scan_positions.append(route[-1])
    stageold=observed.copy();per_scan=[]
    for p in scan_positions:
        free,wall=sensor(p)
        was_unknown=(observed==1)
        seenfree=was_unknown&free
        seenwall=was_unknown&wall
        observed[seenfree]=0;observed[seenwall]=2
        per_scan.append({'robot_rc':list(p),'fresh_free':int(seenfree.sum()),'fresh_wall':int(seenwall.sum())})
    np.savez_compressed(dst,observed=observed)
    return {'event':'SENSING_OK','scan_poses':len(scan_positions),
        'distance_m':round((len(route)-1)*.1,3),
        'end_rc':list(route[-1]),
        'fresh_free_global':int(((stageold==1)&(observed==0)).sum()),
        'fresh_wall_global':int(((stageold==1)&(observed==2)).sum()),
        'scan_evidence':per_scan,
        'output':str(dst)}

print(json.dumps({'event':'SENSOR_READY','approximation':'GT raster first-hit 2500 rays 20m at .25 cell, every .5m'}),flush=True)
for line in sys.stdin:
    try:
        req=json.loads(line)
        if req.get('action')=='CLOSE':
            print(json.dumps({'event':'SENSOR_BYE'}),flush=True);break
        if req.get('action')!='SCAN_ROUTE':raise ValueError('unknown action')
        print(json.dumps(handle(req)),flush=True)
    except Exception as ex:
        print(json.dumps({'event':'SENSOR_ERROR','error':repr(ex),'traceback':traceback.format_exc()}),flush=True)
        break
