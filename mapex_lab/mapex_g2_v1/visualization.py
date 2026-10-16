"""Optional G2-V1 plan debug visualization, built exclusively from current maps."""
from pathlib import Path

import numpy as np
import cv2
from PIL import Image, ImageDraw, ImageFont

from .planner import G2Snapshot, G2Config, ProbabilisticVisibility, _room


def make_coverage(snapshot:G2Snapshot, plan_result:dict, config:G2Config):
    room=_room(snapshot)
    vis=ProbabilisticVisibility(snapshot,room,config,beams=config.final_beams,ray_step_cells=config.final_ray_step_cells)
    existing=vis.zero.copy()
    stages=[]
    for leg in plan_result['route_segments']:
        path=[tuple(p) for p in leg['path_rc']]
        cov,_=vis.project_leg(path)
        delta=cov & ~existing
        existing |= cov
        blend=np.dot(vis.scenario_prob,delta.astype(np.float32))
        raster=np.zeros(snapshot.observed.shape,np.float32)
        raster[vis.target]=blend
        stages.append(raster)
    total=np.zeros(snapshot.observed.shape,np.float32)
    total[vis.target]=np.dot(vis.scenario_prob,existing.astype(np.float32))
    return room,total,stages


def save_visualization(snapshot:G2Snapshot, result:dict, config:G2Config, png_path,
                       coverage_npz_path=None, zoom=3):
    room,coverage,stages=make_coverage(snapshot,result,config)
    obs=snapshot.observed
    rgb=np.full((*obs.shape,3),242,np.uint8)
    rgb[obs==0]=(252,252,252)
    rgb[obs==1]=(177,188,203)
    rgb[obs==2]=(48,59,71)
    rgb[~room]=(236,238,241)
    rgb[(~room)&(obs==2)]=(118,125,134)
    target=(obs==1)&room
    frac=np.clip(coverage,0,1)
    # Predicted region color coding: high projected visibility in teal;
    # otherwise unknown gray. This is not ground truth or simulated scan.
    alpha=frac[target][:,None]
    base=rgb[target].astype(float)
    teal=np.array([34,166,124],float)
    rgb[target]=np.clip(base*(1-alpha)+teal*alpha,0,255).astype(np.uint8)
    contours,_=cv2.findContours(room.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(rgb,contours,-1,(181,74,174),1)
    image=Image.fromarray(rgb).resize((obs.shape[1]*zoom,obs.shape[0]*zoom),Image.Resampling.NEAREST)
    drawer=ImageDraw.Draw(image)
    colors=['#05895f','#bd832b','#3277aa','#874a9f','#996739']
    def pos(y,x):return ((x+.5)*zoom,(y+.5)*zoom)
    for n,leg in enumerate(result['route_segments']):
        color=colors[n%len(colors)]
        drawer.line([pos(int(y),int(x)) for y,x in leg['path_rc']],fill=color,width=max(2,zoom))
        y,x=leg['goal_rc'];p=pos(y,x)
        drawer.ellipse((p[0]-5,p[1]-5,p[0]+5,p[1]+5),fill=color,outline='white',width=2)
        drawer.text((p[0]+8,p[1]-8),str(n+1),fill=color,stroke_fill='white',stroke_width=2)
    robot=pos(*snapshot.robot_rc)
    drawer.ellipse((robot[0]-6,robot[1]-6,robot[0]+6,robot[1]+6),fill='#182e42',outline='white',width=2)
    drawer.text((robot[0]+7,robot[1]-7),'R',fill='#182e42',stroke_fill='white',stroke_width=2)
    dest=Path(png_path);dest.parent.mkdir(parents=True,exist_ok=True);image.save(dest)
    if coverage_npz_path:
        out=Path(coverage_npz_path);out.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(out,current_observed=obs,current_mean=snapshot.mean,
                            current_variance=snapshot.variance,current_semantic=snapshot.semantic,
                            robot_rc=np.array(snapshot.robot_rc,dtype=np.int32),room_mask=room,
                            projected_total_visibility=coverage,
                            projected_leg_visibility=np.stack(stages) if stages else np.zeros((0,*obs.shape),np.float32))
    return str(dest)
