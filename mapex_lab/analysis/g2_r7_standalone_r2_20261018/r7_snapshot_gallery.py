"""Render frozen chronological RN04 inputs with newly calculated R7-R2 plans.

D02 uses exact native-reachable mask. D03/D04 use an OBSERVED-FREE proxy;
these are NOT causal R7 rollouts or native movement certificates.
"""
from __future__ import annotations
from pathlib import Path
import base64
import html
import json
import numpy as np
from PIL import Image,ImageDraw
from r7_standalone import R7Standalone,GridPaths

OUT=Path(__file__).parent/"evidence"
OUT.mkdir(parents=True,exist_ok=True)
ROOT=Path("/home/dell/DELL_NATIVE2D/runs/NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41")
D02=Path("/home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz")


def stage(idx):
    if idx==2:
        x=np.load(D02)
        return (x["observed"],x["mean"],x["variance"],x["semantic"],
                tuple(map(int,x["robot_core_rc"])),x["native_walkable"].astype(bool),
                "EXACT_NATIVE_REACHABLE_P1")
    x=np.load(ROOT/f"inference/decision_{idx:06d}_input.npz")
    y=np.load(ROOT/f"inference/decision_{idx:06d}_semantic.npz")
    record=json.loads((ROOT/f"decisions/decision_{idx:06d}.json").read_text())
    robot=tuple(int(v)-500 for v in record["robot_rc"])
    return (x["observed"],x["mean"],x["variance"],y["semantic"],
            robot,(x["observed"]==0),
            "OFFLINE_OBSERVED_FREE_PROXY_NOT_NATIVE_CERTIFIED")


def figure(idx,inputs,planner,plan):
    obs,mean,var,sem,robot,native,scope=inputs
    H,W=obs.shape
    base=np.zeros((H,W,3),np.uint8)
    base[obs==0]=(252,249,245)
    base[obs==1]=(137,135,145)
    base[obs==2]=(42,45,54)
    for k,color in enumerate([(234,180,177),(232,203,246),(183,216,211),
                              (241,198,147)],1):
        m=(planner.geo.zones==k)&(obs!=2)
        if not m.any():continue
        base[m]=(.52*base[m]+.48*np.asarray(color)).astype(np.uint8)
    outside=(sem==2)&(obs!=2)
    base[outside]=(.53*base[outside]+.47*np.asarray([243,214,105])).astype(np.uint8)
    predicted=(obs==1)&(mean>=.5)
    base[predicted]=(70,65,82)
    img=Image.fromarray(base,"RGB").resize((W*5,H*5),resample=Image.Resampling.NEAREST)
    draw=ImageDraw.Draw(img)
    def pt(q):return (int(q[1]*5+2),int(q[0]*5+2))
    # Red outline of room (deterministic from current RoomNode map).
    import cv2
    contours,_=cv2.findContours(planner.geo.room.astype('uint8'),
                                 cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        pts=[(int(x*5+2),int(y*5+2)) for [[x,y]] in contour]
        if len(pts)>=2:draw.line(pts+[pts[0]],fill=(198,48,57),width=2)
    # Hypothetical path dashed amber, native observed-FREE alternative green.
    cur=robot
    native_graph=GridPaths(planner.native_free)
    for i,goal in enumerate(plan.get("goals_rc",[]),1):
        goal=tuple(goal)
        tentative=planner.paths.path(cur,goal)
        if tentative and len(tentative)>1:
            for j in range(0,len(tentative)-1,5):
                draw.line([pt(q) for q in tentative[j:j+3]],fill=(231,155,43),width=3)
        if planner.native_free[cur] and planner.native_free[goal]:
            safe=native_graph.path(cur,goal)
            if safe and len(safe)>1:
                draw.line([pt(q) for q in safe],fill=(24,165,95),width=4)
        x,y=pt(goal)
        draw.ellipse((x-9,y-9,x+9,y+9),fill=(247,148,20),outline=(255,255,255),width=3)
        draw.text((x+9,y-18),str(i),fill=(12,31,46),stroke_fill=(255,255,255),stroke_width=1)
        cur=goal
    for portal in planner.geo.portal_candidates:
        x,y=pt(portal)
        draw.ellipse((x-5,y-5,x+5,y+5),outline=(180,20,157),width=2)
    for object in planner.geo.obstacles:
        x,y=pt(object["center"])
        draw.ellipse((x-4,y-4,x+4,y+4),fill=(27,191,208))
    x,y=pt(robot)
    draw.ellipse((x-11,y-11,x+11,y+11),fill=(42,102,209),outline=(255,255,255),width=3)
    name=f"R7_R2_D{idx:02d}_STRUCTURAL_PLAN.png"
    img.save(OUT/name,optimize=True)
    return name


def main():
    rows=[]
    slides=[]
    for idx in (2,3,4):
        data=stage(idx)
        obs,mean,var,sem,robot,native,scope=data
        planner=R7Standalone(obs,mean,var,sem,robot,native)
        plan=planner.plan()
        filename=figure(idx,data,planner,plan)
        datum={
            "historical_RN04_decision":idx,
            "source_nav_scope":scope,
            "is_R7_closed_loop":False,
            "plan":plan,
            "image":filename,
            "stated_no_GT":True,
            "causal_same_snapshot_inputs_only":True}
        rows.append(datum)
        uri="data:image/png;base64,"+base64.b64encode((OUT/filename).read_bytes()).decode()
        legs=plan.get("legs",[])
        details="; ".join(f"G{i}: {t['goal_rc']} / {t['candidate_reasons']} / native FREE graph: {t['native_known_free_route_exists']}" for i,t in enumerate(legs,1))
        slides.append(f"<article><h2>D{idx:02d}: {len(legs)} goal — {html.escape(scope)}</h2>"
                      f"<p><b>Goals:</b> {html.escape(str(plan.get('goals_rc',[])))}. "
                      f"<b>Room regions:</b> {plan.get('geometric_chambers','?')}; "
                      f"<b>occluders:</b> {plan.get('observed_occluder_components','?')}; "
                      f"<b>G2 CPU:</b> {plan.get('compute_g2_s','?')}s</p>"
                      f"<p><small>{html.escape(details)}</small></p><img src='{uri}' alt='D{idx:02d} structural map'></article>")
        print("DONE D",idx,"GOALS",plan.get("goals_rc"),"G2",plan.get("compute_g2_s"),flush=True)
    (OUT/"R7_R2_D02_D04_SNAPSHOT_AUDIT.json").write_text(json.dumps(
        {"title":"R7 standalone R2: RN04 historical snapshot diagnostics",
         "NOT_a_closed_loop_R7_run":True,
         "no_hindsight_planner_inputs":True,
         "D03_D04_from_original_RN04_not_R7":True,
         "stages":rows},indent=2,ensure_ascii=False))
    css="""<style>body{background:#f3f5f7;font:16px system-ui;max-width:1120px;margin:auto;color:#172e40}
    header,article{background:white;margin:20px 14px;padding:20px;border-radius:14px;box-shadow:0 2px 10px #0001}
    img{width:100%;height:auto;image-rendering:pixelated}small{font-size:13px;color:#556879}
    b{font-weight:650}p{line-height:1.6}</style>"""
    html_doc=("<!doctype html><meta charset='UTF-8'>"
              "<meta name='viewport' content='width=device-width,initial-scale=1'>"
              "<title>R7 R2 Standalone · D02-D04</title>"+css+
              "<header><h1>R7 R2 Standalone · New Room</h1>"
              "<p><b>Offline only:</b> These three panels use frozen observations "
              "from the existing RN04 trajectory. They are not a closed-loop R7 "
              "run. D02 uses actual saved native reachable-FREE mask; D03-D04 "
              "use current observed-FREE proxy, not a safety certificate.</p>"
              "<p>Blue = robot; orange circles = proposed goals; magenta = predicted "
              "portal; turquoise = object center; green lines = alternative "
              "known-FREE grid paths (native A* still required); dashed amber = "
              "hypothetical paths that may cross UNKNOWN. Predicted objects "
              "and chamber divisions are based only on same-time maps.</p></header>"+
              "".join(slides))
    path=OUT/"R7_R2_D02_D04_SCROLL.html"
    path.write_text(html_doc)
    print("HTML",path,"size",path.stat().st_size,flush=True)


if __name__=="__main__":main()
