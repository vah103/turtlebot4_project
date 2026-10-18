"""D02->D03->D04 held-out-in-time *snapshot evaluation*.

All snapshots come from frozen RN04, each stage reads ONLY that stage's obs/
prediction/semantic. It is NOT an R7 closed-loop rollout; these snapshots
follow RN04's trajectory, not hypothetical G2-R7 goals.
"""
from __future__ import annotations
import base64
import hashlib
import html
import io
import json
from pathlib import Path
import numpy as np
import cv2
from PIL import Image,ImageDraw,ImageFont
from r7_structural_watchman import R7Planner

ROOT=Path("/home/dell/mapex_g2_r7_occlusion_20261018")
BASE=Path("/home/dell/DELL_NATIVE2D/runs/NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41")
D02=Path("/home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz")
out=ROOT/"evidence"
out.mkdir(exist_ok=True)


def read_snapshot(index):
    if index==2:
        a=np.load(D02)
        return {k:a[k] for k in a.files},"NATIVE_SAME_EPOCH_D02"
    a=np.load(BASE/f"inference/decision_{index:06d}_input.npz")
    b=np.load(BASE/f"inference/decision_{index:06d}_semantic.npz")
    record=json.loads((BASE/f"decisions/decision_{index:06d}.json").read_text())
    observed=a["observed"]
    # OFFLINE reachability approximation, NOT the native inflation certificate.
    native=(observed==0)
    data=dict(observed=observed,mean=a["mean"],variance=a["variance"],
              semantic=b["semantic"],native_walkable=native,
              robot_core_rc=np.array(record["robot_rc"])-500)
    return data,"OFFLINE_OBSERVED_FREE_PROXY_NOT_NATIVE_CERTIFIED"


def background(obs,sem,mean):
    rgb=np.zeros((obs.shape[0],obs.shape[1],3),dtype=np.uint8)
    rgb[obs==0]=(251,250,248)
    rgb[obs==1]=(164,164,172)
    rgb[obs==2]=(43,50,59)
    # Overlay accepted room and corridor only on observed-free or UNKNOWN;
    # transparent by blending rather than erasing obstacles.
    room=sem==1; corridor=sem==2
    rgb[room]=(0.55*rgb[room]+0.45*np.array([252,172,165])).astype(np.uint8)
    rgb[corridor]=(0.65*rgb[corridor]+0.35*np.array([240,206,91])).astype(np.uint8)
    pred=(obs==1)&(mean>=.5)
    rgb[pred]=(85,79,89)
    return rgb


def render(index,data,report,mode):
    obs=data["observed"]; sem=data["semantic"]
    robot=tuple(map(int,data["robot_core_rc"]))
    rgb=background(obs,sem,data["mean"])
    img=Image.fromarray(rgb).resize((obs.shape[1]*4,obs.shape[0]*4),resample=Image.Resampling.NEAREST)
    draw=ImageDraw.Draw(img)
    def pos(p):return ((p[1]+.5)*4,(p[0]+.5)*4)
    route=report.get("path_core_rc",[])
    if len(route)>1:draw.line([pos(x) for x in route],fill=(15,141,146),width=4)
    for idx,goal in enumerate(report["goal_sequence_core_rc"]):
        x,y=pos(goal);draw.ellipse((x-10,y-10,x+10,y+10),fill=(247,155,37),outline=(255,255,255),width=3)
        draw.text((x+7,y-18),str(idx+1),fill=(17,28,34))
    x,y=pos(robot);draw.ellipse((x-9,y-9,x+9,y+9),fill=(36,100,214),outline=(255,255,255),width=3)
    filename=f"R7_RN04_D{index:02d}_SNAPSHOT.png"
    img.save(out/filename,optimize=True)
    return filename


def main():
    rows=[]
    images=[]
    for index in (2,3,4,5):
        data,mode=read_snapshot(index)
        robot=tuple(map(int,data["robot_core_rc"]))
        rec=dict(decision=index,robot=list(robot),navigation_scope=mode)
        try:
            p=R7Planner(data["observed"],data["mean"],data["variance"],
                       data["semantic"],robot,data["native_walkable"])
            ans=p.plan()
            rec.update({k:v for k,v in ans.items() if k!="path_core_rc"})
            rec["virtual_goals_not_executable"]=p.predicted_future_candidates()
            image=render(index,data,ans,mode)
            rec["image"]=image
            images.append((index,image,mode,rec))
            print("D",index,"goals",rec["goal_sequence_core_rc"],
                  "time",rec["compute_s"],"scope",mode,flush=True)
        except Exception as exc:
            rec.update(error=repr(exc))
            print("D",index,"FAIL",repr(exc),flush=True)
        rows.append(rec)
    (out/"R7_RN04_SNAPSHOT_AUDIT.json").write_text(json.dumps(
        {"method":"G2-R7 R1", "evaluated_past_trajectory":"RN04",
         "closed_loop_R7":False,"ground_truth_not_used_for_planning":True,
         "future_observations_not_used_for_planning":True,
         "D02_native_scope_exact":True,
         "D03_D05_navigation_scope_is_offline_proxy_not_safety_certified":True,
         "stages":rows},indent=2,ensure_ascii=False))
    parts=["<!doctype html><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>",
           "<title>G2-R7 causal snapshot gallery — D02..D05</title>",
           "<style>body{font:16px system-ui;background:#f4f5f7;margin:20px auto;max-width:1080px;color:#243241}section{padding:18px;background:white;border-radius:14px;margin-bottom:20px}img{width:100%;height:auto;image-rendering:pixelated}small{color:#566}code{white-space:pre-wrap}</style>",
           "<h1>G2-R7 R1 · New Room D02–D05 (snapshot-only)</h1>",
           "<p><strong>Not an R7 robot run.</strong> Each stage is planned using the current frozen RN04 observation, mean map and RoomNode semantic. D03–D05 are only an OFFLINE FREE reachability proxy (not native collision certificate). Green-cyan line = R7 estimated route, orange = R7 observed-FREE goals, blue = robot. No GT or future snapshot enters planning.</p>"]
    for ix,name,mode,row in images:
        uri="data:image/png;base64,"+base64.b64encode((out/name).read_bytes()).decode("ascii")
        goals=html.escape(str(row["goal_sequence_core_rc"]))
        virtual=html.escape(str([x["goal_rc"] for x in row["virtual_goals_not_executable"]]))
        parts.append(f"<section><h2>D{ix:02d} · {html.escape(mode)}</h2><p>R7 safe proposals: <code>{goals}</code>. Prospective UNKNOWN predicted-FREE (NOT executable): <code>{virtual}</code>.</p><p>G2-only compute: {row['compute_s']:.3f}s. Projected path: {row['predicted_distance_m']:.2f}m, not actual coverage/distance.</p><img loading='lazy' alt='R7 D{ix:02d} map and provisional goals' src='{uri}'></section>")
    (out/"R7_RN04_D02_D05_SCROLL.html").write_text("\n".join(parts))
    print("REPORT",out/"R7_RN04_SNAPSHOT_AUDIT.json")
    print("GALLERY",out/"R7_RN04_D02_D05_SCROLL.html")


if __name__=="__main__":main()
