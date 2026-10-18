"""Self-contained HTML visualization of frozen-current-time R7-R3 plans.

D02 exact current native-reachable FREE mask; D03/D04 RN04 historical
snapshots with observed-FREE proxy only. Not a closed-loop R7 rollout.
"""
from pathlib import Path
import base64,html,json
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont
from r7_missions import R7Missions

HERE=Path(__file__).parent
OUT=HERE/"evidence"
D02="/home/dell/mapex_g2_r6_watchman_20261017/D02_R6_CAUSAL_MAP_ONLY.npz"
BASE="/home/dell/DELL_NATIVE2D/runs/NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob/room_r41"


def get_data(i,initial_pose):
    if i==2:
        a=np.load(D02)
        data=(a["observed"],a["mean"],a["variance"],a["semantic"],
              tuple(int(v) for v in a["robot_core_rc"]),a["native_walkable"].astype(bool))
        return data,"FROZEN_NATIVE_REACHABLE_D02"
    a=np.load(f"{BASE}/inference/decision_{i:06d}_input.npz")
    s=np.load(f"{BASE}/inference/decision_{i:06d}_semantic.npz")
    meta=json.loads(Path(f"{BASE}/decisions/decision_{i:06d}.json").read_text())
    data=(a["observed"],a["mean"],a["variance"],s["semantic"],
          tuple(int(v)-500 for v in meta["robot_rc"]),a["observed"]==0)
    return data,"HISTORICAL_RN04_OBSERVED_FREE_PROXY_ONLY"


def draw(i,solver,plan,scope,zoom=False):
    obs=solver.obs;mean=solver.mean;sem=solver.semantic
    rgb=np.empty((*obs.shape,3),dtype=np.uint8)
    rgb[obs==0]=(245,244,240)
    rgb[obs==1]=(135,137,149)
    rgb[obs==2]=(41,45,54)
    for z,color in ((1,(250,189,187)),(2,(183,204,244)),(3,(186,224,187)),(4,(236,216,164))):
        m=(solver.zone==z)&(obs!=2)
        if not m.any():continue
        rgb[m]=(.58*rgb[m]+.42*np.array(color)).astype(np.uint8)
    corridor=(sem==2)&(obs!=2)
    rgb[corridor]=(.52*rgb[corridor]+.48*np.array([245,218,106])).astype(np.uint8)
    pred_wall=(obs==1)&(mean>=.5)
    rgb[pred_wall]=(72,64,73)
    scale=6
    image=Image.fromarray(rgb).resize((obs.shape[1]*scale,obs.shape[0]*scale),
                                         resample=Image.Resampling.NEAREST)
    dr=ImageDraw.Draw(image)
    def xy(q):return (int(q[1]*scale+scale/2),int(q[0]*scale+scale/2))
    outline=np.uint8(solver.geo.room)
    contours,_=cv2.findContours(outline,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        pts=[(int(x*scale+scale/2),int(y*scale+scale/2)) for [[x,y]] in contour]
        if len(pts)>1:dr.line(pts+[pts[0]],fill=(209,42,48),width=3)
    for portal in solver.geo.portal_candidates:
        x,y=xy(portal)
        dr.ellipse((x-8,y-8,x+8,y+8),outline=(159,33,161),width=3)
    for obstacle in solver.objects:
        x,y=xy(obstacle["center"])
        dr.ellipse((x-5,y-5,x+5,y+5),fill=(6,146,152),outline=(255,255,255),width=1)
    current=solver.robot
    for j,task in enumerate(plan["tasks"],1):
        dest=tuple(task["goal_rc"])
        route=task["virtual_predicted_path_rc"]
        route=[tuple(r) for r in route]
        for x in range(0,len(route)-1,6):
            dr.line([xy(p) for p in route[x:x+4]],fill=(230,136,30),width=3)
        if task["native_known_free_path_exists"]:
            native=solver.native_paths.path(current,dest)
            if native and len(native)>1:
                dr.line([xy(p) for p in native],fill=(28,158,93),width=4)
        x,y=xy(dest)
        color=(251,172,24) if j==1 else ((24,177,89) if i==2 else (243,121,27))
        dr.ellipse((x-12,y-12,x+12,y+12),fill=color,outline=(255,255,255),width=4)
        dr.text((x+11,y-22),f"G{j if i==2 else j+2}",fill=(16,38,49),
                stroke_fill=(255,255,255),stroke_width=2)
        current=dest
    x,y=xy(solver.robot)
    dr.ellipse((x-12,y-12,x+12,y+12),fill=(34,91,206),outline=(255,255,255),width=4)
    if zoom:
        # Focus exclusively on D02 room geometry, preserving room-relative
        # coordinates and displayed goal plan.
        image=image.crop((85*scale,138*scale,214*scale,203*scale))
        image=image.resize((image.width*2,image.height*2),
                           resample=Image.Resampling.NEAREST)
    name=f"R7_R3_D{i:02d}_{'ROOM_ZOOM' if zoom else 'FULL_MAP'}.png"
    image.save(OUT/name,optimize=True)
    return name


def main():
    initial=np.load(D02)
    entry=tuple(map(int,initial["robot_core_rc"]))
    rows=[];cards=[]
    for i in (2,3,4):
        data,scope=get_data(i,entry)
        solver=R7Missions(*data,entry_anchor=entry)
        phase="INITIAL" if i==2 else "SUCCESSOR"
        plan=solver.plan(phase)
        for name in ("ROOM_ZOOM","FULL_MAP"):
            draw(i,solver,plan,scope,zoom=(name=="ROOM_ZOOM"))
        row=dict(snapshot=f"D{i:02d}",scope=scope,
                 not_a_closed_loop_R7_run=True,
                 no_gt_or_future_inputs_to_planner=True,
                 plan=plan)
        rows.append(row)
        zoomfile=f"R7_R3_D{i:02d}_ROOM_ZOOM.png"
        fullfile=f"R7_R3_D{i:02d}_FULL_MAP.png"
        uri=lambda f:"data:image/png;base64,"+base64.b64encode(
            (OUT/f).read_bytes()).decode()
        explanations="".join(
            f"<p><b>Goal {j if i==2 else j+2}:</b> "
            f"{html.escape(str(task['goal_rc']))} — "
            f"{html.escape(task['purpose'])}; "
            f"native-FREE path exists: <b>{task['native_known_free_path_exists']}</b>"
            f"</p>" for j,task in enumerate(plan["tasks"],1))
        cards.append(f"<section><h2>D{i:02d} · "
                     f"{'Plan 1: original prediction' if i==2 else 'Possible successor: later RN04 evidence'}</h2>"
                     f"<p class='meta'>{html.escape(scope)} · "
                     f"Current robot {html.escape(str(data[4]))} · "
                     f"R7 mission compute {plan['planning_s']:.3f}s</p>"
                     f"<p>Detected {plan['zones']} chamber(s); "
                     f"{len(plan['objects'])} interior occluder(s)</p>"
                     +explanations+
                     f"<img alt='Room plan D{i:02d}' loading='lazy' src='{uri(zoomfile)}'>"
                     f"<details><summary>View complete observed+predicted map</summary>"
                     f"<img loading='lazy' src='{uri(fullfile)}'></details></section>")
        print("D",i,"GOALS",plan["goals"],"time",plan["planning_s"],flush=True)
    audit=dict(version="G2_R7_R3_STRUCTURAL_OBLIGATIONS",
               initial_D02_is_real_frozen_input=True,
               D03_D04_are_original_RN04_history_not_R7=True,
               no_hindsight_gt_in_planner=True,
               code_selects_viewpoints_without_manual_coordinates=True,
               stages=rows)
    (OUT/"R7_R3_D02_D04_CAUSAL_SNAPSHOT_AUDIT.json").write_text(
        json.dumps(audit,indent=2,ensure_ascii=False))
    style="""<style>*{box-sizing:border-box}body{background:#f4f6f7;
     color:#22303b;font:15px system-ui,sans-serif;max-width:1260px;
     margin:auto;padding:12px}section,header{background:white;border-radius:16px;
     padding:16px;margin-bottom:16px;box-shadow:0 2px 12px #0001}
     img{width:100%;height:auto;image-rendering:pixelated;border-radius:8px}
     p{line-height:1.5}h2{margin:0 0 10px;font-size:21px}
     .meta{color:#6a727c}summary{padding:12px;cursor:pointer}
     details{margin-top:18px}code{font-size:13px}</style>"""
    content=("<!doctype html><html lang='vi'><meta charset='UTF-8'>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             "<title>G2-R7-R3 · structural D02 and successor</title>"+style+
             "<header><h1>G2-R7-R3 · Structural Mission Planning</h1>"
             "<p><b>Không phải run R7 mới:</b> D02 dùng dữ liệu đóng băng "
             "và native-FREE đã lưu, D03/D04 dùng snapshot lịch sử Run04. "
             "Plan 2 chỉ là tính toán trên dữ liệu lịch sử, chưa phải "
             "kết quả robot thực hiện Plan 1 của R7.</p>"
             "<p>Chấm xanh dương = robot, vàng G1 = khe giữa phòng, "
             "xanh lá G2 = góc đối diện vật cản trái; "
             "cam G3/G4 = hai góc đối diện vật cản phòng phải. "
             "Vạch xanh lá = đường trên known FREE ghi lại, vạch cam đứt "
             "= đường dự đoán, có thể đi qua UNKNOWN và không được phép "
             "dùng trực tiếp để điều khiển. "
             "Viền đỏ = G1 room; tím = cổng hình học.</p></header>"
             +"".join(cards)+"</html>")
    htmlpath=OUT/"R7_R3_D02_D04_SCROLL.html"
    htmlpath.write_text(content)
    print("HTML",htmlpath,"bytes",htmlpath.stat().st_size,flush=True)

if __name__=="__main__":
    main()
