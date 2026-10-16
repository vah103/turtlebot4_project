#!/usr/bin/env python3
"""Offline read-only renderer for COM1-verified Hospital full corrected RUN10 vs RUN11."""
from pathlib import Path
from collections import Counter
import base64
import html
import hashlib
import io
import json
import re
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

BASE=Path("/home/dell/mx072_native2d_pair_20261016/HOSPITAL_FULL_CORRECTED_PAIR/COM1_VERIFIED")
PAIR=BASE/"hospital_full_corrected_20261016/PAIR_EVALUATION.json"
R10=BASE/"native_hospital_full_room_run10/HOSP_FULL_ROOM_R41_RUN10_visvarprob"
R11=BASE/"native_hospital_full_plain_run11/HOSP_FULL_PLAIN_MAPEX_RUN11_visvarprob"
D10=R10/"room_r41"
D11=R11/"plain_mapex_evidence"
OUT=Path("/home/dell/mx072_hospital_full_corrected_RUN10_vs_RUN11_20261016")
OUT.mkdir(parents=True,exist_ok=True)
report=json.loads(PAIR.read_text())
room_od=np.load(R10/"odom.npy",allow_pickle=False)
plain_od=np.load(R11/"odom.npy",allow_pickle=False)
assert len(room_od)==report["runA"]["ticks"]
assert len(plain_od)==report["runB"]["ticks"]
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":9,"axes.titlesize":11,
                     "axes.labelsize":8,"axes.edgecolor":"#8494A8","text.color":"#223247"})
# Verified against v4r41.py ROOM,CORRIDOR,WALL,OBSTACLE = 1,2,4,3
# Native controller RoomRun._room_containing_pose uses sem == 1.
SEM_OTHER,SEM_ROOM,SEM_CORRIDOR,SEM_OBSTACLE,SEM_WALL=0,1,2,3,4
ROOM_PALETTE=np.array([
    [240,244,249], # 0 = other/non-floor
    [255,220,231], # 1 = ROOM (pink)
    [252,225,159], # 2 = CORRIDOR (yellow)
    [65,79,98],    # 3 = OBSTACLE (dark gray)
    [201,63,84],   # 4 = WALL (red)
],dtype=np.uint8)
def jsonread(p):return json.loads(p.read_text(encoding="utf-8"))
def at(i,root,folder):
    p=root/"decisions"/f"decision_{i:06d}.json"
    return jsonread(p) if p.exists() else None
def plan_for(d,root):
    if d is None:return None
    p=root/"steps"/f"tick_{d['native_tick']:06d}_plan.json"
    if not p.exists():raise RuntimeError(f"Missing initial native plan: {p}")
    plan=jsonread(p)
    r=plan.get("from_rc",plan.get("pose_rc"))
    if list(r)!=list(d["robot_rc"]):raise RuntimeError(f"Robot/path pose mismatch {p}")
    if "decision" in plan and plan["decision"]!=d["decision"]:raise RuntimeError("Plan decision mismatch")
    return plan
def goal(p):return p.get("goal_rc",p.get("actual_selected_goal_rc"))
def length(p):return float(p.get("astar_length_m",p.get("astar_path_length_m")))
def path(p):return p.get("real_native_astar_rc",p.get("astar_path_rc"))
def pixelpath(poses):
    if poses is None or len(poses)==0:return [],[]
    arr=np.asarray(poses)
    return .1*(arr[:,1].astype(float)-500+.5),.1*(arr[:,0].astype(float)-500+.5)
def render_img(arr, fdata, native, future, typ, title, tick, d):
    h,w=(arr["semantic"].shape if isinstance(arr,dict) else arr.shape[:2])
    f,ax=plt.subplots(figsize=(9.6,5.1),dpi=106)
    f.patch.set_facecolor("#fff")
    if typ=="semantic":
        sem=arr["semantic"] if isinstance(arr,dict) else arr
        obs=arr["observed"] if isinstance(arr,dict) else None
        baseline=arr["baseline"] if isinstance(arr,dict) else None
        rgb=ROOM_PALETTE[np.asarray(sem,dtype=np.uint8)].copy()
        if obs is not None and baseline is not None:
            tentative=(obs==1)&(sem==SEM_ROOM)&(baseline==SEM_CORRIDOR)
            rgb[tentative]=np.array([251,174,197],dtype=np.uint8)
    else:
        rgb=np.full((h,w,3),[255,255,255],dtype=np.uint8)
        if arr.dtype.kind=="f":
            unknown=np.isclose(arr,0.5)
            wall=arr>=0.75
        else:
            unknown=arr==1
            wall=arr==2
        rgb[unknown]=[215,223,234]
        rgb[wall]=[39,55,76]
    ax.imshow(rgb,origin="upper",extent=(0,w*.1,h*.1,0),interpolation="nearest")
    ax.set_xlim(0,w*.1);ax.set_ylim(h*.1,0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("Metres · native P1 0.10 m/cell");ax.set_ylabel("Metres")
    ax.set_title(title,pad=9,fontweight="bold",loc="left",color="#243850")
    ax.grid(False)
    if typ!="semantic" and d:
        for pt in fdata:
            ax.plot(.1*(pt[1]-500+.5),.1*(pt[0]-500+.5),".",ms=2.3,color="#2d9bd0",alpha=.83,zorder=4)
        # Retrospective motion after this decision, not planner input.
        if future is not None and len(future)>1:
            fx,fy=pixelpath(future)
            ax.plot(fx,fy,color="#078c91",lw=1.65,alpha=.9,zorder=5)
        if native and path(native):
            axx,ayy=pixelpath(path(native))
            ax.plot(axx,ayy,color="#8251be",linestyle="--",lw=1.55,alpha=.97,zorder=6)
        if native:
            gr,gc=goal(native)
            gx,gy=(gc-500+.5)*.1,(gr-500+.5)*.1
            ax.scatter([gx],[gy],s=90,marker="*",color="#f7a620",edgecolors="#714d20",linewidths=.7,zorder=9)
        rr,cc=d["robot_rc"]
        rx,ry=(cc-500+.5)*.1,(rr-500+.5)*.1
        ax.scatter([rx],[ry],s=90,marker="+",color="#c43d2f",linewidths=2.1,zorder=10)
    f.subplots_adjust(left=.06,right=.994,top=.915,bottom=.12)
    stream=io.BytesIO()
    f.savefig(stream,format="jpg",dpi=106,pil_kwargs={"quality":81,"optimize":True})
    plt.close(f)
    return "data:image/jpeg;base64,"+base64.b64encode(stream.getvalue()).decode("ascii")
def formatn(x,n=2):
    return "—" if x is None else f"{float(x):,.{n}f}"
def panel(title,src,description):
    if src is None:
        return '<div class="panel absent"><h3>'+html.escape(title)+'</h3><p>Không có decision tương ứng. Run đã kết thúc; không dựng lại snapshot.</p></div>'
    return ('<div class="panel"><h3>'+html.escape(title)+'</h3>'
            '<img loading="lazy" decoding="async" src="'+src+'" alt="'+html.escape(title)+'" onclick="zoom(this)">'
            '<div class="small">'+html.escape(description)+'</div></div>')
def kv(label,val):return '<div class="metric"><strong>'+html.escape(str(val))+'</strong><span>'+html.escape(label)+'</span></div>'
def nativemetrics(d,p,arm):
    if not d:return ""
    count=(len(d["frontiers"]) if arm=="r10" else d["frontier_candidate_count"])
    return (f'<tr><th>{arm.upper()} first A* length</th><td>{formatn(length(p))} m</td></tr>'
            f'<tr><th>{arm.upper()} robot native pose (r,c)</th><td>{html.escape(str(d["robot_rc"]))}</td></tr>'
            f'<tr><th>{arm.upper()} selected native goal (r,c)</th><td>{html.escape(str(goal(p)))}</td></tr>'
            f'<tr><th>{arm.upper()} frontier candidates drawn</th><td>{count}</td></tr>')
def future_from(i, ds,od):
    d=ds[i-1]
    if d is None:return None
    start=d["native_tick"]
    end=ds[i]["native_tick"] if i<len(ds) and ds[i] else len(od)
    if end<start:raise RuntimeError("Tick order invalid")
    return od[start:end+1] if end<len(od) else od[start:]
r10=[at(i,D10,"room_r41") for i in range(1,42)]
r11=[at(i,D11,"plain_mapex_evidence") for i in range(1,40)]
assert all(r10) and all(r11)
assert len(r10)==41 and len(r11)==39
assert r10[0]["native_tick"]==r11[0]["native_tick"]==0
modes=Counter(d["mode"] for d in r10)
assert modes=={"GLOBAL":31,"LOCAL":10},modes
page_parts=[]
qc={"room_decisions":0,"plain_decisions":0,"semantic_panels":0,"room_panels":0,"plain_panels":0,"native_plan_checked":0,
    "first_decision_same_pose":r10[0]["robot_rc"]==r11[0]["robot_rc"],
    "metrics_sha256":hashlib.sha256(PAIR.read_bytes()).hexdigest(),
    "source_root":str(BASE),"decision_pairs_different_time_or_space":True}
tickroom=[d["native_tick"] for d in r10];tickplain=[d["native_tick"] for d in r11]
assert tickroom==sorted(tickroom) and tickplain==sorted(tickplain)
for i in range(1,42):
    x=r10[i-1];y=r11[i-1] if i<=39 else None
    rp=plan_for(x,D10);pp=plan_for(y,D11) if y else None
    qc["native_plan_checked"]+=1+(1 if pp else 0)
    roomin=np.load(D10/"inference"/f"decision_{i:06d}_input.npz",allow_pickle=False)
    roomsem=np.load(D10/"inference"/f"decision_{i:06d}_semantic.npz",allow_pickle=False)
    observed=roomin["observed"]
    sem=roomsem["semantic"];baseline=roomsem["baseline"]
    assert observed.shape==sem.shape==baseline.shape==(267,577)
    assert np.isin(sem,[0,1,2,3,4]).all()
    assert np.array_equal(ROOM_PALETTE[SEM_ROOM], [255,220,231])
    assert np.array_equal(ROOM_PALETTE[SEM_CORRIDOR], [252,225,159])
    # The true native controller checks semantic == 1 (ROOM), not semantic == 2.
    if x["mode"] == "LOCAL":
        rr,cc=np.asarray(x["robot_rc"])-500
        assert sem[rr,cc] == SEM_ROOM, f"LOCAL robot must be in ROOM on D{i}"
    # Semantic predictions are visualization only: no future data, GT or policy mutation.
    img_sem=render_img({"semantic":sem,"observed":observed,"baseline":baseline},
                       [],None,None,"semantic",f"ROOM NODE V4-R4.1 · D{i:02d}",x["native_tick"],x)
    qc["semantic_panels"]+=1
    img_room=render_img(observed,[(t["row"],t["col"]) for t in x["frontiers"]],rp,
                        future_from(i,r10,room_od),"observed",f"RUN10 · ROOM R4.1 · D{i:02d}",
                        x["native_tick"],x)
    qc["room_panels"]+=1
    img_plain=None
    if y:
        pin=np.load(D11/"decisions"/f"decision_{i:06d}_input.npz",allow_pickle=False)
        pobs=pin["observed"]
        assert pobs.shape==(267,577)
        img_plain=render_img(pobs,y["candidates_rc"],pp,future_from(i,r11,plain_od),
                             "observed",f"RUN11 · PLAIN MAPEX · D{i:02d}",y["native_tick"],y)
        qc["plain_panels"]+=1
    rep=jsonread(D10/"inference"/f"decision_{i:06d}_report.json")
    admission=rep.get("report",{})
    hyp=admission.get("active_hypotheses","—")
    tentative=admission.get("extra_room_unknown_cells","—")
    m=[kv("RUN10 controller",x["mode"]+(" · room "+str(x["room_id"]) if x["room_id"] is not None else "")),
       kv("Tick RUN11 / RUN10",str(y["native_tick"]) if y else "—" +" / "+str(x["native_tick"]))]
    m[1]=kv("Tick RUN11 / RUN10",(str(y["native_tick"]) if y else "—")+" / "+str(x["native_tick"]))
    m.extend([kv("Frontiers RUN11 / RUN10",(str(len(y["candidates_rc"])) if y else "—")+" / "+str(len(x["frontiers"]))),
        kv("Native first A* RUN11 / RUN10",(formatn(length(pp)) if pp else "—")+" / "+formatn(length(rp))+" m"),
        kv("Hypotheses / tentative unknown",str(hyp)+" / "+str(tentative)+" cells")])
    description="Robot = dấu + đỏ; frontier = chấm xanh; đích = sao; A* bản ghi = nét tím; đường teal = quỹ đạo SAU decision (hậu kiểm)."
    extra=(' <span class="status local">LOCAL</span>' if x["mode"]=="LOCAL" else ' <span class="status global">GLOBAL</span>')
    htmlsec=(f'<section class="page" id="D{i:02d}"><h2>Decision D{i:02d}'+extra+'</h2>'
        +'<div class="metrics">'+''.join(m)+'</div>'
        +'<div class="panels">'+panel("1 · MapEx thường — RUN11",img_plain,description)
        +panel("2 · MapEx + Room Node — RUN10",img_room,description)
        +panel("3 · Room Node: cấu trúc dự đoán",img_sem,
                "Hồng: ROOM; vàng: CORRIDOR; đỏ: WALL; xám đậm: obstacle; hồng đậm: phòng giả thuyết trên UNKNOWN.")+'</div>'
        +'<div class="reason"><strong>RUN10 reason:</strong> '+html.escape(x.get("reason",""))
        +' &nbsp;|&nbsp; <strong>Scoring scope:</strong> '+html.escape(x.get("scoring_scope",""))
        +' &nbsp;|&nbsp; <strong>Local/global candidates:</strong> '+str(x.get("local_candidate_count"))+'/'+str(x.get("global_candidate_count"))+'</div>'
        +'<details><summary>Chi tiết số liệu gốc decision</summary><table><tbody>'
        +(f'<tr><th>RUN11 native tick</th><td>{y["native_tick"]}</td></tr>' if y else '<tr><th>RUN11 native tick</th><td>Run đã kết thúc</td></tr>')
        +f'<tr><th>RUN10 native tick</th><td>{x["native_tick"]}</td></tr>'
        +nativemetrics(y,pp,"r11")+nativemetrics(x,rp,"r10")
        +(f'<tr><th>RUN10 traveled before decision</th><td>{formatn(x.get("cumulative_distance_m_before_decision"))} m</td></tr>'
        f'<tr><th>RUN10 room LaMa-only inference</th><td>{formatn(x.get("elapsed_lama_prediction_s"),3)} s</td></tr>'
        f'<tr><th>RUN10 active hypotheses</th><td>{hyp}</td></tr>'
        f'<tr><th>RUN10 tentative unknown room cells</th><td>{tentative}</td></tr>'
        f'<tr><th>RUN10 room ID</th><td>{x.get("room_id") if x.get("room_id") is not None else "—"}</td></tr>')
        +'</tbody></table></details></section>')
    page_parts.append(htmlsec)
    if i%10==0:print(f"Rendered {i}/41",flush=True)

a=report["runA"];b=report["runB"];q=report["paired_comparison"]
summary=(
'<section class="page summary" id="SUMMARY"><h2>Tổng kết · Hospital FULL corrected · RUN10 vs RUN11</h2>'
+f'<div class="kpis"><div class="kpi"><strong>{q["room_travel_saved_pct"]:.2f}%</strong><small>Quãng đường RUN10 giảm</small></div>'
+f'<div class="kpi"><strong>{q["room_travel_saved_m"]:.2f} m</strong><small>Tiết kiệm so với RUN11</small></div>'
+f'<div class="kpi adverse"><strong>+{a["wallclock_s"]-b["wallclock_s"]:.2f} s</strong><small>RUN10 chậm hơn</small></div>'
+f'<div class="kpi"><strong>{a["free_coverage_gt"]*100:.4f}%</strong><small>RUN10 free coverage GT offline</small></div></div>'
+'<table><thead><tr><th>Chỉ số</th><th>RUN11 · MapEx thường</th><th>RUN10 · Room Node R4.1</th></tr></thead><tbody>'
+''.join(f'<tr><th>{html.escape(lbl)}</th><td>{html.escape(str(v11))}</td><td>{html.escape(str(v10))}</td></tr>' for lbl,v11,v10 in [
("Kết thúc planner",b["native_receipt_outcome"],a["native_receipt_outcome"]),
("Mission complete (planner)",b["native_completed"],a["native_completed"]),
("Termination reason",b["native_termination_reason"],a["native_termination_reason"]),
("Scored decisions",b["decisions"],a["decisions"]),
("Native ticks",b["ticks"],a["ticks"]),
("Quãng đường",f'{b["distance_m"]:.2f} m',f'{a["distance_m"]:.2f} m'),
("Wall clock",f'{b["wallclock_s"]:.2f} s',f'{a["wallclock_s"]:.2f} s'),
("Known cells cuối",b["known_all_cells"],a["known_all_cells"]),
("Unknown cells cuối",b["unknown_all_cells"],a["unknown_all_cells"]),
("Known free GT cells",b["known_free_gt_cells"],a["known_free_gt_cells"]),
("GT free coverage",f'{b["free_coverage_gt"]*100:.5f}%',f'{a["free_coverage_gt"]*100:.5f}%'),
("GT occupied seen",b["known_wall_gt_cells"],a["known_wall_gt_cells"]),
("Observed known accuracy GT",f'{b["observed_known_accuracy_gt"]*100:.5f}%',f'{a["observed_known_accuracy_gt"]*100:.5f}%'),
("GT errors on known",b["observed_mistakes_vs_structural_gt_cells"],a["observed_mistakes_vs_structural_gt_cells"]),
("GLOBAL / LOCAL",f'{b["room_modes"]["GLOBAL"]} / 0',f'{a["room_modes"]["GLOBAL"]} / {a["room_modes"]["LOCAL"]}'),
("Recorded median prediction time",f'{b["model_timing_median_s"]:.3f} s',f'{a["model_timing_median_s"]:.3f} s'),
])
+'</tbody></table>'
+'<p class="caution"><strong>Giới hạn phép so:</strong> Đây là 1 cặp Native2D Hospital FULL corrected, không phải ROS/Gazebo. '
+'Kết thúc SOURCE_TERMINAL là không còn frontier có thể chọn, không chứng minh GT hoàn tất. '
+'RUN10 có 41 decision, RUN11 có 39; cùng chỉ số D không đại diện cùng thời điểm hoặc vị trí. '
+'Các chỉ số GT chỉ được đánh giá offline. RUN10 thời gian lâu hơn, không được gọi là nhanh hơn. '
+'Hai số median inference sử dụng timing contract KHÁC NHAU nên không so trực tiếp.</p>'
+'<p class="foot">Provenance: SHA256 verified 17,838 files / 221,833,419 bytes từ COM1 sang DELL; '
+'PAIR_EVALUATION.json và QA_INDEPENDENT_DELL_RECALC/RESULT.json. Source exact: '
+'HOSPITAL_FULL_CORRECTED_PAIR/COM1_VERIFIED. Cặp so sánh tại đây không dùng ảnh giả hoặc snapshot từ RUN08–09.</p>'
+'</section>')
nav=''.join(f'<a href="#D{i:02d}">D{i:02d}</a>' for i in range(1,42))+'<a href="#SUMMARY">TỔNG KẾT</a>'
style=r"""
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:#eaf0f6;color:#24364b;font-family:Arial,"DejaVu Sans",sans-serif}
header{background:linear-gradient(110deg,#163653,#2c668e);color:#fff;padding:26px 4%}header h1{font-size:26px;margin:0 0 7px}header p{margin:5px 0 0}
header .pale{font-size:13px;color:#d8e7f2;line-height:1.5}
nav{display:flex;gap:5px;position:sticky;top:0;z-index:8;background:#fefefe;overflow-x:auto;white-space:nowrap;border-bottom:1px solid #bac6d2;padding:9px 12px}
nav a{border:1px solid #c5d5e5;background:#f4f7fa;color:#204a6b;border-radius:5px;text-decoration:none;padding:5px 9px;font-size:12px}
main{max-width:1800px;margin:auto;padding:15px}.page{background:#fff;border-radius:11px;box-shadow:0 3px 13px #162c3a19;margin-bottom:18px;padding:18px;scroll-margin-top:60px}
.page h2{font-size:19px;margin:1px 0 12px}.status{font-size:12px;border-radius:5px;padding:4px 7px;vertical-align:middle}.local{background:#fff0d1;color:#79520b}.global{background:#e1f3fa;color:#165475}
.metrics{display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:8px;margin:13px 0}.metric{background:#f6f9fd;border:1px solid #dbe7f1;border-radius:8px;padding:9px 7px;min-width:0}
.metric strong{display:block;font-size:14px;overflow-wrap:anywhere;color:#2b567c}.metric span{display:block;color:#6d8296;font-size:11px;margin-top:5px}
.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.panel{border:1px solid #d6e2ec;border-radius:8px;background:#fff;min-width:0;overflow:hidden}
.panel h3{background:#f3f7fa;margin:0;padding:8px 10px;font-size:13px}.panel img{display:block;width:100%;object-fit:contain;cursor:zoom-in}.small{font-size:11px;color:#6a7b89;padding:7px;line-height:1.45}
.absent{background:#f7f9fb;min-height:150px}.absent p{color:#6f8294;font-size:13px;padding:15px}
.reason{font-size:12px;background:#f5f8fb;padding:10px;margin-top:8px;border-radius:6px;line-height:1.7;color:#4c6479}
details{margin:10px 0 0}summary{font-size:12px;color:#2f6688;cursor:pointer;padding:5px}
table{width:100%;border-collapse:collapse;margin:8px 0}th,td{border:1px solid #d9e4ee;padding:8px 10px;font-size:12px;text-align:left}
th{background:#f5f8fc}.summary .kpis{display:grid;grid-template-columns:repeat(4,1fr);gap:10px}
.kpi{padding:14px;border:1px solid #dce5ec;border-radius:8px;background:#f7fbff}.kpi strong{display:block;font-size:24px;color:#1c6a86}.kpi small{color:#668094}.kpi.adverse strong{color:#a65f18}
.caution{background:#fff7ed;padding:12px;border-left:4px solid #d49c50;line-height:1.65;font-size:13px}
.foot{font-size:12px;color:#698295;line-height:1.5}
#lightbox{display:none;position:fixed;inset:0;background:#071321ec;z-index:25;align-items:center;justify-content:center;cursor:zoom-out;padding:10px}
#lightbox img{max-height:96vh;max-width:97vw;object-fit:contain}
@media(max-width:1150px){.panels{grid-template-columns:1fr 1fr}.metrics{grid-template-columns:repeat(3,1fr)}}
@media(max-width:750px){main{padding:6px}.page{padding:8px}.panels{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,1fr)}.summary .kpis{grid-template-columns:repeat(2,1fr)}}
@media print{nav{display:none}.page{page-break-after:always;border-radius:0;box-shadow:none;margin:0}body{background:#fff}main{padding:0}}
"""
body=('<!DOCTYPE html><html lang="vi"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
'<title>Hospital FULL Corrected | RUN10 Room R4.1 vs RUN11 MapEx</title><style>'+style+'</style></head><body>'
'<header><h1>MX072 · Hospital FULL Corrected — 41 vs 39 decision thực tế</h1>'
'<p><strong>RUN11: MapEx thường</strong> &nbsp;vs&nbsp; <strong>RUN10: MapEx + Room Node V4-R4.1</strong></p>'
'<p class="pale">Dữ liệu COM1 đã SHA-verified sang DELL; map 0,10m/cell, geometry Hospital corrected. '
'Mỗi panel dùng snapshot từ chính run của nó. Từ sau D01, cùng số decision KHÔNG đồng nghĩa cùng vị trí, tick hay bản đồ.</p></header>'
'<nav>'+nav+'</nav><main>'+''.join(page_parts)+summary+'</main>'
'<div id="lightbox" onclick="this.style.display=\'none\'"><img id="fullimg" alt="Phóng to"></div>'
'<script>function zoom(el){document.getElementById("fullimg").src=el.src;document.getElementById("lightbox").style.display="flex"}'
'document.addEventListener("keydown",function(e){if(e.key==="Escape")document.getElementById("lightbox").style.display="none"})</script>'
'</body></html>')
htmlpath=OUT/"MX072_HOSPITAL_FULL_RUN10_vs_RUN11_REAL_41v39_DECISIONS.html"
htmlpath.write_text(body,encoding="utf-8")
qc.update({"out_html":str(htmlpath),"html_bytes":htmlpath.stat().st_size,"html_sha256":hashlib.sha256(htmlpath.read_bytes()).hexdigest(),
           "total_images":qc["plain_panels"]+qc["room_panels"]+qc["semantic_panels"],
           "pair_distance_saved_m":q["room_travel_saved_m"],"pair_distance_saved_pct":q["room_travel_saved_pct"],
           "wall_s_room_minus_plain":a["wallclock_s"]-b["wallclock_s"],
           "source_com1_verified_files":17838,"independent_science_QA":"NOT_PERFORMED",
           "rendering_future_trace_is_retrospective":True,"rendering_uses_GT_as_input":False,
           "missing_plain_D40_D41":"explicit_blank_panels_not_fabricated"})
assert qc["total_images"]==121 and qc["native_plan_checked"]==80
(OUT/"RENDER_VALIDATION_REPORT.json").write_text(json.dumps(qc,indent=2,ensure_ascii=False),encoding="utf-8")
print("FINISHED",json.dumps(qc,ensure_ascii=False,indent=2),flush=True)
