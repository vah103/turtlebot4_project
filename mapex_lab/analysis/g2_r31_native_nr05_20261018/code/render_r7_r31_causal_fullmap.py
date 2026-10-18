"""Truthful native R7-R3.1 full-map gallery, no future-goal overlays.

At each tick draw only goals known to the actual policy by that timestamp:
LOCAL G2 choose/plan, actual Plan2 observation witness, actual goal ACKs.
No hardcoded D02 coordinates or late-event 'future' annotations. GT not used.
"""
from pathlib import Path
from bisect import bisect_right
import base64,html,json
import numpy as np
from PIL import Image,ImageDraw

ROOT=Path('/home/dell/DELL_NATIVE2D/runs/NR-G2R7-05')
RUN=ROOT/'native_g2_r7_r31_stable_run01/G2_R7_R31_STABLE_RUN01_visvarprob'
STEPS=RUN/'room_r41/steps'
DEC=RUN/'room_r41/decisions'
INFER=RUN/'room_r41/inference'
R7=RUN/'g2_r7/decisions'
OUT=ROOT/'r7_visuals_causal'
OUT.mkdir(exist_ok=True)
DEST=ROOT/'R7_R31_FULL_MAP_CAUSAL_TIMELINE.html'
MAN=ROOT/'R7_R31_FULL_MAP_CAUSAL_TIMELINE.json'

def predicted_background(obs,mean,sem):
    """Only same-epoch current input from LaMa mean and RoomNode, not GT."""
    base=np.zeros((obs.shape[0],obs.shape[1],3),dtype=np.uint8)
    base[obs==0]=(251,250,247)
    base[obs==1]=(59,63,74)
    unknown=obs==.5
    base[unknown]=(142,146,157)
    predicted_free=unknown&(mean<.5)
    base[predicted_free]=(218,217,219)
    room=(sem==1)&(obs!=1)
    base[room]=(.55*base[room]+.45*np.array([249,176,170])).astype(np.uint8)
    corridor=(sem==2)&(obs!=1)
    base[corridor]=(.55*base[corridor]+.45*np.array([247,216,109])).astype(np.uint8)
    return base

def load_events():
    g1={}
    for p in DEC.glob('decision_*.json'):
        a=json.loads(p.read_text());g1[int(a['native_tick'])]=int(p.stem.rsplit('_',1)[1])
    r7={}
    for p in R7.glob('decision_*.json'):
        a=json.loads(p.read_text());r7[int(a['native_tick'])]=a
    ack={}
    for p in R7.glob('arrived_tick_*.json'):
        a=json.loads(p.read_text());ack[int(p.stem.rsplit('_',1)[1])]=a
    return g1,r7,ack

def full_path_lookup(timeline):
    poses=[]
    path_map={}
    for i in range(timeline+1):
        p=STEPS/('tick_%06d_summary.json'%i)
        if not p.exists():continue
        d=json.loads(p.read_text())
        rc=(int(d['pose_rc'][0])-500,int(d['pose_rc'][1])-500)
        if not poses or poses[-1]!=rc:poses.append(rc)
        path_map[i]=len(poses)
    return poses,path_map

def first_seen_goals(r7):
    """Only for stable labelling of actual goals; does not imply prior visibility."""
    lookup={}
    n=0
    for t,event in sorted(r7.items()):
        for p in event.get('goal_sequence_core_rc') or []:
            k=tuple(p)
            if k not in lookup:
                n+=1;lookup[k]='R7-%d'%n
        witness=event.get('plan2_witness') or {}
        for task in witness.get('obligations',[]):
            p=task.get('goal_rc')
            if p is None:continue
            k=tuple(p)
            if k not in lookup:
                n+=1;lookup[k]='R7-%d'%n
    return lookup

def events_as_of(tick,r7,ack,goal_names):
    known_decisions=[(t,e) for t,e in sorted(r7.items()) if t<=tick]
    visited=[(t,e) for t,e in sorted(ack.items()) if t<=tick]
    labels={}
    for tm,e in visited:
        p=e.get('finished_goal_rc')
        if p is None:continue
        k=tuple(p)
        labels[k]={"goal_rc":list(k),"label":goal_names.get(k,'R7'),
                   "state":"ARRIVED","source_tick":tm,"evidence":"NATIVE_ACK"}
    if known_decisions:
        at,e=known_decisions[-1]
        # Current intent only from most recent G2 decision; after Goal1
        # Plan2 witness is a causal *candidate*, not yet an A*-selected goal.
        current=e.get('goal_core_rc')
        current_t=tuple(current) if current is not None else None
        for p in e.get('goal_sequence_core_rc') or []:
            k=tuple(p)
            if k in labels and labels[k]['state']=='ARRIVED':continue
            labels[k]={"goal_rc":list(k),"label":goal_names.get(k,'R7'),
                       "state":"SELECTED" if k==current_t else "PLANNED",
                       "source_tick":at,
                       "evidence":"G2_DECISION"+("_NATIVE_ASTAR" if k==current_t else "_PLAN_ONLY")}
        witness=e.get('plan2_witness') or {}
        witness_time=int(witness.get('detected_native_tick',at))
        if witness_time<=tick:
            for task in witness.get('obligations',[]):
                p=task.get('goal_rc')
                if p is None:continue
                k=tuple(p)
                if k not in labels:
                    labels[k]={"goal_rc":list(k),"label":goal_names.get(k,'R7'),
                               "state":"PLAN2_PREPARED","source_tick":witness_time,
                               "evidence":"OBSERVED_CONTRADICTION_NOT_DISPATCHED"}
        # A goal already ACKed but no longer in the freshest sequence
        # remains historical ARRIVED and cannot become CURRENT by accident.
        for tm,a in visited:
            if tm>at:continue
            p=a.get('finished_goal_rc')
            if p and tuple(p) in labels:
                k=tuple(p);labels[k]['state']='ARRIVED'
    # cached transitions require ACK to take effect as selected next goal,
    # even if no fresh G1 heavy epoch has happened.
    if visited:
        tm,e=visited[-1]
        after=[tuple(p) for p in e.get('remaining_goals_rc') or []]
        # Only cache if no newer official G2 choose epoch is present.
        last_t=known_decisions[-1][0] if known_decisions else -1
        if tm>last_t and after:
            k=after[0]
            if k in labels and labels[k]['state']!='ARRIVED':
                labels[k]['state']='SELECTED'
                labels[k]['evidence']='CACHED_NATIVE_ASTAR_AFTER_ACK'
        elif tm>last_t and not after:
            # Don't accidentally imply old active target remains selected.
            for item in labels.values():
                if item['state']=='SELECTED':item['state']='PLANNED'
    return list(labels.values())

def scene(tick,poses,pose_len,g1,r7,ack,names):
    state=json.loads((STEPS/('tick_%06d_summary.json'%tick)).read_text())
    epoch=int(state['score_epoch'])
    with np.load(STEPS/('tick_%06d_postscan.npz'%tick)) as f:
        obs=f['observed'][500:-500,500:-500].copy()
    prefix=INFER/('decision_%06d'%epoch)
    if Path(str(prefix)+'_input.npz').exists() and Path(str(prefix)+'_semantic.npz').exists():
        with np.load(str(prefix)+'_input.npz') as z: mean=z['mean'].copy()
        with np.load(str(prefix)+'_semantic.npz') as z: sem=z['semantic'].copy()
    else:
        mean=np.ones(obs.shape,np.float32);sem=np.zeros(obs.shape,np.uint8)
    observed_rgb=np.empty((obs.shape[0],obs.shape[1],3),dtype=np.uint8)
    observed_rgb[obs==0]=(248,247,243)
    observed_rgb[obs==.5]=(142,145,154)
    observed_rgb[obs==1]=(44,47,54)
    # Separate OBSERVED-LIDAR and PREDICTED-G1 views to avoid mistaking
    # LaMa-predicted unknown structures for observed walls/rooms.
    views=[("OBSERVED",observed_rgb),
           ("PREDICTED_G1",predicted_background(obs,mean,sem))]
    def pt(p):return (int(p[1])*5+2,int(p[0])*5+2)
    path=poses[:pose_len]
    known=events_as_of(tick,r7,ack,names)
    palette_colors={"ARRIVED":(30,170,104),"SELECTED":(243,149,20),
                    "PLANNED":(244,195,69),"PLAN2_PREPARED":(147,79,208)}
    frames=[]
    for kind,bg in views:
        img=Image.fromarray(bg).resize(
            (obs.shape[1]*5,obs.shape[0]*5),Image.Resampling.NEAREST)
        dr=ImageDraw.Draw(img)
        if len(path)>1:dr.line([pt(p) for p in path],fill=(12,149,113),width=4)
        for item in known:
            k=tuple(item["goal_rc"]);x,y=pt(k)
            status=item['state']
            c=palette_colors[status]
            if status=="PLAN2_PREPARED":
                dr.ellipse((x-10,y-10,x+10,y+10),outline=c,width=3)
            elif status=="PLANNED":
                dr.ellipse((x-10,y-10,x+10,y+10),fill=(255,255,255),outline=c,width=3)
            else:
                dr.ellipse((x-10,y-10,x+10,y+10),fill=c,outline="white",width=3)
            off=-42 if k[1]>obs.shape[1]*.78 else 10
            dr.text((x+off,y-22),item["label"],fill=(32,42,53),
                    stroke_width=2,stroke_fill=(255,255,255))
        if path:
            x,y=pt(path[-1])
            dr.ellipse((x-12,y-12,x+12,y+12),fill=(31,91,217),
                       outline=(255,255,255),width=3)
        frame=OUT/('R7_R31_CAUSAL_T%04d_%s_FULL.png'%(tick,kind))
        img.save(frame,optimize=True)
        frames.append(frame)
    return frames[0],frames[1],known,obs,state

def main():
    g1,r7,ack=load_events()
    name_map=first_seen_goals(r7)
    odom=np.load(RUN/'odom.npy',mmap_mode="r")
    final=len(odom)-1
    stages=sorted(set(g1)|set(r7)|set(ack)|set(range(0,final+1,50))|{0,12,27,55,85,110,130,145,200,350,500,650,850,final})
    stages=[i for i in stages if (STEPS/('tick_%06d_summary.json'%i)).exists() and
            (STEPS/('tick_%06d_postscan.npz'%i)).exists()]
    positions,cumulative=full_path_lookup(final)
    n_early=0
    sections=[];metadata=[]
    for i,tick in enumerate(stages,1):
        f_observed,f_predicted,goals,obs,state=scene(tick,positions,cumulative[tick],g1,r7,ack,name_map)
        if tick<min(r7) and goals:raise RuntimeError('EARLY_FUTURE_GOALS_FOUND %d'%tick)
        if tick==0 and goals:raise RuntimeError('GOALS_AT_INITIAL_GLOBAL!')
        if tick==12 and len(goals)!=2:raise RuntimeError('EXPECTED_TWO_GOALS_AT_12')
        if tick==27 and len(goals)!=4:raise RuntimeError('EXPECTED_FOUR_KNOWN_AT_GOAL1_AFTER_ACK')
        if tick==55 and not any(x["state"]=="SELECTED" and x["goal_rc"]==[196,206] for x in goals):
            raise RuntimeError('NO_SELECTED_RIGHT_BOTTOM_GOAL_AFTER_G2_PLAN2')
        for f in (f_observed,f_predicted):
            with Image.open(f) as im:
                w,h=im.size
                if (w,h)!=(1315,1015):raise RuntimeError('CROPPED_MAP')
        src_observed="data:image/png;base64,"+base64.b64encode(f_observed.read_bytes()).decode()
        src_predicted="data:image/png;base64,"+base64.b64encode(f_predicted.read_bytes()).decode()
        scope=(state.get('current_scope') or {}).get('mode') or "UNDECIDED"
        known=int((obs!=.5).sum())
        distance=float(state['cumulative_distance_m'])
        tags=[]
        if tick in g1:tags.append('G1 decision %d'%g1[tick])
        if tick in ack:tags.append('Robot thực sự tới '+name_map.get(tuple(ack[tick]['finished_goal_rc']),'R7 goal'))
        if tick==final:tags.append("TERMINAL")
        names=" | ".join(tags) if tags else "Mốc quãng đường"
        phase="GLOBAL / MapEx" if scope=="GLOBAL" else "LOCAL / G1 + R7"
        goaltext=", ".join(item['label']+":"+item['state'] for item in goals)
        if not goaltext:goaltext="Chưa có goal R7 nào"
        secs=("<section><h2>%02d/%02d — Tick %d · %s</h2>"
          "<p>%s; đã đi %.2f m; observed %d / 53389 (%.2f%%)</p>"
          "<p class='goals'>%s</p>"
          "<h3>1 · Observed LiDAR — chỉ những ô robot thực sự đã quét</h3>"
          "<img src='%s' width='1315' height='1015' alt='Full observed-only map at native tick %d'>"
          "<h3>2 · LaMa + G1 — giả thuyết cấu trúc, chưa chắc là thực tế</h3>"
          "<img src='%s' width='1315' height='1015' alt='Full predicted map at native tick %d'>"
          "</section>")%(i,len(stages),tick,html.escape(names),html.escape(phase),
                         distance,known,known/53389*100,html.escape(goaltext),
                         src_observed,tick,src_predicted,tick)
        sections.append(secs)
        metadata.append({"tick":tick,"scope":scope,"distance_m":distance,
           "observed_cells":known,"r7_goals_displayed":goals,"is_full_map":True,
           "image_size":[w,h],"image_observed":f_observed.name,
           "image_prediction":f_predicted.name,
           "provenance":"G1 native scope; R7 current-time decisions and ACK"})
        print('%02d/%02d T%04d %-7s %5d known R7-goals %-2d %s'%(i,len(stages),tick,scope,known,len(goals),goaltext),flush=True)
    css="""<style>*{box-sizing:border-box}body{max-width:1370px;margin:0 auto;padding:12px;
    background:#f2f4f6;color:#26384b;font:15px system-ui}
    header,section{padding:15px;margin:14px 0 20px;background:white;border-radius:12px;
    box-shadow:0 1px 8px #00000010}
    h1{font-size:24px}h2{font-size:18px}p{line-height:1.5}
    .goals{font-size:13px;color:#65758a}img{width:100%;height:auto;
    image-rendering:pixelated;border-radius:7px;border:1px solid #e2e8f0}
    .legend{background:#f7f9fb;padding:14px;border-radius:10px}</style>"""
    header="""<!doctype html><html lang='vi'><meta charset='utf-8'>
      <meta name='viewport' content='width=device-width,initial-scale=1'>
      <title>NR-G2R7-05 — Full map / causal goals / native run</title>"""+css+"""
      <header><h1>NR-G2R7-05 — Full map: goal xuất hiện đúng lúc robot biết</h1>
      <p><b>Hai ảnh full map riêng biệt cho mỗi bước:</b> ảnh 1 chỉ là
      LiDAR observed thật; ảnh 2 là cấu trúc mà LaMa và G1 DỰ ĐOÁN
      tại epoch đó — không phải bản đồ robot đã quan sát.
      <b>Bản sửa lỗi hiển thị goal tương lai:</b> trước khi G1 xác nhận
      phòng LOCAL, không có Goal R7. Từ tick12 có Plan1 hai goal. Sau khi
      robot đến Goal1 và LiDAR phát hiện vật cản ở tick27, mới hiển thị
      đề xuất Plan2; sau tick55 hai goal Plan2 mới được giao để đi.</p>
      <p class='legend'><b>Màu:</b> đường xanh lá = robot thực sự đã đi;
      xanh dương = robot đang đứng; cam đặc = goal đã được native
      chọn và kiểm tra A*; vàng rỗng = goal trong Plan1/Plan2 nhưng chưa
      được giao; tím rỗng = phương án Plan2 vừa chuẩn bị từ quan sát thực;
      xanh lá tròn = robot đã tới nơi (native ACK). Chỉ hiển thị bằng chứng
      có từ thời điểm hiện tại hoặc trước đó. Các ảnh đều là FULL MAP,
      không có ảnh cắt D02 hay menu ẩn.</p>
      <p>MapEx GLOBAL &rarr; G1 ROOM LOCAL &rarr; R7 structural goals;
      native MapEx known-FREE A* xác nhận đường đi. Đã kiểm tra số liệu
      sự kiện G1, R7 decisions và ACK từ log; hình dự đoán là LaMa
      ở native decision epoch hiện thời, không phải GT.</p>
      <p><b>Số ảnh:</b> %d; <b>Số G1 decisions:</b> %d;
      <b>native ticks:</b> %d.</p></header>"""%(len(stages),len(g1),final+1)
    DEST.write_text(header+"\n".join(sections)+"</html>")
    result={"run_id":"NR-G2R7-05","gallery":"full_map_actual_causal_goals",
       "previous_gallery_issue":"goal1-goal4 manually drawn at every tick even before any room was confirmed",
       "no_hardcoded_goal_positions":True,
       "first_r7_decision_tick":min(r7),
       "first_r7_goal_count_at_tick0":0,
       "g1_decisions":len(g1),"snapshots":metadata,
       "frames":len(stages),"full_map_every_frame":True,
       "GT_not_used_for_policy":True}
    MAN.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    source=DEST.read_text()
    assert "<details" not in source
    assert source.count("data:image/png;base64,")==2*len(stages)
    assert source.count("<section>")==len(stages)
    print('WRITTEN',DEST,'size',DEST.stat().st_size,'events',len(stages))
    print('CHECK_T0',metadata[0]['r7_goals_displayed'])
    print('CHECK_T12',[x for x in metadata if x['tick']==12][0]['r7_goals_displayed'])
    print('CHECK_T27',[x for x in metadata if x['tick']==27][0]['r7_goals_displayed'])
if __name__=='__main__':main()
