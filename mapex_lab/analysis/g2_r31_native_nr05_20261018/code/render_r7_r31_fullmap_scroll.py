"""All-full-map continuous scrolling gallery of native R7-R3.1 NR-G2R7-05.

Every native G1 decision + G2 real goal arrivals + distance checkpoints.
Never crop; never display mini-room image; no expanding / clicking needed.
"""
from __future__ import annotations
import json
import base64
import html
from pathlib import Path
from PIL import Image
import render_r7_closedloop_nr05 as prior

ROOT = Path('/home/dell/DELL_NATIVE2D/runs/NR-G2R7-05')
RUN = ROOT/'native_g2_r7_r31_stable_run01/G2_R7_R31_STABLE_RUN01_visvarprob'
STEPS = RUN/'room_r41/steps'
DECISIONS = RUN/'room_r41/decisions'
OUT = ROOT/'r7_visuals'
OUTPUT = ROOT/'R7_R31_FULL_MAP_ALL_DECISIONS_SCROLL.html'
MANIFEST = ROOT/'R7_R31_FULL_MAP_SCROLL_MANIFEST.json'

def checkpoints():
    decisions={}
    for p in sorted(DECISIONS.glob('decision_*.json')):
        event=json.loads(p.read_text())
        tick=int(event['native_tick'])
        decisions[tick]=int(p.stem.rsplit('_',1)[-1])
    odom=RUN/'odom.npy'
    import numpy as np
    final=int(np.load(odom,mmap_mode='r').shape[0]-1)
    arrival_ticks={27:'Goal 1',55:'Goal 2',110:'Goal 3',130:'Goal 4'}
    fixed=set([0,12,27,55,85,110,130,145,200,350,500,650,850,final])
    # Every ~50 native ticks gives visible trajectory evolution between all decisions.
    spaced=set(range(0,final+1,50))
    valid=lambda i: (STEPS/('tick_%06d_postscan.npz'%i)).exists() and (STEPS/('tick_%06d_summary.json'%i)).exists()
    ticks=sorted(i for i in set(decisions)|fixed|spaced if valid(i))
    return ticks,decisions,arrival_ticks,final

def main():
    ticks,decisions,arrivals,final=checkpoints()
    if len(decisions)!=27:
        raise RuntimeError('unexpected native G1 decision count %d'%len(decisions))
    # Avoid disk re-reads for every image's trajectory overlay.
    positions=[]
    for i in range(final+1):
        file=STEPS/('tick_%06d_summary.json'%i)
        if file.exists():
            a=json.loads(file.read_text())
            y,x=a['pose_rc']
            newpos=(int(y)-500,int(x)-500)
            if not positions or positions[-1]!=newpos:positions.append(newpos)
        # Store copy of already visited positions only for frames being rendered.
    import functools
    @functools.lru_cache(maxsize=2)
    def full_path_at(t):
        path=[]
        for j in range(t+1):
            fp=STEPS/('tick_%06d_summary.json'%j)
            if fp.exists():
                a=json.loads(fp.read_text())
                pt=(int(a['pose_rc'][0])-500,int(a['pose_rc'][1])-500)
                if not path or path[-1]!=pt:path.append(pt)
        return path
    prior.actual_odom=full_path_at
    snapshots=[]
    sections=[]
    for index,tick in enumerate(ticks,1):
        fname,obs,metadata=prior.picture(tick,focused=False)
        with Image.open(OUT/fname) as im:
            wh=im.size
        if wh!=(1315,1015):
            raise RuntimeError('NOT_FULL_MAP %s %r'%(fname,wh))
        src='data:image/png;base64,'+base64.b64encode((OUT/fname).read_bytes()).decode('ascii')
        distance=float(metadata['cumulative_distance_m'])
        known=int((obs!=.5).sum())
        scope=metadata.get('current_scope') or {}
        mode=scope.get('mode') or 'N/A'
        tags=[]
        if tick in decisions:tags.append('Decision %02d'%decisions[tick])
        if tick in arrivals:tags.append(arrivals[tick]+' • actual arrival')
        if tick==final:tags.append('TERMINAL')
        tag=' | '.join(tags) if tags else 'Position/scan checkpoint'
        fields={'tick':tick,
                'scope':mode,
                'distance_m':round(distance,3),
                'known_p1':known,
                'coverage_pct':round(known/53389*100,4),
                'decision_epoch':decisions.get(tick),
                'native_arrival':arrivals.get(tick),
                'source_full_image':fname,
                'full_map_resolution_px':list(wh),
                'is_full_map':True}
        snapshots.append(fields)
        sections.append('<section id="tick-%04d">'%tick+
                        '<div class="head"><h2>%02d / %02d · T%04d</h2><span class="kind">%s</span></div>'%
                        (index,len(ticks),tick,html.escape(tag))+
                        '<p class="meta">Đường đi: <b>%.2f m</b> · Đã quan sát: <b>%d / 53.389 ô (%.2f%%)</b> · %s</p>'%
                        (distance,known,known/53389*100,html.escape(mode))+
                        '<img src="%s" alt="Full New Room observed map and actual robot path at native tick %d" loading="%s" width="1315" height="1015">'%
                        (src,tick,'eager' if index<=2 else 'lazy')+
                        '</section>')
        print('%02d/%02d tick %4d %7.2fm %d known %s %s'%(index,len(ticks),tick,distance,known,mode,tag),flush=True)
    css="""<style>
      *{box-sizing:border-box}html{scroll-behavior:smooth}
      body{font:15px system-ui,-apple-system,'Segoe UI',sans-serif;background:#f3f5f7;
           color:#263546;max-width:1440px;margin:0 auto;padding:12px}
      header,section{background:#fff;border-radius:14px;padding:18px;margin:12px 0 20px;
                     box-shadow:0 2px 12px #0000000b}
      h1{font-size:23px;margin:0 0 8px}h2{font-size:18px;margin:0}
      p{line-height:1.5}.meta{color:#526071}
      img{display:block;width:100%;height:auto;image-rendering:pixelated;border:1px solid #e0e5ec;border-radius:6px}
      .head{display:flex;justify-content:space-between;gap:16px;flex-wrap:wrap;align-items:center}
      .kind{font-size:13px;font-weight:600;color:#536375}
      .legend{background:#f7f9fb;padding:12px;border-radius:8px}
      @media(max-width:650px){body{padding:6px}header,section{padding:10px}h1{font-size:19px}}
      </style>"""
    intro="""<!doctype html><html lang="vi"><meta charset="utf-8">
       <meta name="viewport" content="width=device-width,initial-scale=1">
       <title>NR-G2R7-05 · Full map toàn run · Từng decision</title>"""+css+"""
       <header><h1>NR-G2R7-05 · Full map toàn run</h1>
       <p><b>Đây là bản full map mới:</b> mỗi ảnh đều là toàn bộ New Room,
       hiển thị trực tiếp trong một trang cuộn liên tục. Không có ảnh cắt D02,
       không có mục gấp/mở. Các khung bao gồm đủ 27 decision G1,
       các mốc đến Goal 1–4, các checkpoint trên đường và bản đồ cuối run.</p>
       <p class="legend"><b>Chú thích:</b> đường xanh lá là quỹ đạo robot đã
       thực sự đi; chấm xanh dương là vị trí hiện tại. G1–G4 màu cam trước khi
       robot thực sự tới, màu xanh lá sau khi có native ACK.
       Màu nền lấy từ observed map và predicted map tại epoch có sẵn,
       tuyệt đối không sử dụng GT trong lập kế hoạch.</p>
       <p><b>Run:</b> NR-G2R7-05 (G2-R7-R3.1).
       <b>Terminal:</b> %d steps.
       <b>Frames:</b> %d ảnh full map.</p></header>"""%(final+1,len(ticks))
    OUTPUT.write_text(intro+'\n'.join(sections)+'</html>')
    audit={"run_id":"NR-G2R7-05","type":"complete_full_map_scroll",
           "all_27_native_g1_decisions_included":True,
           "all_four_native_r7_goal_arrivals_included":True,
           "no_room_crops":True,
           "all_images_visible_without_expansion":True,
           "final_native_tick":final,
           "n_frames":len(ticks),"snapshots":snapshots}
    MANIFEST.write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
    text=OUTPUT.read_text()
    assert '<details' not in text and 'ROOM_ZOOM' not in text
    assert text.count('<section id=')==len(ticks)
    assert text.count('data:image/png;base64,')==len(ticks)
    print('OUTPUT',OUTPUT,'bytes',OUTPUT.stat().st_size)
    print('VERIFIED FULL MAP ONLY',len(ticks),'all_decisions',len(decisions),'terminal',final)

if __name__=='__main__':main()
