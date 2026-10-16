#!/usr/bin/env python3
"""Post-hoc evaluator and visualizer. This runs ONLY after the G2-R3 rollout finished.
GT is read only here to classify already-frozen unresolved unknown; no goal planning.
"""
from pathlib import Path
import json,cv2,numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path('/home/dell/mx072_g2_r3_sequence_optimizer_20261016')
DIR=ROOT/'D02_AUTONOMOUS_CAUSAL_ROLLOUT_20261016'
RUN=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
info=json.loads((DIR/'summary.json').read_text())
events=info['events']
D02=np.load(RUN/'room_r41/inference/decision_000002_input.npz')['observed']
sem0=np.load(RUN/'room_r41/inference/decision_000002_semantic.npz')['semantic']
_,ll=cv2.connectedComponents(((sem0==1)&(D02!=2)).astype('uint8'),connectivity=4)
original=(ll==ll[147,111])
GT=np.asarray(Image.open(RUN/'gt_map.png'))[500:703,500:763,0]==255
assert int((original&(D02==1)).sum())==3889
if events:
 final=np.load(DIR/f"stage_{events[-1]['stage']:02d}_postscan.npz")['observed']
else:final=D02
unknown=original&(final==1)
unknownfree=unknown&~GT
unknownwall=unknown&GT
initial=original&(D02==1)
assessment={
  'evaluation_scope':'GT retrospective only; GT inaccessible to structural/G2/R4/LaMa path chooser',
  'source':'native Run04 gt_map.png core crop, raster wall==255',
  'original_room_initial_unknown':int(initial.sum()),
  'original_unknown_true_free_initial_GT_ONLY':int((initial&~GT).sum()),
  'original_unknown_true_wall_initial_GT_ONLY':int((initial&GT).sum()),
  'original_room_unknown_left':int(unknown.sum()),
  'original_room_true_free_unknown_left_GT_ONLY':int(unknownfree.sum()),
  'original_room_true_wall_unknown_left_GT_ONLY':int(unknownwall.sum()),
  'original_room_newly_observed_true_free_GT_ONLY':int((initial&~GT&(final!=1)).sum()),
  'true_free_unknown_recall_GT_ONLY':round(1-int(unknownfree.sum())/max(1,int((initial&~GT).sum())),6),
  'full_original_room_free_exploration_GT_ONLY':bool(int(unknownfree.sum())==0),
  'observation_all_initial_room_cells':bool(int(unknown.sum())==0),
  'rollout_terminal_reason':info['termination'],
  'original_room_unknown_clusters':[]}
n,label,stats,cent=cv2.connectedComponentsWithStats(unknown.astype('uint8'),8)
for k in range(1,n):
 x,y,w,h,sz=map(int,stats[k])
 region=label==k
 assessment['original_room_unknown_clusters'].append({
 'cells':sz,'bbox_core_rc':[y,x,y+h-1,x+w-1],
 'true_free_evaluation_only':int((region&~GT).sum()),
 'true_wall_evaluation_only':int((region&GT).sum())})
assessment['original_room_unknown_clusters'].sort(key=lambda a:-a['cells'])
(DIR/'GT_RETROSPECTIVE_FINAL_EVALUATION.json').write_text(json.dumps(assessment,indent=2))
print('GT_FINAL',json.dumps({k:v for k,v in assessment.items() if k!='original_room_unknown_clusters'},indent=2),flush=True)

def font(size,b=False):
 return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if b else '')+'.ttf',size)
r0,r1,c0,c1=136,203,83,221
pw,ph=990,482
colors=['#047d63','#3778bd','#ca842f','#9e559e','#338f96','#bf685a','#7361b4','#9e793b','#4b857a','#b34b63','#507eb5','#a25887']
def xy(p):
 r,c=p;return ((c-c0+.5)*pw/(c1-c0),(r-r0+.5)*ph/(r1-r0))
routes=[]
for i,event in enumerate(events):
 p=(ROOT/'results_final/R3_SEQUENCE_D02.json') if i==0 else (DIR/f'stage_{i:02d}_g2r3_plan.json')
 plan=json.loads(p.read_text())
 first=plan['selected']['stages'][0]['path_rc']
 assert tuple(first[-1])==tuple(event['robot_goal_core_rc'])
 routes.append([tuple(q) for q in first])
frames=[('D02_ORIGINAL',D02,(147,111),None,None)]
for event in events:
 i=event['stage']
 obs=np.load(DIR/f'stage_{i:02d}_postscan.npz')['observed']
 semantic=np.load(DIR/f'room_stage_{i+2:02d}.npz')['semantic'] if (DIR/f'room_stage_{i+2:02d}.npz').exists() else None
 frames.append((f'AFTER_GOAL_{i:02d}',obs,tuple(event['robot_goal_core_rc']),semantic,event))
def panel(name,obs,robot,semantic,event,truth_final=False):
 image=np.full((*obs.shape,3),240,dtype=np.uint8)
 image[obs==0]=[253,253,253];image[obs==1]=[182,194,211];image[obs==2]=[54,65,82]
 image[~original]=[244,245,246];image[~original&(obs==2)]=[122,130,140]
 if event is not None:
  st=event['stage'];before=frames[st-1][1]
  fresh=(original&(before==1)&(obs!=1))
  image[fresh&(obs==0)]=[55,169,132]
  image[fresh&(obs==2)]=[224,112,96]
 if truth_final:
  seen=original&(obs==1)
  image[seen&~GT]=[237,178,58]
  image[seen&GT]=[195,74,99]
 con,_=cv2.findContours(original.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 cv2.drawContours(image,con,-1,(169,60,183),1)
 if semantic is not None:
  _,components=cv2.connectedComponents(((semantic==1)&(obs!=2)).astype('uint8'),connectivity=4)
  if components[robot]!=0:
   active=components==components[robot]
   cnt,_=cv2.findContours(active.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
   cv2.drawContours(image,cnt,-1,(29,142,201),1)
 z=Image.fromarray(image[r0:r1,c0:c1]).resize((pw,ph),Image.Resampling.NEAREST)
 pen=ImageDraw.Draw(z)
 stage=event['stage'] if event else 0
 for i,route in enumerate(routes[:stage]):
  pen.line([xy(x) for x in route],fill=colors[i%len(colors)],width=5,joint='curve')
  cx,cy=xy(route[-1])
  pen.ellipse((cx-8,cy-8,cx+8,cy+8),fill=colors[i%len(colors)],outline='white',width=3)
  pen.text((cx+9,cy-20),f'G{i+1}',font=font(16,True),fill=colors[i%len(colors)],stroke_width=3,stroke_fill='white')
 cx,cy=xy((147,111));pen.ellipse((cx-8,cy-8,cx+8,cy+8),fill='#223a53',outline='white',width=2)
 cx,cy=xy(robot);pen.ellipse((cx-7,cy-7,cx+7,cy+7),fill='#192d41',outline='white',width=2)
 W,H=1070,740
 card=Image.new('RGB',(W,H),'#f8fafd');d=ImageDraw.Draw(card)
 d.text((23,14),name.replace('_',' '),font=font(28,True),fill='#213d50')
 if event:
  sub=f"Robot ({robot[1]*.1:.1f}, {robot[0]*.1:.1f})m  |  Total {event['distance_cumulative_m']:.1f}m  |  {event['scans_cumulative']} scans"
  rem=f"Original Room1 unknown: {event['post_scan_original_room_unknown']}  |  New scanned this leg: {event['newly_scanned_original_room']}"
 else:
  sub='Robot (11.1, 14.7)m  |  Initial Run04 D02'
  rem='Original Room1 unknown: 3,889'
 d.text((26,57),sub,font=font(18),fill='#4f667b')
 d.text((26,87),rem,font=font(18,True),fill='#4e6576')
 card.paste(z,(35,128))
 d.text((36,633),'Magenta: fixed D02 Room1  |  Blue: current R4.1  |  Green/red: newly scanned FREE/WALL',font=font(16),fill='#4e6b7c')
 if truth_final:
  d.text((36,667),f"GT evaluator ONLY: {int(unknownfree.sum())} unobserved FREE / {int(unknownwall.sum())} unobserved WALL",font=font(19,True),fill='#9a3b45')
 else:
  d.text((36,667),'All filled colors are simulated LiDAR observation; this is NOT a predicted coverage map',font=font(15),fill='#69788a')
 return card
gallery=[]
for name,obs,pose,semantic,event in frames:
 im=panel(name,obs,pose,semantic,event)
 path=DIR/f'{name}.png';im.save(path,optimize=True)
 gallery.append((name,path.name,event))
if frames:
 name,obs,pose,semantic,event=frames[-1]
 evalim=panel('FINAL_HINDSIGHT_GT_ONLY',obs,pose,semantic,event,truth_final=True)
 evalim.save(DIR/'FINAL_HINDSIGHT_GT_ONLY.png',optimize=True)
 gallery.append(('FINAL_HINDSIGHT_GT_ONLY','FINAL_HINDSIGHT_GT_ONLY.png',None))
html=['<!doctype html><html lang="vi"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">',
'<title>G2-R3: D02 fully causal offline exploration</title>',
'<style>body{font-family:system-ui,-apple-system,sans-serif;margin:0 auto;max-width:1130px;padding:14px;color:#253f53;background:#f5f7fa}h1{font-size:25px}h2{font-size:19px}section{background:white;border:1px solid #d4dee6;border-radius:12px;margin:16px 0;padding:14px}img{max-width:100%;width:100%;height:auto}p{line-height:1.55;color:#5a7180}table{border-collapse:collapse;width:100%}th,td{border-bottom:1px solid #dbe3ea;text-align:left;padding:8px}</style></head><body>',
'<h1>G2-R3 — Tự khám phá liên tục từ D02 (offline, không dùng goal thủ công)</h1>',
'<p>Robot sử dụng goal D02 do G2-R3 tự chọn, quét LiDAR từ môi trường GT ẩn, sau mỗi goal tính lại LaMa ba mô hình, Room Node V4-R4.1 và lập lại chuỗi mới. Không dùng GT để chọn goal. Bản đồ GT trong phần đánh giá cuối chỉ xuất hiện sau khi quá trình chạy kết thúc.</p>',
f'<p><b>Stopped:</b> {info["termination"]} · <b>{info["completed_goals"]} goal(s)</b> · {info["route_distance_m"]} m · {info["sensor_scan_count"]} scans · original room unknown left {info["original_room_unknown_left"]}.</p>',
'<section><h2>Chuỗi goal đã thực sự đi qua</h2><table><tr><th>#</th><th>Goal (x,y)m</th><th>Đường tích lũy</th><th>Unknown trong Room1 cũ</th><th>Next G2 plan</th></tr>']
for e in events:
 html.append(f'<tr><td>{e["stage"]}</td><td>{e["goal_xy_m"]}</td><td>{e["distance_cumulative_m"]} m</td><td>{e["post_scan_original_room_unknown"]}</td><td>{e.get("post_replan_goals",[])}</td></tr>')
html.append('</table></section>')
for name,path,event in gallery:
 html.append(f'<section><h2>{name.replace("_"," ")}</h2><img loading="lazy" src="{path}" alt="{name}"></section>')
html.extend(['<p>Giới hạn: raycast 2D trên GT raster, không phải LiDAR MapEx source-native; replanning sau mỗi goal chứ không phải mỗi scan; chưa chứng minh an toàn vật lý hay toàn cục tối ưu.</p></body></html>'])
(DIR/'D02_R3_FULL_AUTONOMOUS_SCROLL.html').write_text('\n'.join(html),encoding='utf-8')
print('RENDER',len(gallery),'images + html; stages',len(events),flush=True)
