#!/usr/bin/env python3
"""GT-only retrospective D18 evaluation and original-room scroll images.
Never invoked by online G2-R3.1 planner; run AFTER causal test finishes."""
from pathlib import Path
import json,cv2,numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path('/home/dell/mx072_g2_r31_structure_commitment_20261016')
P=ROOT/'D18_R31_CAUSAL_20261016'
F=P/'rollout'
SRC=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
INF=SRC/'room_r41/inference'
summary=json.loads((F/'summary.json').read_text())
events=summary['events']
original=np.load(INF/'decision_000018_input.npz')['observed']
semantic0=np.load(INF/'decision_000018_semantic.npz')['semantic']
_,labels=cv2.connectedComponents(((semantic0==1)&(original!=2)).astype('uint8'),connectivity=4)
room=labels==labels[53,40]
gt=np.asarray(Image.open(SRC/'gt_map.png'))[500:703,500:763,0]==255
observed=original if not events else np.load(F/f"stage_{events[-1]['stage']:02d}_postscan.npz")['observed']
unknown_initial=room&(original==1)
unknown_final=room&(observed==1)
truth_free=unknown_final&~gt;truth_wall=unknown_final&gt
assert unknown_initial.sum()==833
ev={
 'source':'GT-only retrospective, never supplied to G2-R3.1/LaMa/R4.1',
 'D18_original_room_cells':int(room.sum()),
 'original_unknown':int(unknown_initial.sum()),
 'initial_unknown_real_free':int((unknown_initial&~gt).sum()),
 'initial_unknown_real_wall':int((unknown_initial&gt).sum()),
 'remaining_unknown':int(unknown_final.sum()),
 'remaining_unknown_real_free':int(truth_free.sum()),
 'remaining_unknown_real_wall':int(truth_wall.sum()),
 'previously_unknown_real_free_observed':int((unknown_initial&~gt&(observed!=1)).sum()),
 'observed_unknown_fraction':round(1-int(unknown_final.sum())/833,6),
 'real_free_unknown_recall':round(1-int(truth_free.sum())/max(1,int((unknown_initial&~gt).sum())),6),
 'room_free_space_complete_GT_ONLY':int(truth_free.sum())==0,
 'termination':summary['termination'],
 'remaining_unknown_clusters':[]}
n,lb,stats,cc=cv2.connectedComponentsWithStats(unknown_final.astype('uint8'),connectivity=8)
for k in range(1,n):
 x,y,w,h,size=[int(t) for t in stats[k]]
 m=lb==k
 ev['remaining_unknown_clusters'].append({'pixels':size,'bbox_rc':[y,x,y+h-1,x+w-1],
   'true_free_GT_only':int((m&~gt).sum()),'true_wall_GT_only':int((m&gt).sum())})
ev['remaining_unknown_clusters'].sort(key=lambda x:-x['pixels'])
(P/'D18_GT_ONLY_EVALUATION.json').write_text(json.dumps(ev,indent=2))
print('GT_EVAL',json.dumps({k:v for k,v in ev.items() if k!='remaining_unknown_clusters'}),flush=True)
font=lambda size,bold=False:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if bold else '')+'.ttf',size)
routes=[]
for i,event in enumerate(events):
 if i==0:
  j=json.loads((P/'initial/R31_SEQUENCE_D18.json').read_text())
  route=j['selected']['stages'][0]['path_rc']
 else:
  j=json.loads((F/f'stage_{i:02d}_r31_decision.json').read_text())
  route=j['selected_next_leg']['path_rc']
 assert tuple(route[-1])==tuple(event['robot_goal_core_rc'])
 routes.append([tuple(q) for q in route])
inds=np.argwhere(room)
for route in routes:
 inds=np.concatenate((inds,np.asarray(route,dtype=int)),axis=0)
r0,c0=inds.min(axis=0);r1,c1=inds.max(axis=0)
r0=max(0,r0-8);r1=min(203,r1+9);c0=max(0,c0-8);c1=min(263,c1+9)
maxw,maxh=825,700
zoomw=min(maxw,int((c1-c0)/(r1-r0)*maxh))
zoomh=min(maxh,int((r1-r0)/(c1-c0)*maxw))
def pxy(pt):
 r,c=pt
 return ((c-c0+.5)*zoomw/(c1-c0),(r-r0+.5)*zoomh/(r1-r0))
past=[original]+[np.load(F/f"stage_{i:02d}_postscan.npz")['observed'] for i in range(1,len(events)+1)]
colors=['#12886c','#3173b6','#da8833','#ac54a3','#769256','#9563b5','#a65a37','#207d8a','#bd4665','#4572a1']
def render(index,posthoc=False):
 obs=past[index]
 robot=(53,40) if index==0 else tuple(events[index-1]['robot_goal_core_rc'])
 image=np.full((*obs.shape,3),241,np.uint8)
 image[obs==0]=[252,253,255];image[obs==1]=[183,198,214];image[obs==2]=[57,67,79]
 image[~room]=[239,241,244];image[(~room)&(obs==2)]=[122,133,144]
 if index:
  delta=(room&(past[index-1]==1)&(obs!=1))
  image[delta&(obs==0)]=[48,162,120]
  image[delta&(obs==2)]=[210,109,96]
 if posthoc:
  image[truth_free]=[247,186,52]
  image[truth_wall]=[212,69,99]
 contour,_=cv2.findContours(room.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 cv2.drawContours(image,contour,-1,(176,63,186),1)
 if index:
  sem_path=F/f'room_stage_{18+index:02d}.npz'
  if sem_path.exists():
   sem=np.load(sem_path)['semantic']
   _,lc=cv2.connectedComponents(((sem==1)&(obs!=2)).astype('uint8'),connectivity=4)
   if lc[robot]:
    cr=lc==lc[robot]
    ct,_=cv2.findContours(cr.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image,ct,-1,(26,142,205),1)
 im=Image.fromarray(image[r0:r1,c0:c1]).resize((zoomw,zoomh),Image.Resampling.NEAREST)
 pen=ImageDraw.Draw(im)
 for j in range(index):
  pen.line([pxy(v) for v in routes[j]],fill=colors[j%len(colors)],width=4,joint='curve')
  x,y=pxy(routes[j][-1]);pen.ellipse((x-7,y-7,x+7,y+7),fill=colors[j%len(colors)],outline='white',width=2)
  pen.text((x+8,y-22),f'G{j+1}',font=font(15,True),fill=colors[j%len(colors)],stroke_width=3,stroke_fill='white')
 x,y=pxy((53,40));pen.ellipse((x-7,y-7,x+7,y+7),fill='#102f51',outline='white',width=2)
 x,y=pxy(robot);pen.ellipse((x-7,y-7,x+7,y+7),fill='#102f51',outline='white',width=2)
 W=1100;H=990
 card=Image.new('RGB',(W,H),'#fcfdff');d=ImageDraw.Draw(card)
 title=('D18 - ORIGINAL INPUT' if index==0 else f'D18 - AFTER GOAL {index}') if not posthoc else 'FINAL GT EVALUATION ONLY'
 d.text((28,22),title,font=font(33,True),fill='#294658')
 if index:
  evt=events[index-1]
  msg=f"Goal ({evt['goal_xy_m'][0]:.1f}, {evt['goal_xy_m'][1]:.1f})m  /  {evt['distance_cumulative_m']:.1f}m  /  {evt['scans_cumulative']} scans"
  detail=f"New room unknown observed: {evt['newly_scanned_original_room']} | Unknown remain: {evt['post_scan_original_room_unknown']}"
 else:
  msg='Start robot (4.0, 5.3)m'
  detail='Original D18 Room 4: 833 unknown cells, 4,039 room cells'
 d.text((30,75),msg,font=font(20),fill='#516879')
 d.text((30,106),detail,font=font(21,True),fill='#526779')
 card.paste(im,(40,158))
 d.text((40,889),'Purple: frozen D18 Room4 | Blue: current Room Node | Green/red: newly observed free/wall',font=font(17),fill='#536b7e')
 if posthoc:d.text((40,925),f"GT ONLY: remaining unknown = {ev['remaining_unknown_real_free']} free + {ev['remaining_unknown_real_wall']} walls",font=font(20,True),fill='#8b3c44')
 else:d.text((40,923),'No future information used in goal selection; sensor scans are synthetic raster LiDAR.',font=font(16),fill='#5e7285')
 return card
images=[]
for i in range(len(events)+1):
 img=render(i)
 name=f'D18_{i:02d}_'+('INPUT' if i==0 else f'GOAL_{i:02d}')+'.png'
 img.save(P/name,optimize=True);images.append(name)
img=render(len(events),posthoc=True);img.save(P/'D18_FINAL_GT_ONLY.png',optimize=True);images.append('D18_FINAL_GT_ONLY.png')
head=['<!DOCTYPE html><html lang="vi"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">',
'<title>D18 G2-R3.1 causal exploration</title>',
'<style>body{font-family:system-ui,sans-serif;max-width:1160px;margin:0 auto;padding:15px;background:#f5f7fa;color:#254059}section{margin:16px 0;background:#fff;padding:15px;border-radius:12px;border:1px solid #d1dee8}h1{font-size:25px}h2{font-size:19px}img{width:100%;height:auto;max-width:1100px}p{line-height:1.5;color:#536b7f}table{width:100%;border-collapse:collapse}th,td{border-bottom:1px solid #dde5eb;text-align:left;padding:8px}</style></head><body>',
'<h1>G2-R3.1 — Room4 tại Decision 18, Run04</h1>',
'<p>Thuật toán khóa nguyên phiên bản từ D02. Sử dụng observed + LaMa + Room Node D18; robot đi theo chuỗi có cam kết, quét LiDAR mỗi 0,5m, chạy lại LaMa/R4.1 và chỉ tính lại goal khi chuỗi hết hiệu lực. GT chỉ dùng trong cảm biến ẩn và đánh giá sau cùng.</p>',
f"<p><b>End status:</b> {summary['termination']} · <b>Goals:</b> {len(events)} · <b>Distance:</b> {summary['route_distance_m']} m · <b>Scans:</b> {summary['sensor_scan_count']} · <b>Room unknown:</b> {summary['original_room_unknown_left']}.</p>",
'<section><h2>Goal decisions</h2><table><tr><th>#</th><th>Actual goal xy</th><th>Distance</th><th>Observed room unknown</th><th>Decision following scan</th></tr>']
for e in events:
 head.append(f"<tr><td>{e['stage']}</td><td>{e['goal_xy_m']}</td><td>{e['distance_cumulative_m']} m</td><td>{e['post_scan_original_room_unknown']}</td><td>{e.get('post_replan_status','-')}</td></tr>")
head.append('</table></section>')
for im in images:head.append(f'<section><h2>{im}</h2><img loading="lazy" src="{im}" alt="{im}"></section>')
head.append('<p>GT-only panel is retrospective evaluation; never an input to G2/LaMa/Room Node. Synthetic visibility not identical to MapEx native LiDAR. No claim of globally shortest exploration or on-robot safety.</p></body></html>')
(P/'D18_R31_CAUSAL_SCROLL.html').write_text('\n'.join(head),encoding='utf8')
print('RENDERED',len(images),'cards',flush=True)
