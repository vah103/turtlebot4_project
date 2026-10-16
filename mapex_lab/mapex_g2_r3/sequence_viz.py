"""Visualization of a CURRENT G2-R3 conditional sequence; no GT and no false actual-coverage colors."""
from pathlib import Path
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont

def _f(size,bold=False):
 try:
  return ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if bold else '')+'.ttf',size)
 except OSError:return ImageFont.load_default()

def make_route_image(obs,memory,structural,plan,robot,out):
 h,w=obs.shape;Mission=memory.mission
 ys,xs=np.where(Mission)
 if not len(ys):return
 y0=max(0,int(ys.min())-6);y1=min(h,int(ys.max())+7)
 x0=max(0,int(xs.min())-7);x1=min(w,int(xs.max())+8)
 rgb=np.full((h,w,3),245,np.uint8)
 rgb[obs==0]=[252,252,253];rgb[obs==1]=[190,202,214];rgb[obs==2]=[59,70,82]
 rgb[(obs==1)&Mission&~((obs==0)|(obs==2))]=[186,198,214]
 rgb[(obs==1)&Mission&~(memory.initial)]=[240,193,127]
 rids={int(r['region_id']):r for r in structural['regions']}
 for rid in plan.get('regions',[]):
  id_=rid['region_id'];m=(memory.labels==id_)&(obs==1)
  if rids[id_]['kind']=='POSSIBLE_OPEN_SPACE':rgb[m]=[149,207,175]
  else:rgb[m]=[208,128,126]
 contour,_=cv2.findContours(memory.initial.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 cv2.drawContours(rgb,contour,-1,(183,67,197),1)
 maxwidth=1150;maxheight=660
 sx=maxwidth/(x1-x0);sy=maxheight/(y1-y0)
 base=Image.fromarray(rgb[y0:y1,x0:x1]).resize((maxwidth,maxheight),Image.Resampling.NEAREST)
 d=ImageDraw.Draw(base)
 def xy(pt):return ((pt[1]-x0+.5)*sx,(pt[0]-y0+.5)*sy)
 colors=['#007e62','#2c73b7','#d08728','#ad5b8a','#6b59b6','#b26938','#1a8a93']
 if plan.get('selected'):
  for k,leg in enumerate(plan['selected']['stages']):
   pts=[xy(q) for q in leg['path_rc']]
   d.line(pts,fill=colors[k%len(colors)],width=5,joint='curve')
   xx,yy=xy(leg['goal_rc'])
   d.ellipse((xx-9,yy-9,xx+9,yy+9),fill=colors[k%len(colors)],outline='white',width=3)
   d.text((xx+11,yy-25),'G'+str(k+1),font=_f(17,True),fill=colors[k%len(colors)],stroke_fill='white',stroke_width=3)
 xx,yy=xy(robot)
 d.ellipse((xx-9,yy-9,xx+9,yy+9),fill='#162f46',outline='white',width=3)
 d.text((xx+11,yy-25),'R',font=_f(17,True),fill='#162f46',stroke_fill='white',stroke_width=3)
 W,H=1260,1025
 outim=Image.new('RGB',(W,H),'#f8f9fb');paint=ImageDraw.Draw(outim)
 paint.text((31,18),'G2-R3  /  STRUCTURAL CONDITIONAL GOAL SEQUENCE',font=_f(27,True),fill='#223f52')
 paint.text((32,57),'Current snapshot only: observed + LaMa + R4.1 + persistent Room Mission. GT NEVER used in planning.',font=_f(16),fill='#5b7183')
 outim.paste(base,(40,116))
 paint=ImageDraw.Draw(outim)
 if plan.get('selected'):
  paint.text((31,817),f"Planned goals: {plan['goal_count']}  |  observed-free grid route {plan['total_route_m']} m  |  major structural regions {plan['major_regions']}",font=_f(23,True),fill='#233f52')
  for i,g in enumerate(plan['goal_sequence_xy_m']):
   paint.text((32,854+i*23),f"G{i+1}: ({g[0]:.1f}, {g[1]:.1f}) m",font=_f(17),fill='#46677b')
 else:
  paint.text((31,821),'Status: '+plan['status'],font=_f(23,True),fill='#78492d')
 paint.text((32,972),'Green: possible-free structural obligations; red: unverified predicted obstacles; tan: retained mission extension.',font=_f(15),fill='#526d7f')
 paint.text((32,998),'Routes are CONDITIONAL, current observed-free raster paths; not robot-native navigation or observed future coverage.',font=_f(14),fill='#566a7a')
 Path(out).parent.mkdir(parents=True,exist_ok=True)
 outim.save(out,optimize=True)
