"""Diagnostic images only: no GT, no future observations, no invented goal routes."""
from pathlib import Path
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont

def f(n,bold=False):
 for path in ['/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if bold else '')+'.ttf']:
  try:return ImageFont.truetype(path,n)
  except OSError:continue
 return ImageFont.load_default()

def visualize(obs,mean,current,memory,labels,report,path):
 original=memory.initial;mission=memory.mission
 h,w=obs.shape
 ys,xs=np.where(original|mission|current)
 pad_y,pad_x=9,10
 r0=max(0,int(ys.min())-pad_y);r1=min(h,int(ys.max())+pad_y+1)
 c0=max(0,int(xs.min())-pad_x);c1=min(w,int(xs.max())+pad_x+1)
 view_height=r1-r0;view_width=c1-c0
 def map_panel(which):
  out=np.full((h,w,3),242,np.uint8)
  out[obs==0]=[253,253,253]
  out[obs==1]=[186,197,210]
  out[obs==2]=[56,68,82]
  outside=(obs==1)&mission&~current
  if which=='structure':
   out[outside]=[246,170,81]
   free=(obs==1)&mission&(mean<.5)
   wall=(obs==1)&mission&(mean>=.5)
   out[free&current]=[134,209,169]
   out[wall&current]=[205,99,105]
  else:
   out[outside]=[244,185,94]
  c,_=cv2.findContours(original.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  cv2.drawContours(out,c,-1,(183,63,185),1)
  c,_=cv2.findContours(current.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  cv2.drawContours(out,c,-1,(23,140,204),1)
  return Image.fromarray(out[r0:r1,c0:c1])

 W,H=1610,790
 image=Image.new('RGB',(W,H),'#f8fafb');d=ImageDraw.Draw(image)
 d.text((27,19),'G2-R3  /  STRUCTURAL REASONING + ROOM MISSION MEMORY',font=f(27,True),fill='#233c50')
 d.text((29,57),f"Decision: {report['decision_id']}  |  Snapshot only: observed + LaMa + R4.1, no GT",font=f(16),fill='#587083')
 for k,(title,sub,panel) in enumerate([
  ('Evidence + Room Node','Magenta: original mission   Blue: current accepted room',map_panel('observed')),
  ('Unresolved structural regions','Green: possible free   Red: unverified predicted wall   Orange: historic obligation',map_panel('structure'))]):
  x=23+k*798;y=106
  d.rounded_rectangle((x,y,x+778,y+557),radius=15,fill='white',outline='#c3ced7',width=2)
  d.text((x+13,y+12),title,font=f(23,True),fill='#284357')
  d.text((x+13,y+48),sub,font=f(13),fill='#617888')
  crop=panel.resize((752,463),Image.Resampling.NEAREST)
  image.paste(crop,(x+13,y+81))
  overlay=ImageDraw.Draw(image)
  scale_x=752/view_width;scale_y=463/view_height
  pos=report['robot_core_rc'];yy,xx=pos
  rx=x+13+(xx-c0+.5)*scale_x;ry=y+81+(yy-r0+.5)*scale_y
  overlay.ellipse((rx-9,ry-9,rx+9,ry+9),fill='#163f5f',outline='white',width=3)
  if k==1:
   for row in sorted(report['regions'],key=lambda z:-z['cells'])[:15]:
    r0,c0,r1,c1=row['bbox_rc']
    cy=(r0+r1)/2;cx=(c0+c1)/2
    cxp=x+13+(cx-c0+.5)*scale_x;cyp=y+81+(cy-r0+.5)*scale_y
    col='#4a5549' if row['kind']=='POSSIBLE_OPEN_SPACE' else '#922c35'
    overlay.text((cxp+3,cyp-8),str(row['region_id']),font=f(17,True),fill=col,stroke_fill='white',stroke_width=3)
 y=680
 d.text((31,y),f"Mission unknown: {report['mission_unknown']:,}   |   Current room unknown: {report['current_room_unknown']:,}   |   Outside new boundary (retained): {report['unknown_outside_current_room']:,}",font=f(20,True),fill='#263f52')
 d.text((31,y+37),f"Possible-free unknown: {report['predicted_free_like_unknown']:,}   |   Predicted-wall UNKNOWN: {report['unverified_predicted_wall_unknown']:,}   |   Regions: {len(report['regions'])}",font=f(17),fill='#465e71')
 d.text((31,755),'No region is cleared by a predicted wall or Room Node reclassification. Candidate viewpoints in JSON are witnesses, not a planned goal sequence.',font=f(14),fill='#695e47')
 Path(path).parent.mkdir(parents=True,exist_ok=True)
 image.save(path,optimize=True)
