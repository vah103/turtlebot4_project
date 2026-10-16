import os,cv2,json,numpy as np
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
B=Path('/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_MANUAL_G2_FULL_CHAIN_20261016')
RUN=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
f=np.load(RUN/'room_r41/inference/decision_000002_input.npz')
orig=f['observed'];states=[orig]
room=np.load(B/'stage_01_postscan.npz')['original_room']
truth=np.asarray(Image.open(RUN/'gt_map.png'))[500:703,500:763,0]==255
routes=[]
for n in range(1,5):
 z=np.load(B/f'stage_{n:02d}_postscan.npz')
 states.append(z['observed']);routes.append(z['route_rc'])
s=json.load(open(B/'fullchain_sensor_summary.json'));stages=s['stages']
F=lambda n,b=False:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if b else '')+'.ttf',n)
W,H=1875,1400
img=Image.new('RGB',(W,H),'#f7f9fb');draw=ImageDraw.Draw(img)
draw.text((33,21),'RUN 04  /  D02  →  VP1  →  G1  →  G2  →  G3',font=F(34,True),fill='#20394c')
draw.text((35,67),'Fully executed FROZEN viewpoint chain in offline LiDAR simulator · 2500 rays / 20m · scan every 0.5m',font=F(18),fill='#5a7081')
labels=[
('D02 original','3,889 unknown in Room1',0),
('After VP1 · 4.5m','2,868 new · 1,021 unknown left',1),
('After G1 · 9.4m','+98 new · 923 unknown left',2),
('After G2 · 20.6m','+589 new · 334 unknown left',3),
('After G3 · 34.2m','+78 new · 256 unknown left',4),
('Final GT evaluation','0 actual-free unknown  ·  256 wall unknown',5)
]
tints=[None,[53,161,127],[71,129,203],[57,159,216],[233,157,60]]
route_colors=['#09855e','#397abb','#ca7827','#8859b5']
y0,y1,x0,x1=135,203,83,221
for k,(title,sub,n) in enumerate(labels):
 xx=20+(k%3)*620;yy=110+(k//3)*615
 draw.rounded_rectangle((xx,yy,xx+598,yy+590),radius=15,fill='white',outline='#bac9d4',width=2)
 draw.text((xx+13,yy+14),title,font=F(25,True),fill='#223f52')
 draw.text((xx+13,yy+54),sub,font=F(17),fill='#597180')
 arr=states[min(n,4)]
 rgb=np.full((203,263,3),240,np.uint8)
 rgb[arr==0]=[255,255,255]
 rgb[arr==1]=[184,195,208]
 rgb[arr==2]=[54,65,78]
 rgb[~room]=[240,242,245];rgb[~room&(arr==2)]=[126,135,145]
 if n==5:
  unknown=room&(arr==1)
  rgb[unknown&truth]=[208,85,100]
  rgb[unknown&~truth]=[245,179,68]
 elif n>0:
  for i in range(n):
   changed=room&(states[i]==1)&(states[i+1]!=1)
   rgb[changed]=tints[i+1]
 contour,_=cv2.findContours(room.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 cv2.drawContours(rgb,contour,-1,(179,62,190),1)
 if n>=1:
  last_sem=B/f'stage_{min(n,4):02d}_semantic.npz'
  if last_sem.exists():
   sem=np.load(last_sem)['semantic']
   pose=tuple(routes[min(n,4)-1][-1])
   _,lab=cv2.connectedComponents(((sem==1)&(arr!=2)).astype('uint8'),connectivity=4)
   if lab[pose]!=0:
    component=lab==lab[pose]
    cont,_=cv2.findContours(component.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(rgb,cont,-1,(40,144,203),1)
 src=Image.fromarray(rgb[y0:y1,x0:x1]).resize((566,452),Image.Resampling.NEAREST)
 p=ImageDraw.Draw(src)
 def xy(rc):
  y,x=rc
  return ((x-x0+.5)*566/(x1-x0),(y-y0+.5)*452/(y1-y0))
 if n>0:
  for i,path in enumerate(routes[:min(n,4)]):
   p.line([xy(q) for q in path],fill=route_colors[i],width=4,joint='curve')
 for i,rc in enumerate([(147,111),(168,135),(180,102),(192,174),(147,207)]):
  if n!=5 and i>n:continue
  px,py=xy(rc)
  color='#253d52' if i==0 else route_colors[i-1]
  p.ellipse((px-7,py-7,px+7,py+7),fill=color,outline='white',width=2)
  p.text((px+7,py-19),('R' if i==0 else ('VP1' if i==1 else 'G'+str(i-1))),
         font=F(13,True),fill=color,stroke_fill='white',stroke_width=2)
 img.paste(src,(xx+16,yy+104))
draw.text((29,1350),'Magenta: fixed D02 Room1 boundary. Blue outline: updated Room Node, if available. Colored trails: LiDAR-realized observations per stage.',font=F(15),fill='#556a7b')
draw.text((29,1374),'Final GT truth ONLY for evaluation: red = unknown wall; yellow = unknown free (none). No GT used to choose viewpoints.',font=F(15,True),fill='#314c60')
out=B/'D02_VP1_G1_G2_G3_CAUSAL_6PANELS.png'
img.save(out,optimize=True)
print(out,out.stat().st_size)
