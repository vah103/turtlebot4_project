from pathlib import Path
import cv2,json,numpy as np
from PIL import Image,ImageDraw,ImageFont
root=Path('/home/dell/mx072_g2_r3_sequence_optimizer_20261016')
folder=root/'results_final'
plan=json.loads((folder/'R3_SEQUENCE_VP1.json').read_text())
summary=json.loads((folder/'R3_VP1_FROZEN_CHAIN_SENSOR_EVAL.json').read_text())
run=Path('/home/dell/mx072_native2d_pair_20261015/OPTIMIZED_FULL_PAIR_20261015/COM1_VERIFIED/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob')
original=np.load(run/'room_r41/inference/decision_000002_input.npz')['observed']
semantic=np.load(run/'room_r41/inference/decision_000002_semantic.npz')['semantic']
_,labels=cv2.connectedComponents(((semantic==1)&(original!=2)).astype('uint8'),connectivity=4)
mission=labels==labels[147,111]
obs=[np.load('/home/dell/mx072_g2_v1_r2_standalone_20261016/D02_MANUAL_G2_FULL_CHAIN_20261016/stage_01_postscan.npz')['observed']]
for i in [1,2,3]:obs.append(np.load(folder/f'R3_VP1_STAGE_{i:02d}_POSTSCAN.npz')['observed'])
truth=np.asarray(Image.open(run/'gt_map.png'))[500:703,500:763,0]==255
p=plan['selected']['stages']
route=[[(168,135)]]+[[(int(z[0]),int(z[1])) for z in leg['path_rc']] for leg in p]
F=lambda n,b=False:ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans'+('-Bold' if b else '')+'.ttf',n)
W,H=1760,1360
img=Image.new('RGB',(W,H),'#f8fafd');d=ImageDraw.Draw(img)
d.text((25,18),'G2-R3  /  VP1 → SELF-SELECTED 3-GOAL SEQUENCE',font=F(33,True),fill='#243e50')
d.text((26,62),'Offline GT-hidden LiDAR: 2,500 rays · scan each 0.5 m · ALL three goals frozen before simulation',font=F(17),fill='#607487')
r0,r1,c0,c1=140,203,89,217
px_w,px_h=808,440
colors=['#038667','#397bbb','#c8862d']
def scale(pt):
 y,x=pt
 return((x-c0+.5)*px_w/(c1-c0),(y-r0+.5)*px_h/(r1-r0))
for idx in range(4):
 x=25+(idx%2)*864;y=108+(idx//2)*612
 d.rounded_rectangle((x,y,x+842,y+580),radius=13,fill='white',outline='#bbc9d4',width=2)
 if idx==0:
  head='AFTER VP1 · before R3 sequence'
  sub='1,021 original-room unknown  ·  685 GT-free unknown (evaluator only)'
 else:
  s=summary['stages'][idx-1]
  head=f"ARRIVAL G{idx}  ·  {s['cumulative_distance_m']} m total"
  sub=f"+{s['new_original_room_observed']} observed  ·  {s['remaining_original_room_unknown']} unknown remain  ·  {s['remaining_GT_free_unknown_EVAL_ONLY']} GT-free remain"
 d.text((x+14,y+12),head,font=F(23,True),fill='#234456')
 d.text((x+14,y+51),sub,font=F(15),fill='#62798a')
 ob=obs[idx]
 rgb=np.full((*ob.shape,3),243,np.uint8)
 rgb[ob==0]=[255,255,255];rgb[ob==1]=[179,193,210];rgb[ob==2]=[59,70,84]
 rgb[~mission]=[242,243,245];rgb[(~mission)&(ob==2)]=[126,137,151]
 if idx>0:
  newly=(obs[idx-1]==1)&(ob!=1)&mission
  rgb[newly&(ob==0)]=[49,153,121]
  rgb[newly&(ob==2)]=[209,99,92]
 if idx==3:
  unknown=mission&(ob==1)
  rgb[unknown&~truth]=[245,179,71]  # GT free residual
  rgb[unknown&truth]=[198,83,96]   # GT wall residual
 contour,_=cv2.findContours(mission.astype('uint8'),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
 cv2.drawContours(rgb,contour,-1,(178,67,192),1)
 zoom=Image.fromarray(rgb[r0:r1,c0:c1]).resize((px_w,px_h),Image.Resampling.NEAREST)
 paint=ImageDraw.Draw(zoom)
 for k in range(idx):
  paint.line([scale(q) for q in route[k+1]],fill=colors[k],width=4,joint='curve')
  xx,yy=scale(route[k+1][-1]);paint.ellipse((xx-7,yy-7,xx+7,yy+7),fill=colors[k],outline='white',width=2)
  paint.text((xx+8,yy-20),f'G{k+1}',font=F(15,True),fill=colors[k],stroke_fill='white',stroke_width=2)
 xx,yy=scale((168,135));paint.ellipse((xx-8,yy-8,xx+8,yy+8),fill='#273c52',outline='white',width=2)
 paint.text((xx+10,yy-20),'VP1',font=F(14,True),fill='#263e51',stroke_fill='white',stroke_width=2)
 img.paste(zoom,(x+17,y+92))
d.text((27,1310),'Magenta: frozen D02 Room1 boundary. Colored segments: paths selected by R3; colored cells: NEW observations on the current leg.',font=F(16),fill='#536a7d')
d.text((27,1334),'FINAL GT EVAL ONLY: yellow = 4 still-unknown free pixels; red = 262 still-unknown wall pixels. Not source-native LiDAR or dynamic G3.',font=F(15,True),fill='#6f4a37')
file=folder/'R3_VP1_FROZEN_CHAIN_4STAGE_LIDAR.png'
img.save(file,optimize=True)
print(file,file.stat().st_size)
