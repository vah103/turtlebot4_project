from pathlib import Path
import numpy as np,json
ROOT=Path('/home/dell/DELL_NATIVE2D/runs')
SETS={
 'RN04':ROOT/'NR-RN-04/native_opt_r41_lama3small_run04/R41_OPT_LAMA3_SMALLROI_RUN04_visvarprob',
 'R62':ROOT/'NR-G2R62-03/native_g2_r62_general_r2_run01/G2_R62_GENERAL_R2_RUN01_visvarprob'}
data={}
for name,p in SETS.items():
 od=np.load(str(p/'odom.npy')).astype(float)
 cum=np.r_[0,np.cumsum(np.linalg.norm(np.diff(od,axis=0),axis=1)*.1)]
 dec=[json.loads(f.read_text()) for f in sorted((p/'room_r41/decisions').glob('decision_??????.json'))]
 local=[]
 for idx,ev in enumerate(dec):
  if ev['mode']!='LOCAL' or (idx>0 and dec[idx-1]['mode']=='LOCAL' and dec[idx-1].get('room_id')==ev['room_id']):
   continue
  subsequent=next((x for x in dec[idx+1:] if x['mode']!='LOCAL' or x.get('room_id')!=ev['room_id']),None)
  if subsequent:
   start=int(ev['native_tick']);end=int(subsequent['native_tick'])
   local.append({'room_id':ev['room_id'],'entry_tick':start,'exit_tick':end,'entry_pose':list(od[start].astype(int)),
                 'exit_pose':list(od[end].astype(int)), 'inside_m':round(float(cum[end]-cum[start]),6),
                 'exit_status':subsequent['mode'],'exit_reference_decision':subsequent['decision']})
  else:
   local.append({'room_id':ev['room_id'],'entry_tick':ev['native_tick'],'exit_tick':None,'unfinished':True})
 glob=[]
 for i,x in enumerate(local):
  if x.get('exit_tick') is None:continue
  nxt=local[i+1] if i+1<len(local) else None
  if nxt:
   glob.append({'after_room_id':x['room_id'],'from_tick':x['exit_tick'],'to_tick':nxt['entry_tick'],
                'meters':round(float(cum[int(nxt['entry_tick'])]-cum[int(x['exit_tick'])]),6),
                'next_room_id':nxt['room_id']})
 # Actual per-tick segments label using most recent RoomNode classification.
 observed_local=0
 for i in range(len(od)-1):
  recent=max((j for j,e in enumerate(dec) if e['native_tick']<=i),default=0)
  if dec[recent]['mode']=='LOCAL':observed_local+=cum[i+1]-cum[i]
 record={'name':name,'total_m':float(cum[-1]),'local_mode_travel_m':round(float(observed_local),6),
         'global_mode_travel_m':round(float(cum[-1]-observed_local),6),
         'rooms':local,'global_connections':glob}
 data[name]=record
 print('\n',name,'TOTAL',round(record['total_m'],3),'LOCAL',record['local_mode_travel_m'],'GLOBAL',record['global_mode_travel_m'])
 for x in local:print(' ROOM',x)
 for x in glob:print(' CONN',x)
out=Path('/home/dell/DELL_NATIVE2D/analysis/g2_r63_room_global_20261018/SEGMENTED_DISTANCE_AUDIT.json')
out.write_text(json.dumps(data,indent=2,ensure_ascii=False,default=lambda x: int(x) if isinstance(x,np.integer) else float(x) if isinstance(x,np.floating) else str(x))+'\n')
print('WROTE',out)