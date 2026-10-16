#!/usr/bin/env python3
"""Exact MapEx default_map_eval LaMa 3-member inference on a frozen observed raster."""
import os,sys,json,time,hashlib,gc
import numpy as np
import torch
from lama_pred_utils import load_lama_model,get_lama_transform,convert_obsimg_to_model_input
inp,out=sys.argv[1:3]
a=np.load(inp,allow_pickle=False)
obs=a['observed'] if isinstance(a,np.lib.npyio.NpzFile) else a
assert obs.shape==(203,263)
x=np.full(obs.shape,.5,np.float32);x[obs==0]=0;x[obs==2]=1
image=np.stack([x,x,x],axis=-1)
tr=get_lama_transform('default_map_eval',(512,512))
b,_=convert_obsimg_to_model_input(image,tr,'cpu')
assert b['image'].shape[-2:]==(208,272)
top=((16-obs.shape[0]%16)%16)//2;left=((16-obs.shape[1]%16)%16)//2
torch.set_num_threads(2)
stack=[];dur=[]
for k in (1,2,3):
 t=time.monotonic()
 root=f'/home/dell/MapEx/pretrained_models/weights/lama_ensemble/train_{k}'
 model=load_lama_model(root,device='cpu');model.eval()
 with torch.no_grad():q=model(b)['inpainted'][0,0].detach().cpu().numpy().copy()
 arr=q[top:top+203,left:left+263]
 assert arr.shape==(203,263)
 stack.append(arr.astype(np.float32))
 del model;gc.collect();dur.append(round(time.monotonic()-t,3))
mean=np.clip(np.mean(stack,axis=0),0,1)
var=np.var(stack,axis=0,ddof=1).clip(0)
assert np.isfinite(mean).all() and np.isfinite(var).all()
np.savez_compressed(out,observed=obs,mean=mean,variance=var)
print(json.dumps({'output':out,'source':'LaMa G1/G2/G3 default_map_eval','time_per_model_s':dur,'input_unknown':int((obs==1).sum()),'mean_sha':hashlib.sha256(mean.tobytes()).hexdigest()}),flush=True)
