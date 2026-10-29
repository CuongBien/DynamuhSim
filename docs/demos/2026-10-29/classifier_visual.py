from pathlib import Path
import sys,json,hashlib,os
import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import argparse
parser=argparse.ArgumentParser(description='Render validation previews with the frozen classifier; no fitting or test use.')
parser.add_argument('--repo',type=Path,default=next(p for p in Path(__file__).resolve().parents if (p/'pyproject.toml').is_file()))
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
ROOT=args.repo.resolve()
OUT=args.output.resolve();OUT.mkdir(parents=True,exist_ok=False)
STAGE=ROOT/'experiments/social_navigation/perception/depth_person_classifier'
sys.path.insert(0,str(STAGE))
from data import load_split,loader,CORPUS
from model import Model
from inference import normalized,numpy_logits
from metrics import measure,sigmoid
cfg=json.loads((ROOT/'control_bundles/sensor_pilot_v1/classifier/metadata.json').read_text())
file=ROOT/'control_bundles/sensor_pilot_v1/classifier/model.npz'
assert hashlib.sha256(file.read_bytes()).hexdigest()==cfg['model_sha256']
arrays=dict(np.load(file,allow_pickle=False));data=load_split('validation')
assert torch.cuda.is_available();torch.backends.cuda.matmul.allow_tf32=False
m=Model(cfg['hidden'],cfg['modality']).cuda();m.load_state_dict({k:torch.from_numpy(v).cuda() for k,v in arrays.items() if k.startswith(('depth.','lidar.'))});m.eval()
with torch.inference_mode():logits=m(*[torch.from_numpy(x).cuda() for x in normalized(data,arrays)]).cpu().numpy()
reference=numpy_logits(data,arrays,cfg['hidden'],cfg['modality']);np.testing.assert_allclose(logits,reference,atol=1e-6,rtol=1e-6)
prob=sigmoid(logits);metrics=measure(data,prob,cfg['threshold'])
result={'status':'PASS','split':'validation','model':cfg['candidate'],'threshold':cfg['threshold'],'model_sha256':cfg['model_sha256'],'device':torch.cuda.get_device_name(),'cuda':torch.version.cuda,'numpy_max_logit_error':float(np.max(np.abs(logits-reference))),'metrics':metrics,'training_performed':False,'test_split_read':False,'source':'New inference on previously recorded depth validation proposals, not live Gazebo inference'}
(OUT/'classifier_validation.json').write_text(json.dumps(result,indent=2)+'\n')
fig,axes=plt.subplots(2,2,figsize=(15,8.5),layout='constrained')
for row,(scene,frame) in enumerate([('p004',0),('p005',2)]):
 indices=[i for i,r in enumerate(data['rows']) if r['scene_id']==scene and r['frame_index']==frame];assert indices
 sample=loader.load_input(CORPUS,scene,data['rows'][indices[0]]['sample_index']);depth=sample['depth'];valid=np.isfinite(depth)&(depth>=.28)&(depth<=5)
 for col in range(2):
  ax=axes[row,col];im=ax.imshow(np.ma.masked_where(~valid,depth),vmin=.28,vmax=5,cmap='viridis');ax.set_title(f'{scene} frame {frame} | '+('Recorded depth (m)' if col==0 else 'Frozen GPU classifier predictions'));ax.set_axis_off()
 if row==0:fig.colorbar(im,ax=axes[row,0],shrink=.7,label='Depth (m)')
 for i in indices:
  s=loader.load_input(CORPUS,scene,data['rows'][i]['sample_index']);uv=s['point_pixel_uv'];u0,v0=uv.min(0);u1,v1=uv.max(0);pred=int(prob[i]>=cfg['threshold']);label=int(data['y'][i]);color='#15e66c' if pred==1 else '#00bfff'
  if pred!=label:color='#ff4545'
  ax=axes[row,1];ax.add_patch(Rectangle((u0,v0),u1-u0+1,v1-v0+1,fill=False,edgecolor=color,linewidth=2));ax.text(u0,max(0,v0-3),('PERSON' if pred else 'OBJECT')+f' {prob[i]:.2f}',fontsize=8,color='white',bbox={'facecolor':'black','alpha':.7,'pad':1})
a=metrics['overall'];fig.suptitle(f"Depth person/object classifier | VALIDATION | {result['device']}\nTP {a['tp']} / FN {a['fn']} / FP {a['fp']} / TN {a['tn']} | unchanged model + threshold 0.50",fontsize=15)
fig.savefig(OUT/'classifier_validation.png',dpi=150);plt.close(fig)
print(json.dumps(result,indent=2))

missed=np.flatnonzero(data['label_valid'] & (data['y']==1) & (prob<cfg['threshold']))
for j in missed:
 row=data['rows'][int(j)];sample=loader.load_input(CORPUS,row['scene_id'],row['sample_index']);depth=sample['depth'];valid=np.isfinite(depth)&(depth>=.28)&(depth<=5)
 fig,ax=plt.subplots(figsize=(10,5),layout='constrained');ax.imshow(np.ma.masked_where(~valid,depth),vmin=.28,vmax=5,cmap='viridis');uv=sample['point_pixel_uv'];u0,v0=uv.min(0);u1,v1=uv.max(0);ax.add_patch(Rectangle((u0,v0),u1-u0+1,v1-v0+1,fill=False,edgecolor='red',linewidth=2));ax.set_title(f"VALIDATION missed person | {row['scene_id']} frame {row['frame_index']} | score {prob[j]:.3f} < 0.5\nGround truth person; frozen classifier predicted object");ax.set_axis_off();fig.savefig(OUT/'classifier_missed_case.png',dpi=140);plt.close(fig)
(OUT/'classifier_predictions.json').write_text(json.dumps([dict(**r,person_score=float(p),class_index=int(p>=.5),label=int(y),label_valid=bool(v)) for r,p,y,v in zip(data['rows'],prob,data['y'],data['label_valid'])],indent=2)+'\n')
