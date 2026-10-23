#!/usr/bin/env python3
"""Real sensor replay from train/validation only; no synthesis."""
import argparse,json,io
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from PIL import Image

def run(root,out,scene_id='p003'):
 root=Path(root);meta=next(v for v in json.loads((root/'manifest.json').read_text())['scenes'] if v['scene_id']==scene_id);assert meta['split'] in ['train','validation'],'Test preview forbidden';out=Path(out);out.mkdir(parents=True,exist_ok=True);d=np.load(root/'inputs'/scene_id/'depth_SENSOR_ONLY.npz');m=np.load(root/'labels'/scene_id/'instance_pixels_LABEL_ONLY.npz')['person_instance'];p=np.load(root/'inputs'/scene_id/'proposals_SENSOR_ONLY.npz');lab=np.load(root/'labels'/scene_id/'proposal_labels_LABEL_ONLY.npz');frames=[]
 cmap=ListedColormap(['#bbbbbb','#202020','#e55b5b','#f2b546','#59bca5','#5a9de2','#ac81d4','#ee8ec3']);norm=BoundaryNorm(np.arange(-1.5,7.5,1),cmap.N)
 for f in range(len(d['depth'])):
  fig,axes=plt.subplots(1,3,figsize=(15,4.5),gridspec_kw={'width_ratios':[1.25,1.25,1]});im=axes[0].imshow(np.ma.masked_invalid(d['depth'][f]),vmin=.28,vmax=5,cmap='viridis');axes[0].set_title('Native depth (m)');fig.colorbar(im,ax=axes[0],fraction=.04);axes[1].imshow(m[f],cmap=cmap,norm=norm);axes[1].set_title('LABEL ONLY: visible person IDs')
  for i in np.flatnonzero(p['frame_index']==f):
   a,b=p['point_offsets'][i:i+2];xyz=p['points_xyz'][a:b];target=int(lab['class_index'][i]);axes[2].scatter(xyz[:,1],xyz[:,0],s=2,c={1:'#e55b5b',0:'#5a9de2',-1:'#999999'}[target])
  axes[2].scatter([0],[0],marker='^',c='black',s=60);axes[2].set(xlabel='base_link left (m)',ylabel='base_link forward (m)',xlim=(3.2,-3.2),ylim=(0,5.3),title='Proposals: red person / blue environment');axes[2].set_aspect('equal');axes[2].grid(alpha=.2)
  for ax in axes[:2]:ax.set_xlabel('pixel u');ax.set_ylabel('pixel v')
  fig.suptitle(f'{scene_id} / {meta["split"]} — captured sim time {d["stamp_ns"][f]/1e9:.1f}s | scripted diagnostic motion');fig.tight_layout();buf=io.BytesIO();fig.savefig(buf,format='png',dpi=100);plt.close(fig);buf.seek(0);img=Image.open(buf).convert('RGB');frames.append(img)
  if f==len(d['depth'])//2:img.save(out/'depth_labels.png')
 frames[0].save(out/'depth_labels.gif',save_all=True,append_images=frames[1:],duration=850,loop=0);(out/'media_provenance.json').write_text(json.dumps(dict(source_scene=scene_id,split=meta['split'],native_sensor_replay=True,ai_generated=False,frames=len(frames),note='Third panel projects captured depth; instance colors are simulator labels, not predictions'),indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--corpus',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--scene',default='p003');a=p.parse_args();run(a.corpus,a.output,a.scene)
