#!/usr/bin/env python3
"""Replay native validation depth with saved frozen-model scores; no AI generation."""
import hashlib,io,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Patch
from PIL import Image
HERE=Path(__file__).resolve().parent.parent;RUN=HERE/'generated/runs/run01';CORPUS=HERE.parent/'depth_perception_corpus/generated/corpus01'
def main():
 out=HERE/'reports/media';out.mkdir(exist_ok=True);frozen=json.loads((RUN/'selected_frozen.json').read_text());train=json.loads((RUN/'training.json').read_text());rows=json.loads((RUN/'development_rows.json').read_text())['validation'];prob=np.load(RUN/(frozen['candidate']+'_validation_probabilities.npy'));sid='p005';d=np.load(CORPUS/'inputs'/sid/'depth_SENSOR_ONLY.npz');p=np.load(CORPUS/'inputs'/sid/'proposals_SENSOR_ONLY.npz');labels=np.load(CORPUS/'labels'/sid/'proposal_labels_LABEL_ONLY.npz');ids=[i for i,r in enumerate(rows) if r['scene_id']==sid];scores={rows[i]['sample_index']:float(prob[i]) for i in ids};colors=dict(tp='#168add',tn='#36a657',fn='#f09416',fp='#dc3046');errors=[int(p['frame_index'][i]) for i in scores if (scores[i]>=frozen['threshold'])!=int(labels['class_index'][i])];png_frame=min(errors) if errors else len(d['depth'])//2;frames=[]
 for frame,depth in enumerate(d['depth']):
  fig,axes=plt.subplots(1,2,figsize=(12,4.8));img=np.ma.masked_invalid(depth)
  for ax in axes:ax.imshow(img,vmin=.28,vmax=5,cmap='viridis');ax.set(xlabel='pixel u',ylabel='pixel v')
  axes[0].set_title('Captured native depth (m)');axes[1].set_title('Frozen predictions + LABEL review')
  for index in np.flatnonzero(p['frame_index']==frame):
   y=int(labels['class_index'][index]);pred=int(scores[index]>=frozen['threshold']);kind=('tp' if pred else 'fn') if y else ('fp' if pred else 'tn');u,v,x,z=p['bbox_uv'][index];color=colors[kind];axes[1].add_patch(Rectangle((u,v),x-u,z-v,fill=False,edgecolor=color,linewidth=1.7));axes[1].text(u,max(v-3,8),f'{index}: {scores[index]:.2f}',color='white',fontsize=7,bbox=dict(facecolor=color,alpha=.9,pad=.5))
  fig.suptitle(f'Validation {sid}, captured sim {d["stamp_ns"][frame]/1e9:.1f}s — {frozen["candidate"]}, threshold {frozen["threshold"]:.2f}');fig.legend(handles=[Patch(color=colors[k],label=v) for k,v in [('tp','Correct person'),('tn','Correct environment'),('fn','Missed person'),('fp','False person')]],loc='lower center',ncol=4,fontsize=9);fig.tight_layout(rect=[0,.09,1,.94]);buf=io.BytesIO();fig.savefig(buf,format='png',dpi=110);plt.close(fig);buf.seek(0);image=Image.open(buf).convert('RGB');frames.append(image)
  if frame==png_frame:image.save(out/'validation_predictions.png')
 frames[0].save(out/'validation_predictions.gif',save_all=True,append_images=frames[1:],duration=850,loop=0)
 trace=json.loads((RUN/'training_trace.json').read_text());fig,axes=plt.subplots(1,2,figsize=(11,4))
 for name,values in trace.items():axes[0].plot([v['step'] for v in values],[v['weighted_train_bce'] for v in values],label=name)
 axes[0].set(xlabel='Optimizer step',ylabel='Scene/frame/class weighted train BCE',yscale='log',title='Actual CUDA training loss');axes[0].legend(fontsize=8);axes[0].grid(alpha=.2);names=[v['name'] for v in train['candidates']];best=[max([r for r in train['threshold_grid_scores'] if r['candidate']==n],key=lambda v:tuple(v['rank'])) for n in names];axes[1].bar(names,[r['metrics']['overall']['balanced_accuracy'] for r in best]);axes[1].set(ylim=(0,1.05),ylabel='Balanced accuracy',title='Validation only; no test candidate comparison');axes[1].tick_params(axis='x',rotation=20);fig.tight_layout();fig.savefig(out/'training_validation.png',dpi=130);plt.close(fig)
 (out/'provenance.json').write_text(json.dumps(dict(source_split='validation',source_scene=sid,captured_depth_file=str((CORPUS/'inputs'/sid/'depth_SENSOR_ONLY.npz').relative_to(HERE.parents[3])),captured_depth_sha256=hashlib.sha256((CORPUS/'inputs'/sid/'depth_SENSOR_ONLY.npz').read_bytes()).hexdigest(),model_sha256=frozen['model_sha256'],score_file_sha256=hashlib.sha256((RUN/(frozen['candidate']+'_validation_probabilities.npy')).read_bytes()).hexdigest(),png_selection='First validation error frame, not a selected successful frame',png_frame_index=png_frame,gif_frames=len(frames),ai_generated=False,scores_are_uncalibrated=True),indent=2)+'\n')
if __name__=='__main__':main()
