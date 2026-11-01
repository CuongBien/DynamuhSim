#!/usr/bin/env python3
"""Train-only real data preview and self-contained sample, never WM-generated predictions."""
import json,sys,hashlib,shutil
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap,BoundaryNorm
from loader import Dataset
HERE=Path(__file__).resolve().parent;ROOT=HERE/'generated/dataset01'
def main():
 ds=Dataset(ROOT);sid='wm_standing_45';e=ds.episode(sid);assert e.meta['split']=='train';i=min(50,len(e)-1);x=e.observation(i);y=e.supervision(i);m=e.task_masks(i);media=HERE/'reports/media';media.mkdir(exist_ok=True);example=HERE/'examples/window01';example.mkdir(parents=True,exist_ok=False);np.savez_compressed(example/'observation_SENSOR_ONLY.npz',**x['history'],depth_history=x['depth_history'],bev_anchor_estimated=x['bev_anchor_estimated'],bev_frame_valid=np.array(x['bev_frame_valid']));np.savez_compressed(example/'supervision_LABEL_ONLY.npz',**{'trajectory__'+k:v for k,v in y['trajectories'].items()},**{'future_bev__'+k:v for k,v in y['future_bev'].items()});np.savez_compressed(example/'task_masks_LABEL_ONLY.npz',**m);(example/'calibration_SENSOR_ONLY.json').write_text(json.dumps(x['calibration'],indent=2)+'\n');(example/'metadata.json').write_text(json.dumps(dict(episode=sid,split='train',window_index=i,anchor_ns=int(x['history']['anchor_ns']),depth_stamp_ns=int(x['history']['depth_stamp_ns'][-1]),selection='Fixed index50 in first planned train episode; no test viewing/model selection',input_bev_frame='Estimated anchor using causal odometry',future_bev_frame='True frozen anchor using GT ego LABEL ONLY',wm_observed_bev_train_valid=bool(m['wm_observed_bev_train_valid']),human_association_valid=False,H1_H5_valid=False,RL_reward_valid=False),indent=2)+'\n')
 fig,axes=plt.subplots(1,5,figsize=(17,4),gridspec_kw={'width_ratios':[1.5,1,1,1,1]});axes[0].imshow(np.ma.masked_invalid(x['depth_history'][-1]),vmin=.28,vmax=5,cmap='viridis');axes[0].set(title='Actual current depth (m)',xlabel='pixel u',ylabel='pixel v');colors=ListedColormap(['#707070','#f4f4f4','#d94d37']);norm=BoundaryNorm([-1.5,-.5,.5,1.5],3)
 for ax,grid,title in zip(axes[1:],[x['bev_anchor_estimated']]+list(y['future_bev']['occupancy']),['INPUT: odom-estimated anchor']+[f'LABEL: observed +{v/1e9:g}s' for v in y['future_bev']['offset_ns']]):
  ax.imshow(grid[::-1,::-1],cmap=colors,norm=norm,extent=[3.2,-3.2,0,6.4],origin='upper');ax.set(title=title,xlabel='left (m)',ylabel='forward (m)');ax.plot([0],[0],marker='^',color='black',ms=5)
 fig.suptitle(f'Train {sid} / window{i} / anchor {x["history"]["anchor_ns"]/1e9:.1f}s — captured data and LABELS, no WM prediction',fontsize=12);fig.text(.5,.025,'BEV gray = unknown (loss masked); white = observed free; red = observed occupied. Input estimate and target true ego frames can differ.',ha='center',fontsize=10);fig.tight_layout(rect=[0,.08,1,.92]);fig.savefig(media/'wm_sample.png',dpi=120);plt.close(fig)
 raw=HERE/'generated/runs'/sid;shutil.copy2(raw/'camera_preview_4x.gif',media/'native_observer_train_4x.gif');(media/'provenance.json').write_text(json.dumps(dict(ai_generated=False,WM_predictions=False,source_episode=sid,split='train',window_index=i,depth_source_sha256=hashlib.sha256((ROOT/'episodes'/sid/'inputs/depth_SENSOR_ONLY.npy').read_bytes()).hexdigest(),observer_source='Native Gazebo external debug RGB camera, not robot RGB input',observer_sha256=hashlib.sha256((raw/'camera_preview_4x.gif').read_bytes()).hexdigest()),indent=2)+'\n');print('Train-only preview and portable example saved',i)
if __name__=='__main__':main()
