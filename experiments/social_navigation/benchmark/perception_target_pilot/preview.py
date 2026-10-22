#!/usr/bin/env python3
"""Render actual recorded arrays; GT overlays and future grids are labels, not predictions."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from geometry import interpolate
from sensor import to_local


def render(bundle,sensor,labels,output):
    norm=bundle/'normalized';x=np.load(sensor/'track_inputs.npz');bev=np.load(sensor/'current_bev_inputs.npz');y=np.load(labels/'future_bev_labels.npz')
    i=len(x['anchor_ns'])//2;anchor=x['anchor_ns'][i];s=np.load(norm/'samples.npz');di=int(s['depth_index'][i,-1])
    depth=np.load(norm/'depth.npy',mmap_mode='r');frames=[json.loads(l) for l in (sensor/'sensor_tracks.jsonl').read_text().splitlines()];row=frames[di]
    scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([r['stamp_ns'] for r in scene])
    poses=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);poses[:,2]=np.unwrap(poses[:,2])
    people=np.array([[[h['x'],h['y']] for h in r['humans']] for r in scene]);pose,_=interpolate(times,poses,np.array([row['stamp_ns']]));human,_=interpolate(times,people,np.array([row['stamp_ns']]));local=to_local(human[0],pose[0])
    pairs={r['track_id']:r['gt_human_id'] for r in [json.loads(l) for l in (labels/'association_diagnostics.jsonl').read_text().splitlines()] if r['depth_index']==di}
    fig,axes=plt.subplots(2,3,figsize=(14,9));fig.suptitle(f'Actual recorded pilot | fixed middle sample {i} | anchor {anchor/1e9:.1f}s | no model trained',fontsize=15)
    cm=plt.get_cmap('viridis').copy();cm.set_bad('#76588c')
    axes[0,0].imshow(np.ma.masked_invalid(depth[di]),vmin=.28,vmax=5,cmap=cm);axes[0,0].set_title(f'INPUT: depth at {row["stamp_ns"]/1e9:.1f}s');axes[0,0].axis('off')
    color=ListedColormap(['#555555','#eeeeee','#ef5836'])
    def grid(ax,a,title):
        ax.imshow(a,origin='lower',extent=[-3.2,3.2,0,6.4],vmin=-1,vmax=1,cmap=color,interpolation='nearest');ax.set_title(title);ax.set_xlabel('y left (m)');ax.set_ylabel('x forward (m)')
    grid(axes[0,1],bev['occupancy'][i],'INPUT: current BEV, causal odom warp')
    ax=axes[0,2];ax.scatter(local[:,1],local[:,0],s=100,facecolors='none',edgecolors='#777777',label='GT labels at depth time')
    for j,p in enumerate(local):
        if 0<=p[0]<=6.4 and abs(p[1])<=3.2:ax.text(p[1]+.04,p[0]+.03,'GT'+str(scene[0]['humans'][j]['id']),color='#777777',fontsize=8,clip_on=True)
    for t in row['tracks']:
        if not t['confirmed']:continue
        p=t['xy_base'];human=pairs.get(t['track_id'],-1)
        ax.scatter(p[1],p[0],c='#198b6c' if human>=0 else '#d94040',marker='+',s=100)
        ax.text(p[1]+.04,p[0]-.12,'T'+str(t['track_id']),fontsize=8)
    ax.set(xlim=(-3.2,3.2),ylim=(0,6.4),xlabel='y left (m)',ylabel='x forward (m)',title='DIAGNOSTIC ONLY: candidates vs GT');ax.set_aspect('equal');ax.legend(fontsize=7)
    for j,offset in enumerate(y['offset_ns']):grid(axes[1,j],y['occupancy'][i,j],f'LABEL ONLY: observed BEV +{offset/1e9:g}s')
    fig.text(.02,.035,'BEV: gray unknown (loss masked), white observed free, red occupied. Future grids use GT ego warp into frozen anchor frame.',fontsize=10)
    fig.text(.02,.015,'Geometry candidates are not validated person detections. GT pairing is label-side only. This figure contains no predictions or AI-generated images.',fontsize=9)
    fig.tight_layout(rect=[0,.06,1,.95]);output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=120);plt.close(fig)
    print(json.dumps(dict(sample_index=i,anchor_ns=int(anchor),depth_stamp_ns=row['stamp_ns'],image=str(output),source='actual recorded arrays')))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--sensor',type=Path,required=True);p.add_argument('--labels',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();render(a.bundle,a.sensor,a.labels,a.output)
