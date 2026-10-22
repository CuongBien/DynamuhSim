#!/usr/bin/env python3
"""Render the new recording's fixed midpoint and actual frozen candidate outputs."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'depth_track_refinement'))
from snapshots.sensor import to_local


def render(output):
    output.mkdir(parents=True,exist_ok=True);root=HERE/'generated/runs/depth_seed43_01';norm=root/'normalized';sensor=HERE/'generated/sensor01';evaluation=HERE/'generated/evaluation01'
    metrics=json.loads((evaluation/'evaluation.json').read_text());frames={k:[json.loads(l) for l in (sensor/(k+'.jsonl')).read_text().splitlines()] for k in ['baseline','combined']}
    i=len(frames['baseline'])//2;stamp=frames['baseline'][i]['stamp_ns'];depth=np.load(norm/'depth.npy',mmap_mode='r')
    scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([r['stamp_ns'] for r in scene]);poses=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);poses[:,2]=np.unwrap(poses[:,2]);humans=np.array([[[h['x'],h['y']] for h in r['humans']] for r in scene]);r=np.array([np.interp(stamp,times,poses[:,j]) for j in range(3)]);p=np.array([[np.interp(stamp,times,humans[:,j,c]) for c in range(2)] for j in range(6)]);local=to_local(p,r)
    fig,axes=plt.subplots(1,3,figsize=(12,4));cmap=plt.get_cmap('viridis').copy();cmap.set_bad('#76588c')
    axes[0].imshow(np.ma.masked_invalid(depth[i]),cmap=cmap,vmin=.28,vmax=5);axes[0].set_title(f'Actual mesh depth, frame {i}\nt={stamp/1e9:.1f}s (fixed midpoint)',fontsize=10);axes[0].axis('off')
    for ax,name in zip(axes[1:],['baseline','combined']):
        pairs={a['track_id']:a['gt_human_id'] for a in [json.loads(l) for l in (evaluation/(name+'_pairs_LABEL_ONLY.jsonl')).read_text().splitlines()] if a['depth_index']==i}
        ax.scatter(local[:,1],local[:,0],s=75,facecolors='none',edgecolors='#999999')
        for t in frames[name][i]['tracks']:
            if not t['confirmed']:continue
            x,y=t['xy_base'];ax.scatter(y,x,c='#158568' if pairs.get(t['track_id'],-1)>=0 else '#d33737',marker='+',s=80)
            if 0<=x<=6.4 and abs(y)<=3.2:ax.text(y+.04,x-.12,'T'+str(t['track_id']),fontsize=7,clip_on=True)
        m=metrics['variants'][name];ax.set(xlim=(-3.2,3.2),ylim=(0,6.4),xlabel='y left (m)',ylabel='x forward (m)');ax.set_aspect('equal')
        ax.set_title(f'{name}: {m["pairing_rate"]:.1%} geometric pairing\nfragmentation proxy={m["fragmentation_proxy"]} (episode)',fontsize=10)
    decision='PASS' if metrics['comparison']['geometric_improvement'] else 'FAIL';fig.suptitle(f'New seed43 recording | frozen geometric gate {decision} | navigation TIMEOUT retained',fontsize=11)
    fig.text(.015,.015,'Green + paired candidate; red + unpaired; gray circle GT LABEL ONLY. No validated person detector or World Model. Recorded data, no AI imagery.',fontsize=8)
    fig.tight_layout(rect=[0,.05,1,.93]);fig.savefig(output/'seed43_comparison.png',dpi=120);plt.close(fig)
    source=dict(seed=43,frame_index=i,stamp_ns=stamp,selection='Fixed midpoint, no quality selection',source='Actual recorded depth and measured candidate tracker outputs; GT labels for evaluation only',world_model_trained=False)
    (output/'preview_source.json').write_text(json.dumps(source,indent=2)+'\n');print(json.dumps(source))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();render(a.output.resolve())
