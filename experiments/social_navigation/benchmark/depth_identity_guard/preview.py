#!/usr/bin/env python3
"""Measured regression/holdout depth and candidate streams, with GT label overlays."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from freeze import verify
from guard_tracker import CONFIG
from snapshots.sensor import to_local
HERE=Path(__file__).resolve().parent


def panel(axs,key,tag,obj):
    norm=Path(obj['datasets'][key]['normalized']);sensor=HERE/'generated'/(key+'_'+tag);evaluation=HERE/'generated'/(key+'_eval'+tag)
    metrics=json.loads((evaluation/'evaluation.json').read_text());gate=json.loads((evaluation/'identity_gate.json').read_text());frames={n:[json.loads(l) for l in (sensor/(n+'.jsonl')).read_text().splitlines()] for n in ['parent','combined']};i=len(frames['parent'])//2;t=frames['parent'][i]['stamp_ns']
    depth=np.load(norm/'depth.npy',mmap_mode='r');cmap=plt.get_cmap('viridis').copy();cmap.set_bad('#76588c');axs[0].imshow(np.ma.masked_invalid(depth[i]),cmap=cmap,vmin=.28,vmax=5);axs[0].axis('off');axs[0].set_title(f'{key}: actual middle frame {i}\nt={t/1e9:.1f}s',fontsize=10)
    scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([r['stamp_ns'] for r in scene]);r=np.array([[s['robot'][k] for k in ['x','y','yaw']] for s in scene]);r[:,2]=np.unwrap(r[:,2]);h=np.array([[[p['x'],p['y']] for p in s['humans']] for s in scene]);pose=np.array([np.interp(t,times,r[:,j]) for j in range(3)]);humans=np.array([[np.interp(t,times,h[:,j,c]) for c in range(2)] for j in range(6)]);local=to_local(humans,pose)
    for ax,name in zip(axs[1:],['parent','combined']):
        pairs={p['track_id']:p['gt_human_id'] for p in [json.loads(l) for l in (evaluation/(name+'_pairs_LABEL_ONLY.jsonl')).read_text().splitlines()] if p['depth_index']==i};ax.scatter(local[:,1],local[:,0],s=70,facecolors='none',edgecolors='#999999')
        for tr in frames[name][i]['tracks']:
            x,y=tr['xy_base']
            if tr['track_id']<0:ax.scatter(y,x,c='#e9a400',marker='x',s=60);continue
            if not tr['confirmed']:continue
            ax.scatter(y,x,c='#16866b' if pairs.get(tr['track_id'],-1)>=0 else '#dc3737',marker='+',s=70)
            if 0<=x<=6.4 and abs(y)<=3.2:ax.text(y+.04,x-.12,'T'+str(tr['track_id']),fontsize=7,clip_on=True)
        m=metrics['variants'][name];title='previous' if name=='parent' else 'identity guard';ax.set_title(f'{title}: pairing changes={m["paired_identity_changes_within_500ms"]}\nfragmentation={m["fragmentation_proxy"]}, paired rows={m["paired_rows"]}',fontsize=9);ax.set(xlim=(-3.2,3.2),ylim=(0,6.4),xlabel='y left (m)',ylabel='x forward (m)');ax.set_aspect('equal')
    return dict(dataset=key,frame=i,stamp_ns=t,identity_gate=gate['identity_guard_geometric_gate'],selection='Fixed midpoint, not quality-selected')


def render(output):
    output.mkdir(parents=True,exist_ok=True);obj=verify(HERE/'generated/final_with_seed44.json');fig,axes=plt.subplots(2,3,figsize=(12,8));sources=[]
    for row,key,tag in [(0,'regression43','05'),(1,'seed44','44')]:sources.append(panel(axes[row],key,tag,obj))
    fig.suptitle('Recorded data | code frozen before seed44 | GT is diagnostic LABEL ONLY',fontsize=12)
    fig.text(.02,.014,'Green + geometrically paired, red + unpaired, orange x measured but ID unknown. Gray circle GT. No World Model/AI imagery or semantic acceptance.',fontsize=8)
    fig.tight_layout(rect=[0,.04,1,.95]);fig.savefig(output/'identity_comparison.png',dpi=120);plt.close(fig)
    (output/'preview_sources.json').write_text(json.dumps(sources,indent=2)+'\n');print(json.dumps(sources))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();render(a.output.resolve())
