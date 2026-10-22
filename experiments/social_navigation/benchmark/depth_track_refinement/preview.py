#!/usr/bin/env python3
"""Real recorded depth and measured candidate tracks; all GT is diagnostic overlay."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from snapshots.sensor import to_local
HERE=Path(__file__).resolve().parent


def panel(axes,key,run_name,eval_name,fixed):
    norm=Path(fixed['datasets'][key]['normalized']);frames={name:[json.loads(l) for l in (HERE/'generated'/run_name/(name+'.jsonl')).read_text().splitlines()] for name in ['baseline','combined']}
    i=len(frames['baseline'])//2;t=frames['baseline'][i]['stamp_ns'];depth=np.load(norm/'depth.npy',mmap_mode='r')
    result=json.loads((HERE/'generated'/eval_name/'evaluation.json').read_text())
    cm=plt.get_cmap('viridis').copy();cm.set_bad('#76588c');axes[0].imshow(np.ma.masked_invalid(depth[i]),cmap=cm,vmin=.28,vmax=5);axes[0].axis('off')
    title={'development':'Development: mesh depth','mesh_repeat':'Additional mesh repeat (Nav TIMEOUT)','validation':'Initial validation: legacy proxy depth'}[key]
    axes[0].set_title(title+'\n'+f'Actual middle frame {i}, t={t/1e9:.1f}s',fontsize=10)
    scene=[json.loads(l) for l in (norm/'human_states.jsonl').read_text().splitlines()];times=np.array([r['stamp_ns'] for r in scene]);poses=np.array([[r['robot'][k] for k in ['x','y','yaw']] for r in scene]);poses[:,2]=np.unwrap(poses[:,2]);people=np.array([[[h['x'],h['y']] for h in r['humans']] for r in scene]);pose=np.array([np.interp(t,times,poses[:,j]) for j in range(3)]);human=np.array([[np.interp(t,times,people[:,j,c]) for c in range(2)] for j in range(6)]);local=to_local(human,pose)
    for ax,name in zip(axes[1:],['baseline','combined']):
        mapping={r['track_id']:r['gt_human_id'] for r in [json.loads(l) for l in (HERE/'generated'/eval_name/(name+'_pairs_LABEL_ONLY.jsonl')).read_text().splitlines()] if r['depth_index']==i}
        ax.scatter(local[:,1],local[:,0],s=80,facecolors='none',edgecolors='#999999')
        for tr in frames[name][i]['tracks']:
            if not tr['confirmed']:continue
            x,y=tr['xy_base'];color='#158568' if mapping.get(tr['track_id'],-1)>=0 else '#da3939'
            ax.scatter(y,x,c=color,marker='+',s=80)
            if 0<=x<=6.4 and abs(y)<=3.2:ax.text(y+.04,x-.12,'T'+str(tr['track_id']),fontsize=7,clip_on=True)
        m=result['variants'][name];ax.set(xlim=(-3.2,3.2),ylim=(0,6.4),xlabel='y left (m)',ylabel='x forward (m)');ax.set_aspect('equal')
        ax.set_title(f'{name}: {m["pairing_rate"]:.1%} geometric pairing\nfragmentation proxy={m["fragmentation_proxy"]} (whole episode)',fontsize=10)
    return dict(dataset=key,frame_index=i,stamp_ns=t,selection='Fixed midpoint, no quality-based frame selection')


def render(output):
    output.mkdir(parents=True,exist_ok=True);fixed=json.loads((HERE/'generated/frozen02.json').read_text());rows=[('development','development01','development_evaluation01'),('mesh_repeat','mesh_repeat01','mesh_repeat_evaluation01'),('validation','validation01','validation_evaluation01')]
    fig,axes=plt.subplots(3,3,figsize=(12,11));examples=[]
    for axs,args in zip(axes,rows):examples.append(panel(axs,*args,fixed))
    fig.suptitle('Actual recorded comparisons | green: geometrically paired candidate; red: unpaired; gray circle: GT label',fontsize=11)
    fig.text(.02,.012,'Same frozen inference on all recordings. Proxy validation FAIL is retained. No semantic person-detector or World Model acceptance.',fontsize=9)
    fig.tight_layout(rect=[0,.03,1,.96]);fig.savefig(output/'profile_comparison.png',dpi=120);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(12,4));panel(axes,*rows[1],fixed)
    fig.suptitle('Recorded mesh repeat: geometric improvement only; source navigation still TIMEOUT',fontsize=11)
    fig.text(.02,.015,'Green + paired candidate; red + unpaired candidate; gray circle GT LABEL ONLY. No World Model trained; no generated imagery.',fontsize=9)
    fig.tight_layout(rect=[0,.05,1,.93]);fig.savefig(output/'matched_mesh_preview.png',dpi=120);plt.close(fig)
    (output/'preview_sources.json').write_text(json.dumps(examples,indent=2)+'\n');print('Rendered fixed midpoint source frames')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();render(a.output.resolve())
