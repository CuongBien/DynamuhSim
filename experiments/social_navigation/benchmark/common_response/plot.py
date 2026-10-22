#!/usr/bin/env python3
"""Scientific plots/animation rendered solely from recorded trajectories."""
import argparse,io,json,os
from pathlib import Path
os.environ['MPLBACKEND']='Agg';os.environ['OPENBLAS_NUM_THREADS']='1'
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle
from PIL import Image
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=HERE/'reports/run01');a=p.parse_args();out=a.report_dir.resolve()
load=lambda p:json.loads(p.read_text());metrics=load(out/'metrics.json');models=list(load(out/'protocol.json')['models'])
labels=['Soc native individuals','Soc guarded individuals','CSP ORCA+','CSP SFM','CN native mixed*','CN geometry mixed']
values=np.array([[min(r[k] for r in metrics if r['model']==model) for k in ['min_pair_gap_m','min_robot_gap_m','min_static_gap_m']] for model in models])
fig,ax=plt.subplots(figsize=(8.3,4.8),layout='constrained');im=ax.imshow(values,cmap='RdYlGn',vmin=-.4,vmax=.8,aspect='auto')
ax.set_yticks(range(6),labels);ax.set_xticks(range(3),['People / people','Robot / people','Factory / people'])
for i in range(6):
 for j in range(3):ax.text(j,i,f'{values[i,j]:.3f}',ha='center',va='center',fontsize=11)
ax.set_title('Minimum signed proxy gaps across6 cases (m)\nRecorded offline data | Positive is not a social-comfort score',fontsize=11)
fig.colorbar(im,ax=ax,label='Signed gap (m)');fig.savefig(out/'minimum_gaps.png',dpi=160);plt.close(fig)
# Common0.5s samples are actual simulator endpoints for both native timesteps.
scene='crossing_near';fixture=load(HERE/'inputs'/(scene+'.json'));layout=load(HERE/'inputs/layout.json');data={model:load(out/model/(scene+'_trajectory.json')) for model in models};dt=load(out/'protocol.json')['models'];frames=[]
colors=['#2c7bb6','#d7191c','#1a9641','#984ea3','#fdae61','#4d4d4d']
for t in np.arange(0.,10.0001,.5):
 fig,axes=plt.subplots(2,3,figsize=(12,6.8),layout='constrained')
 for ax,model,label in zip(axes.ravel(),models,labels):
  for b in layout['obstacles']:ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],fc='#d4cab8',ec='#8a7c67',lw=.4))
  rows=data[model];index=round(t/dt[model]['dt']);valid=index<len(rows);row=rows[min(index,len(rows)-1)]
  for h,color in zip(row['humans'],colors):ax.add_patch(Circle((h['x'],h['y']),h['radius'],fc=color,ec='white',lw=.5,alpha=.8))
  robot=fixture['robot_trace'][round(t/.05)];ax.add_patch(Circle((robot['x'],robot['y']),.25,fc='black'))
  ax.set(xlim=(-29,-20),ylim=(-3.3,3.3),title=label,aspect='equal');ax.tick_params(labelsize=7);ax.grid(alpha=.1)
  if not valid:ax.text(-28,0,'END OF RECORDED PREFIX',color='red',fontsize=9)
 fig.suptitle(f'OFFLINE SIMULATOR DATA / NOT GAZEBO | crossing,near robot | t={t:.1f}s | playback2x',fontsize=12)
 buffer=io.BytesIO();fig.savefig(buffer,format='png',dpi=100);plt.close(fig);buffer.seek(0)
 with Image.open(buffer) as image:frames.append(image.convert('RGB').quantize(colors=128))
frames[0].save(out/'crossing_near_2d_2x.gif',save_all=True,append_images=frames[1:],duration=250,loop=0)
(out/'media.json').write_text(json.dumps(dict(source='Circle plots rendered from recorded native/variant offline trajectories, not Gazebo RGB or AI',scene=scene,frame_times=[i*.5 for i in range(21)],playback_speed=2,last_frame_hold_ms=250,duration_ms=5250),indent=2)+'\n')
print('Saved measured-gap figure and21-frame offline-only comparison GIF')
