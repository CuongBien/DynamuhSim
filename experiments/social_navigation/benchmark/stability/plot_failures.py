#!/usr/bin/env python3
"""Visual evidence for the recorded mode switch and independently captured reroute."""
import argparse,json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'localization_diagnosis'))
from geometry import MapField,stamp
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();report=a.report_dir
field=MapField(BENCH.parent/'warehouse/warehouse.yaml');boxes=json.loads((BENCH.parent/'warehouse/layout.json').read_text())['obstacles'];events=json.loads((report/'events.json').read_text());summary=json.loads((report/'summary.json').read_text())['trials']
def background(ax):
 for box in boxes:ax.add_patch(Rectangle((box['x']-box['sx']/2,box['y']-box['sy']/2),box['sx'],box['sy'],color='#ddd',zorder=0))
 ax.set_aspect('equal');ax.grid(alpha=.2);ax.set(xlabel='world / map x [m]',ylabel='world / map y [m]')
e=next(e for e in events if e['round']==1 and e['policy']=='orca_plus');assert e['status']=='ABORTED' and e['largest_jump_clouds']['after_largest_jump']['clusters'][0]['mean_static_occupied'];trial=json.loads((report/next(r['report'] for r in summary if r['round']==1 and r['policy']=='orca_plus')).read_text());runtime=BENCH/trial['runtime'];gate=float((runtime/'start_gate').read_text());clouds=[]
for label in ['before_largest_jump','after_largest_jump']:
 target=e['largest_jump_clouds'][label]['cloud_elapsed_s']+gate;match=None
 for line in (runtime/'diagnostics.jsonl').open():
  r=json.loads(line)
  if r['kind']=='particle_cloud' and (match is None or abs(stamp(r['data'])-target)<abs(stamp(match['data'])-target)):match=r
 assert abs(stamp(match['data'])-target)<1e-6;clouds.append(match)
fig,axes=plt.subplots(1,2,figsize=(11,6))
for ax,label,cloud,anchor in zip(axes,['before_largest_jump','after_largest_jump'],clouds,[e['jump_before'],e['jump_after']]):
 background(ax);particles=cloud['data']['particles'];x=np.array([v['pose']['position']['x'] for v in particles]);y=np.array([v['pose']['position']['y'] for v in particles]);occupied=field.occupied_at(x,y)
 ax.scatter(x[~occupied],y[~occupied],s=7,c='#478bbf',alpha=.45,label='free-cell particles');ax.scatter(x[occupied],y[occupied],s=7,c='#c15b4a',alpha=.45,label='occupied-cell particles')
 for key,color,marker in [('actual','#196a35','D'),('amcl','black','x')]:ax.scatter([anchor[key]['x']],[anchor[key]['y']],s=75,c=color,marker=marker,label=key)
 cr=e['largest_jump_clouds'][label]
 for c in cr['clusters']:ax.annotate(f"weight {c['weight']:.3f}",(c['x'],c['y']),xytext=(8,20),textcoords='offset points',fontsize=9,arrowprops=dict(arrowstyle='-',lw=.6))
 ax.set(xlim=(12.3,15.1),ylim=(-2.4,1.1),title=f"{label.split('_')[0]} | cloud {cr['cloud_elapsed_s']:.3f}s | AMCL {anchor['elapsed']:.3f}s")
axes[0].legend(fontsize=8,loc='lower left');fig.suptitle('ORCA+ round1: dominant recorded cluster changes from free space to inside a machine');fig.tight_layout();fig.savefig(report/'orca_mode_switch.png',dpi=160,bbox_inches='tight');plt.close(fig)
e=next(e for e in events if e['round']==3 and e['policy']=='sfm');assert e['status']=='ABORTED' and e['first_plan_leaves_channel_s'] is not None;name=next(r['name'] for r in summary if r['round']==3 and r['policy']=='sfm');tr=json.loads((report/'round03/timeline.json').read_text())[name];t=e['first_plan_leaves_channel_s'];plans=tr['global_plans'];before=[pl for pl in plans if pl['elapsed']<t][-1];after=next(pl for pl in plans if pl['elapsed']==t)
fig,ax=plt.subplots(figsize=(11,8));background(ax)
for plan,color,label in [(before,'#3c9567','last plan before reroute'),(after,'#d28c2a','first plan around machine')]:
 points=np.array(plan['points']);ax.plot(points[:,0],points[:,1],color=color,lw=2,label=f"{label}: {plan['elapsed']:.3f}s")
ar=tr['amcl'];ax.plot([r['actual']['x'] for r in ar],[r['actual']['y'] for r in ar],c='#2875b7',lw=2,label='actual robot');ax.plot([r['amcl']['x'] for r in ar],[r['amcl']['y'] for r in ar],c='#c25042',ls='--',lw=1,label='AMCL')
peak=max(ar,key=lambda r:r['actual']['x']);ax.scatter([peak['actual']['x']],[peak['actual']['y']],c='#2875b7',s=45);ax.annotate(f"maximum actual x: {peak['elapsed']:.3f}s",(peak['actual']['x'],peak['actual']['y']),xytext=(15,-35),textcoords='offset points',arrowprops=dict(arrowstyle='->'),fontsize=9)
points=np.array(before['points']+after['points']);ax.set(xlim=(min(7.5,float(points[:,0].min())-.5),max(25.5,float(points[:,0].max())+.5)),ylim=(min(-2.,float(points[:,1].min())-1),max(2.,float(points[:,1].max())+1)),title='SFM round3: global reroute recorded before return to the left doorway')
ax.legend(fontsize=8,loc='best');fig.tight_layout();fig.savefig(report/'sfm_global_reroute.png',dpi=160,bbox_inches='tight');plt.close(fig)
labels=[f"R{r['round']} {r['policy']}" for r in summary];colors=['#4d83b5' if r['policy']=='sfm' else '#bd793d' for r in summary]
fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
for ax,key,label in zip(axes,['amcl_error_max_m','largest_amcl_step_m','min_robot_human_footprint_m'],['AMCL error max [m]','AMCL step max [m]','Robot-human gap min [m]']):
 values=[r[key] for r in summary];ax.bar(labels,values,color=colors);ax.set_ylabel(label);ax.grid(axis='y',alpha=.2)
 for i,v in enumerate(values):ax.text(i,v,f'{v:.3f}',ha='center',va='bottom',fontsize=8)
 ax.margins(y=.18)
fig.suptitle('Six fixed seed42 repetitions | after cleanup | counts do not rank policies');fig.tight_layout();fig.savefig(report/'stability_metrics.png',dpi=160,bbox_inches='tight');plt.close(fig)
print('Saved mode-switch, full-route and compact metric figures.')
