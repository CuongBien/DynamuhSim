#!/usr/bin/env python3
"""Display recorded local grids, projected to world using the known spawn only."""
import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle,Polygon
from geometry import stamp,yaw
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parents[1]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=HERE/'reports/run01');a=p.parse_args();manifest=json.loads((a.report_dir/'manifest.json').read_text());events=json.loads((a.report_dir/'events.json').read_text());windows=json.loads((a.report_dir/'failure_windows.json').read_text());boxes=json.loads((SOCIAL/'warehouse/layout.json').read_text())['obstacles']
case1='crowdsimplus_individual_goals_bottleneck_s42';case2='crowdsimplus_sfm_individual_goals_bottleneck_s42'
ev=next(e for e in events if e['name']==case1);sfm=next(e for e in windows if e['name']==case2)
choices=[(case1,'ORCA+ jump',ev['after']),(case1,'ORCA+ last pose',next(w for w in windows if w['name']==case1)['anchors']['last_matched_amcl']),
 (case2,'SFM near translation stop',sfm['anchors']['persistent_translation_stop']['sample']),(case2,'SFM estimated center enters box',sfm['anchors']['estimated_center_enters_occupied']['sample'])]
frames=[]
for name,label,row in choices:
 runtime=Path(next(m for m in manifest['inputs'] if m['name']==name)['runtime']);t=row['time'];best={}
 for line in (runtime/'diagnostics.jsonl').open():
    raw=json.loads(line);kind=raw['kind'];d=raw['data']
    if kind not in ('gazebo','costmap'):continue
    ts=d['time'] if kind=='gazebo' else stamp(d)
    if kind not in best or abs(ts-t)<abs(best[kind][0]-t):best[kind]=(ts,d)
 assert abs(best['costmap'][0]-t)<.3 and abs(best['gazebo'][0]-t)<.06
 frames.append((label,row,best))
fig,axes=plt.subplots(2,2,figsize=(11,8));details=[]
for ax,(label,row,best) in zip(axes.flat,frames):
 for b in boxes:ax.add_patch(Rectangle((b['x']-b['sx']/2,b['y']-b['sy']/2),b['sx'],b['sy'],color='#ddd',zorder=0))
 grid=best['costmap'][1];info=grid['info'];origin=info['origin']['position'];assert grid['header']['frame_id']=='odom';data=np.array(grid['data']).reshape(info['height'],info['width']);x0,y0=origin['x']+8.,origin['y'];res=info['resolution']
 im=ax.imshow(np.ma.masked_less_equal(data,0),extent=[x0,x0+res*info['width'],y0,y0+res*info['height']],origin='lower',cmap='autumn_r',vmin=0,vmax=100,alpha=.7,interpolation='nearest',zorder=1)
 actual=best['gazebo'][1]['poses']['mecanum_robot']
 for ro,color,style,title in [(actual,'black','-','actual footprint'),(row['amcl'],'#2774b5','--','AMCL footprint')]:
  c,s=math.cos(ro['yaw']),math.sin(ro['yaw']);corners=[(ro['x']+c*x-s*y,ro['y']+s*x+c*y) for x,y in [(-.13,-.11),(.13,-.11),(.13,.11),(-.13,.11)]];ax.add_patch(Polygon(corners,fill=False,edgecolor=color,ls=style,lw=2,label=title,zorder=5))
 for key,h in best['gazebo'][1]['poses'].items():
  if key!='mecanum_robot':ax.add_patch(Circle((h['x'],h['y']),.36,fill=False,color='#3c966c',zorder=4))
 display={'ORCA+ jump':'ORCA+: jump','ORCA+ last pose':'ORCA+: last AMCL','SFM near translation stop':'SFM: translation stop','SFM estimated center enters box':'SFM: AMCL in obstacle'}[label]
 ax.set(xlim=(actual['x']-2.,actual['x']+2.),ylim=(-1.5,2.5),title=f"{display}\n{row['elapsed']:.3f} s",xlabel='world x [m]',ylabel='world y [m]');ax.set_aspect('equal');ax.grid(alpha=.2)
 details.append(dict(label=label,amcl_elapsed_s=row['elapsed'],grid_offset_s=best['costmap'][0]-row['time'],actual_offset_s=best['gazebo'][0]-row['time'],actual=actual,amcl=row['amcl'],matched_footprint_occupancy_max=row.get('footprint_occupancy_max')))
axes[0,0].legend(fontsize=8,loc='upper left');fig.suptitle('Local grids in odom shifted by known spawn x=8; grey = factory boxes\nFootprints include padding; cost is not force contact');fig.subplots_adjust(left=.07,right=.86,top=.83,bottom=.08,hspace=.42,wspace=.3);cax=fig.add_axes([.9,.16,.02,.6]);fig.colorbar(im,cax=cax,label='local OccupancyGrid [0..100], zero transparent');fig.savefig(a.report_dir/'local_costmap_snapshots.png',dpi=160,bbox_inches='tight');plt.close(fig)
(a.report_dir/'local_costmap_snapshots.json').write_text(json.dumps(details,indent=2));print('Recorded local grids plotted')
