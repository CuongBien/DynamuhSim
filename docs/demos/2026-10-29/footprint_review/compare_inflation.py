from pathlib import Path
import json,base64,zlib,math,os
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle,Polygon
from matplotlib.colors import ListedColormap
OUT=Path(__file__).resolve().parent;BASE=OUT.parent;EP=BASE/'capture/episode'
footprints=json.loads((OUT/'footprints.json').read_text());geometry=json.loads((OUT/'geometry.json').read_text())
load=lambda key:[json.loads(x) for x in (EP/(key+'.jsonl')).read_text().splitlines()]
cm=max([x for x in load('local_costmaps') if x['time']<=33],key=lambda x:x['time']);t=cm['time'];near=lambda key:min(load(key),key=lambda x:abs(x['time']-t));od=near('odom');gt=near('ground_truth');recorded=np.frombuffer(zlib.decompress(base64.b64decode(cm['data'])),dtype=np.int8).reshape(cm['height'],cm['width']);lethal=recorded==100;D=distance_transform_edt(~lethal)*cm['resolution']
yaw=od['yaw']-gt['robot']['yaw'];R=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]]);tr=np.array([od['x'],od['y']])-R@np.array([gt['robot']['x'],gt['robot']['y']]);humans={h['id']:np.array([h['x'],h['y']])@R.T+tr for h in gt['humans']}
x0,y0=cm['origin']['x'],cm['origin']['y'];res=cm['resolution']
def field(inscribed,inflation):
 c=np.where(D==0,254,np.where(D<=inscribed,253,np.floor(252*np.exp(-3*(D-inscribed)))));c[D>inflation+1e-7]=0
 return np.where(c==254,100,np.where(c==253,99,np.where(c==0,0,1+(97*(c-1)//251)))).astype('i1')
def cell(a,xy):
 x,y=np.floor((xy-[x0,y0])/res).astype(int);return int(a[y,x])
baseline=field(footprints[0]['inscribed_radius'],.8);assert np.mean(baseline==recorded)>.999
variants=[('Recorded current',recorded,footprints[0],.8),('ONLY reduce inflation to0.45m',field(footprints[0]['inscribed_radius'],.45),footprints[0],.45),('ONLY fit circular radius to0.17m',field(footprints[1]['inscribed_radius'],.8),footprints[1],.8),('Fit radius0.17m + inflation0.45m',field(footprints[1]['inscribed_radius'],.45),footprints[1],.45)]
colors=['#eeeeee']+list(plt.get_cmap('YlOrRd')(np.linspace(.08,.8,98)))+['#b71632','#4c0010'];cmap=ListedColormap(colors)
fig,axes=plt.subplots(2,3,figsize=(17,10),layout='constrained');ax=axes[0,0]
for c in geometry['collision_entries']:
 center=c['center'];shape=c['shape'];values=c['shape_values']
 if shape=='box':ax.add_patch(Rectangle((center[0]-values[0]/2,center[1]-values[1]/2),values[0],values[1],facecolor='#34bed0',edgecolor='black',label='Gazebo chassis24x20cm'))
 else:ax.add_patch(Circle(center[:2],values,facecolor='#444444',edgecolor='black'))
ax.add_patch(Rectangle((-.14,-.11),.28,.22,fill=False,edgecolor='#bc65c3',ls=':',lw=2,label='URDF chassis28x22cm'))
for fp,color,title in [(footprints[0],'#db3439','Current Nav2:52cm wide'),(footprints[1],'#15864e','Hypothetical fitted circle:36cm wide')]:ax.add_patch(Polygon(fp['points'],fill=False,edgecolor=color,lw=2,ls='--',label=title))
ax.set_xlim(-.31,.31);ax.set_ylim(-.31,.31);ax.set_aspect('equal');ax.set_title('Robot geometry at SAME SCALE\nCollision extent24x28cm incl. wheels');ax.set_xlabel('robot x (m)');ax.set_ylabel('robot y (m)');ax.legend(fontsize=7,loc='upper left');ax.grid(alpha=.2)
locations=[(0,1),(0,2),(1,0),(1,1)];details=[];mid=(humans[5]+humans[6])/2
for (title,a,fp,inflation),(i,j) in zip(variants,locations):
 ax=axes[i,j];im=ax.imshow(a,origin='lower',extent=[x0,x0+a.shape[1]*res,y0,y0+a.shape[0]*res],cmap=cmap,vmin=0,vmax=100,interpolation='nearest')
 for hid,xy in humans.items():ax.add_patch(Circle(xy,.36,fill=False,ec='black',lw=1));ax.text(*xy,f'H{hid}',ha='center',va='center',fontsize=8,clip_on=True)
 ax.plot(*mid,'cx',ms=8,mew=2);ax.set_xlim(3.5,7.1);ax.set_ylim(-1.3,2.2);ax.set_aspect('equal');value=cell(a,mid);ax.set_title(title+f'\nGap A midpoint cost={value}; footprint width={fp["width_y"]:.2f}m',fontsize=11);ax.set_xlabel('odom x (m)');ax.set_ylabel('odom y (m)');details.append({'name':title,'recorded':title=='Recorded current','robot_radius':fp['robot_radius'],'padding':fp['padding'],'inscribed_radius':fp['inscribed_radius'],'inflation_radius':inflation,'midpoint_cost':value,'inscribed_or_lethal_cells':int((a>=99).sum()),'soft_cost_cells':int(((a>0)&(a<99)).sum())})
fig.colorbar(im,ax=[axes[i,j] for i,j in locations],shrink=.55,label='Translated OccupancyGrid cost0..100')
ax=axes[1,2];names=['Current','Inflation only','Radius only','Both'];hard=np.array([x['inscribed_or_lethal_cells'] for x in details]);soft=np.array([x['soft_cost_cells'] for x in details]);ax.bar(names,hard,color='#a92037',label='Inscribed/lethal cost99..100');ax.bar(names,soft,bottom=hard,color='#edb653',label='Soft cost1..98');ax.set_title('Inscribed core vs soft inflation\nOffline diagnostic; no controller rerun');ax.set_ylabel('Number of cells');ax.tick_params(axis='x',rotation=15);ax.legend(fontsize=8);ax.grid(axis='y',alpha=.25)
fig.suptitle('Your observation is correct: Nav2 footprint is much larger than the simulated robot\nOnly reducing inflation does not shrink the inscribed collision core',fontsize=16);fig.savefig(OUT/'footprint_inflation_comparison.png',dpi=145);plt.close(fig)
report={'geometry':geometry,'installed_nav2_library_probe':footprints,'comparison_time_sim_s':33.,'costmap_stamp_s':t,'baseline_reconstruction':{'matching_cells_fraction':float(np.mean(recorded==baseline)),'mismatched_cells':int((recorded!=baseline).sum()),'max_abs_cost_difference':int(np.abs(recorded.astype(int)-baseline.astype(int)).max()),'method':'Distance transform of recorded lethal cells, Jazzy inflation computeCost with installed-library inscribed radius, translated to OccupancyGrid. Diagnostic approximation; recorded marking layer held fixed.'},'variants':details,'controller_rerun_performed':False,'candidate_applied_to_runtime':False,'candidate_validated_for_real_robot':False,'current_recipe_unchanged':True,'urdf_tf_static':json.loads((OUT/'actual_tf_static.json').read_text()),'known_model_mismatch':'SDF chassis24x20cm, URDF28x22cm; saved TF camera_link x0.14m versus SDF mount0.12m. No geometry or TF fix applied in this inspection.','scope':'Parameter/geometry diagnosis, not proof a shortcut is safe or a full social navigation acceptance.'}
(OUT/'comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(details,indent=2));print('baseline',report['baseline_reconstruction'])
