from pathlib import Path
import json,base64,zlib,math,os,bisect
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Polygon
from matplotlib.colors import ListedColormap,BoundaryNorm
ROOT=Path('/home/cuongbien/DynamuhSim');BASE=ROOT/'experiments/social_navigation/benchmark/generated/visual_demo_20261029T101053';EP=BASE/'capture/episode';OUT=Path(__file__).resolve().parent
load=lambda key:[json.loads(x) for x in (EP/(key+'.jsonl')).read_text().splitlines()]
streams={k:load(k) for k in ['local_costmaps','plans','odom','ground_truth','tf','commands']}
for v in streams.values():v.sort(key=lambda x:x['time'])
result=json.loads((EP/'result.json').read_text());frame_times=result['media']['frame_times'];radius=.26
nearest=lambda key,t:min(streams[key],key=lambda x:abs(x['time']-t))
def latest(key,t):
 rows=[x for x in streams[key] if x['time']<=t];return rows[-1] if rows else streams[key][0]
def rotation(yaw):return np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
def snapshot(t):
 cm=latest('local_costmaps',t);stamp=cm['time'];od=nearest('odom',stamp);gt=nearest('ground_truth',stamp);tf=latest('tf',stamp);plan=latest('plans',stamp)
 a=np.frombuffer(zlib.decompress(base64.b64decode(cm['data'])),dtype=np.int8).reshape(cm['height'],cm['width'])
 # For diagnostic drawings only, align GT world geometry to the odom costmap through simultaneous ego poses.
 R=rotation(od['yaw']-gt['robot']['yaw']);tr=np.array([od['x'],od['y']])-R@np.array([gt['robot']['x'],gt['robot']['y']]);world_to_odom=lambda xy:np.asarray(xy)@R.T+tr
 humans=[dict(id=h['id'],xy=world_to_odom([h['x'],h['y']]),world=np.array([h['x'],h['y']]),radius=h['radius']) for h in gt['humans']]
 # Recorded /plan is in map; transform map->odom using the actual logged estimated TF, not world GT.
 mo=tf['map_odom'];path=(np.array([[p['x'],p['y']] for p in plan['poses']])-[mo['x'],mo['y']])@rotation(mo['yaw'])
 odpath=np.array([[x['x'],x['y']] for x in streams['odom'] if x['time']<=stamp]);future=np.array([[x['x'],x['y']] for x in streams['odom'] if stamp<=x['time']<=stamp+8])
 def sample_cost(world):
  xy=world_to_odom(world);ij=np.floor((xy-np.array([cm['origin']['x'],cm['origin']['y']]))/cm['resolution']).astype(int);x,y=ij
  return int(a[y,x]) if 0<=x<a.shape[1] and 0<=y<a.shape[0] else None
 return dict(cm=cm,grid=a,odom=od,gt=gt,tf=tf,plan=plan,humans=humans,path=path,actual=odpath,future=future,sample_cost=sample_cost,world_to_odom=world_to_odom)
colors=['#eeeeee']+list(plt.get_cmap('YlOrRd')(np.linspace(.08,.8,98)))+['#b71632','#4c0010']
cmap=ListedColormap(colors);cmap.set_bad('#8795a4')
def draw(ax,s,title):
 cm=s['cm'];a=s['grid'];x0,y0=cm['origin']['x'],cm['origin']['y'];res=cm['resolution'];im=ax.imshow(np.ma.masked_less(a,0),origin='lower',extent=[x0,x0+a.shape[1]*res,y0,y0+a.shape[0]*res],cmap=cmap,vmin=0,vmax=100,interpolation='nearest')
 for h in s['humans']:
  ax.add_patch(Circle(h['xy'],h['radius'],fill=False,ec='#2a2a2a',lw=1.3));ax.text(*h['xy'],f"H{h['id']}",ha='center',va='center',fontsize=9,color='#163952',clip_on=True)
 od=s['odom'];xy=np.array([od['x'],od['y']]);ax.add_patch(Circle(xy,radius,fill=False,ec='#009eca',lw=2.2,ls='--',label='Nav2 nominal radius ~0.26m'))
 chassis=np.array([[-.12,-.10],[.12,-.10],[.12,.10],[-.12,.10]])@rotation(od['yaw']).T+xy;ax.add_patch(Polygon(chassis,fc='#0bbdda',ec='black',label='Visible chassis 0.24 x 0.20m'))
 ax.arrow(*xy,.42*math.cos(od['yaw']),.42*math.sin(od['yaw']),width=.015,color='#008ab4',length_includes_head=True)
 ax.plot(*s['path'].T,'-',color='#3264cf',lw=2,label='Recorded global /plan (map -> odom)');ax.plot(*s['actual'].T,color='#15a85b',lw=2,label='Executed odometry (past)')
 if len(s['future']):ax.plot(*s['future'].T,color='#00b577',lw=1.6,ls=':',label='Next 8s executed (diagnostic future)')
 ax.set_xlim(x0,x0+a.shape[1]*res);ax.set_ylim(y0,y0+a.shape[0]*res);ax.set_aspect('equal');ax.set_title(title+f"\nCostmap stamp {cm['time']:.3f}s, 5cm cells",fontsize=11);ax.set_xlabel('odom x (m)');ax.set_ylabel('odom y (m)');return im
fig,axes=plt.subplots(2,3,figsize=(18,11),layout='constrained');data=[]
for col,(video,t) in enumerate([(1.0,17.),(5.,33.),(10.,53.)]):
 s=snapshot(t);im=draw(axes[0,col],s,f'Video {video:g}s -> sim {t:g}s');data.append({'video_s':video,'sim_s':t,'costmap_stamp_s':s['cm']['time'],'plan_stamp_s':s['plan']['time'],'tf_stamp_s':s['tf']['time'],'robot_odom':s['odom'],'command':nearest('commands',t)})
 if col==0:axes[0,col].legend(loc='upper left',fontsize=7)
fig.colorbar(im,ax=list(axes[0,:]),shrink=.6,label='Recorded OccupancyGrid cost 0..100 (translated, not native 0..255)')
s=snapshot(33.);ax=axes[1,0];hs={h['id']:h for h in s['humans']}
for h in s['humans']:
 ax.add_patch(Circle(h['world'],.36,facecolor='#8dc9e3',edgecolor='#16425a',alpha=.8));ax.text(*h['world'],f"H{h['id']}",ha='center',va='center')
 ax.add_patch(Circle(h['world'],.36+.8,fill=False,ec='#e58a81',lw=.9,ls=':',alpha=.65))
r=s['gt']['robot'];xy=np.array([r['x'],r['y']]);ax.add_patch(Circle(xy,.26,fill=False,ec='#009eca',lw=2,ls='--'));ax.add_patch(Polygon(np.array([[-.12,-.1],[.12,-.1],[.12,.1],[-.12,.1]])@rotation(r['yaw']).T+xy,fc='#15bfd4'));ax.text(*xy,' robot',ha='left',va='bottom')
pairs=[]
for h1,h2,label,color in [(5,6,'A','#ae2241'),(1,5,'B','#168250')]:
 a,b=hs[h1]['world'],hs[h2]['world'];unit=(b-a)/np.linalg.norm(b-a);aedge=a+unit*.36;bedge=b-unit*.36;mid=(a+b)/2;gap=float(np.linalg.norm(b-a)-.72);cost=s['sample_cost'](mid);ax.annotate('',xy=aedge,xytext=bedge,arrowprops={'arrowstyle':'<->','color':color,'lw':2});ax.text(*(mid+[-.1,-.09]),f'{label}: {gap:.3f}m',fontsize=9,color=color,ha='right');pairs.append(dict(label=label,humans=[h1,h2],proxy_surface_gap_m=gap,nominal_nav2_diameter_m=.52,remaining_geometric_width_m=gap-.52,recorded_midpoint_cost=cost))
ax.set_xlim(-27.1,-20.7);ax.set_ylim(-1.55,2.4);ax.set_aspect('equal');ax.set_title('True geometry at sim33s (diagnostic GT only)\nProxy radius0.36m; dotted inflation extent is SOFT cost');ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)')
ax=axes[1,1];a,b=hs[5]['world'],hs[6]['world'];u=np.linspace(0,1,300);world=a[None,:]+u[:,None]*(b-a)[None,:];vals=[s['sample_cost'](p) for p in world];dist=u*np.linalg.norm(b-a)
ax.plot(dist,vals,color='#b71632');ax.axhspan(98.5,100.5,color='red',alpha=.12,label='99 inscribed / 100 lethal');ax.axvspan(.36+.26,np.linalg.norm(b-a)-.36-.26,color='#00b8d4',alpha=.35,label='Nominal center window (~7mm)');ax.set_ylim(-3,105);ax.set_xlabel('Distance from H5 center toward H6 (m)');ax.set_ylabel('Recorded cost (0..100)');ax.set_title('Gap A cross-section: H5 -> H6\nSmall chassis fits; planned footprint has ~7mm spare');ax.legend(fontsize=8);ax.grid(alpha=.25)
ax=axes[1,2];cmd=[x for x in streams['commands'] if 13<=x['time']<=57];video=(np.array([x['time'] for x in cmd])-13)/4;speed=np.hypot([x['vx'] for x in cmd],[x['vy'] for x in cmd]);ax.plot(video,speed,label='command planar speed (m/s)',color='#099aa7');other=ax.twinx();other.plot(video,[x['wz'] for x in cmd],color='#b45727',alpha=.7,label='command wz (rad/s)');ax.axvline(5,color='black',ls=':');ax.set_xlabel('Web video time (s, encoded4x)');ax.set_ylabel('Planar speed (m/s)',color='#099aa7');other.set_ylabel('Angular speed (rad/s)',color='#b45727');ax.set_title('At video5s: nearly stopped and turning\nMPPI candidate rollouts/critic scores were NOT logged');ax.grid(alpha=.25)
fig.suptitle('Recorded navigation diagnostics: NavFn/Dijkstra global path + MPPI Omni local control',fontsize=17);fig.savefig(OUT/'navigation_costmap.png',dpi=145);plt.close(fig)
# A larger single view at the point the user identified.
fig,ax=plt.subplots(figsize=(10,9),layout='constrained');im=draw(ax,s,'WEB VIDEO 5s = SIM33s');ax.legend(loc='upper left',fontsize=8);fig.colorbar(im,ax=ax,shrink=.65,label='Recorded local costmap 0..100');fig.savefig(OUT/'costmap_video5s.png',dpi=150);plt.close(fig)
summary={'source':'Actual logged local OccupancyGrid, odom, TF and /plan; no new Nav2 run or invented critic scores','video_clock':'Encoded4fps from recorded1Hz camera: frame20 at video5s corresponds to sim33s. PlaybackRate changes wall playback speed, not this timecode mapping.','video_to_sim':data,'gaps_at_sim33s':pairs,'footprint':'Nav2 robot_radius0.25 + padding0.01; nominal circle0.26 drawn (padding details/grid discretization retained as approximation)','inflation_radius_m':.8,'cost_scaling_factor':3.,'chassis_size_m':[.24,.20],'human_proxy_radius_m':.36,'no_global_costmap_array_recorded':True,'counterfactual_shortcut_tested':False,'mppi_critic_scores_recorded':False,'live_controller_inputs':'LiDAR costmap and estimated TF/odom; GT only used to draw geometry overlays','algorithm':{'global':'nav2_navfn_planner::NavfnPlanner, use_astar:false (Dijkstra)','local':'nav2_mppi_controller::MPPIController, Omni,20Hz,280samples,30steps x0.05s','localization':'AMCL OmniMotionModel'}}
(OUT/'navigation_analysis.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary['gaps_at_sim33s'],indent=2))
