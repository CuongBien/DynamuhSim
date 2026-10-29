"""Plot recorded native costmaps at closest encounter; geometry metrics remain separate."""
import argparse,base64,json,math,zlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle,Polygon
from matplotlib.transforms import Affine2D
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
rows=lambda p:[json.loads(s) for s in p.read_text().splitlines() if s.strip()]
xy=lambda p:np.array([p['x'],p['y']])
rotation=lambda a:np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
p=argparse.ArgumentParser();p.add_argument('--results',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
if a.output.exists():raise FileExistsError(a.output)
plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];metrics={v['id']:v for v in json.loads(a.results.read_text())['cases']};fig,axes=plt.subplots(1,2,figsize=(15,6),layout='constrained')
for ax,case in zip(axes.flat,plan['cases']):
 name=case['id']
 if name not in metrics:ax.set_title(name+' — analysis unavailable');continue
 run=raw/name;metric=metrics[name];time=metric['proximity']['minimum_time_sim_s'];data=rows(run/'episode/gazebo_poses.jsonl');pose=min(data,key=lambda v:abs(v['time']-time));robot=pose['poses']['mecanum_robot'];native=[v for v in rows(run/'episode/native_costmaps.jsonl') if v['topic']=='/global_costmap/costmap_raw'];cm=min(native,key=lambda v:abs(v['stamp_ns']/1e9-time));stamp=cm['stamp_ns']/1e9;tf=min(rows(run/'episode/tf.jsonl'),key=lambda v:abs(v['time']-stamp));native_pose=min(data,key=lambda v:abs(v['time']-stamp))['poses']['mecanum_robot'];base=tf['map_base'];angle=native_pose['yaw']-base['yaw'];R=rotation(angle);tr=xy(native_pose)-R@xy(base);grid=np.frombuffer(zlib.decompress(base64.b64decode(cm['data'])),dtype='u1').reshape(cm['height'],cm['width']);o=cm['origin'];res=cm['resolution'];im=ax.imshow(np.ma.masked_equal(grid,255),origin='lower',extent=[o['x'],o['x']+cm['width']*res,o['y'],o['y']+cm['height']*res],transform=Affine2D().rotate(angle).translate(*tr)+ax.transData,cmap='YlOrRd',vmin=0,vmax=254,interpolation='nearest')
 from_xml=__import__('xml.etree.ElementTree',fromlist=['ElementTree']);world=from_xml.parse(run/'runtime/warehouse.sdf').getroot().find('world')
 for model in world.findall('model'):
  if model.findtext('static')!='true' or model.get('name') in ['floor','ground_plane']:continue
  box=model.find('link/collision/geometry/box/size')
  if box is None:continue
  center=list(map(float,model.findtext('pose','0 0 0 0 0 0').split()));size=list(map(float,box.text.split()));ax.add_patch(Rectangle(np.array(center[:2])-np.array(size[:2])/2,*size[:2],fill=False,ec='#30363f',lw=.8))
 for hid in range(1,7):
  h=pose['poses'][f'common_{hid}_proxy'];p=xy(h);ax.add_patch(Circle(p,.36,fc='#6b86b5',ec='black',alpha=.65));ax.arrow(*p,.6*math.cos(h['yaw']),.6*math.sin(h['yaw']),width=.018,head_width=.12,color='#283c5c');ax.text(p[0],p[1]+.42,f'P{hid}',ha='center',fontsize=8,clip_on=True)
 center=xy(robot);rot=rotation(robot['yaw']);ax.add_patch(Polygon(np.array([[-.12,-.1],[.12,-.1],[.12,.1],[-.12,.1]])@rot.T+center,fc='#11bbbd',ec='black',label='Native chassis24x20cm'))
 for wx in [-.08,.08]:
  for wy in [-.1,.1]:ax.add_patch(Circle(rot@np.array([wx,wy])+center,.04,fc='#222'))
 ax.add_patch(Circle(center,.168062484748657,fill=False,ec='#00a8bb',lw=1.8,label='Collision enclosing circle'))
 ax.set(xlim=(center[0]-3.5,center[0]+3.5),ylim=(center[1]-3,center[1]+3),aspect='equal',title=f"{name}: min native center distance{metric['proximity']['minimum_center_distance_m']:.3f}m\nCostmap snapshot vs encounter offset {stamp-time:+.3f}s",xlabel='World X (m)',ylabel='World Y (m)');ax.set_title(ax.get_title(),fontsize=11,pad=10);ax.legend(fontsize=7);ax.grid(alpha=.15)
fig.colorbar(im,ax=list(axes.flat),shrink=.65,label='Recorded global cost uint8 0..254 (255 unknown masked)');fig.suptitle('Closest encounters: native human/robot poses and recorded costmaps\nAsynchronous map/TF alignment is for display; acceptance uses actual native swept geometry.');fig.savefig(a.output,dpi=145);plt.close(fig);print(a.output)
