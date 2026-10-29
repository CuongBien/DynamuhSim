#!/usr/bin/env python3
"""Read-only physical A/B comparison of recorded poses and actual local costmaps."""
import base64,json,math,zlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
RADIUS=.168062484748657
HUMAN=.36

def rows(path):return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]
def xy(p):return np.array([p['x'],p['y']])
def nearest(data,t):return min(data,key=lambda x:abs(x['time']-t))
def rotation(a):return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
def gate(row):
 p=row['poses'];a,b=xy(p['common_5_proxy']),xy(p['common_6_proxy']);u=(b-a)/np.linalg.norm(b-a);n=np.array([-u[1],u[0]])
 if n[0]<0:n=-n
 rel=xy(p['mecanum_robot'])-(a+b)/2
 return float(rel@n),float(rel@u),float(np.linalg.norm(b-a))
def static_boxes(run):
 boxes=[]
 for m in ET.parse(run/'runtime/warehouse.sdf').getroot().findall('world/model'):
  if m.findtext('static')!='true' or m.get('name')=='floor' or m.get('name').endswith('_proxy'):continue
  def pose(node):
   v=[float(x) for x in node.findtext('pose','0 0 0 0 0 0').split()]
   if len(v)!=6 or any(abs(x)>1e-10 for x in v[3:]):raise ValueError('Static diagnostic requires axis-aligned boxes')
   return np.array(v[:3])
  for link in m.findall('link'):
   for c in link.findall('collision'):
    size=c.findtext('geometry/box/size')
    if size is None:raise ValueError('Unsupported static collision')
    center=pose(m)+pose(link)+pose(c);half=np.array([float(x) for x in size.split()])/2
    boxes.append((m.get('name'),center[:2]-half[:2],center[:2]+half[:2]))
 return boxes

def planned_world(run,data,target):
 plan=nearest(rows(run/'episode/plans.jsonl'),target);tf=nearest(rows(run/'episode/tf.jsonl'),plan['time']);actual=nearest(data,plan['time'])['poses']['mecanum_robot'];base=tf['map_base'];R=rotation(actual['yaw']-base['yaw'])
 return (np.array([[v['x'],v['y']] for v in plan['poses']])-xy(base))@R.T+xy(actual),plan['time']

def compute(run):
 result=json.loads((run/'episode/result.json').read_text());data=rows(run/'episode/gazebo_poses.jsonl');events=rows(run/'episode/events.jsonl');times={e['event']:e['stamp_ns']/1e9 for e in events};active=[x for x in data if times['goal_accepted']<=x['time']<=times['action_end']];pts=np.array([xy(x['poses']['mecanum_robot']) for x in active]);minimum=math.inf;crossings=[]
 for a,b in zip(active,active[1:]):
  p0,p1=a['poses'],b['poses'];robot0,robot1=xy(p0['mecanum_robot']),xy(p1['mecanum_robot'])
  for name in p0:
   if not name.endswith('_proxy') or name not in p1:continue
   rel0=robot0-xy(p0[name]);delta=(robot1-xy(p1[name]))-rel0;v=float(delta@delta);f=np.clip(-float(rel0@delta)/v,0,1) if v else 0
   minimum=min(minimum,float(np.linalg.norm(rel0+f*delta))-RADIUS-HUMAN)
  da,sa,la=gate(a);db,sb,lb=gate(b)
  if da<=0<db:
   f=-da/(db-da);along=sa+f*(sb-sa);length=la+f*(lb-la);opening=length-2*HUMAN
   if abs(along)<length/2-HUMAN:
    dt=b['time']-a['time'];speed=max(float(np.linalg.norm(xy(p1[n])-xy(p0[n])))/dt for n in ['common_5_proxy','common_6_proxy'])
    initial_shift=max(float(np.linalg.norm(xy(p0[n])-xy(active[0]['poses'][n]))) for n in ['common_5_proxy','common_6_proxy'])
    crossings.append(dict(time_sim_s=a['time']+f*dt,time_after_goal_s=a['time']+f*dt-times['goal_accepted'],surface_opening_m=opening,center_offset_m=along,conservative_disc_fits=abs(along)+RADIUS<opening/2,pair_speed_m_s=speed,pair_shift_from_start_m=initial_shift,original_stationary_pair=speed<.05 and initial_shift<.1))
 boxes=static_boxes(run);static_gap=math.inf
 for name,lo,hi in boxes:
  distances=np.linalg.norm(np.maximum(np.maximum(lo-pts,pts-hi),0),axis=1)
  lower=np.minimum(distances[:-1],distances[1:])-.5*np.linalg.norm(np.diff(pts,axis=0),axis=1)-RADIUS
  static_gap=min(static_gap,float(lower.min()))
 planned,plan_stamp=planned_world(run,data,times['goal_accepted']+20)
 gate_point=planned[np.argmin(abs(planned[:,0]+22.86))]
 pose=result['final_world_pose'];m=dict(run=str(run.relative_to(ROOT)),status=result['navigation_status'],actual_goal_reached=result['actual_goal_reached'],xy_error_m=result['goal_xy_error_world_m'],active_sim_s=result['active_sim_seconds'],path_length_m=float(np.linalg.norm(np.diff(pts,axis=0),axis=1).sum()),world_y_min_m=float(pts[:,1].min()),world_y_max_m=float(pts[:,1].max()),minimum_swept_robot_human_gap_m=minimum,minimum_static_gap_lower_bound_m=static_gap,static_box_count=len(boxes),global_plan_probe=dict(time_sim_s=plan_stamp,nearest_point_to_initial_gate_x=gate_point.tolist()),native_contact_force_recorded=False,gate_A_crossings=crossings,passes_original_narrow_gap_A=any(c['original_stationary_pair'] and c['conservative_disc_fits'] for c in crossings),rendering=result['rendering']['renderer'])
 return m,active,pts

def main():
 plan=json.loads((OUT/'plan.json').read_text());runs=[('A old radius .25 / inflation .80',ROOT/plan['baseline'])]+[(a['id'],ROOT/a['run']) for a in plan['arms']];all_data=[]
 for name,run in runs:
  m,data,pts=compute(run);m['arm']=name;all_data.append((m,data,pts,run))
 report=dict(schema='geometry_ab_result/1',arms=[v[0] for v in all_data],limitations=plan['comparison_limits'],contact_scope='Swept conservative circular envelopes against recorded human proxy cylinders, linearly interpolated between samples. Static clearance is a conservative continuous segment lower bound against all upright native obstacle boxes, excluding the supporting floor. Neither metric establishes native contact-force truth.',gap_A_definition='Cross H5-H6 moving connecting segment within free surface opening; original stationary pair requires speed<0.05m/s and <0.1m shift from goal start.')
 (OUT/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
 fig,axes=plt.subplots(2,3,figsize=(18,10),layout='constrained')
 for col,(m,data,pts,run) in enumerate(all_data):
  ax=axes[0,col];ax.plot(pts[:,0],pts[:,1],color='#1b8c61',lw=2,label='Actual Gazebo robot path');initial=data[0]['poses']
  for name,lo,hi in static_boxes(run):ax.add_patch(Rectangle(lo,*(hi-lo),color='#8b939d',alpha=.6))
  for i in range(1,7):
   h=initial[f'common_{i}_proxy'];ax.add_patch(Circle(xy(h),HUMAN,color='#deb34c',alpha=.65));ax.text(h['x'],h['y'],f'H{i}',ha='center',va='center',fontsize=8)
  ax.plot([-22.86,-22.86],[-.263538,.263538],color='#b82630',lw=3,label='Initial narrow opening A')
  ax.plot(*pts[0],'bo',label='Start');ax.plot(-20,0,'r*',ms=12,label='Goal');ax.set_xlim(-28.5,-19.5);ax.set_ylim(-2.6,2.8);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_title(m['arm']+'\n'+m['status']+f" | {m['active_sim_s']:.1f}s | {m['path_length_m']:.2f}m");ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.legend(fontsize=7,loc='upper left')
  target=min(data,key=lambda x:abs(x['poses']['mecanum_robot']['x']+23.275));costmaps=rows(run/'episode/local_costmaps.jsonl');cm=nearest(costmaps,target['time']);actual=nearest(data,cm['time'])['poses'];od=nearest(rows(run/'episode/odom.jsonl'),cm['time']);robot=actual['mecanum_robot'];R=rotation(robot['yaw']-od['yaw']);tr=xy(robot)-R@xy(od)
  grid=np.frombuffer(zlib.decompress(base64.b64decode(cm['data'])),dtype=np.int8).reshape(cm['height'],cm['width']);x0=cm['origin']['x'];y0=cm['origin']['y'];res=cm['resolution'];corners=np.array([[x0,y0],[x0+cm['width']*res,y0],[x0+cm['width']*res,y0+cm['height']*res],[x0,y0+cm['height']*res]])@R.T+tr
  from matplotlib.transforms import Affine2D
  ax=axes[1,col];affine=Affine2D().rotate(robot['yaw']-od['yaw']).translate(*tr);im=ax.imshow(grid,origin='lower',extent=[x0,x0+cm['width']*res,y0,y0+cm['height']*res],transform=affine+ax.transData,cmap='YlOrRd',vmin=0,vmax=100,interpolation='nearest')
  for i in range(1,7):
   h=actual[f'common_{i}_proxy'];ax.add_patch(Circle(xy(h),HUMAN,fill=False,edgecolor='black'));ax.text(h['x'],h['y'],f'H{i}',ha='center',va='center',fontsize=8,clip_on=True)
  planned,plan_stamp=planned_world(run,data,cm['time']);ax.plot(planned[:,0],planned[:,1],color='#2364db',lw=1.5,label='Recorded global plan');ax.plot(pts[:,0],pts[:,1],color='#108c55',lw=1.5,label='Actual robot path');ax.legend(fontsize=7);ax.add_patch(Circle(xy(robot),.26 if col==0 else .18,fill=False,edgecolor='#00abde',lw=2));ax.set_xlim(-25.1,-20.7);ax.set_ylim(-1.6,2.8);ax.set_aspect('equal');ax.set_title(f"RECORDED local costmap, sim {cm['time']:.1f}s\nNarrow A passed: {m['passes_original_narrow_gap_A']}; min human gap: {m['minimum_swept_robot_human_gap_m']:.3f}m");ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)')
 fig.colorbar(im,ax=list(axes[1]),shrink=.6,label='Recorded OccupancyGrid cost (0..100)');fig.suptitle('Physical reruns: corrected robot geometry and inflation (standing, seed 45)\nPaths and costmaps are recorded data; no AI-generated images',fontsize=16);fig.savefig(OUT/'physical_comparison.png',dpi=145);plt.close(fig)
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
