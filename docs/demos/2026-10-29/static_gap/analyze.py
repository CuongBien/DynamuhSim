#!/usr/bin/env python3
"""Read-only actual-trajectory and clearance evaluation for both predeclared gap cases."""
import argparse,base64,json,math,zlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Circle,Polygon
from matplotlib.transforms import Affine2D
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent;RADIUS=.168062484748657

def rows(p):return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]
def xy(p):return np.array([p['x'],p['y']])
def rotation(a):return np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
def nearest(data,t):return min(data,key=lambda p:abs(p['time']-t))
def compute(run):
    result=json.loads((run/'episode/result.json').read_text());spec=result['fixture'];events={e['event']:e['stamp_ns']/1e9 for e in rows(run/'episode/events.jsonl')};data=[r for r in rows(run/'episode/gazebo_poses.jsonl') if events['goal_accepted']<=r['time']<=events['action_end']];points=np.array([xy(r['poses']['mecanum_robot']) for r in data]);stamps=np.array([r['time'] for r in data]);steps=np.linalg.norm(np.diff(points,axis=0),axis=1);gap=math.inf;critical=None
    for box in spec['boxes']:
        center=np.array(box['center'][:2]);half=np.array(box['size'][:2])/2;delta=abs(points-center)-half;distance=np.linalg.norm(np.maximum(delta,0),axis=1)+np.minimum(np.max(delta,axis=1),0);bounds=np.minimum(distance[:-1],distance[1:])-.5*steps-RADIUS;i=int(np.argmin(bounds))
        if bounds[i]<gap:gap=float(bounds[i]);critical=dict(box=box['name'],interval_sim_s=[float(stamps[i]),float(stamps[i+1])])
    crossings=[]
    for i,(a,b) in enumerate(zip(points,points[1:])):
        if a[0]<=spec['gate_x']<b[0]:
            f=(spec['gate_x']-a[0])/(b[0]-a[0]);y=float(a[1]+f*(b[1]-a[1]));yaw0=data[i]['poses']['mecanum_robot']['yaw'];yaw1=data[i+1]['poses']['mecanum_robot']['yaw'];yaw=yaw0+f*math.atan2(math.sin(yaw1-yaw0),math.cos(yaw1-yaw0));crossings.append(dict(time_sim_s=float(stamps[i]+f*(stamps[i+1]-stamps[i])),y_m=y,yaw_rad=yaw,circle_fits_gap=abs(y)+RADIUS<spec['gap_width_m']/2,center_plane_clearance_m=spec['gap_width_m']/2-abs(y)-RADIUS))
    succeeded=result['navigation_status']=='SUCCEEDED' and result['actual_goal_reached'];crossed=any(c['circle_fits_gap'] for c in crossings);audit=json.loads((run.parent/(run.name+'_audit.json')).read_text())
    report=dict(case=run.name,gap_width_m=spec['gap_width_m'],navigation_status=result['navigation_status'],actual_goal_reached=result['actual_goal_reached'],world_goal_xy_error_m=result['goal_xy_error_world_m'],active_sim_s=result['active_sim_seconds'],path_length_m=float(steps.sum()),gate_crossings=crossings,passed_gap_with_bounding_circle=crossed,minimum_continuous_static_gap_lower_bound_m=gap,critical_clearance=critical,capture_audit=audit['status'],accepted=succeeded and crossed and gap>0 and audit['status']=='PASS',no_native_contact_force_stream=True,runtime_commit=result['head'],run=str(run.relative_to(ROOT)))
    return report,spec,data,points,result

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--report-dir',type=Path,default=HERE);args=parser.parse_args()
    for n in ['results.json','static_gap_comparison.png']:
        if (args.report_dir/n).exists():raise FileExistsError(args.report_dir/n)
    args.report_dir.mkdir(parents=True,exist_ok=True);plan=json.loads((HERE/'plan.json').read_text());all_data=[]
    for case in plan['cases']:all_data.append(compute(ROOT/plan['runtime']/case['id']))
    fig,axes=plt.subplots(2,2,figsize=(15,11),layout='constrained')
    for col,(m,spec,data,pts,result) in enumerate(all_data):
        ax=axes[0,col]
        for box in spec['boxes']:
            c=np.array(box['center'][:2]);s=np.array(box['size'][:2]);ax.add_patch(Rectangle(c-s/2,*s,color='#e4a03b' if box['name'].startswith('gate_') else '#8c939f'))
        ax.plot(pts[:,0],pts[:,1],color='#128a5c',lw=2,label='Actual recorded robot path');ax.plot(*pts[0],'bo',label='Start');ax.plot(-20,0,'r*',ms=12,label='Goal');ax.set_xlim(-30.3,-17.7);ax.set_ylim(-3.8,3.8);ax.set_aspect('equal');ax.legend(fontsize=8);ax.grid(alpha=.15);ax.set_title(f"{m['case']}: gap {m['gap_width_m']*100:.1f}cm\n{m['navigation_status']} | {m['active_sim_s']:.2f}s | accepted={m['accepted']}");ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)')
        run=ROOT/m['run'];t=m['gate_crossings'][0]['time_sim_s'] if m['gate_crossings'] else min(data,key=lambda x:abs(x['poses']['mecanum_robot']['x']+24))['time'];snapshots=[x for x in rows(run/'episode/native_costmaps.jsonl') if x['topic']=='/global_costmap/costmap_raw'];cm=min(snapshots,key=lambda x:abs(x['stamp_ns']/1e9-t));stamp=cm['stamp_ns']/1e9;tf=nearest(rows(run/'episode/tf.jsonl'),stamp);actual=nearest(data,stamp)['poses']['mecanum_robot'];base=tf['map_base'];angle=actual['yaw']-base['yaw'];R=rotation(angle);tr=xy(actual)-R@xy(base);grid=np.frombuffer(zlib.decompress(base64.b64decode(cm['data'])),dtype='u1').reshape(cm['height'],cm['width']);o=cm['origin'];res=cm['resolution'];ax=axes[1,col];affine=Affine2D().rotate(angle).translate(*tr);im=ax.imshow(np.ma.masked_equal(grid,255),origin='lower',extent=[o['x'],o['x']+cm['width']*res,o['y'],o['y']+cm['height']*res],transform=affine+ax.transData,cmap='YlOrRd',vmin=0,vmax=254,interpolation='nearest')
        for box in spec['boxes'][-2:]:
            c=np.array(box['center'][:2]);s=np.array(box['size'][:2]);ax.add_patch(Rectangle(c-s/2,*s,fill=False,ec='black',lw=1.3))
        plans=rows(run/'episode/plans.jsonl');p=nearest(plans,stamp);planned=np.array([[v['x'],v['y']] for v in p['poses']])@R.T+tr;ax.plot(planned[:,0],planned[:,1],color='#2862d0',lw=1.5,label='Recorded global plan (async)');ax.plot(pts[:,0],pts[:,1],color='#128a5c',label='Actual path');pose=nearest(data,t)['poses']['mecanum_robot'];center=xy(pose);rot=rotation(pose['yaw']);ax.add_patch(Polygon(np.array([[-.12,-.1],[.12,-.1],[.12,.1],[-.12,.1]])@rot.T+center,fc='#1eb9c7',ec='black',label='Native chassis'))
        for x in [-.08,.08]:
            for y in [-.1,.1]:ax.add_patch(Circle(rot@np.array([x,y])+center,.04,color='#333333'))
        ax.add_patch(Circle(center,RADIUS,fill=False,ec='#00a9c4',lw=2,ls='--',label='Native enclosing circle'));ax.set_xlim(-25.15,-22.85);ax.set_ylim(-.8,.8);ax.set_aspect('equal');ax.set_title(f"Actual gate passage and RECORDED native costmap\nMin continuous gap bound: {m['minimum_continuous_static_gap_lower_bound_m']*100:.2f}cm");ax.set_xlabel('world x (m)');ax.set_ylabel('world y (m)');ax.legend(fontsize=7,loc='upper left')
    fig.colorbar(im,ax=list(axes[1]),shrink=.6,label='Recorded native cost 0..254');fig.suptitle('Fixed bottleneck: native Mecanum + NavFn/Dijkstra + MPPI Omni\nNo humans, no forced waypoint or velocity commands; recorded physical trajectories',fontsize=16);fig.savefig(args.report_dir/'static_gap_comparison.png',dpi=145);plt.close(fig)
    report=dict(schema='static_gap_acceptance/1',status='PASS' if all(d[0]['accepted'] for d in all_data) else 'FAIL',cases=[d[0] for d in all_data],criteria=plan['criteria'],scope='Single attempt per width. Geometry/arrival and capture integrity only; no moving-people or social-policy claim. Contact-force truth unavailable.')
    (args.report_dir/'results.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':main()
