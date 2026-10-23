#!/usr/bin/env python3
"""Compare actual SceneBroadcaster poses with the original logged XY/yaw."""
from pathlib import Path
import os,json,time,bisect,math,sys
HERE=Path(__file__).resolve().parent
session=json.loads((HERE/'generated/latest.json').read_text());os.environ['GZ_PARTITION']=session['partition']
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
rows=[json.loads(x) for x in Path(session['source']).read_text().splitlines() if json.loads(x)['phase']=='active']
ts=[x['time'] for x in rows];begin,end=ts[0],ts[-1];duration=end-begin
samples=[];errors=[];positions=[];names=set()
def callback(msg):
 st=msg.header.stamp.sec+msg.header.stamp.nsec*1e-9;t=min(begin+st%(duration+5),end)
 i=max(0,bisect.bisect_right(ts,t)-1);k=min(i+1,len(ts)-1);a=0 if i==k else max(0,min(1,(t-ts[i])/(ts[k]-ts[i])))
 for p in msg.pose:
  if p.name not in rows[i]['poses']:continue
  u,v=rows[i]['poses'][p.name],rows[k]['poses'][p.name];x=u['x']+a*(v['x']-u['x']);y=u['y']+a*(v['y']-u['y']);yaw=u['yaw']+a*math.remainder(v['yaw']-u['yaw'],2*math.pi)
  q=p.orientation;actual_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
  err=max(abs(x-p.position.x),abs(y-p.position.y),abs(math.remainder(yaw-actual_yaw,2*math.pi)));errors.append(err);names.add(p.name)
  if p.name=='mecanum_robot':positions.append([p.position.x,p.position.y])
 samples.append({'sim_time':st,'recorded_time':t,'model_count':sum(p.name in rows[i]['poses'] for p in msg.pose)})
node=Node();assert node.subscribe(Pose_V,'/world/wm_recorded_replay/dynamic_pose/info',callback)
time.sleep(8)
travel=max((math.dist(x,positions[0]) for x in positions),default=0)
result={'status':'PASS' if errors and max(errors)<1e-8 and len(names)==7 and travel>.001 else 'FAIL','messages':len(samples),'compared_model_poses':len(errors),'max_xy_yaw_error':max(errors,default=None),'unique_models':sorted(names),'robot_displacement_during_check_m':travel,'first':samples[0] if samples else None,'last':samples[-1] if samples else None,'method':'Independent SceneBroadcaster dynamic Pose_V callback versus source JSONL and Python interpolation at published simulation timestamp. No plugin status values used.'}
(Path(session['output'])/'verification.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));sys.exit(0 if result['status']=='PASS' else 1)
