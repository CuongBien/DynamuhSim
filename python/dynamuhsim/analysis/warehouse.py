"""Independent measurements for the three warehouse recording analyzers.

Uses recorded Gazebo poses and GEOS, never the motion guard's geometry. Proxy
radii and sensor selection retain the legacy warehouse definitions; acceptance
thresholds, source checks and lifecycle/fault policies stay in the entrypoints.
"""
import bisect
import math
import numpy as np


def robot_human_gaps(before, after, robot_before, robot_after):
    """Minimum signed robot–human gap for each measured linear interval."""
    a = np.asarray(before); b = np.asarray(after)
    ra = np.asarray(robot_before); rb = np.asarray(robot_after)
    rel = a-ra[:,None,:]; vel = (b-a)-(rb-ra)[:,None,:]; den = np.sum(vel*vel,axis=2)
    t = np.clip(np.divide(-np.sum(rel*vel,axis=2), den, out=np.zeros_like(den), where=den>1e-25), 0, 1)
    return np.min(np.linalg.norm(rel+t[:,:,None]*vel,axis=2)-.61,axis=1)


def measured_gaps(before, after, robot_before, robot_after, *, polygons):
    """Signed swept gaps for 0.36m human and 0.25m robot discs."""
    import shapely
    a = np.asarray(before); b = np.asarray(after)
    ra = np.asarray(robot_before); rb = np.asarray(robot_after)
    delta = b-a; pair = 1e9
    for i in range(a.shape[1]):
        for j in range(i):
            rel = a[:,i]-a[:,j]; vel = delta[:,i]-delta[:,j]; den = np.sum(vel*vel, axis=1)
            t = np.clip(np.divide(-np.sum(rel*vel,axis=1), den, out=np.zeros_like(den), where=den>1e-25), 0, 1)
            pair = min(pair, float(np.min(np.linalg.norm(rel+t[:,None]*vel,axis=1)-.72)))
    robot = float(np.min(robot_human_gaps(a,b,ra,rb)))
    segments = shapely.linestrings(np.stack((a,b),axis=2).reshape(-1,2,2))
    static = float(np.min(shapely.distance(segments[:,None],polygons[None,:]))-.36)
    return dict(pair=pair, robot=robot, static=static)


def trajectory_errors(complete, worker, dt, proxies, pose_times):
    """Replay worker interpolation and one-step robot prediction against poses."""
    worker_times = [row['time'] for row in worker]
    errors = []; robot_prediction = []
    for row in complete:
        index = bisect.bisect_right(worker_times,row['time'])-1
        if index < 0:
            continue
        w = worker[index]; alpha = (row['time']-w['time'])/dt
        if not -.00001 <= alpha <= 1.00001:
            continue
        for old,new in zip(w['before'],w['humans']):
            actual = row['poses'][proxies[old['id']]]
            expected = [old[k]+alpha*(new[k]-old[k]) for k in ['x','y']]
            errors.append(math.hypot(actual['x']-expected[0],actual['y']-expected[1]))
    for w in worker:
        i = min(range(max(0,bisect.bisect_left(pose_times,w['time']+dt)-1),
                      min(len(complete),bisect.bisect_left(pose_times,w['time']+dt)+2)),
                key=lambda i:abs(pose_times[i]-w['time']-dt))
        if abs(pose_times[i]-w['time']-dt) < .015:
            r = w['robot_input']; actual = complete[i]['poses']['mecanum_robot']
            robot_prediction.append(math.hypot(actual['x']-r['x']-dt*r['vx'],actual['y']-r['y']-dt*r['vy']))
    return errors, robot_prediction


def pose_intervals(complete, proxies):
    """Adjacent measured pose segments in the caller's human ID order."""
    before = [[[s['poses'][name][k] for k in ['x','y']] for name in proxies.values()] for s in complete[:-1]]
    after = [[[s['poses'][name][k] for k in ['x','y']] for name in proxies.values()] for s in complete[1:]]
    rb = [[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[:-1]]
    re = [[s['poses']['mecanum_robot'][k] for k in ['x','y']] for s in complete[1:]]
    return before, after, rb, re


def human_motion(complete, proxies):
    """Maximum speed/yaw rate and summed human path over measured intervals."""
    max_speed = max_yaw_rate = 0.; human_path = 0.
    for previous,current in zip(complete,complete[1:]):
        dt = current['time']-previous['time']
        if dt <= 1e-10:
            continue
        for name in proxies.values():
            before = previous['poses'][name]; after = current['poses'][name]
            distance = math.hypot(after['x']-before['x'],after['y']-before['y']); human_path += distance
            max_speed = max(max_speed,distance/dt)
            max_yaw_rate = max(max_yaw_rate,abs(math.atan2(math.sin(after['yaw']-before['yaw']),math.cos(after['yaw']-before['yaw'])))/dt)
    return max_speed, max_yaw_rate, human_path


def lidar_measurements(scans, complete, pose_times, boxes, proxies):
    """Match rays only when a measured human is nearer than every static box."""
    lidar_errors = []; lidar_ids = set()
    for scan in scans:
        i = bisect.bisect_left(pose_times,scan['time'])
        indices = range(max(0,i-1),min(len(complete),i+2))
        i = min(indices,key=lambda i:abs(pose_times[i]-scan['time']))
        if abs(pose_times[i]-scan['time']) > .025:
            continue
        s = complete[i]; r = s['poses']['mecanum_robot']
        angles = scan['angle_min']+np.arange(len(scan['ranges']))*scan['angle_increment']+r['yaw']
        dx = np.cos(angles); dy = np.sin(angles); static = np.full(len(angles),np.inf)
        for b in boxes:
            near = np.full(len(angles),-np.inf); far = np.full(len(angles),np.inf)
            for origin,direction,center,size in [(r['x'],dx,b['x'],b['sx']),(r['y'],dy,b['y'],b['sy'])]:
                parallel = np.abs(direction)<1e-12; low = center-size/2; high = center+size/2
                t1 = np.divide(low-origin,direction,out=np.zeros_like(direction),where=~parallel)
                t2 = np.divide(high-origin,direction,out=np.zeros_like(direction),where=~parallel)
                near = np.maximum(near,np.where(parallel,-np.inf if low<=origin<=high else np.inf,np.minimum(t1,t2)))
                far = np.minimum(far,np.where(parallel,np.inf if low<=origin<=high else -np.inf,np.maximum(t1,t2)))
            static = np.minimum(static,np.where((far>=np.maximum(near,0))&(near>=0),near,np.inf))
        dynamic = np.full(len(angles),np.inf); labels = np.zeros(len(angles),dtype=int)
        for hid,name in proxies.items():
            h = s['poses'][name]; ox,oy = r['x']-h['x'],r['y']-h['y']; dot = ox*dx+oy*dy
            disc = dot*dot-(ox*ox+oy*oy-.36**2)
            hit = -dot-np.sqrt(np.maximum(disc,0)); hit = np.where((disc>=0)&(hit>0),hit,np.inf)
            closer = hit<dynamic; labels = np.where(closer,hid,labels); dynamic = np.minimum(dynamic,hit)
        selected = (dynamic>max(.14,scan['range_min']))&(dynamic<min(3.4,scan['range_max']))&(static>dynamic+.03)
        measured = np.array([np.nan if v is None else v for v in scan['ranges']])
        error = np.abs(measured-dynamic); error = np.where(np.isfinite(error),error,10.)
        lidar_errors.extend(error[selected].tolist())
        lidar_ids.update(int(v) for v in labels[selected&(error<.15)])
    return lidar_errors, lidar_ids
