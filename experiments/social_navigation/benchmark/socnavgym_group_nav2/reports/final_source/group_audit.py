"""Membership from native worker; motion, spacing, facing from actual Gazebo poses."""
import bisect,math
import numpy as np

def measure_groups(worker,complete,result,goal_sample):
    times=[r['time'] for r in complete];initial={g['id'] for g in result['ready']['groups']};segments=[];open_segments={};previous_keys=None;changes=[]
    for row in worker:
        t=row['state_time']
        if not row['active_phase'] or not result['goal_start_sim']<=t<=goal_sample+1e-8:continue
        j=bisect.bisect_left(times,t);idx=list(range(max(0,j-1),min(len(complete),j+2)))
        if not idx:continue
        j=min(idx,key=lambda k:abs(times[k]-t))
        if abs(times[j]-t)>.025:continue
        actual=complete[j]['poses'];keys=set()
        for g in row['groups']:
            ids=g['members'];assert len(ids)>=2 and len(ids)==len(set(ids)) and set(ids)<=set(range(1,7))
            key=(g['id'],g['type'],tuple(ids));keys.add(key);points=np.array([[actual[f'common_{i}_proxy']['x'],actual[f'common_{i}_proxy']['y']] for i in ids]);center=np.mean(points,axis=0);diameter=float(np.max(np.linalg.norm(points[:,None,:]-points[None,:,:],axis=2)))
            facing=[]
            for hid,xy in zip(ids,points):
                delta=center-xy
                if np.linalg.norm(delta)>1e-8:facing.append(math.cos(math.atan2(delta[1],delta[0])-actual[f'common_{hid}_proxy']['yaw']))
            if key not in open_segments:open_segments[key]=dict(id=g['id'],type=g['type'],members=ids,initial_object=g['id'] in initial,samples=[])
            open_segments[key]['samples'].append(dict(time=times[j],center=center.tolist(),positions=points.tolist(),diameter=diameter,facing=facing,robot_distance=float(np.linalg.norm(center-np.array([actual['mecanum_robot']['x'],actual['mecanum_robot']['y']])))))
        if previous_keys is not None and keys!=previous_keys:changes.append(dict(time=t,removed=[list(k[:2])+[list(k[2])] for k in sorted(previous_keys-keys)],added=[list(k[:2])+[list(k[2])] for k in sorted(keys-previous_keys)]))
        for key in set(open_segments)-keys:segments.append(open_segments.pop(key))
        previous_keys=keys
    segments+=list(open_segments.values());output=[]
    for segment in segments:
        samples=segment.pop('samples');centers=np.array([s['center'] for s in samples]);duration=samples[-1]['time']-samples[0]['time'];path=float(np.sum(np.linalg.norm(np.diff(centers,axis=0),axis=1)));facing=[v for s in samples for v in s['facing']];positions=np.array([s['positions'] for s in samples]);delta=np.array([s['time'] for s in samples]);mask=np.diff(delta)>1e-10;speeds=(np.linalg.norm(np.diff(positions,axis=0),axis=2)[mask]/np.diff(delta)[mask,None]).flatten() if len(samples)>1 else np.array([])
        output.append(dict(**segment,start=samples[0]['time'],end=samples[-1]['time'],duration_s=duration,actual_samples=len(samples),actual_centroid_path_m=path,actual_centroid_net_m=float(np.linalg.norm(centers[-1]-centers[0])),actual_member_peak_speed_mps=float(np.max(speeds)) if len(speeds) else None,max_pair_diameter_m=max(s['diameter'] for s in samples),inward_facing_fraction_30deg=sum(v>=math.cos(math.pi/6) for v in facing)/len(facing) if facing else None,robot_to_centroid_min_m=min(s['robot_distance'] for s in samples),moving_travel_coverage=segment['type']=='moving' and duration>=2. and path>=.5,standing_duration_coverage=segment['type']=='standing' and duration>=2.))
    return dict(segments=output,membership_or_type_changes=changes,moving_travel_covered=any(s['moving_travel_coverage'] for s in output),standing_duration_covered=any(s['standing_duration_coverage'] for s in output),new_group_objects=sorted({s['id'] for s in output if not s['initial_object']}),scope='Initial groups are fixtures, not spontaneous births. Native labels give membership; Gazebo actual poses give travel/spacing/facing. Duration coverage is not subjective naturalness, speech, or group-aware Nav2.')
