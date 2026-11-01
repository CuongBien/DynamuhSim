"""Convex per-human velocity projection; fixed measured robot, no SciPy dependency."""
import math,time
import numpy as np
from geometry import clearance,point_box

def _project(starts,deltas,radii,robot,robot_delta,robot_radius,boxes,dt,max_speed,fixed,margin=.04,previous_velocities=None,acceleration_limits=None):
    started=time.monotonic();p=np.asarray(starts,dtype=float);r=np.asarray(radii,dtype=float);n=len(p);size=2*n
    robot=np.asarray(robot,dtype=float);robot_delta=np.asarray(robot_delta,dtype=float)
    initial=clearance(starts,[(0.,0.)]*n,radii,robot,(0.,0.),robot_radius,boxes)
    if min(initial.values())<.002-1e-8:raise ValueError('Projection starts without2mm clearance; terminal stop required')
    rows=[];bounds=[]
    def add(i,normal,bound,j=None):
        a=np.zeros(size);a[2*i:2*i+2]=normal
        if j is not None:a[2*j:2*j+2]=-normal
        rows.append(a);bounds.append(bound)
    for i in range(n):
        rel=p[i]-robot;distance=float(np.linalg.norm(rel));normal=rel/distance
        add(i,normal,(r[i]+robot_radius+margin-distance+normal@robot_delta)/dt)
        for j in range(i):
            rel=p[i]-p[j];distance=float(np.linalg.norm(rel));add(i,rel/distance,(r[i]+r[j]+margin-distance)/dt,j)
        for box in boxes:
            if point_box(p[i],box)-r[i]>margin+max_speed*dt+1e-10:continue
            q=np.clip(p[i],[box['x']-box['sx']/2,box['y']-box['sy']/2],[box['x']+box['sx']/2,box['y']+box['sy']/2]);rel=p[i]-q;distance=float(np.linalg.norm(rel))
            if distance<1e-12:raise ValueError('Human center inside AABB')
            add(i,rel/distance,(r[i]+margin-distance)/dt)
    A=np.asarray(rows);b=np.asarray(bounds);norm2=np.sum(A*A,axis=1);limits=np.array([0. if i in fixed else max_speed for i in range(n)])
    support=np.sum(np.linalg.norm(A.reshape(len(rows),n,2),axis=2)*limits[None,:],axis=1)
    if np.any(b>support+1e-9):raise RuntimeError('Separation target exceeds available bounded speed')
    desired=np.asarray(deltas,dtype=float).reshape(-1)/dt;x=desired.copy();cycles=0
    prior=None if previous_velocities is None else np.asarray(previous_velocities,dtype=float).reshape(n,2)
    budgets=None if prior is None else np.asarray(acceleration_limits,dtype=float)*dt
    corrections=np.zeros((len(rows)+n+(n if prior is not None else 0),size))
    def residual(v):
        motion=0. if prior is None else float(np.max(np.linalg.norm(v.reshape(n,2)-prior,axis=1)-budgets))
        return max(float(np.max(b-A@v)),float(np.max(np.linalg.norm(v.reshape(n,2),axis=1)-limits)),motion,0.)
    if residual(x)>1e-9:
        for cycles in range(1,3001):
            previous=x.copy()
            for index,(a,bound,nn) in enumerate(zip(A,b,norm2)):
                y=x+corrections[index];x=y+max(0.,(bound-a@y)/nn)*a;corrections[index]=y-x
            for i,limit in enumerate(limits):
                index=len(rows)+i;y=x+corrections[index];x=y.copy();v=x[2*i:2*i+2];speed=float(np.linalg.norm(v))
                if speed>limit:x[2*i:2*i+2]=v*(limit/speed)
                corrections[index]=y-x
            if prior is not None:
                for i in range(n):
                    index=len(rows)+n+i;y=x+corrections[index];x=y.copy()
                    delta=x[2*i:2*i+2]-prior[i];length=float(np.linalg.norm(delta))
                    if length>budgets[i]:x[2*i:2*i+2]=prior[i]+delta*(budgets[i]/length)
                    corrections[index]=y-x
            if residual(x)<1e-8 and float(np.max(np.abs(x-previous)))<1e-8:break
        else:raise RuntimeError('Per-human convex projection did not converge; terminal stop required')
    displacement=x.reshape(n,2)*dt;gaps=clearance(starts,displacement.tolist(),radii,robot,robot_delta,robot_radius,boxes)
    if min(gaps.values())<.002-1e-8 or residual(x)>1e-7:raise RuntimeError('Projected proposal failed independent swept/bounded validation')
    return displacement.tolist(),dict(method='per_human_support_planes_dykstra',cycles=cycles,margin_m=margin,predicted_clearance=gaps,max_constraint_residual=residual(x),velocity_change_norm=float(np.linalg.norm(x-desired)),compute_ms=1000*(time.monotonic()-started),robot_displacement=robot_delta.tolist())

def project(starts,deltas,radii,robot,robot_delta,robot_radius,boxes,dt,max_speed,fixed,
            *,previous_velocities=None,acceleration_limits=None):
    """Intersect speed, swept safety and optional interval delta-v constraints.

    Safety wins only after all existing margins are tried with reachability.
    The returned audit explicitly marks any emergency motion-limit relaxation.
    Legacy callers without prior velocity retain the original projection.
    """
    n=len(starts)
    if not math.isfinite(dt) or dt<=0:raise ValueError('Positive finite dt required')
    if (previous_velocities is None)!=(acceleration_limits is None):raise ValueError('Prior velocity and acceleration limits must be supplied together')
    if previous_velocities is not None:
        prior=np.asarray(previous_velocities,dtype=float);limits=np.asarray(acceleration_limits,dtype=float)
        if prior.shape!=(n,2) or limits.shape!=(n,) or not np.all(np.isfinite(prior)) or not np.all(np.isfinite(limits)) or np.any(limits<=0):
            raise ValueError('Invalid previous executed velocities or acceleration limits')
    attempts=[]
    for constrained in ([True,False] if previous_velocities is not None else [False]):
        for margin in [.04,.02,.002]:
            try:
                displacement,audit=_project(starts,deltas,radii,robot,robot_delta,robot_radius,boxes,dt,max_speed,fixed,margin,
                                           previous_velocities if constrained else None,acceleration_limits if constrained else None)
                audit.update(target_margin_m=.04,attempts=attempts+[dict(margin=margin,motion_constraints=constrained,status='ACCEPTED')])
                if previous_velocities is not None:
                    measured=np.linalg.norm(np.asarray(displacement)/dt-prior,axis=1)/dt
                    audit.update(interval_acceleration_mps2=measured.tolist(),acceleration_limits_mps2=limits.tolist(),
                                 motion_constraints_satisfied=bool(np.all(measured<=limits+1e-6)),
                                 motion_limit_relaxed=not constrained,
                                 limiting_condition=None if constrained else 'Safety/reachability intersection rejected; safety-only fallback')
                return displacement,audit
            except RuntimeError as error:attempts.append(dict(margin=margin,motion_constraints=constrained,status='REJECTED',reason=str(error)))
    raise RuntimeError('No validated bounded per-human proposal; terminal stop required: '+str(attempts))
