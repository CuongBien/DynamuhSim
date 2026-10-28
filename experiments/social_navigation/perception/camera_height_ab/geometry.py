"""Analytic world geometry checks for the depth-height experiment."""
import math
import numpy as np

def optical_transform(z):
    return np.array([[0,0,1,.12],[-1,0,0,0],[0,-1,0,z],[0,0,0,1]],dtype=float)

def world_rays(k,shape,robot,camera_z):
    v,u=np.mgrid[:shape[0],:shape[1]]
    optical=np.stack(((u-k[2])/k[0],(v-k[5])/k[4],np.ones(shape)),axis=-1)
    c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    rz=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    origin=np.array([robot['x'],robot['y'],.04])+rz@np.array([.12,0,camera_z])
    directions=optical@optical_transform(camera_z)[:3,:3].T@rz.T
    return origin,directions

def floor_depth(origin,directions):
    dz=directions[...,2]
    with np.errstate(divide='ignore',invalid='ignore'):
        depth=-origin[2]/dz
    return np.where((dz<0)&(depth>0),depth,np.inf)

def cylinder_depth(origin,directions,humans,radius=.36,height=1.7):
    best=np.full(directions.shape[:-1],np.inf)
    a=np.sum(directions[...,:2]**2,axis=-1)
    for h in humans:
        delta=origin[:2]-[h['x'],h['y']]
        b=2*np.sum(directions[...,:2]*delta,axis=-1)
        c=float(np.dot(delta,delta)-radius**2)
        discriminant=b*b-4*a*c
        root=np.sqrt(np.maximum(discriminant,0))
        for sign in [-1,1]:
            t=(-b+sign*root)/(2*a)
            z=origin[2]+t*directions[...,2]
            valid=(discriminant>=0)&(t>0)&(z>=0)&(z<=height)
            best=np.minimum(best,np.where(valid,t,np.inf))
        for cap in [0.,height]:
            with np.errstate(divide='ignore',invalid='ignore'):
                t=(cap-origin[2])/directions[...,2]
                radial=origin[:2]+t[...,None]*directions[...,:2]-[h['x'],h['y']]
                good=(t>0)&(np.sum(radial**2,axis=-1)<=radius**2)
            best=np.minimum(best,np.where(good,t,np.inf))
    return best

def proxy_interior(robot,humans):
    x,y=np.meshgrid((np.arange(64)+.5)*.1,(np.arange(64)+.5)*.1-3.2,indexing='ij')
    c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    wx=robot['x']+x*c-y*s;wy=robot['y']+x*s+y*c
    inside=np.zeros((64,64),dtype=bool)
    for h in humans:
        inside|=(wx-h['x'])**2+(wy-h['y'])**2<(.36-math.sqrt(2)*.05)**2
    return inside
