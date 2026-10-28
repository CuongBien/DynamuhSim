"""Independent GEOS + finite relative-motion closest approach, no projector import."""
import numpy as np,shapely

def gaps(before,after,robot_before,robot_after,boxes):
    a=np.asarray(before);b=np.asarray(after);ra=np.asarray(robot_before);rb=np.asarray(robot_after);delta=b-a;pair=1e9
    def relative(p,d,r):
        den=np.sum(d*d,axis=-1);t=np.clip(np.divide(-np.sum(p*d,axis=-1),den,out=np.zeros_like(den),where=den>1e-25),0,1);return float(np.min(np.linalg.norm(p+t[...,None]*d,axis=-1)-r))
    for i in range(a.shape[1]):
        for j in range(i):pair=min(pair,relative(a[:,i]-a[:,j],delta[:,i]-delta[:,j],.72))
    robot=relative(a-ra[:,None,:],delta-(rb-ra)[:,None,:],.61)
    polygons=np.array([shapely.box(v['x']-v['sx']/2,v['y']-v['sy']/2,v['x']+v['sx']/2,v['y']+v['sy']/2) for v in boxes],dtype=object)
    segments=shapely.linestrings(np.stack((a,b),axis=2).reshape(-1,2,2));static=float(np.min(shapely.distance(segments[:,None],polygons[None,:]))-.36)
    return dict(pair=pair,robot=robot,static=static)
