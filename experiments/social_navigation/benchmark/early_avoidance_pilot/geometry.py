"""Offline group-space measurements; not supplied to Nav2."""
import math

def hull(points):
    points=sorted(set(points))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    if len(points)<=2:return points
    lower=[];upper=[]
    for p in points:
        while len(lower)>=2 and cross(lower[-2],lower[-1],p)<=0:lower.pop()
        lower.append(p)
    for p in reversed(points):
        while len(upper)>=2 and cross(upper[-2],upper[-1],p)<=0:upper.pop()
        upper.append(p)
    return lower[:-1]+upper[:-1]

def distance_to_hull(point,polygon):
    if len(polygon)<3:return float('inf')
    cross=[];distance=float('inf')
    for a,b in zip(polygon,polygon[1:]+polygon[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1];px,py=point[0]-a[0],point[1]-a[1]
        cross.append(dx*py-dy*px);t=max(0.,min(1.,(px*dx+py*dy)/(dx*dx+dy*dy)))
        distance=min(distance,math.hypot(px-t*dx,py-t*dy))
    if min(cross)>=-1e-10 or max(cross)<=1e-10:return 0.
    return distance
