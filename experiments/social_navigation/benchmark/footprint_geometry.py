"""Signed distance between the rotated Nav2 footprint and an axis-aligned box."""
import math

def footprint_box(robot,box,hx=.12,hy=.10):
    c,s=math.cos(robot['yaw']),math.sin(robot['yaw'])
    a=[(robot['x']+c*x-s*y,robot['y']+s*x+c*y) for x,y in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]
    x,y,sx,sy=box['x'],box['y'],box['sx']/2,box['sy']/2
    b=[(x-sx,y-sy),(x+sx,y-sy),(x+sx,y+sy),(x-sx,y+sy)]
    overlaps=[]
    for nx,ny in [(1.,0.),(0.,1.),(c,s),(-s,c)]:
        pa=[px*nx+py*ny for px,py in a];pb=[px*nx+py*ny for px,py in b]
        overlaps.append(min(max(pa)-min(pb),max(pb)-min(pa)))
    if min(overlaps)>=0:return -min(overlaps)
    def point_edge(p,u,v):
        dx,dy=v[0]-u[0],v[1]-u[1];t=max(0.,min(1.,((p[0]-u[0])*dx+(p[1]-u[1])*dy)/(dx*dx+dy*dy)))
        return math.hypot(p[0]-u[0]-t*dx,p[1]-u[1]-t*dy)
    return min(point_edge(p,poly[i],poly[(i+1)%4]) for points,poly in [(a,b),(b,a)] for p in points for i in range(4))
