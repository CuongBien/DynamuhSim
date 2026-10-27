"""Independent finite-segment geometry for circle proxies and factory AABBs."""
import math

def point_box(p,b):
    return math.hypot(max(abs(p[0]-b['x'])-b['sx']/2,0.),max(abs(p[1]-b['y'])-b['sy']/2,0.))

def segment_box(p,d,b):
    lo=0.;hi=1.
    for coordinate,delta,center,size in zip(p,d,(b['x'],b['y']),(b['sx'],b['sy'])):
        a=center-size/2;z=center+size/2
        if abs(delta)<1e-15:
            if not a<=coordinate<=z:lo=2.;break
        else:
            first,second=sorted(((a-coordinate)/delta,(z-coordinate)/delta));lo=max(lo,first);hi=min(hi,second)
    if lo<=hi:return 0.
    end=(p[0]+d[0],p[1]+d[1]);best=min(point_box(p,b),point_box(end,b));norm=d[0]**2+d[1]**2
    for x in [b['x']-b['sx']/2,b['x']+b['sx']/2]:
        for y in [b['y']-b['sy']/2,b['y']+b['sy']/2]:
            t=0. if norm<1e-30 else max(0.,min(1.,((x-p[0])*d[0]+(y-p[1])*d[1])/norm))
            best=min(best,math.hypot(x-p[0]-t*d[0],y-p[1]-t*d[1]))
    return best

def relative_gap(p,d,r):
    norm=d[0]**2+d[1]**2;t=0. if norm<1e-30 else max(0.,min(1.,-(p[0]*d[0]+p[1]*d[1])/norm))
    return math.hypot(p[0]+t*d[0],p[1]+t*d[1])-r

def clearance(starts,deltas,radii,robot,robot_delta,robot_radius,boxes):
    pair=robot_gap=static=1e9
    for i,(p,d,r) in enumerate(zip(starts,deltas,radii)):
        robot_gap=min(robot_gap,relative_gap((p[0]-robot[0],p[1]-robot[1]),(d[0]-robot_delta[0],d[1]-robot_delta[1]),r+robot_radius))
        static=min(static,min(segment_box(p,d,b)-r for b in boxes))
        for j in range(i):pair=min(pair,relative_gap((p[0]-starts[j][0],p[1]-starts[j][1]),(d[0]-deltas[j][0],d[1]-deltas[j][1]),r+radii[j]))
    return dict(pair=pair,robot=robot_gap,static=static)

def admissible_fraction(starts,deltas,radii,robot,robot_delta,robot_radius,boxes):
    # Robot displacement remains fixed for the entire tick. Shortening human
    # motion is NOT assumed to improve safety when a robot is approaching.
    for index in range(33):
        fraction=1-index/32;scaled=[(fraction*d[0],fraction*d[1]) for d in deltas]
        gaps=clearance(starts,scaled,radii,robot,robot_delta,robot_radius,boxes)
        if min(gaps.values())>=.002-1e-8:return fraction,gaps
    raise RuntimeError('No admissible bounded human proposal for predicted robot interval; robot motion is not clipped or ignored')
