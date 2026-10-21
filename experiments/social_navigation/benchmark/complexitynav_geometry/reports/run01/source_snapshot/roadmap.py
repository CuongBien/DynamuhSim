"""Visibility roadmap for disc centers among the42 physical factory AABBs."""
import heapq
import math
from shapely.geometry import Point, LineString, box
from shapely.ops import unary_union

class Roadmap:
    def __init__(self, layout, radius=.3):
        self.radius=radius;self.margin=.002;self.cache={}
        self.boxes=layout['obstacles']
        self.solid=unary_union([box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2) for b in self.boxes])
        # Roadmap corner samples sit4cm outside disc-inflated boxes. Exact
        # segment clearance, rather than grid occupancy, decides visibility.
        nodes=[];pad=radius+.04
        for b in self.boxes:
            for x in [b['x']-b['sx']/2-pad,b['x']+b['sx']/2+pad]:
                for y in [b['y']-b['sy']/2-pad,b['y']+b['sy']/2+pad]:
                    if self.valid((x,y)):nodes.append((x,y))
        self.nodes=sorted(set(nodes));self.edges=[[] for _ in self.nodes]
        for i,a in enumerate(self.nodes):
            for j in range(i):
                b=self.nodes[j]
                if self.visible(a,b):
                    length=math.dist(a,b);self.edges[i].append((j,length));self.edges[j].append((i,length))

    def valid(self,p):
        # Inner faces of actual perimeter walls; this is not a sampling-window fence.
        return (-29.85+self.radius+self.margin-1e-9<=p[0]<=29.85-self.radius-self.margin+1e-9 and
                -19.85+self.radius+self.margin-1e-9<=p[1]<=19.85-self.radius-self.margin+1e-9 and
                Point(p).distance(self.solid)>=self.radius+self.margin-1e-9)

    def visible(self,a,b):
        return LineString([a,b]).distance(self.solid)>=self.radius+self.margin-1e-9

    def goal_distances(self,goal):
        goal=tuple(goal)
        if goal in self.cache:return self.cache[goal]
        if not self.valid(goal):raise ValueError('Goal disc intersects factory geometry')
        dist=[math.inf]*len(self.nodes);queue=[]
        for i,p in enumerate(self.nodes):
            if self.visible(p,goal):dist[i]=math.dist(p,goal);heapq.heappush(queue,(dist[i],i))
        while queue:
            cost,i=heapq.heappop(queue)
            if cost>dist[i]+1e-12:continue
            for j,length in self.edges[i]:
                proposal=cost+length
                if proposal<dist[j]:dist[j]=proposal;heapq.heappush(queue,(proposal,j))
        self.cache[goal]=dist;return dist

    def waypoint(self,start,goal):
        start=tuple(start);goal=tuple(goal)
        if not self.valid(start):raise ValueError('Human start disc intersects factory geometry')
        if not self.valid(goal):raise ValueError('Goal disc intersects factory geometry')
        if self.visible(start,goal):return goal,math.dist(start,goal)
        dist=self.goal_distances(goal);choices=[]
        for i,p in enumerate(self.nodes):
            if math.isfinite(dist[i]) and self.visible(start,p):choices.append((math.dist(start,p)+dist[i],i))
        if not choices:raise RuntimeError('No disc-clear roadmap route to native goal')
        length,i=min(choices);return self.nodes[i],length
