"""Deterministic crossing fixture; endpoints specify intent, never scripted motion."""
import copy,math,random

def crossing_scene(baseline,layout,seed):
    if seed not in (42,43):raise ValueError('Crossing fixture supports seeds 42/43')
    scene=copy.deepcopy(baseline);rng=random.Random(seed);humans=[];routes={}
    for index,base_x in enumerate([-25.5,-24.,-22.5]):
        hid=index+1;x=base_x+rng.uniform(-.1,.1)
        sign=(-1 if index%2==0 else 1)*(1 if seed==42 else -1)
        start_y=sign*2.1;goal_y=-start_y
        humans.append(dict(id=hid,x=x,y=start_y,yaw=math.copysign(math.pi/2,goal_y-start_y),speed=0.,radius=.36,vx=0.,vy=0.))
        routes[str(hid)]=[[x,goal_y],[x,start_y]]
    scene.update(condition='crossing',seed=seed,initial=humans,goal_layout='individual',human_routes=routes,
        scenario_version='crossing_v1',crossing_definition='three independent pedestrians patrol across y=0; no lane or timing constraints')
    scene.pop('group_route',None)
    # Screen intended straight segments and starting disks against the unchanged factory.
    minimum=math.inf
    for h in humans:
        for i in range(43):
            y=-2.1+i*.1
            for box in layout['obstacles']:
                qx,qy=abs(h['x']-box['x'])-box['sx']/2,abs(y-box['y'])-box['sy']/2
                gap=math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
                minimum=min(minimum,gap)
                if gap<=0:raise ValueError('Nominal crossing route intersects factory')
    for i,h in enumerate(humans):
        if math.hypot(h['x']+27.,h['y'])<=.36+.25:raise ValueError('Initial human overlaps robot')
        for j in humans[i+1:]:
            if math.hypot(h['x']-j['x'],h['y']-j['y'])<=.72:raise ValueError('Initial people overlap')
    scene['nominal_route_factory_clearance_m_min']=minimum
    return scene
