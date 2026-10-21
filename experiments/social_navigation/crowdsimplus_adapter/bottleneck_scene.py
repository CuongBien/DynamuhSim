"""Counterflow through the existing factory throat, without prescribed trajectories."""
import copy,math,random

def bottleneck_scene(baseline,layout,seed):
    if not isinstance(seed,int) or seed<0:raise ValueError('Bottleneck fixture requires a nonnegative integer seed')
    scene=copy.deepcopy(baseline);rng=random.Random(seed);humans=[];routes={}
    for index,x in enumerate([21.2,22.6,24.]):
        hid=index+1;y=[-.30,0.,.30][index]+rng.uniform(-.04,.04);x+=rng.uniform(-.06,.06)
        humans.append(dict(id=hid,x=x,y=y,yaw=math.pi,speed=0.,radius=.36,vx=0.,vy=0.))
        routes[str(hid)]=[[10.5-1.4*index,y],[x,y]]
    scene.update(condition='bottleneck',seed=seed,initial=humans,goal_layout='individual',human_routes=routes,
        scenario_version='factory_bottleneck_v1',robot_start=[8.,0.,0.],robot_goal=[24.5,0.,0.],
        passage=dict(x_min=12.,x_max=20.,half_width=.9,left_confirmation_x=11.5,right_confirmation_x=20.5),
        scenario_definition='three independent counterflow pedestrians, two private endpoints each; no timing/lane or yielding constraints')
    scene.pop('group_route',None)
    machines={o['name']:o for o in layout['obstacles'] if o['name'] in ['machine_north','machine_south']}
    assert len(machines)==2
    assert abs((machines['machine_north']['y']-machines['machine_north']['sy']/2)-.9)<1e-9
    assert abs((machines['machine_south']['y']+machines['machine_south']['sy']/2)+.9)<1e-9
    minimum=math.inf
    for h in humans:
        left,right=routes[str(h['id'])];steps=math.ceil((right[0]-left[0])/.1)
        for i in range(steps+1):
            x=left[0]+(right[0]-left[0])*i/steps;y=left[1]
            for box in layout['obstacles']:
                qx,qy=abs(x-box['x'])-box['sx']/2,abs(y-box['y'])-box['sy']/2
                gap=math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36
                minimum=min(minimum,gap)
                if gap<=0:raise ValueError('Intended route intersects factory')
    for i,h in enumerate(humans):
        if math.hypot(h['x']-8.,h['y'])<=.61:raise ValueError('Initial robot overlap')
        for j in humans[i+1:]:
            if math.hypot(h['x']-j['x'],h['y']-j['y'])<=.72:raise ValueError('Initial human overlap')
    scene['nominal_route_factory_clearance_m_min']=minimum
    return scene
