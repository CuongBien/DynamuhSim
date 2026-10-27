#!/usr/bin/env python3
"""Declare fixtures/inputs and verify their geometry before simulator execution."""
import hashlib,json,math
from pathlib import Path
import yaml
from shapely.geometry import Point,LineString,box
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent
layout=json.loads((SOCIAL/'warehouse/layout.json').read_text());polys=[box(b['x']-b['sx']/2,b['y']-b['sy']/2,b['x']+b['sx']/2,b['y']+b['sy']/2) for b in layout['obstacles']]
fixtures=[]
for scene in ['opposed','crossing','blocked_narrow']:
    humans=[]
    if scene=='opposed':points=[(x,y,x+14,y) for x in [-25.,-23.,-21.] for y in [-.6,.6]]
    elif scene=='crossing':points=[(x,y,x,-10. if y>0 else 10.) for x in [-26.,-24.,-22.] for y in [-1.8,1.8]]
    else:points=[(8.,-.42,25.,-.42),(9.4,.42,26.,.42),(10.8,-.42,27.5,-.42),(21.2,.42,5.,.42),(22.6,-.42,4.,-.42),(24.,.42,3.,.42)]
    for i,(x,y,gx,gy) in enumerate(points):humans.append(dict(id=i+1,x=x,y=y,gx=gx,gy=gy,radius=.36,yaw=math.atan2(gy-y,gx-x)))
    for h in humans:
        assert min(Point(h['x'],h['y']).distance(poly)-.36 for poly in polys)>.002
        assert min(Point(h['gx'],h['gy']).distance(poly)-.36 for poly in polys)>.002
        assert math.hypot(h['gx']-h['x'],h['gy']-h['y'])>10.
    assert min(math.hypot(a['x']-b['x'],a['y']-b['y'])-.72 for i,a in enumerate(humans) for b in humans[i+1:])>.002
    for condition in ['far','near']:
        trace=[]
        for i in range(201):
            t=i*.05;x,y=28.,18.;vx=vy=0.
            if condition=='near':
                if scene=='opposed':x,y=-17.-.5*min(t,8.),0.;vx=-.5 if t<8 else 0.
                elif scene=='crossing':x,y=-28.+.75*min(t,8.),0.;vx=.75 if t<8 else 0.
                else:x,y=16.,0.
            trace.append(dict(time=t,x=x,y=y,vx=vx,vy=vy,yaw=math.atan2(vy,vx) if vx or vy else 0.))
        for a,b in zip(trace,trace[1:]):
            assert abs(b['x']-a['x']-.05*a['vx'])<1e-10 and abs(b['y']-a['y']-.05*a['vy'])<1e-10
            assert min(LineString([(a['x'],a['y']),(b['x'],b['y'])]).distance(poly)-.25 for poly in polys)>.002
        assert min(math.hypot(h['x']-trace[0]['x'],h['y']-trace[0]['y'])-.61 for h in humans)>.002
        item=dict(name=scene+'_'+condition,scene=scene,condition=condition,seconds=10.,seed=42,humans=humans,robot_trace=trace)
        target=HERE/'inputs'/(item['name']+'.json');assert not target.exists();target.write_text(json.dumps(item,indent=2)+'\n');fixtures.append(item['name'])
cfg=yaml.safe_load((SOCIAL/'benchmark/socnavgym_warehouse_lifecycle/config.yaml').read_text())
cfg['env'].update(min_map_x=60,max_map_x=60,min_map_y=40,max_map_y=40,crowd_formation_probability=0.,crowd_dispersal_probability=0.)
(HERE/'inputs/socnavgym.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
(HERE/'inputs/layout.json').write_bytes((SOCIAL/'warehouse/layout.json').read_bytes())
models=dict(soc_native_orca=dict(dt=.1,runtime='venv',behavior='native individual motion,42-wall factory wrapper; no groups'),soc_guarded_orca=dict(dt=.1,runtime='venv',behavior='native intention plus global disc ORCA/swept guard; no groups'),crowd_orca_plus=dict(dt=.1,runtime='crowd',behavior='native ORCA+ act/constrain/step, external fixed fixture'),crowd_sfm=dict(dt=.1,runtime='crowd',behavior='native SFM act/constrain/step, external fixed fixture'),complex_native_mixed=dict(dt=.25,runtime='complex',behavior='native3ORCA+3SFM; interior obstacles not used'),complex_geometry_mixed=dict(dt=.25,runtime='complex',behavior='3ORCA+3SFM intention + roadmap/ORCA projection; radius-normalized'))
files=list((HERE/'inputs').glob('*'));digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
protocol=dict(stage='OFFLINE_MATCHED_INPUT_RESPONSE',fixtures=fixtures,models=models,human_radius=.36,robot_radius=.25,population=6,seed=42,seconds=10.,input_grid_s=.05,
    geometry_boxes=42,formation_disabled_for_comparison=True,goals='Fixed far endpoints; no arrival expected within10s. Group capabilities evaluated separately from saved native/live evidence.',
    fairness='Same initial discs/targets/geometry/continuous robot input, native worker dt retained. Native intended speed differs; raw travel is not a policy ranking. Compare near-vs-far within each model.',
    failure_rule='Keep every outcome and valid prefix. Negative clearance is a measured behavior result, not a harness exclusion. Runtime errors remain errors, not successful safe stops.',
    repeated_prefix_steps=20,input_sha256={str(p.relative_to(HERE)):digest(p) for p in files})
(HERE/'inputs/protocol.json').write_text(json.dumps(protocol,indent=2)+'\n');print('PASS:6 fixtures,all initial discs/goals and robot sweeps valid against42 boxes; protocol locked')
