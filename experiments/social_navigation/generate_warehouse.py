#!/usr/bin/env python3
"""Generate warehouse SDF and occupancy map from the same collision geometry."""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw
import yaml

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent / 'warehouse'
OUT.mkdir(exist_ok=True)
SDF = ET.Element('sdf', version='1.9')
world = ET.SubElement(SDF, 'world', name='social_warehouse')
def el(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, **attrs)
    if text is not None:
        node.text = str(text)
    return node
for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                       ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu')]:
    el(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
sensors = el(world, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
el(sensors, 'render_engine', 'ogre2')
physics = el(world, 'physics', name='physics', type='ignored')
el(physics, 'max_step_size', .004)
el(physics, 'real_time_factor', 1)
scene = el(world, 'scene')
el(scene, 'ambient', '0.65 0.65 0.65 1')
el(scene, 'background', '0.78 0.83 0.90 1')
el(scene, 'shadows', 'false')
light = el(world, 'light', name='sun', type='directional')
el(light, 'pose', '0 0 12 0 0 0')
el(light, 'direction', '-0.3 0.2 -1')
el(light, 'diffuse', '0.9 0.9 0.9 1')
el(light, 'cast_shadows', 'false')
obstacles = []
def box(name, x, y, sx, sy, height, color, *, z=None, collision=True):
    model = el(world, 'model', name=name)
    el(model, 'static', 'true')
    el(model, 'pose', f'{x} {y} {height/2 if z is None else z} 0 0 0')
    link = el(model, 'link', name='link')
    if collision:
        collider = el(link, 'collision', name='collision')
        el(el(el(collider, 'geometry'), 'box'), 'size', f'{sx} {sy} {height}')
    visual = el(link, 'visual', name='visual')
    el(el(el(visual, 'geometry'), 'box'), 'size', f'{sx} {sy} {height}')
    material = el(visual, 'material')
    el(material, 'ambient', color)
    el(material, 'diffuse', color)
    if collision and name != 'floor':
        obstacles.append(dict(name=name, x=x, y=y, sx=sx, sy=sy, height=height))
box('floor', 0, 0, 62, 42, .1, '0.60 0.63 0.65 1', z=-.05)
for name, x, y, sx, sy in [('west',-30,0,.3,40.3),('east',30,0,.3,40.3),
                           ('north',0,20,60,.3),('south',0,-20,60,.3)]:
    box(f'wall_{name}',x,y,sx,sy,3,'0.77 0.79 0.82 1')
# Four rack rows on each side of a six-metre central cross aisle.
for i,x in enumerate([-20,-12,-4,4]):
    for side,y in [('north',8.5),('south',-8.5)]:
        box(f'rack_{side}_{i}',x,y,2,11,2.6,'0.18 0.33 0.48 1')
        # Bright shelf ledges are visual only; base collision stays simple and deterministic.
        for level,z in enumerate([.35,1.3,2.45]):
            box(f'ledge_{side}_{i}_{level}',x,y,2.12,11,.08,'0.95 0.55 0.12 1',z=z,collision=False)
# An 8m-long, 1.8m-wide bottleneck with two alternate routes around it.
for side,y in [('north',4.5),('south',-4.5)]:
    box(f'machine_{side}',16,y,8,7.2,1.8,'0.35 0.48 0.42 1')
# Structural posts give the existing 3.5m LiDAR returns along the main route.
# At y=0 the furthest point between posts is <3.5m from a post face.
for i,x in enumerate(list(range(-27,12,4))+[23,27]):
    for side,y in [('north',2.8),('south',-2.8)]:
        box(f'column_{side}_{i}',x,y,.4,.4,4,'0.83 0.75 0.27 1')
# Distinct asymmetric landmarks near loading and social areas.
for i,(x,y,sx,sy) in enumerate([(-26,16,3,2),(-25,-16,4,2),(25,-15,3,3),(15,16,4,2)]):
    box(f'loading_crate_{i}',x,y,sx,sy,1.2,'0.63 0.43 0.24 1')
# Floor paint: no collision, no map occupancy, visible from above.
for x in range(-27,28,3):
    box(f'lane_mark_{x+27}',x,0,1,.08,.002,'0.95 0.85 0.12 1',z=.002,collision=False)
box('conversation_zone',24,12,7,7,.002,'0.30 0.59 0.39 1',z=.001,collision=False)
ET.indent(SDF)
ET.ElementTree(SDF).write(OUT/'warehouse.sdf',encoding='utf-8',xml_declaration=True)
# Occupancy is derived from every physical obstacle, including the wall thickness.
resolution=.05
origin=(-31.,-21.)
width,height=1240,840
image=Image.new('L',(width,height),254)
draw=ImageDraw.Draw(image)
for o in obstacles:
    xmin,xmax=o['x']-o['sx']/2,o['x']+o['sx']/2
    ymin,ymax=o['y']-o['sy']/2,o['y']+o['sy']/2
    # Mark every cell whose interior intersects a collision box; no extra edge cell.
    left=math.floor((xmin-origin[0])/resolution+1e-7)
    right=math.ceil((xmax-origin[0])/resolution-1e-7)-1
    top=height-math.ceil((ymax-origin[1])/resolution-1e-7)
    bottom=height-math.floor((ymin-origin[1])/resolution+1e-7)-1
    draw.rectangle((left,top,right,bottom),fill=0)
image.save(OUT/'warehouse.pgm')
(OUT/'warehouse.yaml').write_text(yaml.safe_dump(dict(image='warehouse.pgm',mode='trinary',resolution=resolution,
    origin=[*origin,0.],negate=0,occupied_thresh=.65,free_thresh=.196)))
# Preview is separate from the map consumed by Nav2.
preview=image.convert('RGB')
p=ImageDraw.Draw(preview)
for label,x,y in [('START',-27,0),('GOAL',26,0),('GROUPS',24,12),('GATE 1.8m',16,0)]:
    px,py=int((x-origin[0])/resolution),height-1-int((y-origin[1])/resolution)
    p.ellipse((px-5,py-5,px+5,py+5),fill='red')
    p.text((px+8,py-15),label,fill='red')
preview.save(OUT/'layout.png')
params=yaml.safe_load((ROOT/'src/custom_corridor/config/nav2_mppi_mecanum.yaml').read_text())
params['amcl']['ros__parameters']['initial_pose'].update(x=-27.,y=0.,yaw=0.)
(OUT/'nav2_warehouse.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
(OUT/'layout.json').write_text(json.dumps(dict(size_m=[60,40],resolution=resolution,spawn=[-27,0,0],
    goal=[26,0,0],bottleneck_width_m=1.8,conversation_zone=[24,12,7,7],obstacles=obstacles),indent=2))
print(f'Generated {OUT}: {len(obstacles)} collision boxes, map {width}x{height}')

