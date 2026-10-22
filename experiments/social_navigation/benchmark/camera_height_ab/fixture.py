"""Frozen recorded XY/yaw, paired cameras; no baseline source edits."""
import copy
import difflib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
BENCH=HERE.parent
SOCIAL=BENCH.parent
REPO=SOCIAL.parent.parent
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'))
from run import make_world


def create_fixture(runtime,snapshot):
    humans=[dict(**h,radius=.36) for h in snapshot['humans']]
    make_world(runtime,'',humans,'walking')
    tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world')
    world.remove(world.find("plugin[@name='social_experiment::Adapter']"))
    for m in list(world.findall('model')):
        name=m.get('name','')
        if name=='socnavgym_observer':world.remove(m);continue
        if name.startswith('socnavgym_'):
            m.set('name',name.replace('socnavgym_','common_',1))
            ET.SubElement(m,'static').text='true'
    freezer=ET.SubElement(world,'plugin',filename=str(HERE/'generated/build/libfreeze_actors.so'),name='height_experiment::FreezeActors')
    for actor in world.findall('actor'):
        hid=int(actor.get('name').split('_')[1]);h=next(h for h in humans if h['id']==hid)
        actor.set('name',f'common_{hid}_visual')
        # Existing adapter semantics: actor root at zero, trajectory in world coordinates.
        actor.find('pose').text='0 0 0 0 0 0'
        script=actor.find('script');script.find('auto_start').text='false'
        for waypoint in script.findall('.//waypoint/pose'):
            waypoint.text=f"{h['x']} {h['y']} 1 0 0 {h['yaw']}"
        target=ET.SubElement(freezer,'frozen_actor')
        ET.SubElement(target,'name').text=actor.get('name')
        ET.SubElement(target,'pose').text=f"{h['x']} {h['y']} 1 0 0 {h['yaw']}"
    # Declare the source's unbound gz attribute namespace for XML parsing only.
    source=(REPO/'src/custom_corridor/models/mecanum_robot/model.sdf').read_text()
    source=source.replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">')
    ET.register_namespace('gz','urn:gz')
    robot=ET.fromstring(source).find('model')
    for plugin in list(robot.findall('plugin')):robot.remove(plugin)
    for link in robot.findall('link'):
        for sensor in list(link.findall('sensor')):
            if sensor.get('name')=='depth_camera':template=copy.deepcopy(sensor)
            link.remove(sensor)
    ET.SubElement(robot,'static').text='true'
    r=snapshot['robot'];robot.find('pose').text=f"{r['x']} {r['y']} .04 0 0 {r['yaw']}"
    link=robot.find("link[@name='camera_link']")
    for label,delta in [('low',0.),('high',.28)]:
        sensor=copy.deepcopy(template);sensor.set('name','height_'+label)
        ET.SubElement(sensor,'pose').text=f'0 0 {delta} 0 0 0'
        sensor.find('topic').text=f'/height/{label}/depth'
        sensor.find('gz_frame_id').text=f'camera_{label}_optical_frame'
        sensor.find('camera/camera_info_topic').text=f'/height/{label}/camera_info'
        link.append(sensor)
        # Per-variant URDF copies preserve and verify the camera joint transform.
        u=ET.parse(REPO/'src/custom_corridor/urdf/mecanum_robot.urdf')
        u.getroot().find("joint[@name='camera_joint']/origin").set('xyz',f'.12 0 {.08+delta}')
        ET.indent(u);u.write(runtime/f'robot_{label}.urdf',encoding='unicode')
    world.append(robot)
    world.find('physics/max_step_size').text='.0025'
    world.find('physics/real_time_factor').text='1'
    ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode')
    (runtime/'snapshot.json').write_text(json.dumps(snapshot,indent=2)+'\n')
    return runtime/'warehouse.sdf'
