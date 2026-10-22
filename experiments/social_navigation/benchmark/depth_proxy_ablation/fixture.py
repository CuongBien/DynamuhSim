"""Sensor-selective proxy visibility; copy-only wrappers around frozen height fixture."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent;REPO=SOCIAL.parent.parent
HEIGHT=BENCH/'camera_height_ab'
spec=importlib.util.spec_from_file_location('height_fixture_source',HEIGHT/'fixture.py')
height_fixture=importlib.util.module_from_spec(spec);spec.loader.exec_module(height_fixture)
PROXY_FLAG=0x00000004
ALL_MASK=0xffffffff
NO_PROXY_MASK=ALL_MASK^PROXY_FLAG

def labels(condition):
    if condition=='masked':return ['low_on','low_off','high_on','high_off']
    suffix='on' if condition=='baseline' else 'null'
    return ['low_'+suffix,'high_'+suffix]

def canonical(e):
    return [e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[canonical(c) for c in e]]

def collision_signature(world):
    rows=[]
    for model in world.findall('model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                rows.append([model.get('name'),model.findtext('pose'),model.findtext('static'),link.get('name'),link.findtext('pose'),canonical(collision)])
    raw=json.dumps(rows,sort_keys=True,separators=(',',':')).encode()
    return dict(count=len(rows),sha256=hashlib.sha256(raw).hexdigest())

def create_fixture(runtime,snapshot,condition):
    assert condition in ['baseline','masked','null_visuals']
    height_fixture.create_fixture(runtime,snapshot)
    tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world')
    signature_before=collision_signature(world)
    for model in world.findall('model'):
        if not (model.get('name','').startswith('common_') and model.get('name','').endswith('_proxy')):continue
        for link in model.findall('link'):
            for visual in list(link.findall('visual')):
                if condition=='masked':ET.SubElement(visual,'visibility_flags').text=str(PROXY_FLAG)
                elif condition=='null_visuals':link.remove(visual)
    if condition=='null_visuals':
        for actor in list(world.findall('actor')):world.remove(actor)
    if condition=='masked':
        ET.SubElement(world,'plugin',filename=str(HERE/'generated/build_jazzy/libdepth_proxy_filter.so'),name='depth_proxy_experiment::Filter')
    robot=world.find("model[@name='mecanum_robot']")
    link=robot.find("link[@name='camera_link']")
    old={s.get('name').replace('height_',''):copy.deepcopy(s) for s in link.findall('sensor')}
    for sensor in list(link.findall('sensor')):link.remove(sensor)
    for label in labels(condition):
        level=label.split('_')[0];sensor=copy.deepcopy(old[level]);sensor.set('name','proxy_'+label)
        sensor.find('topic').text=f'/ablation/{condition}/{label}/depth'
        sensor.find('camera/camera_info_topic').text=f'/ablation/{condition}/{label}/camera_info'
        mask=NO_PROXY_MASK if label.endswith('_off') else ALL_MASK
        ET.SubElement(sensor.find('camera'),'visibility_mask').text=str(mask)
        link.append(sensor)
    # Original source LiDAR definition retained except topic; deterministic clone is an extra diagnostic.
    text=(REPO/'src/custom_corridor/models/mecanum_robot/model.sdf').read_text().replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">')
    original=ET.fromstring(text).find("model/link[@name='base_scan']/sensor")
    scanlink=robot.find("link[@name='base_scan']")
    for name in ['native','geometry']:
        sensor=copy.deepcopy(original);sensor.set('name','ablation_'+name)
        sensor.find('topic').text=f'/ablation/{condition}/scan_{name}'
        if name=='geometry':
            lidar=sensor.find('lidar');noise=lidar.find('noise')
            if noise is not None:lidar.remove(noise)
            diagnostic_noise=ET.SubElement(lidar,'noise')
            ET.SubElement(diagnostic_noise,'type').text='none'
            ET.SubElement(diagnostic_noise,'stddev').text='0'
        scanlink.append(sensor)
    assert collision_signature(world)==signature_before, 'Visibility operation changed collision definitions'
    ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode')
    meta=dict(condition=condition,collision=signature_before,
              proxy_flags=PROXY_FLAG if condition=='masked' else ALL_MASK,
              camera_masks={label:NO_PROXY_MASK if label.endswith('_off') else ALL_MASK for label in labels(condition)},
              sensor_levels={label:label.split('_')[0] for label in labels(condition)})
    (runtime/'fixture.json').write_text(json.dumps(meta,indent=2)+'\n')
    return runtime/'warehouse.sdf',meta
