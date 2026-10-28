"""One XML assembly for static, LiDAR, surface and scripted sensor fixtures."""
import copy,importlib.util,json
from pathlib import Path
from dynamuhsim.workspace import stage_directory
import xml.etree.ElementTree as ET


def _clutter(world, items):
 for item in items:
  shape=item.get('shape','box' if 'box' in item else 'cylinder')
  model=ET.SubElement(world,'model',name=item['name']);ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,item['pose']));body=ET.SubElement(model,'link',name='body')
  for kind in ['visual','collision']:
   geo=ET.SubElement(ET.SubElement(body,kind,name=kind),'geometry');geometry=ET.SubElement(geo,shape)
   if shape=='box':ET.SubElement(geometry,'size').text=' '.join(map(str,item['size'] if 'shape' in item else item['box']))
   else:
    ET.SubElement(geometry,'radius').text=str(item['radius'] if 'shape' in item else item['cylinder'][0])
    if shape=='cylinder':ET.SubElement(geometry,'length').text=str(item['length'] if 'shape' in item else item['cylinder'][1])


def _place_human(world, ident, x, y, yaw):
 pose=f"{x} {y} 1 0 0 {yaw}";actor=world.find(f"actor[@name='common_{ident}_visual']");proxy=world.find(f"model[@name='common_{ident}_proxy']")
 proxy.find('pose').text=f"{x} {y} .85 0 0 {yaw}"
 for point in actor.findall('.//waypoint/pose'):point.text=pose
 return actor,proxy,pose


def create_fixture(runtime, *, benchmark, repo, lidar=False, condition='full', snapshot=None, motion=None, motion_library=None):
 """Preserve native sensor bytes, actor frames, object order and metadata sidecars."""
 assert condition in ['full','null']
 repo=Path(repo);benchmark=stage_directory(repo,'depth_visibility_pilot').parent;base=benchmark/'depth_visibility_pilot'
 initial=json.loads((base/'snapshot.json').read_text());modes=json.loads((base/'protocol.json').read_text())['modes']
 spec=importlib.util.spec_from_file_location('sensor_height_fixture',benchmark/'camera_height_ab/fixture.py');height=importlib.util.module_from_spec(spec);spec.loader.exec_module(height)
 height.create_fixture(runtime,initial);tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');robot=world.find("model[@name='mecanum_robot']");ET.register_namespace('gz','urn:gz')
 for link in robot.findall('link'):
  for sensor in list(link.findall('sensor')):
   if sensor.get('name')=='height_low':template=copy.deepcopy(sensor)
   link.remove(sensor)
 link=robot.find("link[@name='camera_link']")
 for mode in modes:
  sensor=copy.deepcopy(template);sensor.set('name','visibility_'+mode);sensor.find('topic').text='/visibility/'+mode+'/depth';sensor.find('update_rate').text='1';sensor.find('camera/camera_info_topic').text='/visibility/'+mode+'/camera_info';sensor.find('gz_frame_id').text='visibility_optical_frame';link.append(sensor)
 _clutter(world,initial['clutter']);ET.SubElement(world,'plugin',filename=str(base/'generated/build/libvisibility_filter.so'),name='visibility_experiment::Filter')
 (runtime/'fixture_snapshot.json').write_text(json.dumps(initial,indent=2)+'\n')
 if lidar:
  source=(repo/'src/custom_corridor/models/mecanum_robot/model.sdf').read_text().replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">');native=copy.deepcopy(ET.fromstring(source).find("model/link[@name='base_scan']/sensor"));native.find('topic').text='/person/scan';robot.find("link[@name='base_scan']").append(native)
  if condition=='null':
   for actor in list(world.findall('actor')):world.remove(actor)
   for model in list(world.findall('model')):
    if model.get('name','').startswith('common_') and model.get('name','').endswith('_proxy'):world.remove(model)
  (runtime/'condition.json').write_text(json.dumps(dict(condition=condition,actors=len(world.findall('actor')),lidar='Original definition; topic only renamed',camera='Original low depth; fifteen LABEL interventions retained'),indent=2)+'\n')
 if snapshot is not None:
  freezer=world.find("plugin[@name='height_experiment::FreezeActors']")
  for human in snapshot['humans']:
   actor,proxy,pose=_place_human(world,human['id'],human['x'],human['y'],human['yaw'])
   for target in freezer.findall('frozen_actor'):
    if target.findtext('name')==actor.get('name'):target.find('pose').text=pose
  for name in ['snapshot.json','fixture_snapshot.json']:(runtime/name).write_text(json.dumps(snapshot,indent=2)+'\n')
 if motion is not None:
  world.remove(world.find("plugin[@name='height_experiment::FreezeActors']"));plugin=ET.SubElement(world,'plugin',filename=str(motion_library),name='perception_corpus::Motion');ET.SubElement(plugin,'duration').text=str(motion['duration_s']);ET.SubElement(plugin,'trace').text=str(runtime/'motion_LABEL_ONLY.jsonl')
  for human in motion['humans']:
   ident=human['id'];x,y=human['start'];actor,proxy,pose=_place_human(world,ident,x,y,human['yaw'])
   if motion['moving']:proxy.find('static').text='false'
   target=ET.SubElement(plugin,'target');ET.SubElement(target,'id').text=str(ident);ET.SubElement(target,'start').text=f'{x} {y} 0';ET.SubElement(target,'end').text=' '.join(map(str,[*human['end'],0]));ET.SubElement(target,'yaw').text=str(human['yaw']);ET.SubElement(target,'phase').text=str(human['animation_phase_s']);ET.SubElement(target,'rate').text=str(human['animation_rate'])
  (runtime/'scene_LABEL_ONLY.json').write_text(json.dumps(motion,indent=2)+'\n')
 if snapshot is not None or motion is not None:
  for model in list(world.findall('model')):
   if model.get('name','').startswith('visibility_'):world.remove(model)
  _clutter(world,snapshot['clutter'] if snapshot is not None else motion['objects'])
 ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode');return runtime/'warehouse.sdf'
