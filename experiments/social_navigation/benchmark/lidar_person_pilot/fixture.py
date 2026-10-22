"""Copy-only wrapper: previous static visibility fixture plus original native LiDAR."""
import copy,importlib.util,json,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];BASE=BENCH/'depth_visibility_pilot'
spec=importlib.util.spec_from_file_location('person_visibility_fixture',BASE/'fixture.py');base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
def create(runtime,condition='full'):
 assert condition in ['full','null'];base.create(runtime);tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');ET.register_namespace('gz','urn:gz')
 robot=world.find("model[@name='mecanum_robot']");text=(REPO/'src/custom_corridor/models/mecanum_robot/model.sdf').read_text().replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">');source=ET.fromstring(text).find("model/link[@name='base_scan']/sensor")
 native=copy.deepcopy(source);native.find('topic').text='/person/scan';robot.find("link[@name='base_scan']").append(native)
 if condition=='null':
  for actor in list(world.findall('actor')):world.remove(actor)
  for model in list(world.findall('model')):
   if model.get('name','').startswith('common_') and model.get('name','').endswith('_proxy'):world.remove(model)
 ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode');(runtime/'condition.json').write_text(json.dumps(dict(condition=condition,actors=len(world.findall('actor')),lidar='Original definition; topic only renamed',camera='Original low depth; fifteen LABEL interventions retained'),indent=2)+'\n')
 return runtime/'warehouse.sdf'
