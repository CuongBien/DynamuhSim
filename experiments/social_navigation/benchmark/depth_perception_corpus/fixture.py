"""Copy-only diagnostic scene assembly, never edits original robot or policies."""
import importlib.util,json,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];BASE=BENCH/'depth_visibility_pilot'
spec=importlib.util.spec_from_file_location('corpus_lidar_fixture',BENCH/'lidar_person_pilot/fixture.py');previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
def get_scene(scene_id):return next(v for v in json.loads((HERE/'scenes_LABEL_ONLY.json').read_text()) if v['scene_id']==scene_id)
def create(runtime,scene_id):
 scene=get_scene(scene_id);previous.create(runtime,'full');tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');ET.register_namespace('gz','urn:gz');old=world.find("plugin[@name='height_experiment::FreezeActors']");world.remove(old)
 motion=ET.SubElement(world,'plugin',filename=str(HERE/'generated/build/libcorpus_motion.so'),name='perception_corpus::Motion');ET.SubElement(motion,'duration').text=str(scene['duration_s']);ET.SubElement(motion,'trace').text=str(runtime/'motion_LABEL_ONLY.jsonl')
 for h in scene['humans']:
  ident=h['id'];x,y=h['start'];pose=f"{x} {y} 1 0 0 {h['yaw']}";actor=world.find(f"actor[@name='common_{ident}_visual']");proxy=world.find(f"model[@name='common_{ident}_proxy']");proxy.find('pose').text=f"{x} {y} .85 0 0 {h['yaw']}"
  if scene['moving']:proxy.find('static').text='false'
  for item in actor.findall('.//waypoint/pose'):item.text=pose
  target=ET.SubElement(motion,'target');ET.SubElement(target,'id').text=str(ident);ET.SubElement(target,'start').text=f'{x} {y} 0';ET.SubElement(target,'end').text=' '.join(map(str,[*h['end'],0]));ET.SubElement(target,'yaw').text=str(h['yaw']);ET.SubElement(target,'phase').text=str(h['animation_phase_s']);ET.SubElement(target,'rate').text=str(h['animation_rate'])
 for model in list(world.findall('model')):
  if model.get('name','').startswith('visibility_'):world.remove(model)
 for item in scene['objects']:
  model=ET.SubElement(world,'model',name=item['name']);ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,item['pose']));body=ET.SubElement(model,'link',name='body')
  for kind in ['visual','collision']:
   geo=ET.SubElement(ET.SubElement(body,kind,name=kind),'geometry');shape=ET.SubElement(geo,item['shape'])
   if item['shape']=='box':ET.SubElement(shape,'size').text=' '.join(map(str,item['size']))
   else:
    ET.SubElement(shape,'radius').text=str(item['radius'])
    if item['shape']=='cylinder':ET.SubElement(shape,'length').text=str(item['length'])
 ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode');(runtime/'scene_LABEL_ONLY.json').write_text(json.dumps(scene,indent=2)+'\n');return runtime/'warehouse.sdf'
