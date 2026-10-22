"""New fixed scene; same native LiDAR/depth, frozen renderer and calibration."""
import importlib.util,json,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];BASE=BENCH/'depth_visibility_pilot'
spec=importlib.util.spec_from_file_location('previous_lidar_fixture',BENCH/'lidar_person_pilot/fixture.py');previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
def create(runtime,condition):
 assert condition=='full';previous.create(runtime,condition);tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');snapshot=json.loads((HERE/'snapshot.json').read_text());ET.register_namespace('gz','urn:gz');freezer=world.find("plugin[@name='height_experiment::FreezeActors']")
 for h in snapshot['humans']:
  i=h['id'];pose=f"{h['x']} {h['y']} 1 0 0 {h['yaw']}";actor=world.find(f"actor[@name='common_{i}_visual']");proxy=world.find(f"model[@name='common_{i}_proxy']");proxy.find('pose').text=f"{h['x']} {h['y']} .85 0 0 {h['yaw']}"
  for p in actor.findall('.//waypoint/pose'):p.text=pose
  for item in freezer.findall('frozen_actor'):
   if item.findtext('name')==actor.get('name'):item.find('pose').text=pose
 for model in list(world.findall('model')):
  if model.get('name','').startswith('visibility_'):world.remove(model)
 for item in snapshot['clutter']:
  m=ET.SubElement(world,'model',name=item['name']);ET.SubElement(m,'static').text='true';ET.SubElement(m,'pose').text=' '.join(map(str,item['pose']));body=ET.SubElement(m,'link',name='body')
  for tag in ['visual','collision']:
   geo=ET.SubElement(ET.SubElement(body,tag,name=tag),'geometry')
   if 'box' in item:ET.SubElement(ET.SubElement(geo,'box'),'size').text=' '.join(map(str,item['box']))
   else:
    cy=ET.SubElement(geo,'cylinder');ET.SubElement(cy,'radius').text=str(item['cylinder'][0]);ET.SubElement(cy,'length').text=str(item['cylinder'][1])
 ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode');(runtime/'snapshot.json').write_text(json.dumps(snapshot,indent=2)+'\n');(runtime/'fixture_snapshot.json').write_text(json.dumps(snapshot,indent=2)+'\n');return runtime/'warehouse.sdf'
