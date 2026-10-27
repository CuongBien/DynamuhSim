"""New static fixture only. Original robot/warehouse and tracker are read-only."""
import copy,hashlib,importlib.util,json,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2]
spec=importlib.util.spec_from_file_location('visibility_height_fixture',BENCH/'camera_height_ab/fixture.py');height=importlib.util.module_from_spec(spec);spec.loader.exec_module(height)
def create(runtime):
 snapshot=json.loads((HERE/'snapshot.json').read_text());height.create_fixture(runtime,snapshot)
 tree=ET.parse(runtime/'warehouse.sdf');world=tree.getroot().find('world');robot=world.find("model[@name='mecanum_robot']")
 # Static fixture has no controller or dynamics; preserve the literal source namespace too.
 ET.register_namespace('gz','urn:gz')
 for link in robot.findall('link'):
  for sensor in list(link.findall('sensor')):
   if sensor.get('name')=='height_low':template=copy.deepcopy(sensor)
   link.remove(sensor)
 link=robot.find("link[@name='camera_link']")
 for mode in json.loads((HERE/'protocol.json').read_text())['modes']:
  sensor=copy.deepcopy(template);sensor.set('name','visibility_'+mode);sensor.find('topic').text='/visibility/'+mode+'/depth';sensor.find('update_rate').text='1'
  sensor.find('camera/camera_info_topic').text='/visibility/'+mode+'/camera_info';sensor.find('gz_frame_id').text='visibility_optical_frame';link.append(sensor)
 for item in snapshot['clutter']:
  model=ET.SubElement(world,'model',name=item['name']);ET.SubElement(model,'static').text='true';ET.SubElement(model,'pose').text=' '.join(map(str,item['pose']));body=ET.SubElement(model,'link',name='body')
  for tag in ['visual','collision']:
   geo=ET.SubElement(ET.SubElement(body,tag,name=tag),'geometry');ET.SubElement(ET.SubElement(geo,'box'),'size').text=' '.join(map(str,item['box']))
 ET.SubElement(world,'plugin',filename=str(HERE/'generated/build/libvisibility_filter.so'),name='visibility_experiment::Filter')
 ET.indent(tree);tree.write(runtime/'warehouse.sdf',encoding='unicode')
 (runtime/'fixture_snapshot.json').write_text(json.dumps(snapshot,indent=2)+'\n')
 return runtime/'warehouse.sdf'
