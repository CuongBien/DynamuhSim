"""Derive a display/TF URDF from the authoritative native SDF, without editing either input."""
import hashlib,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
PROFILE='native_sdf_urdf_v1'

def read_model(path):
 text=Path(path).read_text().lstrip()
 if text.startswith('<?xml'):text=text.split('?>',1)[1]
 root=ET.fromstring('<context xmlns:gz="urn:gz">'+text+'</context>')
 model=root.find('sdf/model')
 if model is None:raise ValueError('Expected one native SDF model')
 return model

def transform(text):
 values=[float(x) for x in (text or '0 0 0 0 0 0').split()]
 if len(values)!=6 or not all(math.isfinite(x) for x in values):raise ValueError('Expected finite XYZ/RPY')
 result=np.eye(4);result[:3,:3]=Rotation.from_euler('xyz',values[3:]).as_matrix();result[:3,3]=values[:3];return result

def origin(parent,matrix):
 xyz=matrix[:3,3];rpy=Rotation.from_matrix(matrix[:3,:3]).as_euler('xyz')
 fmt=lambda a:' '.join(format(0. if abs(v)<1e-12 else float(v),'.17g') for v in a)
 ET.SubElement(parent,'origin',xyz=fmt(xyz),rpy=fmt(rpy))
 return dict(xyz=[float(x) for x in xyz],rpy=[float(x) for x in rpy])

def geometry(parent,source):
 target=ET.SubElement(parent,'geometry');children=list(source)
 if len(children)!=1:raise ValueError('Expected one primitive')
 kind=children[0].tag;node=children[0]
 if kind=='box':ET.SubElement(target,'box',size=node.findtext('size'))
 elif kind=='sphere':ET.SubElement(target,'sphere',radius=node.findtext('radius'))
 elif kind=='cylinder':ET.SubElement(target,'cylinder',radius=node.findtext('radius'),length=node.findtext('length'))
 else:raise ValueError('Unsupported robot geometry: '+kind)

def aligned_urdf(sdf_path,template_path,destination):
 destination=Path(destination)
 if destination.exists():raise FileExistsError(destination)
 model=read_model(sdf_path);links={x.get('name'):x for x in model.findall('link')}
 root=ET.parse(template_path).getroot();cache={'__model__':np.eye(4)}
 def pose(name,stack=()):
  if name in cache:return cache[name]
  if name in stack or name not in links:raise ValueError('Unknown/cyclic native link frame: '+name)
  node=links[name].find('pose');relative=node.get('relative_to','__model__') if node is not None else '__model__'
  cache[name]=pose(relative,(*stack,name))@transform(node.text if node is not None else None);return cache[name]
 for link in root.findall('link'):
  name=link.get('name')
  if name not in links:raise ValueError('URDF link absent from native model: '+name)
  source=links[name]
  for old in list(link):
   if old.tag in ['visual','collision','inertial']:link.remove(old)
  for tag in ['visual','collision']:
   for i,node in enumerate(source.findall(tag)):
    new=ET.SubElement(link,tag,name=node.get('name',tag+str(i)));origin(new,transform(node.findtext('pose')));geometry(new,node.find('geometry'))
    if tag=='visual':
     color=node.findtext('material/diffuse') or '.2 .35 .65 1';material=ET.SubElement(new,'material',name='native_'+name+'_'+str(i));ET.SubElement(material,'color',rgba=color)
  inertial=source.find('inertial')
  if inertial is not None:
   new=ET.SubElement(link,'inertial');origin(new,transform(inertial.findtext('pose')));ET.SubElement(new,'mass',value=inertial.findtext('mass'));tensor=inertial.find('inertia');ET.SubElement(new,'inertia',**{key:tensor.findtext(key) for key in ['ixx','ixy','ixz','iyy','iyz','izz']})
 joints={}
 for joint in root.findall('joint'):
  parent=joint.find('parent').get('link');child=joint.find('child').get('link');matrix=np.linalg.inv(pose(parent))@pose(child)
  for old in list(joint.findall('origin')):joint.remove(old)
  value=origin(joint,matrix);joints[child]=dict(parent=parent,type=joint.get('type'),**value)
 ET.indent(root);ET.ElementTree(root).write(destination,encoding='unicode',xml_declaration=True)
 return dict(profile=PROFILE,authoritative_sdf_sha256=hashlib.sha256(Path(sdf_path).read_bytes()).hexdigest(),template_sha256=hashlib.sha256(Path(template_path).read_bytes()).hexdigest(),urdf_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),joints=joints,physics_sdf_changed=False)
