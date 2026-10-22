"""Preserve Gazebo's literal extension prefix when serializing the experimental SDF."""
import hashlib,xml.etree.ElementTree as ET
from motion_prepare import prepare_camera as parent_prepare,add_filter

def preserve_friction_prefix(path):
    # gz-physics7 DART checks the literal attribute name, not expanded XML namespace URI.
    ET.register_namespace('gz','urn:gz')
    tree=ET.parse(path);frictions=tree.getroot().findall('model/link/collision/surface/friction/ode/fdir1')
    assert len(frictions)==4
    assert all(e.get('{urn:gz}expressed_in')=='base_link' for e in frictions)
    ET.indent(tree);tree.write(path,encoding='unicode')
    text=path.read_text();assert text.count('gz:expressed_in="base_link"')==4 and 'ns0:expressed_in' not in text

def prepare_camera(runtime,repo):
    result=parent_prepare(runtime,repo);path=runtime/'mecanum_camera.sdf';preserve_friction_prefix(path)
    result['friction_prefix_repair']=dict(literal_gz_expressed_in_count=4,scope='Runtime serialization only; restores original Gazebo friction frame',sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return result
