"""Runtime-only redirect of native odometry outputs for the explicit oracle control."""
import hashlib,xml.etree.ElementTree as ET
from motion_prepare import prepare_camera as parent_prepare,add_filter

def prepare_camera(runtime,repo):
    result=parent_prepare(runtime,repo);path=runtime/'mecanum_camera.sdf';tree=ET.parse(path)
    plugin=tree.getroot().find("model/plugin[@name='gz::sim::systems::MecanumDrive']");assert plugin is not None
    plugin.find('odom_topic').text='/oracle_unused/native_odom';plugin.find('tf_topic').text='/oracle_unused/native_tf'
    ET.indent(tree);tree.write(path,encoding='unicode')
    result['oracle_odometry']=dict(mode='Simulation-only measured robot pose; do not deploy',native_drive_unchanged=True,original_odometry_topics_diverted=True,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return result
