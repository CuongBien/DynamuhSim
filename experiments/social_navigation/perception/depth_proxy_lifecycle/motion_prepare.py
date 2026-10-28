"""Runtime copies only: retain native camera pose and add same-pose ON control."""
import copy,hashlib,importlib.util,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;OLD=BENCH.parent/'datasets/dataset_pilot'
spec=importlib.util.spec_from_file_location('dataset_camera_baseline',OLD/'prepare.py');source=importlib.util.module_from_spec(spec);spec.loader.exec_module(source)

def prepare_camera(runtime,repo):
    result=source.prepare_camera(runtime,repo);path=runtime/'mecanum_camera.sdf'
    text=path.read_text().replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">')
    tree=ET.ElementTree(ET.fromstring(text));link=tree.getroot().find("model/link[@name='camera_link']")
    sensor=link.find('sensor');on=copy.deepcopy(sensor);on.set('name','proxy_low_on')
    on.find('topic').text='/proxy_check/on/depth';on.find('camera/camera_info_topic').text='/proxy_check/on/camera_info'
    sensor.set('name','proxy_low_off');link.append(on)
    ET.indent(tree);tree.write(path,encoding='unicode')
    result['depth_proxy_motion']=dict(default_camera='OFF experiment only',on_control='/proxy_check/on/depth',camera_pose_unchanged=True,extra_sensor_only=True,sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return result

def add_filter(world):
    library=BENCH/'depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so'
    assert library.is_file()
    ET.SubElement(world,'plugin',filename=str(library),name='depth_proxy_experiment::Filter')
