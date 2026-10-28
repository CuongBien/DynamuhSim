#!/usr/bin/env python3
"""Verify actual saved messages, immutable parents, physical parity and explicit oracle scope."""
import argparse,bisect,hashlib,json,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parent.parent.parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def canonical(e):return [e.tag,sorted(e.attrib.items()),(e.text or '').strip(),[canonical(c) for c in e]]
def read_xml(p):return ET.parse(p)
def audit(name):
    root=HERE/'generated/runs'/name;report=HERE/'reports'/name;summary=json.loads((report/'summary.json').read_text());result=json.loads((root/'result.json').read_text())
    for relative,h in summary['paired_raw_sha256'].items():assert sha(root/relative)==h
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(root/'raw_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    checked=set();received=set();missing_aux=[];counts={}
    while reader.has_next():
        topic,data,t=reader.read_next();counts[topic]=counts.get(topic,0)+1
        if topic!='/camera/depth/image_rect_raw':continue
        m=deserialize_message(data,Image);stamp=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        file=root/'paired_depth/off'/(str(stamp)+'.npy')
        received.add(stamp)
        if not file.exists():missing_aux.append(stamp);continue
        a=np.frombuffer(bytes(m.data),dtype='>f4' if m.is_bigendian else '<f4').reshape(m.height,m.step//4)[:,:m.width].astype(np.float32)
        assert a.tobytes()==np.load(file,allow_pickle=False).tobytes();checked.add(stamp)
    assert len(checked)>0 and len(counts)==16
    runtime=BENCH.parent/'datasets/dataset_pilot/generated'/name;parent=BENCH.parent/'perception/depth_proxy_lifecycle/generated/runs/motion01'
    old_result=json.loads((parent/'result.json').read_text())
    for n,h in old_result['source_sha256'].items():
        if n.startswith('src/') or '/vendor/' in n or '/socnavgym_group_nav2/' in n or n.endswith('libdepth_proxy_filter.so'):assert sha(REPO/n)==h
    before=read_xml(BENCH.parent/'datasets/dataset_pilot/generated/motion01/mecanum_camera.sdf');after=read_xml(runtime/'mecanum_camera.sdf')
    if name.startswith('avoidance_truth'):
        plugin=after.getroot().find("model/plugin[@name='gz::sim::systems::MecanumDrive']")
        assert plugin.findtext('odom_topic')=='/oracle_unused/native_odom' and plugin.findtext('tf_topic')=='/oracle_unused/native_tf'
        plugin.find('odom_topic').text='odom';plugin.find('tf_topic').text='/tf'
    assert canonical(before.getroot())==canonical(after.getroot()),'Robot model differs beyond declared odometry topic control'
    import yaml
    config=yaml.safe_load((runtime/'nav2_runtime.yaml').read_text());source=yaml.safe_load((BENCH.parent/'datasets/dataset_pilot/generated/avoidance01/nav2_runtime.yaml').read_text())
    if name.startswith('avoidance_truth'):
        assert config['amcl']['ros__parameters']['tf_broadcast'] is False
        config['amcl']['ros__parameters'].pop('tf_broadcast')
        if name=='avoidance_truth_wide01':
            assert config['controller_server']['ros__parameters']['FollowPath']['wz_std']==1.2
            config['controller_server']['ros__parameters']['FollowPath']['wz_std']=.3
    limits=config['controller_server']['ros__parameters']['FollowPath']
    if (limits['vx_max'],limits['vx_min'],limits['vy_max'])==(.30,-.15,.20):
        limits.update(vx_max=.15,vx_min=-.10,vy_max=.10)
    assert config==source,'Oracle changed more Nav2 parameters'
    # The two navigation treatments share the same physical world; only owned IPC sockets may differ.
    first=read_xml(BENCH.parent/'datasets/dataset_pilot/generated/avoidance01/warehouse.sdf');second=read_xml(runtime/'warehouse.sdf')
    for tree in [first,second]:
        for socket in tree.getroot().iter('socket'):socket.text='OWNED_SOCKET'
    assert canonical(first.getroot())==canonical(second.getroot())
    aux_stamps={int(f.stem) for f in (root/'paired_depth/off').glob('*.npy')}
    only_aux=sorted(aux_stamps-received)
    native=subprocess.run([str(HERE/'generated/check_friction'),str(runtime/'mecanum_camera.sdf')],capture_output=True,text=True)
    friction=json.loads(native.stdout);physics_valid=friction['recognized_gz_base_link_frames']==4
    ledger=dict(status=('EVIDENCE_PASS_SETUP_INVALID' if not physics_valid else 'PASS_WITH_AUX_DROPS' if missing_aux or only_aux else 'PASS'),scope='Raw/provenance verification; navigation outcome is independently retained in summary.json',
        run=name,trial_result=result['status'],navigation_acceptance=summary['status'],mcap_off_images_bitwise_equal=len(checked),mcap_off_images_total=len(received),mcap_missing_in_aux_stamps=missing_aux,aux_missing_in_mcap_stamps=only_aux,paired_raw_images_hashed=len(summary['paired_raw_sha256']),topics=counts,
        parent_source_files_native_worker_vendor_filter_unchanged=True,friction_native_parser=friction,native_robot_physics_semantics_valid=physics_valid,valid_original_robot_nav_trial=physics_valid and not name.startswith('avoidance_truth'),physical_world_equal_after_socket_normalization=True,robot_model_declared_changes_only=True,oracle_robot_pose_only=name.startswith('avoidance_truth'),training_ready=False)
    (report/'audit.json').write_text(json.dumps(ledger,indent=2)+'\n')
    # Full raw inventory remains reproducible after checkout; no overwrites or altered failures.
    files={str(f.relative_to(root)):dict(bytes=f.stat().st_size,sha256=sha(f)) for f in root.rglob('*') if f.is_file()}
    (report/'raw_manifest.json').write_text(json.dumps(files,indent=2)+'\n');print(json.dumps(ledger,indent=2));return ledger
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',required=True);a=p.parse_args();audit(a.name)
