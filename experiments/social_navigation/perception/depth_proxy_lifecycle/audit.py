#!/usr/bin/env python3
"""Audit raw MCAP against every OFF array, static controls, lifecycle outcomes and provenance."""
import hashlib,json,math,re,sys
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parent.parent.parent

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def dump(p,r):p.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')

def audit():
    raw=HERE/'generated/runs/motion01';report=HERE/'reports/motion01';summary=json.loads((report/'summary.json').read_text())
    for name,h in summary['raw_pair_sha256'].items():assert sha(raw/name)==h
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(raw/'raw_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    checked=set();counts={}
    while reader.has_next():
        topic,data,t=reader.read_next();counts[topic]=counts.get(topic,0)+1
        if topic!='/camera/depth/image_rect_raw':continue
        msg=deserialize_message(data,Image);stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        a=np.frombuffer(bytes(msg.data),dtype='>f4' if msg.is_bigendian else '<f4').reshape(msg.height,msg.step//4)[:,:msg.width].astype(np.float32)
        saved=np.load(raw/'paired_depth/off'/(str(stamp)+'.npy'),allow_pickle=False)
        assert a.tobytes()==saved.tobytes();checked.add(stamp)
    assert len(checked)==153==summary['paired_frames'] and len(counts)==16
    result=json.loads((raw/'result.json').read_text());assert result['status']=='ERROR' and result['faults'] and result['faults'][0]['world_pause_requested']
    assert summary['navigation_accepted'] is False and summary['training_ready'] is False
    assert summary['sensor_prefix_checks']=='PASS' and summary['renderer_shutdown_checks']=='PASS'
    matches=re.search(r'teardown pre=(\d+) post=(\d+)',(raw/'launch.log').read_text());assert matches and matches[1]==matches[2]=='153'
    static=HERE/'generated/runs/paused01/snapshot03';rmeta=[]
    for condition in ['baseline','masked','null_visuals']:
        r=json.loads((static/condition/'result.json').read_text());rmeta.append(r)
        assert r['gazebo_returncode']==0 and r['cleanup']['status']=='PASS' and not r['cleanup']['remaining']
        assert r['server_shutdown']['quiesce']['pause_acknowledged'] and r['server_shutdown']['quiesce']['simulation_time_stable']
    assert len({r['fixture']['collision']['sha256'] for r in rmeta})==1
    for level in ['low','high']:
        a=np.load(static/'baseline'/f'depth_{level}_on.npy');b=np.load(static/'masked'/f'depth_{level}_on.npy');c=np.load(static/'masked'/f'depth_{level}_off.npy');d=np.load(static/'null_visuals'/f'depth_{level}_null.npy')
        assert a.tobytes()==b.tobytes()==c.tobytes()==d.tobytes()
    a=np.load(static/'baseline/scan_geometry.npz')['ranges'];b=np.load(static/'masked/scan_geometry.npz')['ranges'];assert a.tobytes()==b.tobytes()
    # Only sensors/metadata differ in copied robot; native robot collision XML stays identical.
    import xml.etree.ElementTree as ET
    original=(REPO/'src/custom_corridor/models/mecanum_robot/model.sdf').read_text().replace('<sdf version="1.8">','<sdf version="1.8" xmlns:gz="urn:gz">')
    new=(BENCH.parent/'datasets/dataset_pilot/generated/motion01/mecanum_camera.sdf').read_text()
    def collisions(text):
        rows=[]
        for e in ET.fromstring(text).findall('model/link/collision'):
            for item in e.iter():item.text=(item.text or '').strip();item.tail=None
            rows.append(ET.tostring(e))
        return rows
    assert collisions(original)==collisions(new)
    ledger=[];sessions=set()
    for root in sorted((HERE/'generated/runs').iterdir()):
        candidates=list(root.glob('snapshot*/*/result.json'))+list(root.glob('result.json'))
        for file in candidates:
            r=json.loads(file.read_text());sessions.add(r['cleanup']['session'])
            ledger.append(dict(file=str(file.relative_to(HERE)),sha256=sha(file),data_status=r['status'],server_status=r['server_shutdown']['status'],gazebo_returncode=r.get('gazebo_returncode'),clean_exit=r.get('gazebo_clean_exit'),quiesce=r['server_shutdown'].get('quiesce'),cleanup=r['cleanup'],diagnostic_preload=r.get('shutdown_mode',{}).get('diagnostic_preload')))
    live=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:env=(proc/'environ').read_bytes()
        except OSError:continue
        if any(('DYNAMUHSIM_EXPERIMENT_SESSION='+s).encode() in env for s in sessions):live.append(int(proc.name))
    assert not live
    result=dict(status='PASS',scope='Evidence audit only; driver root cause unresolved and dynamic navigation incomplete',
        mcap_off_images_bitwise_equal=len(checked),raw_pairs_hashed=len(summary['raw_pair_sha256']),topics=counts,
        empty_front_static_control='ON/OFF/null depth bitwise equal at both camera heights; original/tagged geometry LiDAR bitwise equal',
        original_robot_collisions_equal=True,owned_processes_remaining=live,
        source_sha256={f.name:sha(f) for f in HERE.glob('*.py')},audit_sha256=sha(Path(__file__)))
    dump(HERE/'reports/audit.json',result);dump(HERE/'reports/attempts.json',ledger)
    print(json.dumps(result,indent=2));return result
if __name__=='__main__':audit()
