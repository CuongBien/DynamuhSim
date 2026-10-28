#!/usr/bin/env python3
"""Independent closed-raw roundtrip + label masks, bundle provenance and relocation checks."""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import String
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=HERE.parents[3]
sys.path.insert(0,str(REPO/'python'))
from provenance_checks import verify as verify_provenance
from dynamuhsim.workspace import source_path

def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def dump(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def audit(root,out,artifact_root=None):
    assert not out.exists();out.mkdir(parents=True)
    lineage=verify_provenance(root);dump(out/'provenance_timing_audit.json',lineage)
    from dynamuhsim.datasets.normalized_audit import audit as audit_normalized
    base=audit_normalized(root,root/'normalized',repository=REPO);dump(out/'legacy_raw_audit.json',base)
    provenance=json.loads((root/'provenance.json').read_text());manifest=json.loads((root/'bundle_manifest.json').read_text());checked=0
    for name,v in manifest['files'].items():
        f=(root/name).resolve();assert f.is_relative_to(root) and f.stat().st_size==v['bytes'] and sha(f)==v['sha256'];checked+=1
    for name,v in provenance['copied_files'].items():assert sha(root/name)==v['sha256']
    source=source_path(artifact_root or REPO,'experiments/social_navigation/benchmark/early_avoidance_pilot')/'generated/runs/avoidance_moderate01'
    for f in (source/'raw_bag').iterdir():
        if f.is_file():assert sha(f)==sha(root/'raw_bag'/f.name)
    assert sha(root/'config_snapshot/runtime/mecanum_camera.sdf')==provenance['final_artifact_sha256']['mecanum_camera.sdf']
    assert (root/'config_snapshot/runtime/mecanum_camera.sdf').read_text().count('gz:expressed_in="base_link"')==4
    x=np.load(root/'normalized/samples.npz',allow_pickle=False);y=np.load(root/'normalized/labels.npz',allow_pickle=False);m=np.load(root/'supervision_masks.npz',allow_pickle=False);n=len(x['anchor_ns'])
    assert m['trajectory_gt_available'].shape==(n,30,6) and m['trajectory_gt_available'].all()
    assert m['sensor_track_association_valid'].shape==(n,6) and not m['sensor_track_association_valid'].any()
    assert not m['m1_track_aligned_supervision_valid'].any() and not m['persona_valid'].any()
    assert not m['future_bev_target_valid'].any() and not m['episode_training_eligible'].any()
    assert m['current_depth_bev_available'].all() and not y['persona_valid'].any() and (y['persona_index']==-1).all()
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(root/'raw_bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    groups=[]
    while reader.has_next():
        topic,data,t=reader.read_next()
        if topic=='/common/ground_truth':groups.append(json.loads(deserialize_message(data,String).data))
    groups.sort(key=lambda r:r['time']);times=np.array([round(r['time']*1e9) for r in groups],dtype=np.int64);ids=y['human_ids'].tolist();modes={'moving':1,'walking':1,'standing':2,'none':0};expected_id=[];expected_mode=[]
    for row in groups:
        lookup={}
        for group in row['groups']:
            for human in group['members']:
                assert human not in lookup;lookup[human]=(group['id'],modes[group['type']])
        expected_id.append([lookup.get(ident,(-1,0))[0] for ident in ids]);expected_mode.append([lookup.get(ident,(-1,0))[1] for ident in ids])
    at=np.searchsorted(times,y['future_stamp_ns'],side='right')-1;assert (at>=0).all()
    np.testing.assert_array_equal(np.array(expected_id)[at],y['group_id_future']);np.testing.assert_array_equal(np.array(expected_mode)[at],y['group_mode_future'])
    # Check occupied endpoints from a separate explicit pinhole projection, without BEV helper imports.
    depth=np.load(root/'normalized/depth.npy',mmap_mode='r',allow_pickle=False);bev=np.load(root/'normalized/bev_observed.npz',allow_pickle=False);cal=json.loads((root/'normalized/calibration.json').read_text());k=cal['k'];transform=np.array(cal['optical_to_base_link']);endpoints=0
    for i in [0,len(bev['depth_index'])//2,len(bev['depth_index'])-1]:
        a=depth[bev['depth_index'][i]];v,u=np.mgrid[0:a.shape[0]:4,0:a.shape[1]:4];z=a[::4,::4];good=np.isfinite(z)&(z>=.28)&(z<=5);z=z[good]
        optical=np.stack(((u[good]-k[2])*z/k[0],(v[good]-k[5])*z/k[4],z),axis=1);points=optical@transform[:3,:3].T+transform[:3,3]
        inside=(points[:,0]>=0)&(points[:,0]<6.4)&(points[:,1]>=-3.2)&(points[:,1]<3.2)&(points[:,2]>=.03)&(points[:,2]<=1.8);cells=np.floor((points[inside,:2]+[0,3.2])/.1).astype(int)
        assert np.all(bev['occupancy'][i,cells[:,0],cells[:,1]]==1);endpoints+=len(cells)
    # Relocate all files with hardlinks (read-only smoke), so no second full data copy is needed.
    relocated=HERE/'generated/relocation'/out.name;assert not relocated.exists();shutil.copytree(root,relocated,copy_function=os.link)
    env=os.environ.copy();env.pop('PYTHONPATH',None);env.pop('AMENT_PREFIX_PATH',None)
    probe=subprocess.run([sys.executable,'-I',str(REPO/'tools/probe_portable_episode.py'),str(relocated)],capture_output=True,text=True,env=env,timeout=60)
    assert probe.returncode==0,probe.stderr;relocation=json.loads(probe.stdout);assert relocation['status']=='PASS'
    # Reading through the relocated bundle must not modify any original file.
    assert all(sha(root/name)==v['sha256'] for name,v in manifest['files'].items())
    result=dict(status='PASS',sample_count=n,independent_episodes=1,raw_depth_bitwise_equal=base['depth_frames_bitwise_equal'],
        future_human_max_error_m=base['human_future_max_abs_error_m'],future_actions_raw_exact=True,group_future_labels_raw_exact=True,
        gt_trajectory_available_mask_count=int(m['trajectory_gt_available'].sum()),sensor_association_valid_count=0,persona_valid_count=0,future_bev_target_valid_count=0,training_eligible_count=0,
        occupied_depth_endpoints_checked=endpoints,complete_bundle_files_verified=checked,bundle_manifest_sha256=sha(root/'bundle_manifest.json'),original_raw_unchanged=True,
        relocation=dict(method='Read-only hardlinked copy at a different root, isolated Python without ROS/Gazebo imports',probe=relocation),
        scope='Offline data integrity/portability QA only; not tracker/persona/future-BEV/split acceptance',training_ready=False)
    dump(out/'audit.json',result);print(json.dumps(result,indent=2));return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--report-dir',type=Path,required=True);p.add_argument('--artifact-root',type=Path);a=p.parse_args();audit(a.bundle.resolve(),a.report_dir.resolve(),a.artifact_root)
