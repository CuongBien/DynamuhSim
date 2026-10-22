#!/usr/bin/env python3
"""Byte-copy one accepted raw episode plus self-contained tools/provenance, never source edits."""
import argparse,hashlib,json,shutil,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parent.parent.parent
SOURCE=BENCH/'early_avoidance_pilot';NAME='avoidance_moderate01'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def dump(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def copy_checked(source,destination):
    destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,destination)
    assert sha(source)==sha(destination);return dict(bytes=source.stat().st_size,sha256=sha(source))
def prepare(output):
    result=json.loads((SOURCE/'generated/runs'/NAME/'result.json').read_text());summary=json.loads((SOURCE/'reports'/NAME/'summary.json').read_text());audit=json.loads((SOURCE/'reports'/NAME/'audit.json').read_text())
    assert summary['status']=='PASS' and result['actual_goal_reached'] and not result['faults']
    assert audit['native_robot_physics_semantics_valid'] and audit['valid_original_robot_nav_trial'] and not audit['oracle_robot_pose_only']
    assert result['gazebo_clean_exit'] and result['recording']['returncode']==0 and not result['cleanup']['remaining']
    raw=SOURCE/'generated/runs'/NAME;runtime=REPO/result['runtime'];assert sha(runtime/'mecanum_camera.sdf')==result['camera_runtime_corrections']['friction_prefix_repair']['sha256']
    assert sha(runtime/'warehouse.sdf')==result['runtime_world_sha256'] and sha(runtime/'nav2_runtime.yaml')==result['runtime_nav_params_sha256']
    output.mkdir(parents=True,exist_ok=False);copied={}
    def keep(source,relative):copied[relative]=copy_checked(source,output/relative)
    for f in (raw/'raw_bag').iterdir():
        if f.is_file():keep(f,'raw_bag/'+f.name)
    for name in ['result.json','events.jsonl']:keep(raw/name,name)
    for name in ['warehouse.sdf','nav2_runtime.yaml','mecanum_camera.sdf','mecanum_camera.launch.py','mecanum_camera.launch.py.diff','mecanum_camera.sdf.diff','ready.json','rendering.json','ogre2_session.log','worker_trajectory.jsonl','quiesce_shutdown.json']:
        keep(runtime/name,'config_snapshot/runtime/'+name)
    for name in ['live_parameters.json','config_changes.json']:keep(raw/name,'config_snapshot/'+name)
    for name in ['summary.json','audit.json','setup_status.json']:
        if (SOURCE/'reports'/NAME/name).exists():keep(SOURCE/'reports'/NAME/name,'config_snapshot/acceptance/'+name)
    for f in (SOURCE/'reports/drivers'/NAME).iterdir():
        if f.is_file():keep(f,'config_snapshot/driver/'+f.name)
    for name in ['namespace_prepare.py','moderate_config.py','config_runtime.py','check_friction.cc']:
        keep(SOURCE/name,'config_snapshot/helpers/'+name)
    for name in ['worker.py','reactive_motion.py','standing_projection.py','fixture.py','config.yaml']:
        keep(BENCH/'socnavgym_group_nav2'/name,'config_snapshot/human_worker/'+name)
    for name in ['depth_proxy_filter.cc','CMakeLists.txt']:
        keep(BENCH/'depth_proxy_ablation'/name,'config_snapshot/depth_filter/'+name)
    keep(BENCH/'depth_proxy_ablation/generated/build_jazzy/libdepth_proxy_filter.so','config_snapshot/depth_filter/libdepth_proxy_filter.so')
    for name in ['warehouse.sdf','warehouse.yaml','warehouse.pgm']:keep(BENCH.parent/'warehouse'/name,'config_snapshot/world_source/'+name)
    keep(REPO/'src/custom_corridor/models/mecanum_robot/model.sdf','config_snapshot/robot_source.sdf')
    keep(REPO/'src/custom_corridor/urdf/mecanum_robot.urdf','config_snapshot/robot_source.urdf')
    for name in ['build.py','dataset_core.py','contract.json','audit.py','test_contract.py']:
        keep(BENCH/'dataset_pilot'/name,'tools/legacy/'+name)
    provenance=dict(schema='social_provenance/1',episode_id='socnavgym_standing42_earlyavoid_01',source_episode=NAME,backend='SocNavGym',response_variant=result['response_variant'],seed=result['seed'],
        captured_head=result['head'],evidence_commit='adb42ad454d425612a9eaff45a61b38ca14dab71',friction_repair_commit='33427f42109d1fdfbd355f1727a7bc02dd27c539',
        capture_was_uncommitted_experiment=True,capture_sources=result['source_sha256'],
        robot_pose_feedback='native odometry + AMCL; no oracle',camera_mode='Native low camera, proxy OFF in depth; native LiDAR conservative proxy ON',
        native_physics_friction_frames=4,final_artifact_sha256={name:sha(runtime/name) for name in ['warehouse.sdf','nav2_runtime.yaml','mecanum_camera.sdf','mecanum_camera.launch.py']},
        intermediate_camera_metadata=result['camera_runtime_corrections'],intermediate_hash_note='Parent metadata includes pre-camera-clone hashes; final_artifact_sha256 is authoritative for packaged artifacts',
        nav2_profile='moderate circle .25+padding .01, inflation .8/scaling3, speeds .30/-.15/.20mps',
        data_units='SI, int64 simulation ns',split='pilot_only',human_persona_valid=False,track_association_valid=False,future_bev_target_defined=False,training_ready=False,
        raw_camera_authority='MCAP all1013 frames; auxiliary NPY missed5warmup frames; exporter reads MCAP directly',
        behavior_limits='Native group lifecycle may disperse; standing cap .2mps, free individuals .8mps; no H1-H5 validation',
        portability_scope='Arrays and loader independent of ROS/simulator; historical world/driver absolute paths require asset/environment remapping to rerun simulation',
        hu_nav_sim='held-out; no train or tuning',copied_files=copied)
    dump(output/'provenance.json',provenance);print(json.dumps(dict(status='COPIED',bundle=str(output),raw_files=len(copied),bytes=sum(v['bytes'] for v in copied.values())),indent=2))
    return provenance
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();prepare(a.output.resolve())
