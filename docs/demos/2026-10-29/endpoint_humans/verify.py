"""Verify treatment ownership and exact initial geometry, without mutating captures."""
import argparse,hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.capture import config_changes
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];old=ROOT/'experiments/social_navigation/benchmark/generated/factory_regression_20261029';checks={};evidence={}
def normalized_world(path):
 tree=ET.parse(path).getroot();items=tree.findall("world/plugin[@name='response_experiment::Adapter']/socket")
 if len(items)!=1:raise ValueError('Expected one worker socket')
 items[0].text='<RUN_LOCAL_SOCKET>';return ET.tostring(tree)
for scenario in ['standing','walking']:
 runs=[old/(scenario+'_h5'),raw/(scenario+'_resample'),raw/(scenario+'_group')];results=[json.loads((run/'episode/result.json').read_text()) for run in runs];configs=[yaml.safe_load((run/'runtime/nav2_runtime.yaml').read_text()) for run in runs]
 changes=config_changes(configs[0],configs[1]);checks[scenario+':v6_to_v7_only_resampling']={c['path'] for c in changes}=={'/controller_server/ros__parameters/FollowPath/regenerate_noises'};checks[scenario+':human_treatment_same_nav_config']=configs[1]==configs[2]
 for name in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf']:
  checks[scenario+':same:'+name]=len({(run/name).read_bytes() for run in runs})==1
 checks[scenario+':same_initial_humans']=results[0]['ready']['humans']==results[1]['ready']['humans']==results[2]['ready']['humans'];checks[scenario+':same_initial_groups']=results[0]['ready']['groups']==results[1]['ready']['groups']==results[2]['ready']['groups'];checks[scenario+':world_same_except_socket']=len({normalized_world(run/'runtime/warehouse.sdf') for run in runs})==1
 checks[scenario+':same_vendor_backend'] = all(results[0]['source_sha256'][k]==results[1]['source_sha256'].get(k)==results[2]['source_sha256'].get(k) for k in results[0]['source_sha256'] if '/vendor/SocNavGym/' in k)
 for run,result in zip(runs[1:],results[1:]):
  audit=json.loads((raw/(run.name+'_audit.json')).read_text());checks[run.name+':integrity']=audit['status']=='PASS';checks[run.name+':native_gpu']=result['rendering']['hardware_verified'] and 'NVIDIA' in result['rendering']['renderer'];checks[run.name+':no_injected_velocity']=result['no_scripted_robot_velocity_publisher'];checks[run.name+':clean_exit']=result['gazebo_clean_exit'] and result['cleanup']['status']=='PASS'
  checks[run.name+':human_profile']=result.get('human_motion_profile','legacy')==('group_orca_v1' if run.name.endswith('_group') else 'legacy');files=['capture_freeze.json','plan.json','runtime/nav2_runtime.yaml','runtime/warehouse.sdf','episode/result.json','episode/gazebo_poses.jsonl','episode/native_costmaps.jsonl','episode/events.jsonl','episode/camera_preview_4x.gif','runtime/worker_trajectory.jsonl'];evidence[run.name]=dict(head=result['head'],files_sha256={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in files})
receipt=dict(schema='endpoint_humans_verification/1',status='PASS' if all(checks.values()) else 'FAIL',checks=checks,runs=evidence)
with a.output.open('x') as f:f.write(json.dumps(receipt,indent=2)+'\n')
print(receipt['status'],[k for k,v in checks.items() if not v])
