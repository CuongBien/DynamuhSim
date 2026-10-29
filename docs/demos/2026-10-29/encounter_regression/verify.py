"""Compare frozen inputs and treatment isolation without rewriting historical files."""
import argparse,hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.simulation.capture import config_changes
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];checks={};evidence={}
def world(path):
 tree=ET.parse(path).getroot();sockets=tree.findall("world/plugin[@name='response_experiment::Adapter']/socket")
 if len(sockets)!=1:raise ValueError('Expected one worker socket')
 sockets[0].text='<RUN_LOCAL_SOCKET>';return ET.tostring(tree)
for profile in ['head_on','crossing']:
 runs=[raw/(profile+'_'+suffix) for suffix in ['legacy','group']];results=[json.loads((run/'episode/result.json').read_text()) for run in runs]
 for name in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf','assets/encounter.json']:checks[profile+':same:'+name]=(runs[0]/name).read_bytes()==(runs[1]/name).read_bytes()
 checks[profile+':same_initial_humans']=results[0]['ready']['humans']==results[1]['ready']['humans'];checks[profile+':same_initial_groups']=results[0]['ready']['groups']==results[1]['ready']['groups'];checks[profile+':same_nav_config']=(runs[0]/'runtime/nav2_runtime.yaml').read_bytes()==(runs[1]/'runtime/nav2_runtime.yaml').read_bytes();checks[profile+':world_same_except_socket']=world(runs[0]/'runtime/warehouse.sdf')==world(runs[1]/'runtime/warehouse.sdf');checks[profile+':same_runtime_sources']=results[0]['head']==results[1]['head'] and all(results[1]['source_sha256'].get(k)==v for k,v in results[0]['source_sha256'].items() if not k.startswith(plan['raw_root']))
for case in plan['gap_cases']:
 old=ROOT/'experiments/social_navigation/benchmark/generated/static_gap_20261029'/(('wide80' if case['case']=='wide' else 'narrow52')+'_horizon');run=raw/case['id'];before=yaml.safe_load((old/'runtime/nav2_runtime.yaml').read_text());after=yaml.safe_load((run/'runtime/nav2_runtime.yaml').read_text());diff=config_changes(before,after);checks[run.name+':only_noise_control_change']={c['path'] for c in diff}=={'/controller_server/ros__parameters/FollowPath/regenerate_noises'}
 for name in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf','assets/gap_fixture.json','assets/gap.pgm','assets/gap.yaml']:checks[run.name+':same_old_fixture:'+name]=(old/name).read_bytes()==(run/name).read_bytes()
for case in plan['social_cases']+plan['gap_cases']:
 name=case['id'];run=raw/name;result=json.loads((run/'episode/result.json').read_text());audit=json.loads((raw/(name+'_audit.json')).read_text());checks[name+':integrity']=audit['status']=='PASS';checks[name+':native_gpu']=result['rendering']['hardware_verified'] and 'NVIDIA' in result['rendering']['renderer'];checks[name+':no_injected_velocity']=result['no_scripted_robot_velocity_publisher'];checks[name+':cleanup']=result['gazebo_clean_exit'] and result['cleanup']['status']=='PASS';files=['capture_freeze.json','plan.json','runtime/nav2_runtime.yaml','runtime/warehouse.sdf','episode/result.json','episode/gazebo_poses.jsonl','episode/native_costmaps.jsonl','episode/events.jsonl','episode/camera_preview_4x.gif'];evidence[name]=dict(runtime_commit=result['head'],file_sha256={f:hashlib.sha256((run/f).read_bytes()).hexdigest() for f in files})
receipt=dict(schema='encounter_regression_verification/1',status='PASS' if all(checks.values()) else 'FAIL',checks=checks,evidence=evidence)
with a.output.open('x') as f:f.write(json.dumps(receipt,indent=2)+'\n')
print(receipt['status'],[k for k,v in checks.items() if not v])
