"""Read-only command/schema evidence and legacy recovery loaders.

TASK03R removes perception replay/evaluation and the monolithic completeness report.
Original source and measured artifacts remain in task03_contract_source_before/.
Factual WM eligibility lives in single_episode_export; experimental tracks are optional.
"""
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import numpy as np
from .single_episode_export import causal, sha, check_export_gate, NS

REQUEST_STAGE = 'nav2_output_pre_gazebo_acceleration_limiter'
SENSOR_KEYS = ['depth', 'depth_stamp_ns', 'depth_record_ns', 'camera_info_stamp_ns',
               'camera_info_record_ns', 'camera_k', 'camera_d', 'odom_stamp_ns',
               'odom_record_ns', 'odom_pose', 'static_parent', 'static_child',
               'static_transform', 'static_record_ns']



def write_json(path, value):
    encoded=json.dumps(value, indent=2, allow_nan=False)+'\n'
    with Path(path).open('x') as f:
        f.write(encoded)



def read_jsonl(path):
    return [json.loads(s) for s in Path(path).read_text().splitlines() if s.strip()]



def sensor_inputs(path):
    """Whitelist extraction: no GT, map, simulator IDs, events or future bank."""
    with np.load(path, allow_pickle=False) as bank:
        return {k:bank[k] for k in SENSOR_KEYS}



def trace_commands(run,repository):
    run=Path(run);repository=Path(repository);freeze=json.loads((run/'capture_freeze.json').read_text())
    relevant=['src/custom_corridor/launch/_dynamuhsim_nav2.py','src/custom_corridor/launch/_dynamuhsim_simulation.py',
              'src/custom_corridor/config/mecanum_bridge.yaml','python/dynamuhsim/simulation/capture_launch.py']
    hashes={p:sha(repository/p) for p in relevant}
    if not all(freeze['source_sha256'].get(p)==h for p,h in hashes.items()):
        raise ValueError('Routing source differs from the recorded source freeze')
    result=json.loads((run/'episode/result.json').read_text());text=(run/'episode/launch.log').read_text()
    publishers={r['node'] for key in ['cmd_vel_publishers_before_goal','cmd_vel_publishers_after_goal'] for r in result[key]}
    if publishers!={'controller_server','behavior_server'}:raise ValueError('Unexpected command publishers')
    bridge_lines=[s for s in text.splitlines() if 'Creating ROS->GZ Bridge: [cmd_vel ' in s]
    if len(bridge_lines)!=1:raise ValueError('No unique recorded bridge evidence')
    processes=[re.search(r'\[INFO\] \[([^]]+)\]: process started',s).group(1) for s in text.splitlines() if re.search(r'\[INFO\] \[([^]]+)\]: process started',s)]
    if any(s in p for p in processes for s in ['velocity_smoother','collision_monitor']):raise ValueError('Untraced downstream ROS filter')
    model=ET.parse(run/'runtime/mecanum_camera.sdf').getroot().find('model')
    plugin=model.find("plugin[@name='gz::sim::systems::MecanumDrive']")
    if plugin.findtext('topic')!='cmd_vel':raise ValueError('Unexpected native actuator input')
    return dict(status='RECOVERED_FROM_RAW',meaning='Requested body twist at Nav2 public robot-command interface',
        stage=REQUEST_STAGE,source='/cmd_vel',columns=['vx','vy','wz'],frame='base_footprint',
        units=['m/s','m/s','rad/s'],publishers=sorted(publishers),
        per_message_publisher_identity='NOT_MEASURED: MCAP channel has no publisher GID',
        pipeline=['controller_server MPPI output OR behavior_server output','ROS /cmd_vel TwistStamped',
                  'ros_gz_bridge ROS_TO_GZ Twist','Gazebo MecanumDrive acceleration limiter','wheel/physics','odom / Gazebo pose'],
        ros_post_filter_present=False,internal_mppi_constraints_precede_publication=True,
        internal_mppi_pre_constraint_sequence_recovered=False,applied_actuator_signal_recovered=False,
        native_min_acceleration=float(plugin.findtext('min_acceleration')),
        native_max_acceleration=float(plugin.findtext('max_acceleration')),
        bridge_log_lines=bridge_lines,launch_processes=processes,source_sha256=hashes,
        publisher_evidence={k:result[k] for k in ['cmd_vel_publishers_before_goal','cmd_vel_publishers_after_goal']},
        logs_sha256={str(run/p):sha(run/p) for p in ['episode/launch.log','episode/result.json','runtime/nav2_runtime.yaml','runtime/mecanum_camera.sdf']})



def executed_body_twist(robot_world,velocity_world,available):
    """Measured pose-displacement twist, not command or native actuator feedback."""
    out=np.full_like(velocity_world,np.nan);available=np.asarray(available,dtype=bool)
    yaw=robot_world[available,2];vx,vy=velocity_world[available,:2].T
    out[available,0]=np.cos(yaw)*vx+np.sin(yaw)*vy
    out[available,1]=-np.sin(yaw)*vx+np.cos(yaw)*vy
    out[available,2]=velocity_world[available,2]
    return out



def causal_candidates(frames,queries,max_age_ns=150_000_000):
    """Select only actual reconstructed measurements available by each query.

    Empty successful inference is distinct from unavailable inference. Missing
    detections do not become predicted/extrapolated person observations.
    """
    idx,good=causal([r['stamp_ns'] for r in frames],[r['available_sim_ns'] for r in frames],queries,max_age_ns)
    selected=[];available=[];stamps=[]
    for i,ok in zip(idx,good):
        valid=bool(ok and frames[i]['observation_valid'])
        selected.append(frames[i]['tracks'] if valid else [])
        available.append(valid);stamps.append(frames[i]['stamp_ns'] if valid else -1)
    return selected,np.asarray(available,dtype=bool),np.asarray(stamps,dtype=np.int64)



def load_recovered_sample(directory,sample_id,allow_invalid=False,*,expected_base=None):
    from .single_episode_export import load_sample
    directory=Path(directory);report=json.loads((directory/'missing_sources.json').read_text())
    base=Path(report['base_export'])
    if expected_base is not None and Path(expected_base).resolve()!=base.resolve():
        raise ValueError('Recovery belongs to a different base export')
    proof=json.loads((directory/'command_provenance.json').read_text())
    if sha(directory/'command_provenance.json')!=report['recovery_artifact_sha256']['command_provenance.json'] or proof['stage']!=REQUEST_STAGE:
        raise ValueError('Requested-command provenance not verified')
    for name,digest in report['base_artifact_sha256'].items():
        if sha(base/name)!=digest:raise ValueError('Base artifact changed: '+name)
    prediction_path=directory/'perception/observed_candidates.jsonl'
    if sha(prediction_path)!=report['recovery_artifact_sha256']['perception/observed_candidates.jsonl']:
        raise ValueError('Reconstruction artifact changed')
    sample=load_sample(base,sample_id,allow_invalid)
    act=sample['act'];act['past_requested_commands']=act['past_published_commands'].copy()
    act['past_requested_commands_available']=act['availability'].copy()
    act['requested_stage']=REQUEST_STAGE;act['internal_mppi_pre_constraint_available']=False
    act['future_candidate_requested']=None;act['future_candidate_requested_available']=False
    # Retrospective measured movement belongs only in GT, never in ACT condition.
    gt=sample['gt'];gt['future_executed_actions']=executed_body_twist(gt['robot_world'],gt['robot_velocity_world'],gt['robot_velocity_available'])
    gt['future_executed_actions_available']=gt['robot_velocity_available'].copy()
    gt['future_executed_actions_frame']='base_footprint'
    gt['future_executed_actions_method']='100-125ms backward Gazebo pose displacement; rotate world velocity by measured endpoint yaw; not actuator feedback'
    anchor=sample['index']['anchor_ns'];end=anchor+3*NS
    gt['future_issued_commands']=[r for r in read_jsonl(base/'controls.jsonl')
        if r['record_type']=='observed_published_command' and anchor<r['source_stamp_ns']<=end]
    tracks,available,stamps=causal_candidates(read_jsonl(prediction_path),sample['obs']['query_ns'])
    sample['obs'].update(experimental_person_candidates=tracks,experimental_candidate_frame_available=available,
        experimental_candidate_stamp_ns=stamps,experimental_candidate_frame='odom',
        candidate_semantic_validated=False,candidate_confidence_calibrated=False,
        observed_occlusion_known=np.zeros(len(available),dtype=bool))
    sample['complete_requested_obs_act']=False
    return sample



def recover_action_status(run, output, audit):
    """Verified installed definitions as sidecar; never patch the original MCAP."""
    from ament_index_python.packages import get_package_share_directory
    from rclpy.serialization import deserialize_message
    from action_msgs.msg import GoalStatusArray
    from dynamuhsim.analysis.mcap_records import McapFile
    root_type='action_msgs/msg/GoalStatusArray';proof=audit['local_type_hashes'][root_type]
    package=Path(get_package_share_directory('action_msgs'))
    description=json.loads((package/'msg/GoalStatusArray.json').read_text())
    current=next(v['hash_string'] for v in description['type_hashes'] if v['type_name']==root_type)
    if not proof['match'] or current!=proof['recorded']:raise ValueError('Action schema type hash mismatch')
    names=[root_type,'action_msgs/msg/GoalStatus','action_msgs/msg/GoalInfo',
           'unique_identifier_msgs/msg/UUID','builtin_interfaces/msg/Time']
    definitions={}
    for name in names:
        pkg,folder,base=name.split('/');p=Path(get_package_share_directory(pkg))/folder/(base+'.msg')
        definitions[name]=dict(source_path=str(p),sha256=sha(p),text=p.read_text())
    rows=[]
    for path in sorted((Path(run)/'episode/raw_bag').glob('*.mcap')):
        for r in McapFile(path).messages():
            if r['topic']!='/navigate_to_pose/_action/status':continue
            m=deserialize_message(r['payload'],GoalStatusArray)
            rows.append(dict(record_ns=r['record_ns'],publish_wall_ns=r['publish_ns'],
                statuses=[dict(status=s.status,goal_id=[int(v) for v in s.goal_info.goal_id.uuid],
                    goal_stamp_ns=s.goal_info.stamp.sec*NS+s.goal_info.stamp.nanosec) for s in m.status_list]))
    report=dict(root_type=root_type,type_hash=current,definitions=definitions,messages=rows,
        source='Installed ROS interfaces validated against recorded RIHS01; definitions were NOT recovered from MCAP',
        raw_embedded_schema_repaired=False,original_raw_verdict=audit['verdict'])
    write_json(output,report)
    return report



def verify_base(audit_path,base):
    audit=check_export_gate(audit_path);base=Path(base)
    export=json.loads((base/'export_report.json').read_text())
    if Path(export['run_dir']).resolve()!=Path(audit['run_dir']).resolve():raise ValueError('Wrong base episode')
    if export['audit_sha256']!=sha(audit_path):raise ValueError('Audit receipt changed')
    for name,digest in export['output_sha256'].items():
        if sha(base/name)!=digest:raise ValueError('Base export checksum mismatch: '+name)
    return audit,export



def command_measurements(base,audit):
    controls=[r for r in read_jsonl(Path(base)/'controls.jsonl') if r['record_type']=='observed_published_command']
    values=np.asarray([r['value'] for r in controls]);stamps=np.asarray([r['source_stamp_ns'] for r in controls])
    with np.load(Path(base)/'observations.npz',allow_pickle=False) as bank:
        ids,valid=causal(stamps,[r['record_stamp_ns'] for r in controls],bank['odom_stamp_ns'],50_000_000,
                         [r['publish_wall_ns'] for r in controls])
        actual=bank['odom_twist_body'][:,[0,1,5]][valid]
        error=actual-values[ids[valid]]
    return dict(messages=len(controls),unique_sim_stamps=len(set(stamps)),duplicate_sim_stamps=len(stamps)-len(set(stamps)),
        changing_adjacent_commands_at_same_stamp=int(np.count_nonzero((np.diff(stamps)==0)&np.any(np.diff(values,axis=0)!=0,axis=1))),
        source_start_ns=int(stamps[0]),source_end_ns=int(stamps[-1]),
        publish_wall_hz=audit['topic_statistics']['/cmd_vel']['clocks']['publish_wall']['frequency_hz']['median'],
        source_record_offset_s=audit['topic_statistics']['/cmd_vel']['record_minus_source_s'],
        max_abs_vx_vy_wz=np.max(np.abs(values),axis=0).tolist(),nonzero_vy_messages=int(np.count_nonzero(np.abs(values[:,1])>1e-6)),
        odom_comparison=dict(method='same base_footprint components; newest command source+record <= odom header, max age 50ms',
            pairs=int(valid.sum()),columns=['vx','vy','wz'],rmse=np.sqrt(np.mean(error**2,axis=0)).tolist(),
            max_abs_error=np.max(np.abs(error),axis=0).tolist(),
            interpretation='Agreement is a motion cross-check, not proof of identity or actuator application'),
        gazebo_norm_comparison=audit['commands']['source_cross_validation']['velocity_norm_comparisons']['observed_cmd_vel'])



def recover_episode(audit_path,base,output,repository):
    """Verify command/schema evidence only; never run perception or simulation."""
    from rclpy.serialization import deserialize_message
    audit,_=verify_base(audit_path,base);output=Path(output);output.mkdir(parents=True,exist_ok=False)
    proof=trace_commands(audit['run_dir'],repository)
    write_json(output/'command_provenance.json',proof)
    status=recover_action_status(audit['run_dir'],output/'action_status_schema_sidecar.json',audit)
    result=dict(status='COMMAND_SCHEMA_EVIDENCE_VERIFIED',stage=proof['stage'],
        action_status_messages=len(status['messages']),raw_input_unchanged=all(sha(p)==h for p,h in audit['input_sha256'].items()),
        perception_executed=False,simulation_executed=False)
    write_json(output/'evidence_report.json',result)
    return result
