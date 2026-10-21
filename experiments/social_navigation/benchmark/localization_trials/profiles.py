"""Explicit runtime AMCL ablations; baseline robot YAML is never edited."""
import copy
PROFILES={
    'baseline':{},
    'lf_prob':dict(laser_model_type='likelihood_field_prob',do_beamskip=False),
    'lf_prob_beamskip':dict(laser_model_type='likelihood_field_prob',do_beamskip=True,
        beam_skip_distance=0.5,beam_skip_threshold=0.3,beam_skip_error_threshold=0.9),
}
READ_NAMES=['laser_model_type','do_beamskip','beam_skip_distance','beam_skip_threshold',
    'beam_skip_error_threshold','transform_tolerance','robot_model_type','max_beams',
    'min_particles','max_particles','laser_max_range','laser_min_range','update_min_d',
    'update_min_a','alpha1','alpha2','alpha3','alpha4','alpha5','z_hit','z_rand','sigma_hit',
    'laser_likelihood_max_dist','initial_pose.x','initial_pose.y','initial_pose.yaw']

def configure(original,profile,pose=None):
    config=copy.deepcopy(original);params=config['amcl']['ros__parameters'];overrides=PROFILES[profile]
    if pose is not None:params['initial_pose'].update(x=pose[0],y=pose[1],yaw=pose[2])
    params.update(overrides)
    restored=copy.deepcopy(config);target=restored['amcl']['ros__parameters'];before=original['amcl']['ros__parameters']
    for key in overrides:
        if key in before:target[key]=copy.deepcopy(before[key])
        else:target.pop(key)
    if pose is not None:target['initial_pose']=copy.deepcopy(before['initial_pose'])
    assert restored==original,'Unexpected mutation outside isolated AMCL settings'
    return config

def read_parameters(node,wait,profile,pose):
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import parameter_value_to_python
    client=node.create_client(GetParameters,'/amcl/get_parameters')
    try:
        if not wait(client.service_is_ready,5):raise RuntimeError('AMCL parameter service missing')
        request=GetParameters.Request();request.names=READ_NAMES
        future=client.call_async(request)
        if not wait(future.done,5):raise RuntimeError('AMCL parameter read timed out')
        values={name:parameter_value_to_python(value) for name,value in zip(READ_NAMES,future.result().values)}
        expected=dict(laser_model_type='likelihood_field',**{}) if profile=='baseline' else PROFILES[profile]
        for name,value in expected.items():
            if values.get(name)!=value:raise RuntimeError(f'AMCL override not applied: {name}: {values.get(name)} != {value}')
        for name,value in zip(['initial_pose.x','initial_pose.y','initial_pose.yaw'],pose):
            if abs(values[name]-value)>1e-9:raise RuntimeError('AMCL initial pose mismatch')
        return values
    finally:node.destroy_client(client)
