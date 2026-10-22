"""Final bounded speed variant, unchanged inflation and robot geometry."""
from config_runtime import apply_runtime_config as parent_apply

def apply_runtime_config(config):
    parent_apply(config)
    config['controller_server']['ros__parameters']['FollowPath'].update(vx_max=.30,vx_min=-.15,vy_max=.20)
    return config

def capture_live_parameters(node,wait,output):
    import json
    from rclpy.parameter_client import AsyncParameterClient
    expected={
      '/controller_server':{'FollowPath.vx_max':.30,'FollowPath.vx_min':-.15,'FollowPath.vy_max':.20,'FollowPath.CostCritic.consider_footprint':False},
    }
    for name in ['local_costmap','global_costmap']:
        expected[f'/{name}/{name}']={'robot_radius':.25,'footprint_padding':.01,'inflation_layer.inflation_radius':.8,'inflation_layer.cost_scaling_factor':3.}
    observed={}
    for remote,parameters in expected.items():
        client=AsyncParameterClient(node,remote)
        assert wait(client.services_are_ready,10),f'Parameter service not ready: {remote}'
        future=client.get_parameters(list(parameters));assert wait(future.done,10)
        values=future.result().values;assert len(values)==len(parameters)
        observed[remote]={name:(value.bool_value if isinstance(wanted,bool) else value.double_value) for (name,wanted),value in zip(parameters.items(),values)}
        assert observed[remote]==parameters,(remote,observed[remote],parameters)
    (output/'live_parameters.json').write_text(json.dumps(observed,indent=2)+'\n')
    return observed
