"""Opt-in conservative sensor-based costmaps and bounded MPPI speeds; no oracle people input."""
import copy

def apply_runtime_config(config):
    for name in ['local_costmap','global_costmap']:
        params=config[name][name]['ros__parameters']
        params.pop('footprint',None)
        params.update(robot_radius=.25,footprint_padding=.01)
        params['inflation_layer'].update(inflation_radius=.8,cost_scaling_factor=3.)
    controller=config['controller_server']['ros__parameters']['FollowPath']
    controller.update(vx_max=.15,vx_min=-.10,vy_max=.10)
    # Jazzy CostCritic radius mode requires consider_footprint=false; keep native mode.
    controller['CostCritic']['consider_footprint']=False
    return config

def changes(before,after,prefix=''):
    result=[]
    for key in sorted(set(before)|set(after)):
        path=prefix+'/'+key
        a,b=before.get(key),after.get(key)
        if isinstance(a,dict) and isinstance(b,dict):result.extend(changes(a,b,path))
        elif a!=b:result.append(dict(path=path,before=a,after=b))
    return result


def capture_live_parameters(node,wait,output):
    import json
    from rclpy.parameter_client import AsyncParameterClient
    expected={
      '/controller_server':{'FollowPath.vx_max':.15,'FollowPath.vx_min':-.10,'FollowPath.vy_max':.10,'FollowPath.CostCritic.consider_footprint':False},
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
