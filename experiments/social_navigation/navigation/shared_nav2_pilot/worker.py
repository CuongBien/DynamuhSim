#!/usr/bin/env python3
"""Compatible worker entrypoint; IPC implementation lives in common_gazebo."""
from pathlib import Path
import json
import os
import runpy
import sys


def policy_observer(destination):
    """Observe native Python calls without wrapping or replacing any policy method."""
    audit = dict(human_act_calls=0, safety_constraint_calls=0, policy_predict_calls={}, steps=0)
    def observe(frame, event, arg):
        module = frame.f_globals.get('__name__', '')
        name = frame.f_code.co_name
        if event == 'call':
            if module.startswith('crowd_sim_plus.envs.utils.') and name == 'act':
                agent = frame.f_locals.get('self')
                if type(agent).__name__ == 'Human':
                    audit['human_act_calls'] += 1
            elif module.startswith('crowd_sim_plus.envs.policy.') and name == 'predict':
                calls = audit['policy_predict_calls']
                calls[module] = calls.get(module, 0) + 1
            elif module == 'crowd_sim_plus.envs.crowd_sim_plus' and name == 'constrain_agent_action_exact':
                audit['safety_constraint_calls'] += 1
        elif event == 'return' and module == 'crowd_kernel_n6' and name == 'advance':
            kernel = frame.f_locals['self']
            audit.update(steps=kernel.steps, human_policy=kernel.human_policy,
                         policy_classes=[type(h.policy).__module__ + '.' + type(h.policy).__name__ for h in kernel.env.humans],
                         robot_update='state injection only; no native robot act/step in this kernel')
            Path(destination).write_text(json.dumps(audit) + '\n')
    return observe

if __name__ == '__main__':
    if os.environ.get('DYNAMUHSIM_POLICY_AUDIT'):
        sys.setprofile(policy_observer(os.environ['DYNAMUHSIM_POLICY_AUDIT']))
    try:
        runpy.run_path(str(Path(__file__).resolve().parents[1]/'common_gazebo/worker.py'),
                      init_globals={'WORKER_VARIANT': 'shared'}, run_name='__main__')
    finally:
        sys.setprofile(None)
