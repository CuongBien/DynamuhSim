"""Stop only this experiment's Gazebo partition, before shutting down ROS/worker."""
import os,subprocess,time,shlex,shutil
from pathlib import Path

def stop_server(guard):
    partition=os.environ.get('GZ_PARTITION','')
    servers=lambda:[p for p in guard.members() if p['command'].lstrip().startswith('gz sim ')]
    initial=servers()
    if not initial:return dict(status='NOT_RUNNING',partition=partition)
    for server in initial:
        try:(guard.runtime/('gazebo_maps_before_stop_'+str(server['pid'])+'.txt')).write_bytes((Path('/proc')/str(server['pid'])/'maps').read_bytes())
        except OSError:pass
    begin=time.monotonic()
    try:
        result=subprocess.run(['gz','service','-s','/server_control','--reqtype','gz.msgs.ServerControl',
            '--reptype','gz.msgs.Boolean','--timeout','3000','--req','stop: true'],capture_output=True,text=True,timeout=8)
    except (OSError,subprocess.TimeoutExpired) as error:
        return dict(status='REQUEST_ERROR',partition=partition,error=str(error),remaining=servers())
    deadline=time.monotonic()+8
    while servers() and time.monotonic()<deadline:time.sleep(.05)
    remaining=servers()
    return dict(status='STOPPED' if not remaining else 'STILL_RUNNING',partition=partition,initial=initial,
        request_returncode=result.returncode,response=result.stdout.strip(),stderr=result.stderr.strip(),
        elapsed_wall_s=time.monotonic()-begin,remaining=remaining)


def launch_environment(runtime):
    """Keep WSL D3D12 core resident until process exit; scope to gz sim only.

    WSL driver thread exit callbacks can outlive dlclose during Sensors teardown.
    The opt-in wrapper uses the loader's startup preload lifetime, without patching
    system libraries, Gazebo, ROS launch files, or the caller's environment.
    """
    core=Path('/usr/lib/wsl/lib/libd3d12core.so')
    if not core.is_file():raise RuntimeError('WSL D3D12 shutdown mode requires '+str(core))
    executable=shutil.which('gz')
    if not executable:raise RuntimeError('gz executable missing')
    directory=runtime/'shutdown_bin';directory.mkdir()
    wrapper=directory/'gz'
    wrapper.write_text('#!/bin/sh\nif [ "$1" = "sim" ]; then\n  export LD_PRELOAD='+shlex.quote(str(core))+'"${LD_PRELOAD:+:$LD_PRELOAD}"\nfi\nexec '+shlex.quote(executable)+' "$@"\n')
    wrapper.chmod(0o755)
    env=os.environ.copy();env['PATH']=str(directory)+os.pathsep+env['PATH']
    return env,dict(scope='gz sim only',library=str(core),real_gz=executable,wrapper=str(wrapper),mode='startup LD_PRELOAD; no system files changed')
