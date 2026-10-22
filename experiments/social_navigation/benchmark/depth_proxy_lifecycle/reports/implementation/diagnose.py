#!/usr/bin/env python3
"""One owned failing fixture; collect native waits/stacks only after graceful stop timeout."""
import argparse,ctypes,fcntl,json,os,signal,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;OLD=BENCH/'depth_proxy_ablation'
sys.path.insert(0,str(OLD));import capture
original_launch=capture.launch_environment;original_stop=capture.stop_server
TRACE=False

def launch(runtime):
    env,meta=original_launch(runtime)
    library=HERE/'generated/liblifecycle_trace.so'
    env['LD_PRELOAD']=str(library)+(':'+env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
    meta['diagnostic_preload']=str(library);return env,meta

def stop(guard):
    result=original_stop(guard);diagnostic=[]
    libc=ctypes.CDLL(None,use_errno=True)
    for member in result.get('remaining',[]):
        pid=member['pid'];proc=Path('/proc')/str(pid);tasks=[]
        try:
            (guard.runtime/f'maps_hung_{pid}.txt').write_bytes((proc/'maps').read_bytes())
            for task in sorted((proc/'task').iterdir()):
                data=dict(tid=int(task.name))
                for name in ['comm','wchan','syscall','stack']:
                    try:data[name]=(task/name).read_text().strip()
                    except OSError as e:data[name]=str(e)
                tasks.append(data)
            # tgkill is confined to this guard's enumerated owned PID and its own live TIDs.
            for task in tasks:
                if TRACE:libc.syscall(234,pid,task['tid'],signal.SIGUSR2);time.sleep(.025)
        except OSError as e:tasks.append(dict(error=str(e)))
        diagnostic.append(dict(pid=pid,tasks=tasks))
    result['hung_threads']=diagnostic
    (guard.runtime/'shutdown_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--name',required=True);parser.add_argument('--trace',action='store_true');args=parser.parse_args()
    TRACE=args.trace
    output=HERE/'generated/runs'/args.name;output.mkdir(parents=True,exist_ok=False)
    lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    capture.launch_environment=launch if TRACE else original_launch;capture.stop_server=stop
    protocol=json.loads((OLD/'protocol.json').read_text());snapshot=json.loads((OLD/'snapshots.json').read_text())[2]
    (output/'protocol.json').write_text(json.dumps(dict(mode='Diagnostic only; optional signal handler; not accepted sensor cohort',trace_preload=TRACE,source_protocol=protocol),indent=2)+'\n')
    try:capture.capture(3,snapshot,'masked',output,protocol)
    except AssertionError:print('Diagnostic retained; failure expected, see result and native thread traces',flush=True)
