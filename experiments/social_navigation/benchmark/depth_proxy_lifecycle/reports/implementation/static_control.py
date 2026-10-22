#!/usr/bin/env python3
"""One complete static third-scene control with isolated ordered shutdown."""
import fcntl,hashlib,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;OLD=BENCH/'depth_proxy_ablation'
sys.path.insert(0,str(OLD));import capture
from quiesce import stop_paused
original_stop=capture.stop_server

def stop(guard):return stop_paused(guard,original_stop)

if __name__=='__main__':
    output=HERE/'generated/runs/paused01';output.mkdir(parents=True,exist_ok=False)
    lock=(BENCH/'common_gazebo/generated/domain91.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    capture.stop_server=stop
    protocol=json.loads((OLD/'protocol.json').read_text());snapshot=json.loads((OLD/'snapshots.json').read_text())[2]
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    results=[]
    for condition in protocol['conditions']:
        if results:
            import time;time.sleep(protocol['gpu_cooldown_wall_s'])
        result=capture.capture(3,snapshot,condition,output,protocol)
        assert result['server_shutdown']['quiesce']['pause_acknowledged'] and result['server_shutdown']['quiesce']['simulation_time_stable']
        results.append(result)
    manifest=dict(status='PASS',scope='One static third-scene control; mitigation, root cause unresolved',fixtures=len(results),
        original_filter_sha256=hashlib.sha256((OLD/'generated/build_jazzy/libdepth_proxy_filter.so').read_bytes()).hexdigest(),
        shutdown_source_sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['quiesce.py','static_control.py']})
    (output/'capture.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest,indent=2))
