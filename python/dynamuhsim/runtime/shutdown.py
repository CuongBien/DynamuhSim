"""Shared pause/quiesce mitigation before ordered warehouse Gazebo shutdown."""
import json
import subprocess


def stop_paused(guard,original_stop,world='social_warehouse'):
    """Pause is a mitigation, not a GPU fence or proof of root cause."""
    import threading, time
    from gz.transport13 import Node
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    QUIET_SECONDS = .5
    event=threading.Event();stats=[];node=Node()
    def received(msg):
        if msg.paused:
            stats.append(dict(paused=True,sim_sec=msg.sim_time.sec,sim_nsec=msg.sim_time.nsec));event.set()
    topic=f'/world/{world}/stats';subscribed=node.subscribe(WorldStatistics,topic,received)
    request=subprocess.run(['gz','service','-s',f'/world/{world}/control','--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause: true'],capture_output=True,text=True,timeout=5)
    acknowledged=bool(subscribed and request.returncode==0 and 'data: true' in request.stdout and event.wait(3))
    settled=False
    if acknowledged:
        time.sleep(QUIET_SECONDS)
        # The statistics stream must confirm unchanged simulation time while paused.
        settled=bool(stats and len({(r['sim_sec'],r['sim_nsec']) for r in stats})==1)
    node.unsubscribe(topic)
    result=original_stop(guard)
    result['quiesce']=dict(pause_acknowledged=acknowledged,simulation_time_stable=settled,
        settle_wall_s=QUIET_SECONDS,stats=stats,request_returncode=request.returncode,
        response=request.stdout.strip(),stderr=request.stderr.strip(),is_gpu_fence=False)
    (guard.runtime/'quiesce_shutdown.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


def wait_gazebo_exit(process, launch_log, timeout=2):
    """Wait for launch's child watcher, without polling/reaping its Gazebo child.

    Closing launch before its pidfd callback can race subprocess transport.close
    in Python 3.12 and replace the child's exit status with 255. A terminal log
    record acknowledges that callback; a failed record is never made clean.
    """
    import time
    deadline = time.monotonic() + timeout
    while True:
        try:
            lines = launch_log.read_text(errors="replace").splitlines()
        except FileNotFoundError:
            lines = []
        for line in reversed(lines):
            if line.startswith("[INFO] [gz-1]: process has finished cleanly") or line.startswith("[ERROR] [gz-1]: process has died"):
                return dict(status="OBSERVED", clean=line.startswith("[INFO]"), record=line)
        if process.poll() is not None:
            return dict(status="LAUNCH_EXITED", clean=False)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return dict(status="TIMEOUT", clean=False)
        time.sleep(min(.02, remaining))
