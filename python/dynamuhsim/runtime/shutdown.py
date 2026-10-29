"""Shared pause/quiesce mitigation before ordered warehouse Gazebo shutdown."""
import json
import subprocess
from .ordered_stop import launch_environment, stop_server


def stop_paused(guard,original_stop,world='social_warehouse', *, before_stop=None):
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
    bridge = (before_stop() if acknowledged and settled else dict(status="NOT_QUIESCENT", clean=False)) if before_stop else None
    result=original_stop(guard)
    if bridge is not None:
        if "bridges" in bridge:
            result["transport_bridge_shutdown"] = bridge
            result["ground_truth_bridge_shutdown"] = next((v for v in bridge["bridges"] if v.get("role") == "ground_truth"), dict(status="NOT_OBSERVED",clean=False))
        else:
            result["ground_truth_bridge_shutdown"] = bridge
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


def stop_ground_truth_bridge(guard, launch_log, timeout=5):
    """Stop the exact session-owned bridge while its Gazebo publisher is alive.

    A pidfd pins identity across the last ownership check and signal. ROS launch
    remains the child's reaper; its terminal log record determines clean exit.
    No retry, broad process-name signal, or conversion of an error to success.
    """
    import os
    import signal
    import time
    from pathlib import Path
    expected = "/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg"

    def matches(item):
        try:
            args = (Path('/proc') / str(item['pid']) / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            return False
        return (args[0] == b'/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge'
                and expected.encode() in args
                and b'__node:=nav2_pilot_ground_truth_bridge' in args)

    candidates = [item for item in guard.members() if matches(item)]
    if len(candidates) != 1:
        return dict(status="OWNERSHIP_OR_CARDINALITY_ERROR", clean=False, candidates=candidates)
    item = candidates[0]
    try:
        descriptor = os.pidfd_open(item['pid'])
        try:
            if not any(p['pid'] == item['pid'] and matches(p) for p in guard.members()):
                return dict(status="IDENTITY_CHANGED", clean=False, initial=item)
            signal.pidfd_send_signal(descriptor, signal.SIGINT)
        finally:
            os.close(descriptor)
    except OSError as error:
        return dict(status="SIGNAL_ERROR", clean=False, initial=item, error=str(error))
    deadline = time.monotonic() + timeout
    while True:
        try:
            lines = launch_log.read_text(errors="replace").splitlines()
        except FileNotFoundError:
            lines = []
        for line in reversed(lines):
            if f"[pid {item['pid']}]" in line and "]: process has finished cleanly" in line:
                return dict(status="OBSERVED", clean=True, initial=item, signal="SIGINT", record=line)
            if f"[pid {item['pid']}," in line and "]: process has died" in line:
                return dict(status="OBSERVED", clean=False, initial=item, signal="SIGINT", record=line)
        if time.monotonic() >= deadline:
            return dict(status="TIMEOUT", clean=False, initial=item, signal="SIGINT")
        time.sleep(.02)
