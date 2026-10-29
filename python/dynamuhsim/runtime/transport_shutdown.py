"""Ordered stop of exact, session-owned ROS launch Gazebo transport children."""
import os
from pathlib import Path
import re
import signal
import time

START = re.compile(r"^\[INFO\] \[((?:parameter_bridge|image_bridge)-\d+)\]: process started with pid \[(\d+)\]$")
PARAM = b"/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge"
IMAGE = b"/opt/ros/jazzy/lib/ros_gz_image/image_bridge"
ORDER = ["ground_truth", "obstacle", "camera_image", "robot"]


def argv(pid):
    return (Path('/proc')/str(pid)/'cmdline').read_bytes().split(b'\0')


def role(args):
    if args[0] == PARAM:
        if b'__node:=nav2_pilot_ground_truth_bridge' in args and b'/common/ground_truth@std_msgs/msg/String[gz.msgs.StringMsg' in args:return 'ground_truth'
        if b'__node:=obstacle_type_bridge' in args and b'/obstacle_type@std_msgs/msg/String]gz.msgs.StringMsg' in args and b'/obstacle_type/status@std_msgs/msg/String[gz.msgs.StringMsg' in args:return 'obstacle'
        if b'--params-file' in args and not any(x.startswith(b'__node:=') for x in args):return 'robot'
    if args[0] == IMAGE and b'__node:=camera_image_bridge' in args and b'/camera/image' in args and b'/camera/depth_image' in args:return 'camera_image'
    return None


def stop_launch_bridges(guard, launch_log, timeout=5):
    """One SIGINT per identity-pinned child, while paused Gazebo remains alive.

    Only startup PIDs from this launch log that also match the guard's same-UID
    session token and exact supported executable/arguments can be signalled.
    ROS launch reaps its own children and records clean/failed terminal status.
    The separately owned paired-depth ros2-run bridge is not selected here.
    """
    try:lines=launch_log.read_text(errors='replace').splitlines()
    except FileNotFoundError:return dict(status='MISSING_LAUNCH_LOG',clean=False,bridges=[])
    owned={p['pid']:p for p in guard.members()};selected={}
    for line in lines:
        match=START.fullmatch(line)
        if not match:continue
        label,pid=match[1],int(match[2])
        if pid not in owned:return dict(status='BRIDGE_NOT_OWNED_OR_ALIVE',clean=False,bridges=[],pid=pid,label=label)
        try:args=argv(pid)
        except OSError:return dict(status='IDENTITY_UNAVAILABLE',clean=False,bridges=[],pid=pid)
        name=role(args)
        if name is None or name in selected:return dict(status='BRIDGE_ROLE_OR_CARDINALITY_ERROR',clean=False,bridges=[])
        selected[name]=dict(role=name,label=label,initial=owned[pid],args=args)
    if set(selected)!=set(ORDER):return dict(status='BRIDGE_ROLE_OR_CARDINALITY_ERROR',clean=False,bridges=[])
    receipts=[]
    for name in ORDER:
        item=selected[name];pid=item['initial']['pid'];entry={k:v for k,v in item.items() if k!='args'}
        try:
            descriptor=os.pidfd_open(pid)
            try:
                if not any(p['pid']==pid for p in guard.members()) or argv(pid)!=item['args']:
                    entry.update(status='IDENTITY_CHANGED',clean=False)
                else:signal.pidfd_send_signal(descriptor,signal.SIGINT);entry['signal']='SIGINT'
            finally:os.close(descriptor)
        except OSError as error:entry.update(status='SIGNAL_ERROR',clean=False,error=str(error))
        if 'signal' in entry:
            deadline=time.monotonic()+timeout
            while True:
                text=launch_log.read_text(errors='replace').splitlines();record=next((v for v in reversed(text) if v.startswith(f"[INFO] [{item['label']}]: process has finished cleanly [pid {pid}]") or v.startswith(f"[ERROR] [{item['label']}]: process has died [pid {pid},")),None)
                if record is not None:entry.update(status='OBSERVED',clean=record.startswith('[INFO]'),record=record);break
                if time.monotonic()>=deadline:entry.update(status='TIMEOUT',clean=False);break
                time.sleep(.02)
        receipts.append(entry)
    return dict(status='OBSERVED',clean=all(v['status']=='OBSERVED' and v['clean'] for v in receipts),bridges=receipts,scope='Only four ROS-launch-owned bridge children, before server teardown')
