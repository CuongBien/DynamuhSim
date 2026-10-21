#!/usr/bin/env python3
"""Persistent, bounded native group viewer; no Nav2 or goal-driven auto-close."""
import argparse,json,os,signal,sys,threading,time
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;SOCIAL=BENCH.parent
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--hold-seconds',type=float,default=600);a=p.parse_args();out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
sys.path.insert(0,str(BENCH));from shutdown_fix.ordered_stop import launch_environment
env,preload=launch_environment(out);os.environ.update(env)
os.environ.update(ROS_DOMAIN_ID='89',GZ_PARTITION='dynamuhsim_group_viewer_'+str(os.getpid()),QT_QPA_PLATFORM='xcb')
meta=dict(pid=os.getpid(),mode='walking',nav=False,gui=True,qt_platform='xcb',hold_wall_seconds=a.hold_seconds,ros_domain_id=89,gz_partition=os.environ['GZ_PARTITION'],preload=preload,scope='Live Gazebo observer/GUI with native group. Robot stationary; not a navigation benchmark. Auto-stop timer only; no goal closes this viewer.')
(out/'viewer.json').write_text(json.dumps(meta,indent=2));print('VIEWER',json.dumps(meta),flush=True)
timer=threading.Timer(a.hold_seconds,lambda:os.kill(os.getpid(),signal.SIGINT));timer.daemon=True;timer.start()
sys.path.insert(0,str(SOCIAL/'gazebo_adapter'));from run import main
sys.argv=[str(SOCIAL/'gazebo_adapter/run.py'),'--mode','walking','--seed','42','--gui']
try:code=main()
finally:timer.cancel()
(out/'viewer_end.json').write_text(json.dumps(dict(returncode=code,finished_epoch=time.time()),indent=2));raise SystemExit(0 if code in (0,130) else code)
