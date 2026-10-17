#!/usr/bin/env python3
"""Own the isolated worker, generated world, and ROS launch process group."""
import argparse
import fcntl
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent

def element(parent, tag, value=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if value is not None:
        e.text = str(value)
    return e

def make_world(runtime, socket_path, initial, mode):
    tree = ET.parse(ROOT/'warehouse/warehouse.sdf')
    world = tree.getroot().find('world')
    plugin = element(world,'plugin',filename=str(HERE/'build/libsocnavgym_adapter.so'),name='social_experiment::Adapter')
    element(plugin,'socket',socket_path)
    # Copy distribution GUI plugins into this generated world only.
    # Gazebo GUI config files contain XML fragments, rather than one root.
    gui_config=Path('/usr/share/gz/gz-sim8/gui/gui.config')
    fragment=re.sub(r'<\?xml[^>]*\?>','',gui_config.read_text())
    defaults=ET.fromstring('<config>'+fragment+'</config>')
    gui=element(world,'gui',fullscreen='false')
    for gui_plugin in defaults.findall('plugin'):
        if gui_plugin.get('filename')=='MinimalScene':
            gui_plugin.find('camera_pose').text='-29 -7 8 0 0.75 0.8'
        gui.append(gui_plugin)
    camera_model = element(world,'model',name='socnavgym_observer')
    element(camera_model,'static','true')
    element(camera_model,'pose','-21 -5 12 0 1.17 1.57079632679' if mode=='walking' else '-24 -6 7 0 0.85 1.57079632679')
    camera_link = element(camera_model,'link',name='camera_link')
    sensor = element(camera_link,'sensor',name='camera',type='camera')
    element(sensor,'always_on','true')
    element(sensor,'update_rate',3)
    element(sensor,'topic','/socnavgym/observer')
    optics = element(sensor,'camera')
    element(optics,'horizontal_fov',1.4 if mode=='walking' else 1.3)
    image = element(optics,'image')
    element(image,'width',800)
    element(image,'height',600)
    element(image,'format','R8G8B8')
    clip = element(optics,'clip')
    element(clip,'near',.1)
    element(clip,'far',80)
    for h in initial:
        name = f"socnavgym_{h['id']}"
        pose = f"{h['x']} {h['y']} .85 0 0 {h['yaw']}"
        model = element(world,'model',name=name+'_proxy')
        element(model,'pose',pose)
        link = element(model,'link',name='body')
        element(link,'gravity','false')
        element(link,'kinematic','true')
        for tag in ['collision','visual']:
            shape = element(link,tag,name='body_'+tag)
            cylinder = element(element(shape,'geometry'),'cylinder')
            element(cylinder,'radius',h['radius'])
            element(cylinder,'length',1.7)
            if tag == 'visual':
                # GPU LiDAR traces visual geometry. A visible translucent proxy is
                # intentional: it makes the exact sensor benchmark shape inspectable.
                element(shape,'transparency',.65)
                material = element(shape,'material')
                element(material,'ambient','.15 .7 .35 1')
                element(material,'diffuse','.15 .7 .35 1')
        actor = element(world,'actor',name=name+'_visual')
        element(actor,'pose','0 0 0 0 0 0')
        mesh = ROOT.parent.parent/'src/custom_corridor/models/human/meshes'/('stand.dae' if mode=='conversation' else 'walk_blue.dae')
        skin = element(actor,'skin')
        element(skin,'filename',mesh)
        element(skin,'scale',1.)
        animation = element(actor,'animation',name='walk')
        element(animation,'filename',mesh)
        element(animation,'interpolate_x','false')
        script = element(actor,'script')
        element(script,'auto_start','false')
        element(script,'loop','false')
        trajectory = element(script,'trajectory',id='0',type='walk')
        for t in [0,1]:
            point = element(trajectory,'waypoint')
            element(point,'time',t)
            element(point,'pose',f"{h['x']} {h['y']} 1 0 0 {h['yaw']}")
    ET.indent(tree)
    tree.write(runtime/'warehouse.sdf', encoding='unicode')

def stop(process):
    if process.poll() is not None:
        return
    for sig, timeout in [(signal.SIGINT,8),(signal.SIGTERM,5),(signal.SIGKILL,2)]:
        try:
            os.killpg(process.pid,sig)
            process.wait(timeout)
            return
        except ProcessLookupError:
            return
        except subprocess.TimeoutExpired:
            pass

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--mode',choices=['walking','conversation'],default='walking')
    p.add_argument('--seed',type=int,default=42)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--rviz',action='store_true')
    p.add_argument('--nav',action='store_true')
    a = p.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID','74')
    os.environ.setdefault('GZ_PARTITION','dynamuhsim_socnavgym_adapter')
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS','UDPv4')
    os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    (HERE/'generated').mkdir(exist_ok=True)
    lock=(HERE/'generated'/('ros_domain_'+os.environ['ROS_DOMAIN_ID']+'.lock')).open('a')
    try:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('An adapter session already uses this ROS domain; stop it first')
    runtime = Path(tempfile.mkdtemp(prefix=a.mode+'_',dir=HERE/'generated'))
    config = yaml.safe_load((ROOT/'configs/groups.yaml').read_text())
    config['env'].update(min_dynamic_humans=0,max_dynamic_humans=0,
        min_h_h_dynamic_interactions=int(a.mode=='walking'),max_h_h_dynamic_interactions=int(a.mode=='walking'),
        min_h_h_static_interactions=int(a.mode=='conversation'),max_h_h_static_interactions=int(a.mode=='conversation'),
        set_shape='rectangle',min_map_x=60,max_map_x=60,min_map_y=40,max_map_y=40)
    (runtime/'config.yaml').write_text(yaml.safe_dump(config))
    processes = []
    try:
        with tempfile.TemporaryDirectory(prefix='socnavgym_') as socket_dir:
            socket_path = str(Path(socket_dir)/'worker.sock')
            worker_log = (runtime/'worker.log').open('w')
            worker = subprocess.Popen([str(ROOT/'.venv-socnavgym/bin/python'),str(HERE/'worker.py'),
                '--runtime',str(runtime),'--socket',socket_path,'--mode',a.mode,'--seed',str(a.seed)],
                stdout=worker_log,stderr=subprocess.STDOUT,start_new_session=True)
            processes.append(worker)
            deadline=time.monotonic()+90
            while not (runtime/'ready.json').exists():
                if worker.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError(f'Worker did not become ready: {runtime}/worker.log')
                time.sleep(.1)
            initial = json.loads((runtime/'ready.json').read_text())
            (runtime/'manifest.json').write_text(json.dumps(dict(mode=a.mode,seed=a.seed,
                gui=a.gui,nav_requested=a.nav,dt=.1,world='social_warehouse',
                ros_domain=os.environ['ROS_DOMAIN_ID'],gz_partition=os.environ['GZ_PARTITION'],
                obstacles=initial['obstacles'],native_initial_observation_in_space=initial['native_initial_observation_in_space'],
                native_methods=['compute_orca_interaction_velocities','Human_Human_Interaction.update_speed','handle_collision_and_update'],
                behavior_scope='one persistent group; externally assigned patrol goals; no formation or RL episode/reward loop'),indent=2))
            make_world(runtime,socket_path,initial['humans'],a.mode)
            os.environ['SOCNAVGYM_EXPERIMENT_WORLD']=str(runtime/'warehouse.sdf')
            print(f'Runtime artifacts: {runtime}',flush=True)
            process = subprocess.Popen(['ros2','launch',str(HERE/'warehouse.launch.py'),
                f'gui:={str(a.gui).lower()}',f'rviz:={str(a.rviz).lower()}',f'nav:={str(a.nav).lower()}'],start_new_session=True)
            processes.append(process)
            while process.poll() is None:
                if worker.poll() is not None:
                    raise RuntimeError('Worker exited; experiment requires restart')
                time.sleep(.25)
            return process.returncode
    except KeyboardInterrupt:
        return 130
    finally:
        for process in reversed(processes):
            stop(process)

if __name__=='__main__':
    raise SystemExit(main())
