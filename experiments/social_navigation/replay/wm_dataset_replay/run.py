#!/usr/bin/env python3
"""Isolated Gazebo viewer for recorded train episode XY/yaw, no live simulator policy."""
from pathlib import Path
import argparse,os,subprocess,json,signal,time,uuid,copy,xml.etree.ElementTree as E
HERE=Path(__file__).resolve().parent
BATCH=HERE.parent.parent/'datasets/wm_episode_batch'
def make_world(out, raw=None, runtime=None):
 raw=Path(raw) if raw is not None else BATCH/'generated/runs/wm_standing_45';runtime=Path(runtime) if runtime is not None else BATCH/'generated/runtime/wm_standing_45'
 root=E.parse(runtime/'warehouse.sdf').getroot();world=root.find('world');world.set('name','wm_recorded_replay')
 for plugin in list(world.findall('plugin')):
  if plugin.get('name') not in ['gz::sim::systems::SceneBroadcaster','gz::sim::systems::UserCommands']:world.remove(plugin)
 for m in list(world.findall('model')):
  if m.get('name')=='common_observer':world.remove(m);continue
  for link in m.findall('link'):
   for sensor in list(link.findall('sensor')):link.remove(sensor)
  if m.get('name','').endswith('_proxy'):
   for link in m.findall('link'):
    for v in link.findall('visual'):link.remove(v)
 robot=E.parse(runtime/'mecanum_camera.sdf').getroot().find('model')
 for plugin in list(robot.findall('plugin')):robot.remove(plugin)
 for link in robot.findall('link'):
  for sensor in list(link.findall('sensor')):link.remove(sensor)
 robot.find('pose').text='-28 0 0.04 0 0 0';world.append(robot)
 # Direct pose updates, no Physics system: no contacts, live Nav2, workers or sensor regeneration.
 physics=world.find('physics');physics.find('real_time_factor').text='1.0'
 gui=world.find('gui');minimal=next(x for x in gui.findall('plugin') if x.get('filename')=='MinimalScene')
 minimal.find('camera_pose').text='-29 -7 8 0 0.75 0.8'
 plugin=E.SubElement(world,'plugin',{'filename':str(HERE/'generated/build/libwm_recorded_replay.so'),'name':'wm_recorded::Replay'})
 E.SubElement(plugin,'poses').text=str(raw/'gazebo_poses.jsonl')
 E.indent(root);E.ElementTree(root).write(out/'replay.sdf',encoding='unicode',xml_declaration=True)
 (out/'gui.config').write_text('<window><width>1200</width><height>850</height><dialog_on_exit>false</dialog_on_exit></window>\n'+'\n'.join(E.tostring(x,encoding='unicode') for x in gui.findall('plugin')))
 return raw

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--raw-root',type=Path,help='Recorded train episode directory');parser.add_argument('--runtime-root',type=Path,help='Matching recorded world/robot directory');parser.add_argument('--headless',action='store_true');parser.add_argument('--gui-platform',choices=['xcb','wayland'],default='xcb');parser.add_argument('--seconds',type=float,default=1800);args=parser.parse_args()
 out=HERE/'generated'/('view_'+time.strftime('%Y%m%dT%H%M%S'));out.mkdir(exist_ok=False)
 raw=make_world(out,args.raw_root,args.runtime_root);env=os.environ.copy();env.update(GZ_PARTITION='wm_replay_'+uuid.uuid4().hex[:10],GALLIUM_DRIVER='d3d12',MESA_D3D12_DEFAULT_ADAPTER_NAME='NVIDIA',QT_QPA_PLATFORM='xcb',DISPLAY=':0',WAYLAND_DISPLAY='wayland-0',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
 for key in ['LIBGL_ALWAYS_SOFTWARE','LIBGL_ALWAYS_INDIRECT','MESA_LOADER_DRIVER_OVERRIDE','QT_XCB_GL_INTEGRATION','QT_QUICK_BACKEND','QT_QPA_PLATFORM_PLUGIN_PATH']:env.pop(key,None)
 env['LD_PRELOAD']='/usr/lib/wsl/lib/libd3d12core.so'
 env['QSG_RENDER_LOOP']='basic'
 probe=HERE.parents[3]/'python/dynamuhsim/runtime/generated/gl_probe'
 checked=subprocess.run([str(probe)],env=env,capture_output=True,text=True,timeout=15)
 (out/'gpu_preflight.json').write_text(json.dumps({'returncode':checked.returncode,'stdout':checked.stdout,'stderr':checked.stderr},indent=2)+'\n')
 if checked.returncode or 'NVIDIA' not in checked.stdout:raise RuntimeError('Native GPU preflight failed: '+checked.stderr)
 resources=[str(HERE.parents[3]/'src/custom_corridor/models'),env.get('GZ_SIM_RESOURCE_PATH','')];env['GZ_SIM_RESOURCE_PATH']=':'.join(resources)
 cmd=['gz','sim','-s','-r','-v','3',str(out/'replay.sdf')]
 log=(out/'server.log').open('w');server=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 session={'mode':'recorded_xy_yaw_visualization','episode':raw.name if raw.name!='episode' else raw.parent.parent.name,'source':str(raw/'gazebo_poses.jsonl'),'partition':env['GZ_PARTITION'],'server_pid':server.pid,'gui_pid':None,'gui_platform':args.gui_platform,'output':str(out),'gpu_requested':'NVIDIA D3D12','physics':False,'live_nav2':False,'sensors_regenerated':False,'limitations':'Logged XY/yaw interpolated in a copied factory scene. Robot height/roll/pitch and skeletal animation were not recorded; fixed for display. This is not exact RGB replay or physics re-execution. Sensor MCAP replay is separate.'}
 gui=None
 try:
  time.sleep(3)
  if server.poll() is not None:raise RuntimeError('Replay server exited: '+str(server.returncode))
  if not args.headless:
   glog=(out/'gui.log').open('w');gui_env=env.copy();gui_env['QT_QPA_PLATFORM']=args.gui_platform;gui=subprocess.Popen(['gz','sim','-g','-v','3','--gui-config',str(out/'gui.config')],env=gui_env,stdout=glog,stderr=subprocess.STDOUT,start_new_session=True);session['gui_pid']=gui.pid
  (out/'session.json').write_text(json.dumps(session,indent=2)+'\n');(HERE/'generated/latest.json').write_text(json.dumps(session,indent=2)+'\n');print(json.dumps(session,indent=2),flush=True)
  start=time.monotonic()
  while time.monotonic()-start<args.seconds and server.poll() is None:
   if gui is not None and gui.poll() is not None:break
   time.sleep(.5)
 finally:
  for proc in [gui,server]:
   if proc is not None and proc.poll() is None:
    os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=10)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
  session['server_returncode']=server.returncode;session['gui_returncode']=gui.returncode if gui else None;(out/'closed.json').write_text(json.dumps(session,indent=2)+'\n')
if __name__=='__main__':main()
