#!/usr/bin/env python3
"""Build a reviewable runtime copy of the accepted dataset pilot, then run it once."""
import difflib,hashlib,json,os,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;OLD=BENCH/'dataset_pilot'

def build_driver():
    before=(OLD/'trial.py').read_text();after=before
    edits={
      'from prepare import prepare_camera':'from motion_prepare import prepare_camera,add_filter\nfrom pair_recording import Pairs\nfrom quiesce import stop_paused\noriginal_stop=stop_server\ndef stop_server(guard):return stop_paused(guard,original_stop)',
      'HERE=Path(__file__).resolve().parent;':f'HERE=Path({str(OLD)!r});',
      'recording=None;worker=None;':'recording=None;pairs=None;worker=None;',
      'ET.indent(tree);tree.write(runtime/\'warehouse.sdf\',encoding=\'unicode\');config=':'add_filter(world);ET.indent(tree);tree.write(runtime/\'warehouse.sdf\',encoding=\'unicode\');config=',
      'recording=Recording(node,out)':'recording=Recording(node,out);pairs=Pairs(node,out,lambda:phase)',
      'wait(recording.ready,30)':'wait(lambda:recording.ready() and pairs.ready(),30)',
      '    if recording:\n':'    if pairs:result[\'depth_pairs\']=pairs.close()\n    if recording:\n',
      "result['counts']=dict(counts);":"result['counts']=dict(counts);result['experiment']='Depth-only proxy motion pilot, native camera height, no default changes';",
    }
    for needle,replacement in edits.items():
        assert after.count(needle)==1,(needle,after.count(needle));after=after.replace(needle,replacement)
    # Bridge only the extra diagnostic camera. Native topics and robot commands stay on old launch.
    needle="launch=subprocess.Popen(['ros2','launch'"
    insert="""pair_log=(out/'pair_bridge.log').open('w');files.append(pair_log);processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge','/proxy_check/on/depth@sensor_msgs/msg/Image[gz.msgs.Image'],stdout=pair_log,stderr=subprocess.STDOUT,start_new_session=True,env=launch_env));launch=subprocess.Popen(['ros2','launch'"""
    assert after.count(needle)==1;after=after.replace(needle,insert)
    target=HERE/'generated/motion_driver.py';target.parent.mkdir(exist_ok=True);target.write_text(after)
    (HERE/'generated/motion_driver.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='dataset_pilot/trial.py',tofile='runtime motion driver')))
    compile(after,str(target),'exec');return target

if __name__=='__main__':
    assert json.loads((HERE/'generated/runs/paused01/capture.json').read_text())['status']=='PASS'
    target=build_driver();out=HERE/'generated/runs/motion01'
    env=os.environ.copy();env['PYTHONPATH']=str(HERE)+os.pathsep+str(OLD)+os.pathsep+env.get('PYTHONPATH','')
    cmd=[sys.executable,str(target),'--scenario','standing','--seed','42','--report-dir',str(out)]
    print('One isolated standingv3 + Nav2 motion pilot; additional ON/OFF diagnostics',flush=True)
    result=subprocess.run(cmd,env=env)
    raise SystemExit(result.returncode)
