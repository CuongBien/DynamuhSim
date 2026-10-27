#!/usr/bin/env python3
"""Behavioral checks for orphan cleanup and the real hardware OpenGL path."""
import argparse,hashlib,json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.runtime import gpu
from dynamuhsim.runtime.gpu import RuntimeGuard,gpu_preflight,TOKEN
HERE=Path(gpu.__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
output=parser.parse_args().output;output.mkdir(parents=True,exist_ok=False)
(HERE/'generated').mkdir(exist_ok=True)
checks={};records=[]
# Unrelated session must survive, even though it has the same executable name.
unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'],env={k:v for k,v in os.environ.items() if k!=TOKEN})
try:
 with tempfile.TemporaryDirectory(dir=HERE/'generated',prefix='cleanup_test_') as directory:
  runtime=Path(directory);guard=RuntimeGuard(runtime)
  # Child starts a new session, ignores graceful shutdown, while its parent exits.
  script="import subprocess,sys; subprocess.Popen([sys.executable,'-c','import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); signal.signal(signal.SIGTERM,signal.SIG_IGN); time.sleep(60)'],start_new_session=True)"
  parent=subprocess.Popen([sys.executable,'-c',script],start_new_session=True);parent.wait(timeout=5)
  time.sleep(.2);assert parent.poll() is not None and guard.members()
  cleanup=guard.cleanup();assert not cleanup['remaining'] and any(r['signal']=='SIGKILL' for r in cleanup['signals'])
  assert unrelated.poll() is None
  records.append(cleanup);checks['detached_child_after_parent_exit_force_cleanup']='PASS';checks['unrelated_process_preserved']='PASS'
 # Simulate old troubleshooting shell variables. Preflight must remove them and
 # verify a real NVIDIA context, rather than trusting the environment string.
 before=os.environ.copy()
 try:
  os.environ['LIBGL_ALWAYS_SOFTWARE']='1';os.environ['GALLIUM_DRIVER']='llvmpipe';os.environ['LIBGL_ALWAYS_INDIRECT']='1'
  info=gpu_preflight(HERE/'generated');assert info['hardware_verified'] and 'NVIDIA' in info['renderer']
  assert 'LIBGL_ALWAYS_SOFTWARE' not in info['environment_used'] and 'LIBGL_ALWAYS_INDIRECT' not in info['environment_used']
  checks['stale_software_flags_removed_and_actual_nvidia_context']='PASS'
 finally:
  os.environ.clear();os.environ.update(before)
finally:
 unrelated.terminate();unrelated.wait(timeout=5)
report=dict(status='PASS',checks=checks,cleanup_behavior=records,gpu=info,source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [Path(gpu.__file__),HERE/'gl_probe.cpp',Path(__file__)]})
(output/'checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(dict(status='PASS',checks=checks,renderer=info['renderer']),indent=2))
