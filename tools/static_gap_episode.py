#!/usr/bin/env python3
"""Isolated static bottleneck test; default ROS launch and social captures are unchanged."""
import argparse,sys
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.runtime.static_gap import prepare,StaticGapCapture
from dynamuhsim.runtime.capture import verify_run
from dynamuhsim.runtime.nav2_trial import run_trial
from dynamuhsim.runtime.profiles import PROFILES

def main():
 p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='mode',required=True)
 for name in ['prepare','run','verify']:
  cmd=sub.add_parser(name);cmd.add_argument('--run-dir',type=Path,required=True)
  if name=='prepare':cmd.add_argument('--case',choices=['wide','narrow'],required=True)
 a=p.parse_args()
 if a.mode=='prepare':prepare(ROOT,a.run_dir,a.case);print('Prepared',a.run_dir);return 0
 verify_run(ROOT,a.run_dir)
 if a.mode=='verify':print('PASS: static gap freeze unchanged');return 0
 capture=StaticGapCapture(ROOT,a.run_dir);args=SimpleNamespace(condition='empty',gui=False,report_dir=capture.run_dir/'episode')
 return run_trial(PROFILES['socnavgym_nav2_pilot'],args,ROOT,Path(__file__).resolve(),capture=capture)
if __name__=='__main__':raise SystemExit(main())
