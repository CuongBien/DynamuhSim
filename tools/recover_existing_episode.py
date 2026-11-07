#!/usr/bin/python3
"""Verify existing command/schema evidence only. No perception or simulator."""
import argparse
import json
import os
from pathlib import Path
import sys

os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
os.environ.setdefault('OMP_NUM_THREADS','1')
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.datasets.episode_recovery import recover_episode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit',type=Path,required=True)
    p.add_argument('--base-export',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if sys.version_info[:2]!=(3,12):
        p.error('ROS 2 Jazzy requires Python 3.12 here. Use /usr/bin/python3 tools/recover_existing_episode.py (current: '+sys.executable+')')
    try:
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        p.error('Source /opt/ros/jazzy/setup.bash, then use /usr/bin/python3. ROS import failed: '+str(exc))
    if a.output.exists():p.error('Preserve existing artifacts; choose a NEW directory')
    r=recover_episode(a.audit,a.base_export,a.output,ROOT)
    print(json.dumps(r,indent=2))


if __name__=='__main__':main()
