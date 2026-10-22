#!/usr/bin/env python3
"""Thin adapter calling the unchanged frozen functions on the additional data key."""
import argparse
from pathlib import Path
from replay import run as replay
from evaluate import run as evaluate
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['replay','evaluate']);p.add_argument('--frozen',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--sensor',type=Path);a=p.parse_args()
    if a.mode=='replay':replay(a.frozen.resolve(),'mesh_repeat',a.output.resolve())
    else:
        assert a.sensor is not None;evaluate(a.frozen.resolve(),'mesh_repeat',a.sensor.resolve(),a.output.resolve())
