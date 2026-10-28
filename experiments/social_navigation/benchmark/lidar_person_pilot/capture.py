#!/usr/bin/env python3
"""Profile-specific fixture and provenance for the shared bounded sensor capture."""
import argparse,sys
from pathlib import Path
from fixture import create,HERE,BENCH,REPO,BASE
sys.path[:0]=[str(REPO/'python'),str(REPO)]
from dynamuhsim.runtime.sensor_capture import run_capture

def run(out,condition):
 sources=[p for p in HERE.iterdir() if p.is_file()]+[BASE/'fixture.py',BASE/'visibility_filter.cc',BASE/'generated/build/libvisibility_filter.so',BASE/'snapshot.json']+[BENCH/'camera_height_ab/fixture.py',BENCH/'camera_height_ab/freeze_actors.cc',BENCH/'camera_height_ab/generated/build/libfreeze_actors.so',REPO/'src/custom_corridor/models/mecanum_robot/model.sdf',BENCH.parent/'gazebo_adapter/run.py',BENCH.parent/'warehouse/warehouse.sdf']
 return run_capture(out,here=HERE,repo=REPO,benchmark=BENCH,sources=sources,
                    create_world=lambda runtime:create(runtime,condition),protocol_name='protocol.json',condition=condition)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--condition',choices=['full','null'],required=True);a=p.parse_args();run(a.output.resolve(),a.condition)
