#!/usr/bin/env python3
"""Profile-specific fixture and provenance for the shared bounded sensor capture."""
import argparse,sys
from pathlib import Path
from fixture import create,get_scene,HERE,BENCH,REPO,BASE
sys.path[:0]=[str(REPO/'python'),str(REPO)]
from dynamuhsim.runtime.sensor_capture import run_capture

def run(out,scene_id):
 scene=get_scene(scene_id);condition='full'
 sources=[p for p in HERE.iterdir() if p.is_file()]+[HERE/'generated/build/libcorpus_motion.so',BENCH/'lidar_person_pilot/fixture.py',BASE/'fixture.py',BASE/'visibility_filter.cc',BASE/'generated/build/libvisibility_filter.so',BASE/'snapshot.json']+[BENCH/'camera_height_ab/fixture.py',BENCH/'camera_height_ab/freeze_actors.cc',BENCH/'camera_height_ab/generated/build/libfreeze_actors.so',REPO/'src/custom_corridor/models/mecanum_robot/model.sdf',BENCH.parent/'gazebo_adapter/run.py',BENCH.parent/'warehouse/warehouse.sdf']
 return run_capture(out,here=HERE,repo=REPO,benchmark=BENCH,sources=sources,
                    create_world=lambda runtime:create(runtime,scene_id),protocol_name='capture_protocol.json',condition=condition,scene_id=scene_id,scene=scene)
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--scene',required=True);a=p.parse_args();run(a.output.resolve(),a.scene)
