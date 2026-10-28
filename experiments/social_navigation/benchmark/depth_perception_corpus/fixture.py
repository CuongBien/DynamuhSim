"""Profile inputs for the shared sensor world assembly."""
import json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];BASE=BENCH/'depth_visibility_pilot'
sys.path[:0]=[str(REPO/'python'),str(REPO)]
from dynamuhsim.simulation.sensor_fixture import create_fixture
def get_scene(scene_id):return next(v for v in json.loads((HERE/'scenes_LABEL_ONLY.json').read_text()) if v['scene_id']==scene_id)
def create(runtime,scene_id):return create_fixture(runtime,benchmark=BENCH,repo=REPO,lidar=True,motion=get_scene(scene_id),motion_library=HERE/'generated/build/libcorpus_motion.so')
