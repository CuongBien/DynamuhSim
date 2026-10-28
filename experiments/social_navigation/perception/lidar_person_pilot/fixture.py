"""Profile inputs for the shared sensor world assembly."""
import json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=BENCH.parents[2];BASE=BENCH/'depth_visibility_pilot'
sys.path[:0]=[str(REPO/'python'),str(REPO)]
from dynamuhsim.simulation.sensor_fixture import create_fixture
def create(runtime,condition='full'):return create_fixture(runtime,benchmark=BENCH,repo=REPO,lidar=True,condition=condition)
