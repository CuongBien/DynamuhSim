"""Compatibility entry point; one shared MCAP recorder implementation."""
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[3]/'python'))
from dynamuhsim.runtime.capture_recording import Recording as _Recording,TOPICS

class Recording(_Recording):
    def __init__(self,node,output):
        super().__init__(node,output,config_dir=HERE)
