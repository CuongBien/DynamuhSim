"""Compatibility import for the shared float32 ON/OFF recorder."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'python'))
from dynamuhsim.runtime.capture_recording import Pairs
