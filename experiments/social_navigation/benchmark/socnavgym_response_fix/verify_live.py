#!/usr/bin/env python3
"""Record this pilot with the shared bounded velocity runner."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'python'))
from dynamuhsim.runtime.velocity_trial import main
if __name__=='__main__':
    raise SystemExit(main('socnavgym_response_fix',Path(__file__)))
