#!/usr/bin/env python3
"""Compatible worker entrypoint; IPC implementation lives in common_gazebo."""
from pathlib import Path
import runpy

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1]/'common_gazebo/worker.py'),
                  init_globals={'WORKER_VARIANT': 'response'}, run_name='__main__')
