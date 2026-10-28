#!/usr/bin/env python3
"""Compatibility entrypoint for the maintained isolated episode probe."""
import runpy
from pathlib import Path
if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[4] / 'tools/probe_portable_episode.py'), run_name='__main__')
