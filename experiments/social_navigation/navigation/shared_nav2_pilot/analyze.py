#!/usr/bin/env python3
"""Audit this pilot through the shared Nav2 analyzer."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'python'))
from dynamuhsim.analysis.nav2 import main

if __name__=='__main__':
    raise SystemExit(main(Path(__file__).resolve().parent,'container'))
