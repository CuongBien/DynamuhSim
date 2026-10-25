#!/usr/bin/env python3
"""Export a new immutable control bundle from committed Git objects only."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.controls.bundle import export_bundle


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ref',required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    print(json.dumps(export_bundle(ROOT,a.ref,a.output),sort_keys=True))


if __name__=='__main__':main()
