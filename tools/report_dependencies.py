#!/usr/bin/env python3
"""Classify archived report groups from committed source without executing it."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.artifacts.report_plan import create_report_plan
from dynamuhsim.analysis.evidence import load_json


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--index',type=Path,default=ROOT/'docs/artifacts/history_2026-10-25.json')
    p.add_argument('--ref',default='HEAD');p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists() or a.output.is_symlink():raise FileExistsError('Preserve existing plan')
    plan=create_report_plan(ROOT,load_json(a.index),a.ref)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:f.write(json.dumps(plan,sort_keys=True,indent=2)+'\n')
    groups=[r for r in plan['groups'].values() if r['action']=='externalize_whole_group']
    print('PLAN',len(groups),'groups',sum(r['files'] for r in groups),'files',sum(r['logical_bytes'] for r in groups),'bytes')


if __name__=='__main__':main()
