#!/usr/bin/env python3
"""TASK03: export only verified TASK01 modality banks; no simulation or old dataset writes."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.datasets.single_episode_export import export_episode,correct_export_contract


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit',type=Path,help='Legacy raw-bank creation only')
    parser.add_argument('--base-export',type=Path,help='Existing verified banks; recompute factual eligibility')
    parser.add_argument('--recovery-dir',type=Path,help='Existing command provenance receipt; no perception is run')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    if args.output.exists():parser.error('Output exists; choose a fresh export directory')
    if args.base_export:
        if not args.recovery_dir:parser.error('--base-export requires --recovery-dir')
        report=correct_export_contract(args.base_export,args.recovery_dir,args.output)
        print(json.dumps(report['eligible_windows'],indent=2));return 0
    if not args.audit:parser.error('Provide --base-export/--recovery-dir or --audit for legacy raw banks')
    report=export_episode(args.audit,args.output)
    print(json.dumps({k:report[k] for k in ['status','windows_valid','windows_invalid','rejection_counts_nonexclusive','example_sample_id','training_ready']},indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
