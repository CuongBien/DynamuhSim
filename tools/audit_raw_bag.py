#!/usr/bin/env python3
"""Audit an existing TASK01 raw bag; never launch simulation or modify inputs."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.analysis.raw_bag import choose_run,read_raw,analyze,write_outputs,digest


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True,help='TASK01 parent or exact successful run directory')
    parser.add_argument('--output',type=Path,required=True,help='NEW audit directory; never overwritten')
    args=parser.parse_args(argv)
    if args.output.exists():parser.error('Preserve existing audit; choose a new output directory')
    run=choose_run(args.input);args.output.mkdir(parents=True,exist_ok=False)
    inputs=[run/'manifest.json',run/'episode/raw_bag/metadata.yaml',*sorted((run/'episode/raw_bag').glob('*.mcap'))]
    inputs += [run/p for p in ['capture_freeze.json','plan.json','episode/result.json','episode/recorder.log',
                'runtime/nav2_runtime.yaml','runtime/warehouse.sdf','runtime/mecanum_camera.sdf','runtime/mecanum_aligned.urdf'] if (run/p).exists()]
    before={str(p):digest(p) for p in inputs}
    try:
        raw=read_raw(run);report,times,speed,tracks,raw=analyze(run,raw)
        report['input_sha256']=before;report['audit_tool_sha256']=digest(Path(__file__))
        report['input_unchanged']=all(digest(Path(p))==sha for p,sha in before.items())
        write_outputs(args.output,report,times,speed,tracks,raw)
    except Exception as error:
        report=dict(verdict='FAIL',run_dir=str(run),error=repr(error),input_sha256=before,
                    task01_fix='Inspect exact raw file/chunk/record location in the error. Original input preserved; no replacement run.')
        path=args.output/'raw_audit.json'
        if not path.exists():path.write_text(json.dumps(report,indent=2)+'\n')
        for name in ['topic_statistics.csv','time_statistics.csv']:
            path=args.output/name
            if not path.exists():path.write_text('status,measurement\nFAIL,NOT_MEASURED\n')
        path=args.output/'first_episode_report.md'
        if not path.exists():path.write_text('# TASK02 — FAIL\n\nNo statistics are assumed for unreadable input.\n\n```text\n'+str(error)+'\n```\n\nOriginal input preserved. Do not rerun simulation in TASK02.\n')
        print(json.dumps(report,indent=2));return 1
    print(report['verdict'],args.output.resolve())
    return 1 if report['verdict']=='FAIL' else 0


if __name__=='__main__':raise SystemExit(main())
