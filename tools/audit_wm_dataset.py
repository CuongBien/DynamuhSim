#!/usr/bin/env python3
"""Audit the closed WM cohort without writing to its dataset or historical reports."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.workspace import stage_directory
from dynamuhsim.analysis.wm_dataset import audit_dataset,report_directory,RECORDED_SOURCE_REVISION,STAGE

def main(argv=None):
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,default=stage_directory(ROOT,'wm_episode_batch')/'generated/dataset01');p.add_argument('--raw-root',type=Path);p.add_argument('--runtime-root',type=Path,default=ROOT);p.add_argument('--source-root',type=Path,action='append',default=[]);p.add_argument('--source-revision',default=RECORDED_SOURCE_REVISION);p.add_argument('--output',type=Path,required=True);args=p.parse_args(argv)
 raw=args.raw_root or args.dataset.parent/'runs'
 output=report_directory(args.output,args.dataset,raw,ROOT,args.runtime_root)
 try:
  result,provenance=audit_dataset(args.dataset,repository=ROOT,raw_root=raw,runtime_root=args.runtime_root,work_dir=output/'work',source_roots=args.source_root,source_revision=args.source_revision)
  (output/'audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');(output/'source_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
 except Exception as error:
  (output/'error.json').write_text(json.dumps(dict(status='FAIL',error=str(error)),indent=2)+'\n');raise
 print(json.dumps({k:v for k,v in result.items() if k not in ['episodes','raw_result_sha256']},indent=2));return 0

if __name__=='__main__':sys.exit(main())
