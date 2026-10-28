#!/usr/bin/env python3
"""Run final corpus01 integrity QA to a fresh output directory, without ROS."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.analysis.perception_corpus import audit_corpus,STAGE,RECORDED_SOURCE_REVISION
from dynamuhsim.analysis.evidence import reserve_audit_directory

def main(argv=None):
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--corpus',type=Path,required=True);p.add_argument('--raw-root',type=Path);p.add_argument('--source-root',type=Path,action='append',default=[]);p.add_argument('--source-revision',default=RECORDED_SOURCE_REVISION);p.add_argument('--output',type=Path,required=True);a=p.parse_args(argv);raw=a.raw_root or a.corpus.parent/'runs'
 output=reserve_audit_directory(a.output,a.corpus,raw,ROOT,STAGE,a.source_root)
 try:
  result,provenance=audit_corpus(a.corpus,repository=ROOT,raw_root=raw,source_roots=a.source_root,source_revision=a.source_revision)
  (output/'audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');(output/'source_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
 except Exception as error:
  (output/'error.json').write_text(json.dumps(dict(status='FAIL',error=str(error)),indent=2)+'\n');raise
 print(json.dumps({k:v for k,v in result.items() if k!='scenes'},indent=2));return 0
if __name__=='__main__':sys.exit(main())
