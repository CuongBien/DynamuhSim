#!/usr/bin/env python3
"""Plan, check or hydrate report media without replacing controls or model/test state."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.analysis.evidence import load_json
from dynamuhsim.artifacts.report_media import create_media_plan,hydrate_report_media


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['plan','check','hydrate'])
    p.add_argument('--index',type=Path,default=ROOT/'docs/artifacts/history_2026-10-25.json')
    p.add_argument('--dependency-plan',type=Path,default=ROOT/'docs/artifacts/report_dependency_plan_2026-10-25.json')
    p.add_argument('--media-plan',type=Path,default=ROOT/'docs/artifacts/report_media_plan_2026-10-25.json')
    p.add_argument('--ref',default='HEAD');p.add_argument('--checkout',type=Path,default=ROOT)
    select=p.add_mutually_exclusive_group();select.add_argument('--group',action='append');select.add_argument('--all',action='store_true')
    p.add_argument('--source',choices=['git','archive'],default='git');p.add_argument('--store',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists() or a.output.is_symlink():raise FileExistsError('Preserve existing output')
    index=load_json(a.index)
    if a.command=='plan':report=create_media_plan(ROOT,index,load_json(a.dependency_plan),a.ref)
    else:
        if not a.all and not a.group:p.error('Choose --group or --all')
        plan=load_json(a.media_plan);selected=a.group or list(plan['groups'])
        report=hydrate_report_media(a.checkout,index,plan,selected,source=a.source,store=a.store,check=a.command=='check')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:f.write(json.dumps(report,sort_keys=True,indent=2)+'\n')
    print(report.get('status','PLAN_CREATED'),report.get('changed_files',report.get('files')),'files')
    return 1 if report.get('status')=='MEDIA_MISSING' else 0


if __name__=='__main__':raise SystemExit(main())
