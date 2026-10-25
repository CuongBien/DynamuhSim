#!/usr/bin/env python3
"""Hydrate selected historical report trees and their known static dependencies."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.analysis.evidence import load_json
from dynamuhsim.artifacts.report_plan import hydrate_reports,validate_report_plan,dependency_closure


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkout',type=Path,default=ROOT)
    p.add_argument('--index',type=Path,default=ROOT/'docs/artifacts/history_2026-10-25.json')
    p.add_argument('--plan',type=Path,default=ROOT/'docs/artifacts/report_dependency_plan_2026-10-25.json')
    p.add_argument('--media-plan',type=Path,help='Approved media policy; auto-selected for the bundled index/plan only')
    choices=p.add_mutually_exclusive_group(required=True)
    choices.add_argument('--group',action='append');choices.add_argument('--all-externalized',action='store_true')
    p.add_argument('--source',choices=['git','archive'],default='git');p.add_argument('--store',type=Path)
    p.add_argument('--dry-run',action='store_true');p.add_argument('--report',type=Path,required=True)
    a=p.parse_args()
    if a.report.exists() or a.report.is_symlink():raise FileExistsError('Preserve existing report')
    index,plan=load_json(a.index),load_json(a.plan)
    groups=validate_report_plan(index,plan)
    selected=a.group or [name for name,row in groups.items() if row['action']=='externalize_whole_group']
    if a.dry_run:
        report=dict(status='PLANNED',requested=selected,dependency_closure=dependency_closure(groups,selected),changed_groups=0)
    else:
        media_path=a.media_plan
        if media_path is None and a.index.resolve()==ROOT/'docs/artifacts/history_2026-10-25.json' and a.plan.resolve()==ROOT/'docs/artifacts/report_dependency_plan_2026-10-25.json':
            candidate=ROOT/'docs/artifacts/report_media_plan_2026-10-25.json'
            if candidate.exists():media_path=candidate
        report=hydrate_reports(a.checkout,index,plan,selected,source=a.source,store=a.store,
                               media_plan=load_json(media_path) if media_path is not None else None)
    a.report.parent.mkdir(parents=True,exist_ok=True)
    with a.report.open('x') as f:f.write(json.dumps(report,sort_keys=True,indent=2)+'\n')
    print(report['status'],len(report['dependency_closure']),'groups; changed',report['changed_groups'])


if __name__=='__main__':main()
