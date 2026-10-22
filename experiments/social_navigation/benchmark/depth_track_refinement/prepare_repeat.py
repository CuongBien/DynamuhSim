#!/usr/bin/env python3
"""Copy a closed, navigation-failed but physics-valid recording for perception-only export."""
import argparse,json,shutil
from pathlib import Path
from freeze import sha
HERE=Path(__file__).resolve().parent


def prepare(output):
    source=HERE.parent/'early_avoidance_pilot/generated/runs/avoidance_fixed01'
    result=json.loads((source/'result.json').read_text());audit=json.loads((HERE.parent/'early_avoidance_pilot/reports/avoidance_fixed01/audit.json').read_text())
    assert result['recording']['returncode']==0 and result['gazebo_clean_exit'] and not result['faults']
    assert audit['native_robot_physics_semantics_valid'] and result['actual_goal_reached'] is False
    output.mkdir(parents=True,exist_ok=False);files={}
    for p in [source/'result.json',source/'events.jsonl',*(source/'raw_bag').iterdir()]:
        if not p.is_file():continue
        rel=p.relative_to(source);destination=output/rel;destination.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,destination);assert sha(p)==sha(destination)
        files[str(rel)]=dict(sha256=sha(p),bytes=p.stat().st_size)
    metadata=dict(source=str(source),purpose='Ad hoc matched-depth perception diagnostic only, not a new held-out validation or navigation success',navigation_status='TIMEOUT',actual_goal_reached=False,training_ready=False,copied_files=files,builder='portable_episode/generated/episode01/tools/legacy/build.py')
    (output/'copy_receipt.json').write_text(json.dumps(metadata,indent=2)+'\n');print('Copied closed recording; navigation TIMEOUT retained')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();prepare(a.output.resolve())
