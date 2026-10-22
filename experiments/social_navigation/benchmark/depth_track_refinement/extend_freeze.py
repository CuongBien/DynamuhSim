#!/usr/bin/env python3
"""Add a post-validation diagnostic dataset without changing the frozen inference engine."""
import argparse,datetime,json
from pathlib import Path
from freeze import verify,sha


def extend(source,normalized,output):
    obj=verify(source);assert not output.exists()
    obj['extension']=dict(parent_frozen_sha256=sha(source),created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        purpose='Post-validation matched-mesh diagnostic; not a replacement for failed proxy-on validation or a held-out test',
        engine_and_config_unchanged=True,script_sha256=sha(Path(__file__)))
    obj['datasets']['mesh_repeat']=dict(normalized=str(normalized.resolve()),input_sha256={n:sha(normalized/n) for n in ['depth.npy','observations.npz','calibration.json','samples.npz']},scene_sha256=sha(normalized/'human_states.jsonl'),manifest_sha256=sha(normalized/'manifest.json'),
        source_episode='avoidance_fixed01',source_navigation_status='TIMEOUT',actual_goal_reached=False,
        evaluation_role='Same-family perception diagnostic only, not training or navigation acceptance')
    output.write_text(json.dumps(obj,indent=2)+'\n');print('Extended data declaration; inference and config remain byte-identical')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--normalized',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();extend(a.source.resolve(),a.normalized.resolve(),a.output.resolve())
