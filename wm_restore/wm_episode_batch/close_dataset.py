#!/usr/bin/env python3
"""Final accounting after all10attempts; never alter raw/arrays or accept failed shutdowns."""
import json,shutil
from export import HERE,DATASET,verify,sha,dump
verify();review=json.loads((HERE/'reports/quarantine_review02.json').read_text());meta=json.loads((DATASET/'dataset.json').read_text());plan=json.loads((HERE/'protocol.json').read_text());q={v['episode_id']:v for v in review['quarantined']};assert len(q)==2 and len(meta['episodes'])==8;assert {e['episode_id'] for e in meta['episodes']}=={e['episode_id'] for e in plan['episodes'] if e['episode_id'] not in q};ledger=[]
for e in plan['episodes']:
 raw=HERE/'generated/runs'/e['episode_id'];a=json.loads((raw/'capture_audit.json').read_text());assert a['raw_result_sha256']==sha(raw/'result.json');assert bool(a['infrastructure_pass'])==(e['episode_id'] not in q)
 if e['episode_id'] in q:assert q[e['episode_id']]['raw_result_sha256']==sha(raw/'result.json') and not (DATASET/'episodes'/e['episode_id']).exists()
 ledger.append(a)
meta.update(complete=True,accepted_episodes=8,quarantined_episodes=review['quarantined'],review_amendment_sha256=sha(HERE/'reports/quarantine_review02.json'),test_coverage_limit='Only standing49 accepted; walking49 quarantined, no held-out walking episode in this batch');dump(DATASET/'dataset.json',meta);shutil.copy2(HERE/'reports/quarantine_review02.json',DATASET/'quarantine_review.json');dump(HERE/'reports/dataset_summary.json',meta);dump(HERE/'reports/capture_ledger.json',ledger);print('Final metadata:8accepted+2quarantined, all10attempts accounted; data arrays unchanged')
