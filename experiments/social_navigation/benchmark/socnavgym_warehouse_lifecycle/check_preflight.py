"""Refuse launching measured code that differs from the completed offline probe."""
import hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent
protocol=json.loads((HERE/'reports/offline02/protocol.json').read_text())
for name,digest in protocol['source_sha256'].items():
    p=Path(name)
    if not p.exists():p=HERE.parent.parent.parent.parent/Path(name).relative_to('/home/cuongbien/DynamuhSim')
    assert hashlib.sha256(p.read_bytes()).hexdigest()==digest,('Offline preflight source changed; run and validate a new preflight',name)
summary=json.loads((HERE/'reports/offline02/summary.json').read_text());assert len(summary)==3 and all(r['status']=='PASS' for r in summary)
print('PASS: offline preflight and current source match')
