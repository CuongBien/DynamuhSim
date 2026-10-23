"""Frozen source and corpus binding, without interpreting sealed test contents."""
import hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def verify():
 frozen=json.loads((HERE/'frozen.json').read_text());assert all(sha(REPO/n)==h for n,h in frozen['source_sha256'].items()),'Frozen source changed';root=REPO/frozen['corpus_path'];assert {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file()}==set(frozen['corpus_sha256']);assert all(sha(root/n)==h for n,h in frozen['corpus_sha256'].items()),'Original corpus changed';return frozen
