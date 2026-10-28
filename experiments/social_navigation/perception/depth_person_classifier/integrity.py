"""Frozen math/corpus binding with explicit legacy-to-current source locations."""
import hashlib,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[3]
sys.path.insert(0,str(REPO/'python'))
from dynamuhsim.workspace import source_path
from dynamuhsim.analysis.evidence import verify_sources
ORIGINAL_VERIFIER_REVISION='5f3f7cdc9a99208f806d5bea2575418a61cdc7af'
ORIGINAL_RUNNER_REVISION='9833565fa546d55cb0e553f8388bf4a703c59583'
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def verify(corpus_root=None):
 frozen=json.loads((HERE/'frozen.json').read_text())
 for name,digest in frozen['source_sha256'].items():
  current=source_path(REPO,name)
  if current==Path(__file__).resolve() or name=='experiments/social_navigation/benchmark/depth_person_classifier/run.sh':
   # Original verifier/runner bootstraps are provenance; all model/data/math files remain exact.
   revision=ORIGINAL_VERIFIER_REVISION if current==Path(__file__).resolve() else ORIGINAL_RUNNER_REVISION
   verify_sources({name:digest},repository=REPO,revision=revision)
  else:assert sha(current)==digest,'Frozen executable source changed: '+name
 root=Path(corpus_root) if corpus_root is not None else source_path(REPO,frozen['corpus_path'])
 assert {str(p.relative_to(root)) for p in root.rglob('*') if p.is_file()}==set(frozen['corpus_sha256'])
 assert all(sha(root/name)==digest for name,digest in frozen['corpus_sha256'].items()),'Original corpus changed'
 return frozen
