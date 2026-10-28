"""Corpus QA rejects changed live contracts and isolates frozen math imports."""
import hashlib,json,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
from dynamuhsim.analysis.perception_corpus import STAGE,audit_corpus
from dynamuhsim.perception.corpus import corpus_math
ROOT=Path(__file__).resolve().parents[1]

class CorpusAudit(unittest.TestCase):
 def test_receipt_mismatch_stops_before_loading_code(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);stage=root/STAGE;stage.mkdir(parents=True);(stage/'frozen.json').write_text('{}');dataset=root/'data';dataset.mkdir();(dataset/'manifest.json').write_text(json.dumps(dict(source_frozen_sha256='wrong')))
   with patch('dynamuhsim.analysis.perception_corpus.module_from_path') as imported:
    with self.assertRaisesRegex(ValueError,'receipt identity'):audit_corpus(dataset,repository=root,raw_root=root/'raw')
    imported.assert_not_called()
 def test_historical_bytes_do_not_authorize_changed_live_loader(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);stage=root/STAGE;stage.mkdir(parents=True);name=str(STAGE/'loader.py');(stage/'loader.py').write_text('changed live reader');(stage/'frozen.json').write_text(json.dumps(dict(source_sha256={name:'0'*64})));dataset=root/'data';dataset.mkdir();(dataset/'manifest.json').write_text(json.dumps(dict(source_frozen_sha256=hashlib.sha256((stage/'frozen.json').read_bytes()).hexdigest())))
   with patch('dynamuhsim.analysis.perception_corpus.resolve_sources',return_value=({name:dict(origin='recorded_git')},[])),patch('dynamuhsim.analysis.perception_corpus.module_from_path') as imported:
    with self.assertRaisesRegex(ValueError,'Active corpus contract differs'):audit_corpus(dataset,repository=root,raw_root=root/'raw')
    imported.assert_not_called()
 def test_legacy_short_names_cannot_replace_verified_label_math(self):
  import numpy as np
  polluted=types.ModuleType('labels');polluted.ownership=lambda *a:(_ for _ in ()).throw(AssertionError('wrong module'))
  with patch.dict(sys.modules,{'labels':polluted,'sensor':polluted}):
   math=corpus_math(ROOT);a=np.array([[2.]])
   owner,qa=math.label_pixels(a,[a]*6,[np.full((1,1),np.inf)]*6,np.array([[4.]]));self.assertEqual(owner.item(),-1);self.assertEqual(qa['null_effect_without_owner'],1);self.assertIs(sys.modules['labels'],polluted)
if __name__=='__main__':unittest.main()
