"""Regression checks for read-only WM auditing and frozen source resolution."""
import hashlib,json,subprocess,sys,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
from dynamuhsim.analysis.wm_dataset import STAGE,_module,audit_dataset,report_directory,source_context

class WMAudit(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);self.repo=self.root/'repo';self.repo.mkdir();self.stage=self.repo/STAGE;self.stage.mkdir(parents=True)
  self.names=[STAGE/'loader.py',STAGE/'targets.py',STAGE/'protocol.json',Path('experiments/social_navigation/benchmark/perception_target_pilot/geometry.py'),Path('experiments/social_navigation/benchmark/perception_target_pilot/sensor.py')]
  self.retired=STAGE/'export.py';self.pins={}
  for name in [*self.names,self.retired]:
   p=self.repo/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text("raise RuntimeError('historical source must never execute')\n");self.pins[str(name)]=hashlib.sha256(p.read_bytes()).hexdigest()
  for name in ['capture_frozen.json','export_frozen.json']:(self.stage/name).write_text(json.dumps(dict(source_sha256=self.pins)))
  def git(*args):return subprocess.check_output(['git','-C',str(self.repo),*args],stderr=subprocess.DEVNULL,text=True).strip()
  git('init','-q');git('add','.');git('-c','user.name=Audit Test','-c','user.email=audit@example.invalid','commit','-qm','fixture');self.revision=git('rev-parse','HEAD');(self.repo/self.retired).unlink()
 def context(self):return source_context(self.repo,self.repo,source_revision=self.revision)
 def test_retired_source_read_from_exact_commit_without_execution(self):
  context=self.context();self.assertEqual(context['export_frozen.json'][str(self.retired)],'recorded_git');self.assertEqual(context['export_frozen.json'][str(self.names[0])],'file')
 def test_changed_active_math_rejected_even_when_git_has_original(self):
  for name in self.names:
   with self.subTest(name=name):
    p=self.repo/name;old=p.read_bytes();p.write_text('changed')
    try:
     with self.assertRaisesRegex(ValueError,'Active dataset math differs'):self.context()
    finally:p.write_bytes(old)
 def test_missing_historical_source_rejected(self):
  receipt=self.stage/'capture_frozen.json';value=json.loads(receipt.read_text());value['source_sha256']['missing.py']='0'*64;receipt.write_text(json.dumps(value))
  with self.assertRaisesRegex(ValueError,'Historical source unavailable: missing.py'):self.context()
 def test_bad_frozen_identity_stops_before_import(self):
  dataset=self.root/'dataset';dataset.mkdir();(dataset/'dataset.json').write_text(json.dumps(dict(source_capture_freeze_sha256='wrong')))
  with patch('dynamuhsim.analysis.wm_dataset._module') as imported:
   with self.assertRaisesRegex(ValueError,'receipt identity'):audit_dataset(dataset,repository=self.repo,raw_root=self.root/'raw',runtime_root=self.repo,work_dir=self.root/'work')
   imported.assert_not_called()
 def test_output_rejects_protected_paths_and_their_ancestors(self):
  dataset=self.root/'dataset';raw=self.root/'raw';runtime=self.root/'runtime'
  for output in [dataset/'new',raw/'new',self.repo/'experiments/archive/new',self.stage/'reports/new',runtime/'experiments/archive/new',runtime/STAGE/'reports/new',self.root]:
   with self.subTest(output=output),self.assertRaises(ValueError):report_directory(output,dataset,raw,self.repo,runtime)
  self.assertFalse(dataset.exists());self.assertFalse(raw.exists());self.assertFalse(runtime.exists())
 def test_output_never_overwrites_and_resolves_symlinks(self):
  dataset=self.root/'dataset';raw=self.root/'raw';output=self.root/'fresh';self.assertEqual(report_directory(output,dataset,raw,self.repo),output)
  with self.assertRaises(FileExistsError):report_directory(output,dataset,raw,self.repo)
  dataset.mkdir();alias=self.root/'alias';alias.symlink_to(dataset,target_is_directory=True)
  with self.assertRaises(ValueError):report_directory(alias/'new',dataset,raw,self.repo)
  self.assertFalse((dataset/'new').exists())
 def test_verified_short_import_ignores_global_module_pollution(self):
  source=self.root/'math.py';source.write_text('from sensor import value\nimport math\nresult=value+math.sqrt(4)\n');verified=types.ModuleType('verified_sensor');verified.value=7;unrelated=types.ModuleType('sensor');unrelated.value=-99
  with patch.dict(sys.modules,{'sensor':unrelated}):
   module=_module(source,'isolated_math',{'sensor':verified});self.assertEqual(module.result,9);self.assertIs(sys.modules['sensor'],unrelated);self.assertNotIn('isolated_math',sys.modules)

if __name__=='__main__':unittest.main()
