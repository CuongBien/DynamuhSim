"""Handoff entrypoint must preserve artifacts and keep checks free of simulator runs."""
from contextlib import redirect_stdout
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('handoff_cli',ROOT/'tools/handoff.py');cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)


def pilot_for(path):
 return dict(archive_bytes=path.stat().st_size,archive_sha256=cli.sha(path),top_directory='handoff_data_pilot01')


class HandoffToolsTests(unittest.TestCase):
 def test_baseline_reports_source_tampering(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);(root/'configs/handoff').mkdir(parents=True);source=root/'module.py';source.write_text('accepted');manifest=dict(reference_commit='baseline',protected_sha256={'module.py':cli.sha(source)});(root/'configs/handoff/baseline.json').write_text(json.dumps(manifest))
   self.assertEqual(cli.verify_baseline(root)['status'],'PASS');source.write_text('changed');report=cli.verify_baseline(root);self.assertEqual(report['status'],'FAIL');self.assertEqual(report['failures'][0]['reason'],'HASH_MISMATCH')

 def test_data_doctor_does_not_probe_ros_gpu_or_install(self):
  with patch.object(cli,'probe_modules',return_value=[dict(module='numpy',status='PASS',version='test')]) as probe,patch.object(cli.subprocess,'run') as shell:
   result=cli.doctor('data')
  self.assertEqual(result['status'],'PASS');self.assertEqual(probe.call_args.args[1],['numpy']);shell.assert_not_called();self.assertFalse(result['auto_install']);self.assertEqual(result['actual_rendering_test'],'NOT_RUN')

 def test_missing_dependency_is_not_success(self):
  with patch.object(cli,'probe_modules',return_value=[dict(module='numpy',status='FAIL',error='missing')]):result=cli.doctor('data')
  self.assertEqual(result['status'],'FAIL')

 def test_reference_uses_frozen_v7_flags_without_spawning(self):
  allowed=ROOT/'experiments/social_navigation/benchmark/generated/handoff_reference/unit_not_created'
  with patch.object(cli.subprocess,'run') as shell:
   result=cli.reference_commands('conversation',allowed,46)
  shell.assert_not_called();self.assertFalse(allowed.exists());self.assertFalse(result['simulator_started']);prepare=result['commands'][0];self.assertIn('persistent_conversation_v1',prepare);self.assertEqual(prepare[prepare.index('--scenario')+1],'standing');self.assertTrue(prepare[prepare.index('--config')+1].endswith('geometry_v7_mppi_resample.json'))
  walking=cli.reference_commands('natural_group_walking',allowed,46)['commands'][0];self.assertIn('group_cohesion_v2',walking);self.assertEqual(walking[walking.index('--human-lifecycle-profile')+1],'native')

 def test_reference_rejects_protected_output_and_existing_run(self):
  with self.assertRaises(ValueError):cli.reference_commands('conversation',ROOT/'experiments/social_navigation/benchmark/wm_episode_batch/generated/dataset01',46)
  with patch.object(Path,'exists',return_value=True):
   with self.assertRaises(ValueError):cli.reference_commands('conversation',ROOT/'experiments/social_navigation/benchmark/generated/handoff_reference/existing',46)

 def test_archive_paths_and_symlinks_rejected_before_restore(self):
  for name,symlink in [('../escape',False),('/absolute',False),('handoff_data_pilot01/../escape',False),('handoff_data_pilot01/evil',True)]:
   with self.subTest(name=name),tempfile.TemporaryDirectory() as directory:
    root=Path(directory);archive=root/'data.zip';work=root/'restore';info=zipfile.ZipInfo(name)
    if symlink:info.external_attr=0o120777<<16
    with zipfile.ZipFile(archive,'w') as z:z.writestr(info,'payload')
    with self.assertRaises(ValueError):cli.restore_archive(archive,work,pilot_for(archive))
    self.assertFalse(work.exists())

 def test_archive_checksum_and_no_overwrite(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);archive=root/'data.zip';work=root/'restore'
   with zipfile.ZipFile(archive,'w') as z:z.writestr('handoff_data_pilot01/value.txt','accepted')
   approved=pilot_for(archive);restored=cli.restore_archive(archive,work,approved);self.assertEqual((restored/'value.txt').read_text(),'accepted')
   with self.assertRaises(FileExistsError):cli.restore_archive(archive,work,approved)
   bad=dict(approved,archive_sha256='0'*64)
   with self.assertRaises(ValueError):cli.restore_archive(archive,root/'another',bad)
   self.assertFalse((root/'another').exists())

 def test_inventory_detects_mutation_and_unrecorded_code(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);file=root/'array.npy';file.write_bytes(b'original');inventory={'array.npy':dict(bytes=file.stat().st_size,sha256=cli.sha(file))};p=root/'bundle_inventory.json';p.write_text(json.dumps(inventory));pilot=dict(bundle_inventory_sha256=cli.sha(p));self.assertEqual(cli.verify_dataset(root,pilot)['status'],'PASS');file.write_bytes(b'changed!')
   with self.assertRaises(ValueError):cli.verify_dataset(root,pilot)
   file.write_bytes(b'original');(root/'numpy.py').write_text('unexpected')
   with self.assertRaisesRegex(ValueError,'Unrecorded'):cli.verify_dataset(root,pilot)

 def test_inventory_itself_cannot_be_resealed_silently(self):
  with tempfile.TemporaryDirectory() as directory:
   root=Path(directory);p=root/'bundle_inventory.json';p.write_text('{}');pilot=dict(bundle_inventory_sha256=cli.sha(p));p.write_text('{"new":{}}')
   with self.assertRaisesRegex(ValueError,'Inventory differs'):cli.verify_dataset(root,pilot)

 def test_existing_receipt_is_preserved(self):
  with tempfile.TemporaryDirectory() as directory:
   p=Path(directory)/'receipt.json';p.write_text('original')
   with redirect_stdout(io.StringIO()) as out:code=cli.main(['baseline','--output',str(p)])
   self.assertEqual(code,1);self.assertEqual(p.read_text(),'original');self.assertEqual(json.loads(out.getvalue())['status'],'ERROR')

 def test_archive_mode_requires_explicit_new_directory(self):
  with patch.object(cli,'restore_archive') as restore,redirect_stdout(io.StringIO()) as out:code=cli.main(['smoke-data','--archive','missing.zip'])
  self.assertEqual(code,1);restore.assert_not_called();self.assertIn('--work-dir',json.loads(out.getvalue())['error'])
