"""Live normalizer bindings and historical writer bytes have different roles."""
import hashlib, json, tempfile, unittest
from pathlib import Path
from dynamuhsim.analysis.evidence import verify_sources
from dynamuhsim.workspace import source_path
from dynamuhsim import datasets
ROOT = Path(__file__).resolve().parents[1]

class DatasetBindings(unittest.TestCase):
    def test_frozen_export_resolves_live_math_and_retired_writer_by_exact_hash(self):
        expected = json.loads((ROOT / 'experiments/social_navigation/datasets/wm_episode_batch/export_frozen.json').read_text())['source_sha256']
        selected = {key:value for key,value in expected.items() if '/tools/legacy/' in key}
        self.assertEqual(len(selected), 5)
        origins = verify_sources(selected, repository=ROOT, revision='9833565fa546d55cb0e553f8388bf4a703c59583')
        core = 'experiments/social_navigation/benchmark/wm_episode_batch/tools/legacy/dataset_core.py'
        self.assertEqual(source_path(ROOT,core), ROOT/'python/dynamuhsim/datasets/normalization.py')
        self.assertEqual(origins[core], 'file')
        self.assertEqual(hashlib.sha256(source_path(ROOT,core).read_bytes()).hexdigest(), expected[core])
        self.assertEqual(origins['experiments/social_navigation/benchmark/wm_episode_batch/tools/legacy/build.py'], 'recorded_git')
    def test_changed_live_math_rejected_before_execution(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'normalization.py').write_text('raise AssertionError("must not execute")')
            (root/'normalization_contract.json').write_text('{}')
            with self.assertRaisesRegex(RuntimeError, 'Frozen normalizer math changed'):
                exec(compile(Path(datasets.__file__).read_text(), str(root/'__init__.py'), 'exec'), dict(__file__=str(root/'__init__.py')))
    def test_missing_current_math_is_not_shadowed_by_legacy_cache(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory);package=root/'python/dynamuhsim/datasets';package.mkdir(parents=True);(package/'__init__.py').write_text('')
            old=root/'experiments/social_navigation/benchmark/wm_episode_batch/tools/legacy/dataset_core.py';old.parent.mkdir(parents=True);old.write_text('cached math')
            self.assertEqual(source_path(root,old.relative_to(root).as_posix()),package/'normalization.py')
            self.assertFalse(source_path(root,old.relative_to(root).as_posix()).exists())
