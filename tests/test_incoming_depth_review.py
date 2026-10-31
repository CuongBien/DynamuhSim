"""Guard regressions exposed by incoming review; no Torch, simulator, training or test data."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT/'experiments/social_navigation/perception/depth_generalization'
sys.path.insert(0,str(STAGE))
from integration_guard import new_artifact_dir
from review_incoming import morphology_leaks

class IncomingReview(unittest.TestCase):
    def test_recolored_families_do_not_hide_cross_split_geometry(self):
        rows=[dict(case_id='a',split='train',geometry_arrays_sha256='same'),dict(case_id='b',split='validation',geometry_arrays_sha256='same'),dict(case_id='c',split='test',geometry_arrays_sha256='same')]
        self.assertEqual(morphology_leaks(rows),['b','c'])

    def test_same_family_within_train_is_not_cross_split(self):
        self.assertEqual(morphology_leaks([dict(case_id='a',split='train',geometry_arrays_sha256='same'),dict(case_id='b',split='train',geometry_arrays_sha256='same')]),[])

    def test_artifact_guard_rejects_default_and_descendant(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name in ['artifacts','artifacts/nested','artifacts_legacy','artifacts_legacy/nested']:
                with self.assertRaises(ValueError):new_artifact_dir(root/name,root)
            self.assertEqual(list(root.iterdir()),[])

    def test_artifact_guard_preserves_existing_output(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); target=root/'experiment';target.mkdir();(target/'weights').write_text('keep')
            with self.assertRaises(FileExistsError):new_artifact_dir(target,root)
            self.assertEqual((target/'weights').read_text(),'keep')

    def test_new_experiment_is_allowed(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(new_artifact_dir(Path(tmp)/'new',tmp).is_dir())

    def test_frozen_default_is_original_and_incoming_is_opt_in(self):
        parent=ROOT/'experiments/social_navigation/perception/depth_person_classifier'
        self.assertEqual(hashlib.sha256((parent/'artifacts/selected_model.npz').read_bytes()).hexdigest(),'41f990bd581afd87d93bb93bd57ea758b51a914f31fd6bdb6f027ddd6e27f292')
        cfg=json.loads((parent/'artifacts_experimental_2ffd7c75/selected_frozen.json').read_text())
        self.assertFalse(cfg['validation_gate']);self.assertFalse(cfg['promotion_to_default'])
        self.assertEqual(hashlib.sha256((parent/'artifacts_experimental_2ffd7c75/selected_model.npz').read_bytes()).hexdigest(),cfg['model_sha256'])

if __name__=='__main__':unittest.main()
