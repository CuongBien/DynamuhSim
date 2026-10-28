"""Portable observations remain independent of labels and executable bundle tools."""
import json, runpy, tempfile, unittest
from pathlib import Path
import numpy as np
from dynamuhsim.datasets.portable import Episode

class PortableReader(unittest.TestCase):
    def fixture(self, root):
        paths=dict(samples='samples.npz',depth='depth.npy',bev='bev.npz',calibration='calibration.json',labels='labels.npz',task_masks='masks.npz')
        (root/'bundle.json').write_text(json.dumps(dict(schema='social_bundle/1',paths=paths,training_ready=False)))
        np.savez(root/'samples.npz',anchor_ns=np.array([10,20]),depth_index=np.zeros((2,20),np.int32),action=np.zeros((2,20,3)))
        np.save(root/'depth.npy',np.ones((1,2,3),np.float32));np.savez(root/'bev.npz',occupancy=np.zeros((1,64,64),np.int8),depth_index=np.array([0]));(root/'calibration.json').write_text('{}')
        (root/'tools').mkdir();(root/'tools/portable_loader.py').write_text('raise AssertionError("dataset code executed")')
        return Episode(root)
    def test_observation_does_not_load_labels_or_bundle_code(self):
        shim=Path(__file__).resolve().parents[1]/"experiments/social_navigation/datasets/portable_episode/portable_loader.py"
        self.assertIs(runpy.run_path(str(shim))["Episode"], Episode)
        with tempfile.TemporaryDirectory() as directory:
            episode=self.fixture(Path(directory));a=episode.observation(1);self.assertIsNone(episode.labels)
            self.assertEqual(a['depth_history'].shape,(20,2,3));self.assertEqual(a['current_bev'].shape,(64,64));self.assertNotIn('labels',a)
            self.assertFalse((episode.root/'labels.npz').exists())
    def test_supervision_is_explicit_and_preserves_global_human_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            episode=self.fixture(Path(directory));np.savez(episode.root/'labels.npz',human_ids=np.arange(6),human_xy_future_m=np.zeros((2,30,6,2)))
            self.assertEqual(episode.supervision(1)['human_ids'].shape,(6,));self.assertEqual(episode.supervision(1)['human_xy_future_m'].shape,(30,6,2))
    def test_bundle_paths_cannot_escape_the_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            episode=self.fixture(Path(directory));episode.metadata['paths']['samples']='../outside.npz'
            with self.assertRaises(AssertionError):episode.locate('samples')
