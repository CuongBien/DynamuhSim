import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'experiments/social_navigation/perception/depth_generalization'
spec=importlib.util.spec_from_file_location('depth_benchmark_readiness',STAGE/'benchmark_readiness.py')
br=importlib.util.module_from_spec(spec);spec.loader.exec_module(br)


class BenchmarkReadinessTests(unittest.TestCase):
    def setUp(self):self.manifest=json.loads((STAGE/'benchmark_manifest.json').read_text())
    def report(self):return br.check(self.manifest,Path('/tmp/depth-readiness-uncollected-data'))

    def test_planned_cases_never_become_ready(self):
        r=self.report();self.assertEqual(r['status'],'NOT_READY');self.assertTrue(r['pending']);self.assertFalse(r['training_authorized']);self.assertFalse(r['test_evaluation_authorized'])

    def test_same_avatar_geometry_blocks_new_family_name(self):
        self.manifest['cases'][0]['avatar_geometry_sha256']=['a'*64]
        self.manifest['cases'][4]['avatar_geometry_sha256']=['a'*64]
        self.assertTrue(any('avatar geometry' in e for e in self.report()['errors']))

    def test_same_scene_geometry_blocks_new_scene_name(self):
        self.manifest['cases'][0]['scene_geometry_sha256']='b'*64
        self.manifest['cases'][4]['scene_geometry_sha256']='b'*64
        self.assertTrue(any('scene_hash' in e for e in self.report()['errors']))

    def test_episode_or_window_group_cannot_cross_splits(self):
        self.manifest['cases'][4]['episode_group']=self.manifest['cases'][0]['episode_group']
        self.assertTrue(any('episode_group' in e for e in self.report()['errors']))

    def test_no_human_control_is_required(self):
        for case in self.manifest['cases']:
            if case['split']=='test':case['has_people']=True
        self.assertTrue(any('empty/clutter control' in e for e in self.report()['errors']))

    def test_detector_labels_are_not_trusted_instance_labels(self):
        self.manifest['cases'][0]['labels_kind']='detector_pseudo_label'
        self.assertTrue(any('trusted separate labels' in e for e in self.report()['errors']))

    def test_paths_must_stay_inside_dataset(self):
        self.manifest['cases'][0]['files']={'depth.npy':dict(path='../escape.npy',sha256='a'*64)}
        self.assertTrue(any('escaped data root' in e for e in self.report()['errors']))

    def test_changed_data_hash_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp,'sample.npy').write_bytes(b'changed')
            self.manifest['cases'][0]['files']={'depth.npy':dict(path='sample.npy',sha256='a'*64)}
            r=br.check(self.manifest,Path(tmp))
            self.assertTrue(any('changed depth.npy' in e for e in r['errors']))


if __name__=='__main__':unittest.main()
