"""Recovery regression tests: source stages, causal proposals, GT separation."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.datasets.episode_recovery import (SENSOR_KEYS,REQUEST_STAGE,causal_candidates,
    executed_body_twist,sensor_inputs,load_recovered_sample,sha,write_json)


class RecoveryTests(unittest.TestCase):
    def test_sensor_whitelist_never_opens_poisoned_gt_object(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'observations.npz'
            np.savez(path,**{k:np.array([1.]) for k in SENSOR_KEYS},
                     human_world=np.array([object()],dtype=object))
            result=sensor_inputs(path)  # allow_pickle=False would fail if GT were read.
            self.assertEqual(set(result),set(SENSOR_KEYS))
            self.assertNotIn('human_world',result)

    def test_candidates_require_source_and_availability_before_query(self):
        rows=[dict(stamp_ns=10,available_sim_ns=12,observation_valid=True,tracks=[{'track_id':1}]),
              dict(stamp_ns=20,available_sim_ns=25,observation_valid=True,tracks=[{'track_id':999}])]
        tracks,mask,stamp=causal_candidates(rows,[10,12,20,25,100],max_age_ns=30)
        self.assertEqual(tracks,[[],[{'track_id':1}],[{'track_id':1}],[{'track_id':999}],[]])
        np.testing.assert_array_equal(mask,[False,True,True,True,False])
        np.testing.assert_array_equal(stamp,[-1,10,10,20,-1])

    def test_empty_inference_and_failed_inference_are_different(self):
        rows=[dict(stamp_ns=10,available_sim_ns=10,observation_valid=True,tracks=[]),
              dict(stamp_ns=20,available_sim_ns=20,observation_valid=False,tracks=[])]
        _,mask,_=causal_candidates(rows,[10,20]);np.testing.assert_array_equal(mask,[True,False])

    def test_world_to_body_velocity_direction_and_mask(self):
        pose=np.array([[0.,0.,np.pi/2],[0,0,0]])
        velocity=np.array([[1.,0.,.2],[999,999,999]])
        actual=executed_body_twist(pose,velocity,[True,False])
        np.testing.assert_allclose(actual[0],[0,-1,.2],atol=1e-12)
        self.assertTrue(np.isnan(actual[1]).all())

    def fixture(self,folder):
        base=Path(folder)/'base';base.mkdir();out=Path(folder)/'recovery';out.mkdir();(out/'perception').mkdir()
        write_json(out/'command_provenance.json',dict(stage=REQUEST_STAGE))
        (base/'controls.jsonl').write_text(json.dumps(dict(record_type='observed_published_command',source_stamp_ns=1_100_000_000,value=[999,999,999]))+'\n')
        (out/'perception/observed_candidates.jsonl').write_text(json.dumps(dict(stamp_ns=1_000_000_000,
            available_sim_ns=1_000_000_000,observation_valid=True,tracks=[dict(track_id=5)]))+'\n')
        write_json(out/'missing_sources.json',dict(base_export=str(base),base_artifact_sha256={'controls.jsonl':sha(base/'controls.jsonl')},
            recovery_artifact_sha256={'command_provenance.json':sha(out/'command_provenance.json'),
                'perception/observed_candidates.jsonl':sha(out/'perception/observed_candidates.jsonl')}))
        sample=dict(index=dict(anchor_ns=1_000_000_000),obs=dict(query_ns=np.array([1_000_000_000])),
            act=dict(past_published_commands=np.zeros((21,3)),availability=np.ones(21,dtype=bool)),
            gt=dict(robot_world=np.zeros((30,3)),robot_velocity_world=np.ones((30,3))*999,
                    robot_velocity_available=np.ones(30,dtype=bool)))
        return base,out,sample

    def test_recovered_requests_and_future_execution_stay_separate(self):
        with tempfile.TemporaryDirectory() as folder:
            base,out,sample=self.fixture(folder)
            with patch('dynamuhsim.datasets.single_episode_export.load_sample',return_value=copy.deepcopy(sample)):
                value=load_recovered_sample(out,0,expected_base=base)
            self.assertFalse(value['complete_requested_obs_act'])
            self.assertEqual(value['act']['requested_stage'],REQUEST_STAGE)
            np.testing.assert_array_equal(value['act']['past_requested_commands'],np.zeros((21,3)))
            self.assertIsNone(value['act']['future_candidate_requested'])
            self.assertFalse(value['act']['future_candidate_requested_available'])
            self.assertEqual(value['gt']['future_executed_actions'][0,0],999)
            self.assertEqual(len(value['gt']['future_issued_commands']),1)
            self.assertFalse(value['obs']['candidate_semantic_validated'])
            self.assertFalse(value['obs']['observed_occlusion_known'].any())

    def test_changed_prediction_or_wrong_episode_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            base,out,_=self.fixture(folder)
            with self.assertRaisesRegex(ValueError,'different base'):load_recovered_sample(out,0,expected_base='/another')
            (out/'perception/observed_candidates.jsonl').write_text('changed')
            with self.assertRaisesRegex(ValueError,'artifact changed'):load_recovered_sample(out,0)

    def test_json_serialization_error_does_not_leave_a_partial_file(self):
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'result.json'
            with self.assertRaises(TypeError):write_json(path,dict(bad=np.uint8(1)))
            self.assertFalse(path.exists())


if __name__=='__main__':unittest.main()
